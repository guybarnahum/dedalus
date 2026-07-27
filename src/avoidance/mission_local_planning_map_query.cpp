// mission_local_planning_map_query.cpp
//
// Stage 2.5: L2 planning query API.
// Implements: ray_cast(), query_occupied_in_box().
//
// Both methods are pure queries on the in-memory window — no side effects
// on L2 state, no DB access.  Call slide_window() first to ensure the
// region of interest is in memory.
//
// ray_cast — 3D Amanatides-Woo DDA
// ──────────────────────────────────
// Marches one voxel at a time along the ray.  At each step the algorithm
// selects the axis whose next boundary is crossed soonest (smallest t_max),
// advances the voxel index on that axis, and checks cell_index_ for occupancy.
// Handles non-unit direction vectors and negative world coordinates correctly.
//
// query_occupied_in_box / query_occupied_ts_in_box
// ──────────────────────────────────────────────────
// Both delegate to occupied_cells_in_box(), which walks spatial_buckets_ —
// a coarse (~10 m) grouping of cell_index_'s keys, maintained incrementally
// at every insert/remove site (see MissionLocalPlanningMap::bucket_insert/
// bucket_remove) — visiting only the buckets that overlap bbox instead of
// scanning every cell in the map. Cost scales with local density near the
// query plus the (small, bounded) number of buckets touched, not with L2's
// total size — unlike enumerating every fine grid cell in the box directly,
// which for a sparse map (few cells over a wide horizon_m) can cost more
// probes than L2 even has cells. Each candidate still gets the exact score
// + bbox containment check below; buckets are a locality filter, not a
// substitute for it.

#include "dedalus/avoidance/mission_local_planning_map.hpp"

#include <cmath>
#include <limits>

namespace dedalus {

// ─── ray_cast ────────────────────────────────────────────────────────────────

std::optional<Vec3> MissionLocalPlanningMap::ray_cast(
    const Vec3& origin, const Vec3& dir, const double max_range_m) const noexcept {

    const double len = std::sqrt(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
    if (!(len > 0.0)) {
        return std::nullopt;
    }
    const double inv = 1.0 / len;
    const Vec3 d{dir.x * inv, dir.y * inv, dir.z * inv};

    const double sx = config_.cell_size_m;
    const double sy = config_.cell_size_m;
    const double sz = config_.vertical_cell_size_m;
    const float  min_score = static_cast<float>(config_.min_occupied_score);

    const auto occupied = [&](const CellKey& k) noexcept -> bool {
        const auto it = cell_index_.find(k);
        return it != cell_index_.end() &&
               cells_[it->second].cell.occupied_score >= min_score;
    };

    // Starting voxel.
    CellKey cur = key_for_point(origin);

    // Check origin voxel itself.
    if (occupied(cur)) {
        return center_for_key(cur);
    }

    // Step direction per axis.
    const int step_x = (d.x >= 0.0) ? 1 : -1;
    const int step_y = (d.y >= 0.0) ? 1 : -1;
    const int step_z = (d.z >= 0.0) ? 1 : -1;

    constexpr double kInf = std::numeric_limits<double>::infinity();

    // t increment for one full voxel crossing on each axis.
    const double dt_x = (d.x != 0.0) ? sx / std::abs(d.x) : kInf;
    const double dt_y = (d.y != 0.0) ? sy / std::abs(d.y) : kInf;
    const double dt_z = (d.z != 0.0) ? sz / std::abs(d.z) : kInf;

    // t to the next boundary crossing on each axis.
    // For step > 0: boundary is at (idx+1)*cell_size.
    // For step < 0: boundary is at  idx*cell_size.
    const auto t_first = [](double pos, int idx, int step, double cell_size,
                             double dir_comp) noexcept {
        const double boundary = (step > 0)
            ? (static_cast<double>(idx + 1) * cell_size)
            : (static_cast<double>(idx) * cell_size);
        return (boundary - pos) / dir_comp;
    };

    double tx = (d.x != 0.0) ? t_first(origin.x, cur.x, step_x, sx, d.x) : kInf;
    double ty = (d.y != 0.0) ? t_first(origin.y, cur.y, step_y, sy, d.y) : kInf;
    double tz = (d.z != 0.0) ? t_first(origin.z, cur.z, step_z, sz, d.z) : kInf;

    while (true) {
        double t;
        if (tx <= ty && tx <= tz) {
            t = tx;
            cur.x += step_x;
            tx += dt_x;
        } else if (ty <= tz) {
            t = ty;
            cur.y += step_y;
            ty += dt_y;
        } else {
            t = tz;
            cur.z += step_z;
            tz += dt_z;
        }

        if (t > max_range_m) {
            break;
        }

        if (occupied(cur)) {
            return center_for_key(cur);
        }
    }

    return std::nullopt;
}

// ─── occupied_cells_in_box ───────────────────────────────────────────────────

std::vector<const MissionLocalPlanningMap::StoredCell*>
MissionLocalPlanningMap::occupied_cells_in_box(const Bounds3& bbox) const {
    const float min_score = static_cast<float>(config_.min_occupied_score);

    const auto bucket_min = bucket_for_cell_key(key_for_point(bbox.min));
    const auto bucket_max = bucket_for_cell_key(key_for_point(bbox.max));

    std::vector<const StoredCell*> result;
    for (int bx = bucket_min.x; bx <= bucket_max.x; ++bx) {
        for (int by = bucket_min.y; by <= bucket_max.y; ++by) {
            for (int bz = bucket_min.z; bz <= bucket_max.z; ++bz) {
                const auto bucket_it = spatial_buckets_.find(BucketKey{bx, by, bz});
                if (bucket_it == spatial_buckets_.end()) {
                    continue;
                }
                for (const auto& key : bucket_it->second) {
                    const auto cell_it = cell_index_.find(key);
                    if (cell_it == cell_index_.end()) {
                        continue;  // shouldn't happen if the two indexes stay in sync
                    }
                    const auto& entry = cells_[cell_it->second];
                    if (entry.cell.occupied_score < min_score) continue;
                    const Vec3& p = entry.cell.center_map;
                    if (p.x >= bbox.min.x && p.x <= bbox.max.x &&
                        p.y >= bbox.min.y && p.y <= bbox.max.y &&
                        p.z >= bbox.min.z && p.z <= bbox.max.z) {
                        result.push_back(&entry);
                    }
                }
            }
        }
    }
    return result;
}

// ─── query_occupied_in_box ───────────────────────────────────────────────────

std::vector<Vec3> MissionLocalPlanningMap::query_occupied_in_box(
    const Bounds3& bbox) const {
    std::vector<Vec3> result;
    for (const auto* entry : occupied_cells_in_box(bbox)) {
        result.push_back(entry->cell.center_map);
    }
    return result;
}

// ─── query_occupied_ts_in_box ─────────────────────────────────────────────────

std::vector<std::pair<Vec3, std::int64_t>>
MissionLocalPlanningMap::query_occupied_ts_in_box(const Bounds3& bbox) const {
    std::vector<std::pair<Vec3, std::int64_t>> result;
    for (const auto* entry : occupied_cells_in_box(bbox)) {
        result.emplace_back(entry->cell.center_map, entry->cell.last_updated_ns);
    }
    return result;
}

}  // namespace dedalus
