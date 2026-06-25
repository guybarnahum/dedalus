// local_esdf_map.cpp
//
// LocalESDFMap public interface: query(), repulsion(), is_clear().
// The EDT computation lives in compute_esdf.cpp.
// L3 is never saved to disk — always recomputed from L2 (~6 ms).

#include "dedalus/avoidance/local_esdf_map.hpp"

#include <algorithm>
#include <cmath>

namespace dedalus {

// ─── CellKeyHash ─────────────────────────────────────────────────────────────

std::size_t LocalESDFMap::CellKeyHash::operator()(const CellKey& k) const noexcept {
    std::size_t h = 14695981039346656037ULL;
    auto mix = [&](int v) {
        h ^= static_cast<std::size_t>(static_cast<std::uint32_t>(v));
        h *= 1099511628211ULL;
    };
    mix(k.x);
    mix(k.y);
    mix(k.z);
    return h;
}

// ─── key_for_point ───────────────────────────────────────────────────────────

LocalESDFMap::CellKey LocalESDFMap::key_for_point(const Vec3& pos) const noexcept {
    return CellKey{
        static_cast<int>(std::floor(pos.x / config_.cell_size_m)),
        static_cast<int>(std::floor(pos.y / config_.cell_size_m)),
        static_cast<int>(std::floor(pos.z / config_.vertical_cell_size_m)),
    };
}

// ─── query ───────────────────────────────────────────────────────────────────

LocalESDFQueryResult LocalESDFMap::query(const Vec3& pos) const noexcept {
    const auto it = cells_.find(key_for_point(pos));
    if (it == cells_.end()) {
        return {static_cast<float>(config_.d0_m), Vec3{0.0, 0.0, 0.0}};
    }
    return {it->second.d, it->second.grad};
}

// ─── repulsion ───────────────────────────────────────────────────────────────

Vec3 LocalESDFMap::repulsion(const Vec3& pos, double d0, double k) const noexcept {
    // Trilinear interpolation over the 8 surrounding coarse cells.
    // APF force at each cell = k * (1/d - 1/d0) / d * sgrad.
    // Cells missing from the map contribute zero weight (boundary handling).
    const double csx = config_.cell_size_m;
    const double csz = config_.vertical_cell_size_m;

    // Fractional position within coarse voxel
    const double fx = pos.x / csx;
    const double fy = pos.y / csx;
    const double fz = pos.z / csz;

    const int x0 = static_cast<int>(std::floor(fx));
    const int y0 = static_cast<int>(std::floor(fy));
    const int z0 = static_cast<int>(std::floor(fz));

    const double tx = fx - x0;
    const double ty = fy - y0;
    const double tz = fz - z0;

    Vec3   result{0.0, 0.0, 0.0};
    double w_found = 0.0;

    for (int di = 0; di <= 1; ++di) {
        for (int dj = 0; dj <= 1; ++dj) {
            for (int dk = 0; dk <= 1; ++dk) {
                const auto it = cells_.find(CellKey{x0+di, y0+dj, z0+dk});
                if (it == cells_.end()) continue;
                const auto& cell = it->second;
                if (cell.d <= 0.0f) continue;
                const double d = static_cast<double>(cell.d);
                if (d >= d0) continue;

                const double wx = (di == 0) ? (1.0 - tx) : tx;
                const double wy = (dj == 0) ? (1.0 - ty) : ty;
                const double wz = (dk == 0) ? (1.0 - tz) : tz;
                const double w  = wx * wy * wz;

                const double scale = k * (1.0/d - 1.0/d0) / (d * d) * w;
                result.x += scale * cell.sgrad.x;
                result.y += scale * cell.sgrad.y;
                result.z += scale * cell.sgrad.z;
                w_found  += w;
            }
        }
    }
    // If boundary cells are missing, scale by found fraction so magnitude is consistent.
    if (w_found > 1.0e-10 && w_found < 0.999) {
        const double inv_w = 1.0 / w_found;
        result.x *= inv_w;
        result.y *= inv_w;
        result.z *= inv_w;
    }
    return result;
}

// ─── repulsion_smoothed ──────────────────────────────────────────────────────

Vec3 LocalESDFMap::repulsion_smoothed(const Vec3& pos,
                                       const Vec3& /* vel */,
                                       double d0,
                                       double k,
                                       double /* a_max_mps2 */) const noexcept {
    // L3 cells are pre-averaged (Gaussian-weighted APF computed in compute_esdf).
    // The stored sgrad already encodes spatial smoothing at sample_spacing_m scale.
    // Trilinear interpolation in repulsion() provides sub-cell continuity.
    // Velocity-dependent radius is implicit: at high speed, the planner calls this
    // over a trajectory horizon, not at a single point.
    return repulsion(pos, d0, k);
}

// ─── is_clear ────────────────────────────────────────────────────────────────

bool LocalESDFMap::is_clear(const Vec3& pos, double r) const noexcept {
    return static_cast<double>(query(pos).d) >= r;
}

// ─── snapshot ────────────────────────────────────────────────────────────────

LocalESDFMapSnapshot LocalESDFMap::snapshot(const Vec3& query_pos, double repulsion_k) const {
    LocalESDFMapSnapshot snap;
    snap.config = config_;
    snap.cell_count = cells_.size();
    snap.cells.reserve(cells_.size());
    for (const auto& [key, cell] : cells_) {
        snap.cells.push_back(cell);
    }
    snap.net_repulsion = repulsion(query_pos, config_.d0_m, repulsion_k);
    return snap;
}

}  // namespace dedalus
