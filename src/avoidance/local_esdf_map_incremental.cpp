// local_esdf_map_incremental.cpp
//
// Stage 4: LocalESDFMap incremental update methods.
//
// update_incremental(l2, dirty_positions, d0)
//   Recomputes the ESDF for the sub-window that covers dirty_positions ± d0.
//   All other cells are left unchanged.  Equivalent to a full recompute in
//   the affected region; error < 0.01 m when dirty cells are not near the
//   original window boundary.
//
// update_tube(l2, waypoints, tube_radius)
//   Computes or refreshes the ESDF for the bounding volume of the waypoints
//   expanded by tube_radius + d0.  Intended for JIT path validation; only the
//   cells along the tube need to be accurate.
//
// Both methods:
//   1. Derive a sub-window (centre + half-extents).
//   2. Run compute_esdf on that sub-window.
//   3. Erase stale cells from *this that fall inside the sub-window.
//   4. Merge the fresh cells from the sub-ESDF into *this.

#include "dedalus/avoidance/local_esdf_map.hpp"
#include "dedalus/avoidance/mission_local_planning_map.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>

namespace dedalus {

// ─── update_incremental ──────────────────────────────────────────────────────

void LocalESDFMap::update_incremental(
    const MissionLocalPlanningMap& l2,
    const std::vector<Vec3>& dirty_world_positions,
    double d0_m) {

    if (dirty_world_positions.empty()) return;

    const double sx = config_.cell_size_m;
    const double sz = config_.vertical_cell_size_m;

    // Bounding box of dirty positions.
    Vec3 mn = dirty_world_positions[0];
    Vec3 mx = dirty_world_positions[0];
    for (const auto& p : dirty_world_positions) {
        mn.x = std::min(mn.x, p.x);  mn.y = std::min(mn.y, p.y);  mn.z = std::min(mn.z, p.z);
        mx.x = std::max(mx.x, p.x);  mx.y = std::max(mx.y, p.y);  mx.z = std::max(mx.z, p.z);
    }

    // Sub-window: dirty bbox centre ± (half dirty extent + d0 + one cell margin).
    const Vec3 centre{(mn.x + mx.x) * 0.5, (mn.y + mx.y) * 0.5, (mn.z + mx.z) * 0.5};
    const double horiz_half =
        std::max((mx.x - mn.x) * 0.5, (mx.y - mn.y) * 0.5) + d0_m + sx;
    const double vert_half = (mx.z - mn.z) * 0.5 + d0_m + sz;

    // Fresh ESDF for this sub-window using current L2 state.
    // Pass sample_spacing_m so the sub-ESDF uses the same coarse resolution as *this.
    auto sub = compute_esdf(l2, centre, horiz_half, vert_half, d0_m,
                             config_.sample_spacing_m);

    // Generous window bounds (include the grid-snap margin).
    const Vec3 win_min{centre.x - horiz_half - sx,
                       centre.y - horiz_half - sx,
                       centre.z - vert_half  - sz};
    const Vec3 win_max{centre.x + horiz_half + sx,
                       centre.y + horiz_half + sx,
                       centre.z + vert_half  + sz};

    // Erase stale shell cells that fall inside the sub-window.
    for (auto it = cells_.begin(); it != cells_.end(); ) {
        const Vec3& c = it->second.centre;
        if (c.x >= win_min.x && c.x <= win_max.x &&
            c.y >= win_min.y && c.y <= win_max.y &&
            c.z >= win_min.z && c.z <= win_max.z) {
            it = cells_.erase(it);
        } else {
            ++it;
        }
    }

    // Merge fresh cells from sub-ESDF.
    for (auto& [key, cell] : sub.cells_) {
        cells_[key] = cell;
    }
}

// ─── update_tube ─────────────────────────────────────────────────────────────

void LocalESDFMap::update_tube(
    const MissionLocalPlanningMap& l2,
    const std::vector<Vec3>& waypoints,
    double tube_radius_m) {

    if (waypoints.empty()) return;

    const double sx  = config_.cell_size_m;
    const double sz  = config_.vertical_cell_size_m;
    const double d0  = config_.d0_m;
    const double exp_xy = tube_radius_m + d0 + sx;
    const double exp_z  = tube_radius_m + d0 + sz;

    // Bounding box of all waypoints.
    Vec3 mn = waypoints[0];
    Vec3 mx = waypoints[0];
    for (const auto& p : waypoints) {
        mn.x = std::min(mn.x, p.x);  mn.y = std::min(mn.y, p.y);  mn.z = std::min(mn.z, p.z);
        mx.x = std::max(mx.x, p.x);  mx.y = std::max(mx.y, p.y);  mx.z = std::max(mx.z, p.z);
    }

    const Vec3 centre{(mn.x + mx.x) * 0.5, (mn.y + mx.y) * 0.5, (mn.z + mx.z) * 0.5};
    const double horiz_half =
        std::max((mx.x - mn.x) * 0.5, (mx.y - mn.y) * 0.5) + exp_xy;
    const double vert_half = (mx.z - mn.z) * 0.5 + exp_z;

    auto sub = compute_esdf(l2, centre, horiz_half, vert_half, d0,
                             config_.sample_spacing_m);

    const Vec3 win_min{centre.x - horiz_half - sx,
                       centre.y - horiz_half - sx,
                       centre.z - vert_half  - sz};
    const Vec3 win_max{centre.x + horiz_half + sx,
                       centre.y + horiz_half + sx,
                       centre.z + vert_half  + sz};

    for (auto it = cells_.begin(); it != cells_.end(); ) {
        const Vec3& c = it->second.centre;
        if (c.x >= win_min.x && c.x <= win_max.x &&
            c.y >= win_min.y && c.y <= win_max.y &&
            c.z >= win_min.z && c.z <= win_max.z) {
            it = cells_.erase(it);
        } else {
            ++it;
        }
    }

    for (auto& [key, cell] : sub.cells_) {
        cells_[key] = cell;
    }
}

}  // namespace dedalus
