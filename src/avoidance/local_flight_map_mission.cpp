#include "dedalus/avoidance/local_flight_map.hpp"

#include "dedalus/avoidance/mission_local_obstacle_map.hpp"
#include "dedalus/geometry/pose_transform.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace dedalus {
namespace {

std::uint64_t timestamp_ns(const TimePoint& time) {
    return static_cast<std::uint64_t>(time.timestamp_ns);
}

float clamp01(const double value) {
    if (!std::isfinite(value) || value <= 0.0) {
        return 0.0F;
    }
    if (value >= 1.0) {
        return 1.0F;
    }
    return static_cast<float>(value);
}

float local_range_m(const Vec3& local) {
    return static_cast<float>(std::sqrt(
        (local.x * local.x) +
        (local.y * local.y) +
        (local.z * local.z)));
}

}  // namespace

LocalFlightMapSnapshot LocalFlightMapAccumulator::update_from_mission_local_map(
    const MissionLocalObstacleMapSnapshot& mission_map,
    const Pose3& map_T_body,
    const TimePoint timestamp) {
    latest_.timestamp = timestamp;
    latest_.source_frame_id = FrameId{};
    latest_.has_source_frame = false;

    // The local flight map is a view in the current ego frame. Do not retain
    // cells from the previous ego pose; persistence belongs to the mission map.
    reset_cells();

    latest_.source_mission_cell_count = mission_map.summary.observed_cell_count;
    latest_.projected_mission_cell_count = 0U;
    latest_.projected_local_cell_update_count = 0U;
    latest_.exclusion_inflation_radius_m = std::max(
        0.0F,
        config_.vehicle_radius_m + config_.safety_margin_m);

    const auto now_ns = timestamp_ns(timestamp);

    for (const auto& mission_cell : mission_map.cells) {
        if (!mission_cell.observed) {
            continue;
        }

        if (!mission_cell.occupied && !mission_cell.free && mission_cell.risk_score <= 0.0) {
            continue;
        }

        const auto center_local = inverse_transform_point(map_T_body, mission_cell.center_map);
        const auto maybe_center = local_to_grid(center_local);
        if (!maybe_center.has_value()) {
            continue;
        }

        const auto footprint_xy_m = static_cast<float>(
            std::max(mission_cell.size_m.x, mission_cell.size_m.y));
        const auto footprint_radius_m = std::max(
            config_.cell_size_m,
            0.5F * footprint_xy_m);
        const int radius_cells = mission_cell.occupied
            ? footprint_radius_cells(footprint_radius_m)
            : 0;

        const float occupied_score = clamp01(mission_cell.occupied_score);
        const float free_score = clamp01(mission_cell.free_score);
        const float risk_score = clamp01(mission_cell.risk_score);
        const float range_m = local_range_m(center_local);

        bool projected_any_local_cell = false;
        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
            for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                if ((dx * dx) + (dy * dy) > radius_cells * radius_cells) {
                    continue;
                }

                auto* local_cell = mutable_cell_at(maybe_center->ix + dx, maybe_center->iy + dy);
                if (local_cell == nullptr) {
                    continue;
                }

                local_cell->occupied_score = std::max(local_cell->occupied_score, occupied_score);
                local_cell->free_score = std::max(local_cell->free_score, free_score);
                local_cell->risk_score = std::max(local_cell->risk_score, risk_score);
                local_cell->nearest_range_m = std::min(local_cell->nearest_range_m, range_m);
                local_cell->min_z_m = static_cast<float>(
                    center_local.z - (0.5 * mission_cell.size_m.z));
                local_cell->max_z_m = static_cast<float>(
                    center_local.z + (0.5 * mission_cell.size_m.z));
                local_cell->last_observed_ns = std::max(
                    local_cell->last_observed_ns,
                    mission_cell.last_observed_timestamp_ns != 0U
                        ? mission_cell.last_observed_timestamp_ns
                        : now_ns);
                local_cell->recently_observed = true;
                projected_any_local_cell = true;
                ++latest_.projected_local_cell_update_count;
            }
        }

        if (projected_any_local_cell) {
            ++latest_.projected_mission_cell_count;
        }
    }

    classify_cells();
    inflate_blocked_cells();
    update_summary();

    last_update_ns_ = now_ns;
    has_last_update_ = true;
    return latest_;
}

}  // namespace dedalus
