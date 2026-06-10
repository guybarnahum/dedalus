#include "dedalus/avoidance/trajectory_safety_evaluator.hpp"

#include <algorithm>
#include <cmath>

namespace dedalus {

bool TrajectorySafetyEvaluator::local_to_grid(
    const LocalFlightMapSnapshot& map,
    const Vec3& local,
    int& ix,
    int& iy) {
    if (map.cell_size_m <= 0.0F || map.x_cells <= 0 || map.y_cells <= 0) {
        return false;
    }
    if (local.x < -map.rear_range_m || local.x >= map.forward_range_m) {
        return false;
    }
    if (local.y < -map.lateral_range_m || local.y >= map.lateral_range_m) {
        return false;
    }

    ix = static_cast<int>(std::floor((local.x + map.rear_range_m) / map.cell_size_m));
    iy = static_cast<int>(std::floor((local.y + map.lateral_range_m) / map.cell_size_m));
    return ix >= 0 && ix < map.x_cells && iy >= 0 && iy < map.y_cells;
}

const LocalFlightMapCell* TrajectorySafetyEvaluator::cell_at(
    const LocalFlightMapSnapshot& map,
    const int ix,
    const int iy) {
    if (ix < 0 || ix >= map.x_cells || iy < 0 || iy >= map.y_cells) {
        return nullptr;
    }
    const auto index = static_cast<std::size_t>((iy * map.x_cells) + ix);
    if (index >= map.cells.size()) {
        return nullptr;
    }
    return &map.cells[index];
}

float TrajectorySafetyEvaluator::distance_xy_m(const Vec3& a, const Vec3& b) {
    const auto dx = a.x - b.x;
    const auto dy = a.y - b.y;
    return static_cast<float>(std::sqrt((dx * dx) + (dy * dy)));
}

float TrajectorySafetyEvaluator::clearance_to_blocked_cells_m(
    const LocalFlightMapSnapshot& map,
    const Vec3& local) {
    float best = std::numeric_limits<float>::infinity();
    for (const auto& cell : map.cells) {
        if (!cell.inflated_blocked && !cell.occupied) {
            continue;
        }
        best = std::min(best, distance_xy_m(local, cell.center_local));
    }
    if (std::isfinite(best)) {
        best = std::max(0.0F, best - (0.5F * map.cell_size_m));
    }
    return best;
}

TrajectorySafetyResult TrajectorySafetyEvaluator::evaluate(
    const LocalFlightMapSnapshot& map,
    const std::vector<LocalTrajectorySample>& trajectory) const {
    TrajectorySafetyResult result;
    result.sample_count = trajectory.size();
    result.nearest_obstacle_m = map.nearest_obstacle_m;
    result.has_valid_query = !trajectory.empty() && map.cell_size_m > 0.0F && !map.cells.empty();

    if (!result.has_valid_query) {
        return result;
    }

    for (std::size_t i = 0; i < trajectory.size(); ++i) {
        const auto& sample = trajectory[i];

        const auto clearance = clearance_to_blocked_cells_m(map, sample.position_local);
        result.minimum_clearance_m = std::min(result.minimum_clearance_m, clearance);

        int ix = 0;
        int iy = 0;
        if (!local_to_grid(map, sample.position_local, ix, iy)) {
            continue;
        }

        const auto* cell = cell_at(map, ix, iy);
        if (cell == nullptr) {
            continue;
        }

        if (cell->inflated_blocked) {
            ++result.blocked_sample_count;
            if (!result.blocked) {
                result.blocked = true;
                result.clear = false;
                result.first_blocked_sample_index = i;
                result.first_blocked_position_local = sample.position_local;
            }
        }
    }

    return result;
}

std::vector<LocalTrajectorySample> make_forward_trajectory_samples(
    const double speed_mps,
    const double horizon_s,
    const double step_s) {
    std::vector<LocalTrajectorySample> samples;
    if (horizon_s <= 0.0 || step_s <= 0.0) {
        return samples;
    }

    const auto clamped_speed = std::max(0.0, speed_mps);
    for (double t = 0.0; t <= horizon_s + 1.0e-9; t += step_s) {
        samples.push_back(LocalTrajectorySample{
            .position_local = Vec3{clamped_speed * t, 0.0, 0.0},
            .time_s = t,
        });
    }
    return samples;
}

}  // namespace dedalus
