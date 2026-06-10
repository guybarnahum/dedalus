#include "dedalus/avoidance/trajectory_safety_evaluator.hpp"

#include <cassert>
#include <iostream>

using namespace dedalus;

LocalFlightMapSnapshot simple_map() {
    LocalFlightMapSnapshot map;
    map.cell_size_m = 1.0F;
    map.forward_range_m = 10.0F;
    map.rear_range_m = 0.0F;
    map.lateral_range_m = 2.0F;
    map.x_cells = 10;
    map.y_cells = 4;
    map.cells.resize(40);

    for (int iy = 0; iy < map.y_cells; ++iy) {
        for (int ix = 0; ix < map.x_cells; ++ix) {
            auto& cell = map.cells[static_cast<std::size_t>((iy * map.x_cells) + ix)];
            cell.center_local = Vec3{static_cast<double>(ix) + 0.5, -1.5 + static_cast<double>(iy), 0.0};
        }
    }

    auto& blocked = map.cells[static_cast<std::size_t>((1 * map.x_cells) + 4)];
    blocked.occupied = true;
    blocked.inflated_blocked = true;
    blocked.nearest_range_m = 4.5F;
    map.nearest_obstacle_m = 4.5F;
    return map;
}

void clear_trajectory_is_clear() {
    TrajectorySafetyEvaluator evaluator;
    const auto map = simple_map();
    const std::vector<LocalTrajectorySample> samples{
        {Vec3{0.5, 1.0, 0.0}, 0.0},
        {Vec3{1.5, 1.0, 0.0}, 0.5},
        {Vec3{2.5, 1.0, 0.0}, 1.0},
    };

    const auto result = evaluator.evaluate(map, samples);
    assert(result.has_valid_query);
    assert(result.clear);
    assert(!result.blocked);
    assert(result.blocked_sample_count == 0U);
}

void blocked_trajectory_reports_first_blocked_sample() {
    TrajectorySafetyEvaluator evaluator;
    const auto map = simple_map();
    const std::vector<LocalTrajectorySample> samples{
        {Vec3{2.5, -0.5, 0.0}, 0.0},
        {Vec3{3.5, -0.5, 0.0}, 0.5},
        {Vec3{4.5, -0.5, 0.0}, 1.0},
        {Vec3{5.5, -0.5, 0.0}, 1.5},
    };

    const auto result = evaluator.evaluate(map, samples);
    assert(result.has_valid_query);
    assert(!result.clear);
    assert(result.blocked);
    assert(result.blocked_sample_count == 1U);
    assert(result.first_blocked_sample_index == 2U);
    assert(result.minimum_clearance_m <= 0.5F);
}

int main() {
    clear_trajectory_is_clear();
    blocked_trajectory_reports_first_blocked_sample();
    std::cout << "trajectory safety evaluator tests passed\n";
    return 0;
}
