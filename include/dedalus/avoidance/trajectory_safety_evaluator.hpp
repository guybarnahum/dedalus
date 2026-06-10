#pragma once

#include <cstddef>
#include <limits>
#include <vector>

#include "dedalus/avoidance/local_flight_map.hpp"
#include "dedalus/core/types.hpp"

namespace dedalus {

struct LocalTrajectorySample {
    Vec3 position_local;
    double time_s{0.0};
};

struct TrajectorySafetyResult {
    bool clear{true};
    bool blocked{false};
    bool has_valid_query{false};

    std::size_t sample_count{0U};
    std::size_t blocked_sample_count{0U};
    std::size_t first_blocked_sample_index{0U};

    Vec3 first_blocked_position_local;

    float minimum_clearance_m{std::numeric_limits<float>::infinity()};
    float nearest_obstacle_m{std::numeric_limits<float>::infinity()};
};

class TrajectorySafetyEvaluator {
public:
    TrajectorySafetyResult evaluate(
        const LocalFlightMapSnapshot& map,
        const std::vector<LocalTrajectorySample>& trajectory) const;

private:
    static bool local_to_grid(
        const LocalFlightMapSnapshot& map,
        const Vec3& local,
        int& ix,
        int& iy);

    static const LocalFlightMapCell* cell_at(
        const LocalFlightMapSnapshot& map,
        int ix,
        int iy);

    static float distance_xy_m(const Vec3& a, const Vec3& b);
    static float clearance_to_blocked_cells_m(
        const LocalFlightMapSnapshot& map,
        const Vec3& local);
};

std::vector<LocalTrajectorySample> make_forward_trajectory_samples(
    double speed_mps,
    double horizon_s,
    double step_s);

}  // namespace dedalus
