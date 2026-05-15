#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "dedalus/core/types.hpp"

namespace dedalus {

struct TrajectoryKeyframe {
    double t_s{0.0};
    double vx_mps{0.0};
    double vy_mps{0.0};
    double vz_mps{0.0};
};

struct TrajectorySegment {
    std::string type{"hold"};
    std::string label;
    double duration_s{0.0};
    double vx_mps{0.0};
    double vy_mps{0.0};
    double vz_mps{0.0};
    double speed_mps{0.0};
    double radius_m{1.0};
    double scale_m{1.0};
    std::string direction{"ccw"};
    std::vector<TrajectoryKeyframe> keyframes;
};

class VelocityTrajectory final {
public:
    VelocityTrajectory() = default;
    explicit VelocityTrajectory(std::vector<TrajectorySegment> segments);

    [[nodiscard]] static VelocityTrajectory load_from_file(const std::string& path);
    [[nodiscard]] static VelocityTrajectory parse_json(const std::string& text);
    [[nodiscard]] static VelocityTrajectory default_hold(double duration_s = 1.0);

    [[nodiscard]] bool empty() const;
    [[nodiscard]] std::size_t size() const;
    [[nodiscard]] const std::vector<TrajectorySegment>& segments() const;
    [[nodiscard]] const TrajectorySegment& segment(std::size_t index) const;
    [[nodiscard]] Vec3 velocity_at(std::size_t segment_index, double segment_elapsed_s) const;

private:
    std::vector<TrajectorySegment> segments_;
};

}  // namespace dedalus