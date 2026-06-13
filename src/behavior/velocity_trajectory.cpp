#include "dedalus/behavior/velocity_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace dedalus {
namespace {

constexpr double kPi = 3.14159265358979323846;

double lerp(double a, double b, double u) {
    return a + (b - a) * u;
}

Vec3 interpolate_keyframes(const std::vector<TrajectoryKeyframe>& keyframes, double t_s) {
    if (keyframes.empty()) {
        return Vec3{0.0, 0.0, 0.0};
    }
    if (t_s <= keyframes.front().t_s) {
        return Vec3{keyframes.front().vx_mps, keyframes.front().vy_mps, keyframes.front().vz_mps};
    }
    if (t_s >= keyframes.back().t_s) {
        return Vec3{keyframes.back().vx_mps, keyframes.back().vy_mps, keyframes.back().vz_mps};
    }

    for (std::size_t i = 0; i + 1U < keyframes.size(); ++i) {
        const auto& a = keyframes[i];
        const auto& b = keyframes[i + 1U];
        if (a.t_s <= t_s && t_s <= b.t_s) {
            const double span = std::max(b.t_s - a.t_s, 1e-6);
            const double u = (t_s - a.t_s) / span;
            return Vec3{
                lerp(a.vx_mps, b.vx_mps, u),
                lerp(a.vy_mps, b.vy_mps, u),
                lerp(a.vz_mps, b.vz_mps, u)};
        }
    }

    return Vec3{keyframes.back().vx_mps, keyframes.back().vy_mps, keyframes.back().vz_mps};
}

}  // namespace

VelocityTrajectory::VelocityTrajectory(std::vector<TrajectorySegment> segments)
    : segments_(std::move(segments)) {}

VelocityTrajectory VelocityTrajectory::default_hold(double duration_s) {
    TrajectorySegment hold;
    hold.type = "hold";
    hold.label = "default_hold";
    hold.duration_s = duration_s;
    return VelocityTrajectory{{hold}};
}

bool VelocityTrajectory::empty() const {
    return segments_.empty();
}

std::size_t VelocityTrajectory::size() const {
    return segments_.size();
}

const std::vector<TrajectorySegment>& VelocityTrajectory::segments() const {
    return segments_;
}

const TrajectorySegment& VelocityTrajectory::segment(std::size_t index) const {
    return segments_.at(index);
}

Vec3 VelocityTrajectory::velocity_at(std::size_t segment_index, double segment_elapsed_s) const {
    const auto& segment = segments_.at(segment_index);

    if (segment.type == "hold") {
        return Vec3{segment.vx_mps, segment.vy_mps, segment.vz_mps};
    }
    if (segment.type == "velocity_keyframes") {
        return interpolate_keyframes(segment.keyframes, segment_elapsed_s);
    }
    if (segment.type == "circle_velocity") {
        const double speed = segment.speed_mps > 0.0 ? segment.speed_mps : 2.0;
        const double radius = std::max(segment.radius_m, 1e-6);
        const double sign = segment.direction == "cw" ? -1.0 : 1.0;
        const double theta = speed / radius * segment_elapsed_s;
        return Vec3{
            speed * std::cos(theta),
            sign * speed * std::sin(theta),
            segment.vz_mps};
    }
    if (segment.type == "figure8_velocity") {
        const double duration = std::max(segment.duration_s, 1e-6);
        const double speed = segment.speed_mps > 0.0 ? segment.speed_mps : 2.0;
        const double scale = std::max(segment.scale_m, 1e-6);
        const double theta = 2.0 * kPi * segment_elapsed_s / duration;
        const double dx = scale * std::cos(theta);
        const double dy = scale * std::cos(2.0 * theta);
        const double norm = std::max(std::hypot(dx, dy), 1e-6);
        return Vec3{speed * dx / norm, speed * dy / norm, segment.vz_mps};
    }

    throw std::runtime_error("unknown trajectory segment type: " + segment.type);
}

}  // namespace dedalus
