#include "dedalus/behavior/trajectory_mission_controller.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace dedalus {
namespace {

constexpr double kLandHeightM = 0.25;
constexpr double kHeadingEpsilonMps = 0.05;

std::string strip_json_string(std::string value) {
    value.erase(std::remove(value.begin(), value.end(), '"'), value.end());
    value.erase(std::remove(value.begin(), value.end(), '\''), value.end());
    return value;
}

double norm_xy(const Vec3& value) {
    return std::sqrt(value.x * value.x + value.y * value.y);
}

}  // namespace

TrajectoryMissionConfig load_trajectory_mission_config(const MissionOptions& options) {
    TrajectoryMissionConfig config;
    config.safe_height_m           = options.safe_height_m;
    config.takeoff_velocity_mps    = options.takeoff_velocity_mps;
    config.go_home_velocity_mps    = options.go_home_velocity_mps;
    config.land_velocity_mps       = options.land_velocity_mps;
    config.yaw_offset_rad          = options.yaw_offset_rad;
    config.arm_retry_interval_s    = options.arm_retry_interval_s;
    config.arm_timeout_s           = options.arm_timeout_s;
    config.arm_dispatch_fallback_s = options.arm_dispatch_fallback_s;
    config.takeoff_retry_interval_s = options.takeoff_retry_interval_s;
    config.land_retry_interval_s   = options.land_retry_interval_s;
    config.land_timeout_s          = options.land_timeout_s;
    config.disarm_retry_interval_s = options.disarm_retry_interval_s;
    config.disarm_timeout_s        = options.disarm_timeout_s;
    config.home_policy             = options.home_policy;

    if (!options.trajectory_path.empty()) {
        config.trajectory = VelocityTrajectory::load_from_file(strip_json_string(options.trajectory_path));
    }
    if (config.trajectory.empty()) {
        config.trajectory = VelocityTrajectory::default_hold();
    }
    return config;
}

TrajectoryMissionController::TrajectoryMissionController(TrajectoryMissionConfig config)
    : config_(std::move(config)) {
    if (config_.trajectory.empty()) {
        config_.trajectory = VelocityTrajectory::default_hold();
    }
}

VelocityCommand TrajectoryMissionController::command_from_velocity(
    TimePoint timestamp,
    Vec3 velocity_local_mps,
    double yaw_offset_rad) const {
    VelocityCommand command;
    command.kind = FlightCommandKind::Velocity;
    command.timestamp = timestamp;
    command.velocity_local_mps = velocity_local_mps;
    command.yaw_rate_radps = 0.0;
    command.yaw_rate_valid = true;
    command.yaw_valid = false;
    if (norm_xy(velocity_local_mps) > kHeadingEpsilonMps) {
        command.yaw_valid = true;
        command.yaw_rad = std::atan2(velocity_local_mps.y, velocity_local_mps.x) + yaw_offset_rad;
    }
    return command;
}

VelocityCommand TrajectoryMissionController::command_with_kind(
    TimePoint timestamp,
    FlightCommandKind kind) const {
    VelocityCommand command;
    command.kind = kind;
    command.timestamp = timestamp;
    command.velocity_local_mps = Vec3{0.0, 0.0, 0.0};
    command.yaw_rate_valid = false;
    command.yaw_valid = false;
    return command;
}

VelocityCommand TrajectoryMissionController::trajectory_command(TimePoint timestamp) const {
    const auto& segment = config_.trajectory.segment(segment_index_);
    return command_from_velocity(
        timestamp,
        config_.trajectory.velocity_at(segment_index_, segment_elapsed_s_),
        config_.yaw_offset_rad + segment.yaw_offset_rad);
}

bool TrajectoryMissionController::trajectory_complete() const {
    return segment_index_ >= config_.trajectory.size();
}

void TrajectoryMissionController::advance_segment_if_needed() {
    while (!trajectory_complete() && segment_elapsed_s_ >= config_.trajectory.segment(segment_index_).duration_s) {
        segment_elapsed_s_ -= config_.trajectory.segment(segment_index_).duration_s;
        ++segment_index_;
    }
}

void TrajectoryMissionController::begin_abort_recovery(
    TimePoint now,
    double height_m,
    const std::string& reason) {
    aborting_ = true;
    abort_reason_ = reason;
    state_start_ = now;
    if (height_m > kLandHeightM) {
        state_ = home_initialized_ ? MissionLifecycleState::GoHome : MissionLifecycleState::Land;
    } else {
        state_ = MissionLifecycleState::Complete;
    }
}

}  // namespace dedalus
