#include "dedalus/behavior/trajectory_mission_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace dedalus {
namespace {

constexpr double kMinArrivedDistanceM = 0.5;
constexpr double kLandHeightM = 0.25;
constexpr double kTakeoffVelocityAssistHeightM = 0.5;

std::string strip_json_string(std::string value) {
    value.erase(std::remove(value.begin(), value.end(), '"'), value.end());
    value.erase(std::remove(value.begin(), value.end(), '\''), value.end());
    return value;
}

double seconds_between(TimePoint start, TimePoint end) {
    return static_cast<double>(end.timestamp_ns - start.timestamp_ns) / 1'000'000'000.0;
}

bool elapsed_at_least(TimePoint start, TimePoint end, double seconds) {
    return seconds_between(start, end) >= seconds;
}

double norm_xy(const Vec3& value) {
    return std::sqrt(value.x * value.x + value.y * value.y);
}

Vec3 velocity_toward_xy(const Vec3& from, const Vec3& to, double speed_mps) {
    const Vec3 delta{to.x - from.x, to.y - from.y, 0.0};
    const double distance = norm_xy(delta);
    if (distance <= kMinArrivedDistanceM || speed_mps <= 0.0) {
        return Vec3{0.0, 0.0, 0.0};
    }
    return Vec3{delta.x / distance * speed_mps, delta.y / distance * speed_mps, 0.0};
}

}  // namespace

TrajectoryMissionConfig load_trajectory_mission_config(const MissionOptions& options) {
    TrajectoryMissionConfig config;
    config.safe_height_m = std::stod(options.get_or("flight_safe_height_m", "8"));
    config.takeoff_velocity_mps = std::stod(options.get_or("flight_takeoff_velocity_mps", "1.0"));
    config.go_home_velocity_mps = std::stod(options.get_or("flight_go_home_velocity_mps", "1.0"));
    config.land_velocity_mps = std::stod(options.get_or("flight_land_velocity_mps", "0.5"));
    config.arm_retry_interval_s = std::stod(options.get_or("flight_arm_retry_interval_s", "1.0"));
    config.arm_timeout_s = std::stod(options.get_or("flight_arm_timeout_s", "10.0"));
    config.takeoff_retry_interval_s = std::stod(options.get_or("flight_takeoff_retry_interval_s", "1.0"));
    config.land_retry_interval_s = std::stod(options.get_or("flight_land_retry_interval_s", "1.0"));
    config.land_timeout_s = std::stod(options.get_or("flight_land_timeout_s", "60.0"));
    config.disarm_retry_interval_s = std::stod(options.get_or("flight_disarm_retry_interval_s", "1.0"));
    config.disarm_timeout_s = std::stod(options.get_or("flight_disarm_timeout_s", "10.0"));
    config.home_policy = options.get_or("flight_home_policy", "initial_ego_pose");

    const auto trajectory_path = options.get_or("flight_trajectory_path", "");
    if (!trajectory_path.empty()) {
        config.trajectory = VelocityTrajectory::load_from_file(strip_json_string(trajectory_path));
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
    Vec3 velocity_local_mps) const {
    VelocityCommand command;
    command.kind = FlightCommandKind::Velocity;
    command.timestamp = timestamp;
    command.velocity_local_mps = velocity_local_mps;
    command.yaw_rate_radps = 0.0;
    command.yaw_rate_valid = true;
    command.yaw_valid = false;
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
    return command_from_velocity(timestamp, config_.trajectory.velocity_at(segment_index_, segment_elapsed_s_));
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

MissionTickOutput TrajectoryMissionController::tick(const MissionTickInput& input) {
    MissionTickOutput output;
    output.state = state_;

    if (!mission_started_) {
        mission_started_ = true;
        mission_start_ = input.now;
        state_start_ = input.now;
        last_tick_time_ = input.now;
        home_pose_ = input.snapshot.ego.local_T_body;
        home_initialized_ = true;
        state_ = MissionLifecycleState::Prepare;
    }

    const double dt_s = std::max(0.0, seconds_between(last_tick_time_, input.now));
    last_tick_time_ = input.now;

    const auto& ego = input.snapshot.ego;
    const double height_m = ego.height_valid ? ego.height_m : -ego.local_T_body.position.z;

    switch (state_) {
        case MissionLifecycleState::Prepare:
            if (input.finish_requested && ego.armed_valid && !ego.armed) {
                state_ = MissionLifecycleState::Complete;
                state_start_ = input.now;
                output.status = "finish_requested_before_arm";
            } else if (ego.armed_valid && ego.armed) {
                state_ = MissionLifecycleState::Takeoff;
                state_start_ = input.now;
                output.status = "armed_confirmed_by_ego";
            } else if (elapsed_at_least(state_start_, input.now, config_.arm_timeout_s)) {
                state_ = MissionLifecycleState::Abort;
                output.status = "arm_timeout";
            } else if (!arm_command_sent_ || elapsed_at_least(arm_last_command_time_, input.now, config_.arm_retry_interval_s)) {
                arm_command_sent_ = true;
                arm_last_command_time_ = input.now;
                output.command = command_with_kind(input.now, FlightCommandKind::Arm);
                output.status = "arming";
            } else {
                output.status = ego.armed_valid ? "waiting_for_armed_state" : "waiting_for_armed_telemetry";
            }
            break;
        case MissionLifecycleState::Takeoff:
            if (input.finish_requested) {
                state_ = height_m > kLandHeightM ? MissionLifecycleState::Land : MissionLifecycleState::Complete;
                state_start_ = input.now;
                output.status = height_m > kLandHeightM ? "finish_requested_land" : "finish_requested_complete";
            } else if (height_m >= config_.safe_height_m) {
                state_ = MissionLifecycleState::ExecuteMission;
                state_start_ = input.now;
                segment_index_ = 0U;
                segment_elapsed_s_ = 0.0;
                output.status = "takeoff_complete";
            } else if (!takeoff_command_sent_) {
                takeoff_command_sent_ = true;
                takeoff_last_command_time_ = input.now;
                output.command = command_with_kind(input.now, FlightCommandKind::Takeoff);
                output.status = "takeoff_request";
            } else if (height_m >= kTakeoffVelocityAssistHeightM) {
                output.command = command_from_velocity(
                    input.now,
                    Vec3{0.0, 0.0, -std::abs(config_.takeoff_velocity_mps)});
                output.status = "takeoff_climb";
            } else if (elapsed_at_least(takeoff_last_command_time_, input.now, config_.takeoff_retry_interval_s)) {
                output.status = "waiting_for_takeoff_climb";
            } else {
                output.status = "waiting_for_takeoff_command_settle";
            }
            break;
        case MissionLifecycleState::ExecuteMission:
            if (input.finish_requested) {
                state_ = MissionLifecycleState::GoHome;
                state_start_ = input.now;
                output.status = "finish_requested_go_home";
            } else {
                segment_elapsed_s_ += dt_s;
                advance_segment_if_needed();
                if (trajectory_complete()) {
                    state_ = MissionLifecycleState::GoHome;
                    state_start_ = input.now;
                    output.status = "trajectory_complete";
                } else {
                    output.command = trajectory_command(input.now);
                    output.status = "trajectory_execute";
                }
            }
            break;
        case MissionLifecycleState::GoHome: {
            const Vec3 velocity = home_initialized_
                ? velocity_toward_xy(ego.local_T_body.position, home_pose_.position, config_.go_home_velocity_mps)
                : Vec3{0.0, 0.0, 0.0};
            if (norm_xy(velocity) <= 0.0) {
                state_ = MissionLifecycleState::Land;
                state_start_ = input.now;
                output.status = "home_reached";
            } else {
                output.command = command_from_velocity(input.now, velocity);
                output.status = "go_home";
            }
            break;
        }
        case MissionLifecycleState::Land:
            if (height_m <= kLandHeightM) {
                state_ = MissionLifecycleState::Complete;
                state_start_ = input.now;
                output.status = "landed";
            } else if (!land_command_sent_) {
                land_command_sent_ = true;
                land_last_command_time_ = input.now;
                output.command = command_with_kind(input.now, FlightCommandKind::Land);
                output.status = "landing_command_sent";
            } else if (elapsed_at_least(land_last_command_time_, input.now, config_.land_timeout_s)) {
                state_ = MissionLifecycleState::Abort;
                output.status = "land_timeout";
            } else {
                output.status = "waiting_for_landed_telemetry";
            }
            break;
        case MissionLifecycleState::Complete:
            if (ego.armed_valid && !ego.armed) {
                output.status = "complete";
            } else if (elapsed_at_least(state_start_, input.now, config_.disarm_timeout_s)) {
                state_ = MissionLifecycleState::Abort;
                output.status = "disarm_timeout";
            } else if (!disarm_command_sent_ || elapsed_at_least(disarm_last_command_time_, input.now, config_.disarm_retry_interval_s)) {
                disarm_command_sent_ = true;
                disarm_last_command_time_ = input.now;
                output.command = command_with_kind(input.now, FlightCommandKind::Disarm);
                output.status = "disarming";
            } else {
                output.status = ego.armed_valid ? "waiting_for_disarmed_state" : "waiting_for_disarmed_telemetry";
            }
            break;
        case MissionLifecycleState::Abort:
            output.status = "abort";
            break;
        case MissionLifecycleState::Idle:
        default:
            state_ = MissionLifecycleState::Prepare;
            state_start_ = input.now;
            output.status = "idle";
            break;
    }

    output.state = state_;
    return output;
}

}  // namespace dedalus