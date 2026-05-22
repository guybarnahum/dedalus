#include "dedalus/behavior/object_behavior_mission_controller.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace dedalus {
namespace {

constexpr double kMinArrivedDistanceM = 0.5;
constexpr double kLandHeightM = 0.25;
constexpr double kTakeoffVelocityAssistHeightM = 0.5;
constexpr double kHeadingEpsilonMps = 0.05;
constexpr double kSafeHeightHoldBandM = 1.0;
constexpr double kSafeHeightCorrectionGain = 0.5;
constexpr double kMinSafeHeightClimbMps = 0.35;

std::string json_escape(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size());
    for (const char ch : value) {
        switch (ch) {
            case '\\':
                escaped += "\\\\";
                break;
            case '"':
                escaped += "\\\"";
                break;
            case '\n':
                escaped += "\\n";
                break;
            case '\r':
                escaped += "\\r";
                break;
            case '\t':
                escaped += "\\t";
                break;
            default:
                escaped.push_back(ch);
                break;
        }
    }
    return escaped;
}

std::string q(const std::string& value) {
    return "\"" + json_escape(value) + "\"";
}

double seconds_between(TimePoint start, TimePoint end) {
    return static_cast<double>(end.timestamp_ns - start.timestamp_ns) / 1'000'000'000.0;
}

bool elapsed_at_least(TimePoint start, TimePoint end, double seconds) {
    return seconds_between(start, end) >= seconds;
}

bool last_result_matches_success(
    const std::optional<FlightCommandResult>& result,
    FlightCommandKind kind) {
    return result.has_value() && result->kind == kind && result->success;
}

double norm_xy(const Vec3& value) {
    return std::sqrt(value.x * value.x + value.y * value.y);
}

double norm3(const Vec3& value) {
    return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

double clamp_abs(double value, double limit) {
    if (limit <= 0.0) {
        return 0.0;
    }
    return std::clamp(value, -limit, limit);
}

Vec3 velocity_toward_xy(const Vec3& from, const Vec3& to, double speed_mps) {
    const Vec3 delta{to.x - from.x, to.y - from.y, 0.0};
    const double distance = norm_xy(delta);
    if (distance <= kMinArrivedDistanceM || speed_mps <= 0.0) {
        return Vec3{0.0, 0.0, 0.0};
    }
    return Vec3{delta.x / distance * speed_mps, delta.y / distance * speed_mps, 0.0};
}

Vec3 clamp_velocity(const Vec3& desired, double max_horizontal_mps, double max_vertical_mps) {
    Vec3 velocity = desired;
    const double horizontal = norm_xy(velocity);
    if (horizontal > max_horizontal_mps && max_horizontal_mps > 0.0) {
        const double scale = max_horizontal_mps / horizontal;
        velocity.x *= scale;
        velocity.y *= scale;
    }
    velocity.z = clamp_abs(velocity.z, max_vertical_mps);
    return velocity;
}

Vec3 enforce_safe_height_floor(
    Vec3 velocity,
    double height_m,
    double safe_height_m,
    double max_vertical_speed_mps) {
    if (safe_height_m <= 0.0 || max_vertical_speed_mps <= 0.0) {
        return velocity;
    }

    // LOCAL_NED convention: negative vz climbs, positive vz descends. During ExecuteMission/GoHome,
    // behavior-relative altitude must not command descent into roofs or terrain below the mission floor.
    if (height_m < safe_height_m) {
        const double climb = std::clamp(
            (safe_height_m - height_m) * kSafeHeightCorrectionGain,
            kMinSafeHeightClimbMps,
            max_vertical_speed_mps);
        velocity.z = std::min(velocity.z, -climb);
    } else if (height_m <= safe_height_m + kSafeHeightHoldBandM && velocity.z > 0.0) {
        velocity.z = 0.0;
    }
    return velocity;
}

Vec3 desired_follow_position(const EgoState& ego, const TargetSelection& selection, const BehaviorSpec& behavior) {
    const auto& offset = behavior.relative_offset_m;
    Vec3 desired = selection.position_local;
    const double target_speed_xy = norm_xy(selection.velocity_local);
    if (behavior.target_frame == ReferenceFrame::TargetHeadingFrame && target_speed_xy > kHeadingEpsilonMps) {
        const double forward_x = selection.velocity_local.x / target_speed_xy;
        const double forward_y = selection.velocity_local.y / target_speed_xy;
        const double right_x = -forward_y;
        const double right_y = forward_x;
        desired.x += forward_x * offset.x + right_x * offset.y;
        desired.y += forward_y * offset.x + right_y * offset.y;
    } else if (behavior.target_frame == ReferenceFrame::DroneHeadingFrame) {
        const double yaw = ego.local_T_body.rotation_rpy.z;
        const double forward_x = std::cos(yaw);
        const double forward_y = std::sin(yaw);
        const double right_x = -forward_y;
        const double right_y = forward_x;
        desired.x += forward_x * offset.x + right_x * offset.y;
        desired.y += forward_y * offset.x + right_y * offset.y;
    } else {
        desired.x += offset.x;
        desired.y += offset.y;
    }
    desired.z -= offset.z;
    return desired;
}

Vec3 bounded_follow_velocity(const EgoState& ego, const TargetSelection& selection, const BehaviorSpec& behavior) {
    const Vec3 desired = desired_follow_position(ego, selection, behavior);
    const Vec3 error{
        desired.x - ego.local_T_body.position.x,
        desired.y - ego.local_T_body.position.y,
        desired.z - ego.local_T_body.position.z};
    if (norm3(error) <= behavior.position_tolerance_m) {
        return Vec3{0.0, 0.0, 0.0};
    }
    return clamp_velocity(error, behavior.max_speed_mps, behavior.max_vertical_speed_mps);
}

Vec3 behavior_velocity(const EgoState& ego, const TargetSelection& selection, const BehaviorSpec& behavior) {
    if (behavior.type == BehaviorType::Follow) {
        return bounded_follow_velocity(ego, selection, behavior);
    }
    return Vec3{0.0, 0.0, 0.0};
}

std::string class_label_event_string(ClassLabel label) {
    switch (label) {
        case ClassLabel::Person:
            return "person";
        case ClassLabel::Drone:
            return "drone";
        case ClassLabel::Car:
            return "car";
        case ClassLabel::Boat:
            return "boat";
        case ClassLabel::House:
            return "house";
        case ClassLabel::Building:
            return "building";
        case ClassLabel::Tree:
            return "tree";
        case ClassLabel::Road:
            return "road";
        case ClassLabel::River:
            return "river";
        case ClassLabel::Terrain:
            return "terrain";
        case ClassLabel::Unknown:
        default:
            return "unknown";
    }
}

}  // namespace

ObjectBehaviorMissionConfig load_object_behavior_mission_config(const MissionOptions& options) {
    ObjectBehaviorMissionConfig config;
    const auto behavior_spec_path = options.get_or("behavior_spec_path", "");
    if (behavior_spec_path.empty()) {
        throw std::invalid_argument("object_behavior mission_controller requires mission_options.behavior_spec_path");
    }
    config.behavior_spec = parse_behavior_spec_file(behavior_spec_path);
    config.hold_velocity_mps = std::stod(options.get_or("object_behavior_hold_velocity_mps", "0.0"));
    config.yaw_offset_rad = std::stod(options.get_or(
        "object_behavior_yaw_offset_rad",
        options.get_or("flight_yaw_offset_rad", "0.0")));
    const auto completion_after_override = options.get_or("object_behavior_completion_after_s", "");
    if (!completion_after_override.empty()) {
        config.behavior_spec.completion.after_s = std::stod(completion_after_override);
    }
    config.safe_height_m = std::stod(options.get_or("flight_safe_height_m", "8"));
    config.takeoff_velocity_mps = std::stod(options.get_or("flight_takeoff_velocity_mps", "1.0"));
    config.go_home_velocity_mps = std::stod(options.get_or("flight_go_home_velocity_mps", "1.0"));
    config.arm_retry_interval_s = std::stod(options.get_or("flight_arm_retry_interval_s", "1.0"));
    config.arm_timeout_s = std::stod(options.get_or("flight_arm_timeout_s", "10.0"));
    config.arm_dispatch_fallback_s = std::stod(options.get_or("flight_arm_dispatch_fallback_s", "0.0"));
    config.takeoff_retry_interval_s = std::stod(options.get_or("flight_takeoff_retry_interval_s", "1.0"));
    config.land_retry_interval_s = std::stod(options.get_or("flight_land_retry_interval_s", "1.0"));
    config.land_timeout_s = std::stod(options.get_or("flight_land_timeout_s", "60.0"));
    config.disarm_retry_interval_s = std::stod(options.get_or("flight_disarm_retry_interval_s", "1.0"));
    config.disarm_timeout_s = std::stod(options.get_or("flight_disarm_timeout_s", "10.0"));
    config.home_policy = options.get_or("flight_home_policy", "initial_ego_pose");
    return config;
}

ObjectBehaviorMissionController::ObjectBehaviorMissionController(ObjectBehaviorMissionConfig config)
    : config_(std::move(config)) {}

VelocityCommand ObjectBehaviorMissionController::command_from_velocity(
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
    const double horizontal = norm_xy(velocity_local_mps);
    if (horizontal > kHeadingEpsilonMps) {
        command.yaw_valid = true;
        command.yaw_rad = std::atan2(velocity_local_mps.y, velocity_local_mps.x) + yaw_offset_rad;
    }
    return command;
}

VelocityCommand ObjectBehaviorMissionController::command_with_kind(
    TimePoint timestamp, FlightCommandKind kind) const {
    VelocityCommand command;
    command.kind = kind;
    command.timestamp = timestamp;
    command.velocity_local_mps = Vec3{0.0, 0.0, 0.0};
    command.yaw_rate_valid = false;
    command.yaw_valid = false;
    return command;
}

std::string ObjectBehaviorMissionController::target_event(const TargetSelection& selection) const {
    return "\"event\":\"target_selected\""
        ",\"agent_id\":" + q(selection.agent_id.value) +
        ",\"source_track_id\":" + q(selection.source_track_id.value) +
        ",\"identity_id\":" + q(selection.identity_id.value) +
        ",\"class\":" + q(class_label_event_string(selection.class_label)) +
        ",\"confidence\":" + std::to_string(selection.confidence) +
        ",\"reason\":" + q(selection.reason);
}

std::string ObjectBehaviorMissionController::behavior_event(
    const std::string& event,
    const std::string& reason) const {
    std::string fields = "\"event\":" + q(event) +
        ",\"behavior\":" + q(to_string(config_.behavior_spec.behavior.type)) +
        ",\"mission\":" + q(config_.behavior_spec.mission_name) +
        ",\"reason\":" + q(reason);
    if (previous_selection_.has_value()) {
        fields += ",\"agent_id\":" + q(previous_selection_->agent_id.value) +
            ",\"source_track_id\":" + q(previous_selection_->source_track_id.value) +
            ",\"identity_id\":" + q(previous_selection_->identity_id.value);
    }
    return fields;
}

std::string behavior_tick_event(const BehaviorMissionSpec& spec, const TargetSelection& selection, const Vec3& velocity) {
    return "\"event\":\"behavior_tick_sample\""
        ",\"behavior\":" + q(to_string(spec.behavior.type)) +
        ",\"mission\":" + q(spec.mission_name) +
        ",\"agent_id\":" + q(selection.agent_id.value) +
        ",\"source_track_id\":" + q(selection.source_track_id.value) +
        ",\"vx\":" + std::to_string(velocity.x) +
        ",\"vy\":" + std::to_string(velocity.y) +
        ",\"vz\":" + std::to_string(velocity.z);
}

bool ObjectBehaviorMissionController::completion_elapsed(TimePoint now) const {
    const double after_s = config_.behavior_spec.completion.after_s;
    if (after_s <= 0.0) {
        return false;
    }
    return seconds_between(behavior_start_, now) >= after_s;
}

Vec3 ObjectBehaviorMissionController::go_home_velocity(const EgoState& ego) const {
    if (!home_initialized_) {
        return Vec3{0.0, 0.0, 0.0};
    }
    return velocity_toward_xy(ego.local_T_body.position, home_pose_.position, config_.go_home_velocity_mps);
}

void ObjectBehaviorMissionController::begin_abort_recovery(
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

void ObjectBehaviorMissionController::reset_behavior_run(TimePoint now) {
    behavior_start_ = now;
    target_selected_emitted_ = false;
    behavior_start_emitted_ = false;
    behavior_complete_emitted_ = false;
    previous_selection_.reset();
}

MissionTickOutput ObjectBehaviorMissionController::tick(const MissionTickInput& input) {
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
            } else if (config_.arm_dispatch_fallback_s > 0.0 &&
                       last_result_matches_success(input.last_command_result, FlightCommandKind::Arm) &&
                       elapsed_at_least(arm_last_command_time_, input.now, config_.arm_dispatch_fallback_s)) {
                state_ = MissionLifecycleState::Takeoff;
                state_start_ = input.now;
                output.status = "arm_dispatch_ok_waiting_for_takeoff_height";
            } else if (elapsed_at_least(state_start_, input.now, config_.arm_timeout_s)) {
                if (ego.armed_valid && !ego.armed) {
                    state_ = MissionLifecycleState::Abort;
                    output.status = "abort";
                } else {
                    begin_abort_recovery(input.now, height_m, "arm_timeout");
                    output.status = "abort_recovery_start_arm_timeout";
                }
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
                reset_behavior_run(input.now);
                output.status = "takeoff_complete";
            } else if (!takeoff_command_sent_) {
                takeoff_command_sent_ = true;
                takeoff_last_command_time_ = input.now;
                output.command = command_with_kind(input.now, FlightCommandKind::Takeoff);
                output.status = "takeoff_request";
            } else if (height_m >= kTakeoffVelocityAssistHeightM) {
                output.command = command_from_velocity(input.now, Vec3{0.0, 0.0, -std::abs(config_.takeoff_velocity_mps)});
                output.status = "takeoff_climb";
            } else if (elapsed_at_least(takeoff_last_command_time_, input.now, config_.takeoff_retry_interval_s)) {
                output.status = "waiting_for_takeoff_climb";
            } else {
                output.status = "waiting_for_takeoff_command_settle";
            }
            break;
        case MissionLifecycleState::ExecuteMission: {
            auto selection = selector_.select(input.snapshot, config_.behavior_spec.target, previous_selection_);
            if (selection.selected) {
                previous_selection_ = selection;
                if (!target_selected_emitted_) {
                    target_selected_emitted_ = true;
                    output.events.push_back(target_event(selection));
                }
                if (!behavior_start_emitted_) {
                    behavior_start_emitted_ = true;
                    behavior_start_ = input.now;
                    output.events.push_back(behavior_event("behavior_start", "target_selected"));
                }
                if (input.finish_requested || completion_elapsed(input.now)) {
                    if (!behavior_complete_emitted_) {
                        behavior_complete_emitted_ = true;
                        output.events.push_back(behavior_event(
                            "behavior_complete",
                            input.finish_requested ? "finish_requested" : "duration_elapsed"));
                    }
                    state_ = MissionLifecycleState::GoHome;
                    state_start_ = input.now;
                    output.status = input.finish_requested ? "object_behavior_finish_requested" : "object_behavior_complete";
                } else {
                    const Vec3 raw_velocity = behavior_velocity(ego, selection, config_.behavior_spec.behavior);
                    const Vec3 velocity = enforce_safe_height_floor(
                        raw_velocity,
                        height_m,
                        config_.safe_height_m,
                        config_.behavior_spec.behavior.max_vertical_speed_mps);
                    output.command = command_from_velocity(
                        input.now,
                        velocity,
                        config_.yaw_offset_rad + config_.behavior_spec.behavior.yaw_offset_rad);
                    if (config_.behavior_spec.behavior.type == BehaviorType::Follow && !behavior_tick_sample_emitted_) {
                        behavior_tick_sample_emitted_ = true;
                        output.events.push_back(behavior_tick_event(config_.behavior_spec, selection, velocity));
                    }
                    output.status = config_.behavior_spec.behavior.type == BehaviorType::Follow ? "object_behavior_follow" : "object_behavior_hold";
                }
            } else if (input.finish_requested) {
                state_ = MissionLifecycleState::GoHome;
                state_start_ = input.now;
                output.status = "object_behavior_finish_requested_no_target";
            } else {
                output.command = command_from_velocity(input.now, Vec3{0.0, 0.0, 0.0});
                output.status = "object_behavior_waiting_for_target_" + to_string(selection.status);
            }
            break;
        }
        case MissionLifecycleState::GoHome: {
            const Vec3 raw_velocity = go_home_velocity(ego);
            const Vec3 velocity = enforce_safe_height_floor(
                raw_velocity,
                height_m,
                config_.safe_height_m,
                config_.behavior_spec.behavior.max_vertical_speed_mps);
            if (norm_xy(velocity) <= 0.0 && height_m >= config_.safe_height_m) {
                state_ = MissionLifecycleState::Land;
                state_start_ = input.now;
                output.status = aborting_ ? "abort_recovery_home_reached" : "home_reached";
            } else {
                output.command = command_from_velocity(input.now, velocity, config_.yaw_offset_rad);
                output.status = aborting_ ? "abort_recovery_go_home" : "go_home";
            }
            break;
        }
        case MissionLifecycleState::Land:
            if (height_m <= kLandHeightM) {
                state_ = MissionLifecycleState::Complete;
                state_start_ = input.now;
                output.status = aborting_ ? "abort_recovery_landed" : "landed";
            } else if (!land_command_sent_) {
                land_command_sent_ = true;
                land_last_command_time_ = input.now;
                output.command = command_with_kind(input.now, FlightCommandKind::Land);
                output.status = aborting_ ? "abort_recovery_landing_command_sent" : "landing_command_sent";
            } else if (elapsed_at_least(land_last_command_time_, input.now, config_.land_timeout_s)) {
                state_ = MissionLifecycleState::Abort;
                output.status = "abort";
            } else {
                output.status = aborting_ ? "abort_recovery_waiting_for_landed_telemetry" : "waiting_for_landed_telemetry";
            }
            break;
        case MissionLifecycleState::Complete:
            if (ego.armed_valid && !ego.armed) {
                if (aborting_) {
                    state_ = MissionLifecycleState::Abort;
                    output.status = "abort";
                } else {
                    output.status = "complete";
                }
            } else if (elapsed_at_least(state_start_, input.now, config_.disarm_timeout_s)) {
                state_ = MissionLifecycleState::Abort;
                output.status = "abort";
            } else if (!disarm_command_sent_ || elapsed_at_least(disarm_last_command_time_, input.now, config_.disarm_retry_interval_s)) {
                disarm_command_sent_ = true;
                disarm_last_command_time_ = input.now;
                output.command = command_with_kind(input.now, FlightCommandKind::Disarm);
                output.status = aborting_ ? "abort_recovery_disarming" : "disarming";
            } else {
                output.status = aborting_
                    ? (ego.armed_valid ? "abort_recovery_waiting_for_disarmed_state" : "abort_recovery_waiting_for_disarmed_telemetry")
                    : (ego.armed_valid ? "waiting_for_disarmed_state" : "waiting_for_disarmed_telemetry");
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
