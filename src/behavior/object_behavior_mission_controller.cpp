#include "dedalus/behavior/object_behavior_mission_controller.hpp"

#include "dedalus/behavior/behavior_osd.hpp"
#include "dedalus/behavior/follow_geometry_policy.hpp"
#include "dedalus/core/json_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dedalus {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kLandHeightM = 0.25;
constexpr double kTakeoffVelocityAssistHeightM = 0.5;
constexpr double kMinArrivedDistanceM = 0.5;

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

double rad_to_deg(double rad) {
    return rad * 180.0 / kPi;
}

double norm_xy(const Vec3& value) {
    return std::sqrt(value.x * value.x + value.y * value.y);
}

double circle_direction_sign(CircleDirection direction) {
    return direction == CircleDirection::Clockwise ? -1.0 : 1.0;
}

Vec3 velocity_toward_xy(const Vec3& from, const Vec3& to, double speed_mps) {
    const Vec3 delta{to.x - from.x, to.y - from.y, 0.0};
    const double distance = norm_xy(delta);
    if (distance <= kMinArrivedDistanceM || speed_mps <= 0.0) {
        return Vec3{0.0, 0.0, 0.0};
    }
    return Vec3{delta.x / distance * speed_mps, delta.y / distance * speed_mps, 0.0};
}

std::string json_string_array(const std::vector<std::string>& values) {
    std::string out = "[";
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i > 0) out += ",";
        out += q(values[i]);
    }
    out += "]";
    return out;
}

ObjectBehaviorYawMode parse_yaw_mode(const std::string& value) {
    if (value.empty() || value == "trajectory" || value == "travel_direction" || value == "from_heading") {
        return ObjectBehaviorYawMode::Trajectory;
    }
    if (value == "target" || value == "to_target" || value == "stare_at_target") {
        return ObjectBehaviorYawMode::Target;
    }
    if (value == "hold") {
        return ObjectBehaviorYawMode::Hold;
    }
    if (value == "none" || value == "disabled") {
        return ObjectBehaviorYawMode::None;
    }
    throw std::invalid_argument("unknown object_behavior_yaw_mode: " + value);
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
        case ClassLabel::Animal:
            return "animal";
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
    command.yaw_source = "disabled";

    const double horizontal = norm_xy(velocity_local_mps);
    if (horizontal >= config_.yaw_min_speed_mps) {
        command.yaw_valid = true;
        command.yaw_rad = std::atan2(velocity_local_mps.y, velocity_local_mps.x) + yaw_offset_rad;
        command.yaw_source = "trajectory";
        last_stable_yaw_valid_ = true;
        last_stable_yaw_rad_ = command.yaw_rad;
    } else if (config_.yaw_hold_last_when_unstable && last_stable_yaw_valid_) {
        command.yaw_valid = true;
        command.yaw_rad = last_stable_yaw_rad_;
        command.yaw_source = "hold_last";
    }
    return command;
}

VelocityCommand ObjectBehaviorMissionController::command_from_behavior_velocity(
    TimePoint timestamp,
    Vec3 velocity_local_mps,
    const EgoState& ego,
    const TargetSelection& selection,
    double yaw_offset_rad,
    ObjectBehaviorYawMode yaw_mode) const {
    if (yaw_mode == ObjectBehaviorYawMode::Trajectory) {
        return command_from_velocity(timestamp, velocity_local_mps, yaw_offset_rad);
    }

    VelocityCommand command;
    command.kind = FlightCommandKind::Velocity;
    command.timestamp = timestamp;
    command.velocity_local_mps = velocity_local_mps;
    command.yaw_rate_radps = 0.0;
    command.yaw_rate_valid = true;
    command.yaw_valid = false;
    command.yaw_source = "disabled";

    if (yaw_mode == ObjectBehaviorYawMode::None) {
        return command;
    }

    if (yaw_mode == ObjectBehaviorYawMode::Hold) {
        if (last_stable_yaw_valid_) {
            command.yaw_valid = true;
            command.yaw_rad = last_stable_yaw_rad_;
            command.yaw_source = "hold_last";
        }
        return command;
    }

    if (yaw_mode == ObjectBehaviorYawMode::Target) {
        const Vec3 target_delta{
            selection.position_local.x - ego.local_T_body.position.x,
            selection.position_local.y - ego.local_T_body.position.y,
            0.0};
        const double target_range_xy_m = norm_xy(target_delta);
        if (target_range_xy_m > 1e-6) {
            command.yaw_valid = true;
            command.yaw_rad = std::atan2(target_delta.y, target_delta.x) + yaw_offset_rad;
            command.yaw_source = "target";
            last_stable_yaw_valid_ = true;
            last_stable_yaw_rad_ = command.yaw_rad;
        } else if (config_.yaw_hold_last_when_unstable && last_stable_yaw_valid_) {
            command.yaw_valid = true;
            command.yaw_rad = last_stable_yaw_rad_;
            command.yaw_source = "hold_last";
        }
        return command;
    }

    return command_from_velocity(timestamp, velocity_local_mps, yaw_offset_rad);
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

ControllerEvent ObjectBehaviorMissionController::target_event(const TargetSelection& selection) const {
    return {ControllerEventKind::TargetSelected,
        ",\"agent_id\":" + q(selection.agent_id.value) +
        ",\"source_track_id\":" + q(selection.source_track_id.value) +
        ",\"identity_id\":" + q(selection.identity_id.value) +
        ",\"class\":" + q(class_label_event_string(selection.class_label)) +
        ",\"confidence\":" + std::to_string(selection.confidence) +
        ",\"reason\":" + q(selection.reason)};
}

ControllerEvent ObjectBehaviorMissionController::behavior_event(
    ControllerEventKind kind,
    const std::string& reason) const {
    std::string fields =
        ",\"behavior\":" + q(to_string(config_.behavior_spec.behavior.type)) +
        ",\"mission\":" + q(config_.behavior_spec.mission_name) +
        ",\"reason\":" + q(reason);
    fields += behavior_display_fields(behavior_detail_for_event(kind));
    if (previous_selection_.has_value()) {
        fields += ",\"agent_id\":" + q(previous_selection_->agent_id.value) +
            ",\"source_track_id\":" + q(previous_selection_->source_track_id.value) +
            ",\"identity_id\":" + q(previous_selection_->identity_id.value);
    }
    return {kind, std::move(fields)};
}

ControllerEvent ObjectBehaviorMissionController::sequence_step_event(
    ControllerEventKind kind,
    const BehaviorSpec& behavior,
    std::size_t index,
    const std::string& reason) const {
    return {kind,
        ",\"behavior\":\"sequence\"" +
        std::string{",\"step_index\":"} + std::to_string(index) +
        ",\"step_behavior\":" + q(to_string(behavior.type)) +
        ",\"step_yaw_mode\":" + q(behavior.yaw_mode.empty() ? "inherit" : behavior.yaw_mode) +
        ",\"step_camera_pointing_mode\":" + q(behavior.camera_pointing_mode.empty() ? "inherit" : behavior.camera_pointing_mode) +
        ",\"mission\":" + q(config_.behavior_spec.mission_name) +
        ",\"reason\":" + q(reason) +
        behavior_display_fields(behavior_detail_for_event(kind))};
}

bool ObjectBehaviorMissionController::completion_elapsed(TimePoint now) const {
    const double after_s = config_.behavior_spec.completion.after_s;
    if (after_s <= 0.0) {
        return false;
    }
    return seconds_between(behavior_start_, now) >= after_s;
}

std::optional<CameraPointingCommand> ObjectBehaviorMissionController::camera_pointing_command(
    TimePoint timestamp,
    const EgoState& ego,
    const TargetSelection& selection,
    const std::string& mode) const {
    const std::string effective_mode = mode.empty() ? "target" : mode;
    if (effective_mode == "neutral" || effective_mode == "reset") {
        return neutral_camera_pointing_command(timestamp, effective_mode);
    }
    if ((effective_mode == "home" || effective_mode == "landing_area") && home_initialized_) {
        return camera_pointing_command_to_point(timestamp, ego, home_pose_.position, effective_mode);
    }
    auto command = camera_pointing_command_to_point(
        timestamp,
        ego,
        selection.position_local,
        effective_mode);
    if (!command) {
        return std::nullopt;
    }
    command->source_track_id = selection.source_track_id.value;
    command->agent_id = selection.agent_id.value;
    command->identity_id = selection.identity_id.value;
    return command;
}

std::optional<CameraPointingCommand> ObjectBehaviorMissionController::camera_pointing_command_for_behavior(
    TimePoint timestamp,
    const EgoState& ego,
    const TargetSelection& selection,
    const BehaviorSpec& behavior) const {
    const std::string mode = behavior.camera_pointing_mode.empty()
        ? "target"
        : behavior.camera_pointing_mode;
    return camera_pointing_command(timestamp, ego, selection, mode);
}

std::optional<CameraPointingCommand> ObjectBehaviorMissionController::camera_pointing_command_to_point(
    TimePoint timestamp,
    const EgoState& ego,
    const Vec3& target_position_local,
    const std::string& mode) const {
    if (config_.vertical_stare_mode == ObjectBehaviorVerticalStareMode::None || mode == "none" || mode == "disabled") {
        return std::nullopt;
    }
    if (mode == "neutral" || mode == "reset") {
        return neutral_camera_pointing_command(timestamp, mode);
    }

    CameraPointingCommand command;
    command.timestamp = timestamp;
    command.cameras = config_.camera_pointing_cameras;
    if (command.cameras.empty()) {
        command.cameras.push_back("front_center");
    }
    command.mode = mode;

    const double dx = target_position_local.x - ego.local_T_body.position.x;
    const double dy = target_position_local.y - ego.local_T_body.position.y;
    const double dz = target_position_local.z - ego.local_T_body.position.z;
    const double raw_range_xy = std::hypot(dx, dy);
    const double range_xy = std::max(raw_range_xy, 1e-6);
    const double elevation_rad = std::atan2(dz, range_xy);
    const double bearing_rad = raw_range_xy > 1e-6 ? std::atan2(dy, dx) : ego.local_T_body.rotation_rpy.z;
    const double unclamped_pitch_rad = config_.camera_pitch_sign * elevation_rad + config_.camera_pitch_offset_rad;
    const double pitch_rad = std::max(config_.camera_pitch_min_rad, std::min(config_.camera_pitch_max_rad, unclamped_pitch_rad));

    command.pitch_rad = pitch_rad;
    command.pitch_unclamped_rad = unclamped_pitch_rad;
    command.pitch_min_rad = config_.camera_pitch_min_rad;
    command.pitch_max_rad = config_.camera_pitch_max_rad;
    command.target_elevation_rad = elevation_rad;
    command.target_bearing_rad = bearing_rad;
    command.yaw_rad = bearing_rad;
    command.yaw_valid = true;
    command.range_xy_m = range_xy;
    command.delta_z_m = dz;
    command.pitch_sign = config_.camera_pitch_sign;
    command.pitch_offset_rad = config_.camera_pitch_offset_rad;
    command.pitch_valid = true;
    command.pitch_clamped = std::abs(pitch_rad - unclamped_pitch_rad) > 1e-9;
    return command;
}

std::optional<CameraPointingCommand> ObjectBehaviorMissionController::neutral_camera_pointing_command(
    TimePoint timestamp,
    const std::string& mode) const {
    if (config_.vertical_stare_mode == ObjectBehaviorVerticalStareMode::None || mode == "none" || mode == "disabled") {
        return std::nullopt;
    }

    CameraPointingCommand command;
    command.timestamp = timestamp;
    command.cameras = config_.camera_pointing_cameras;
    if (command.cameras.empty()) {
        command.cameras.push_back("front_center");
    }
    command.mode = mode;
    command.pitch_rad = 0.0;
    command.pitch_unclamped_rad = 0.0;
    command.pitch_min_rad = config_.camera_pitch_min_rad;
    command.pitch_max_rad = config_.camera_pitch_max_rad;
    command.target_elevation_rad = 0.0;
    command.target_bearing_rad = 0.0;
    command.yaw_rad = 0.0;
    command.yaw_valid = true;
    command.range_xy_m = 0.0;
    command.delta_z_m = 0.0;
    command.pitch_sign = config_.camera_pitch_sign;
    command.pitch_offset_rad = config_.camera_pitch_offset_rad;
    command.pitch_valid = true;
    command.pitch_clamped = false;
    return command;
}

ControllerEvent ObjectBehaviorMissionController::camera_pointing_intent_event(
    const CameraPointingCommand& command) const {
    return {ControllerEventKind::CameraPointingIntent,
        ",\"camera_pointing_mode\":" + q(command.mode) +
        ",\"vertical_stare_mode\":" + q(config_.vertical_stare_mode == ObjectBehaviorVerticalStareMode::Gimbal ? "gimbal" : "debug_only") +
        ",\"cameras\":" + json_string_array(command.cameras) +
        ",\"pitch_valid\":" + std::string(command.pitch_valid ? "true" : "false") +
        ",\"pitch_rad\":" + std::to_string(command.pitch_rad) +
        ",\"pitch_deg\":" + std::to_string(rad_to_deg(command.pitch_rad)) +
        ",\"pitch_unclamped_rad\":" + std::to_string(command.pitch_unclamped_rad) +
        ",\"pitch_unclamped_deg\":" + std::to_string(rad_to_deg(command.pitch_unclamped_rad)) +
        ",\"pitch_min_rad\":" + std::to_string(command.pitch_min_rad) +
        ",\"pitch_max_rad\":" + std::to_string(command.pitch_max_rad) +
        ",\"pitch_sign\":" + std::to_string(command.pitch_sign) +
        ",\"pitch_offset_rad\":" + std::to_string(command.pitch_offset_rad) +
        ",\"pitch_clamped\":" + std::string(command.pitch_clamped ? "true" : "false") +
        ",\"yaw_valid\":" + std::string(command.yaw_valid ? "true" : "false") +
        ",\"yaw_rad\":" + std::to_string(command.yaw_rad) +
        ",\"yaw_deg\":" + std::to_string(rad_to_deg(command.yaw_rad)) +
        ",\"target_elevation_rad\":" + std::to_string(command.target_elevation_rad) +
        ",\"target_elevation_deg\":" + std::to_string(rad_to_deg(command.target_elevation_rad)) +
        ",\"target_bearing_rad\":" + std::to_string(command.target_bearing_rad) +
        ",\"target_bearing_deg\":" + std::to_string(rad_to_deg(command.target_bearing_rad)) +
        ",\"range_xy_m\":" + std::to_string(command.range_xy_m) +
        ",\"delta_z_m\":" + std::to_string(command.delta_z_m) +
        ",\"agent_id\":" + q(command.agent_id) +
        ",\"source_track_id\":" + q(command.source_track_id) +
        ",\"identity_id\":" + q(command.identity_id) +
        behavior_display_fields("arriving")};
}

void ObjectBehaviorMissionController::emit_camera_pointing(
    MissionTickOutput& output,
    const CameraPointingCommand& command) const {
    output.camera_pointing = command;
    output.events.push_back(camera_pointing_intent_event(command));
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

bool ObjectBehaviorMissionController::update_circle_orbit_progress(
    const BehaviorSpec& behavior,
    bool circling,
    double orbit_angle_rad) {
    if (behavior.type != BehaviorType::Circle || behavior.orbit_count <= 0.0) {
        return false;
    }

    if (!circling) {
        circle_orbit_tracking_ = false;
        return false;
    }

    if (!circle_orbit_tracking_) {
        circle_orbit_tracking_ = true;
        circle_previous_angle_rad_ = orbit_angle_rad;
        circle_completed_orbits_ = 0.0;
        return false;
    }

    double delta = orbit_angle_rad - circle_previous_angle_rad_;
    while (delta > kPi) {
        delta -= 2.0 * kPi;
    }
    while (delta < -kPi) {
        delta += 2.0 * kPi;
    }

    const double directed_delta = circle_direction_sign(behavior.direction) * delta;
    if (directed_delta > 0.0) {
        circle_completed_orbits_ += directed_delta / (2.0 * kPi);
    }

    circle_previous_angle_rad_ = orbit_angle_rad;
    return circle_completed_orbits_ >= behavior.orbit_count;
}

bool ObjectBehaviorMissionController::sequence_active() const {
    return config_.behavior_spec.behavior.type == BehaviorType::Sequence;
}

const BehaviorSpec& ObjectBehaviorMissionController::active_behavior() const {
    if (!sequence_active()) {
        return config_.behavior_spec.behavior;
    }
    const auto& steps = config_.behavior_spec.behavior.steps;
    if (steps.empty()) {
        return config_.behavior_spec.behavior;
    }
    const std::size_t index = std::min(sequence_step_index_, steps.size() - 1U);
    return steps[index];
}

bool ObjectBehaviorMissionController::active_behavior_is_last_sequence_step() const {
    if (!sequence_active()) {
        return true;
    }
    const auto& steps = config_.behavior_spec.behavior.steps;
    return steps.empty() || sequence_step_index_ + 1U >= steps.size();
}

ObjectBehaviorYawMode ObjectBehaviorMissionController::yaw_mode_for_behavior(const BehaviorSpec& behavior) const {
    if (behavior.yaw_mode.empty()) {
        return config_.yaw_mode;
    }
    return parse_yaw_mode(behavior.yaw_mode);
}

void ObjectBehaviorMissionController::reset_sequence_step(TimePoint now) {
    sequence_step_start_ = now;
    sequence_step_started_ = true;
    circle_in_orbit_mode_ = false;
    circle_orbit_tracking_ = false;
    circle_previous_angle_rad_ = 0.0;
    circle_completed_orbits_ = 0.0;
    last_behavior_display_detail_.clear();
}

void ObjectBehaviorMissionController::reset_behavior_run(TimePoint now) {
    behavior_start_ = now;
    target_selected_emitted_ = false;
    behavior_start_emitted_ = false;
    behavior_complete_emitted_ = false;
    behavior_tick_sample_emitted_ = false;
    execute_tick_count_ = 0;
    last_behavior_display_detail_.clear();
    previous_selection_.reset();
    circle_in_orbit_mode_ = false;
    circle_orbit_tracking_ = false;
    circle_previous_angle_rad_ = 0.0;
    circle_completed_orbits_ = 0.0;
    sequence_step_index_ = 0U;
    sequence_step_started_ = false;
    reset_sequence_step(now);
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
            if (auto camera_pointing = neutral_camera_pointing_command(
                    input.now,
                    config_.camera_pointing_prepare_mode)) {
                emit_camera_pointing(output, *camera_pointing);
            }
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
            if (auto camera_pointing = neutral_camera_pointing_command(
                    input.now,
                    config_.camera_pointing_takeoff_mode)) {
                emit_camera_pointing(output, *camera_pointing);
            }
            if (input.finish_requested) {
                state_ = height_m > kLandHeightM ? MissionLifecycleState::Land : MissionLifecycleState::Complete;
                state_start_ = input.now;
                output.status = height_m > kLandHeightM ? "finish_requested_land" : "finish_requested_complete";
            } else if (height_m >= config_.takeoff_height_m) {
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
        case MissionLifecycleState::ExecuteMission:
            tick_execute_mission(input, ego, height_m, output);
            break;
        case MissionLifecycleState::GoHome:
            tick_go_home(input, ego, height_m, output);
            break;
        case MissionLifecycleState::Land:
            if (home_initialized_) {
                if (auto camera_pointing = camera_pointing_command_to_point(
                        input.now,
                        ego,
                        home_pose_.position,
                        config_.camera_pointing_land_mode)) {
                    emit_camera_pointing(output, *camera_pointing);
                }
            }

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
            if (auto camera_pointing = neutral_camera_pointing_command(input.now, config_.camera_pointing_complete_mode)) {
                emit_camera_pointing(output, *camera_pointing);
            }

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

void ObjectBehaviorMissionController::tick_execute_mission(
    const MissionTickInput& input,
    const EgoState& ego,
    double height_m,
    MissionTickOutput& output) {
    ++execute_tick_count_;
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
            if (sequence_active()) {
                output.events.push_back(sequence_step_event(ControllerEventKind::SequenceStepStart, active_behavior(), sequence_step_index_, "sequence_start"));
            }
            output.events.push_back(behavior_event(ControllerEventKind::BehaviorStart, "target_selected"));
        }
        const bool duration_complete = completion_elapsed(input.now);
        bool orbit_count_complete = false;
        TargetSelection control_selection = selection;
        if (config_.zero_target_velocity) {
            control_selection.velocity_local = Vec3{0.0, 0.0, 0.0};
        }

        const BehaviorSpec& behavior = active_behavior();
        if (auto camera_pointing = camera_pointing_command_for_behavior(input.now, ego, control_selection, behavior)) {
            emit_camera_pointing(output, *camera_pointing);
        }

        FollowGeometry geometry;
        Vec3 raw_velocity{0.0, 0.0, 0.0};
        if (!input.finish_requested && !duration_complete) {
            if (behavior.type == BehaviorType::Circle) {
                geometry = circle_geometry(
                    ego,
                    control_selection,
                    behavior,
                    config_,
                    circle_in_orbit_mode_);
                const bool circling = geometry.circle_phase == "circling";
                if (circling) {
                    circle_in_orbit_mode_ = true;
                }
                raw_velocity = clamp_velocity(
                    geometry.desired_velocity,
                    behavior.max_speed_mps,
                    behavior.max_vertical_speed_mps);
                orbit_count_complete = update_circle_orbit_progress(behavior, circling, geometry.orbit_angle_rad);
            } else {
                raw_velocity = behavior_velocity(
                    ego,
                    control_selection,
                    behavior,
                    config_,
                    &geometry);
                orbit_count_complete = update_circle_orbit_progress(behavior, geometry.circle_phase == "circling", geometry.orbit_angle_rad);
            }
            geometry.circle_completed_orbits = circle_completed_orbits_;
            geometry.orbit_count_target = behavior.orbit_count;
        }

        if (!input.finish_requested && !duration_complete) {
            raw_velocity = apply_altitude_profile(raw_velocity, ego, behavior, seconds_between(sequence_step_start_, input.now), geometry);
        }

        const bool step_duration_complete =
            sequence_active() && behavior.duration_s > 0.0 &&
            elapsed_at_least(sequence_step_start_, input.now, behavior.duration_s);
        const bool approach_complete =
            sequence_active() && behavior.type == BehaviorType::Approach &&
            geometry.behavior_step_complete;
        const bool terminal_step =
            sequence_active() && (
                behavior.type == BehaviorType::GoHome ||
                behavior.type == BehaviorType::GoHomeLand ||
                behavior.type == BehaviorType::Land);
        const bool step_complete =
            sequence_active() && (step_duration_complete || orbit_count_complete || approach_complete || terminal_step);

        if (step_complete && !active_behavior_is_last_sequence_step()) {
            output.events.push_back(sequence_step_event(
                ControllerEventKind::SequenceStepComplete,
                behavior,
                sequence_step_index_,
                terminal_step ? "terminal_step" : (orbit_count_complete ? "orbit_count_elapsed" : (approach_complete ? "approach_standoff_reached" : "duration_elapsed"))));
            ++sequence_step_index_;
            reset_sequence_step(input.now);
            output.events.push_back(sequence_step_event(
                ControllerEventKind::SequenceStepStart,
                active_behavior(),
                sequence_step_index_,
                "previous_step_complete"));
            output.status = "object_behavior_sequence_step_complete";
            return;
        }

        const bool sequence_complete =
            step_complete && active_behavior_is_last_sequence_step();
        const ObjectBehaviorYawMode active_yaw_mode = yaw_mode_for_behavior(behavior);
        const std::string active_camera_pointing_mode = behavior.camera_pointing_mode.empty()
            ? "target"
            : behavior.camera_pointing_mode;
        if (input.finish_requested || duration_complete || orbit_count_complete || sequence_complete) {
            if (!behavior_complete_emitted_) {
                behavior_complete_emitted_ = true;
                output.events.push_back(behavior_event(
                    ControllerEventKind::BehaviorComplete,
                    input.finish_requested ? "finish_requested" :
                        (sequence_complete ? "sequence_complete" :
                            (orbit_count_complete ? "orbit_count_elapsed" : "duration_elapsed"))));
            }
            state_ = MissionLifecycleState::GoHome;
            state_start_ = input.now;
            output.status = input.finish_requested ? "object_behavior_finish_requested" : "object_behavior_complete";
        } else {
            const Vec3 velocity = apply_altitude_policy(
                raw_velocity,
                config_,
                behavior,
                height_m);
            output.command = command_from_behavior_velocity(
                input.now,
                velocity,
                ego,
                control_selection,
                config_.yaw_offset_rad + behavior.yaw_offset_rad,
                active_yaw_mode);
            const std::string behavior_detail =
                behavior_detail_for_tick(behavior, geometry);
            if (!behavior_tick_sample_emitted_ || behavior_detail != last_behavior_display_detail_) {
                behavior_tick_sample_emitted_ = true;
                last_behavior_display_detail_ = behavior_detail;
                output.events.push_back(behavior_tick_event(
                    config_.behavior_spec,
                    behavior,
                    control_selection,
                    velocity,
                    geometry,
                    sequence_active()
                        ? std::optional<std::size_t>{sequence_step_index_}
                        : std::nullopt,
                    sequence_active() ? config_.behavior_spec.behavior.steps.size() : 0U,
                    active_yaw_mode,
                    active_camera_pointing_mode));
            }
            if (config_.debug_every_n_ticks > 0 && execute_tick_count_ % config_.debug_every_n_ticks == 0) {
                output.events.push_back(behavior_debug_event(
                    execute_tick_count_,
                    config_.debug_level,
                    ego,
                    control_selection,
                    raw_velocity,
                    velocity,
                    *output.command,
                    geometry));
            }
            output.status = object_behavior_status(behavior, geometry);
        }
    } else if (input.finish_requested) {
        state_ = MissionLifecycleState::GoHome;
        state_start_ = input.now;
        output.status = "object_behavior_finish_requested_no_target";
    } else {
        output.command = command_from_velocity(input.now, Vec3{0.0, 0.0, 0.0});
        output.status = "object_behavior_waiting_for_target_" + to_string(selection.status);
    }
}

void ObjectBehaviorMissionController::tick_go_home(
    const MissionTickInput& input,
    const EgoState& ego,
    double height_m,
    MissionTickOutput& output) {
    BehaviorSpec transit_behavior = config_.behavior_spec.behavior;
    transit_behavior.max_vertical_speed_mps = std::max(0.1, transit_behavior.max_vertical_speed_mps);

    if (home_initialized_) {
        if (auto camera_pointing = camera_pointing_command_to_point(
                input.now,
                ego,
                home_pose_.position,
                config_.camera_pointing_go_home_mode)) {
            emit_camera_pointing(output, *camera_pointing);
        }
    } else if (auto camera_pointing = neutral_camera_pointing_command(
                   input.now,
                   config_.camera_pointing_complete_mode)) {
        emit_camera_pointing(output, *camera_pointing);
    }

    Vec3 raw_velocity = go_home_velocity(ego);
    Vec3 velocity = enforce_takeoff_height_floor(
        raw_velocity,
        height_m,
        config_.takeoff_height_m,
        transit_behavior.max_vertical_speed_mps);

    if (norm_xy(velocity) <= 0.0 &&
        height_m >= config_.takeoff_height_m) {
        state_ = MissionLifecycleState::Land;
        state_start_ = input.now;
        output.status = aborting_ ? "abort_recovery_home_reached" : "home_reached";
    } else {
        output.command = command_from_velocity(input.now, velocity, config_.yaw_offset_rad);
        output.status = aborting_ ? "abort_recovery_go_home" : "go_home";
    }
}

}  // namespace dedalus
