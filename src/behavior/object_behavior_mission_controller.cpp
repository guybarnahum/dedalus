#include "dedalus/behavior/object_behavior_mission_controller.hpp"

#include "dedalus/behavior/behavior_osd.hpp"
#include "dedalus/behavior/follow_geometry_policy.hpp"
#include "dedalus/core/json_utils.hpp"

#include <cmath>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace dedalus {
namespace {

constexpr double kLandHeightM = 0.25;
constexpr double kMinArrivedDistanceM = 0.5;

double seconds_between(TimePoint start, TimePoint end) {
    return static_cast<double>(end.timestamp_ns - start.timestamp_ns) / 1'000'000'000.0;
}

double norm_xy(const Vec3& value) {
    return std::sqrt(value.x * value.x + value.y * value.y);
}

// Pure geometry: fly toward `to` at exactly `speed_mps`, stopping at kMinArrivedDistanceM.
// Callers are responsible for passing a frame-rate-appropriate speed.
Vec3 velocity_toward_xy(const Vec3& from, const Vec3& to, double speed_mps) {
    const Vec3 delta{to.x - from.x, to.y - from.y, 0.0};
    const double distance = norm_xy(delta);
    if (distance <= kMinArrivedDistanceM || speed_mps <= 0.0) {
        return Vec3{0.0, 0.0, 0.0};
    }
    return Vec3{delta.x / distance * speed_mps, delta.y / distance * speed_mps, 0.0};
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

Vec3 ObjectBehaviorMissionController::go_home_velocity(const EgoState& ego, double speed_mps) const {
    if (!home_initialized_) {
        return Vec3{0.0, 0.0, 0.0};
    }
    return velocity_toward_xy(ego.local_T_body.position, home_pose_.position, speed_mps);
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

}  // namespace dedalus
