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
constexpr double kMinArrivedDistanceM = 0.5;

double seconds_between(TimePoint start, TimePoint end) {
    return static_cast<double>(end.timestamp_ns - start.timestamp_ns) / 1'000'000'000.0;
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

}  // namespace dedalus
