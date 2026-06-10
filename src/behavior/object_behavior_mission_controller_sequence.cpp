#include "dedalus/behavior/object_behavior_mission_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace dedalus {
namespace {

constexpr double kPi = 3.14159265358979323846;

double circle_direction_sign(CircleDirection direction) {
    return direction == CircleDirection::Clockwise ? -1.0 : 1.0;
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

}  // namespace

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
