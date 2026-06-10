#include "dedalus/behavior/object_behavior_mission_controller.hpp"

#include "dedalus/behavior/behavior_osd.hpp"
#include "dedalus/behavior/follow_geometry_policy.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace dedalus {
namespace {

constexpr double kLandHeightM = 0.25;
constexpr double kTakeoffVelocityAssistHeightM = 0.5;

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


}  // namespace

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
