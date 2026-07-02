#include "dedalus/behavior/object_behavior_mission_controller.hpp"

#include "dedalus/behavior/behavior_osd.hpp"
#include "dedalus/behavior/follow_geometry_policy.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <string>
#include <vector>

namespace dedalus {
namespace {

double seconds_between(TimePoint start, TimePoint end) {
    return static_cast<double>(end.timestamp_ns - start.timestamp_ns) / 1'000'000'000.0;
}

bool elapsed_at_least(TimePoint start, TimePoint end, double seconds) {
    return seconds_between(start, end) >= seconds;
}

double norm_xy(const Vec3& value) {
    return std::sqrt(value.x * value.x + value.y * value.y);
}

}  // namespace

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

    // Compute XY distance to home first — needed for both the frame-rate-safe
    // speed cap and the per-tick diagnostic event.
    const double dx = home_initialized_ ? home_pose_.position.x - ego.local_T_body.position.x : 0.0;
    const double dy = home_initialized_ ? home_pose_.position.y - ego.local_T_body.position.y : 0.0;
    const double dist_xy_m = std::sqrt(dx * dx + dy * dy);

    // Frame-rate-adaptive speed cap.
    // Track when the ego snapshot changes to estimate the inter-frame interval.
    // This makes GoHome convergence self-correcting: the commanded speed is capped
    // so the drone cannot overshoot past kMinArrivedDistanceM in a single frame,
    // regardless of how slow (or fast) the ego provider is running.
    if (ego.timestamp.timestamp_ns > 0 &&
        last_ego_ts_go_home_.timestamp_ns > 0 &&
        ego.timestamp.timestamp_ns != last_ego_ts_go_home_.timestamp_ns) {
        const double observed_s = static_cast<double>(
            ego.timestamp.timestamp_ns - last_ego_ts_go_home_.timestamp_ns) / 1e9;
        if (observed_s > 0.0 && observed_s < 30.0) {  // sanity-clamp to [0, 30]s
            go_home_estimated_frame_interval_s_ = (go_home_estimated_frame_interval_s_ > 0.0)
                ? 0.3 * observed_s + 0.7 * go_home_estimated_frame_interval_s_  // EMA α=0.3
                : observed_s;
        }
    }
    if (ego.timestamp.timestamp_ns > 0) {
        last_ego_ts_go_home_ = ego.timestamp;
    }

    // Cap speed so we cover at most (dist − ½·arrival_threshold) per frame → guaranteed
    // single-frame convergence at any fps. Fall back to config speed before first estimate.
    constexpr double kMinGoHomeSpeedMps = 0.05;
    constexpr double kArrivalThresholdM = 0.5;  // mirrors kMinArrivedDistanceM
    double safe_speed = config_.go_home_velocity_mps;
    if (go_home_estimated_frame_interval_s_ > 0.0 && dist_xy_m > kArrivalThresholdM) {
        const double frame_cap = (dist_xy_m - 0.5 * kArrivalThresholdM) / go_home_estimated_frame_interval_s_;
        safe_speed = std::min(safe_speed, std::max(kMinGoHomeSpeedMps, frame_cap));
    }

    Vec3 raw_velocity = go_home_velocity(ego, safe_speed);

    // Per-tick convergence sample — visible in event stream for post-run diagnostics.
    output.events.push_back(ControllerEvent{
        ControllerEventKind::BehaviorTickSample,
        ",\"dist_to_home_xy_m\":" + std::to_string(dist_xy_m) +
        ",\"height_m\":" + std::to_string(height_m) +
        ",\"home_initialized\":" + (home_initialized_ ? "true" : "false") +
        ",\"go_home_safe_speed_mps\":" + std::to_string(safe_speed) +
        ",\"go_home_frame_interval_s\":" + std::to_string(go_home_estimated_frame_interval_s_)
    });
    if (std::getenv("DEDALUS_DEBUG_EGO")) {
        std::fprintf(stderr,
            "[GoHomeDebug] ego=(%.3f,%.3f) home=(%.3f,%.3f) dist=%.3f h=%.2f"
            " speed=%.3f interval=%.3f\n",
            ego.local_T_body.position.x, ego.local_T_body.position.y,
            home_initialized_ ? home_pose_.position.x : 0.0,
            home_initialized_ ? home_pose_.position.y : 0.0,
            dist_xy_m, height_m, safe_speed, go_home_estimated_frame_interval_s_);
    }

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
