#include "dedalus/behavior/mission_runtime.hpp"

#include "dedalus/core/json_utils.hpp"

#include <iostream>
#include <string>
#include <sstream>
#include <cmath>

namespace dedalus {
namespace {

bool output_is_terminal_settled(const MissionTickOutput& output) {
    return (output.state == MissionLifecycleState::Complete && output.status == "complete") ||
           (output.state == MissionLifecycleState::Abort && output.status == "abort");
}

std::string display_primary_for_state(MissionLifecycleState state) {
    switch (state) {
        case MissionLifecycleState::Prepare:
            return "Arm";
        case MissionLifecycleState::Takeoff:
            return "Takeoff";
        case MissionLifecycleState::ExecuteMission:
            return "Mission";
        case MissionLifecycleState::GoHome:
            return "GoHome";
        case MissionLifecycleState::Land:
            return "Land";
        case MissionLifecycleState::Complete:
            return "Settled";
        case MissionLifecycleState::Abort:
            return "Failed";
        case MissionLifecycleState::Idle:
        default:
            return "Unknown";
    }
}

std::string display_primary_for_command(FlightCommandKind command) {
    switch (command) {
        case FlightCommandKind::Arm:
            return "Arm";
        case FlightCommandKind::Takeoff:
            return "Takeoff";
        case FlightCommandKind::Land:
            return "Land";
        case FlightCommandKind::Disarm:
            return "Disarm";
        case FlightCommandKind::Velocity:
            return "Mission";
        default:
            return "Unknown";
    }
}

std::string compact_display_detail(const std::string& status) {
    if (status.empty()) {
        return "-";
    }
    if (status.find("failed") != std::string::npos ||
        status.find("error") != std::string::npos ||
        status.find("exception") != std::string::npos ||
        status.find("timeout") != std::string::npos ||
        status.find("abort") != std::string::npos) {
        return "failed";
    }
    if (status.find("arm") != std::string::npos) {
        return "arming";
    }
    if (status.find("takeoff") != std::string::npos || status.find("climb") != std::string::npos) {
        return "climbing";
    }
    if (status.find("land") != std::string::npos) {
        return "landing";
    }
    if (status.find("home") != std::string::npos) {
        return "returning";
    }
    if (status.find("complete") != std::string::npos) {
        return "done";
    }
    if (status.find("object_behavior_arriving") != std::string::npos) {
        return "arriving";
    }
    if (status.find("object_behavior_following") != std::string::npos) {
        return "following";
    }
    if (status.find("object_behavior_positioned") != std::string::npos) {
        return "positioned";
    }
    if (status.find("object_behavior_circling") != std::string::npos) {
        return "circling";
    }
    if (status == "ok") {
        return "ok";
    }
    return status.size() > 12 ? status.substr(0, 11) + "~" : status;
}

std::string display_fields(const std::string& primary, const std::string& detail) {
    return ",\"display_state\":" + q(primary) + ",\"display_detail\":" + q(detail);
}

std::string trajectory_safety_event_fields(const TrajectorySafetyResult& safety) {
    std::ostringstream out;

    const auto append_float_or_null = [&out](const float value) {
        if (std::isfinite(value)) {
            out << value;
        } else {
            out << "null";
        }
    };

    out << "\"event\":\"trajectory_safety\""
        << ",\"clear\":" << (safety.clear ? "true" : "false")
        << ",\"blocked\":" << (safety.blocked ? "true" : "false")
        << ",\"has_valid_query\":" << (safety.has_valid_query ? "true" : "false")
        << ",\"sample_count\":" << safety.sample_count
        << ",\"blocked_sample_count\":" << safety.blocked_sample_count
        << ",\"first_blocked_sample_index\":" << safety.first_blocked_sample_index
        << ",\"minimum_clearance_m\":";
    append_float_or_null(safety.minimum_clearance_m);
    out << ",\"nearest_obstacle_m\":";
    append_float_or_null(safety.nearest_obstacle_m);

    return out.str();
}

}  // namespace

bool MissionRuntime::tick_once() {
    const auto snapshot = snapshots_->latest();
    if (!snapshot) {
        if (config_.verbosity >= 3 && tick_count_ == 0U) {
            std::cerr << "dedalus_mission: waiting for first WorldSnapshot\n";
        }
        return false;
    }

    // Stale-snapshot detection: same frame timestamp for N consecutive ticks
    // means the frame pipeline has stalled (e.g. slow inference).  Emit a
    // throttled warning; do NOT abort — continue with best-effort stale data.
    const std::uint64_t cur_ts = snapshot->timestamp.timestamp_ns;
    if (cur_ts > 0U && cur_ts == last_snapshot_ts_ns_) {
        ++consecutive_stale_ticks_;
        if (config_.stale_snapshot_warn_ticks > 0 &&
            consecutive_stale_ticks_ >= config_.stale_snapshot_warn_ticks &&
            consecutive_stale_ticks_ % config_.stale_snapshot_warn_ticks == 0) {
            write_event(
                "\"event\":\"snapshot_stale\""
                ",\"tick\":" + std::to_string(tick_count_) +
                ",\"stale_ticks\":" + std::to_string(consecutive_stale_ticks_) +
                ",\"snapshot_ts_ns\":" + std::to_string(cur_ts) +
                ",\"state\":" + q(to_string(last_state_)));
            std::cerr << "dedalus_mission: WARN snapshot_stale"
                      << " stale_ticks=" << consecutive_stale_ticks_
                      << " tick=" << tick_count_
                      << " state=" << to_string(last_state_) << "\n";
        }
    } else {
        if (consecutive_stale_ticks_ > 0 && config_.verbosity >= 1) {
            std::cerr << "dedalus_mission: snapshot_fresh after "
                      << consecutive_stale_ticks_ << " stale tick(s)\n";
        }
        consecutive_stale_ticks_ = 0;
        last_snapshot_ts_ns_ = cur_ts;
    }

    MissionTickInput input;
    input.now = now_timepoint();
    input.snapshot = *snapshot;
    flight_control_tracker_.apply_to_snapshot(input.snapshot);
    input.last_command_result = last_command_result_;
    input.finish_requested = finish_requested_.load();
    if (input.snapshot.timestamp.timestamp_ns > 0) {
        input.now = input.snapshot.timestamp;
    }

    const auto output = controller_->tick(input);
    const auto previous_state = last_state_;
    last_state_ = output.state;
    if (output_is_terminal_settled(output)) {
        terminal_settled_.store(true);
    }
    ++tick_count_;

    const bool state_changed = previous_state != output.state;
    const bool detailed_tick = config_.verbosity >= 3 && (tick_count_ <= 3U || output.command.has_value());
    if (state_changed) {
        write_event(
            "\"event\":\"state_transition\",\"tick\":" + std::to_string(tick_count_) +
            ",\"from\":" + q(to_string(previous_state)) +
            ",\"to\":" + q(to_string(output.state)) +
            ",\"status\":" + q(output.status) +
            ",\"timestamp_ns\":" + std::to_string(input.now.timestamp_ns) +
            ",\"ego_height_m\":" + std::to_string(input.snapshot.ego.height_m) +
            ",\"finish_requested\":" + (input.finish_requested ? std::string{"true"} : std::string{"false"}) +
            display_fields(display_primary_for_state(output.state), compact_display_detail(output.status)));
    }
    for (const auto& event : output.events) {
        write_event(
            "\"event\":" + q(to_string(event.kind)) +
            ",\"tick\":" + std::to_string(tick_count_) +
            ",\"state\":" + q(to_string(output.state)) +
            event.json_fields);
    }
    if (input.snapshot.has_trajectory_safety &&
        (state_changed || detailed_tick || input.snapshot.trajectory_safety.blocked)) {
        write_event(
            trajectory_safety_event_fields(input.snapshot.trajectory_safety) +
            ",\"tick\":" + std::to_string(tick_count_) +
            ",\"state\":" + q(to_string(output.state)));
    }

    if (state_changed || detailed_tick) {
        std::cerr << "dedalus_mission: tick=" << tick_count_
                  << " state=" << to_string(output.state)
                  << " status=" << output.status
                  << " ego_height_m=" << input.snapshot.ego.height_m
                  << " finish_requested=" << (input.finish_requested ? "true" : "false")
                  << " command=" << (output.command.has_value() ? "yes" : "no");
        if (config_.verbosity >= 2) {
            std::cerr << " flight_control=" << static_cast<int>(input.snapshot.flight_control.arm_state);
        }
        if (config_.verbosity >= 3 && input.last_command_result.has_value()) {
            std::cerr << " last_result=" << to_string(input.last_command_result->kind)
                      << ":" << (input.last_command_result->success ? "ok" : "failed");
        }
        std::cerr << "\n";
    }

    if (output.camera_pointing.has_value()) {
        dispatch_camera_pointing(output, input);
    }

    if (output.command.has_value()) {
        dispatch_command(output, input);
    }

    return true;
}

void MissionRuntime::dispatch_camera_pointing(
    const MissionTickOutput& output,
    const MissionTickInput& /*input*/) {
    const auto& camera_pointing = *output.camera_pointing;
    write_event(
        "\"event\":\"camera_pointing_dispatch\",\"tick\":" + std::to_string(tick_count_) +
        ",\"state\":" + q(to_string(output.state)) +
        ",\"timestamp_ns\":" + std::to_string(camera_pointing.timestamp.timestamp_ns) +
        ",\"camera_pointing_mode\":" + q(camera_pointing.mode) +
        ",\"pitch_valid\":" + (camera_pointing.pitch_valid ? std::string{"true"} : std::string{"false"}) +
        ",\"pitch_rad\":" + std::to_string(camera_pointing.pitch_rad) +
        ",\"pitch_deg\":" + std::to_string(camera_pointing.pitch_rad * 180.0 / 3.14159265358979323846) +
        ",\"pitch_clamped\":" + (camera_pointing.pitch_clamped ? std::string{"true"} : std::string{"false"}) +
        ",\"yaw_valid\":" + (camera_pointing.yaw_valid ? std::string{"true"} : std::string{"false"}) +
        ",\"yaw_rad\":" + std::to_string(camera_pointing.yaw_rad) +
        ",\"yaw_deg\":" + std::to_string(camera_pointing.yaw_rad * 180.0 / 3.14159265358979323846) +
        ",\"source_track_id\":" + q(camera_pointing.source_track_id) +
        ",\"agent_id\":" + q(camera_pointing.agent_id) +
        ",\"identity_id\":" + q(camera_pointing.identity_id) +
        display_fields("Camera", camera_pointing.mode.empty() ? "pointing" : camera_pointing.mode));
    if (config_.verbosity >= 2) {
        std::cerr << "dedalus_mission: send_camera_pointing mode=" << camera_pointing.mode
                  << " pitch_deg=" << camera_pointing.pitch_rad * 180.0 / 3.14159265358979323846
                  << " cameras=" << camera_pointing.cameras.size()
                  << "\n";
    }
    if (camera_pointing_handoff_) {
        camera_pointing_handoff_(camera_pointing);
    }
    try {
        last_camera_pointing_result_ = camera_pointing_sink_->send(camera_pointing);
    } catch (const std::exception& ex) {
        last_camera_pointing_result_ = CameraPointingResult{false, ex.what()};
        write_event(
            "\"event\":\"camera_pointing_exception\",\"tick\":" + std::to_string(tick_count_) +
            ",\"state\":" + q(to_string(output.state)) +
            ",\"camera_pointing_mode\":" + q(camera_pointing.mode) +
            ",\"error\":" + q(ex.what()) +
            display_fields("Failed", "camera"));
        std::cerr << "dedalus_mission: camera_pointing_exception mode=" << camera_pointing.mode
                  << " status=" << ex.what() << "\n";
    }
    if (!last_camera_pointing_result_.has_value()) {
        last_camera_pointing_result_ = CameraPointingResult{false, "camera_pointing_sink_missing_result"};
    }
    write_event(
        "\"event\":\"camera_pointing_result\",\"tick\":" + std::to_string(tick_count_) +
        ",\"state\":" + q(to_string(output.state)) +
        ",\"camera_pointing_mode\":" + q(camera_pointing.mode) +
        ",\"success\":" + (last_camera_pointing_result_->success ? std::string{"true"} : std::string{"false"}) +
        ",\"status\":" + q(last_camera_pointing_result_->status) +
        display_fields(
            last_camera_pointing_result_->success ? "Camera" : "Failed",
            last_camera_pointing_result_->success ? camera_pointing.mode : "camera"));
    if (config_.verbosity >= 2) {
        std::cerr << "dedalus_mission: camera_pointing_result success="
                  << (last_camera_pointing_result_->success ? "true" : "false")
                  << " status=" << last_camera_pointing_result_->status;
        if (last_camera_pointing_result_->status.empty() || last_camera_pointing_result_->status.back() != '\n') {
            std::cerr << "\n";
        }
    }
}

void MissionRuntime::dispatch_command(
    const MissionTickOutput& output,
    const MissionTickInput& input) {
    const auto& command = *output.command;
    write_event(
        "\"event\":\"command_dispatch\",\"tick\":" + std::to_string(tick_count_) +
        ",\"state\":" + q(to_string(output.state)) +
        ",\"command\":" + q(to_string(command.kind)) +
        ",\"timestamp_ns\":" + std::to_string(command.timestamp.timestamp_ns) +
        ",\"vx\":" + std::to_string(command.velocity_local_mps.x) +
        ",\"vy\":" + std::to_string(command.velocity_local_mps.y) +
        ",\"vz\":" + std::to_string(command.velocity_local_mps.z) +
        ",\"yaw_rate\":" + std::to_string(command.yaw_rate_radps) +
        display_fields(
            display_primary_for_command(command.kind),
            command.kind == FlightCommandKind::Velocity ? compact_display_detail(output.status) : "send"));
    if (config_.verbosity >= 2) {
        const auto& v = output.command->velocity_local_mps;
        std::cerr << "dedalus_mission: send_command kind=" << to_string(output.command->kind)
                  << " vx=" << v.x
                  << " vy=" << v.y
                  << " vz=" << v.z
                  << " yaw_rate=" << output.command->yaw_rate_radps
                  << "\n";
    }
    try {
        last_command_result_ = sink_->send(*output.command);
        if (last_command_result_->success) {
            flight_control_tracker_.on_command_dispatched(
                output.command->kind,
                input.now,
                last_command_result_->status);
        } else {
            flight_control_tracker_.on_command_failed(
                output.command->kind,
                input.now,
                last_command_result_->status);
        }
    } catch (const std::exception& ex) {
        last_command_result_ = FlightCommandResult{output.command->kind, false, ex.what()};
        flight_control_tracker_.on_command_failed(output.command->kind, input.now, ex.what());
        write_event(
            "\"event\":\"command_exception\",\"tick\":" + std::to_string(tick_count_) +
            ",\"state\":" + q(to_string(output.state)) +
            ",\"command\":" + q(to_string(output.command->kind)) +
            ",\"error\":" + q(ex.what()) +
            display_fields("Failed", display_primary_for_command(output.command->kind)));
        std::cerr << "dedalus_mission: command_exception kind=" << to_string(output.command->kind)
                  << " status=" << ex.what() << "\n";
    }
    write_event(
        "\"event\":\"command_result\",\"tick\":" + std::to_string(tick_count_) +
        ",\"state\":" + q(to_string(output.state)) +
        ",\"command\":" + q(to_string(last_command_result_->kind)) +
        ",\"success\":" + (last_command_result_->success ? std::string{"true"} : std::string{"false"}) +
        ",\"status\":" + q(last_command_result_->status) +
        display_fields(
            last_command_result_->success ? display_primary_for_command(last_command_result_->kind) : "Failed",
            last_command_result_->success
                ? (last_command_result_->kind == FlightCommandKind::Velocity ? compact_display_detail(output.status) : "ok")
                : display_primary_for_command(last_command_result_->kind)));
    if (config_.verbosity >= 2) {
        std::cerr << "dedalus_mission: command_result kind=" << to_string(last_command_result_->kind)
                  << " success=" << (last_command_result_->success ? "true" : "false")
                  << " status=" << last_command_result_->status;
        if (last_command_result_->status.empty() || last_command_result_->status.back() != '\n') {
            std::cerr << "\n";
        }
    }
}

}  // namespace dedalus
