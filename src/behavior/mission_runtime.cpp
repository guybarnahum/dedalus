#include "dedalus/behavior/mission_runtime.hpp"

#include "dedalus/core/json_utils.hpp"

#include <chrono>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace dedalus {
namespace {

using SteadyClock = std::chrono::steady_clock;

std::uint64_t elapsed_us(const SteadyClock::time_point start) {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(SteadyClock::now() - start).count());
}

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

class NullCameraPointingSink final : public CameraPointingSink {
public:
    CameraPointingResult send(const CameraPointingCommand& command) override {
        if (!command.pitch_valid) {
            return CameraPointingResult{false, "camera_pointing_invalid"};
        }
        return CameraPointingResult{true, "camera_pointing_ignored"};
    }
};

}  // namespace

MissionRuntime::MissionRuntime(
    MissionRuntimeConfig config,
    std::shared_ptr<LatestWorldSnapshot> snapshots,
    std::unique_ptr<MissionController> controller,
    std::unique_ptr<FlightCommandSink> sink,
    std::shared_ptr<MissionEventPublisher> mission_event_publisher,
    std::unique_ptr<CameraPointingSink> camera_pointing_sink)
    : config_(std::move(config)),
      snapshots_(std::move(snapshots)),
      controller_(std::move(controller)),
      sink_(std::move(sink)),
      camera_pointing_sink_(std::move(camera_pointing_sink)),
      mission_event_publisher_(std::move(mission_event_publisher)) {
    if (config_.tick_hz <= 0.0) {
        throw std::invalid_argument("MissionRuntime requires positive tick_hz");
    }
    if (!snapshots_) {
        throw std::invalid_argument("MissionRuntime requires a latest snapshot handoff");
    }
    if (!controller_) {
        throw std::invalid_argument("MissionRuntime requires a mission controller");
    }
    if (!sink_) {
        throw std::invalid_argument("MissionRuntime requires a flight command sink");
    }
    if (!camera_pointing_sink_) {
        camera_pointing_sink_ = std::make_unique<NullCameraPointingSink>();
    }
    if (!config_.event_log_path.empty()) {
        event_log_.open(config_.event_log_path, std::ios::out | std::ios::trunc);
        if (!event_log_) {
            throw std::runtime_error("failed to open mission event log: " + config_.event_log_path);
        }
    }
}

MissionRuntime::~MissionRuntime() {
    stop();
}

void MissionRuntime::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
        return;
    }
    write_event("\"event\":\"runtime_start\",\"tick_hz\":" + std::to_string(config_.tick_hz) +
                display_fields("Unknown", "start"));
    if (config_.verbosity >= 1) {
        std::cerr << "dedalus_mission: starting async loop @ " << config_.tick_hz << " Hz\n";
    }
    thread_ = std::thread([this]() { loop(); });
}

void MissionRuntime::stop() {
    const bool was_running = running_.exchange(false);
    if (thread_.joinable()) {
        thread_.join();
    }
    if (was_running || tick_count_ > 0U) {
        write_event(
            "\"event\":\"runtime_stop\",\"tick_count\":" + std::to_string(tick_count_) +
            ",\"state\":" + q(to_string(last_state_)) +
            ",\"terminal_settled\":" + (terminal_settled_.load() ? std::string{"true"} : std::string{"false"}) +
            display_fields(terminal_settled_.load() ? "Settled" : "Failed", terminal_settled_.load() ? "done" : "stopped"));
    }
    if (config_.verbosity >= 1) {
        std::cerr << "dedalus_mission: stopped after " << tick_count_ << " tick(s)\n";
    }
}

void MissionRuntime::request_finish() {
    finish_requested_.store(true);
    write_event(
        "\"event\":\"finish_requested\",\"tick\":" + std::to_string(tick_count_) +
        ",\"state\":" + q(to_string(last_state_)) +
        display_fields(display_primary_for_state(last_state_), "finish"));
    if (config_.verbosity >= 1) {
        std::cerr << "dedalus_mission: finish requested\n";
    }
}

bool MissionRuntime::tick_once() {
    const auto snapshot = snapshots_->latest();
    if (!snapshot) {
        if (config_.verbosity >= 3 && tick_count_ == 0U) {
            std::cerr << "dedalus_mission: waiting for first WorldSnapshot\n";
        }
        return false;
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

bool MissionRuntime::running() const {
    return running_.load();
}

bool MissionRuntime::finish_requested() const {
    return finish_requested_.load();
}

bool MissionRuntime::terminal_settled() const {
    return terminal_settled_.load();
}

std::size_t MissionRuntime::tick_count() const {
    return tick_count_;
}

MissionLifecycleState MissionRuntime::last_state() const {
    return last_state_;
}

MissionRuntimeStats MissionRuntime::stats() const {
    return MissionRuntimeStats{
        .events_written = events_written_.load(),
        .event_publish_total_us = event_publish_total_us_.load(),
        .event_log_write_total_us = event_log_write_total_us_.load(),
        .event_log_flush_total_us = event_log_flush_total_us_.load()};
}

void MissionRuntime::loop() {
    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / config_.tick_hz));
    // Track a fixed deadline that advances by exactly one period per tick.
    // Using sleep_until instead of sleep_for(period - elapsed) prevents
    // accumulated drift: any OS wakeup overrun from the previous sleep is
    // automatically absorbed into the next sleep rather than being lost.
    auto deadline = std::chrono::steady_clock::now();
    while (running_.load()) {
        deadline += period;
        try {
            (void)tick_once();
        } catch (const std::exception& exc) {
            loop_exception_ = std::current_exception();
            write_event("\"event\":\"runtime_error\",\"tick\":" + std::to_string(tick_count_) +
                        ",\"error\":" + q(exc.what()));
            running_.store(false);
            break;
        } catch (...) {
            loop_exception_ = std::current_exception();
            write_event("\"event\":\"runtime_error\",\"tick\":" + std::to_string(tick_count_) +
                        ",\"error\":\"unknown exception\"");
            running_.store(false);
            break;
        }
        std::this_thread::sleep_until(deadline);
    }
}

void MissionRuntime::rethrow_if_exception() const {
    if (loop_exception_) {
        std::rethrow_exception(loop_exception_);
    }
}

void MissionRuntime::write_event(std::string json_fields) {
    const auto event_json = "{" + json_fields + "}";
    if (mission_event_publisher_) {
        const auto publish_start = SteadyClock::now();
        mission_event_publisher_->publish(MissionEvent{.timestamp = now_timepoint(), .json = event_json});
        event_publish_total_us_.fetch_add(elapsed_us(publish_start));
    }
    if (event_log_) {
        std::lock_guard<std::mutex> lock{event_log_mutex_};
        const auto write_start = SteadyClock::now();
        event_log_ << event_json << "\n";
        event_log_write_total_us_.fetch_add(elapsed_us(write_start));
        const auto flush_start = SteadyClock::now();
        event_log_.flush();
        event_log_flush_total_us_.fetch_add(elapsed_us(flush_start));
    }
    events_written_.fetch_add(1U);
}

TimePoint MissionRuntime::now_timepoint() const {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return TimePoint{std::chrono::duration_cast<std::chrono::nanoseconds>(now).count()};
}

}  // namespace dedalus
