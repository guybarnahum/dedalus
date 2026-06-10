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
    std::unique_ptr<CameraPointingSink> camera_pointing_sink,
    CameraPointingHandoff camera_pointing_handoff)
    : config_(std::move(config)),
      snapshots_(std::move(snapshots)),
      controller_(std::move(controller)),
      sink_(std::move(sink)),
      camera_pointing_sink_(std::move(camera_pointing_sink)),
      camera_pointing_handoff_(std::move(camera_pointing_handoff)),
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
