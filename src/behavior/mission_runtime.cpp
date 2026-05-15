#include "dedalus/behavior/mission_runtime.hpp"

#include <iostream>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace dedalus {
namespace {

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

}  // namespace

MissionRuntime::MissionRuntime(
    MissionRuntimeConfig config,
    std::shared_ptr<LatestWorldSnapshot> snapshots,
    std::unique_ptr<MissionController> controller,
    std::unique_ptr<FlightCommandSink> sink)
    : config_(std::move(config)),
      snapshots_(std::move(snapshots)),
      controller_(std::move(controller)),
      sink_(std::move(sink)) {
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
    write_event("\"event\":\"runtime_start\",\"tick_hz\":" + std::to_string(config_.tick_hz));
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
            ",\"state\":" + q(to_string(last_state_)));
    }
    if (config_.verbosity >= 1) {
        std::cerr << "dedalus_mission: stopped after " << tick_count_ << " tick(s)\n";
    }
}

void MissionRuntime::request_finish() {
    finish_requested_.store(true);
    write_event(
        "\"event\":\"finish_requested\",\"tick\":" + std::to_string(tick_count_) +
        ",\"state\":" + q(to_string(last_state_)));
    if (config_.verbosity >= 1) {
        std::cerr << "dedalus_mission: finish requested\n";
    }
}

bool MissionRuntime::tick_once() {
    const auto snapshot = snapshots_->latest();
    if (!snapshot.has_value()) {
        if (config_.verbosity >= 3 && tick_count_ == 0U) {
            std::cerr << "dedalus_mission: waiting for first WorldSnapshot\n";
        }
        return false;
    }

    MissionTickInput input;
    input.now = now_timepoint();
    input.snapshot = *snapshot;
    input.last_command_result = last_command_result_;
    input.finish_requested = finish_requested_.load();
    if (input.snapshot.timestamp.timestamp_ns > 0) {
        input.now = input.snapshot.timestamp;
    }

    const auto output = controller_->tick(input);
    const auto previous_state = last_state_;
    last_state_ = output.state;
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
            ",\"finish_requested\":" + (input.finish_requested ? std::string{"true"} : std::string{"false"}));
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

    if (output.command.has_value()) {
        const auto& command = *output.command;
        write_event(
            "\"event\":\"command_dispatch\",\"tick\":" + std::to_string(tick_count_) +
            ",\"state\":" + q(to_string(output.state)) +
            ",\"command\":" + q(to_string(command.kind)) +
            ",\"timestamp_ns\":" + std::to_string(command.timestamp.timestamp_ns) +
            ",\"vx\":" + std::to_string(command.velocity_local_mps.x) +
            ",\"vy\":" + std::to_string(command.velocity_local_mps.y) +
            ",\"vz\":" + std::to_string(command.velocity_local_mps.z) +
            ",\"yaw_rate\":" + std::to_string(command.yaw_rate_radps));
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
                snapshots_->mark_command_dispatched(
                    output.command->kind,
                    input.now,
                    last_command_result_->status);
            } else {
                snapshots_->mark_command_failed(
                    output.command->kind,
                    input.now,
                    last_command_result_->status);
            }
        } catch (const std::exception& ex) {
            last_command_result_ = FlightCommandResult{output.command->kind, false, ex.what()};
            snapshots_->mark_command_failed(output.command->kind, input.now, ex.what());
            write_event(
                "\"event\":\"command_exception\",\"tick\":" + std::to_string(tick_count_) +
                ",\"state\":" + q(to_string(output.state)) +
                ",\"command\":" + q(to_string(output.command->kind)) +
                ",\"error\":" + q(ex.what()));
            std::cerr << "dedalus_mission: command_exception kind=" << to_string(output.command->kind)
                      << " status=" << ex.what() << "\n";
        }
        write_event(
            "\"event\":\"command_result\",\"tick\":" + std::to_string(tick_count_) +
            ",\"state\":" + q(to_string(output.state)) +
            ",\"command\":" + q(to_string(last_command_result_->kind)) +
            ",\"success\":" + (last_command_result_->success ? std::string{"true"} : std::string{"false"}) +
            ",\"status\":" + q(last_command_result_->status));
        if (config_.verbosity >= 2) {
            std::cerr << "dedalus_mission: command_result kind=" << to_string(last_command_result_->kind)
                      << " success=" << (last_command_result_->success ? "true" : "false")
                      << " status=" << last_command_result_->status;
            if (last_command_result_->status.empty() || last_command_result_->status.back() != '\n') {
                std::cerr << "\n";
            }
        }
    }

    return true;
}

bool MissionRuntime::running() const {
    return running_.load();
}

bool MissionRuntime::finish_requested() const {
    return finish_requested_.load();
}

std::size_t MissionRuntime::tick_count() const {
    return tick_count_;
}

MissionLifecycleState MissionRuntime::last_state() const {
    return last_state_;
}

void MissionRuntime::loop() {
    const auto period = std::chrono::duration<double>(1.0 / config_.tick_hz);
    while (running_.load()) {
        const auto tick_start = std::chrono::steady_clock::now();
        (void)tick_once();
        const auto elapsed = std::chrono::steady_clock::now() - tick_start;
        const auto sleep_for = period - elapsed;
        if (sleep_for.count() > 0.0) {
            std::this_thread::sleep_for(sleep_for);
        }
    }
}

void MissionRuntime::write_event(std::string json_fields) {
    if (!event_log_) {
        return;
    }
    std::lock_guard<std::mutex> lock{event_log_mutex_};
    event_log_ << "{" << json_fields << "}\n";
    event_log_.flush();
}

TimePoint MissionRuntime::now_timepoint() const {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return TimePoint{std::chrono::duration_cast<std::chrono::nanoseconds>(now).count()};
}

}  // namespace dedalus
