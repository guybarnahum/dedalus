#include "dedalus/behavior/mission_runtime.hpp"

#include <stdexcept>

namespace dedalus {

MissionRuntime::MissionRuntime(
    MissionRuntimeConfig config,
    std::shared_ptr<LatestWorldSnapshot> snapshots,
    std::unique_ptr<MissionController> controller,
    std::unique_ptr<FlightCommandSink> sink)
    : config_(config),
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
}

MissionRuntime::~MissionRuntime() {
    stop();
}

void MissionRuntime::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
        return;
    }
    thread_ = std::thread([this]() { loop(); });
}

void MissionRuntime::stop() {
    running_.store(false);
    if (thread_.joinable()) {
        thread_.join();
    }
}

bool MissionRuntime::tick_once() {
    const auto snapshot = snapshots_->latest();
    if (!snapshot.has_value()) {
        return false;
    }

    MissionTickInput input;
    input.now = now_timepoint();
    input.snapshot = *snapshot;
    if (input.snapshot.timestamp.timestamp_ns > 0) {
        input.now = input.snapshot.timestamp;
    }

    const auto output = controller_->tick(input);
    last_state_ = output.state;
    ++tick_count_;

    if (output.command.has_value()) {
        sink_->send(*output.command);
    }

    return true;
}

bool MissionRuntime::running() const {
    return running_.load();
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

TimePoint MissionRuntime::now_timepoint() const {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return TimePoint{std::chrono::duration_cast<std::chrono::nanoseconds>(now).count()};
}

}  // namespace dedalus
