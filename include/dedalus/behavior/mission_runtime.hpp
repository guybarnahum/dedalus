#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "dedalus/behavior/flight_control_state_tracker.hpp"
#include "dedalus/behavior/latest_world_snapshot.hpp"
#include "dedalus/behavior/mission_controller.hpp"
#include "dedalus/runtime/pubsub.hpp"

namespace dedalus {

struct MissionEvent {
    TimePoint timestamp;
    std::string json;
};

class MissionEventSubscriber : public EventSubscriber<MissionEvent> {
public:
    ~MissionEventSubscriber() override = default;

    void on_event(const MissionEvent& event) final {
        on_mission_event(event);
    }

    virtual void on_mission_event(const MissionEvent& event) = 0;
};

using MissionEventPublisher = EventPublisher<MissionEvent>;
using CameraPointingHandoff = std::function<void(const CameraPointingCommand&)>;

struct MissionRuntimeConfig {
    double tick_hz{10.0};
    int verbosity{0};
    std::string event_log_path{};
    // In-flight overrun abort: if N consecutive tick_overrun events occur while
    // in ExecuteMission or GoHome, request a graceful finish so the drone lands
    // before PX4 disarms from Offboard link loss.  0 = disabled.
    int max_consecutive_inflight_overruns{5};
    // Stale snapshot warning: if the same snapshot timestamp is seen for N
    // consecutive ticks, emit a snapshot_stale event + stderr warning.
    // This fires every N ticks while stale (throttled), never aborts.  0 = disabled.
    int stale_snapshot_warn_ticks{3};
};

struct MissionRuntimeStats {
    std::uint64_t events_written{0};
    std::uint64_t event_publish_total_us{0};
    std::uint64_t event_log_write_total_us{0};
    std::uint64_t event_log_flush_total_us{0};
};

class MissionRuntime {
public:
    MissionRuntime(
        MissionRuntimeConfig config,
        std::shared_ptr<LatestWorldSnapshot> snapshots,
        std::unique_ptr<MissionController> controller,
        std::unique_ptr<FlightCommandSink> sink,
        std::shared_ptr<MissionEventPublisher> mission_event_publisher = nullptr,
        std::unique_ptr<CameraPointingSink> camera_pointing_sink = nullptr,
        CameraPointingHandoff camera_pointing_handoff = {});
    ~MissionRuntime();

    MissionRuntime(const MissionRuntime&) = delete;
    MissionRuntime& operator=(const MissionRuntime&) = delete;
    MissionRuntime(MissionRuntime&&) = delete;
    MissionRuntime& operator=(MissionRuntime&&) = delete;

    void start();
    void stop();
    void request_finish();
    bool tick_once();

    void rethrow_if_exception() const;

    [[nodiscard]] bool running() const;
    [[nodiscard]] bool finish_requested() const;
    [[nodiscard]] bool terminal_settled() const;
    [[nodiscard]] std::size_t tick_count() const;
    [[nodiscard]] MissionLifecycleState last_state() const;
    [[nodiscard]] MissionRuntimeStats stats() const;

private:
    void loop();
    void write_event(std::string json_fields);
    [[nodiscard]] TimePoint now_timepoint() const;
    void dispatch_camera_pointing(const MissionTickOutput& output, const MissionTickInput& input);
    void dispatch_command(const MissionTickOutput& output, const MissionTickInput& input);

    MissionRuntimeConfig config_;
    std::shared_ptr<LatestWorldSnapshot> snapshots_;
    std::unique_ptr<MissionController> controller_;
    std::unique_ptr<FlightCommandSink> sink_;
    std::unique_ptr<CameraPointingSink> camera_pointing_sink_;
    CameraPointingHandoff camera_pointing_handoff_;
    std::shared_ptr<MissionEventPublisher> mission_event_publisher_;
    std::atomic<bool> running_{false};
    std::atomic<bool> finish_requested_{false};
    std::atomic<bool> terminal_settled_{false};
    std::thread thread_;
    std::exception_ptr loop_exception_;
    std::size_t tick_count_{0U};
    MissionLifecycleState last_state_{MissionLifecycleState::Idle};
    FlightControlStateTracker flight_control_tracker_;
    std::optional<FlightCommandResult> last_command_result_;
    std::optional<CameraPointingResult> last_camera_pointing_result_;
    std::int64_t consecutive_inflight_overruns_{0};
    std::uint64_t last_snapshot_ts_ns_{0};
    int consecutive_stale_ticks_{0};
    bool stale_cr_pending_{false};  // true when last stale print used \r (needs \n to close)
    std::ofstream event_log_;
    mutable std::mutex event_log_mutex_;
    std::atomic<std::uint64_t> events_written_{0};
    std::atomic<std::uint64_t> event_publish_total_us_{0};
    std::atomic<std::uint64_t> event_log_write_total_us_{0};
    std::atomic<std::uint64_t> event_log_flush_total_us_{0};
};

}  // namespace dedalus
