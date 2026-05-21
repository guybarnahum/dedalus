#pragma once

#include <atomic>
#include <chrono>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

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

struct MissionRuntimeConfig {
    double tick_hz{10.0};
    int verbosity{0};
    std::string event_log_path{};
};

class MissionRuntime {
public:
    MissionRuntime(
        MissionRuntimeConfig config,
        std::shared_ptr<LatestWorldSnapshot> snapshots,
        std::unique_ptr<MissionController> controller,
        std::unique_ptr<FlightCommandSink> sink,
        std::shared_ptr<MissionEventPublisher> mission_event_publisher = nullptr);
    ~MissionRuntime();

    MissionRuntime(const MissionRuntime&) = delete;
    MissionRuntime& operator=(const MissionRuntime&) = delete;
    MissionRuntime(MissionRuntime&&) = delete;
    MissionRuntime& operator=(MissionRuntime&&) = delete;

    void start();
    void stop();
    void request_finish();
    bool tick_once();

    [[nodiscard]] bool running() const;
    [[nodiscard]] bool finish_requested() const;
    [[nodiscard]] bool terminal_settled() const;
    [[nodiscard]] std::size_t tick_count() const;
    [[nodiscard]] MissionLifecycleState last_state() const;

private:
    void loop();
    void write_event(std::string json_fields);
    [[nodiscard]] TimePoint now_timepoint() const;

    MissionRuntimeConfig config_;
    std::shared_ptr<LatestWorldSnapshot> snapshots_;
    std::unique_ptr<MissionController> controller_;
    std::unique_ptr<FlightCommandSink> sink_;
    std::shared_ptr<MissionEventPublisher> mission_event_publisher_;
    std::atomic<bool> running_{false};
    std::atomic<bool> finish_requested_{false};
    std::atomic<bool> terminal_settled_{false};
    std::thread thread_;
    std::size_t tick_count_{0U};
    MissionLifecycleState last_state_{MissionLifecycleState::Idle};
    std::optional<FlightCommandResult> last_command_result_;
    std::ofstream event_log_;
    mutable std::mutex event_log_mutex_;
};

}  // namespace dedalus
