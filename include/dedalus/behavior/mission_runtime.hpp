#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <thread>

#include "dedalus/behavior/latest_world_snapshot.hpp"
#include "dedalus/behavior/mission_controller.hpp"

namespace dedalus {

struct MissionRuntimeConfig {
    double tick_hz{10.0};
    bool debug_logging{false};
};

class MissionRuntime {
public:
    MissionRuntime(
        MissionRuntimeConfig config,
        std::shared_ptr<LatestWorldSnapshot> snapshots,
        std::unique_ptr<MissionController> controller,
        std::unique_ptr<FlightCommandSink> sink);
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
    [[nodiscard]] std::size_t tick_count() const;
    [[nodiscard]] MissionLifecycleState last_state() const;

private:
    void loop();
    [[nodiscard]] TimePoint now_timepoint() const;

    MissionRuntimeConfig config_;
    std::shared_ptr<LatestWorldSnapshot> snapshots_;
    std::unique_ptr<MissionController> controller_;
    std::unique_ptr<FlightCommandSink> sink_;
    std::atomic<bool> running_{false};
    std::atomic<bool> finish_requested_{false};
    std::thread thread_;
    std::size_t tick_count_{0U};
    MissionLifecycleState last_state_{MissionLifecycleState::Idle};
    std::optional<FlightCommandResult> last_command_result_;
};

}  // namespace dedalus