#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "dedalus/behavior/latest_world_snapshot.hpp"
#include "dedalus/behavior/mission_runtime.hpp"
#include "dedalus/runtime/core_stack_runner.hpp"
#include "dedalus/runtime/provider_registry.hpp"
#include "dedalus/world_model/world_snapshot_subscribers.hpp"

namespace {

class CountingController final : public dedalus::MissionController {
public:
    dedalus::MissionTickOutput tick(const dedalus::MissionTickInput& input) override {
        ++ticks;
        last_height_m = input.snapshot.ego.height_m;
        last_result = input.last_command_result;
        dedalus::MissionTickOutput output;
        output.state = dedalus::MissionLifecycleState::ExecuteMission;
        output.status = "counting";
        dedalus::VelocityCommand command;
        command.timestamp = input.now;
        command.velocity_local_mps = dedalus::Vec3{1.0, 0.0, 0.0};
        output.command = command;
        return output;
    }

    int ticks{0};
    double last_height_m{0.0};
    std::optional<dedalus::FlightCommandResult> last_result;
};

class RecordingSink final : public dedalus::FlightCommandSink {
public:
    dedalus::FlightCommandResult send(const dedalus::VelocityCommand& command) override {
        commands.push_back(command);
        return dedalus::FlightCommandResult{command.kind, true, "OK recording sink"};
    }

    std::vector<dedalus::VelocityCommand> commands;
};

class RecordingMissionEventSubscriber final : public dedalus::MissionEventSubscriber {
public:
    void on_mission_event(const dedalus::MissionEvent& event) override {
        events.push_back(event.json);
    }

    std::vector<std::string> events;
};

bool file_contains(const std::filesystem::path& path, const std::string& needle) {
    std::ifstream input{path};
    if (!input) {
        return false;
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return buffer.str().find(needle) != std::string::npos;
}

bool any_event_contains(const std::vector<std::string>& events, const std::string& needle) {
    for (const auto& event : events) {
        if (event.find(needle) != std::string::npos) {
            return true;
        }
    }
    return false;
}

}  // namespace

int main() {
    auto latest_snapshot = std::make_shared<dedalus::LatestWorldSnapshot>();
    auto snapshot_publisher = std::make_shared<dedalus::WorldSnapshotPublisher>();
    auto latest_snapshot_subscriber = std::make_shared<dedalus::LatestWorldSnapshotSubscriber>(latest_snapshot);
    snapshot_publisher->subscribe(latest_snapshot_subscriber);

    dedalus::ProviderRegistry registry;
    dedalus::CoreStackProviderConfig config;
    config.frame_source = "synthetic";
    config.ego_provider = "frame_hint";
    config.detector = "scripted";
    config.camera_stabilizer = "null";
    config.tracker = "simple_centroid";
    config.identity_resolver = "appearance_only";
    config.projector = "flat_ground";
    config.world_model = "in_memory";
    config.frame_annotator = "null";

    dedalus::CoreStackRunner runner{registry.create(config), nullptr, snapshot_publisher};
    if (!runner.run_once()) {
        std::cerr << "CoreStackRunner failed to publish a snapshot\n";
        return 1;
    }

    const auto published = latest_snapshot->latest();
    if (!published.has_value()) {
        std::cerr << "LatestWorldSnapshot did not receive a published snapshot\n";
        return 1;
    }
    if (!published->ego.height_valid || published->ego.height_m <= 0.0) {
        std::cerr << "Published snapshot did not contain mission-ready ego height\n";
        return 1;
    }

    auto controller = std::make_unique<CountingController>();
    auto* controller_ptr = controller.get();
    auto sink = std::make_unique<RecordingSink>();
    auto* sink_ptr = sink.get();
    auto mission_event_publisher = std::make_shared<dedalus::MissionEventPublisher>();
    auto mission_event_subscriber = std::make_shared<RecordingMissionEventSubscriber>();
    mission_event_publisher->subscribe(mission_event_subscriber);
    const auto event_log_path = std::filesystem::temp_directory_path() / "dedalus_mission_runtime_events_test.jsonl";
    const auto lifecycle_log_path = std::filesystem::temp_directory_path() / "dedalus_mission_runtime_lifecycle_test.jsonl";
    std::filesystem::remove(event_log_path);
    std::filesystem::remove(lifecycle_log_path);

    {
        dedalus::MissionRuntime runtime{
            dedalus::MissionRuntimeConfig{
                .tick_hz = 10.0,
                .verbosity = 0,
                .event_log_path = event_log_path.string()},
            latest_snapshot,
            std::move(controller),
            std::move(sink),
            mission_event_publisher};

        if (!runtime.tick_once()) {
            std::cerr << "MissionRuntime did not tick with available snapshot\n";
            return 1;
        }

        if (controller_ptr->ticks != 1) {
            std::cerr << "MissionRuntime did not tick the controller exactly once\n";
            return 1;
        }
        if (controller_ptr->last_result.has_value()) {
            std::cerr << "MissionRuntime should not pass a command result before the first command\n";
            return 1;
        }
        if (controller_ptr->last_height_m <= 0.0) {
            std::cerr << "MissionRuntime did not pass mission-ready ego height to controller\n";
            return 1;
        }
        if (sink_ptr->commands.size() != 1U) {
            std::cerr << "MissionRuntime did not forward controller command to sink\n";
            return 1;
        }

        if (!runtime.tick_once()) {
            std::cerr << "MissionRuntime did not tick a second time\n";
            return 1;
        }
        if (!controller_ptr->last_result.has_value() || !controller_ptr->last_result->success) {
            std::cerr << "MissionRuntime did not surface previous command result to controller\n";
            return 1;
        }
    }

    if (!file_contains(event_log_path, "\"event\":\"command_dispatch\"")) {
        std::cerr << "MissionRuntime event log missing command_dispatch\n";
        return 1;
    }
    if (!file_contains(event_log_path, "\"event\":\"command_result\"")) {
        std::cerr << "MissionRuntime event log missing command_result\n";
        return 1;
    }
    if (!file_contains(event_log_path, "\"event\":\"runtime_stop\"")) {
        std::cerr << "MissionRuntime event log missing runtime_stop\n";
        return 1;
    }
    if (!any_event_contains(mission_event_subscriber->events, "\"event\":\"command_dispatch\"")) {
        std::cerr << "MissionRuntime publisher missing command_dispatch\n";
        return 1;
    }
    if (!any_event_contains(mission_event_subscriber->events, "\"event\":\"command_result\"")) {
        std::cerr << "MissionRuntime publisher missing command_result\n";
        return 1;
    }
    if (!any_event_contains(mission_event_subscriber->events, "\"event\":\"runtime_stop\"")) {
        std::cerr << "MissionRuntime publisher missing runtime_stop\n";
        return 1;
    }

    std::filesystem::remove(event_log_path);
    std::filesystem::remove(lifecycle_log_path);
    return 0;
}
