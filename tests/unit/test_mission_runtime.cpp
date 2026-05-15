#include <iostream>
#include <optional>
#include <vector>

#include "dedalus/behavior/latest_world_snapshot.hpp"
#include "dedalus/behavior/mission_runtime.hpp"
#include "dedalus/runtime/core_stack_runner.hpp"
#include "dedalus/runtime/provider_registry.hpp"

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

}  // namespace

int main() {
    auto latest_snapshot = std::make_shared<dedalus::LatestWorldSnapshot>();

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

    dedalus::CoreStackRunner runner{registry.create(config), nullptr, latest_snapshot};
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

    dedalus::MissionRuntime runtime{
        dedalus::MissionRuntimeConfig{.tick_hz = 10.0},
        latest_snapshot,
        std::move(controller),
        std::move(sink)};

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
        std::cerr << "MissionRuntime did not tick a second time with available snapshot\n";
        return 1;
    }
    if (!controller_ptr->last_result.has_value() || !controller_ptr->last_result->success) {
        std::cerr << "MissionRuntime did not pass successful command result to next controller tick\n";
        return 1;
    }

    if (runtime.tick_count() != 2U || runtime.last_state() != dedalus::MissionLifecycleState::ExecuteMission) {
        std::cerr << "MissionRuntime did not expose expected tick count/state\n";
        return 1;
    }

    auto empty_snapshots = std::make_shared<dedalus::LatestWorldSnapshot>();
    auto empty_runtime = dedalus::MissionRuntime{
        dedalus::MissionRuntimeConfig{.tick_hz = 10.0},
        empty_snapshots,
        std::make_unique<CountingController>(),
        std::make_unique<RecordingSink>()};
    if (empty_runtime.tick_once()) {
        std::cerr << "MissionRuntime ticked without an available snapshot\n";
        return 1;
    }

    return 0;
}
