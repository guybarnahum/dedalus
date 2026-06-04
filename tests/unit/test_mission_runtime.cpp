#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "dedalus/behavior/latest_world_snapshot.hpp"
#include "dedalus/behavior/mission_runtime.hpp"
#include "dedalus/runtime/camera_pointing_state_store.hpp"
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

class CameraPointingController final : public dedalus::MissionController {
public:
    dedalus::MissionTickOutput tick(const dedalus::MissionTickInput& input) override {
        ++ticks;
        dedalus::MissionTickOutput output;
        output.state = dedalus::MissionLifecycleState::ExecuteMission;
        output.status = "camera_pointing";
        dedalus::CameraPointingCommand command;
        command.timestamp = input.now;
        command.cameras = {"front_center", "0"};
        command.mode = "target";
        command.pitch_rad = commanded_pitch_rad;
        command.pitch_valid = true;
        command.source_track_id = "ghost_person_001";
        output.camera_pointing = command;
        return output;
    }

    double commanded_pitch_rad{0.42};
    int ticks{0};
};

class RecordingSink final : public dedalus::FlightCommandSink {
public:
    dedalus::FlightCommandResult send(const dedalus::VelocityCommand& command) override {
        commands.push_back(command);
        return dedalus::FlightCommandResult{command.kind, true, "OK recording sink"};
    }

    std::vector<dedalus::VelocityCommand> commands;
};

class RecordingCameraPointingSink final : public dedalus::CameraPointingSink {
public:
    dedalus::CameraPointingResult send(const dedalus::CameraPointingCommand& command) override {
        commands.push_back(command);
        return dedalus::CameraPointingResult{true, "OK camera sink"};
    }

    std::vector<dedalus::CameraPointingCommand> commands;
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

bool near(double lhs, double rhs, double tolerance = 1.0e-6) {
    return std::abs(lhs - rhs) <= tolerance;
}

dedalus::CoreStackProviderConfig synthetic_sensing_config() {
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

    dedalus::CameraSensingConfig camera;
    camera.camera_id = dedalus::CameraId{"front_center"};
    camera.camera_name = "front_center";
    camera.role = "visual_obstacle_detector";
    camera.horizontal_fov_rad = 1.5707963267948966;
    camera.vertical_fov_rad = 1.0471975511965976;
    camera.near_range_m = 0.5;
    camera.far_range_m = 80.0;
    camera.pointing_source = "camera_pointing_intent";
    config.mission_options.obstacle_sensing_cameras.push_back(camera);
    return config;
}

}  // namespace

int main() {
    auto latest_snapshot = std::make_shared<dedalus::LatestWorldSnapshot>();
    auto snapshot_publisher = std::make_shared<dedalus::WorldSnapshotPublisher>();
    auto latest_snapshot_subscriber = std::make_shared<dedalus::LatestWorldSnapshotSubscriber>(latest_snapshot);
    snapshot_publisher->subscribe(latest_snapshot_subscriber);

    dedalus::ProviderRegistry registry;
    const auto config = synthetic_sensing_config();

    dedalus::CoreStackRunner runner{
        registry.create(config),
        dedalus::CoreStackRunnerConfig{.snapshot_publisher = snapshot_publisher}};
    if (!runner.run_once()) {
        std::cerr << "CoreStackRunner failed to publish a snapshot\n";
        return 1;
    }

    const auto published = latest_snapshot->latest();
    if (!published) {
        std::cerr << "LatestWorldSnapshot did not receive a published snapshot\n";
        return 1;
    }
    if (!published->ego.height_valid || published->ego.height_m <= 0.0) {
        std::cerr << "Published snapshot did not contain mission-ready ego height\n";
        return 1;
    }
    if (published->obstacle_sensing_volumes.size() != 1U) {
        std::cerr << "Initial runner did not publish neutral sensing coverage\n";
        return 1;
    }
    const auto& neutral_volume = published->obstacle_sensing_volumes.front();
    if (!near(neutral_volume.forward_axis_local.x, 1.0) ||
        !near(neutral_volume.forward_axis_local.y, 0.0) ||
        !near(neutral_volume.forward_axis_local.z, 0.0)) {
        std::cerr << "Initial sensing coverage should be neutral body-forward camera coverage\n";
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
    const auto camera_event_log_path = std::filesystem::temp_directory_path() / "dedalus_mission_runtime_camera_events_test.jsonl";
    const auto lifecycle_log_path = std::filesystem::temp_directory_path() / "dedalus_mission_runtime_lifecycle_test.jsonl";
    std::filesystem::remove(event_log_path);
    std::filesystem::remove(camera_event_log_path);
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

    auto camera_controller = std::make_unique<CameraPointingController>();
    auto* camera_controller_ptr = camera_controller.get();
    const double commanded_pitch_rad = camera_controller_ptr->commanded_pitch_rad;
    auto flight_sink = std::make_unique<RecordingSink>();
    auto camera_sink = std::make_unique<RecordingCameraPointingSink>();
    auto* camera_sink_ptr = camera_sink.get();
    dedalus::CameraPointingStateStore pointing_state_store;
    {
        dedalus::MissionRuntime runtime{
            dedalus::MissionRuntimeConfig{
                .tick_hz = 10.0,
                .verbosity = 0,
                .event_log_path = camera_event_log_path.string()},
            latest_snapshot,
            std::move(camera_controller),
            std::move(flight_sink),
            nullptr,
            std::move(camera_sink),
            [&](const dedalus::CameraPointingCommand& command) {
                pointing_state_store.apply(command);
            }};

        if (!runtime.tick_once()) {
            std::cerr << "MissionRuntime did not tick camera-pointing controller\n";
            return 1;
        }
        if (camera_controller_ptr->ticks != 1) {
            std::cerr << "MissionRuntime did not tick camera-pointing controller exactly once\n";
            return 1;
        }
        if (camera_sink_ptr->commands.size() != 1U) {
            std::cerr << "MissionRuntime did not send camera pointing command to sink\n";
            return 1;
        }
    }

    const auto front_state = pointing_state_store.state_for_camera_name("front_center");
    const auto numeric_state = pointing_state_store.state_for_camera(dedalus::CameraId{"0"});
    if (!front_state.has_value() || !numeric_state.has_value()) {
        std::cerr << "MissionRuntime camera pointing handoff did not populate store for all cameras\n";
        return 1;
    }
    if (!front_state->valid || front_state->source != "camera_pointing_intent" || !near(front_state->pitch_rad, commanded_pitch_rad)) {
        std::cerr << "MissionRuntime camera pointing handoff did not preserve front camera state\n";
        return 1;
    }

    dedalus::CoreStackRunner pitched_runner{
        registry.create(config),
        dedalus::CoreStackRunnerConfig{.snapshot_publisher = snapshot_publisher}};
    pitched_runner.update_camera_pointing_states(pointing_state_store.states());
    if (!pitched_runner.run_once()) {
        std::cerr << "Pitched CoreStackRunner failed to publish a snapshot\n";
        return 1;
    }
    const auto pitched_snapshot = latest_snapshot->latest();
    if (!pitched_snapshot || pitched_snapshot->obstacle_sensing_volumes.size() != 1U) {
        std::cerr << "Pitched runner did not publish sensing coverage\n";
        return 1;
    }
    const auto& pitched_volume = pitched_snapshot->obstacle_sensing_volumes.front();
    if (!near(pitched_volume.forward_axis_local.x, std::cos(commanded_pitch_rad)) ||
        !near(pitched_volume.forward_axis_local.y, 0.0) ||
        !near(pitched_volume.forward_axis_local.z, -std::sin(commanded_pitch_rad))) {
        std::cerr << "Pitched sensing coverage did not reflect MissionRuntime camera-pointing handoff: got ["
                  << pitched_volume.forward_axis_local.x << ", "
                  << pitched_volume.forward_axis_local.y << ", "
                  << pitched_volume.forward_axis_local.z << "]\n";
        return 1;
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
    if (!file_contains(camera_event_log_path, "\"event\":\"camera_pointing_dispatch\"")) {
        std::cerr << "MissionRuntime camera event log missing camera_pointing_dispatch\n";
        return 1;
    }
    if (!file_contains(camera_event_log_path, "\"event\":\"camera_pointing_result\"")) {
        std::cerr << "MissionRuntime camera event log missing camera_pointing_result\n";
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
    std::filesystem::remove(camera_event_log_path);
    std::filesystem::remove(lifecycle_log_path);
    return 0;
}
