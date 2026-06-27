#include <cmath>
#include <iostream>
#include <string>

#include "dedalus/runtime/config_loader.hpp"
#include "dedalus/runtime/core_stack_runner.hpp"
#include "dedalus/runtime/provider_registry.hpp"

namespace {

bool nearly_equal(double lhs, double rhs) {
    return std::abs(lhs - rhs) < 1.0e-9;
}

}  // namespace

int main() {
    dedalus::ProviderRegistry registry;

    const auto example_config = dedalus::load_core_stack_config("config/ci/core_stack_airsim_example.yaml");
    if (example_config.frame_source != "airsim" || example_config.ego_provider != "airsim" ||
        example_config.detector != "airsim_ground_truth" || example_config.projector != "airsim_depth") {
        std::cerr << "AirSim example config did not parse expected provider names\n";
        return 1;
    }

    const auto bridge_config = dedalus::load_core_stack_config("config/ci/core_stack_airsim_bridge_ci.yaml");
    if (bridge_config.frame_source != "airsim" || bridge_config.ego_provider != "airsim" ||
        bridge_config.detector != "scripted" || bridge_config.projector != "flat_ground") {
        std::cerr << "AirSim bridge config did not parse expected provider names\n";
        return 1;
    }

    auto bridge_providers = registry.create(bridge_config);
    const auto frame = bridge_providers.frame_source->next_frame();
    if (!frame.has_value() || frame->camera_id.value != "front_center" || frame->image.width != 2 ||
        frame->image.height != 2 || frame->image.bytes.size() != 12U) {
        std::cerr << "AirSim bridge frame source did not parse expected PPM frame\n";
        return 1;
    }

    const auto ego_estimate = bridge_providers.ego_provider->estimate(*frame);
    if (!ego_estimate.ego.has_value() || !ego_estimate.telemetry_available || ego_estimate.confidence < 0.8F) {
        std::cerr << "AirSim ego bridge did not report telemetry-backed ego state\n";
        return 1;
    }

    const auto ego = *ego_estimate.ego;
    if (ego.map_frame_id.value != "map_airsim_bridge_ci_0001" ||
        ego.timestamp.timestamp_ns != 123456789 ||
        !nearly_equal(ego.local_T_body.position.x, 1.0) ||
        !nearly_equal(ego.velocity_local.x, 4.0) ||
        !nearly_equal(ego.angular_velocity_body.z, 0.03)) {
        std::cerr << "AirSim ego bridge did not parse expected pose/velocity values\n";
        return 1;
    }

    dedalus::CoreStackRunner runner{registry.create(bridge_config)};
    if (!runner.run_once()) {
        std::cerr << "AirSim bridge runner failed\n";
        return 1;
    }

    const auto snapshot = runner.snapshot();
    if (snapshot.active_map_frame_id.value != "map_airsim_bridge_ci_0001" || snapshot.agents.empty()) {
        std::cerr << "AirSim bridge snapshot missing expected state\n";
        return 1;
    }

    if (!nearly_equal(snapshot.ego.local_T_body.position.x, 1.0) ||
        !nearly_equal(snapshot.ego.velocity_local.y, 5.0)) {
        std::cerr << "AirSim bridge snapshot did not preserve ego telemetry\n";
        return 1;
    }

    const auto stream_config = dedalus::load_core_stack_config("config/ci/core_stack_airsim_stream_ci.yaml");
    if (stream_config.bridge_mode != "stream_jsonl" || stream_config.frame_source != "airsim") {
        std::cerr << "AirSim stream config did not parse expected bridge mode\n";
        return 1;
    }

    auto stream_providers = registry.create(stream_config);
    const auto stream_frame_1 = stream_providers.frame_source->next_frame();
    const auto stream_frame_2 = stream_providers.frame_source->next_frame();
    const auto stream_frame_3 = stream_providers.frame_source->next_frame();
    if (!stream_frame_1.has_value() || !stream_frame_2.has_value() || stream_frame_3.has_value()) {
        std::cerr << "AirSim stream bridge did not produce exactly two CI frames\n";
        return 1;
    }

    if (stream_frame_1->frame_id.value != "airsim_stream_frame_0001" ||
        stream_frame_2->frame_id.value != "airsim_stream_frame_0002" ||
        stream_frame_1->timestamp.timestamp_ns != 1000 ||
        stream_frame_2->timestamp.timestamp_ns != 1001 ||
        stream_frame_1->image.bytes.size() != 12U) {
        std::cerr << "AirSim stream bridge did not preserve frame metadata or payload\n";
        return 1;
    }

    const auto binary_config = dedalus::load_core_stack_config("config/ci/core_stack_airsim_binary_ci.yaml");
    if (binary_config.bridge_mode != "stream_binary" || binary_config.frame_source != "airsim") {
        std::cerr << "AirSim binary config did not parse expected bridge mode\n";
        return 1;
    }

    auto binary_providers = registry.create(binary_config);
    const auto binary_frame_1 = binary_providers.frame_source->next_frame();
    const auto binary_frame_2 = binary_providers.frame_source->next_frame();
    const auto binary_frame_3 = binary_providers.frame_source->next_frame();
    if (!binary_frame_1.has_value() || !binary_frame_2.has_value() || binary_frame_3.has_value()) {
        std::cerr << "AirSim binary bridge did not produce exactly two CI frames\n";
        return 1;
    }

    if (binary_frame_1->frame_id.value != "binary_stream_frame_1" ||
        binary_frame_2->frame_id.value != "binary_stream_frame_2" ||
        binary_frame_1->timestamp.timestamp_ns != 2000 ||
        binary_frame_2->timestamp.timestamp_ns != 2001 ||
        binary_frame_1->image.width != 2 ||
        binary_frame_1->image.height != 2 ||
        binary_frame_1->image.channels != 3 ||
        binary_frame_1->image.bytes.size() != 12U) {
        std::cerr << "AirSim binary bridge did not preserve frame metadata or payload\n";
        return 1;
    }

    const auto state_config = dedalus::load_core_stack_config("config/ci/core_stack_airsim_binary_state_ci.yaml");
    if (state_config.bridge_mode != "stream_binary_ego" || state_config.ego_provider != "frame_hint") {
        std::cerr << "AirSim binary state config did not parse expected bridge mode/providers\n";
        return 1;
    }

    auto state_providers = registry.create(state_config);
    const auto state_frame_1 = state_providers.frame_source->next_frame();
    const auto state_frame_2 = state_providers.frame_source->next_frame();
    const auto state_frame_3 = state_providers.frame_source->next_frame();
    if (!state_frame_1.has_value() || !state_frame_2.has_value() || state_frame_3.has_value()) {
        std::cerr << "AirSim binary state bridge did not produce exactly two CI frames\n";
        return 1;
    }

    if (!state_frame_1->ego_hint.has_value() || !state_frame_2->ego_hint.has_value()) {
        std::cerr << "AirSim binary state bridge did not attach ego hints\n";
        return 1;
    }

    const auto state_ego_estimate = state_providers.ego_provider->estimate(*state_frame_1);
    if (!state_ego_estimate.ego.has_value() || !state_ego_estimate.telemetry_available ||
        state_ego_estimate.confidence < 0.9F) {
        std::cerr << "FrameHintEgoProvider did not use binary state sidecar\n";
        return 1;
    }

    const auto state_ego = *state_ego_estimate.ego;
    if (state_ego.map_frame_id.value != "map_airsim_binary_state_ci_0001" ||
        state_ego.timestamp.timestamp_ns != 3000 ||
        !nearly_equal(state_ego.local_T_body.position.x, 2.0) ||
        !nearly_equal(state_ego.velocity_local.y, 6.0)) {
        std::cerr << "AirSim binary state sidecar did not preserve expected ego values\n";
        return 1;
    }

    dedalus::CoreStackRunner state_runner{registry.create(state_config)};
    if (!state_runner.run_once()) {
        std::cerr << "AirSim binary state runner failed\n";
        return 1;
    }
    const auto state_snapshot = state_runner.snapshot();
    if (state_snapshot.active_map_frame_id.value != "map_airsim_binary_state_ci_0001" ||
        !nearly_equal(state_snapshot.ego.local_T_body.position.x, 2.0) ||
        !nearly_equal(state_snapshot.ego.velocity_local.y, 6.0)) {
        std::cerr << "AirSim binary state runner did not preserve frame-attached ego telemetry\n";
        return 1;
    }

    return 0;
}
