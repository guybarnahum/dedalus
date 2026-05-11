#include <iostream>
#include <stdexcept>
#include <string>

#include "dedalus/runtime/config_loader.hpp"
#include "dedalus/runtime/core_stack_runner.hpp"
#include "dedalus/runtime/provider_registry.hpp"

namespace {

bool contains(const std::string& haystack, const std::string& needle) {
    return haystack.find(needle) != std::string::npos;
}

}  // namespace

int main() {
    dedalus::ProviderRegistry registry;

    const auto example_config = dedalus::load_core_stack_config("config/core_stack_airsim_example.yaml");
    if (example_config.frame_source != "airsim" || example_config.ego_provider != "airsim" ||
        example_config.detector != "airsim_ground_truth" || example_config.projector != "airsim_depth") {
        std::cerr << "AirSim example config did not parse expected provider names\n";
        return 1;
    }

    auto example_providers = registry.create(example_config);
    bool ego_unavailable = false;
    try {
        dedalus::FramePacket frame;
        frame.timestamp = dedalus::TimePoint{123};
        (void)example_providers.ego_provider->estimate(frame);
    } catch (const std::runtime_error& error) {
        ego_unavailable = contains(error.what(), "AirSimEgoStateProvider") &&
                          contains(error.what(), "integration provider");
    }
    if (!ego_unavailable) {
        std::cerr << "AirSim ego provider did not fail explicitly as unavailable\n";
        return 1;
    }

    const auto bridge_config = dedalus::load_core_stack_config("config/core_stack_airsim_bridge_ci.yaml");
    if (bridge_config.frame_source != "airsim" || bridge_config.ego_provider != "no_telemetry" ||
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

    return 0;
}
