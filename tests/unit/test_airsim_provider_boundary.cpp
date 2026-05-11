#include <iostream>
#include <stdexcept>
#include <string>

#include "dedalus/runtime/config_loader.hpp"
#include "dedalus/runtime/provider_registry.hpp"
#include "dedalus/simulation/airsim_providers.hpp"

namespace {

bool contains(const std::string& haystack, const std::string& needle) {
    return haystack.find(needle) != std::string::npos;
}

}  // namespace

int main() {
    const auto config = dedalus::load_core_stack_config("config/core_stack_airsim_example.yaml");

    if (config.frame_source != "airsim" || config.ego_provider != "airsim" ||
        config.detector != "airsim_ground_truth" || config.projector != "airsim_depth") {
        std::cerr << "AirSim example config did not parse expected provider names\n";
        return 1;
    }

    if (config.airsim_host != "127.0.0.1" || config.airsim_rpc_port != 41451 ||
        config.airsim_vehicle_name != "PX4" || config.airsim_camera_name != "front_center") {
        std::cerr << "AirSim example config did not parse expected connection settings\n";
        return 1;
    }

    dedalus::ProviderRegistry registry;
    const auto frame_sources = registry.frame_sources();
    bool has_airsim = false;
    for (const auto& name : frame_sources) {
        if (name == "airsim") {
            has_airsim = true;
        }
    }
    if (!has_airsim) {
        std::cerr << "ProviderRegistry does not list AirSim frame source\n";
        return 1;
    }

    auto providers = registry.create(config);

    bool threw = false;
    try {
        (void)providers.frame_source->next_frame();
    } catch (const std::runtime_error& error) {
        threw = contains(error.what(), "AirSimFrameSource") &&
                contains(error.what(), "integration provider");
    }

    if (!threw) {
        std::cerr << "AirSimFrameSource did not fail explicitly as unavailable in dependency-free build\n";
        return 1;
    }

    return 0;
}
