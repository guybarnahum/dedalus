#include <iostream>

#include "dedalus/runtime/config_loader.hpp"
#include "dedalus/runtime/core_stack_runner.hpp"
#include "dedalus/runtime/provider_registry.hpp"

int main() {
    const auto config = dedalus::load_core_stack_config("config/core_stack_ci.yaml");

    if (config.frame_source != "synthetic" || config.ego_provider != "frame_hint" ||
        config.detector != "scripted" || config.tracker != "simple_centroid" ||
        config.identity_resolver != "appearance_only" || config.projector != "flat_ground" ||
        config.world_model != "in_memory") {
        std::cerr << "core_stack_ci provider config did not parse expected provider names\n";
        return 1;
    }

    if (config.pipeline_timing_enabled) {
        std::cerr << "core_stack_ci should keep pipeline timing disabled by default\n";
        return 1;
    }

    if (config.fallback_map_frame_id.value != "map_local_0001") {
        std::cerr << "core_stack_ci config did not parse expected fallback map frame\n";
        return 1;
    }

    const auto profile_config = dedalus::load_core_stack_config("config/core_stack_profile_ci.yaml");
    if (!profile_config.pipeline_timing_enabled) {
        std::cerr << "core_stack_profile_ci did not enable pipeline timing\n";
        return 1;
    }
    if (profile_config.pipeline_timing_output_path != "out/profile/pipeline_profile.jsonl") {
        std::cerr << "core_stack_profile_ci did not parse expected timing output path\n";
        return 1;
    }

    dedalus::ProviderRegistry registry;
    dedalus::CoreStackRunner runner{registry.create(config)};
    if (!runner.run_once()) {
        std::cerr << "config-composed runner failed\n";
        return 1;
    }

    const auto snapshot = runner.snapshot();
    if (snapshot.active_map_frame_id.value != "map_local_0001" || snapshot.agents.empty()) {
        std::cerr << "config-composed snapshot missing expected state\n";
        return 1;
    }

    return 0;
}
