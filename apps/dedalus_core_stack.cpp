#include <iostream>

#include "dedalus/runtime/core_stack_runner.hpp"
#include "dedalus/runtime/provider_registry.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

int main() {
    dedalus::ProviderRegistry registry;
    dedalus::CoreStackProviderConfig config;
    config.frame_source = "synthetic";
    config.ego_provider = "frame_hint";
    config.fallback_map_frame_id = dedalus::MapFrameId{"map_local_0001"};

    dedalus::CoreStackRunner runner{registry.create(config)};
    if (!runner.run_once()) {
        return 1;
    }

    std::cout << dedalus::to_json(runner.snapshot());
    return 0;
}
