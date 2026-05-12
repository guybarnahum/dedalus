#include <iostream>
#include <stdexcept>

#include "dedalus/runtime/core_stack_runner.hpp"
#include "dedalus/runtime/provider_registry.hpp"

int main() {
    dedalus::ProviderRegistry registry;

    const auto frame_sources = registry.frame_sources();
    if (frame_sources.size() < 2U) {
        std::cerr << "provider registry missing expected frame sources\n";
        return 1;
    }

    if (registry.camera_stabilizers().empty() || registry.frame_annotators().empty()) {
        std::cerr << "provider registry missing stabilizer or annotator providers\n";
        return 1;
    }

    dedalus::CoreStackProviderConfig synthetic_config;
    synthetic_config.frame_source = "synthetic";
    synthetic_config.ego_provider = "frame_hint";
    synthetic_config.camera_stabilizer = "null";
    synthetic_config.frame_annotator = "null";
    synthetic_config.fallback_map_frame_id = dedalus::MapFrameId{"map_local_0001"};

    dedalus::CoreStackRunner synthetic_runner{registry.create(synthetic_config)};
    if (!synthetic_runner.run_once()) {
        std::cerr << "synthetic provider-composed runner failed\n";
        return 1;
    }

    const auto synthetic_snapshot = synthetic_runner.snapshot();
    if (synthetic_snapshot.active_map_frame_id.value != "map_local_0001" || synthetic_snapshot.agents.empty()) {
        std::cerr << "synthetic provider-composed snapshot missing expected state\n";
        return 1;
    }

    dedalus::CoreStackProviderConfig video_config;
    video_config.frame_source = "video_only";
    video_config.ego_provider = "no_telemetry";
    video_config.camera_stabilizer = "null";
    video_config.frame_annotator = "null";
    video_config.fallback_map_frame_id = dedalus::MapFrameId{"map_video_only_0001"};

    dedalus::CoreStackRunner video_runner{registry.create(video_config)};
    if (!video_runner.run_once()) {
        std::cerr << "video-only provider-composed runner failed\n";
        return 1;
    }

    const auto video_snapshot = video_runner.snapshot();
    if (video_snapshot.active_map_frame_id.value != "map_video_only_0001" || video_snapshot.agents.empty()) {
        std::cerr << "video-only provider-composed snapshot missing expected degraded state\n";
        return 1;
    }

    bool threw = false;
    try {
        dedalus::CoreStackProviderConfig bad_config;
        bad_config.frame_source = "missing_provider";
        (void)registry.create(bad_config);
    } catch (const std::invalid_argument&) {
        threw = true;
    }

    if (!threw) {
        std::cerr << "provider registry accepted an unknown provider\n";
        return 1;
    }

    threw = false;
    try {
        dedalus::CoreStackProviderConfig bad_config;
        bad_config.camera_stabilizer = "missing_provider";
        (void)registry.create(bad_config);
    } catch (const std::invalid_argument&) {
        threw = true;
    }

    if (!threw) {
        std::cerr << "provider registry accepted an unknown stabilizer provider\n";
        return 1;
    }

    return 0;
}
