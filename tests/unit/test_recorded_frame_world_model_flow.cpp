#include <iostream>

#include "dedalus/runtime/config_loader.hpp"
#include "dedalus/runtime/core_stack_runner.hpp"
#include "dedalus/runtime/provider_registry.hpp"
#include "dedalus/sensors/recorded_frame_source.hpp"

int main() {
    dedalus::RecordedFrameSource source{"tests/fixtures/recorded_frames/manifest.txt"};
    const auto frame = source.next_frame();
    if (!frame.has_value()) {
        std::cerr << "recorded frame source produced no frame\n";
        return 1;
    }

    if (frame->frame_id.value != "recorded_frame_0001" || frame->camera_id.value != "recorded_front_center") {
        std::cerr << "recorded frame source did not preserve manifest metadata\n";
        return 1;
    }

    if (frame->image.width != 2 || frame->image.height != 2 || frame->image.channels != 3 ||
        frame->image.bytes.size() != 12U) {
        std::cerr << "recorded frame source did not load expected image dimensions\n";
        return 1;
    }

    const auto config = dedalus::load_core_stack_config("config/core_stack_recorded_ci.yaml");
    if (config.frame_source != "recorded_frames" ||
        config.recorded_manifest_path != "tests/fixtures/recorded_frames/manifest.txt") {
        std::cerr << "recorded provider config did not parse expected frame source settings\n";
        return 1;
    }

    dedalus::ProviderRegistry registry;
    dedalus::CoreStackRunner runner{registry.create(config)};
    if (!runner.run_once()) {
        std::cerr << "recorded provider-composed runner failed\n";
        return 1;
    }

    const auto snapshot = runner.snapshot();
    if (snapshot.active_map_frame_id.value != "map_recorded_ci_0001") {
        std::cerr << "recorded flow did not preserve fallback map frame\n";
        return 1;
    }

    if (snapshot.agents.empty() || snapshot.tactical_exclusion_zones.empty() || snapshot.flight_corridors.empty()) {
        std::cerr << "recorded flow failed to emit world-model artifacts\n";
        return 1;
    }

    return 0;
}
