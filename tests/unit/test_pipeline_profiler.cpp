#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "dedalus/runtime/core_stack_runner.hpp"
#include "dedalus/runtime/pipeline_profiler.hpp"
#include "dedalus/runtime/provider_registry.hpp"

namespace {

std::string read_text_file(const std::filesystem::path& path) {
    std::ifstream input{path, std::ios::binary};
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
}

}  // namespace

int main() {
    const auto output_dir = std::filesystem::temp_directory_path() /
                            "dedalus_pipeline_profiler_test";
    const auto profile_path = output_dir / "pipeline_profile.jsonl";
    std::filesystem::remove_all(output_dir);

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
    config.fallback_map_frame_id = dedalus::MapFrameId{"map_local_0001"};

    auto timing_writer = std::make_unique<dedalus::PipelineProfiler>(profile_path);
    dedalus::CoreStackRunner runner{registry.create(config), std::move(timing_writer)};

    if (!runner.run_once()) {
        std::cerr << "pipeline profiler runner failed to process synthetic frame\n";
        return 1;
    }

    if (runner.run_once()) {
        std::cerr << "synthetic frame source unexpectedly emitted a second frame\n";
        return 1;
    }

    if (!std::filesystem::exists(profile_path)) {
        std::cerr << "pipeline profiler did not create JSONL output\n";
        return 1;
    }

    const auto profile = read_text_file(profile_path);
    if (profile.empty()) {
        std::cerr << "pipeline profiler output is empty\n";
        return 1;
    }

    const std::string required_tokens[] = {
        "\"frame_id\":\"frame_0001\"",
        "\"timestamp_ns\":123456789",
        "\"total_us\":",
        "\"frame_source.next_frame\":",
        "\"ego_provider.estimate\":",
        "\"perception_pipeline.process\":",
        "\"world_model.update_ego\":",
        "\"world_model.ingest\":",
        "\"world_model.snapshot\":",
        "\"frame_annotator.annotate\":"};

    for (const auto& token : required_tokens) {
        if (profile.find(token) == std::string::npos) {
            std::cerr << "pipeline profiler output missing token: " << token << "\n";
            std::cerr << profile << "\n";
            return 1;
        }
    }

    std::filesystem::remove_all(output_dir);
    return 0;
}
