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
    dedalus::CoreStackRunner runner{
        registry.create(config),
        dedalus::CoreStackRunnerConfig{.timing_writer = std::move(timing_writer)}};

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
        "\"frame_source.next_frame_wait\":",
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

    // --- RAII flush test ---
    // Verify that destroying a PipelineProfiler with an open frame (begin_frame
    // called but end_frame never called) writes the frame to disk via the
    // destructor rather than silently discarding it.
    const auto raii_profile_path = output_dir / "raii_pipeline_profile.jsonl";
    {
        dedalus::PipelineProfiler raii_profiler{raii_profile_path};
        dedalus::FramePacket raii_frame;
        raii_frame.frame_id = dedalus::FrameId{"frame_raii"};
        raii_frame.timestamp = dedalus::TimePoint{987654321};
        raii_profiler.begin_frame(raii_frame);
        raii_profiler.record_stage("dummy_stage", 42);
        // Deliberately omit end_frame() — destructor must flush.
    }  // raii_profiler destroyed here

    if (!std::filesystem::exists(raii_profile_path)) {
        std::cerr << "RAII flush test: profiler destructor did not create output\n";
        return 1;
    }
    {
        const auto raii_profile = read_text_file(raii_profile_path);
        if (raii_profile.find("\"frame_id\":\"frame_raii\"") == std::string::npos) {
            std::cerr << "RAII flush test: in-progress frame not written by destructor\n";
            std::cerr << raii_profile << "\n";
            return 1;
        }
        if (raii_profile.find("\"dummy_stage\":42") == std::string::npos) {
            std::cerr << "RAII flush test: stage data missing from destructor-flushed frame\n";
            std::cerr << raii_profile << "\n";
            return 1;
        }
    }

    std::filesystem::remove_all(output_dir);
    return 0;
}
