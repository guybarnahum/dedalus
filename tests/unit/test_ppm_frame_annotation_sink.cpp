#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "dedalus/runtime/core_stack_runner.hpp"
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
                            "dedalus_ppm_frame_annotation_sink_test";
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
    config.frame_annotator = "ppm_sequence";
    config.annotation_output_path = output_dir.string();
    config.annotation_output_fps = 5.0;
    config.fallback_map_frame_id = dedalus::MapFrameId{"map_local_0001"};

    dedalus::CoreStackRunner runner{registry.create(config)};
    if (!runner.run_once()) {
        std::cerr << "ppm annotation runner failed to process synthetic frame\n";
        return 1;
    }

    if (runner.run_once()) {
        std::cerr << "synthetic frame source unexpectedly emitted a second frame\n";
        return 1;
    }

    const auto frame_path = output_dir / "frame_000001.ppm";
    const auto manifest_path = output_dir / "manifest.txt";

    if (!std::filesystem::exists(frame_path)) {
        std::cerr << "ppm annotation sink did not create frame artifact\n";
        return 1;
    }
    if (!std::filesystem::exists(manifest_path)) {
        std::cerr << "ppm annotation sink did not create manifest artifact\n";
        return 1;
    }

    const auto frame_bytes = read_text_file(frame_path);
    if (frame_bytes.rfind("P6\n640 480\n255\n", 0U) != 0U) {
        std::cerr << "ppm annotation artifact has an unexpected header\n";
        return 1;
    }

    const auto expected_size = std::string{"P6\n640 480\n255\n"}.size() +
                               static_cast<std::size_t>(640 * 480 * 3);
    if (std::filesystem::file_size(frame_path) != expected_size) {
        std::cerr << "ppm annotation artifact has an unexpected size\n";
        return 1;
    }

    const auto manifest = read_text_file(manifest_path);
    if (manifest.find("frame_index,frame_id,timestamp_ns,path,output_fps") == std::string::npos ||
        manifest.find("frame_0001") == std::string::npos ||
        manifest.find("frame_000001.ppm") == std::string::npos) {
        std::cerr << "ppm annotation manifest missing expected frame metadata\n";
        return 1;
    }

    std::filesystem::remove_all(output_dir);
    return 0;
}
