#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include "dedalus/runtime/config_loader.hpp"
#include "dedalus/runtime/core_stack_runner.hpp"
#include "dedalus/runtime/provider_registry.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

namespace {

struct Args {
    std::string config_path{"config/core_stack_recorded_ci.yaml"};
    std::filesystem::path output_dir{"out/replay_snapshots"};
    int max_frames{0};
};

std::string zero_padded(int value, int width) {
    std::ostringstream out;
    out << std::setw(width) << std::setfill('0') << value;
    return out.str();
}

Args parse_args(int argc, char** argv) {
    Args args;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--config") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--config requires a path");
            }
            args.config_path = argv[++i];
        } else if (arg == "--output-dir") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--output-dir requires a path");
            }
            args.output_dir = argv[++i];
        } else if (arg == "--max-frames") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--max-frames requires a value");
            }
            args.max_frames = std::stoi(argv[++i]);
            if (args.max_frames < 0) {
                throw std::invalid_argument("--max-frames must be >= 0");
            }
        } else {
            throw std::invalid_argument("unknown argument: " + arg);
        }
    }

    return args;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const auto args = parse_args(argc, argv);
        std::filesystem::create_directories(args.output_dir);

        const auto config = dedalus::load_core_stack_config(args.config_path);
        dedalus::ProviderRegistry registry;
        dedalus::CoreStackRunner runner{registry.create(config)};

        const auto manifest_path = args.output_dir / "snapshot_manifest.txt";
        std::ofstream manifest{manifest_path};
        if (!manifest) {
            throw std::runtime_error("failed to open snapshot manifest: " + manifest_path.string());
        }
        manifest << "# index path timestamp_ns active_map_frame_id\n";

        int frame_count = 0;
        while (args.max_frames == 0 || frame_count < args.max_frames) {
            if (!runner.run_once()) {
                break;
            }

            ++frame_count;
            const auto snapshot = runner.snapshot();
            const auto snapshot_name = "snapshot_" + zero_padded(frame_count, 4) + ".json";
            const auto snapshot_path = args.output_dir / snapshot_name;

            std::ofstream snapshot_file{snapshot_path};
            if (!snapshot_file) {
                throw std::runtime_error("failed to open snapshot output: " + snapshot_path.string());
            }
            snapshot_file << dedalus::to_json(snapshot);

            manifest << frame_count << " " << snapshot_name << " "
                     << snapshot.timestamp.timestamp_ns << " "
                     << snapshot.active_map_frame_id.value << "\n";
        }

        if (frame_count == 0) {
            std::cerr << "dedalus_replay_recording: no frames processed\n";
            return 1;
        }

        std::cout << "Wrote " << frame_count << " snapshot(s) to " << args.output_dir << "\n";
        std::cout << "Manifest: " << manifest_path << "\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "dedalus_replay_recording: " << ex.what() << "\n";
        return 2;
    }
}
