#include <chrono>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include <unistd.h>

#include "dedalus/runtime/config_loader.hpp"
#include "dedalus/runtime/core_stack_runner.hpp"
#include "dedalus/runtime/pipeline_profiler.hpp"
#include "dedalus/runtime/provider_registry.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

namespace {

enum class ProgressMode {
    Auto,
    On,
    Off,
};

struct Args {
    std::string config_path{"config/ci/core_stack_recorded_ci.yaml"};
    std::filesystem::path output_dir{"out/replay_snapshots"};
    int max_frames{0};
    ProgressMode progress_mode{ProgressMode::Auto};
};

class ProgressReporter {
public:
    ProgressReporter(ProgressMode mode, int max_frames)
        : enabled_(should_enable(mode)), max_frames_(max_frames), last_emit_(Clock::now()) {
        if (enabled_) {
            emit(0, false);
        }
    }

    ~ProgressReporter() {
        clear_line_if_needed();
    }

    void update(int frame_count) {
        if (!enabled_) {
            return;
        }
        const auto now = Clock::now();
        if (now - last_emit_ < std::chrono::seconds{1} &&
            !(max_frames_ > 0 && frame_count >= max_frames_)) {
            return;
        }
        last_emit_ = now;
        emit(frame_count, false);
    }

    void finish(int frame_count) {
        if (!enabled_) {
            return;
        }
        emit(frame_count, true);
        std::cerr << '\n';
        finished_ = true;
    }

private:
    using Clock = std::chrono::steady_clock;

    static bool should_enable(ProgressMode mode) {
        if (mode == ProgressMode::On) {
            return true;
        }
        if (mode == ProgressMode::Off) {
            return false;
        }
        return isatty(STDERR_FILENO) != 0;
    }

    void clear_line_if_needed() {
        if (enabled_ && !finished_) {
            std::cerr << "\r\033[K";
        }
    }

    void emit(int frame_count, bool done) const {
        std::cerr << "\r\033[Kdedalus_replay_recording: processed " << frame_count;
        if (max_frames_ > 0) {
            const int percent = frame_count >= max_frames_ ? 100 : (frame_count * 100 / max_frames_);
            std::cerr << "/" << max_frames_ << " frame(s) (" << percent << "%)";
        } else {
            std::cerr << " frame(s)";
        }
        if (done) {
            std::cerr << " done";
        }
        std::cerr.flush();
    }

    bool enabled_{false};
    int max_frames_{0};
    Clock::time_point last_emit_;
    bool finished_{false};
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
        } else if (arg == "--progress") {
            args.progress_mode = ProgressMode::On;
        } else if (arg == "--no-progress") {
            args.progress_mode = ProgressMode::Off;
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

        const auto app_config = dedalus::load_core_stack_app_config(args.config_path);
        auto& config = app_config.providers;
        dedalus::ProviderRegistry registry;

        std::unique_ptr<dedalus::PipelineProfiler> timing_writer;
        if (config.pipeline_timing_enabled) {
            timing_writer = std::make_unique<dedalus::PipelineProfiler>(config.pipeline_timing_output_path);
        }

        dedalus::CoreStackRunner runner{
            registry.create(config),
            dedalus::CoreStackRunnerConfig{.timing_writer = std::move(timing_writer)}};

        const auto manifest_path = args.output_dir / "snapshot_manifest.txt";
        std::ofstream manifest{manifest_path};
        if (!manifest) {
            throw std::runtime_error("failed to open snapshot manifest: " + manifest_path.string());
        }
        manifest << "# index path timestamp_ns active_map_frame_id\n";

        ProgressReporter progress{args.progress_mode, args.max_frames};

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
            progress.update(frame_count);
        }

        progress.finish(frame_count);

        if (frame_count == 0) {
            std::cerr << "dedalus_replay_recording: no frames processed\n";
            return 1;
        }

        std::cout << "Wrote " << frame_count << " snapshot(s) to " << args.output_dir << "\n";
        std::cout << "Manifest: " << manifest_path << "\n";
        if (config.pipeline_timing_enabled) {
            std::cout << "Pipeline timing: " << config.pipeline_timing_output_path << "\n";
        }
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "dedalus_replay_recording: " << ex.what() << "\n";
        return 2;
    }
}
