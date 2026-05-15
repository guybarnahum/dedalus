#include <chrono>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include <unistd.h>

#include "dedalus/behavior/flight_command_sinks.hpp"
#include "dedalus/behavior/latest_world_snapshot.hpp"
#include "dedalus/behavior/mission_runtime.hpp"
#include "dedalus/behavior/trajectory_mission_controller.hpp"
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
    std::string config_path{"config/core_stack_trajectory_mission_placeholder.yaml"};
    std::filesystem::path output_dir{"out/mission_loop_snapshots"};
    int max_frames{0};
    int shutdown_max_frames{300};
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
        std::cerr << "\r\033[Kdedalus_mission_loop: processed " << frame_count;
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

bool mission_finished(dedalus::MissionLifecycleState state) {
    return state == dedalus::MissionLifecycleState::Complete ||
           state == dedalus::MissionLifecycleState::Abort;
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
        } else if (arg == "--shutdown-max-frames") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--shutdown-max-frames requires a value");
            }
            args.shutdown_max_frames = std::stoi(argv[++i]);
            if (args.shutdown_max_frames < 0) {
                throw std::invalid_argument("--shutdown-max-frames must be >= 0");
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

std::unique_ptr<dedalus::MissionController> create_mission_controller(
    const dedalus::CoreStackProviderConfig& config) {
    if (config.mission_controller == "trajectory_mission") {
        return std::make_unique<dedalus::TrajectoryMissionController>(
            dedalus::load_trajectory_mission_config(config.mission_options));
    }
    if (config.mission_controller == "disabled") {
        return nullptr;
    }
    throw std::invalid_argument("unknown mission_controller: " + config.mission_controller);
}

std::unique_ptr<dedalus::FlightCommandSink> create_flight_command_sink(
    const dedalus::CoreStackProviderConfig& config) {
    if (config.flight_command_sink == "disabled") {
        return std::make_unique<dedalus::NullFlightCommandSink>();
    }
    if (config.flight_command_sink == "airsim_velocity") {
        dedalus::AirSimVelocityCommandSinkConfig sink_config;
        sink_config.host = config.source_host;
        sink_config.rpc_port = config.source_rpc_port;
        sink_config.vehicle_name = config.vehicle_name;
        sink_config.command_duration_s = 1.0 / config.mission_tick_hz;
        sink_config.max_velocity_mps = std::stod(config.mission_options.get_or("flight_max_velocity_mps", "5.0"));
        sink_config.bridge_command = config.mission_options.get_or(
            "flight_velocity_command_bridge",
            sink_config.bridge_command);
        sink_config.debug_logging = true;
        return std::make_unique<dedalus::AirSimVelocityCommandSink>(sink_config);
    }
    if (config.flight_command_sink == "px4_mavlink") {
        dedalus::Px4MavlinkCommandSinkConfig sink_config;
        sink_config.command_duration_s = 1.0 / config.mission_tick_hz;
        sink_config.max_velocity_mps = std::stod(config.mission_options.get_or("flight_max_velocity_mps", "5.0"));
        sink_config.endpoints = config.mission_options.get_or(
            "flight_mavlink_command_endpoints",
            sink_config.endpoints);
        sink_config.target_system_id = static_cast<std::uint8_t>(std::stoi(
            config.mission_options.get_or("flight_mavlink_target_system_id", "1")));
        sink_config.target_component_id = static_cast<std::uint8_t>(std::stoi(
            config.mission_options.get_or("flight_mavlink_target_component_id", "1")));
        sink_config.source_system_id = static_cast<std::uint8_t>(std::stoi(
            config.mission_options.get_or("flight_mavlink_source_system_id", "255")));
        sink_config.source_component_id = static_cast<std::uint8_t>(std::stoi(
            config.mission_options.get_or("flight_mavlink_source_component_id", "190")));
        sink_config.takeoff_altitude_m = std::stod(config.mission_options.get_or(
            "flight_safe_height_m",
            "8.0"));
        sink_config.set_offboard_on_velocity = config.mission_options.get_or(
            "flight_mavlink_set_offboard_on_velocity",
            "true") != "false";
        sink_config.debug_logging = true;
        return std::make_unique<dedalus::Px4MavlinkCommandSink>(sink_config);
    }
    throw std::invalid_argument("unknown flight_command_sink: " + config.flight_command_sink);
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const auto args = parse_args(argc, argv);
        std::filesystem::create_directories(args.output_dir);

        const auto config = dedalus::load_core_stack_config(args.config_path);
        dedalus::ProviderRegistry registry;

        std::unique_ptr<dedalus::PipelineProfiler> timing_writer;
        if (config.pipeline_timing_enabled) {
            timing_writer = std::make_unique<dedalus::PipelineProfiler>(config.pipeline_timing_output_path);
        }

        std::cerr << "dedalus_mission_loop: frame_source=" << config.frame_source
                  << " bridge_mode=" << config.bridge_mode
                  << " flight_sink=" << config.flight_command_sink
                  << "\n";
        if (config.frame_source == "airsim") {
            std::cerr << "dedalus_mission_loop: using LIVE AirSim bridge frames; snapshots are debug artifacts, not replay input\n";
        }

        auto latest_snapshot = std::make_shared<dedalus::LatestWorldSnapshot>();
        dedalus::CoreStackRunner runner{
            registry.create(config),
            std::move(timing_writer),
            latest_snapshot};

        std::unique_ptr<dedalus::MissionRuntime> mission_runtime;
        auto controller = create_mission_controller(config);
        if (controller) {
            mission_runtime = std::make_unique<dedalus::MissionRuntime>(
                dedalus::MissionRuntimeConfig{.tick_hz = config.mission_tick_hz, .debug_logging = true},
                latest_snapshot,
                std::move(controller),
                create_flight_command_sink(config));
            mission_runtime->start();
            std::cout << "Mission runtime: " << config.mission_controller
                      << " @ " << config.mission_tick_hz << " Hz"
                      << ", sink=" << config.flight_command_sink << "\n";
        } else {
            std::cout << "Mission runtime: disabled\n";
        }

        const auto manifest_path = args.output_dir / "snapshot_manifest.txt";
        std::ofstream manifest{manifest_path};
        if (!manifest) {
            throw std::runtime_error("failed to open snapshot manifest: " + manifest_path.string());
        }
        manifest << "# index path timestamp_ns active_map_frame_id\n";

        ProgressReporter progress{args.progress_mode, args.max_frames};

        int frame_count = 0;
        int shutdown_frame_count = 0;
        bool finish_requested = false;
        while (true) {
            const bool frame_limit_reached = args.max_frames > 0 && frame_count >= args.max_frames;
            if (frame_limit_reached && mission_runtime && !finish_requested) {
                finish_requested = true;
                mission_runtime->request_finish();
                std::cerr << "dedalus_mission_loop: max frame limit reached; requesting graceful mission finish\n";
            }
            if (frame_limit_reached && !mission_runtime) {
                break;
            }
            if (finish_requested) {
                if (mission_finished(mission_runtime->last_state())) {
                    break;
                }
                if (shutdown_frame_count >= args.shutdown_max_frames) {
                    std::cerr << "dedalus_mission_loop: graceful shutdown frame budget exhausted; stopping at mission state="
                              << dedalus::to_string(mission_runtime->last_state()) << "\n";
                    break;
                }
                ++shutdown_frame_count;
            }

            if (!runner.run_once()) {
                break;
            }

            ++frame_count;
            const auto latest = latest_snapshot->latest();
            const auto snapshot = latest.has_value() ? *latest : runner.snapshot();
            if (frame_count <= 3 || frame_count % 30 == 0) {
                std::cerr << "dedalus_mission_loop: world_snapshot frame=" << frame_count
                          << " ts=" << snapshot.timestamp.timestamp_ns
                          << " ego_height_m=" << snapshot.ego.height_m
                          << " agents=" << snapshot.agents.size()
                          << "\n";
            }
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
        if (mission_runtime) {
            // Let the mission loop observe the final published snapshot before shutdown.
            std::this_thread::sleep_for(std::chrono::milliseconds{100});
            mission_runtime->stop();
            std::cout << "Mission ticks: " << mission_runtime->tick_count() << "\n";
            std::cout << "Mission final state: " << dedalus::to_string(mission_runtime->last_state()) << "\n";
        }

        if (frame_count == 0) {
            std::cerr << "dedalus_mission_loop: no frames processed\n";
            return 1;
        }

        std::cout << "Wrote " << frame_count << " snapshot(s) to " << args.output_dir << "\n";
        std::cout << "Manifest: " << manifest_path << "\n";
        if (finish_requested) {
            std::cout << "Graceful shutdown frames: " << shutdown_frame_count << "\n";
        }
        if (config.pipeline_timing_enabled) {
            std::cout << "Pipeline timing: " << config.pipeline_timing_output_path << "\n";
        }
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "dedalus_mission_loop: " << ex.what() << "\n";
        return 2;
    }
}