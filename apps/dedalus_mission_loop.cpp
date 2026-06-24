#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include <unistd.h>

#include "dedalus/behavior/flight_command_sinks.hpp"
#include "dedalus/behavior/latest_world_snapshot.hpp"
#include "dedalus/behavior/mission_runtime.hpp"
#include "dedalus/behavior/object_behavior_mission_controller.hpp"
#include "dedalus/behavior/trajectory_mission_controller.hpp"
#include "dedalus/perception/ghost_targets.hpp"
#include "dedalus/runtime/camera_pointing_state_store.hpp"
#include "dedalus/runtime/config_loader.hpp"
#include "dedalus/runtime/core_stack_runner.hpp"
#include "dedalus/runtime/pipeline_profiler.hpp"
#include "dedalus/runtime/provider_registry.hpp"
#include "dedalus/runtime/world_snapshot_stream_server.hpp"
#include "dedalus/world_model/world_snapshot.hpp"
#include "dedalus/world_model/world_snapshot_publisher.hpp"
#include "dedalus/world_model/world_snapshot_subscribers.hpp"

namespace {

volatile std::sig_atomic_t g_interrupt_count = 0;

void handle_interrupt_signal(int) {
    const auto current = g_interrupt_count;
    if (current < 2) {
        g_interrupt_count = static_cast<std::sig_atomic_t>(current + 1);
    }
}

void install_interrupt_handlers() {
    std::signal(SIGINT, handle_interrupt_signal);
    std::signal(SIGTERM, handle_interrupt_signal);
}

int interrupt_count() {
    return static_cast<int>(g_interrupt_count);
}

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
    int verbosity{0};
    std::string world_snapshot_stream_host{"127.0.0.1"};
    int world_snapshot_stream_port{0};
    std::string runtime_event_http_host{"127.0.0.1"};
    int runtime_event_http_port{0};
    std::string runtime_event_static_root{};
    double safe_height_override_m{-1.0};
    double behavior_min_height_override_m{-1.0};
    double behavior_duration_override_s{-1.0};
    bool safe_height_override_explicit{false};
    // L2 persistent map path — derived from DEDALUS_SITE_ID if not empty.
    std::string planning_map_persistence_path;
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

int run_shell_command(const std::string& command, int verbosity) {
    if (command.empty() || command == "disabled") {
        return 0;
    }
    if (verbosity >= 1) {
        std::cerr << "dedalus_mission_loop: prepare_session command=" << command << "\n";
    }
    if (verbosity <= 0) {
        const std::string quiet_command = "(" + command + ") >/dev/null 2>&1";
        return std::system(quiet_command.c_str());
    }
    return std::system(command.c_str());
}

std::string format_double_for_cli(double value) {
    std::ostringstream out;
    out << std::setprecision(12) << value;
    return out.str();
}

void apply_takeoff_height_override(dedalus::CoreStackProviderConfig& config, double safe_height_m) {
    if (safe_height_m <= 0.0) {
        throw std::invalid_argument("--safe-height must be > 0");
    }
    config.mission_options.safe_height_m = safe_height_m;
}

void apply_behavior_min_height_override(dedalus::CoreStackProviderConfig& config, double min_height_m) {
    if (min_height_m <= 0.0) {
        throw std::invalid_argument("--behavior-min-height must be > 0");
    }
    config.mission_options.behavior_min_height_m = min_height_m;
}

void apply_behavior_duration_override(dedalus::CoreStackProviderConfig& config, double duration_s) {
    if (duration_s <= 0.0) {
        throw std::invalid_argument("--behavior-duration-s must be > 0");
    }
    config.mission_options.completion_after_s = duration_s;
}

void apply_cli_overrides(dedalus::CoreStackProviderConfig& config, const Args& args) {
    if (args.safe_height_override_explicit && args.safe_height_override_m > 0.0) {
        apply_takeoff_height_override(config, args.safe_height_override_m);
    }
    if (args.behavior_min_height_override_m > 0.0) {
        apply_behavior_min_height_override(config, args.behavior_min_height_override_m);
    }
    if (args.behavior_duration_override_s > 0.0) {
        apply_behavior_duration_override(config, args.behavior_duration_override_s);
    }
}

int verbosity_from_flag(const std::string& arg) {
    if (arg == "--verbose") {
        return 3;
    }
    if (arg.size() >= 2U && arg[0] == '-' && arg[1] == 'v') {
        return static_cast<int>(std::min<std::size_t>(3U, arg.size() - 1U));
    }
    return -1;
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
        } else if (arg == "--safe-height") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--safe-height requires a value in meters");
            }
            args.safe_height_override_m = std::stod(argv[++i]);
            if (args.safe_height_override_m <= 0.0) {
                throw std::invalid_argument("--safe-height must be > 0");
            }
            args.safe_height_override_explicit = true;
        } else if (arg == "--behavior-duration-s") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--behavior-duration-s requires a value in seconds");
            }
            args.behavior_duration_override_s = std::stod(argv[++i]);
            if (args.behavior_duration_override_s <= 0.0) {
                throw std::invalid_argument("--behavior-duration-s must be > 0");
            }
        } else if (arg == "--behavior-min-height") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--behavior-min-height requires a value in meters");
            }
            args.behavior_min_height_override_m = std::stod(argv[++i]);
            if (args.behavior_min_height_override_m <= 0.0) {
                throw std::invalid_argument("--behavior-min-height must be > 0");
            }
        } else if (arg == "--progress") {
            args.progress_mode = ProgressMode::On;
        } else if (arg == "--no-progress") {
            args.progress_mode = ProgressMode::Off;
        } else if (arg == "--world-snapshot-stream-host") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--world-snapshot-stream-host requires a value");
            }
            args.world_snapshot_stream_host = argv[++i];
        } else if (arg == "--world-snapshot-stream-port") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--world-snapshot-stream-port requires a value");
            }
            args.world_snapshot_stream_port = std::stoi(argv[++i]);
            if (args.world_snapshot_stream_port < 0 || args.world_snapshot_stream_port > 65535) {
                throw std::invalid_argument("--world-snapshot-stream-port must be in [0, 65535]");
            }
        } else if (arg == "--runtime-event-http-host") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--runtime-event-http-host requires a value");
            }
            args.runtime_event_http_host = argv[++i];
        } else if (arg == "--runtime-event-http-port") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--runtime-event-http-port requires a value");
            }
            args.runtime_event_http_port = std::stoi(argv[++i]);
            if (args.runtime_event_http_port < 0 || args.runtime_event_http_port > 65535) {
                throw std::invalid_argument("--runtime-event-http-port must be in [0, 65535]");
            }
        } else if (arg == "--runtime-event-static-root") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--runtime-event-static-root requires a value");
            }
            args.runtime_event_static_root = argv[++i];
        } else {
            const int parsed_verbosity = verbosity_from_flag(arg);
            if (parsed_verbosity >= 0) {
                args.verbosity = std::max(args.verbosity, parsed_verbosity);
            } else {
                throw std::invalid_argument("unknown argument: " + arg);
            }
        }
    }
    // Derive L2 persistence path from DEDALUS_SITE_ID if not set by any other means.
    if (args.planning_map_persistence_path.empty()) {
        if (const char* site_id = std::getenv("DEDALUS_SITE_ID")) {
            args.planning_map_persistence_path = std::string{"maps/"} + site_id + "/l2_map.db";
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
    if (config.mission_controller == "object_behavior") {
        return std::make_unique<dedalus::ObjectBehaviorMissionController>(
            dedalus::load_object_behavior_mission_config(config.mission_options));
    }
    if (config.mission_controller == "disabled") {
        return nullptr;
    }
    throw std::invalid_argument("unknown mission_controller: " + config.mission_controller);
}

std::unique_ptr<dedalus::FlightCommandSink> create_flight_command_sink(
    const dedalus::CoreStackProviderConfig& config,
    int verbosity) {
    const bool sink_debug_logging = verbosity >= 3;
    const auto& opts = config.mission_options;
    if (config.flight_command_sink == "disabled") {
        return std::make_unique<dedalus::NullFlightCommandSink>();
    }
    if (config.flight_command_sink == "airsim_velocity") {
        dedalus::AirSimVelocityCommandSinkConfig sink_config;
        sink_config.host = config.source_host;
        sink_config.rpc_port = config.source_rpc_port;
        sink_config.vehicle_name = config.vehicle_name;
        sink_config.command_duration_s = 1.0 / config.mission_tick_hz;
        sink_config.max_velocity_mps = opts.max_velocity_mps;
        if (!opts.velocity_command_bridge.empty()) {
            sink_config.bridge_command = opts.velocity_command_bridge;
        }
        sink_config.debug_logging = sink_debug_logging;
        return std::make_unique<dedalus::AirSimVelocityCommandSink>(sink_config);
    }
    if (config.flight_command_sink == "px4_bridge") {
        dedalus::Px4BridgeCommandSinkConfig sink_config;
        sink_config.command_duration_s = 1.0 / config.mission_tick_hz;
        sink_config.max_velocity_mps = opts.max_velocity_mps;
        if (!opts.px4_command_bridge.empty()) {
            sink_config.bridge_command = opts.px4_command_bridge;
        }
        // flight_safe_height_m is the canonical safe-height value.  Append it
        // as --safe-height so a CLI override propagates to the bridge script
        // without string surgery; argparse takes the last occurrence.
        sink_config.bridge_command += " --safe-height " + format_double_for_cli(opts.safe_height_m);
        sink_config.verbosity = verbosity;
        sink_config.debug_logging = sink_debug_logging;
        return std::make_unique<dedalus::Px4BridgeCommandSink>(sink_config);
    }
    if (config.flight_command_sink == "px4_mavlink") {
        dedalus::Px4MavlinkCommandSinkConfig sink_config;
        sink_config.command_duration_s = 1.0 / config.mission_tick_hz;
        sink_config.max_velocity_mps      = opts.max_velocity_mps;
        sink_config.endpoints             = opts.mavlink_command_endpoints.empty()
                                              ? sink_config.endpoints
                                              : opts.mavlink_command_endpoints;
        sink_config.px4_tmux_target       = opts.px4_tmux_target.empty()
                                              ? sink_config.px4_tmux_target
                                              : opts.px4_tmux_target;
        sink_config.use_px4_shell_lifecycle = opts.use_px4_shell_lifecycle;
        sink_config.target_system_id      = opts.mavlink_target_system_id;
        sink_config.target_component_id   = opts.mavlink_target_component_id;
        sink_config.source_system_id      = opts.mavlink_source_system_id;
        sink_config.source_component_id   = opts.mavlink_source_component_id;
        sink_config.takeoff_altitude_m    = opts.safe_height_m;
        sink_config.set_offboard_on_velocity = opts.set_offboard_on_velocity;
        sink_config.debug_logging = sink_debug_logging;
        return std::make_unique<dedalus::Px4MavlinkCommandSink>(sink_config);
    }
    throw std::invalid_argument("unknown flight_command_sink: " + config.flight_command_sink);
}

std::unique_ptr<dedalus::CameraPointingSink> create_camera_pointing_sink(
    const dedalus::CoreStackProviderConfig& config,
    int verbosity) {
    const auto& opts = config.mission_options;
    const auto& sink = opts.camera_pointing_sink;
    if (sink == "null" || sink == "disabled" || sink == "runtime_stream") {
        return nullptr;
    }
    if (sink == "mavlink_gimbal") {
        dedalus::MavlinkGimbalPointingSinkConfig sink_config;
        if (!opts.camera_pointing_mavlink_endpoints.empty()) {
            sink_config.endpoints = opts.camera_pointing_mavlink_endpoints;
        }
        sink_config.source_system_id    = opts.camera_pointing_mavlink_source_system_id;
        sink_config.source_component_id = opts.camera_pointing_mavlink_source_component_id;
        sink_config.target_system_id    = opts.camera_pointing_mavlink_target_system_id;
        sink_config.target_component_id = opts.camera_pointing_mavlink_target_component_id;
        sink_config.gimbal_device_id    = opts.camera_pointing_mavlink_gimbal_device_id;
        sink_config.gimbal_manager_flags = opts.camera_pointing_mavlink_flags;
        sink_config.deadband_rad        = opts.camera_pointing_deadband_rad;
        sink_config.resend_interval_s   = opts.camera_pointing_resend_s;
        sink_config.debug_logging = verbosity >= 2;
        return std::make_unique<dedalus::MavlinkGimbalPointingSink>(sink_config);
    }
    throw std::invalid_argument("unknown object_behavior_camera_pointing_sink: " + sink);
}

void wait_for_graceful_runtime_settle(
    dedalus::MissionRuntime& runtime,
    double tick_hz,
    int max_ticks,
    int& consumed_ticks) {
    consumed_ticks = 0;
    const auto sleep_ms = std::chrono::milliseconds{
        std::max<int>(1, static_cast<int>(1000.0 / std::max(1.0, tick_hz)))};
    while (!runtime.terminal_settled() && consumed_ticks < max_ticks && interrupt_count() < 2) {
        std::this_thread::sleep_for(sleep_ms);
        ++consumed_ticks;
    }
}

}  // namespace

int main(int argc, char** argv) {
    try {
        install_interrupt_handlers();
        const auto args = parse_args(argc, argv);
        std::filesystem::create_directories(args.output_dir);

        auto app_config = dedalus::load_core_stack_app_config(args.config_path);
        auto& config = app_config.providers;
        apply_cli_overrides(config, args);
        dedalus::ProviderRegistry registry;

        std::unique_ptr<dedalus::PipelineProfiler> timing_writer;
        if (config.pipeline_timing_enabled) {
            timing_writer = std::make_unique<dedalus::PipelineProfiler>(config.pipeline_timing_output_path);
        }

        std::cerr << "dedalus_mission_loop: frame_source=" << config.frame_source
                  << " bridge_mode=" << config.bridge_mode
                  << " flight_sink=" << config.flight_command_sink
                  << " verbosity=" << args.verbosity;
        if (args.safe_height_override_m > 0.0) {
            std::cerr << " safe_height_override_m=" << args.safe_height_override_m;
        }
        if (args.behavior_duration_override_s > 0.0) {
            std::cerr << " behavior_duration_override_s=" << args.behavior_duration_override_s;
        }
        std::cerr << "\n";
        if (config.frame_source == "airsim" && args.verbosity >= 1) {
            std::cerr << "dedalus_mission_loop: using LIVE AirSim bridge frames; snapshots are debug artifacts, not replay input\n";
        }

        const auto prepare_command = config.mission_options.prepare_session_command;
        if (run_shell_command(prepare_command, args.verbosity) != 0) {
            throw std::runtime_error("flight prepare session command failed");
        }

        auto latest_snapshot = std::make_shared<dedalus::LatestWorldSnapshot>();
        auto snapshot_publisher = std::make_shared<dedalus::WorldSnapshotPublisher>();
        auto ghost_detections_publisher = std::make_shared<dedalus::GhostDetectionsPublisher>();
        auto mission_event_publisher = std::make_shared<dedalus::MissionEventPublisher>();
        auto mission_obstacle_map_delta_publisher = std::make_shared<dedalus::MissionObstacleMapDeltaPublisher>();
        auto traversability_map_publisher = std::make_shared<dedalus::MissionLocalTraversabilityMapPublisher>();
        auto planning_map_publisher = std::make_shared<dedalus::MissionLocalPlanningMapPublisher>();
        auto esdf_map_publisher = std::make_shared<dedalus::LocalESDFMapPublisher>();
        auto latest_snapshot_subscriber = std::make_shared<dedalus::LatestWorldSnapshotSubscriber>(latest_snapshot);
        auto artifact_snapshot_writer = std::make_shared<dedalus::ArtifactSnapshotWriter>(
            dedalus::ArtifactSnapshotWriterConfig{.output_dir = args.output_dir});

        std::shared_ptr<dedalus::RuntimeEventStreamServer> runtime_event_stream_server;
        if (args.world_snapshot_stream_port > 0) {
            runtime_event_stream_server = std::make_shared<dedalus::RuntimeEventStreamServer>(
                dedalus::RuntimeEventStreamServerConfig{
                    .bind_host = args.world_snapshot_stream_host,
                    .port = static_cast<std::uint16_t>(args.world_snapshot_stream_port),
                    .http_bind_host = args.runtime_event_http_host,
                    .http_port = static_cast<std::uint16_t>(args.runtime_event_http_port),
                    .http_static_root = args.runtime_event_static_root});
            runtime_event_stream_server->start();
            snapshot_publisher->subscribe(runtime_event_stream_server);
            ghost_detections_publisher->subscribe(runtime_event_stream_server);
            mission_event_publisher->subscribe(runtime_event_stream_server);
            mission_obstacle_map_delta_publisher->subscribe(runtime_event_stream_server);
            traversability_map_publisher->subscribe(runtime_event_stream_server);
            planning_map_publisher->subscribe(runtime_event_stream_server);
            esdf_map_publisher->subscribe(runtime_event_stream_server);
            std::cerr << "dedalus_mission_loop: runtime event stream listening on "
                      << args.world_snapshot_stream_host << ":" << runtime_event_stream_server->port() << "\n";
            if (runtime_event_stream_server->http_port() > 0) {
                std::cerr << "dedalus_mission_loop: runtime event HTTP/SSE listening on "
                          << args.runtime_event_http_host << ":" << runtime_event_stream_server->http_port() << "\n";
            }
        }

        app_config.runner.timing_writer = std::move(timing_writer);
        app_config.runner.snapshot_publisher = snapshot_publisher;
        app_config.runner.ghost_detections_publisher = ghost_detections_publisher;
        app_config.runner.mission_obstacle_map_delta_publisher = mission_obstacle_map_delta_publisher;
        app_config.runner.traversability_map_publisher = traversability_map_publisher;
        app_config.runner.planning_map_publisher = planning_map_publisher;
        app_config.runner.esdf_map_publisher = esdf_map_publisher;
        app_config.runner.snapshot_subscribers = {latest_snapshot_subscriber, artifact_snapshot_writer};
        if (!args.planning_map_persistence_path.empty()) {
            std::filesystem::create_directories(
                std::filesystem::path{args.planning_map_persistence_path}.parent_path());
            app_config.runner.planning_map_persistence_path = args.planning_map_persistence_path;
        }

        dedalus::CoreStackRunner runner{
            registry.create(config),
            std::move(app_config.runner)};

        const auto mission_events_path = args.output_dir / "mission_events.jsonl";
        dedalus::CameraPointingStateStore camera_pointing_state_store;
        std::vector<dedalus::CameraPointingState> latest_camera_pointing_states;
        std::mutex camera_pointing_state_mutex;

        auto camera_pointing_handoff = [&](const dedalus::CameraPointingCommand& command) {
            std::lock_guard<std::mutex> lock(camera_pointing_state_mutex);
            camera_pointing_state_store.apply(command);
            latest_camera_pointing_states = camera_pointing_state_store.states();
        };

        std::unique_ptr<dedalus::MissionRuntime> mission_runtime;
        auto controller = create_mission_controller(config);
        if (controller) {
            mission_runtime = std::make_unique<dedalus::MissionRuntime>(
                dedalus::MissionRuntimeConfig{
                    .tick_hz = config.mission_tick_hz,
                    .verbosity = args.verbosity,
                    .event_log_path = mission_events_path.string()},
                latest_snapshot,
                std::move(controller),
                create_flight_command_sink(config, args.verbosity),
                mission_event_publisher,
                create_camera_pointing_sink(config, args.verbosity),
                camera_pointing_handoff);
            mission_runtime->start();
            std::cout << "Mission runtime: " << config.mission_controller
                      << " @ " << config.mission_tick_hz << " Hz"
                      << ", sink=" << config.flight_command_sink << "\n";
        } else {
            std::cout << "Mission runtime: disabled\n";
        }

        ProgressReporter progress{args.progress_mode, args.max_frames};

        int shutdown_wait_ticks = 0;
        bool finish_requested = false;
        bool frame_source_ended = false;
        while (true) {
            const int signals = interrupt_count();
            if (signals >= 2) {
                std::cerr << "\ndedalus_mission_loop: second interrupt received; stopping after local cleanup\n";
                break;
            }
            if (signals >= 1 && !finish_requested) {
                finish_requested = true;
                if (mission_runtime) {
                    mission_runtime->request_finish();
                }
                std::cerr << "\ndedalus_mission_loop: interrupt received; requesting graceful mission finish"
                          << " (press Ctrl-C again to force local stop)\n";
            }

            if (!finish_requested && mission_runtime && mission_runtime->terminal_settled()) {
                std::cerr << "dedalus_mission_loop: mission terminal state settled="
                          << dedalus::to_string(mission_runtime->last_state()) << "; stopping frame loop\n";
                break;
            }

            const int frame_count = artifact_snapshot_writer->frame_count();
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
                if (!mission_runtime || mission_runtime->terminal_settled()) {
                    break;
                }
                if (shutdown_wait_ticks >= args.shutdown_max_frames) {
                    std::cerr << "dedalus_mission_loop: graceful shutdown frame budget exhausted; stopping at mission state="
                              << dedalus::to_string(mission_runtime->last_state()) << "\n";
                    break;
                }
                ++shutdown_wait_ticks;
            }

            {
                std::lock_guard<std::mutex> lock(camera_pointing_state_mutex);
                runner.update_camera_pointing_states(latest_camera_pointing_states);
            }

            if (!runner.run_once()) {
                frame_source_ended = true;
                const int frames = artifact_snapshot_writer->frame_count();
                std::cerr << "dedalus_mission_loop: frame source ended after " << frames << " frame(s)";
                if (mission_runtime && !mission_runtime->terminal_settled()) {
                    std::cerr << "; requesting graceful mission finish before runtime_stop\n";
                    if (!finish_requested) {
                        finish_requested = true;
                        mission_runtime->request_finish();
                    }
                    int consumed = 0;
                    wait_for_graceful_runtime_settle(
                        *mission_runtime,
                        config.mission_tick_hz,
                        args.shutdown_max_frames,
                        consumed);
                    shutdown_wait_ticks += consumed;
                    if (!mission_runtime->terminal_settled()) {
                        std::cerr << "dedalus_mission_loop: frame source ended; graceful shutdown wait exhausted at mission state="
                                  << dedalus::to_string(mission_runtime->last_state()) << "\n";
                    }
                } else {
                    std::cerr << "\n";
                }
                break;
            }

            const int updated_frame_count = artifact_snapshot_writer->frame_count();
            if (args.verbosity >= 2 && (updated_frame_count <= 3 || updated_frame_count % 30 == 0)) {
                const auto latest = latest_snapshot->latest();
                if (latest) {
                    std::cerr << "dedalus_mission_loop: world_snapshot frame=" << updated_frame_count
                              << " ts=" << latest->timestamp.timestamp_ns
                              << " ego_height_m=" << latest->ego.height_m
                              << " agents=" << latest->agents.size()
                              << "\n";
                }
            }
            progress.update(updated_frame_count);
        }

        const int frame_count = artifact_snapshot_writer->frame_count();
        progress.finish(frame_count);
        if (mission_runtime) {
            std::this_thread::sleep_for(std::chrono::milliseconds{100});
            mission_runtime->stop();
            std::cout << "Mission events: " << mission_events_path << "\n";
        }

        if (runtime_event_stream_server) {
            const auto stats = runtime_event_stream_server->stats();
            std::cerr << "dedalus_mission_loop: runtime event stream stats published_seq=" << stats.published_seq
                      << " accepted_clients=" << stats.accepted_clients
                      << " connected_clients=" << stats.connected_clients
                      << " dropped_clients=" << stats.dropped_clients << "\n";
            runtime_event_stream_server->stop();
        }

        if (frame_count == 0) {
            std::cerr << "dedalus_mission_loop: no frames processed\n";
            return 1;
        }

        std::cout << "Mission summary: snapshots=" << frame_count
                  << " final_state=" << (mission_runtime ? dedalus::to_string(mission_runtime->last_state()) : "disabled")
                  << " graceful_shutdown_ticks=" << shutdown_wait_ticks
                  << " frame_source_ended=" << (frame_source_ended ? "true" : "false") << "\n";
        std::cout << "Wrote " << frame_count << " snapshot(s) to " << artifact_snapshot_writer->output_dir() << "\n";
        std::cout << "Manifest: " << artifact_snapshot_writer->manifest_path() << "\n";
        if (finish_requested) {
            std::cout << "Graceful shutdown ticks: " << shutdown_wait_ticks << "\n";
        }
        if (frame_source_ended) {
            std::cout << "Frame source ended before max frame limit\n";
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
