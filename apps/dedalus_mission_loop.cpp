#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

#include "dedalus/behavior/flight_command_sinks.hpp"
#include "dedalus/behavior/latest_world_snapshot.hpp"
#include "dedalus/behavior/mission_runtime.hpp"
#include "dedalus/behavior/object_behavior_mission_controller.hpp"
#include "dedalus/behavior/trajectory_mission_controller.hpp"
#include "dedalus/perception/ghost_targets.hpp"
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
    double safe_height_override_m{-1.0};
};

struct CommandCounts {
    int ok{0};
    int failed{0};
};

struct MissionEventSummary {
    bool valid{false};
    int event_count{0};
    int tick_count{0};
    int failure_count{0};
    std::string final_state{"unknown"};
    std::vector<std::string> state_path;
    std::map<std::string, CommandCounts> commands;
    std::vector<std::string> failures;
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

std::string json_string_field(const std::string& line, const std::string& key) {
    const std::string marker = "\"" + key + "\":";
    const auto marker_pos = line.find(marker);
    if (marker_pos == std::string::npos) {
        return {};
    }
    const auto open = line.find('"', marker_pos + marker.size());
    if (open == std::string::npos) {
        return {};
    }
    std::string value;
    bool escaped = false;
    for (std::size_t i = open + 1U; i < line.size(); ++i) {
        const char ch = line[i];
        if (escaped) {
            switch (ch) {
                case 'n':
                    value.push_back('\n');
                    break;
                case 'r':
                    value.push_back('\r');
                    break;
                case 't':
                    value.push_back('\t');
                    break;
                default:
                    value.push_back(ch);
                    break;
            }
            escaped = false;
            continue;
        }
        if (ch == '\\') {
            escaped = true;
            continue;
        }
        if (ch == '"') {
            return value;
        }
        value.push_back(ch);
    }
    return {};
}

int json_int_field(const std::string& line, const std::string& key, int fallback = 0) {
    const std::string marker = "\"" + key + "\":";
    const auto marker_pos = line.find(marker);
    if (marker_pos == std::string::npos) {
        return fallback;
    }
    const auto start = marker_pos + marker.size();
    std::size_t end = start;
    while (end < line.size() && (line[end] == '-' || (line[end] >= '0' && line[end] <= '9'))) {
        ++end;
    }
    if (end == start) {
        return fallback;
    }
    try {
        return std::stoi(line.substr(start, end - start));
    } catch (...) {
        return fallback;
    }
}

bool json_bool_field(const std::string& line, const std::string& key, bool fallback = false) {
    const std::string marker = "\"" + key + "\":";
    const auto marker_pos = line.find(marker);
    if (marker_pos == std::string::npos) {
        return fallback;
    }
    const auto start = marker_pos + marker.size();
    if (line.compare(start, 4U, "true") == 0) {
        return true;
    }
    if (line.compare(start, 5U, "false") == 0) {
        return false;
    }
    return fallback;
}

void append_state_if_new(std::vector<std::string>& states, const std::string& state) {
    if (state.empty()) {
        return;
    }
    if (states.empty() || states.back() != state) {
        states.push_back(state);
    }
}

std::string join_states(const std::vector<std::string>& states) {
    std::ostringstream out;
    for (std::size_t i = 0; i < states.size(); ++i) {
        if (i > 0U) {
            out << " -> ";
        }
        out << states[i];
    }
    return out.str();
}

MissionEventSummary read_mission_event_summary(const std::filesystem::path& path) {
    MissionEventSummary summary;
    std::ifstream input{path};
    if (!input) {
        return summary;
    }

    std::string line;
    while (std::getline(input, line)) {
        if (line.empty()) {
            continue;
        }
        summary.valid = true;
        ++summary.event_count;
        const auto event = json_string_field(line, "event");
        if (event == "state_transition") {
            append_state_if_new(summary.state_path, json_string_field(line, "from"));
            const auto to = json_string_field(line, "to");
            append_state_if_new(summary.state_path, to);
            summary.final_state = to.empty() ? summary.final_state : to;
            summary.tick_count = std::max(summary.tick_count, json_int_field(line, "tick", summary.tick_count));
        } else if (event == "command_result") {
            const auto command = json_string_field(line, "command");
            if (!command.empty()) {
                auto& counts = summary.commands[command];
                if (json_bool_field(line, "success", false)) {
                    ++counts.ok;
                } else {
                    ++counts.failed;
                    ++summary.failure_count;
                    summary.failures.push_back(command + ": " + json_string_field(line, "status"));
                }
            }
            summary.tick_count = std::max(summary.tick_count, json_int_field(line, "tick", summary.tick_count));
        } else if (event == "command_exception") {
            ++summary.failure_count;
            const auto command = json_string_field(line, "command");
            const auto error = json_string_field(line, "error");
            summary.failures.push_back((command.empty() ? std::string{"command"} : command) + ": " + error);
            summary.tick_count = std::max(summary.tick_count, json_int_field(line, "tick", summary.tick_count));
        } else if (event == "runtime_stop") {
            const auto state = json_string_field(line, "state");
            if (!state.empty()) {
                summary.final_state = state;
                append_state_if_new(summary.state_path, state);
            }
            summary.tick_count = std::max(summary.tick_count, json_int_field(line, "tick_count", summary.tick_count));
        } else if (event == "finish_requested") {
            summary.tick_count = std::max(summary.tick_count, json_int_field(line, "tick", summary.tick_count));
        } else if (event == "target_selected" || event == "behavior_start" || event == "behavior_complete") {
            summary.tick_count = std::max(summary.tick_count, json_int_field(line, "tick", summary.tick_count));
        }
    }

    return summary;
}

void print_mission_event_summary(const std::filesystem::path& path) {
    const auto summary = read_mission_event_summary(path);
    if (!summary.valid) {
        std::cout << "Mission summary: unavailable; no event records found\n";
        return;
    }

    std::cout << "Mission summary:\n";
    std::cout << "  final_state: " << summary.final_state << "\n";
    std::cout << "  ticks: " << summary.tick_count << "\n";
    std::cout << "  events: " << summary.event_count << "\n";
    if (!summary.state_path.empty()) {
        std::cout << "  state_path: " << join_states(summary.state_path) << "\n";
    }
    if (!summary.commands.empty()) {
        std::cout << "  commands:\n";
        for (const auto& [command, counts] : summary.commands) {
            std::cout << "    " << command << ": ok=" << counts.ok << " failed=" << counts.failed << "\n";
        }
    }
    std::cout << "  failures: " << summary.failure_count << "\n";
    for (const auto& failure : summary.failures) {
        std::cout << "    - " << failure << "\n";
    }
}

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

bool replace_option_value(std::string& command, const std::string& option, const std::string& value) {
    const auto option_pos = command.find(option);
    if (option_pos == std::string::npos) {
        return false;
    }
    const auto value_start = command.find_first_not_of(" \t", option_pos + option.size());
    if (value_start == std::string::npos) {
        command += " " + value;
        return true;
    }
    const auto value_end = command.find_first_of(" \t", value_start);
    command.replace(value_start, value_end == std::string::npos ? std::string::npos : value_end - value_start, value);
    return true;
}

void apply_safe_height_override(dedalus::CoreStackProviderConfig& config, double safe_height_m) {
    if (safe_height_m <= 0.0) {
        throw std::invalid_argument("--safe-height must be > 0");
    }
    const auto value = format_double_for_cli(safe_height_m);
    config.mission_options.values["flight_safe_height_m"] = value;

    auto bridge_it = config.mission_options.values.find("flight_px4_command_bridge");
    if (bridge_it != config.mission_options.values.end() && !bridge_it->second.empty()) {
        auto& command = bridge_it->second;
        if (!replace_option_value(command, "--safe-height", value) &&
            !replace_option_value(command, "--safe-height-m", value)) {
            command += " --safe-height " + value;
        }
    }
}

void apply_cli_overrides(dedalus::CoreStackProviderConfig& config, const Args& args) {
    if (args.safe_height_override_m > 0.0) {
        apply_safe_height_override(config, args.safe_height_override_m);
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
        } else {
            const int parsed_verbosity = verbosity_from_flag(arg);
            if (parsed_verbosity >= 0) {
                args.verbosity = std::max(args.verbosity, parsed_verbosity);
            } else {
                throw std::invalid_argument("unknown argument: " + arg);
            }
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
        sink_config.debug_logging = sink_debug_logging;
        return std::make_unique<dedalus::AirSimVelocityCommandSink>(sink_config);
    }
    if (config.flight_command_sink == "px4_bridge") {
        dedalus::Px4BridgeCommandSinkConfig sink_config;
        sink_config.command_duration_s = 1.0 / config.mission_tick_hz;
        sink_config.max_velocity_mps = std::stod(config.mission_options.get_or("flight_max_velocity_mps", "5.0"));
        sink_config.bridge_command = config.mission_options.get_or(
            "flight_px4_command_bridge",
            sink_config.bridge_command);
        sink_config.verbosity = verbosity;
        sink_config.debug_logging = sink_debug_logging;
        return std::make_unique<dedalus::Px4BridgeCommandSink>(sink_config);
    }
    if (config.flight_command_sink == "px4_mavlink") {
        dedalus::Px4MavlinkCommandSinkConfig sink_config;
        sink_config.command_duration_s = 1.0 / config.mission_tick_hz;
        sink_config.max_velocity_mps = std::stod(config.mission_options.get_or("flight_max_velocity_mps", "5.0"));
        sink_config.endpoints = config.mission_options.get_or(
            "flight_mavlink_command_endpoints",
            sink_config.endpoints);
        sink_config.px4_tmux_target = config.mission_options.get_or(
            "flight_px4_tmux_target",
            sink_config.px4_tmux_target);
        sink_config.use_px4_shell_lifecycle = config.mission_options.get_or(
            "flight_use_px4_shell_lifecycle",
            "true") != "false";
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
        sink_config.debug_logging = sink_debug_logging;
        return std::make_unique<dedalus::Px4MavlinkCommandSink>(sink_config);
    }
    throw std::invalid_argument("unknown flight_command_sink: " + config.flight_command_sink);
}

}  // namespace

int main(int argc, char** argv) {
    try {
        install_interrupt_handlers();
        const auto args = parse_args(argc, argv);
        std::filesystem::create_directories(args.output_dir);

        auto config = dedalus::load_core_stack_config(args.config_path);
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
        std::cerr << "\n";
        if (config.frame_source == "airsim" && args.verbosity >= 1) {
            std::cerr << "dedalus_mission_loop: using LIVE AirSim bridge frames; snapshots are debug artifacts, not replay input\n";
        }

        const auto prepare_command = config.mission_options.get_or("flight_prepare_session_command", "");
        if (run_shell_command(prepare_command, args.verbosity) != 0) {
            throw std::runtime_error("flight prepare session command failed");
        }

        auto latest_snapshot = std::make_shared<dedalus::LatestWorldSnapshot>();
        auto snapshot_publisher = std::make_shared<dedalus::WorldSnapshotPublisher>();
        auto ghost_detections_publisher = std::make_shared<dedalus::GhostDetectionsPublisher>();
        auto mission_event_publisher = std::make_shared<dedalus::MissionEventPublisher>();
        auto latest_snapshot_subscriber = std::make_shared<dedalus::LatestWorldSnapshotSubscriber>(latest_snapshot);
        auto artifact_snapshot_writer = std::make_shared<dedalus::ArtifactSnapshotWriter>(args.output_dir);
        snapshot_publisher->subscribe(latest_snapshot_subscriber);
        snapshot_publisher->subscribe(artifact_snapshot_writer);

        std::shared_ptr<dedalus::RuntimeEventStreamServer> runtime_event_stream_server;
        if (args.world_snapshot_stream_port > 0) {
            runtime_event_stream_server = std::make_shared<dedalus::RuntimeEventStreamServer>(
                dedalus::RuntimeEventStreamServerConfig{
                    .bind_host = args.world_snapshot_stream_host,
                    .port = static_cast<std::uint16_t>(args.world_snapshot_stream_port)});
            runtime_event_stream_server->start();
            snapshot_publisher->subscribe(runtime_event_stream_server);
            ghost_detections_publisher->subscribe(runtime_event_stream_server);
            mission_event_publisher->subscribe(runtime_event_stream_server);
            std::cerr << "dedalus_mission_loop: runtime event stream listening on "
                      << args.world_snapshot_stream_host << ":" << runtime_event_stream_server->port() << "\n";
        }

        dedalus::CoreStackRunner runner{
            registry.create(config),
            std::move(timing_writer),
            snapshot_publisher,
            ghost_detections_publisher};

        const auto mission_events_path = args.output_dir / "mission_events.jsonl";
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
                mission_event_publisher);
            mission_runtime->start();
            std::cout << "Mission runtime: " << config.mission_controller
                      << " @ " << config.mission_tick_hz << " Hz"
                      << ", sink=" << config.flight_command_sink << "\n";
        } else {
            std::cout << "Mission runtime: disabled\n";
        }

        ProgressReporter progress{args.progress_mode, args.max_frames};

        int shutdown_frame_count = 0;
        bool finish_requested = false;
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

            const int updated_frame_count = artifact_snapshot_writer->frame_count();
            if (args.verbosity >= 2 && (updated_frame_count <= 3 || updated_frame_count % 30 == 0)) {
                const auto latest = latest_snapshot->latest();
                if (latest.has_value()) {
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
            print_mission_event_summary(mission_events_path);
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

        std::cout << "Wrote " << frame_count << " snapshot(s) to " << artifact_snapshot_writer->output_dir() << "\n";
        std::cout << "Manifest: " << artifact_snapshot_writer->manifest_path() << "\n";
        if (mission_runtime) {
            std::cout << "Mission events: " << mission_events_path << "\n";
        }
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
