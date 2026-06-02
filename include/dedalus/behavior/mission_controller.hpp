#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "dedalus/core/types.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

namespace dedalus {

// Typed configuration for the mission runtime, controllers, and flight sinks.
// Each field maps one-to-one to a YAML key of the form mission_options.<name>.
// All parsing and range checks happen once in config_loader; consumers read
// typed fields directly.
struct MissionOptions {
    // ── behavior spec ─────────────────────────────────────────────────────────
    std::string behavior_spec_path;            // required for object_behavior controller

    // ── follow / approach behavior ────────────────────────────────────────────
    bool follow_observation_geometry_enabled{false};
    bool zero_target_velocity{false};
    double follow_min_standoff_m{8.0};
    double follow_max_elevation_angle_deg{35.0};
    double follow_arrival_slow_radius_m{8.0};
    double follow_arrival_hold_radius_m{2.0};
    double follow_arrival_kp{0.35};
    std::optional<double> completion_after_s;  // overrides behavior spec when set

    // ── object behavior core ──────────────────────────────────────────────────
    double hold_velocity_mps{0.0};
    std::string yaw_mode{"trajectory"};        // trajectory|target|hold|none
    double yaw_min_speed_mps{0.35};
    bool yaw_hold_last_when_unstable{true};
    // object_behavior_yaw_offset_rad takes precedence over yaw_offset_rad when set
    std::optional<double> object_behavior_yaw_offset_rad;
    std::string vertical_stare_mode{"none"};   // none|gimbal
    bool vertical_stare_warn_if_unavailable{true};
    int debug_every_n_ticks{0};
    int debug_level{1};
    std::string altitude_policy{"target_relative"};  // target_relative|safe_height_floor

    // ── camera pointing ───────────────────────────────────────────────────────
    std::string camera_pointing_cameras;       // comma-separated camera names
    std::optional<double> camera_pitch_min_deg;
    std::optional<double> camera_pitch_max_deg;
    double camera_pitch_sign{-1.0};
    std::optional<double> camera_pitch_offset_deg;
    std::string camera_pointing_prepare_mode{"neutral"};
    std::string camera_pointing_takeoff_mode{"neutral"};
    std::string camera_pointing_go_home_mode{"home"};
    std::string camera_pointing_land_mode{"landing_area"};
    std::string camera_pointing_complete_mode{"neutral"};
    std::string camera_pointing_sink{"null"};  // null|mavlink_gimbal|runtime_stream
    std::string camera_pointing_mavlink_endpoints;
    std::uint8_t camera_pointing_mavlink_source_system_id{255};
    std::uint8_t camera_pointing_mavlink_source_component_id{191};
    std::uint8_t camera_pointing_mavlink_target_system_id{1};
    std::uint8_t camera_pointing_mavlink_target_component_id{1};
    std::uint8_t camera_pointing_mavlink_gimbal_device_id{0};
    std::uint32_t camera_pointing_mavlink_flags{0};
    double camera_pointing_deadband_rad{0.004363323129985824};
    double camera_pointing_resend_s{0.25};

    // ── flight lifecycle ──────────────────────────────────────────────────────
    double safe_height_m{8.0};                // takeoff/transit altitude floor
    // takeoff_height_m and behavior_min_height_m default to safe_height_m at use site
    std::optional<double> takeoff_height_m;
    std::optional<double> behavior_min_height_m;
    double takeoff_velocity_mps{1.0};
    double go_home_velocity_mps{1.0};
    double land_velocity_mps{0.5};
    double arm_retry_interval_s{1.0};
    double arm_timeout_s{10.0};
    double arm_dispatch_fallback_s{0.0};
    double takeoff_retry_interval_s{1.0};
    double land_retry_interval_s{1.0};
    double land_timeout_s{60.0};
    double disarm_retry_interval_s{1.0};
    double disarm_timeout_s{10.0};
    std::string home_policy{"initial_ego_pose"};
    double yaw_offset_rad{0.0};              // flight_yaw_offset_rad; also trajectory fallback
    double max_velocity_mps{5.0};
    std::string prepare_session_command;
    std::string trajectory_path;
    std::string flight_control_mode;         // consumed by run.sh, not by C++ runtime

    // ── flight sinks ──────────────────────────────────────────────────────────
    std::string px4_command_bridge;
    std::string velocity_command_bridge;
    std::string mavlink_command_endpoints;
    std::string px4_tmux_target;
    bool use_px4_shell_lifecycle{true};
    std::uint8_t mavlink_target_system_id{1};
    std::uint8_t mavlink_target_component_id{1};
    std::uint8_t mavlink_source_system_id{255};
    std::uint8_t mavlink_source_component_id{190};
    bool set_offboard_on_velocity{true};
};

enum class FlightCommandKind {
    Velocity,
    Arm,
    Takeoff,
    Land,
    Disarm,
};

inline const char* to_string(FlightCommandKind kind) {
    switch (kind) {
        case FlightCommandKind::Velocity:
            return "Velocity";
        case FlightCommandKind::Arm:
            return "Arm";
        case FlightCommandKind::Takeoff:
            return "Takeoff";
        case FlightCommandKind::Land:
            return "Land";
        case FlightCommandKind::Disarm:
            return "Disarm";
        default:
            return "Unknown";
    }
}

struct VelocityCommand {
    FlightCommandKind kind{FlightCommandKind::Velocity};
    TimePoint timestamp;
    Vec3 velocity_local_mps;
    double yaw_rate_radps{0.0};
    double yaw_rad{0.0};
    bool yaw_rate_valid{true};
    bool yaw_valid{false};
    std::string yaw_source{"disabled"};
};

struct CameraPointingCommand {
    TimePoint timestamp;
    std::vector<std::string> cameras;
    std::string mode{"target"};
    std::string source_track_id;
    std::string agent_id;
    std::string identity_id;
    double pitch_rad{0.0};
    double pitch_unclamped_rad{0.0};
    double pitch_min_rad{0.0};
    double pitch_max_rad{0.0};
    double target_elevation_rad{0.0};
    double range_xy_m{0.0};
    double delta_z_m{0.0};
    double pitch_sign{-1.0};
    double pitch_offset_rad{0.0};
    bool pitch_valid{false};
    bool pitch_clamped{false};
};

struct CameraPointingResult {
    bool success{false};
    std::string status;
};

struct FlightCommandResult {
    FlightCommandKind kind{FlightCommandKind::Velocity};
    bool success{false};
    std::string status;
};

enum class MissionLifecycleState {
    Idle,
    Prepare,
    Takeoff,
    ExecuteMission,
    GoHome,
    Land,
    Complete,
    Abort,
};

inline const char* to_string(MissionLifecycleState state) {
    switch (state) {
        case MissionLifecycleState::Idle:
            return "Idle";
        case MissionLifecycleState::Prepare:
            return "Prepare";
        case MissionLifecycleState::Takeoff:
            return "Takeoff";
        case MissionLifecycleState::ExecuteMission:
            return "ExecuteMission";
        case MissionLifecycleState::GoHome:
            return "GoHome";
        case MissionLifecycleState::Land:
            return "Land";
        case MissionLifecycleState::Complete:
            return "Complete";
        case MissionLifecycleState::Abort:
            return "Abort";
        default:
            return "Unknown";
    }
}

struct MissionTickInput {
    TimePoint now;
    WorldSnapshot snapshot;
    std::optional<FlightCommandResult> last_command_result;
    bool finish_requested{false};
};

struct MissionTickOutput {
    MissionLifecycleState state{MissionLifecycleState::Idle};
    std::optional<VelocityCommand> command;
    std::optional<CameraPointingCommand> camera_pointing;
    std::string status;
    std::vector<std::string> events;
};

class MissionController {
public:
    virtual ~MissionController() = default;
    virtual MissionTickOutput tick(const MissionTickInput& input) = 0;
};

class FlightCommandSink {
public:
    virtual ~FlightCommandSink() = default;
    virtual FlightCommandResult send(const VelocityCommand& command) = 0;
};

class CameraPointingSink {
public:
    virtual ~CameraPointingSink() = default;
    virtual CameraPointingResult send(const CameraPointingCommand& command) = 0;
};

}  // namespace dedalus
