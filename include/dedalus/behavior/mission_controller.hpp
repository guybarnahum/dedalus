#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "dedalus/core/types.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

namespace dedalus {

struct MissionOptions {
    std::unordered_map<std::string, std::string> values;

    [[nodiscard]] bool contains(const std::string& key) const {
        return values.find(key) != values.end();
    }

    [[nodiscard]] std::string get_or(const std::string& key, const std::string& fallback) const {
        const auto it = values.find(key);
        return it == values.end() ? fallback : it->second;
    }

    // All recognized mission_options keys. An unrecognized key in a YAML config
    // produces a warning at load time rather than silently using fallback values.
    [[nodiscard]] static const std::unordered_set<std::string>& known_keys() {
        // clang-format off
        static const std::unordered_set<std::string> s{
            // behavior spec
            "behavior_spec_path",
            // follow behavior
            "object_behavior_follow_observation_geometry_enabled",
            "object_behavior_zero_target_velocity",
            "object_behavior_follow_min_standoff_m",
            "object_behavior_follow_max_elevation_angle_deg",
            "object_behavior_follow_arrival_slow_radius_m",
            "object_behavior_follow_arrival_hold_radius_m",
            "object_behavior_follow_arrival_kp",
            "object_behavior_completion_after_s",
            // object behavior core
            "object_behavior_hold_velocity_mps",
            "object_behavior_yaw_mode",
            "object_behavior_yaw_min_speed_mps",
            "object_behavior_yaw_hold_last_when_unstable",
            "object_behavior_vertical_stare_mode",
            "object_behavior_vertical_stare_warn_if_unavailable",
            "object_behavior_debug_every_n_ticks",
            "object_behavior_debug_level",
            "object_behavior_altitude_policy",
            "object_behavior_min_height_m",
            // camera pointing
            "object_behavior_camera_pointing_cameras",
            "object_behavior_camera_pitch_min_deg",
            "object_behavior_camera_pitch_max_deg",
            "object_behavior_camera_pitch_sign",
            "object_behavior_camera_pitch_offset_deg",
            "object_behavior_camera_pointing_prepare_mode",
            "object_behavior_camera_pointing_takeoff_mode",
            "object_behavior_camera_pointing_go_home_mode",
            "object_behavior_camera_pointing_land_mode",
            "object_behavior_camera_pointing_complete_mode",
            "object_behavior_camera_pointing_sink",
            "object_behavior_camera_pointing_mavlink_endpoints",
            "object_behavior_camera_pointing_mavlink_source_system_id",
            "object_behavior_camera_pointing_mavlink_source_component_id",
            "object_behavior_camera_pointing_mavlink_target_system_id",
            "object_behavior_camera_pointing_mavlink_target_component_id",
            "object_behavior_camera_pointing_mavlink_gimbal_device_id",
            "object_behavior_camera_pointing_mavlink_flags",
            "object_behavior_camera_pointing_deadband_rad",
            "object_behavior_camera_pointing_resend_s",
            // flight lifecycle
            "flight_safe_height_m",
            "flight_takeoff_height_m",
            "flight_takeoff_velocity_mps",
            "flight_go_home_velocity_mps",
            "flight_land_velocity_mps",
            "flight_arm_retry_interval_s",
            "flight_arm_timeout_s",
            "flight_arm_dispatch_fallback_s",
            "flight_takeoff_retry_interval_s",
            "flight_land_retry_interval_s",
            "flight_land_timeout_s",
            "flight_disarm_retry_interval_s",
            "flight_disarm_timeout_s",
            "flight_home_policy",
            "flight_yaw_offset_rad",
            "flight_max_velocity_mps",
            "flight_prepare_session_command",
            "flight_trajectory_path",
            // flight sinks
            "flight_px4_command_bridge",
            "flight_velocity_command_bridge",
            "flight_mavlink_command_endpoints",
            "flight_px4_tmux_target",
            "flight_use_px4_shell_lifecycle",
            "flight_mavlink_target_system_id",
            "flight_mavlink_target_component_id",
            "flight_mavlink_source_system_id",
            "flight_mavlink_source_component_id",
            "flight_mavlink_set_offboard_on_velocity",
        };
        // clang-format on
        return s;
    }
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
