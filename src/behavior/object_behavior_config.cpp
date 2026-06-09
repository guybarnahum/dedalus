#include "dedalus/behavior/object_behavior_mission_controller.hpp"

#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dedalus {
namespace {

constexpr double kPi = 3.14159265358979323846;

double deg_to_rad(double deg) {
    return deg * kPi / 180.0;
}

std::vector<std::string> split_csv(const std::string& value, const std::vector<std::string>& fallback) {
    if (value.empty()) {
        return fallback;
    }
    std::vector<std::string> result;
    std::string token;
    for (char c : value) {
        if (c == ',') {
            // strip surrounding whitespace/CR/LF
            std::string t;
            for (char ch : token) {
                if (ch != ' ' && ch != '\t' && ch != '\r' && ch != '\n') {
                    t += ch;
                }
            }
            if (!t.empty()) {
                result.push_back(std::move(t));
            }
            token.clear();
        } else {
            token += c;
        }
    }
    std::string t;
    for (char ch : token) {
        if (ch != ' ' && ch != '\t' && ch != '\r' && ch != '\n') {
            t += ch;
        }
    }
    if (!t.empty()) {
        result.push_back(std::move(t));
    }
    return result.empty() ? fallback : result;
}

ObjectBehaviorAltitudePolicy parse_altitude_policy(const std::string& value) {
    if (value.empty() || value == "target_relative") {
        return ObjectBehaviorAltitudePolicy::TargetRelative;
    }
    if (value == "safe_height_floor") {
        return ObjectBehaviorAltitudePolicy::SafeHeightFloor;
    }
    throw std::invalid_argument("unknown object_behavior_altitude_policy: " + value);
}

ObjectBehaviorYawMode parse_yaw_mode(const std::string& value) {
    if (value.empty() || value == "trajectory" || value == "travel_direction" || value == "from_heading") {
        return ObjectBehaviorYawMode::Trajectory;
    }
    if (value == "target" || value == "to_target" || value == "stare_at_target") {
        return ObjectBehaviorYawMode::Target;
    }
    if (value == "hold") {
        return ObjectBehaviorYawMode::Hold;
    }
    if (value == "none" || value == "disabled") {
        return ObjectBehaviorYawMode::None;
    }
    throw std::invalid_argument("unknown object_behavior_yaw_mode: " + value);
}

ObjectBehaviorVerticalStareMode parse_vertical_stare_mode(const std::string& value) {
    if (value.empty() || value == "none" || value == "disabled") {
        return ObjectBehaviorVerticalStareMode::None;
    }
    if (value == "debug" || value == "debug_only") {
        return ObjectBehaviorVerticalStareMode::DebugOnly;
    }
    if (value == "gimbal") {
        return ObjectBehaviorVerticalStareMode::Gimbal;
    }
    throw std::invalid_argument("unknown object_behavior_vertical_stare_mode: " + value);
}

void parse_camera_pointing_config(const MissionOptions& options, ObjectBehaviorMissionConfig& config) {
    config.camera_pointing_cameras = split_csv(options.camera_pointing_cameras, {});
    if (options.camera_pitch_min_deg.has_value()) {
        config.camera_pitch_min_rad = deg_to_rad(*options.camera_pitch_min_deg);
    }
    if (options.camera_pitch_max_deg.has_value()) {
        config.camera_pitch_max_rad = deg_to_rad(*options.camera_pitch_max_deg);
    }
    config.camera_pitch_sign = options.camera_pitch_sign;
    if (options.camera_pitch_offset_deg.has_value()) {
        config.camera_pitch_offset_rad = deg_to_rad(*options.camera_pitch_offset_deg);
    }
    config.camera_pointing_prepare_mode = options.camera_pointing_prepare_mode;
    config.camera_pointing_takeoff_mode = options.camera_pointing_takeoff_mode;
    config.camera_pointing_go_home_mode = options.camera_pointing_go_home_mode;
    config.camera_pointing_land_mode    = options.camera_pointing_land_mode;
    config.camera_pointing_complete_mode = options.camera_pointing_complete_mode;
    if (config.camera_pitch_min_rad > config.camera_pitch_max_rad) {
        std::swap(config.camera_pitch_min_rad, config.camera_pitch_max_rad);
    }
}

void parse_follow_config(const MissionOptions& options, ObjectBehaviorMissionConfig& config) {
    config.follow_observation_geometry_enabled = options.follow_observation_geometry_enabled;
    config.zero_target_velocity               = options.zero_target_velocity;
    config.follow_min_standoff_m              = options.follow_min_standoff_m;
    config.follow_max_elevation_angle_deg     = options.follow_max_elevation_angle_deg;
    config.follow_arrival_slow_radius_m       = options.follow_arrival_slow_radius_m;
    config.follow_arrival_hold_radius_m       = options.follow_arrival_hold_radius_m;
    config.follow_arrival_kp                  = options.follow_arrival_kp;
    if (options.completion_after_s.has_value()) {
        config.behavior_spec.completion.after_s = *options.completion_after_s;
    }
}

void parse_height_config(const MissionOptions& options, ObjectBehaviorMissionConfig& config) {
    // flight_safe_height_m is the legacy single value.
    // flight_takeoff_height_m overrides the Takeoff -> ExecuteMission altitude gate.
    // object_behavior_min_height_m overrides the ExecuteMission floor, allowing
    // lower circling/inspection after a higher takeoff.
    config.takeoff_height_m      = options.takeoff_height_m.value_or(options.safe_height_m);
    config.behavior_min_height_m = options.behavior_min_height_m.value_or(options.safe_height_m);
}

void parse_flight_ops_config(const MissionOptions& options, ObjectBehaviorMissionConfig& config) {
    config.takeoff_velocity_mps    = options.takeoff_velocity_mps;
    config.go_home_velocity_mps    = options.go_home_velocity_mps;
    config.arm_retry_interval_s    = options.arm_retry_interval_s;
    config.arm_timeout_s           = options.arm_timeout_s;
    config.arm_dispatch_fallback_s = options.arm_dispatch_fallback_s;
    config.takeoff_retry_interval_s = options.takeoff_retry_interval_s;
    config.land_retry_interval_s   = options.land_retry_interval_s;
    config.land_timeout_s          = options.land_timeout_s;
    config.disarm_retry_interval_s = options.disarm_retry_interval_s;
    config.disarm_timeout_s        = options.disarm_timeout_s;
    config.home_policy             = options.home_policy;
}

}  // namespace

ObjectBehaviorMissionConfig load_object_behavior_mission_config(const MissionOptions& options) {
    ObjectBehaviorMissionConfig config;
    if (options.behavior_spec_path.empty()) {
        throw std::invalid_argument("object_behavior mission_controller requires mission_options.behavior_spec_path");
    }
    config.behavior_spec = parse_behavior_spec_file(options.behavior_spec_path);
    config.hold_velocity_mps          = options.hold_velocity_mps;
    config.yaw_offset_rad             = options.object_behavior_yaw_offset_rad.value_or(options.yaw_offset_rad);
    config.yaw_min_speed_mps          = options.yaw_min_speed_mps;
    config.yaw_hold_last_when_unstable = options.yaw_hold_last_when_unstable;
    config.yaw_mode                   = parse_yaw_mode(options.yaw_mode);
    config.vertical_stare_mode        = parse_vertical_stare_mode(options.vertical_stare_mode);
    config.vertical_stare_warn_if_unavailable = options.vertical_stare_warn_if_unavailable;
    parse_camera_pointing_config(options, config);
    config.debug_every_n_ticks = options.debug_every_n_ticks;
    config.debug_level         = options.debug_level;
    config.altitude_policy     = parse_altitude_policy(options.altitude_policy);
    parse_follow_config(options, config);
    parse_height_config(options, config);
    parse_flight_ops_config(options, config);
    return config;
}

}  // namespace dedalus
