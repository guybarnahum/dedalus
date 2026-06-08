#include "dedalus/behavior/behavior_osd.hpp"

#include "dedalus/core/json_utils.hpp"

#include <cmath>
#include <cstddef>
#include <optional>
#include <string>

namespace dedalus {
namespace {

// ---- Constants --------------------------------------------------------------

constexpr double kPi = 3.14159265358979323846;
constexpr double kHeadingEpsilonMps = 0.05;

// ---- Math helpers -----------------------------------------------------------

double norm_xy(const Vec3& value) {
    return std::sqrt(value.x * value.x + value.y * value.y);
}

double rad_to_deg(double rad) {
    return rad * 180.0 / kPi;
}

// ---- Enum-to-string helpers -------------------------------------------------

std::string yaw_mode_event_string(ObjectBehaviorYawMode mode) {
    switch (mode) {
        case ObjectBehaviorYawMode::Trajectory:
            return "trajectory";
        case ObjectBehaviorYawMode::Target:
            return "target";
        case ObjectBehaviorYawMode::Hold:
            return "hold";
        case ObjectBehaviorYawMode::None:
            return "none";
    }
    return "unknown";
}

std::string yaw_source_for(const VelocityCommand& command) {
    return command.yaw_source;
}

}  // namespace

// ---- Public OSD functions ---------------------------------------------------

std::string behavior_display_fields(const std::string& detail) {
    return ",\"display_state\":\"Mission\",\"display_detail\":" + q(detail);
}

std::string behavior_detail_for_event(ControllerEventKind kind) {
    if (kind == ControllerEventKind::BehaviorComplete) {
        return "done";
    }
    return "arriving";
}

std::string behavior_detail_for_tick(
    const BehaviorSpec& behavior,
    const FollowGeometry& geometry) {
    switch (behavior.type) {
        case BehaviorType::Follow:
            if (geometry.arrival_mode == "hold") {
                return geometry.target_speed_xy_mps > kHeadingEpsilonMps ? "following" : "positioned";
            }
            if (geometry.arrival_mode == "slow" || geometry.arrival_mode == "cruise") {
                return "arriving";
            }
            return "following";
        case BehaviorType::Circle:
            return geometry.circle_phase == "circling" ? "circling" : "arriving";
        case BehaviorType::Hold:
            return "positioned";
        case BehaviorType::Approach:
            return "arriving";
        default:
            return "active";
    }
}

std::string object_behavior_status(
    const BehaviorSpec& behavior,
    const FollowGeometry& geometry) {
    return "object_behavior_" + behavior_detail_for_tick(behavior, geometry);
}

ControllerEvent behavior_tick_event(
    const BehaviorMissionSpec& spec,
    const BehaviorSpec& active_behavior,
    const TargetSelection& selection,
    const Vec3& velocity,
    const FollowGeometry& geometry,
    const std::optional<std::size_t>& sequence_step_index,
    std::size_t sequence_step_count,
    ObjectBehaviorYawMode yaw_mode,
    const std::string& camera_pointing_mode) {
    JsonFields f;
    f.kv("behavior", to_string(spec.behavior.type))
     .kv("active_behavior", to_string(active_behavior.type))
     .kv("mission", spec.mission_name)
     .kv("agent_id", selection.agent_id.value)
     .kv("source_track_id", selection.source_track_id.value)
     .kv("vx", velocity.x)
     .kv("vy", velocity.y)
     .kv("vz", velocity.z)
     .kv("arrival_mode", geometry.arrival_mode)
     .kv("desired_error_xy_m", geometry.desired_error_xy_m)
     .kv("target_speed_xy_mps", geometry.target_speed_xy_mps)
     .kv("relative_speed_xy_mps", geometry.relative_speed_xy_mps)
     .kv("follow_bearing_source", geometry.bearing_source)
     .kv("follow_bearing_x", geometry.bearing_x)
     .kv("follow_bearing_y", geometry.bearing_y)
     .kv("follow_dh_m", geometry.dh_m)
     .kv("follow_required_r_m", geometry.required_r_m)
     .kv("follow_actual_r_m", geometry.actual_r_m)
     .kv("follow_elevation_deg", geometry.elevation_deg)
     .kv("circle_phase", geometry.circle_phase)
     .kv("orbit_radius_m", geometry.orbit_radius_m)
     .kv("actual_radius_m", geometry.actual_radius_m)
     .kv("radius_error_m", geometry.radius_error_m)
     .kv("radial_correction_mps", geometry.radial_correction_mps)
     .kv("tangent_velocity_mps", geometry.tangent_velocity_mps)
     .kv("target_velocity_mps", norm_xy(geometry.target_velocity))
     .kv("desired_velocity_mps", geometry.desired_velocity_mps)
     .kv("tangent_blend", geometry.tangent_blend)
     .kv("orbit_count_target", geometry.orbit_count_target)
     .kv("circle_completed_orbits", geometry.circle_completed_orbits)
     .kv("orbit_angle_rad", geometry.orbit_angle_rad)
     .kv("altitude_profile_active", geometry.altitude_profile_active)
     .kv("altitude_profile_easing", geometry.altitude_profile_easing)
     .kv("altitude_profile_t", geometry.altitude_profile_t)
     .kv("desired_height_m", geometry.desired_height_m)
     .kv("current_height_m", geometry.current_height_m)
     .kv("height_error_m", geometry.height_error_m)
     .kv("orbit_mode_latched", geometry.orbit_mode_latched)
     .kv("active_yaw_mode", yaw_mode_event_string(yaw_mode))
     .kv("active_camera_pointing_mode", camera_pointing_mode);
    if (sequence_step_index.has_value()) {
        f.kv("sequence_step_index", *sequence_step_index)
         .kv("sequence_step_count", sequence_step_count)
         .kv("sequence_step_behavior", to_string(active_behavior.type))
         .kv("sequence_step_yaw_mode", active_behavior.yaw_mode.empty() ? "inherit" : active_behavior.yaw_mode)
         .kv("sequence_step_camera_pointing_mode", active_behavior.camera_pointing_mode.empty() ? "inherit" : active_behavior.camera_pointing_mode);
    }
    return {ControllerEventKind::BehaviorTickSample,
        f.str() + behavior_display_fields(behavior_detail_for_tick(active_behavior, geometry))};
}

ControllerEvent behavior_debug_event(
    int execute_tick,
    int debug_level,
    const EgoState& ego,
    const TargetSelection& selection,
    const Vec3& raw_velocity,
    const Vec3& final_velocity,
    const VelocityCommand& command,
    const FollowGeometry& geometry) {
    const double velocity_xy = norm_xy(final_velocity);
    const double yaw_deg = command.yaw_valid ? rad_to_deg(command.yaw_rad) : 0.0;
    JsonFields f;
    f.kv("debug_level", debug_level)
     .kv("execute_tick", execute_tick)
     .kv("source_track_id", selection.source_track_id.value)
     .kv("arrival_mode", geometry.arrival_mode)
     .kv("desired_error_xy_m", geometry.desired_error_xy_m)
     .kv("target_speed_xy_mps", geometry.target_speed_xy_mps)
     .kv("closing_speed_mps", geometry.closing_speed_mps)
     .kv("relative_speed_xy_mps", geometry.relative_speed_xy_mps)
     .kv("velocity_xy_mps", velocity_xy)
     .kv("yaw_valid", command.yaw_valid)
     .kv("yaw_source", yaw_source_for(command))
     .kv("yaw_deg", yaw_deg)
     .kv("follow_bearing_source", geometry.bearing_source)
     .kv("follow_required_r_m", geometry.required_r_m)
     .kv("follow_actual_r_m", geometry.actual_r_m)
     .kv("circle_phase", geometry.circle_phase)
     .kv("orbit_radius_m", geometry.orbit_radius_m)
     .kv("actual_radius_m", geometry.actual_radius_m)
     .kv("radius_error_m", geometry.radius_error_m)
     .kv("radial_correction_mps", geometry.radial_correction_mps)
     .kv("tangent_velocity_mps", geometry.tangent_velocity_mps)
     .kv("target_velocity_mps", norm_xy(geometry.target_velocity))
     .kv("desired_velocity_mps", geometry.desired_velocity_mps)
     .kv("tangent_blend", geometry.tangent_blend)
     .kv("orbit_count_target", geometry.orbit_count_target)
     .kv("circle_completed_orbits", geometry.circle_completed_orbits)
     .kv("orbit_angle_rad", geometry.orbit_angle_rad)
     .kv("altitude_profile_active", geometry.altitude_profile_active)
     .kv("altitude_profile_easing", geometry.altitude_profile_easing)
     .kv("altitude_profile_t", geometry.altitude_profile_t)
     .kv("desired_height_m", geometry.desired_height_m)
     .kv("current_height_m", geometry.current_height_m)
     .kv("height_error_m", geometry.height_error_m)
     .kv("orbit_mode_latched", geometry.orbit_mode_latched);
    if (debug_level >= 2) {
        const double yaw_delta_deg = command.yaw_valid ? rad_to_deg(command.yaw_rad - ego.local_T_body.rotation_rpy.z) : 0.0;
        f.kv("agent_id", selection.agent_id.value)
         .kv("ego_x", ego.local_T_body.position.x)
         .kv("ego_y", ego.local_T_body.position.y)
         .kv("ego_z", ego.local_T_body.position.z)
         .kv("ego_yaw_rad", ego.local_T_body.rotation_rpy.z)
         .kv("ego_height_m", ego.height_m)
         .kv("sel_x", selection.position_local.x)
         .kv("sel_y", selection.position_local.y)
         .kv("sel_z", selection.position_local.z)
         .kv("desired_x", geometry.desired_position.x)
         .kv("desired_y", geometry.desired_position.y)
         .kv("desired_z", geometry.desired_position.z)
         .kv("target_vx", geometry.target_velocity.x)
         .kv("target_vy", geometry.target_velocity.y)
         .kv("closing_vx", geometry.closing_velocity.x)
         .kv("closing_vy", geometry.closing_velocity.y)
         .kv("raw_vx", raw_velocity.x)
         .kv("desired_vx", geometry.desired_velocity.x)
         .kv("desired_vy", geometry.desired_velocity.y)
         .kv("tangent_vx", geometry.tangent_velocity.x)
         .kv("tangent_vy", geometry.tangent_velocity.y)
         .kv("radial_correction_vx", geometry.radial_correction_velocity.x)
         .kv("radial_correction_vy", geometry.radial_correction_velocity.y)
         .kv("raw_vy", raw_velocity.y)
         .kv("raw_vz", raw_velocity.z)
         .kv("vx", final_velocity.x)
         .kv("vy", final_velocity.y)
         .kv("vz", final_velocity.z)
         .kv("yaw_rad", command.yaw_valid ? command.yaw_rad : 0.0)
         .kv("yaw_delta_from_ego_deg", yaw_delta_deg)
         .kv("follow_bearing_x", geometry.bearing_x)
         .kv("follow_bearing_y", geometry.bearing_y)
         .kv("follow_dh_m", geometry.dh_m)
         .kv("follow_elevation_deg", geometry.elevation_deg);
    }
    return {ControllerEventKind::BehaviorDebug, f.str()};
}

}  // namespace dedalus
