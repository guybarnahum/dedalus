#include "dedalus/behavior/follow_geometry_policy.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace dedalus {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kHeadingEpsilonMps = 0.05;
constexpr double kMinObservationAngleDeg = 5.0;
constexpr double kMaxObservationAngleDeg = 85.0;

double deg_to_rad(double deg) {
    return deg * kPi / 180.0;
}

double rad_to_deg(double rad) {
    return rad * 180.0 / kPi;
}

double norm_xy(const Vec3& value) {
    return std::sqrt(value.x * value.x + value.y * value.y);
}

Vec3 target_frame_follow_offset(
    const EgoState& ego,
    const TargetSelection& selection,
    const BehaviorSpec& behavior) {
    const auto& offset = behavior.relative_offset_m;
    Vec3 delta{0.0, 0.0, 0.0};
    const double target_speed_xy = norm_xy(selection.velocity_local);
    if (behavior.target_frame == ReferenceFrame::TargetHeadingFrame && target_speed_xy > kHeadingEpsilonMps) {
        const double forward_x = selection.velocity_local.x / target_speed_xy;
        const double forward_y = selection.velocity_local.y / target_speed_xy;
        const double right_x = -forward_y;
        const double right_y = forward_x;
        delta.x = forward_x * offset.x + right_x * offset.y;
        delta.y = forward_y * offset.x + right_y * offset.y;
    } else if (behavior.target_frame == ReferenceFrame::DroneHeadingFrame) {
        const double yaw = ego.local_T_body.rotation_rpy.z;
        const double forward_x = std::cos(yaw);
        const double forward_y = std::sin(yaw);
        const double right_x = -forward_y;
        const double right_y = forward_x;
        delta.x = forward_x * offset.x + right_x * offset.y;
        delta.y = forward_y * offset.x + right_y * offset.y;
    } else {
        delta.x = offset.x;
        delta.y = offset.y;
    }
    delta.z = -offset.z;
    return delta;
}

Vec3 target_to_ego_xy(const EgoState& ego, const TargetSelection& selection) {
    return Vec3{
        ego.local_T_body.position.x - selection.position_local.x,
        ego.local_T_body.position.y - selection.position_local.y,
        0.0};
}

Vec3 desired_follow_position(
    const EgoState& ego,
    const TargetSelection& selection,
    const BehaviorSpec& behavior) {
    const Vec3 offset = target_frame_follow_offset(ego, selection, behavior);
    return Vec3{
        selection.position_local.x + offset.x,
        selection.position_local.y + offset.y,
        selection.position_local.z + offset.z};
}

void choose_follow_bearing(
    const EgoState& ego,
    const TargetSelection& selection,
    const BehaviorSpec& behavior,
    FollowGeometry& geometry) {
    const Vec3 offset = target_frame_follow_offset(ego, selection, behavior);
    const double offset_norm = norm_xy(offset);
    const double target_speed_xy = norm_xy(selection.velocity_local);

    // The desired observation bearing must be behavior/target-state-relative, not ego-position-relative.
    // Moving targets prefer the behavior offset expressed in the target heading frame; if the offset has
    // no XY component, fall back to trailing the target velocity. Static targets use the configured
    // behavior offset directly. Ego-relative bearing is intentionally not used because it makes the
    // desired point rotate around a static target as the drone moves.
    if (target_speed_xy > kHeadingEpsilonMps && offset_norm <= kHeadingEpsilonMps) {
        geometry.bearing_x = -selection.velocity_local.x / target_speed_xy;
        geometry.bearing_y = -selection.velocity_local.y / target_speed_xy;
        geometry.bearing_source = "target_velocity";
        return;
    }
    if (offset_norm > kHeadingEpsilonMps) {
        geometry.bearing_x = offset.x / offset_norm;
        geometry.bearing_y = offset.y / offset_norm;
        geometry.bearing_source = "behavior_offset";
        return;
    }

    geometry.bearing_x = -1.0;
    geometry.bearing_y = 0.0;
    geometry.bearing_source = "default_fallback";
}

FollowGeometry follow_observation_geometry(
    const EgoState& ego,
    const TargetSelection& selection,
    const BehaviorSpec& behavior,
    const ObjectBehaviorMissionConfig& config) {
    FollowGeometry geometry;
    const Vec3 base_desired = desired_follow_position(ego, selection, behavior);
    geometry.desired_position = base_desired;
    geometry.target_velocity = selection.velocity_local;
    geometry.target_speed_xy_mps = norm_xy(selection.velocity_local);

    const Vec3 target_to_ego = target_to_ego_xy(ego, selection);
    geometry.actual_r_m = norm_xy(target_to_ego);
    geometry.dh_m = std::abs(ego.local_T_body.position.z - selection.position_local.z);

    const double angle_deg = std::clamp(
        config.follow_max_elevation_angle_deg,
        kMinObservationAngleDeg,
        kMaxObservationAngleDeg);
    const double min_by_angle = geometry.dh_m / std::tan(deg_to_rad(angle_deg));
    geometry.required_r_m = std::max(config.follow_min_standoff_m, min_by_angle);
    geometry.elevation_deg = rad_to_deg(std::atan2(geometry.dh_m, std::max(geometry.actual_r_m, 1e-6)));

    if (!config.follow_observation_geometry_enabled || geometry.required_r_m <= 0.0) {
        geometry.bearing_source = "disabled";
        return geometry;
    }

    choose_follow_bearing(ego, selection, behavior, geometry);
    geometry.desired_position.x = selection.position_local.x + geometry.bearing_x * geometry.required_r_m;
    geometry.desired_position.y = selection.position_local.y + geometry.bearing_y * geometry.required_r_m;
    geometry.desired_position.z = base_desired.z;
    return geometry;
}

Vec3 follow_arrival_velocity(
    const EgoState& ego,
    const TargetSelection& selection,
    const BehaviorSpec& behavior,
    const ObjectBehaviorMissionConfig& config,
    FollowGeometry& geometry) {
    const Vec3 error{
        geometry.desired_position.x - ego.local_T_body.position.x,
        geometry.desired_position.y - ego.local_T_body.position.y,
        geometry.desired_position.z - ego.local_T_body.position.z};
    geometry.desired_error_xy_m = norm_xy(error);
    geometry.target_velocity = selection.velocity_local;
    geometry.target_speed_xy_mps = norm_xy(selection.velocity_local);

    double closing_speed = 0.0;
    if (geometry.desired_error_xy_m <= config.follow_arrival_hold_radius_m) {
        geometry.arrival_mode = "hold";
        closing_speed = 0.0;
    } else if (geometry.desired_error_xy_m <= config.follow_arrival_slow_radius_m) {
        geometry.arrival_mode = "slow";
        closing_speed = std::min(
            behavior.max_speed_mps,
            std::max(0.0, config.follow_arrival_kp * geometry.desired_error_xy_m));
    } else {
        geometry.arrival_mode = "cruise";
        closing_speed = behavior.max_speed_mps;
    }

    if (closing_speed > 0.0 && geometry.desired_error_xy_m > 1e-6) {
        geometry.closing_velocity.x = error.x / geometry.desired_error_xy_m * closing_speed;
        geometry.closing_velocity.y = error.y / geometry.desired_error_xy_m * closing_speed;
    }
    geometry.closing_speed_mps = norm_xy(geometry.closing_velocity);

    Vec3 velocity{
        selection.velocity_local.x + geometry.closing_velocity.x,
        selection.velocity_local.y + geometry.closing_velocity.y,
        error.z,
    };
    velocity = clamp_velocity(velocity, behavior.max_speed_mps, behavior.max_vertical_speed_mps);
    geometry.relative_speed_xy_mps = norm_xy(Vec3{
        velocity.x - selection.velocity_local.x,
        velocity.y - selection.velocity_local.y,
        0.0});
    return velocity;
}

FollowGeometry approach_geometry(
    const EgoState& ego,
    const TargetSelection& selection,
    const BehaviorSpec& behavior) {
    FollowGeometry geometry;
    geometry.target_velocity = selection.velocity_local;
    const Vec3 target_to_ego = target_to_ego_xy(ego, selection);
    geometry.actual_r_m = norm_xy(target_to_ego);
    geometry.required_r_m = behavior.stop_distance_m;
    geometry.behavior_step_complete =
        geometry.actual_r_m <= behavior.stop_distance_m + behavior.position_tolerance_m;
    geometry.arrival_mode = geometry.behavior_step_complete ? "hold" : "cruise";
    return geometry;
}

Vec3 approach_velocity(
    const EgoState& ego,
    const TargetSelection& selection,
    const BehaviorSpec& behavior,
    FollowGeometry& geometry) {
    geometry = approach_geometry(ego, selection, behavior);
    geometry.target_speed_xy_mps = norm_xy(selection.velocity_local);
    if (geometry.behavior_step_complete) {
        geometry.desired_position = ego.local_T_body.position;
        geometry.desired_error_xy_m = 0.0;
        geometry.desired_velocity = Vec3{0.0, 0.0, 0.0};
        return Vec3{0.0, 0.0, 0.0};
    }

    const Vec3 ego_to_target{
        selection.position_local.x - ego.local_T_body.position.x,
        selection.position_local.y - ego.local_T_body.position.y,
        0.0};
    const double range_xy = std::max(norm_xy(ego_to_target), 1e-6);
    const Vec3 unit_to_target{
        ego_to_target.x / range_xy,
        ego_to_target.y / range_xy,
        0.0};

    geometry.desired_position = Vec3{
        selection.position_local.x - unit_to_target.x * behavior.stop_distance_m,
        selection.position_local.y - unit_to_target.y * behavior.stop_distance_m,
        selection.position_local.z - behavior.altitude_offset_m};
    const Vec3 error{
        geometry.desired_position.x - ego.local_T_body.position.x,
        geometry.desired_position.y - ego.local_T_body.position.y,
        geometry.desired_position.z - ego.local_T_body.position.z};
    geometry.desired_error_xy_m = norm_xy(error);
    geometry.closing_velocity = clamp_velocity(error, behavior.max_speed_mps, behavior.max_vertical_speed_mps);
    geometry.desired_velocity = Vec3{
        selection.velocity_local.x + geometry.closing_velocity.x,
        selection.velocity_local.y + geometry.closing_velocity.y,
        geometry.closing_velocity.z};
    Vec3 velocity = clamp_velocity(
        geometry.desired_velocity,
        behavior.max_speed_mps,
        behavior.max_vertical_speed_mps);
    geometry.desired_velocity_mps = norm_xy(geometry.desired_velocity);
    geometry.closing_speed_mps = norm_xy(velocity);
    geometry.relative_speed_xy_mps = norm_xy(Vec3{velocity.x - selection.velocity_local.x, velocity.y - selection.velocity_local.y, 0.0});
    return velocity;
}

}  // namespace

Vec3 behavior_velocity(
    const EgoState& ego,
    const TargetSelection& selection,
    const BehaviorSpec& behavior,
    const ObjectBehaviorMissionConfig& config,
    FollowGeometry* geometry_out) {
    if (behavior.type == BehaviorType::Follow) {
        FollowGeometry geometry = follow_observation_geometry(ego, selection, behavior, config);
        const Vec3 velocity = follow_arrival_velocity(ego, selection, behavior, config, geometry);
        if (geometry_out != nullptr) {
            *geometry_out = geometry;
        }
        return velocity;
    }
    if (behavior.type == BehaviorType::Approach) {
        FollowGeometry geometry;
        const Vec3 velocity = approach_velocity(ego, selection, behavior, geometry);
        if (geometry_out != nullptr) *geometry_out = geometry;
        return velocity;
    }
    if (behavior.type == BehaviorType::Circle) {
        FollowGeometry geometry = circle_geometry(ego, selection, behavior, config, false);
        const Vec3 velocity = clamp_velocity(geometry.desired_velocity, behavior.max_speed_mps, behavior.max_vertical_speed_mps);
        if (geometry_out != nullptr) {
            *geometry_out = geometry;
        }
        return velocity;
    }
    if (geometry_out != nullptr) {
        geometry_out->desired_position = ego.local_T_body.position;
    }
    return Vec3{0.0, 0.0, 0.0};
}

}  // namespace dedalus
