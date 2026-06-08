#include "dedalus/behavior/follow_geometry_policy.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace dedalus {
namespace {

// ---- Constants --------------------------------------------------------------

constexpr double kPi = 3.14159265358979323846;
constexpr double kHeadingEpsilonMps = 0.05;
constexpr double kSafeHeightCorrectionGain = 0.5;
constexpr double kMinSafeHeightClimbMps = 0.35;
constexpr double kMinObservationAngleDeg = 5.0;
constexpr double kMaxObservationAngleDeg = 85.0;
constexpr double kCircleDefaultEntryToleranceM = 1.0;
constexpr double kCircleRadialCorrectionGain = 0.6;
constexpr double kDefaultAltitudeProfileDurationS = 8.0;
constexpr double kCircleMaxRadialCorrectionMps = 2.0;
constexpr double kCircleRadiusToleranceFraction = 0.25;  // tolerance = 25% of radius
constexpr double kCircleMinRadiusToleranceM = 1.0;       // floor
constexpr double kCircleMaxRadiusToleranceM = 3.0;       // cap

// ---- Math helpers -----------------------------------------------------------

double deg_to_rad(double deg) {
    return deg * kPi / 180.0;
}

double rad_to_deg(double rad) {
    return rad * 180.0 / kPi;
}

double norm_xy(const Vec3& value) {
    return std::sqrt(value.x * value.x + value.y * value.y);
}

double clamp_abs(double value, double limit) {
    if (limit <= 0.0) {
        return 0.0;
    }
    return std::clamp(value, -limit, limit);
}

Vec3 clamp_xy_norm(Vec3 velocity, double max_horizontal_mps) {
    const double horizontal = norm_xy(velocity);
    if (horizontal > max_horizontal_mps && max_horizontal_mps > 0.0) {
        const double scale = max_horizontal_mps / horizontal;
        velocity.x *= scale;
        velocity.y *= scale;
    }
    return velocity;
}

// ---- Internal geometry helpers ----------------------------------------------

Vec3 enforce_min_height_floor(
    Vec3 velocity,
    double height_m,
    double min_height_m,
    double max_vertical_speed_mps) {
    if (min_height_m <= 0.0 || max_vertical_speed_mps <= 0.0) {
        return velocity;
    }

    // AirSim/PX4 local NED convention:
    //   velocity.z < 0 climbs up and increases height above ground.
    //   velocity.z > 0 descends and decreases height above ground.
    //
    // Minimum-height floor means "do not go below this height"; it must not mean
    // "never descend while above this height". 2.31A altitude profiles rely on
    // bounded descent from takeoff/safe height down to a lower circling height.
    // If already below the floor, force a climb. If at the floor, block further
    // descent. If safely above the floor, preserve the requested vertical
    // velocity and let the profile/controller descend toward its target.
    if (height_m < min_height_m) {
        const double climb = std::clamp(
            (min_height_m - height_m) * kSafeHeightCorrectionGain,
            kMinSafeHeightClimbMps,
            max_vertical_speed_mps);
        velocity.z = std::min(velocity.z, -climb);
    } else if (height_m <= min_height_m && velocity.z > 0.0) {
        velocity.z = 0.0;
    }
    return velocity;
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

double circle_direction_sign(CircleDirection direction) {
    return direction == CircleDirection::Clockwise ? -1.0 : 1.0;
}

double smoothstep(double t) {
    t = std::clamp(t, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

double altitude_profile_duration_s(const BehaviorSpec& behavior) {
    if (behavior.altitude_profile.duration_s > 0.0) {
        return behavior.altitude_profile.duration_s;
    }
    if (behavior.duration_s > 0.0) {
        return behavior.duration_s;
    }
    return kDefaultAltitudeProfileDurationS;
}

double altitude_profile_fraction(const BehaviorSpec& behavior, double elapsed_s) {
    const double duration_s = altitude_profile_duration_s(behavior);
    if (duration_s <= 0.0) {
        return 1.0;
    }
    return std::clamp(elapsed_s / duration_s, 0.0, 1.0);
}

double altitude_profile_eased_fraction(const BehaviorSpec& behavior, double elapsed_s) {
    const double t = altitude_profile_fraction(behavior, elapsed_s);
    if (behavior.altitude_profile.easing == "linear") {
        return t;
    }
    return smoothstep(t);
}

Vec3 circle_tangent_velocity(const Vec3& radial_unit, const BehaviorSpec& behavior) {
    const double tangent_speed = behavior.radius_m * deg_to_rad(behavior.angular_speed_deg_s);
    const double sign = circle_direction_sign(behavior.direction);
    return Vec3{
        sign * -radial_unit.y * tangent_speed,
        sign * radial_unit.x * tangent_speed,
        0.0};
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

// ---- Public geometry API ----------------------------------------------------

Vec3 clamp_velocity(
    const Vec3& desired,
    double max_horizontal_mps,
    double max_vertical_mps) {
    Vec3 velocity = clamp_xy_norm(desired, max_horizontal_mps);
    velocity.z = clamp_abs(velocity.z, max_vertical_mps);
    return velocity;
}

Vec3 apply_altitude_policy(
    Vec3 velocity,
    const ObjectBehaviorMissionConfig& config,
    const BehaviorSpec& behavior,
    double height_m) {
    if (config.altitude_policy == ObjectBehaviorAltitudePolicy::SafeHeightFloor) {
        return enforce_min_height_floor(
            velocity,
            height_m,
            config.behavior_min_height_m,
            behavior.max_vertical_speed_mps);
    }
    return velocity;
}

Vec3 enforce_takeoff_height_floor(
    Vec3 velocity,
    double height_m,
    double takeoff_height_m,
    double max_vertical_speed_mps) {
    return enforce_min_height_floor(
        velocity,
        height_m,
        takeoff_height_m,
        max_vertical_speed_mps);
}

FollowGeometry circle_geometry(
    const EgoState& ego,
    const TargetSelection& selection,
    const BehaviorSpec& behavior,
    const ObjectBehaviorMissionConfig& /*config*/,
    bool orbit_mode_latched) {
    FollowGeometry geometry;
    geometry.orbit_radius_m = behavior.radius_m;
    geometry.target_velocity = selection.velocity_local;
    geometry.target_speed_xy_mps = norm_xy(selection.velocity_local);
    geometry.orbit_count_target = behavior.orbit_count;

    const Vec3 entry_axis{1.0, 0.0, 0.0};

    const Vec3 target_to_ego = target_to_ego_xy(ego, selection);

    geometry.actual_radius_m = norm_xy(target_to_ego);
    geometry.actual_r_m = geometry.actual_radius_m;
    geometry.required_r_m = behavior.radius_m;
    geometry.radius_error_m = geometry.actual_radius_m - behavior.radius_m;

    if (geometry.actual_radius_m > 1e-6) {
        geometry.radial_unit = Vec3{
            target_to_ego.x / geometry.actual_radius_m,
            target_to_ego.y / geometry.actual_radius_m,
            0.0};
    } else {
        // Degenerate case: directly over target. Pick a deterministic radial
        // direction so tangent/radial control can immediately separate.
        geometry.radial_unit = entry_axis;
    }

    geometry.orbit_angle_rad = std::atan2(geometry.radial_unit.y, geometry.radial_unit.x);

    // Nominal desired point for debug/operator display. This is no longer a
    // hard waypoint target; orbit capture is radius/tangent based.
    geometry.desired_position = Vec3{
        selection.position_local.x + behavior.radius_m * geometry.radial_unit.x,
        selection.position_local.y + behavior.radius_m * geometry.radial_unit.y,
        selection.position_local.z - behavior.altitude_offset_m};

    const Vec3 desired_error{
        geometry.desired_position.x - ego.local_T_body.position.x,
        geometry.desired_position.y - ego.local_T_body.position.y,
        geometry.desired_position.z - ego.local_T_body.position.z};
    geometry.desired_error_xy_m = norm_xy(desired_error);

    const double entry_tolerance_m = std::max(
        kCircleDefaultEntryToleranceM,
        behavior.position_tolerance_m);

    // Forgiving orbit insertion:
    // - Do not require threading one exact 3 o'clock entry coordinate.
    // - If radius is close enough, enter orbit mode from the current radial
    //   angle.
    // - Once latched, stay in orbit mode so small live-control deviations do
    //   not drop the controller back into insertion.
    const double radius_capture_tolerance_m = std::max(
        entry_tolerance_m,
        std::min(kCircleMaxRadiusToleranceM,
                 std::max(kCircleMinRadiusToleranceM, behavior.radius_m * kCircleRadiusToleranceFraction)));
    const bool radius_captured = std::abs(geometry.radius_error_m) <= radius_capture_tolerance_m;
    const bool on_entry = orbit_mode_latched || radius_captured;

    geometry.orbit_mode_latched = orbit_mode_latched || on_entry;
    geometry.circle_phase = on_entry ? "circling" : "arriving";
    geometry.arrival_mode = geometry.circle_phase;

    // Continuous orbit-capture controller:
    //
    //   target velocity
    // + tangent velocity at the current radial angle
    // + radial correction toward requested radius
    // + altitude correction
    //
    // This is intentionally used in both arriving and circling. The phase only
    // affects display/orbit counting; the command law remains continuous and
    // recoverable from bad initial radius, overshoot, or imperfect bearing.
    geometry.tangent_blend = 1.0;
    geometry.tangent_velocity = circle_tangent_velocity(geometry.radial_unit, behavior);
    geometry.tangent_velocity_mps = norm_xy(geometry.tangent_velocity);

    geometry.radial_correction_mps = std::clamp(
        -geometry.radius_error_m * kCircleRadialCorrectionGain,
        -kCircleMaxRadialCorrectionMps,
        kCircleMaxRadialCorrectionMps);
    geometry.radial_correction_velocity = Vec3{
        geometry.radial_unit.x * geometry.radial_correction_mps,
        geometry.radial_unit.y * geometry.radial_correction_mps,
        0.0};

    const double altitude_error = geometry.desired_position.z - ego.local_T_body.position.z;
    Vec3 relative_orbit_velocity{
        geometry.tangent_velocity.x + geometry.radial_correction_velocity.x,
        geometry.tangent_velocity.y + geometry.radial_correction_velocity.y,
        0.0};
    relative_orbit_velocity = clamp_xy_norm(relative_orbit_velocity, behavior.max_speed_mps);

    geometry.desired_velocity = Vec3{
        selection.velocity_local.x + relative_orbit_velocity.x,
        selection.velocity_local.y + relative_orbit_velocity.y,
        altitude_error};

    geometry.tangent_velocity_mps = norm_xy(geometry.tangent_velocity);
    geometry.radial_correction_velocity = Vec3{
        relative_orbit_velocity.x - geometry.tangent_velocity.x,
        relative_orbit_velocity.y - geometry.tangent_velocity.y,
        0.0};

    const Vec3 velocity = clamp_velocity(
        geometry.desired_velocity,
        behavior.max_speed_mps,
        behavior.max_vertical_speed_mps);
    geometry.desired_velocity_mps = norm_xy(geometry.desired_velocity);
    geometry.closing_velocity = geometry.radial_correction_velocity;
    geometry.closing_speed_mps = std::abs(geometry.radial_correction_mps);
    geometry.relative_speed_xy_mps = norm_xy(Vec3{
        velocity.x - selection.velocity_local.x,
        velocity.y - selection.velocity_local.y,
        0.0});
    return geometry;
}

Vec3 apply_altitude_profile(
    Vec3 velocity,
    const EgoState& ego,
    const BehaviorSpec& behavior,
    double elapsed_s,
    FollowGeometry& geometry) {
    if (!behavior.altitude_profile.enabled) {
        return velocity;
    }
    const double t = altitude_profile_fraction(behavior, elapsed_s);
    const double eased_t = altitude_profile_eased_fraction(behavior, elapsed_s);
    const double desired_height_m =
        behavior.altitude_profile.start_height_m +
        (behavior.altitude_profile.end_height_m - behavior.altitude_profile.start_height_m) * eased_t;
    const double current_height_m = ego.height_valid ? ego.height_m : -ego.local_T_body.position.z;
    const double height_error_m = desired_height_m - current_height_m;
    velocity.z = clamp_abs(-height_error_m, behavior.max_vertical_speed_mps);
    geometry.altitude_profile_active = true;
    geometry.altitude_profile_easing = behavior.altitude_profile.easing;
    geometry.altitude_profile_t = t;
    geometry.desired_height_m = desired_height_m;
    geometry.current_height_m = current_height_m;
    geometry.height_error_m = height_error_m;
    return velocity;
}

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
