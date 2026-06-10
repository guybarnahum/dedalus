#include "dedalus/behavior/follow_geometry_policy.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace dedalus {
namespace {

// ---- Constants (circle / altitude-profile) ----------------------------------

constexpr double kPi = 3.14159265358979323846;
constexpr double kCircleDefaultEntryToleranceM = 1.0;
constexpr double kCircleRadialCorrectionGain = 0.6;
constexpr double kDefaultAltitudeProfileDurationS = 8.0;
constexpr double kCircleMaxRadialCorrectionMps = 2.0;
constexpr double kCircleRadiusToleranceFraction = 0.25;  // tolerance = 25% of radius
constexpr double kCircleMinRadiusToleranceM = 1.0;       // floor
constexpr double kCircleMaxRadiusToleranceM = 3.0;       // cap

// ---- Helpers (duplicated from follow_geometry_policy.cpp where shared) ------

double deg_to_rad(double deg) {
    return deg * kPi / 180.0;
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

Vec3 target_to_ego_xy(const EgoState& ego, const TargetSelection& selection) {
    return Vec3{
        ego.local_T_body.position.x - selection.position_local.x,
        ego.local_T_body.position.y - selection.position_local.y,
        0.0};
}

// ---- Circle / altitude-profile helpers (exclusive to this translation unit) -

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

}  // namespace

// ---- Public circle/altitude-profile API -------------------------------------

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

}  // namespace dedalus
