#include "dedalus/behavior/follow_geometry_policy.hpp"

#include <algorithm>
#include <cmath>

namespace dedalus {
namespace {

// ---- Constants --------------------------------------------------------------

constexpr double kSafeHeightCorrectionGain = 0.5;
constexpr double kMinSafeHeightClimbMps = 0.35;
// ---- Math helpers -----------------------------------------------------------

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

// circle_geometry is implemented in follow_geometry_policy_circle.cpp

// apply_altitude_profile is implemented in follow_geometry_policy_circle.cpp

// behavior_velocity is implemented in follow_geometry_policy_follow.cpp

}  // namespace dedalus

