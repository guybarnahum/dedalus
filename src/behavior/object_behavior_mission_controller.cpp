#include "dedalus/behavior/object_behavior_mission_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dedalus {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kMinArrivedDistanceM = 0.5;
constexpr double kLandHeightM = 0.25;
constexpr double kTakeoffVelocityAssistHeightM = 0.5;
constexpr double kHeadingEpsilonMps = 0.05;
constexpr double kSafeHeightCorrectionGain = 0.5;
constexpr double kMinSafeHeightClimbMps = 0.35;
constexpr double kMinObservationAngleDeg = 5.0;
constexpr double kMaxObservationAngleDeg = 85.0;
constexpr double kCircleDefaultEntryToleranceM = 1.0;
constexpr double kCircleRadialCorrectionGain = 0.6;
constexpr double kCircleMaxRadialCorrectionMps = 2.0;

struct FollowGeometry {
    Vec3 desired_position;
    Vec3 closing_velocity;
    Vec3 target_velocity;
    double dh_m{0.0};
    double required_r_m{0.0};
    double actual_r_m{0.0};
    double elevation_deg{0.0};
    double bearing_x{0.0};
    double bearing_y{0.0};
    std::string bearing_source{"disabled"};
    double desired_error_xy_m{0.0};
    double closing_speed_mps{0.0};
    double target_speed_xy_mps{0.0};
    double relative_speed_xy_mps{0.0};
    std::string arrival_mode{"none"};
    std::string circle_phase{"none"};
    bool behavior_step_complete{false};
    double orbit_radius_m{0.0};
    double actual_radius_m{0.0};
    double radius_error_m{0.0};
    double radial_correction_mps{0.0};
    double tangent_velocity_mps{0.0};
    double desired_velocity_mps{0.0};
    Vec3 radial_unit{1.0, 0.0, 0.0};
    double orbit_angle_rad{0.0};
    double orbit_count_target{0.0};
    double circle_completed_orbits{0.0};
    double tangent_blend{0.0};
    bool orbit_mode_latched{false};
    Vec3 tangent_velocity;
    Vec3 radial_correction_velocity;
    Vec3 desired_velocity;
};

std::string json_escape(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size());
    for (const char ch : value) {
        switch (ch) {
            case '\\':
                escaped += "\\\\";
                break;
            case '"':
                escaped += "\\\"";
                break;
            case '\n':
                escaped += "\\n";
                break;
            case '\r':
                escaped += "\\r";
                break;
            case '\t':
                escaped += "\\t";
                break;
            default:
                escaped.push_back(ch);
                break;
        }
    }
    return escaped;
}

std::string q(const std::string& value) {
    return "\"" + json_escape(value) + "\"";
}

double seconds_between(TimePoint start, TimePoint end) {
    return static_cast<double>(end.timestamp_ns - start.timestamp_ns) / 1'000'000'000.0;
}

bool elapsed_at_least(TimePoint start, TimePoint end, double seconds) {
    return seconds_between(start, end) >= seconds;
}

bool last_result_matches_success(
    const std::optional<FlightCommandResult>& result,
    FlightCommandKind kind) {
    return result.has_value() && result->kind == kind && result->success;
}

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

bool parse_bool(const std::string& value, bool fallback) {
    if (value.empty()) {
        return fallback;
    }
    if (value == "true" || value == "1" || value == "yes" || value == "on") {
        return true;
    }
    if (value == "false" || value == "0" || value == "no" || value == "off") {
        return false;
    }
    throw std::invalid_argument("invalid boolean value: " + value);
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

std::string json_string_array(const std::vector<std::string>& values) {
    std::string out = "[";
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i > 0) out += ",";
        out += q(values[i]);
    }
    out += "]";
    return out;
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

Vec3 velocity_toward_xy(const Vec3& from, const Vec3& to, double speed_mps) {
    const Vec3 delta{to.x - from.x, to.y - from.y, 0.0};
    const double distance = norm_xy(delta);
    if (distance <= kMinArrivedDistanceM || speed_mps <= 0.0) {
        return Vec3{0.0, 0.0, 0.0};
    }
    return Vec3{delta.x / distance * speed_mps, delta.y / distance * speed_mps, 0.0};
}

Vec3 clamp_velocity(const Vec3& desired, double max_horizontal_mps, double max_vertical_mps) {
    Vec3 velocity = clamp_xy_norm(desired, max_horizontal_mps);
    velocity.z = clamp_abs(velocity.z, max_vertical_mps);
    return velocity;
}

Vec3 enforce_safe_height_floor(
    Vec3 velocity,
    double height_m,
    double safe_height_m,
    double max_vertical_speed_mps) {
    if (safe_height_m <= 0.0 || max_vertical_speed_mps <= 0.0) {
        return velocity;
    }
    if (height_m < safe_height_m) {
        const double climb = std::clamp(
            (safe_height_m - height_m) * kSafeHeightCorrectionGain,
            kMinSafeHeightClimbMps,
            max_vertical_speed_mps);
        velocity.z = std::min(velocity.z, -climb);
    } else if (velocity.z > 0.0) {
        velocity.z = 0.0;
    }
    return velocity;
}

Vec3 apply_altitude_policy(
    Vec3 velocity,
    const ObjectBehaviorMissionConfig& config,
    const BehaviorSpec& behavior,
    double height_m) {
    if (config.altitude_policy == ObjectBehaviorAltitudePolicy::SafeHeightFloor) {
        return enforce_safe_height_floor(
            velocity,
            height_m,
            config.safe_height_m,
            behavior.max_vertical_speed_mps);
    }
    return velocity;
}

Vec3 target_frame_follow_offset(const EgoState& ego, const TargetSelection& selection, const BehaviorSpec& behavior) {
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

Vec3 desired_follow_position(const EgoState& ego, const TargetSelection& selection, const BehaviorSpec& behavior) {
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

    const Vec3 target_to_ego{
        ego.local_T_body.position.x - selection.position_local.x,
        ego.local_T_body.position.y - selection.position_local.y,
        0.0};
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
    const Vec3 target_to_ego{
        ego.local_T_body.position.x - selection.position_local.x,
        ego.local_T_body.position.y - selection.position_local.y,
        0.0};
    geometry.actual_r_m = norm_xy(target_to_ego);
    geometry.required_r_m = behavior.stop_distance_m;
    geometry.behavior_step_complete =
        geometry.actual_r_m <= behavior.stop_distance_m + behavior.position_tolerance_m;
    geometry.arrival_mode = geometry.behavior_step_complete ? "hold" : "cruise";
    return geometry;
}

FollowGeometry circle_geometry(
    const EgoState& ego,
    const TargetSelection& selection,
    const BehaviorSpec& behavior,
    const ObjectBehaviorMissionConfig& /*config*/,
    bool orbit_mode_latched = false) {
    FollowGeometry geometry;
    geometry.orbit_radius_m = behavior.radius_m;
    geometry.target_velocity = selection.velocity_local;
    geometry.target_speed_xy_mps = norm_xy(selection.velocity_local);
    geometry.orbit_count_target = behavior.orbit_count;

    const Vec3 entry_axis{1.0, 0.0, 0.0};

    const Vec3 target_to_ego{
        ego.local_T_body.position.x - selection.position_local.x,
        ego.local_T_body.position.y - selection.position_local.y,
        0.0};

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
        std::min(3.0, std::max(1.0, behavior.radius_m * 0.25)));
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

Vec3 behavior_velocity(
    const EgoState& ego,
    const TargetSelection& selection,
    const BehaviorSpec& behavior,
    const ObjectBehaviorMissionConfig& config,
    FollowGeometry* geometry_out = nullptr) {
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

std::string class_label_event_string(ClassLabel label) {
    switch (label) {
        case ClassLabel::Person:
            return "person";
        case ClassLabel::Drone:
            return "drone";
        case ClassLabel::Car:
            return "car";
        case ClassLabel::Boat:
            return "boat";
        case ClassLabel::Animal:
            return "animal";
        case ClassLabel::House:
            return "house";
        case ClassLabel::Building:
            return "building";
        case ClassLabel::Tree:
            return "tree";
        case ClassLabel::Road:
            return "road";
        case ClassLabel::River:
            return "river";
        case ClassLabel::Terrain:
            return "terrain";
        case ClassLabel::Unknown:
        default:
            return "unknown";
    }
}

std::string yaw_source_for(const VelocityCommand& command, double /*min_speed_mps*/) {
    return command.yaw_source;
}

}  // namespace

ObjectBehaviorMissionConfig load_object_behavior_mission_config(const MissionOptions& options) {
    ObjectBehaviorMissionConfig config;
    const auto behavior_spec_path = options.get_or("behavior_spec_path", "");
    if (behavior_spec_path.empty()) {
        throw std::invalid_argument("object_behavior mission_controller requires mission_options.behavior_spec_path");
    }
    config.behavior_spec = parse_behavior_spec_file(behavior_spec_path);
    config.hold_velocity_mps = std::stod(options.get_or("object_behavior_hold_velocity_mps", "0.0"));
    config.yaw_offset_rad = std::stod(options.get_or(
        "object_behavior_yaw_offset_rad",
        options.get_or("flight_yaw_offset_rad", "0.0")));
    config.yaw_min_speed_mps = std::stod(options.get_or("object_behavior_yaw_min_speed_mps", "0.35"));
    config.yaw_hold_last_when_unstable = parse_bool(
        options.get_or("object_behavior_yaw_hold_last_when_unstable", "true"),
        true);
    config.yaw_mode = parse_yaw_mode(options.get_or("object_behavior_yaw_mode", "trajectory"));
    config.vertical_stare_mode = parse_vertical_stare_mode(
        options.get_or("object_behavior_vertical_stare_mode", "none"));
    config.vertical_stare_warn_if_unavailable = parse_bool(
        options.get_or("object_behavior_vertical_stare_warn_if_unavailable", "true"),
        true);
    config.camera_pointing_cameras = split_csv(
        options.get_or("object_behavior_camera_pointing_cameras", ""),
        {});
    {
        const auto min_deg_str = options.get_or("object_behavior_camera_pitch_min_deg", "");
        if (!min_deg_str.empty()) {
            config.camera_pitch_min_rad = deg_to_rad(std::stod(min_deg_str));
        }
    }
    {
        const auto max_deg_str = options.get_or("object_behavior_camera_pitch_max_deg", "");
        if (!max_deg_str.empty()) {
            config.camera_pitch_max_rad = deg_to_rad(std::stod(max_deg_str));
        }
    }
    config.camera_pitch_sign = std::stod(options.get_or("object_behavior_camera_pitch_sign", "-1"));
    {
        const auto offset_deg_str = options.get_or("object_behavior_camera_pitch_offset_deg", "");
        if (!offset_deg_str.empty()) {
            config.camera_pitch_offset_rad = deg_to_rad(std::stod(offset_deg_str));
        }
    }
    config.camera_pointing_prepare_mode = options.get_or(
        "object_behavior_camera_pointing_prepare_mode",
        "neutral");
    config.camera_pointing_takeoff_mode = options.get_or(
        "object_behavior_camera_pointing_takeoff_mode",
        "neutral");
    config.camera_pointing_go_home_mode = options.get_or(
        "object_behavior_camera_pointing_go_home_mode",
        "home");
    config.camera_pointing_land_mode = options.get_or(
        "object_behavior_camera_pointing_land_mode",
        "landing_area");
    config.camera_pointing_complete_mode = options.get_or(
        "object_behavior_camera_pointing_complete_mode",
        "neutral");
    if (config.camera_pitch_min_rad > config.camera_pitch_max_rad) {
        throw std::invalid_argument(
            "object_behavior_camera_pitch_min_deg must be <= object_behavior_camera_pitch_max_deg");
    }

    config.debug_every_n_ticks = std::stoi(options.get_or("object_behavior_debug_every_n_ticks", "0"));
    config.debug_level = std::stoi(options.get_or(
        "object_behavior_debug_level",
        "1"));
    config.altitude_policy = parse_altitude_policy(options.get_or(
        "object_behavior_altitude_policy",
        "target_relative"));
    config.follow_observation_geometry_enabled = parse_bool(
        options.get_or("object_behavior_follow_observation_geometry_enabled", "false"),
        false);
    config.zero_target_velocity = parse_bool(
        options.get_or("object_behavior_zero_target_velocity", "false"),
        false);
    config.follow_min_standoff_m = std::stod(options.get_or("object_behavior_follow_min_standoff_m", "8.0"));
    config.follow_max_elevation_angle_deg = std::stod(options.get_or(
        "object_behavior_follow_max_elevation_angle_deg",
        "35.0"));
    config.follow_arrival_slow_radius_m = std::stod(options.get_or(
        "object_behavior_follow_arrival_slow_radius_m",
        "8.0"));
    config.follow_arrival_hold_radius_m = std::stod(options.get_or(
        "object_behavior_follow_arrival_hold_radius_m",
        "2.0"));
    config.follow_arrival_kp = std::stod(options.get_or(
        "object_behavior_follow_arrival_kp",
        "0.35"));
    const auto completion_after_override = options.get_or("object_behavior_completion_after_s", "");
    if (!completion_after_override.empty()) {
        config.behavior_spec.completion.after_s = std::stod(completion_after_override);
    }
    config.safe_height_m = std::stod(options.get_or("flight_safe_height_m", "8"));
    config.takeoff_velocity_mps = std::stod(options.get_or("flight_takeoff_velocity_mps", "1.0"));
    config.go_home_velocity_mps = std::stod(options.get_or("flight_go_home_velocity_mps", "1.0"));
    config.arm_retry_interval_s = std::stod(options.get_or("flight_arm_retry_interval_s", "1.0"));
    config.arm_timeout_s = std::stod(options.get_or("flight_arm_timeout_s", "10.0"));
    config.arm_dispatch_fallback_s = std::stod(options.get_or("flight_arm_dispatch_fallback_s", "0.0"));
    config.takeoff_retry_interval_s = std::stod(options.get_or("flight_takeoff_retry_interval_s", "1.0"));
    config.land_retry_interval_s = std::stod(options.get_or("flight_land_retry_interval_s", "1.0"));
    config.land_timeout_s = std::stod(options.get_or("flight_land_timeout_s", "60.0"));
    config.disarm_retry_interval_s = std::stod(options.get_or("flight_disarm_retry_interval_s", "1.0"));
    config.disarm_timeout_s = std::stod(options.get_or("flight_disarm_timeout_s", "10.0"));
    config.home_policy = options.get_or("flight_home_policy", "initial_ego_pose");
    return config;
}

ObjectBehaviorMissionController::ObjectBehaviorMissionController(ObjectBehaviorMissionConfig config)
    : config_(std::move(config)) {}

VelocityCommand ObjectBehaviorMissionController::command_from_velocity(
    TimePoint timestamp,
    Vec3 velocity_local_mps,
    double yaw_offset_rad) const {
    VelocityCommand command;
    command.kind = FlightCommandKind::Velocity;
    command.timestamp = timestamp;
    command.velocity_local_mps = velocity_local_mps;
    command.yaw_rate_radps = 0.0;
    command.yaw_rate_valid = true;
    command.yaw_valid = false;
    command.yaw_source = "disabled";

    const double horizontal = norm_xy(velocity_local_mps);
    if (horizontal >= config_.yaw_min_speed_mps) {
        command.yaw_valid = true;
        command.yaw_rad = std::atan2(velocity_local_mps.y, velocity_local_mps.x) + yaw_offset_rad;
        command.yaw_source = "trajectory";
        last_stable_yaw_valid_ = true;
        last_stable_yaw_rad_ = command.yaw_rad;
    } else if (config_.yaw_hold_last_when_unstable && last_stable_yaw_valid_) {
        command.yaw_valid = true;
        command.yaw_rad = last_stable_yaw_rad_;
        command.yaw_source = "hold_last";
    }
    return command;
}

VelocityCommand ObjectBehaviorMissionController::command_from_behavior_velocity(
    TimePoint timestamp,
    Vec3 velocity_local_mps,
    const EgoState& ego,
    const TargetSelection& selection,
    double yaw_offset_rad,
    ObjectBehaviorYawMode yaw_mode) const {
    if (yaw_mode == ObjectBehaviorYawMode::Trajectory) {
        return command_from_velocity(timestamp, velocity_local_mps, yaw_offset_rad);
    }

    VelocityCommand command;
    command.kind = FlightCommandKind::Velocity;
    command.timestamp = timestamp;
    command.velocity_local_mps = velocity_local_mps;
    command.yaw_rate_radps = 0.0;
    command.yaw_rate_valid = true;
    command.yaw_valid = false;
    command.yaw_source = "disabled";

    if (yaw_mode == ObjectBehaviorYawMode::None) {
        return command;
    }

    if (yaw_mode == ObjectBehaviorYawMode::Hold) {
        if (last_stable_yaw_valid_) {
            command.yaw_valid = true;
            command.yaw_rad = last_stable_yaw_rad_;
            command.yaw_source = "hold_last";
        }
        return command;
    }

    if (yaw_mode == ObjectBehaviorYawMode::Target) {
        const Vec3 target_delta{
            selection.position_local.x - ego.local_T_body.position.x,
            selection.position_local.y - ego.local_T_body.position.y,
            0.0};
        const double target_range_xy_m = norm_xy(target_delta);
        if (target_range_xy_m > 1e-6) {
            command.yaw_valid = true;
            command.yaw_rad = std::atan2(target_delta.y, target_delta.x) + yaw_offset_rad;
            command.yaw_source = "target";
            last_stable_yaw_valid_ = true;
            last_stable_yaw_rad_ = command.yaw_rad;
        } else if (config_.yaw_hold_last_when_unstable && last_stable_yaw_valid_) {
            command.yaw_valid = true;
            command.yaw_rad = last_stable_yaw_rad_;
            command.yaw_source = "hold_last";
        }
        return command;
    }

    return command_from_velocity(timestamp, velocity_local_mps, yaw_offset_rad);
}

VelocityCommand ObjectBehaviorMissionController::command_with_kind(
    TimePoint timestamp, FlightCommandKind kind) const {
    VelocityCommand command;
    command.kind = kind;
    command.timestamp = timestamp;
    command.velocity_local_mps = Vec3{0.0, 0.0, 0.0};
    command.yaw_rate_valid = false;
    command.yaw_valid = false;
    return command;
}

std::string ObjectBehaviorMissionController::target_event(const TargetSelection& selection) const {
    return "\"event\":\"target_selected\""
        ",\"agent_id\":" + q(selection.agent_id.value) +
        ",\"source_track_id\":" + q(selection.source_track_id.value) +
        ",\"identity_id\":" + q(selection.identity_id.value) +
        ",\"class\":" + q(class_label_event_string(selection.class_label)) +
        ",\"confidence\":" + std::to_string(selection.confidence) +
        ",\"reason\":" + q(selection.reason);
}

std::string behavior_display_fields(const std::string& detail) {
    return ",\"display_state\":\"Mission\",\"display_detail\":" + q(detail);
}

std::string behavior_detail_for_event(const std::string& event) {
    if (event == "behavior_complete") {
        return "done";
    }
    return "arriving";
}

std::string behavior_detail_for_tick(const BehaviorSpec& behavior, const FollowGeometry& geometry) {
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

std::string object_behavior_status(const BehaviorSpec& behavior, const FollowGeometry& geometry) {
    return "object_behavior_" + behavior_detail_for_tick(behavior, geometry);
}

std::string ObjectBehaviorMissionController::behavior_event(
    const std::string& event,
    const std::string& reason) const {
    std::string fields = "\"event\":" + q(event) +
        ",\"behavior\":" + q(to_string(config_.behavior_spec.behavior.type)) +
        ",\"mission\":" + q(config_.behavior_spec.mission_name) +
        ",\"reason\":" + q(reason);
    fields += behavior_display_fields(behavior_detail_for_event(event));
    if (previous_selection_.has_value()) {
        fields += ",\"agent_id\":" + q(previous_selection_->agent_id.value) +
            ",\"source_track_id\":" + q(previous_selection_->source_track_id.value) +
            ",\"identity_id\":" + q(previous_selection_->identity_id.value);
    }
    return fields;
}

std::string ObjectBehaviorMissionController::sequence_step_event(
    const std::string& event,
    const BehaviorSpec& behavior,
    std::size_t index,
    const std::string& reason) const {
    return "\"event\":" + q(event) +
        ",\"behavior\":\"sequence\"" +
        ",\"step_index\":" + std::to_string(index) +
        ",\"step_behavior\":" + q(to_string(behavior.type)) +
        ",\"step_yaw_mode\":" + q(behavior.yaw_mode.empty() ? "inherit" : behavior.yaw_mode) +
        ",\"step_camera_pointing_mode\":" + q(behavior.camera_pointing_mode.empty() ? "inherit" : behavior.camera_pointing_mode) +
        ",\"mission\":" + q(config_.behavior_spec.mission_name) +
        ",\"reason\":" + q(reason) +
        behavior_display_fields(behavior_detail_for_event(event));
}

std::string behavior_tick_event(
    const BehaviorMissionSpec& spec,
    const BehaviorSpec& active_behavior,
    const TargetSelection& selection,
    const Vec3& velocity,
    const FollowGeometry& geometry,
    const std::optional<std::size_t>& sequence_step_index,
    std::size_t sequence_step_count,
    ObjectBehaviorYawMode yaw_mode,
    const std::string& camera_pointing_mode) {
    std::string event = "\"event\":\"behavior_tick_sample\""
        ",\"behavior\":" + q(to_string(spec.behavior.type)) +
        ",\"active_behavior\":" + q(to_string(active_behavior.type)) +
        ",\"mission\":" + q(spec.mission_name) +
        ",\"agent_id\":" + q(selection.agent_id.value) +
        ",\"source_track_id\":" + q(selection.source_track_id.value) +
        ",\"vx\":" + std::to_string(velocity.x) +
        ",\"vy\":" + std::to_string(velocity.y) +
        ",\"vz\":" + std::to_string(velocity.z) +
        ",\"arrival_mode\":" + q(geometry.arrival_mode) +
        ",\"desired_error_xy_m\":" + std::to_string(geometry.desired_error_xy_m) +
        ",\"target_speed_xy_mps\":" + std::to_string(geometry.target_speed_xy_mps) +
        ",\"relative_speed_xy_mps\":" + std::to_string(geometry.relative_speed_xy_mps) +
        ",\"follow_bearing_source\":" + q(geometry.bearing_source) +
        ",\"follow_bearing_x\":" + std::to_string(geometry.bearing_x) +
        ",\"follow_bearing_y\":" + std::to_string(geometry.bearing_y) +
        ",\"follow_dh_m\":" + std::to_string(geometry.dh_m) +
        ",\"follow_required_r_m\":" + std::to_string(geometry.required_r_m) +
        ",\"follow_actual_r_m\":" + std::to_string(geometry.actual_r_m) +
        ",\"follow_elevation_deg\":" + std::to_string(geometry.elevation_deg) +
        ",\"circle_phase\":" + q(geometry.circle_phase) +
        ",\"orbit_radius_m\":" + std::to_string(geometry.orbit_radius_m) +
        ",\"actual_radius_m\":" + std::to_string(geometry.actual_radius_m) +
        ",\"radius_error_m\":" + std::to_string(geometry.radius_error_m) +
        ",\"radial_correction_mps\":" + std::to_string(geometry.radial_correction_mps) +
        ",\"tangent_velocity_mps\":" + std::to_string(geometry.tangent_velocity_mps) +
        ",\"target_velocity_mps\":" + std::to_string(norm_xy(geometry.target_velocity)) +
        ",\"desired_velocity_mps\":" + std::to_string(geometry.desired_velocity_mps) +
        ",\"tangent_blend\":" + std::to_string(geometry.tangent_blend) +
        ",\"orbit_count_target\":" + std::to_string(geometry.orbit_count_target) +
        ",\"circle_completed_orbits\":" + std::to_string(geometry.circle_completed_orbits) +
        ",\"orbit_angle_rad\":" + std::to_string(geometry.orbit_angle_rad) +
        ",\"orbit_mode_latched\":" + std::string(geometry.orbit_mode_latched ? "true" : "false") +
        ",\"active_yaw_mode\":" + q(yaw_mode_event_string(yaw_mode)) +
        ",\"active_camera_pointing_mode\":" + q(camera_pointing_mode);
    if (sequence_step_index.has_value()) {
        event += ",\"sequence_step_index\":" + std::to_string(*sequence_step_index) +
            ",\"sequence_step_count\":" + std::to_string(sequence_step_count) +
            ",\"sequence_step_behavior\":" + q(to_string(active_behavior.type)) +
            ",\"sequence_step_yaw_mode\":" + q(active_behavior.yaw_mode.empty() ? "inherit" : active_behavior.yaw_mode) +
            ",\"sequence_step_camera_pointing_mode\":" + q(active_behavior.camera_pointing_mode.empty() ? "inherit" : active_behavior.camera_pointing_mode);
    }
    event += behavior_display_fields(behavior_detail_for_tick(active_behavior, geometry));
    return event;
}

std::string behavior_debug_event(
    int execute_tick,
    int debug_level,
    const EgoState& ego,
    const TargetSelection& selection,
    const Vec3& raw_velocity,
    const Vec3& final_velocity,
    const VelocityCommand& command,
    const FollowGeometry& geometry,
    double yaw_min_speed_mps) {
    const double velocity_xy = norm_xy(final_velocity);
    const double yaw_deg = command.yaw_valid ? rad_to_deg(command.yaw_rad) : 0.0;
    std::string event = "\"event\":\"behavior_debug\""
        ",\"debug_level\":" + std::to_string(debug_level) +
        ",\"execute_tick\":" + std::to_string(execute_tick) +
        ",\"source_track_id\":" + q(selection.source_track_id.value) +
        ",\"arrival_mode\":" + q(geometry.arrival_mode) +
        ",\"desired_error_xy_m\":" + std::to_string(geometry.desired_error_xy_m) +
        ",\"target_speed_xy_mps\":" + std::to_string(geometry.target_speed_xy_mps) +
        ",\"closing_speed_mps\":" + std::to_string(geometry.closing_speed_mps) +
        ",\"relative_speed_xy_mps\":" + std::to_string(geometry.relative_speed_xy_mps) +
        ",\"velocity_xy_mps\":" + std::to_string(velocity_xy) +
        ",\"yaw_valid\":" + std::string(command.yaw_valid ? "true" : "false") +
        ",\"yaw_source\":" + q(yaw_source_for(command, yaw_min_speed_mps)) +
        ",\"yaw_deg\":" + std::to_string(yaw_deg) +
        ",\"follow_bearing_source\":" + q(geometry.bearing_source) +
        ",\"follow_required_r_m\":" + std::to_string(geometry.required_r_m) +
        ",\"follow_actual_r_m\":" + std::to_string(geometry.actual_r_m) +
        ",\"circle_phase\":" + q(geometry.circle_phase) +
        ",\"orbit_radius_m\":" + std::to_string(geometry.orbit_radius_m) +
        ",\"actual_radius_m\":" + std::to_string(geometry.actual_radius_m) +
        ",\"radius_error_m\":" + std::to_string(geometry.radius_error_m) +
        ",\"radial_correction_mps\":" + std::to_string(geometry.radial_correction_mps) +
        ",\"tangent_velocity_mps\":" + std::to_string(geometry.tangent_velocity_mps) +
        ",\"target_velocity_mps\":" + std::to_string(norm_xy(geometry.target_velocity)) +
        ",\"desired_velocity_mps\":" + std::to_string(geometry.desired_velocity_mps) +
        ",\"tangent_blend\":" + std::to_string(geometry.tangent_blend) +
        ",\"orbit_count_target\":" + std::to_string(geometry.orbit_count_target) +
        ",\"circle_completed_orbits\":" + std::to_string(geometry.circle_completed_orbits) +
        ",\"orbit_angle_rad\":" + std::to_string(geometry.orbit_angle_rad) +
        ",\"orbit_mode_latched\":" + std::string(geometry.orbit_mode_latched ? "true" : "false");

    if (debug_level >= 2) {
        const double yaw_delta_deg = command.yaw_valid ? rad_to_deg(command.yaw_rad - ego.local_T_body.rotation_rpy.z) : 0.0;
        event += ",\"agent_id\":" + q(selection.agent_id.value) +
            ",\"ego_x\":" + std::to_string(ego.local_T_body.position.x) +
            ",\"ego_y\":" + std::to_string(ego.local_T_body.position.y) +
            ",\"ego_z\":" + std::to_string(ego.local_T_body.position.z) +
            ",\"ego_yaw_rad\":" + std::to_string(ego.local_T_body.rotation_rpy.z) +
            ",\"ego_height_m\":" + std::to_string(ego.height_m) +
            ",\"sel_x\":" + std::to_string(selection.position_local.x) +
            ",\"sel_y\":" + std::to_string(selection.position_local.y) +
            ",\"sel_z\":" + std::to_string(selection.position_local.z) +
            ",\"desired_x\":" + std::to_string(geometry.desired_position.x) +
            ",\"desired_y\":" + std::to_string(geometry.desired_position.y) +
            ",\"desired_z\":" + std::to_string(geometry.desired_position.z) +
            ",\"target_vx\":" + std::to_string(geometry.target_velocity.x) +
            ",\"target_vy\":" + std::to_string(geometry.target_velocity.y) +
            ",\"closing_vx\":" + std::to_string(geometry.closing_velocity.x) +
            ",\"closing_vy\":" + std::to_string(geometry.closing_velocity.y) +
            ",\"raw_vx\":" + std::to_string(raw_velocity.x) +
            ",\"desired_vx\":" + std::to_string(geometry.desired_velocity.x) +
            ",\"desired_vy\":" + std::to_string(geometry.desired_velocity.y) +
            ",\"tangent_vx\":" + std::to_string(geometry.tangent_velocity.x) +
            ",\"tangent_vy\":" + std::to_string(geometry.tangent_velocity.y) +
            ",\"radial_correction_vx\":" + std::to_string(geometry.radial_correction_velocity.x) +
            ",\"radial_correction_vy\":" + std::to_string(geometry.radial_correction_velocity.y) +
            ",\"raw_vy\":" + std::to_string(raw_velocity.y) +
            ",\"raw_vz\":" + std::to_string(raw_velocity.z) +
            ",\"vx\":" + std::to_string(final_velocity.x) +
            ",\"vy\":" + std::to_string(final_velocity.y) +
            ",\"vz\":" + std::to_string(final_velocity.z) +
            ",\"yaw_rad\":" + std::to_string(command.yaw_valid ? command.yaw_rad : 0.0) +
            ",\"yaw_delta_from_ego_deg\":" + std::to_string(yaw_delta_deg) +
            ",\"follow_bearing_x\":" + std::to_string(geometry.bearing_x) +
            ",\"follow_bearing_y\":" + std::to_string(geometry.bearing_y) +
            ",\"follow_dh_m\":" + std::to_string(geometry.dh_m) +
            ",\"follow_elevation_deg\":" + std::to_string(geometry.elevation_deg);
    }
    return event;
}

bool ObjectBehaviorMissionController::completion_elapsed(TimePoint now) const {
    const double after_s = config_.behavior_spec.completion.after_s;
    if (after_s <= 0.0) {
        return false;
    }
    return seconds_between(behavior_start_, now) >= after_s;
}

std::optional<CameraPointingCommand> ObjectBehaviorMissionController::camera_pointing_command(
    TimePoint timestamp,
    const EgoState& ego,
    const TargetSelection& selection,
    const std::string& mode) const {
    const std::string effective_mode = mode.empty() ? "target" : mode;
    if (effective_mode == "neutral" || effective_mode == "reset") {
        return neutral_camera_pointing_command(timestamp, effective_mode);
    }
    if ((effective_mode == "home" || effective_mode == "landing_area") && home_initialized_) {
        return camera_pointing_command_to_point(timestamp, ego, home_pose_.position, effective_mode);
    }
    auto command = camera_pointing_command_to_point(
        timestamp,
        ego,
        selection.position_local,
        effective_mode);
    if (!command) {
        return std::nullopt;
    }
    command->source_track_id = selection.source_track_id.value;
    command->agent_id = selection.agent_id.value;
    command->identity_id = selection.identity_id.value;
    return command;
}

std::optional<CameraPointingCommand> ObjectBehaviorMissionController::camera_pointing_command_for_behavior(
    TimePoint timestamp,
    const EgoState& ego,
    const TargetSelection& selection,
    const BehaviorSpec& behavior) const {
    const std::string mode = behavior.camera_pointing_mode.empty()
        ? "target"
        : behavior.camera_pointing_mode;
    return camera_pointing_command(timestamp, ego, selection, mode);
}

std::optional<CameraPointingCommand> ObjectBehaviorMissionController::camera_pointing_command_to_point(
    TimePoint timestamp,
    const EgoState& ego,
    const Vec3& target_position_local,
    const std::string& mode) const {
    if (config_.vertical_stare_mode == ObjectBehaviorVerticalStareMode::None || mode == "none" || mode == "disabled") {
        return std::nullopt;
    }
    if (mode == "neutral" || mode == "reset") {
        return neutral_camera_pointing_command(timestamp, mode);
    }

    CameraPointingCommand command;
    command.timestamp = timestamp;
    command.cameras = config_.camera_pointing_cameras;
    if (command.cameras.empty()) {
        command.cameras.push_back("front_center");
    }
    command.mode = mode;

    const double dx = target_position_local.x - ego.local_T_body.position.x;
    const double dy = target_position_local.y - ego.local_T_body.position.y;
    const double dz = target_position_local.z - ego.local_T_body.position.z;
    const double range_xy = std::max(std::hypot(dx, dy), 1e-6);
    const double elevation_rad = std::atan2(dz, range_xy);
    const double unclamped_pitch_rad = config_.camera_pitch_sign * elevation_rad + config_.camera_pitch_offset_rad;
    const double pitch_rad = std::max(config_.camera_pitch_min_rad, std::min(config_.camera_pitch_max_rad, unclamped_pitch_rad));

    command.pitch_rad = pitch_rad;
    command.pitch_unclamped_rad = unclamped_pitch_rad;
    command.pitch_min_rad = config_.camera_pitch_min_rad;
    command.pitch_max_rad = config_.camera_pitch_max_rad;
    command.target_elevation_rad = elevation_rad;
    command.range_xy_m = range_xy;
    command.delta_z_m = dz;
    command.pitch_sign = config_.camera_pitch_sign;
    command.pitch_offset_rad = config_.camera_pitch_offset_rad;
    command.pitch_valid = true;
    command.pitch_clamped = std::abs(pitch_rad - unclamped_pitch_rad) > 1e-9;
    return command;
}

std::optional<CameraPointingCommand> ObjectBehaviorMissionController::neutral_camera_pointing_command(
    TimePoint timestamp,
    const std::string& mode) const {
    if (config_.vertical_stare_mode == ObjectBehaviorVerticalStareMode::None || mode == "none" || mode == "disabled") {
        return std::nullopt;
    }

    CameraPointingCommand command;
    command.timestamp = timestamp;
    command.cameras = config_.camera_pointing_cameras;
    if (command.cameras.empty()) {
        command.cameras.push_back("front_center");
    }
    command.mode = mode;
    command.pitch_rad = 0.0;
    command.pitch_unclamped_rad = 0.0;
    command.pitch_min_rad = config_.camera_pitch_min_rad;
    command.pitch_max_rad = config_.camera_pitch_max_rad;
    command.target_elevation_rad = 0.0;
    command.range_xy_m = 0.0;
    command.delta_z_m = 0.0;
    command.pitch_sign = config_.camera_pitch_sign;
    command.pitch_offset_rad = config_.camera_pitch_offset_rad;
    command.pitch_valid = true;
    command.pitch_clamped = false;
    return command;
}

std::string ObjectBehaviorMissionController::camera_pointing_intent_event(
    const CameraPointingCommand& command) const {
    return "\"event\":\"camera_pointing_intent\""
        ",\"camera_pointing_mode\":" + q(command.mode) +
        ",\"vertical_stare_mode\":" + q(config_.vertical_stare_mode == ObjectBehaviorVerticalStareMode::Gimbal ? "gimbal" : "debug_only") +
        ",\"cameras\":" + json_string_array(command.cameras) +
        ",\"pitch_valid\":" + std::string(command.pitch_valid ? "true" : "false") +
        ",\"pitch_rad\":" + std::to_string(command.pitch_rad) +
        ",\"pitch_deg\":" + std::to_string(rad_to_deg(command.pitch_rad)) +
        ",\"pitch_unclamped_rad\":" + std::to_string(command.pitch_unclamped_rad) +
        ",\"pitch_unclamped_deg\":" + std::to_string(rad_to_deg(command.pitch_unclamped_rad)) +
        ",\"pitch_min_rad\":" + std::to_string(command.pitch_min_rad) +
        ",\"pitch_max_rad\":" + std::to_string(command.pitch_max_rad) +
        ",\"pitch_sign\":" + std::to_string(command.pitch_sign) +
        ",\"pitch_offset_rad\":" + std::to_string(command.pitch_offset_rad) +
        ",\"pitch_clamped\":" + std::string(command.pitch_clamped ? "true" : "false") +
        ",\"target_elevation_rad\":" + std::to_string(command.target_elevation_rad) +
        ",\"target_elevation_deg\":" + std::to_string(rad_to_deg(command.target_elevation_rad)) +
        ",\"range_xy_m\":" + std::to_string(command.range_xy_m) +
        ",\"delta_z_m\":" + std::to_string(command.delta_z_m) +
        ",\"agent_id\":" + q(command.agent_id) +
        ",\"source_track_id\":" + q(command.source_track_id) +
        ",\"identity_id\":" + q(command.identity_id) +
        behavior_display_fields("arriving");
}

void ObjectBehaviorMissionController::emit_camera_pointing(
    MissionTickOutput& output,
    const CameraPointingCommand& command) const {
    output.camera_pointing = command;
    output.events.push_back(camera_pointing_intent_event(command));
}

Vec3 ObjectBehaviorMissionController::go_home_velocity(const EgoState& ego) const {
    if (!home_initialized_) {
        return Vec3{0.0, 0.0, 0.0};
    }
    return velocity_toward_xy(ego.local_T_body.position, home_pose_.position, config_.go_home_velocity_mps);
}

void ObjectBehaviorMissionController::begin_abort_recovery(
    TimePoint now,
    double height_m,
    const std::string& reason) {
    aborting_ = true;
    abort_reason_ = reason;
    state_start_ = now;
    if (height_m > kLandHeightM) {
        state_ = home_initialized_ ? MissionLifecycleState::GoHome : MissionLifecycleState::Land;
    } else {
        state_ = MissionLifecycleState::Complete;
    }
}

bool ObjectBehaviorMissionController::update_circle_orbit_progress(
    const BehaviorSpec& behavior,
    bool circling,
    double orbit_angle_rad) {
    if (behavior.type != BehaviorType::Circle || behavior.orbit_count <= 0.0) {
        return false;
    }

    if (!circling) {
        circle_orbit_tracking_ = false;
        return false;
    }

    if (!circle_orbit_tracking_) {
        circle_orbit_tracking_ = true;
        circle_previous_angle_rad_ = orbit_angle_rad;
        circle_completed_orbits_ = 0.0;
        return false;
    }

    double delta = orbit_angle_rad - circle_previous_angle_rad_;
    while (delta > kPi) {
        delta -= 2.0 * kPi;
    }
    while (delta < -kPi) {
        delta += 2.0 * kPi;
    }

    const double directed_delta = circle_direction_sign(behavior.direction) * delta;
    if (directed_delta > 0.0) {
        circle_completed_orbits_ += directed_delta / (2.0 * kPi);
    }

    circle_previous_angle_rad_ = orbit_angle_rad;
    return circle_completed_orbits_ >= behavior.orbit_count;
}

bool ObjectBehaviorMissionController::sequence_active() const {
    return config_.behavior_spec.behavior.type == BehaviorType::Sequence;
}

const BehaviorSpec& ObjectBehaviorMissionController::active_behavior() const {
    if (!sequence_active()) {
        return config_.behavior_spec.behavior;
    }
    const auto& steps = config_.behavior_spec.behavior.steps;
    if (steps.empty()) {
        return config_.behavior_spec.behavior;
    }
    const std::size_t index = std::min(sequence_step_index_, steps.size() - 1U);
    return steps[index];
}

bool ObjectBehaviorMissionController::active_behavior_is_last_sequence_step() const {
    if (!sequence_active()) {
        return true;
    }
    const auto& steps = config_.behavior_spec.behavior.steps;
    return steps.empty() || sequence_step_index_ + 1U >= steps.size();
}

ObjectBehaviorYawMode ObjectBehaviorMissionController::yaw_mode_for_behavior(const BehaviorSpec& behavior) const {
    if (behavior.yaw_mode.empty()) {
        return config_.yaw_mode;
    }
    return parse_yaw_mode(behavior.yaw_mode);
}

void ObjectBehaviorMissionController::reset_sequence_step(TimePoint now) {
    sequence_step_start_ = now;
    sequence_step_started_ = true;
    circle_in_orbit_mode_ = false;
    circle_orbit_tracking_ = false;
    circle_previous_angle_rad_ = 0.0;
    circle_completed_orbits_ = 0.0;
    last_behavior_display_detail_.clear();
}

void ObjectBehaviorMissionController::reset_behavior_run(TimePoint now) {
    behavior_start_ = now;
    target_selected_emitted_ = false;
    behavior_start_emitted_ = false;
    behavior_complete_emitted_ = false;
    behavior_tick_sample_emitted_ = false;
    execute_tick_count_ = 0;
    last_behavior_display_detail_.clear();
    previous_selection_.reset();
    circle_in_orbit_mode_ = false;
    circle_orbit_tracking_ = false;
    circle_previous_angle_rad_ = 0.0;
    circle_completed_orbits_ = 0.0;
    sequence_step_index_ = 0U;
    sequence_step_started_ = false;
    reset_sequence_step(now);
}

MissionTickOutput ObjectBehaviorMissionController::tick(const MissionTickInput& input) {
    MissionTickOutput output;
    output.state = state_;

    if (!mission_started_) {
        mission_started_ = true;
        mission_start_ = input.now;
        state_start_ = input.now;
        last_tick_time_ = input.now;
        home_pose_ = input.snapshot.ego.local_T_body;
        home_initialized_ = true;
        state_ = MissionLifecycleState::Prepare;
    }

    last_tick_time_ = input.now;

    const auto& ego = input.snapshot.ego;
    const double height_m = ego.height_valid ? ego.height_m : -ego.local_T_body.position.z;

    switch (state_) {
        case MissionLifecycleState::Prepare:
            if (auto camera_pointing = neutral_camera_pointing_command(
                    input.now,
                    config_.camera_pointing_prepare_mode)) {
                emit_camera_pointing(output, *camera_pointing);
            }
            if (input.finish_requested && ego.armed_valid && !ego.armed) {
                state_ = MissionLifecycleState::Complete;
                state_start_ = input.now;
                output.status = "finish_requested_before_arm";
            } else if (ego.armed_valid && ego.armed) {
                state_ = MissionLifecycleState::Takeoff;
                state_start_ = input.now;
                output.status = "armed_confirmed_by_ego";
            } else if (config_.arm_dispatch_fallback_s > 0.0 &&
                       last_result_matches_success(input.last_command_result, FlightCommandKind::Arm) &&
                       elapsed_at_least(arm_last_command_time_, input.now, config_.arm_dispatch_fallback_s)) {
                state_ = MissionLifecycleState::Takeoff;
                state_start_ = input.now;
                output.status = "arm_dispatch_ok_waiting_for_takeoff_height";
            } else if (elapsed_at_least(state_start_, input.now, config_.arm_timeout_s)) {
                if (ego.armed_valid && !ego.armed) {
                    state_ = MissionLifecycleState::Abort;
                    output.status = "abort";
                } else {
                    begin_abort_recovery(input.now, height_m, "arm_timeout");
                    output.status = "abort_recovery_start_arm_timeout";
                }
            } else if (!arm_command_sent_ || elapsed_at_least(arm_last_command_time_, input.now, config_.arm_retry_interval_s)) {
                arm_command_sent_ = true;
                arm_last_command_time_ = input.now;
                output.command = command_with_kind(input.now, FlightCommandKind::Arm);
                output.status = "arming";
            } else {
                output.status = ego.armed_valid ? "waiting_for_armed_state" : "waiting_for_armed_telemetry";
            }
            break;
        case MissionLifecycleState::Takeoff:
            if (auto camera_pointing = neutral_camera_pointing_command(
                    input.now,
                    config_.camera_pointing_takeoff_mode)) {
                emit_camera_pointing(output, *camera_pointing);
            }
            if (input.finish_requested) {
                state_ = height_m > kLandHeightM ? MissionLifecycleState::Land : MissionLifecycleState::Complete;
                state_start_ = input.now;
                output.status = height_m > kLandHeightM ? "finish_requested_land" : "finish_requested_complete";
            } else if (height_m >= config_.safe_height_m) {
                state_ = MissionLifecycleState::ExecuteMission;
                state_start_ = input.now;
                reset_behavior_run(input.now);
                output.status = "takeoff_complete";
            } else if (!takeoff_command_sent_) {
                takeoff_command_sent_ = true;
                takeoff_last_command_time_ = input.now;
                output.command = command_with_kind(input.now, FlightCommandKind::Takeoff);
                output.status = "takeoff_request";
            } else if (height_m >= kTakeoffVelocityAssistHeightM) {
                output.command = command_from_velocity(input.now, Vec3{0.0, 0.0, -std::abs(config_.takeoff_velocity_mps)});
                output.status = "takeoff_climb";
            } else if (elapsed_at_least(takeoff_last_command_time_, input.now, config_.takeoff_retry_interval_s)) {
                output.status = "waiting_for_takeoff_climb";
            } else {
                output.status = "waiting_for_takeoff_command_settle";
            }
            break;
        case MissionLifecycleState::ExecuteMission: {
            ++execute_tick_count_;
            auto selection = selector_.select(input.snapshot, config_.behavior_spec.target, previous_selection_);
            if (selection.selected) {
                previous_selection_ = selection;
                if (!target_selected_emitted_) {
                    target_selected_emitted_ = true;
                    output.events.push_back(target_event(selection));
                }
                if (!behavior_start_emitted_) {
                    behavior_start_emitted_ = true;
                    behavior_start_ = input.now;
                    if (sequence_active()) {
                        output.events.push_back(sequence_step_event("behavior_sequence_step_start", active_behavior(), sequence_step_index_, "sequence_start"));
                    }
                    output.events.push_back(behavior_event("behavior_start", "target_selected"));
                }
                const bool duration_complete = completion_elapsed(input.now);
                bool orbit_count_complete = false;
                TargetSelection control_selection = selection;
                if (config_.zero_target_velocity) {
                    control_selection.velocity_local = Vec3{0.0, 0.0, 0.0};
                }

                const BehaviorSpec& behavior = active_behavior();
                if (auto camera_pointing = camera_pointing_command_for_behavior(input.now, ego, control_selection, behavior)) {
                    emit_camera_pointing(output, *camera_pointing);
                }

                FollowGeometry geometry;
                Vec3 raw_velocity{0.0, 0.0, 0.0};
                if (!input.finish_requested && !duration_complete) {
                    if (behavior.type == BehaviorType::Circle) {
                        geometry = circle_geometry(
                            ego,
                            control_selection,
                            behavior,
                            config_,
                            circle_in_orbit_mode_);
                        const bool circling = geometry.circle_phase == "circling";
                        if (circling) {
                            circle_in_orbit_mode_ = true;
                        }
                        raw_velocity = clamp_velocity(
                            geometry.desired_velocity,
                            behavior.max_speed_mps,
                            behavior.max_vertical_speed_mps);
                        orbit_count_complete = update_circle_orbit_progress(behavior, circling, geometry.orbit_angle_rad);
                    } else {
                        raw_velocity = behavior_velocity(
                            ego,
                            control_selection,
                            behavior,
                            config_,
                            &geometry);
                        orbit_count_complete = update_circle_orbit_progress(behavior, geometry.circle_phase == "circling", geometry.orbit_angle_rad);
                    }
                    geometry.circle_completed_orbits = circle_completed_orbits_;
                    geometry.orbit_count_target = behavior.orbit_count;
                }

                const bool step_duration_complete =
                    sequence_active() && behavior.duration_s > 0.0 &&
                    elapsed_at_least(sequence_step_start_, input.now, behavior.duration_s);
                const bool approach_complete =
                    sequence_active() && behavior.type == BehaviorType::Approach &&
                    geometry.behavior_step_complete;
                const bool terminal_step =
                    sequence_active() && (
                        behavior.type == BehaviorType::GoHome ||
                        behavior.type == BehaviorType::GoHomeLand ||
                        behavior.type == BehaviorType::Land);
                const bool step_complete =
                    sequence_active() && (step_duration_complete || orbit_count_complete || approach_complete || terminal_step);

                if (step_complete && !active_behavior_is_last_sequence_step()) {
                    output.events.push_back(sequence_step_event(
                        "behavior_sequence_step_complete",
                        behavior,
                        sequence_step_index_,
                        terminal_step ? "terminal_step" : (orbit_count_complete ? "orbit_count_elapsed" : (approach_complete ? "approach_standoff_reached" : "duration_elapsed"))));
                    ++sequence_step_index_;
                    reset_sequence_step(input.now);
                    output.events.push_back(sequence_step_event(
                        "behavior_sequence_step_start",
                        active_behavior(),
                        sequence_step_index_,
                        "previous_step_complete"));
                    output.status = "object_behavior_sequence_step_complete";
                    break;
                }

                const bool sequence_complete =
                    step_complete && active_behavior_is_last_sequence_step();
                const ObjectBehaviorYawMode active_yaw_mode = yaw_mode_for_behavior(behavior);
                const std::string active_camera_pointing_mode = behavior.camera_pointing_mode.empty()
                    ? "target"
                    : behavior.camera_pointing_mode;
                if (input.finish_requested || duration_complete || orbit_count_complete || sequence_complete) {
                    if (!behavior_complete_emitted_) {
                        behavior_complete_emitted_ = true;
                        output.events.push_back(behavior_event(
                            "behavior_complete",
                            input.finish_requested ? "finish_requested" :
                                (sequence_complete ? "sequence_complete" :
                                    (orbit_count_complete ? "orbit_count_elapsed" : "duration_elapsed"))));
                    }
                    state_ = MissionLifecycleState::GoHome;
                    state_start_ = input.now;
                    output.status = input.finish_requested ? "object_behavior_finish_requested" : "object_behavior_complete";
                } else {
                    const Vec3 velocity = apply_altitude_policy(
                        raw_velocity,
                        config_,
                        behavior,
                        height_m);
                    output.command = command_from_behavior_velocity(
                        input.now,
                        velocity,
                        ego,
                        control_selection,
                        config_.yaw_offset_rad + behavior.yaw_offset_rad,
                        active_yaw_mode);
                    const std::string behavior_detail =
                        behavior_detail_for_tick(behavior, geometry);
                    if (!behavior_tick_sample_emitted_ || behavior_detail != last_behavior_display_detail_) {
                        behavior_tick_sample_emitted_ = true;
                        last_behavior_display_detail_ = behavior_detail;
                        output.events.push_back(behavior_tick_event(
                            config_.behavior_spec,
                            behavior,
                            control_selection,
                            velocity,
                            geometry,
                            sequence_active()
                                ? std::optional<std::size_t>{sequence_step_index_}
                                : std::nullopt,
                            sequence_active() ? config_.behavior_spec.behavior.steps.size() : 0U,
                            active_yaw_mode,
                            active_camera_pointing_mode));
                    }
                    if (config_.debug_every_n_ticks > 0 && execute_tick_count_ % config_.debug_every_n_ticks == 0) {
                        output.events.push_back(behavior_debug_event(
                            execute_tick_count_,
                            config_.debug_level,
                            ego,
                            control_selection,
                            raw_velocity,
                            velocity,
                            *output.command,
                            geometry,
                            config_.yaw_min_speed_mps));
                    }
                    output.status = object_behavior_status(behavior, geometry);
                }
            } else if (input.finish_requested) {
                state_ = MissionLifecycleState::GoHome;
                state_start_ = input.now;
                output.status = "object_behavior_finish_requested_no_target";
            } else {
                output.command = command_from_velocity(input.now, Vec3{0.0, 0.0, 0.0});
                output.status = "object_behavior_waiting_for_target_" + to_string(selection.status);
            }
            break;
        }
        case MissionLifecycleState::GoHome: {
            if (home_initialized_) {
                if (auto camera_pointing = camera_pointing_command_to_point(
                        input.now,
                        ego,
                        home_pose_.position,
                        config_.camera_pointing_go_home_mode)) {
                    emit_camera_pointing(output, *camera_pointing);
                }
            } else if (auto camera_pointing = neutral_camera_pointing_command(
                           input.now,
                           config_.camera_pointing_complete_mode)) {
                emit_camera_pointing(output, *camera_pointing);
            }

            const Vec3 raw_velocity = go_home_velocity(ego);
            const Vec3 velocity = apply_altitude_policy(
                raw_velocity,
                config_,
                config_.behavior_spec.behavior,
                height_m);
            if (norm_xy(velocity) <= 0.0 &&
                (config_.altitude_policy != ObjectBehaviorAltitudePolicy::SafeHeightFloor || height_m >= config_.safe_height_m)) {
                state_ = MissionLifecycleState::Land;
                state_start_ = input.now;
                output.status = aborting_ ? "abort_recovery_home_reached" : "home_reached";
            } else {
                output.command = command_from_velocity(input.now, velocity, config_.yaw_offset_rad);
                output.status = aborting_ ? "abort_recovery_go_home" : "go_home";
            }
            break;
        }
        case MissionLifecycleState::Land:
            if (home_initialized_) {
                if (auto camera_pointing = camera_pointing_command_to_point(
                        input.now,
                        ego,
                        home_pose_.position,
                        config_.camera_pointing_land_mode)) {
                    emit_camera_pointing(output, *camera_pointing);
                }
            }

            if (height_m <= kLandHeightM) {
                state_ = MissionLifecycleState::Complete;
                state_start_ = input.now;
                output.status = aborting_ ? "abort_recovery_landed" : "landed";
            } else if (!land_command_sent_) {
                land_command_sent_ = true;
                land_last_command_time_ = input.now;
                output.command = command_with_kind(input.now, FlightCommandKind::Land);
                output.status = aborting_ ? "abort_recovery_landing_command_sent" : "landing_command_sent";
            } else if (elapsed_at_least(land_last_command_time_, input.now, config_.land_timeout_s)) {
                state_ = MissionLifecycleState::Abort;
                output.status = "abort";
            } else {
                output.status = aborting_ ? "abort_recovery_waiting_for_landed_telemetry" : "waiting_for_landed_telemetry";
            }
            break;
        case MissionLifecycleState::Complete:
            if (auto camera_pointing = neutral_camera_pointing_command(input.now, config_.camera_pointing_complete_mode)) {
                emit_camera_pointing(output, *camera_pointing);
            }

            if (ego.armed_valid && !ego.armed) {
                if (aborting_) {
                    state_ = MissionLifecycleState::Abort;
                    output.status = "abort";
                } else {
                    output.status = "complete";
                }
            } else if (elapsed_at_least(state_start_, input.now, config_.disarm_timeout_s)) {
                state_ = MissionLifecycleState::Abort;
                output.status = "abort";
            } else if (!disarm_command_sent_ || elapsed_at_least(disarm_last_command_time_, input.now, config_.disarm_retry_interval_s)) {
                disarm_command_sent_ = true;
                disarm_last_command_time_ = input.now;
                output.command = command_with_kind(input.now, FlightCommandKind::Disarm);
                output.status = aborting_ ? "abort_recovery_disarming" : "disarming";
            } else {
                output.status = aborting_
                    ? (ego.armed_valid ? "abort_recovery_waiting_for_disarmed_state" : "abort_recovery_waiting_for_disarmed_telemetry")
                    : (ego.armed_valid ? "waiting_for_disarmed_state" : "waiting_for_disarmed_telemetry");
            }
            break;
        case MissionLifecycleState::Abort:
            output.status = "abort";
            break;
        case MissionLifecycleState::Idle:
        default:
            state_ = MissionLifecycleState::Prepare;
            state_start_ = input.now;
            output.status = "idle";
            break;
    }

    output.state = state_;
    return output;
}

}  // namespace dedalus
