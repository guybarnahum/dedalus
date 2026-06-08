#pragma once

#include <string>

#include "dedalus/behavior/object_behavior_mission_controller.hpp"

// forward declarations come via object_behavior_mission_controller.hpp which
// includes mission_controller.hpp (EgoState via world_snapshot.hpp, Vec3 via
// types.hpp) and target_selector.hpp (TargetSelection) and behavior_spec.hpp.

namespace dedalus {

// ---- Follow/orbit geometry observation state --------------------------------
//
// Populated by the geometry computation functions below and used by
// ObjectBehaviorMissionController::tick() for velocity output and OSD events.

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
    bool altitude_profile_active{false};
    std::string altitude_profile_easing{"none"};
    double altitude_profile_t{0.0};
    double desired_height_m{0.0};
    double current_height_m{0.0};
    double height_error_m{0.0};
};

// ---- Geometry computation functions -----------------------------------------
//
// These are called from ObjectBehaviorMissionController::tick() and are
// implemented in follow_geometry_policy.cpp.  Internal helpers used only
// within that translation unit live in its anonymous namespace.

Vec3 clamp_velocity(
    const Vec3& desired,
    double max_horizontal_mps,
    double max_vertical_mps);

Vec3 apply_altitude_policy(
    Vec3 velocity,
    const ObjectBehaviorMissionConfig& config,
    const BehaviorSpec& behavior,
    double height_m);

Vec3 enforce_takeoff_height_floor(
    Vec3 velocity,
    double height_m,
    double takeoff_height_m,
    double max_vertical_speed_mps);

FollowGeometry circle_geometry(
    const EgoState& ego,
    const TargetSelection& selection,
    const BehaviorSpec& behavior,
    const ObjectBehaviorMissionConfig& config,
    bool orbit_mode_latched = false);

Vec3 apply_altitude_profile(
    Vec3 velocity,
    const EgoState& ego,
    const BehaviorSpec& behavior,
    double elapsed_s,
    FollowGeometry& geometry);

Vec3 behavior_velocity(
    const EgoState& ego,
    const TargetSelection& selection,
    const BehaviorSpec& behavior,
    const ObjectBehaviorMissionConfig& config,
    FollowGeometry* geometry_out = nullptr);

}  // namespace dedalus
