#pragma once

#include <cstddef>
#include <optional>
#include <string>

#include "dedalus/behavior/follow_geometry_policy.hpp"

// Provides the types used in signatures below via the include chain:
//   follow_geometry_policy.hpp
//     -> object_behavior_mission_controller.hpp
//          -> behavior_spec.hpp      (BehaviorSpec, BehaviorMissionSpec)
//          -> mission_controller.hpp (ControllerEvent, ControllerEventKind,
//                                     VelocityCommand, EgoState via
//                                     world_snapshot.hpp)
//          -> target_selector.hpp    (TargetSelection)
//     -> FollowGeometry, ObjectBehaviorYawMode

namespace dedalus {

// ---- Display-state string helpers ------------------------------------------

std::string behavior_display_fields(const std::string& detail);

std::string behavior_detail_for_event(ControllerEventKind kind);

std::string behavior_detail_for_tick(
    const BehaviorSpec& behavior,
    const FollowGeometry& geometry);

std::string object_behavior_status(
    const BehaviorSpec& behavior,
    const FollowGeometry& geometry);

// ---- OSD controller events -------------------------------------------------

ControllerEvent behavior_tick_event(
    const BehaviorMissionSpec& spec,
    const BehaviorSpec& active_behavior,
    const TargetSelection& selection,
    const Vec3& velocity,
    const FollowGeometry& geometry,
    const std::optional<std::size_t>& sequence_step_index,
    std::size_t sequence_step_count,
    ObjectBehaviorYawMode yaw_mode,
    const std::string& camera_pointing_mode);

ControllerEvent behavior_debug_event(
    int execute_tick,
    int debug_level,
    const EgoState& ego,
    const TargetSelection& selection,
    const Vec3& raw_velocity,
    const Vec3& final_velocity,
    const VelocityCommand& command,
    const FollowGeometry& geometry);

}  // namespace dedalus
