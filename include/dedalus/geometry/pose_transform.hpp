#pragma once

#include "dedalus/core/types.hpp"

namespace dedalus {

// Pose3 represents parent_T_child:
//   point_parent = transform_point(parent_T_child, point_child)
//
// rotation_rpy is interpreted as roll, pitch, yaw in radians, applied as
// Rz(yaw) * Ry(pitch) * Rx(roll). This matches the mission-local mapping
// need without introducing a duplicate pose contract.
Vec3 rotate_vector(const Pose3& parent_T_child, const Vec3& vector_child);
Vec3 inverse_rotate_vector(const Pose3& parent_T_child, const Vec3& vector_parent);

Vec3 transform_point(const Pose3& parent_T_child, const Vec3& point_child);
Vec3 inverse_transform_point(const Pose3& parent_T_child, const Vec3& point_parent);

Pose3 inverse_pose(const Pose3& parent_T_child);
Pose3 compose_pose(const Pose3& parent_T_mid, const Pose3& mid_T_child);

double yaw_rad(const Pose3& pose);
Pose3 make_yaw_pose(const Vec3& position, double yaw_rad);

}  // namespace dedalus
