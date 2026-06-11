#include "dedalus/geometry/pose_transform.hpp"

#include <cmath>

namespace dedalus {
namespace {

struct Matrix3 {
    double m[3][3]{};
};

Matrix3 rotation_matrix_from_rpy(const Vec3& rpy) {
    const double roll = rpy.x;
    const double pitch = rpy.y;
    const double yaw = rpy.z;

    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);

    Matrix3 r;
    r.m[0][0] = cy * cp;
    r.m[0][1] = (cy * sp * sr) - (sy * cr);
    r.m[0][2] = (cy * sp * cr) + (sy * sr);

    r.m[1][0] = sy * cp;
    r.m[1][1] = (sy * sp * sr) + (cy * cr);
    r.m[1][2] = (sy * sp * cr) - (cy * sr);

    r.m[2][0] = -sp;
    r.m[2][1] = cp * sr;
    r.m[2][2] = cp * cr;
    return r;
}

Vec3 multiply(const Matrix3& r, const Vec3& v) {
    return Vec3{
        (r.m[0][0] * v.x) + (r.m[0][1] * v.y) + (r.m[0][2] * v.z),
        (r.m[1][0] * v.x) + (r.m[1][1] * v.y) + (r.m[1][2] * v.z),
        (r.m[2][0] * v.x) + (r.m[2][1] * v.y) + (r.m[2][2] * v.z),
    };
}

Vec3 multiply_transpose(const Matrix3& r, const Vec3& v) {
    return Vec3{
        (r.m[0][0] * v.x) + (r.m[1][0] * v.y) + (r.m[2][0] * v.z),
        (r.m[0][1] * v.x) + (r.m[1][1] * v.y) + (r.m[2][1] * v.z),
        (r.m[0][2] * v.x) + (r.m[1][2] * v.y) + (r.m[2][2] * v.z),
    };
}

Vec3 add(const Vec3& a, const Vec3& b) {
    return Vec3{a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 subtract(const Vec3& a, const Vec3& b) {
    return Vec3{a.x - b.x, a.y - b.y, a.z - b.z};
}

double normalize_angle(double angle) {
    constexpr double kPi = 3.141592653589793238462643383279502884;
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

}  // namespace

Vec3 rotate_vector(const Pose3& parent_T_child, const Vec3& vector_child) {
    return multiply(rotation_matrix_from_rpy(parent_T_child.rotation_rpy), vector_child);
}

Vec3 inverse_rotate_vector(const Pose3& parent_T_child, const Vec3& vector_parent) {
    return multiply_transpose(rotation_matrix_from_rpy(parent_T_child.rotation_rpy), vector_parent);
}

Vec3 transform_point(const Pose3& parent_T_child, const Vec3& point_child) {
    return add(parent_T_child.position, rotate_vector(parent_T_child, point_child));
}

Vec3 inverse_transform_point(const Pose3& parent_T_child, const Vec3& point_parent) {
    return inverse_rotate_vector(parent_T_child, subtract(point_parent, parent_T_child.position));
}

Pose3 inverse_pose(const Pose3& parent_T_child) {
    Pose3 child_T_parent;
    child_T_parent.rotation_rpy = Vec3{
        -parent_T_child.rotation_rpy.x,
        -parent_T_child.rotation_rpy.y,
        -parent_T_child.rotation_rpy.z,
    };
    child_T_parent.position = inverse_rotate_vector(parent_T_child, Vec3{
        -parent_T_child.position.x,
        -parent_T_child.position.y,
        -parent_T_child.position.z,
    });
    return child_T_parent;
}

Pose3 compose_pose(const Pose3& parent_T_mid, const Pose3& mid_T_child) {
    Pose3 parent_T_child;
    parent_T_child.position = transform_point(parent_T_mid, mid_T_child.position);

    // The first mission-local mapping slices only need yaw-stable composition.
    // Keep roll/pitch additive for small-angle camera/body mounts and make yaw explicit.
    parent_T_child.rotation_rpy = Vec3{
        parent_T_mid.rotation_rpy.x + mid_T_child.rotation_rpy.x,
        parent_T_mid.rotation_rpy.y + mid_T_child.rotation_rpy.y,
        normalize_angle(parent_T_mid.rotation_rpy.z + mid_T_child.rotation_rpy.z),
    };
    return parent_T_child;
}

double yaw_rad(const Pose3& pose) {
    return pose.rotation_rpy.z;
}

Pose3 make_yaw_pose(const Vec3& position, const double yaw) {
    Pose3 pose;
    pose.position = position;
    pose.rotation_rpy = Vec3{0.0, 0.0, yaw};
    return pose;
}

}  // namespace dedalus
