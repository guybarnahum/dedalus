#include "dedalus/visualization/world_to_image_projector.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace dedalus {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kDegToRad = kPi / 180.0;

bool finite_vec3(const Vec3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool valid_intrinsics(const WorldImageCameraIntrinsics& intrinsics) {
    return intrinsics.width > 0 && intrinsics.height > 0 && intrinsics.fx > 0.0 && intrinsics.fy > 0.0 &&
           std::isfinite(intrinsics.fx) && std::isfinite(intrinsics.fy) && std::isfinite(intrinsics.cx) &&
           std::isfinite(intrinsics.cy) && std::isfinite(intrinsics.near_plane_m) && intrinsics.near_plane_m > 0.0;
}

Vec3 subtract(const Vec3& lhs, const Vec3& rhs) {
    return Vec3{lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}


double norm(const Vec3& value) {
    return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

struct Mat3 {
    double m[3][3]{};
};

Mat3 rotation_body_from_local(const Vec3& rpy) {
    const double roll = rpy.x;
    const double pitch = rpy.y;
    const double yaw = rpy.z;

    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);

    // R_local_from_body = Rz(yaw) * Ry(pitch) * Rx(roll). We need its transpose.
    Mat3 r{};
    r.m[0][0] = cy * cp;
    r.m[0][1] = sy * cp;
    r.m[0][2] = -sp;

    r.m[1][0] = cy * sp * sr - sy * cr;
    r.m[1][1] = sy * sp * sr + cy * cr;
    r.m[1][2] = cp * sr;

    r.m[2][0] = cy * sp * cr + sy * sr;
    r.m[2][1] = sy * sp * cr - cy * sr;
    r.m[2][2] = cp * cr;
    return r;
}

Mat3 rotation_camera_from_body(const Vec3& rpy) {
    // Same convention as above: body_T_camera.rotation_rpy describes camera axes in body frame,
    // so camera_from_body is the transpose of body_from_camera.
    return rotation_body_from_local(rpy);
}

Vec3 multiply(const Mat3& matrix, const Vec3& value) {
    return Vec3{
        matrix.m[0][0] * value.x + matrix.m[0][1] * value.y + matrix.m[0][2] * value.z,
        matrix.m[1][0] * value.x + matrix.m[1][1] * value.y + matrix.m[1][2] * value.z,
        matrix.m[2][0] * value.x + matrix.m[2][1] * value.y + matrix.m[2][2] * value.z,
    };
}

ProjectedWorldPoint invalid_result(std::string reason) {
    ProjectedWorldPoint result;
    result.visible = false;
    result.reason = std::move(reason);
    return result;
}

}  // namespace

WorldImageCameraIntrinsics pinhole_intrinsics_from_horizontal_fov(
    int width,
    int height,
    double horizontal_fov_deg,
    double near_plane_m) {
    if (width <= 0 || height <= 0) {
        throw std::invalid_argument("camera intrinsics require positive image dimensions");
    }
    if (!std::isfinite(horizontal_fov_deg) || horizontal_fov_deg <= 0.0 || horizontal_fov_deg >= 180.0) {
        throw std::invalid_argument("camera horizontal_fov_deg must be in (0, 180)");
    }
    if (!std::isfinite(near_plane_m) || near_plane_m <= 0.0) {
        throw std::invalid_argument("camera near_plane_m must be positive");
    }

    const double horizontal_fov_rad = horizontal_fov_deg * kDegToRad;
    const double fx = (static_cast<double>(width) * 0.5) / std::tan(horizontal_fov_rad * 0.5);
    return WorldImageCameraIntrinsics{
        .width = width,
        .height = height,
        .fx = fx,
        .fy = fx,
        .cx = (static_cast<double>(width) - 1.0) * 0.5,
        .cy = (static_cast<double>(height) - 1.0) * 0.5,
        .near_plane_m = near_plane_m,
    };
}

ProjectedWorldPoint project_local_point_to_image(
    const Vec3& point_local,
    const EgoState& ego,
    const WorldToImageProjectionConfig& config) {
    if (!finite_vec3(point_local) || !finite_vec3(ego.local_T_body.position) ||
        !finite_vec3(ego.local_T_body.rotation_rpy) || !valid_intrinsics(config.intrinsics) ||
        !finite_vec3(config.extrinsics.body_T_camera.position) ||
        !finite_vec3(config.extrinsics.body_T_camera.rotation_rpy)) {
        return invalid_result("invalid_input");
    }

    const Vec3 position_ego_relative = subtract(point_local, ego.local_T_body.position);
    const Mat3 body_from_local = rotation_body_from_local(ego.local_T_body.rotation_rpy);
    const Vec3 position_body = multiply(body_from_local, position_ego_relative);

    const Mat3 camera_from_body = rotation_camera_from_body(config.extrinsics.body_T_camera.rotation_rpy);
    const Vec3 body_relative_to_camera_origin = subtract(position_body, config.extrinsics.body_T_camera.position);
    const Vec3 position_camera = multiply(camera_from_body, body_relative_to_camera_origin);

    ProjectedWorldPoint result;
    result.position_ego_relative = position_ego_relative;
    result.position_body = position_body;
    result.position_camera = position_camera;
    result.range_m = norm(position_ego_relative);
    result.bearing_deg = std::atan2(position_body.y, position_body.x) * kRadToDeg;
    result.depth_m = position_camera.x;

    if (position_camera.x <= config.intrinsics.near_plane_m) {
        result.reason = "behind_camera";
        return result;
    }

    result.u_px = config.intrinsics.fx * (position_camera.y / position_camera.x) + config.intrinsics.cx;
    result.v_px = config.intrinsics.fy * (position_camera.z / position_camera.x) + config.intrinsics.cy;

    if (!std::isfinite(result.u_px) || !std::isfinite(result.v_px)) {
        result.reason = "invalid_projection";
        return result;
    }

    if (result.u_px < 0.0 || result.u_px > static_cast<double>(config.intrinsics.width - 1) ||
        result.v_px < 0.0 || result.v_px > static_cast<double>(config.intrinsics.height - 1)) {
        result.reason = "outside_image";
        return result;
    }

    result.visible = true;
    result.reason = "visible";
    return result;
}

}  // namespace dedalus
