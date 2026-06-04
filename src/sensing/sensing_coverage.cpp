#include "dedalus/sensing/sensing_coverage.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace dedalus {
namespace {

constexpr double kEpsilon = 1.0e-9;

struct Mat3 {
    double m[3][3]{};
};

Vec3 add(const Vec3& lhs, const Vec3& rhs) {
    return Vec3{lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

Vec3 subtract(const Vec3& lhs, const Vec3& rhs) {
    return Vec3{lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

Vec3 scale(const Vec3& value, double scalar) {
    return Vec3{value.x * scalar, value.y * scalar, value.z * scalar};
}

double dot(const Vec3& lhs, const Vec3& rhs) {
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

double norm(const Vec3& value) {
    return std::sqrt(dot(value, value));
}

Vec3 normalize_or(const Vec3& value, const Vec3& fallback) {
    const double length = norm(value);
    if (length <= kEpsilon) {
        return fallback;
    }
    return scale(value, 1.0 / length);
}

Mat3 multiply(const Mat3& lhs, const Mat3& rhs) {
    Mat3 out;
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            out.m[row][col] = lhs.m[row][0] * rhs.m[0][col] + lhs.m[row][1] * rhs.m[1][col] + lhs.m[row][2] * rhs.m[2][col];
        }
    }
    return out;
}

Vec3 transform(const Mat3& matrix, const Vec3& value) {
    return Vec3{
        matrix.m[0][0] * value.x + matrix.m[0][1] * value.y + matrix.m[0][2] * value.z,
        matrix.m[1][0] * value.x + matrix.m[1][1] * value.y + matrix.m[1][2] * value.z,
        matrix.m[2][0] * value.x + matrix.m[2][1] * value.y + matrix.m[2][2] * value.z};
}

Mat3 rotation_x(double radians) {
    const double c = std::cos(radians);
    const double s = std::sin(radians);
    return Mat3{{{1.0, 0.0, 0.0}, {0.0, c, -s}, {0.0, s, c}}};
}

Mat3 rotation_y(double radians) {
    const double c = std::cos(radians);
    const double s = std::sin(radians);
    return Mat3{{{c, 0.0, s}, {0.0, 1.0, 0.0}, {-s, 0.0, c}}};
}

Mat3 rotation_z(double radians) {
    const double c = std::cos(radians);
    const double s = std::sin(radians);
    return Mat3{{{c, -s, 0.0}, {s, c, 0.0}, {0.0, 0.0, 1.0}}};
}

Mat3 rotation_from_rpy(const Vec3& rpy) {
    return multiply(rotation_z(rpy.z), multiply(rotation_y(rpy.y), rotation_x(rpy.x)));
}

CameraSensingConfig default_config_for_frame(const FramePacket& frame) {
    CameraSensingConfig config;
    config.camera_id = frame.camera_id.value.empty() ? CameraId{"front_center"} : frame.camera_id;
    config.camera_name = config.camera_id.value.empty() ? "front_center" : config.camera_id.value;
    return config;
}

bool camera_matches(const CameraSensingConfig& config, const CameraId& camera_id) {
    if (camera_id.value.empty()) {
        return config.camera_id.value.empty() || config.camera_name == "front_center";
    }
    return config.camera_id.value == camera_id.value || config.camera_name == camera_id.value;
}

const CameraSensingConfig& find_config_or_default(
    const std::vector<CameraSensingConfig>& configs,
    const FramePacket& frame,
    CameraSensingConfig& fallback) {
    for (const auto& config : configs) {
        if (camera_matches(config, frame.camera_id)) {
            return config;
        }
    }
    fallback = default_config_for_frame(frame);
    return fallback;
}

CameraPointingState find_pointing_state(
    const CameraSensingConfig& config,
    const std::vector<CameraPointingState>& pointing_states) {
    for (const auto& state : pointing_states) {
        if (state.camera_id.value == config.camera_id.value || state.camera_name == config.camera_name) {
            return state;
        }
    }
    CameraPointingState neutral;
    neutral.camera_id = config.camera_id;
    neutral.camera_name = config.camera_name;
    neutral.source = "neutral";
    neutral.valid = false;
    return neutral;
}

}  // namespace

SensingCoverageProvider::SensingCoverageProvider(std::vector<CameraSensingConfig> camera_configs)
    : camera_configs_(std::move(camera_configs)) {}

SensingCoverageSnapshot SensingCoverageProvider::snapshot(
    const std::vector<FramePacket>& frames,
    const EgoState& ego,
    const std::vector<CameraPointingState>& pointing_states) const {
    SensingCoverageSnapshot coverage;
    coverage.timestamp = ego.timestamp;
    coverage.map_frame_id = ego.map_frame_id;
    coverage.camera_volumes.reserve(frames.size());
    for (const auto& frame : frames) {
        coverage.camera_volumes.push_back(volume_for_frame(frame, ego, pointing_states));
    }
    return coverage;
}

CameraSensingVolume SensingCoverageProvider::volume_for_frame(
    const FramePacket& frame,
    const EgoState& ego,
    const std::vector<CameraPointingState>& pointing_states) const {
    CameraSensingConfig fallback;
    const auto& config = find_config_or_default(camera_configs_, frame, fallback);
    auto pointing_state = find_pointing_state(config, pointing_states);
    if (pointing_state.timestamp.timestamp_ns == 0) {
        pointing_state.timestamp = frame.timestamp.timestamp_ns != 0 ? frame.timestamp : ego.timestamp;
    }

    const Mat3 map_R_body = rotation_from_rpy(ego.local_T_body.rotation_rpy);
    const Mat3 body_R_mount = rotation_from_rpy(config.body_T_camera_rpy_rad);
    const Mat3 mount_R_pointing = rotation_from_rpy(Vec3{pointing_state.roll_rad, pointing_state.pitch_rad, pointing_state.yaw_rad});
    const Mat3 body_R_camera = multiply(body_R_mount, mount_R_pointing);
    const Mat3 map_R_camera = multiply(map_R_body, body_R_camera);

    const Vec3 camera_origin_body = config.body_T_camera_xyz_m;
    const Vec3 camera_origin_map = add(ego.local_T_body.position, transform(map_R_body, camera_origin_body));

    CameraSensingVolume volume;
    volume.timestamp = frame.timestamp.timestamp_ns != 0 ? frame.timestamp : ego.timestamp;
    volume.frame_id = frame.frame_id;
    volume.has_frame_id = !frame.frame_id.value.empty();
    volume.camera_id = config.camera_id;
    volume.camera_name = config.camera_name;
    volume.role = config.role;
    volume.map_frame_id = ego.map_frame_id;
    volume.body_T_camera_mount.position = config.body_T_camera_xyz_m;
    volume.body_T_camera_mount.rotation_rpy = config.body_T_camera_rpy_rad;
    volume.body_T_camera_current.position = config.body_T_camera_xyz_m;
    volume.body_T_camera_current.rotation_rpy = Vec3{
        config.body_T_camera_rpy_rad.x + pointing_state.roll_rad,
        config.body_T_camera_rpy_rad.y + pointing_state.pitch_rad,
        config.body_T_camera_rpy_rad.z + pointing_state.yaw_rad};
    volume.map_T_camera_current.position = camera_origin_map;
    volume.map_T_camera_current.rotation_rpy = Vec3{
        ego.local_T_body.rotation_rpy.x + volume.body_T_camera_current.rotation_rpy.x,
        ego.local_T_body.rotation_rpy.y + volume.body_T_camera_current.rotation_rpy.y,
        ego.local_T_body.rotation_rpy.z + volume.body_T_camera_current.rotation_rpy.z};
    volume.intrinsics = frame.intrinsics;
    volume.horizontal_fov_rad = config.horizontal_fov_rad;
    volume.vertical_fov_rad = config.vertical_fov_rad;
    volume.near_range_m = config.near_range_m;
    volume.far_range_m = config.far_range_m;
    volume.min_reliable_range_m = config.min_reliable_range_m;
    volume.max_reliable_range_m = config.max_reliable_range_m;
    volume.origin_local = camera_origin_map;
    volume.forward_axis_local = normalize_or(transform(map_R_camera, Vec3{1.0, 0.0, 0.0}), Vec3{1.0, 0.0, 0.0});
    volume.right_axis_local = normalize_or(transform(map_R_camera, Vec3{0.0, 1.0, 0.0}), Vec3{0.0, 1.0, 0.0});
    volume.up_axis_local = normalize_or(transform(map_R_camera, Vec3{0.0, 0.0, -1.0}), Vec3{0.0, 0.0, -1.0});
    volume.pointing_state = pointing_state;
    return volume;
}

SensingVolumeQueryResult query_point_in_camera_sensing_volume(
    const CameraSensingVolume& volume,
    const Vec3& point_local) {
    SensingVolumeQueryResult result;
    const Vec3 delta = subtract(point_local, volume.origin_local);
    const Vec3 forward = normalize_or(volume.forward_axis_local, Vec3{1.0, 0.0, 0.0});
    const Vec3 right = normalize_or(volume.right_axis_local, Vec3{0.0, 1.0, 0.0});
    const Vec3 up = normalize_or(volume.up_axis_local, Vec3{0.0, 0.0, -1.0});

    result.forward_m = dot(delta, forward);
    result.right_m = dot(delta, right);
    result.up_m = dot(delta, up);
    result.range_m = norm(delta);

    const double lateral_norm = std::sqrt(result.forward_m * result.forward_m + result.right_m * result.right_m);
    result.bearing_rad = std::atan2(result.right_m, result.forward_m);
    result.elevation_rad = std::atan2(result.up_m, lateral_norm);

    if (result.forward_m <= kEpsilon || result.range_m < volume.near_range_m || result.range_m > volume.far_range_m) {
        result.inside = false;
        return result;
    }

    const double half_width_at_forward = std::tan(volume.horizontal_fov_rad * 0.5) * result.forward_m;
    const double half_height_at_forward = std::tan(volume.vertical_fov_rad * 0.5) * result.forward_m;
    result.inside = std::abs(result.right_m) <= half_width_at_forward && std::abs(result.up_m) <= half_height_at_forward;
    return result;
}

ObstacleSensingVolume to_obstacle_sensing_volume(const CameraSensingVolume& volume) {
    ObstacleSensingVolume out;
    out.timestamp = volume.timestamp;
    out.source_frame_id = volume.frame_id;
    out.has_source_frame = volume.has_frame_id;
    out.sensor_name = volume.camera_name;
    out.provider_name = volume.pointing_state.valid ? volume.pointing_state.source : "configured_camera_coverage";
    out.map_frame_id = volume.map_frame_id;
    out.origin_local = volume.origin_local;
    out.forward_axis_local = volume.forward_axis_local;
    out.right_axis_local = volume.right_axis_local;
    out.up_axis_local = volume.up_axis_local;
    out.near_range_m = static_cast<float>(volume.near_range_m);
    out.far_range_m = static_cast<float>(volume.far_range_m);
    out.horizontal_fov_rad = static_cast<float>(volume.horizontal_fov_rad);
    out.vertical_fov_rad = static_cast<float>(volume.vertical_fov_rad);
    out.min_reliable_range_m = static_cast<float>(volume.min_reliable_range_m);
    out.max_reliable_range_m = static_cast<float>(volume.max_reliable_range_m);
    out.min_surface_area_m2 = 0.0F;
    out.min_angular_size_rad = 0.0F;
    out.min_confidence = 0.0F;
    return out;
}

}  // namespace dedalus
