#include "dedalus/sensing/airsim_depth_obstacle_detector.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace dedalus {
namespace {

Vec3 add(const Vec3& lhs, const Vec3& rhs) {
    return Vec3{lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

Vec3 scale(const Vec3& value, double scalar) {
    return Vec3{value.x * scalar, value.y * scalar, value.z * scalar};
}

Vec3 subtract(const Vec3& lhs, const Vec3& rhs) {
    return Vec3{lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

Vec3 cross(const Vec3& lhs, const Vec3& rhs) {
    return Vec3{lhs.y * rhs.z - lhs.z * rhs.y, lhs.z * rhs.x - lhs.x * rhs.z, lhs.x * rhs.y - lhs.y * rhs.x};
}

double norm_xy(double x, double y) {
    return std::sqrt(x * x + y * y);
}

double norm3(const Vec3& value) {
    return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

Vec3 normalize_or_zero(const Vec3& value) {
    const double length = norm3(value);
    if (!std::isfinite(length) || length <= 1.0e-6) {
        return Vec3{};
    }
    return Vec3{value.x / length, value.y / length, value.z / length};
}

Vec3 zero_vec3() {
    return Vec3{0.0, 0.0, 0.0};
}

bool finite_positive(float value) {
    return std::isfinite(value) && value > 0.0F;
}

float clamp_positive(float value, float fallback) {
    return value > 0.0F ? value : fallback;
}

struct ProjectedDepthSample {
    bool valid{false};
    Vec3 center_local;
    double forward_m{0.0};
    double right_m{0.0};
    double up_m{0.0};
    float range_m{0.0F};
    float bearing_rad{0.0F};
    float elevation_rad{0.0F};
};

const ProjectedDepthSample* projected_at(
    const std::vector<ProjectedDepthSample>& samples,
    int width,
    int height,
    int x,
    int y) {
    if (x < 0 || y < 0 || x >= width || y >= height) {
        return nullptr;
    }
    const auto index = static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
                       static_cast<std::size_t>(x);
    const auto& sample = samples[index];
    return sample.valid ? &sample : nullptr;
}

bool depth_derived_surface_normal(
    const std::vector<ProjectedDepthSample>& samples,
    int width,
    int height,
    int x,
    int y,
    Vec3& normal_local) {
    const auto* center = projected_at(samples, width, height, x, y);
    if (center == nullptr) {
        return false;
    }

    const auto* right = projected_at(samples, width, height, x + 1, y);
    const auto* left  = projected_at(samples, width, height, x - 1, y);
    const auto* down  = projected_at(samples, width, height, x, y + 1);
    const auto* up    = projected_at(samples, width, height, x, y - 1);

    Vec3 tangent_x{};
    bool has_tangent_x = false;
    if (right != nullptr) {
        tangent_x = subtract(right->center_local, center->center_local);
        has_tangent_x = true;
    } else if (left != nullptr) {
        tangent_x = subtract(center->center_local, left->center_local);
        has_tangent_x = true;
    }

    Vec3 tangent_y{};
    bool has_tangent_y = false;
    if (down != nullptr) {
        tangent_y = subtract(down->center_local, center->center_local);
        has_tangent_y = true;
    } else if (up != nullptr) {
        tangent_y = subtract(center->center_local, up->center_local);
        has_tangent_y = true;
    }

    if (!has_tangent_x || !has_tangent_y) {
        return false;
    }

    normal_local = normalize_or_zero(cross(tangent_x, tangent_y));
    return norm3(normal_local) > 0.0;
}

}  // namespace

std::vector<ObstacleEvidence> detect_airsim_depth_obstacles(
    const AirSimDepthFrame& frame,
    const ObstacleSensingVolume& sensing_volume,
    const AirSimDepthObstacleDetectorConfig& config) {
    std::vector<ObstacleEvidence> evidence;

    if (frame.width <= 0 || frame.height <= 0) {
        return evidence;
    }
    const auto expected_size = static_cast<std::size_t>(frame.width) * static_cast<std::size_t>(frame.height);
    if (frame.depth_m.size() < expected_size) {
        return evidence;
    }
    if (sensing_volume.far_range_m <= sensing_volume.near_range_m ||
        sensing_volume.horizontal_fov_rad <= 0.0F ||
        sensing_volume.vertical_fov_rad <= 0.0F) {
        return evidence;
    }

    const std::size_t stride = std::max<std::size_t>(1U, config.pixel_stride);
    const float min_depth = std::max(config.min_depth_m, sensing_volume.min_reliable_range_m);
    const float max_depth = std::min(
        config.max_depth_m > 0.0F ? config.max_depth_m : sensing_volume.far_range_m,
        sensing_volume.max_reliable_range_m > 0.0F ? sensing_volume.max_reliable_range_m : sensing_volume.far_range_m);
    if (max_depth <= min_depth) {
        return evidence;
    }

    const double tan_half_h = std::tan(static_cast<double>(sensing_volume.horizontal_fov_rad) * 0.5);
    const double tan_half_v = std::tan(static_cast<double>(sensing_volume.vertical_fov_rad) * 0.5);
    const double width_minus_one = std::max(1, frame.width - 1);
    const double height_minus_one = std::max(1, frame.height - 1);
    const float voxel = clamp_positive(config.voxel_size_m, 0.75F);
    const float confidence = std::clamp(config.confidence, 0.0F, 1.0F);
    const float normal_confidence = std::clamp(config.normal_confidence, 0.0F, 1.0F);
    const float patch_depth_m = std::max(0.05F, voxel * 0.25F);
    const float patch_side_m = std::max(0.10F, voxel);

    std::vector<ProjectedDepthSample> projected(expected_size);
    for (int y = 0; y < frame.height; ++y) {
        for (int x = 0; x < frame.width; ++x) {
            const auto index = static_cast<std::size_t>(y) * static_cast<std::size_t>(frame.width) +
                               static_cast<std::size_t>(x);
            const float depth = frame.depth_m[index];
            if (!finite_positive(depth) || depth < min_depth || depth > max_depth) {
                continue;
            }

            const double nx = (static_cast<double>(x) / width_minus_one) * 2.0 - 1.0;
            const double ny = (static_cast<double>(y) / height_minus_one) * 2.0 - 1.0;
            const double forward_m = depth;
            const double right_m = forward_m * nx * tan_half_h;
            const double up_m = -forward_m * ny * tan_half_v;

            Vec3 center = sensing_volume.origin_local;
            center = add(center, scale(sensing_volume.forward_axis_local, forward_m));
            center = add(center, scale(sensing_volume.right_axis_local, right_m));
            center = add(center, scale(sensing_volume.up_axis_local, up_m));

            auto& sample = projected[index];
            sample.valid = true;
            sample.center_local = center;
            sample.forward_m = forward_m;
            sample.right_m = right_m;
            sample.up_m = up_m;
            sample.range_m = static_cast<float>(std::sqrt(forward_m * forward_m + right_m * right_m + up_m * up_m));
            sample.bearing_rad = static_cast<float>(std::atan2(right_m, forward_m));
            sample.elevation_rad = static_cast<float>(std::atan2(up_m, norm_xy(forward_m, right_m)));
        }
    }

    for (int y = 0; y < frame.height; y += static_cast<int>(stride)) {
        for (int x = 0; x < frame.width; x += static_cast<int>(stride)) {
            if (config.max_evidence > 0U && evidence.size() >= config.max_evidence) {
                return evidence;
            }
            const auto index = static_cast<std::size_t>(y) * static_cast<std::size_t>(frame.width) +
                               static_cast<std::size_t>(x);
            const auto& projected_sample = projected[index];
            if (!projected_sample.valid) {
                continue;
            }

            ObstacleEvidence item;
            item.timestamp = frame.timestamp.timestamp_ns != 0 ? frame.timestamp : sensing_volume.timestamp;
            item.source_frame_id = frame.has_source_frame ? frame.source_frame_id : sensing_volume.source_frame_id;
            item.has_source_frame = frame.has_source_frame || sensing_volume.has_source_frame;
            item.sensor_name = !frame.sensor_name.empty() ? frame.sensor_name : sensing_volume.sensor_name;
            item.source_provider = "airsim_depth_obstacle_detector";
            item.source_kind = OccupancySourceKind::DepthProvider;
            item.map_frame_id = sensing_volume.map_frame_id;
            item.state = ObstacleEvidenceState::Occupied;
            item.shape = ObstacleEvidenceShape::SurfacePatch;
            item.center_local = projected_sample.center_local;
            item.size_m = Vec3{patch_side_m, patch_side_m, patch_depth_m};

            Vec3 normal_local{};
            if (config.derive_surface_normals_from_depth &&
                depth_derived_surface_normal(projected, frame.width, frame.height, x, y, normal_local)) {
                item.has_surface_normal = true;
                item.surface_normal_local = normal_local;
                item.normal_confidence = normal_confidence;
            }
            item.occupancy_probability = confidence;
            item.confidence = confidence;
            item.range_m = projected_sample.range_m;
            item.bearing_rad = projected_sample.bearing_rad;
            item.elevation_rad = projected_sample.elevation_rad;
            item.inside_sensing_volume = true;
            item.inside_swept_volume = false;
            item.is_static_hint = true;

            evidence.push_back(item);
        }
    }

    return evidence;
}

}  // namespace dedalus
