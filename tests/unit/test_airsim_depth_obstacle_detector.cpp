#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

#include "dedalus/sensing/airsim_depth_obstacle_detector.hpp"

namespace {

bool near(double lhs, double rhs, double eps = 1.0e-6) {
    return std::abs(lhs - rhs) < eps;
}

dedalus::ObstacleSensingVolume front_volume() {
    dedalus::ObstacleSensingVolume volume;
    volume.timestamp = dedalus::TimePoint{1000000};
    volume.source_frame_id = dedalus::FrameId{"frame_0001"};
    volume.has_source_frame = true;
    volume.sensor_name = "front_center";
    volume.provider_name = "camera_pointing_intent";
    volume.map_frame_id = dedalus::MapFrameId{"map_local_0001"};
    volume.origin_local = dedalus::Vec3{1.0, 2.0, -3.0};
    volume.forward_axis_local = dedalus::Vec3{1.0, 0.0, 0.0};
    volume.right_axis_local = dedalus::Vec3{0.0, 1.0, 0.0};
    volume.up_axis_local = dedalus::Vec3{0.0, 0.0, -1.0};
    volume.near_range_m = 0.5F;
    volume.far_range_m = 80.0F;
    volume.horizontal_fov_rad = 1.57079632679F;
    volume.vertical_fov_rad = 1.57079632679F;
    volume.min_reliable_range_m = 1.0F;
    volume.max_reliable_range_m = 60.0F;
    volume.min_confidence = 0.1F;
    return volume;
}

}  // namespace

int main() {
    dedalus::AirSimDepthFrame frame;
    frame.timestamp = dedalus::TimePoint{2000000};
    frame.source_frame_id = dedalus::FrameId{"frame_depth_0001"};
    frame.has_source_frame = true;
    frame.sensor_name = "front_center";
    frame.map_frame_id = dedalus::MapFrameId{"map_local_0001"};
    frame.width = 3;
    frame.height = 3;
    frame.depth_m.assign(9, std::numeric_limits<float>::infinity());
    frame.depth_m[4] = 10.0F;

    dedalus::AirSimDepthFrame dense_frame = frame;
    dense_frame.depth_m.assign(9, 10.0F);

    dedalus::AirSimDepthObstacleDetectorConfig config;
    config.pixel_stride = 1;
    config.min_depth_m = 0.5F;
    config.max_depth_m = 30.0F;
    config.voxel_size_m = 0.5F;
    config.confidence = 0.8F;
    config.max_evidence = 32U;
    config.coalesce_surface_patches = false;

    const auto volume = front_volume();
    const auto evidence = dedalus::detect_airsim_depth_obstacles(frame, volume, config);
    if (evidence.size() != 1U) {
        std::cerr << "expected one classless depth evidence item, got " << evidence.size() << "\n";
        return 1;
    }
    const auto& item = evidence.front();
    if (item.source_provider != "airsim_depth_obstacle_detector" ||
        item.source_kind != dedalus::OccupancySourceKind::DepthProvider) {
        std::cerr << "depth detector evidence did not use depth provider provenance\n";
        return 1;
    }
    if (item.state != dedalus::ObstacleEvidenceState::Occupied ||
        item.shape != dedalus::ObstacleEvidenceShape::SurfacePatch) {
        std::cerr << "depth detector evidence should be occupied surface geometry\n";
        return 1;
    }
    if (!item.inside_sensing_volume || item.inside_swept_volume) {
        std::cerr << "depth detector should emit in-sensing-volume evidence and leave swept-volume unset\n";
        return 1;
    }
    if (!item.has_source_frame || item.source_frame_id.value != "frame_depth_0001" ||
        item.sensor_name != "front_center") {
        std::cerr << "depth detector did not preserve frame/sensor metadata\n";
        return 1;
    }
    if (!near(item.center_local.x, 11.0) || !near(item.center_local.y, 2.0) ||
        !near(item.center_local.z, -3.0)) {
        std::cerr << "center pixel depth did not project along sensing forward axis\n";
        return 1;
    }
    if (!near(item.range_m, 10.0F) || !near(item.bearing_rad, 0.0F) ||
        !near(item.elevation_rad, 0.0F)) {
        std::cerr << "center pixel range/bearing/elevation are wrong\n";
        return 1;
    }

    const auto dense_evidence = dedalus::detect_airsim_depth_obstacles(dense_frame, volume, config);
    if (dense_evidence.size() != 9U) {
        std::cerr << "dense depth grid should emit one evidence item per valid sample when coalescing is disabled\n";
        return 1;
    }
    const auto& dense_center = dense_evidence[4];
    if (!dense_center.has_surface_normal) {
        std::cerr << "center sample in dense depth grid should derive a normal from neighboring depth samples\n";
        return 1;
    }
    const double dense_len2 =
        dense_center.surface_normal_local.x * dense_center.surface_normal_local.x +
        dense_center.surface_normal_local.y * dense_center.surface_normal_local.y +
        dense_center.surface_normal_local.z * dense_center.surface_normal_local.z;
    if (!near(dense_len2, 1.0)) {
        std::cerr << "depth-derived local normal should be unit length\n";
        return 1;
    }

    frame.depth_m.assign(9, std::numeric_limits<float>::infinity());
    frame.depth_m[2] = 5.0F;  // top-right pixel.
    const auto corner = dedalus::detect_airsim_depth_obstacles(frame, volume, config);
    if (corner.size() != 1U) {
        std::cerr << "expected one corner evidence item\n";
        return 1;
    }
    const auto& corner_item = corner.front();
    if (!near(corner_item.center_local.x, 6.0) || !near(corner_item.center_local.y, 7.0) ||
        !near(corner_item.center_local.z, -8.0)) {
        std::cerr << "corner pixel did not project through FOV axes\n";
        return 1;
    }
    if (corner_item.bearing_rad <= 0.0F || corner_item.elevation_rad <= 0.0F) {
        std::cerr << "corner pixel should have positive bearing/elevation in camera coordinates\n";
        return 1;
    }

    frame.depth_m.assign(9, std::numeric_limits<float>::infinity());
    frame.depth_m[4] = 100.0F;
    const auto out_of_range = dedalus::detect_airsim_depth_obstacles(frame, volume, config);
    if (!out_of_range.empty()) {
        std::cerr << "out-of-range depth should not emit obstacle evidence\n";
        return 1;
    }

    dedalus::AirSimDepthFrame sidecar_frame;
    sidecar_frame.timestamp = dedalus::TimePoint{3000000};
    sidecar_frame.source_frame_id = dedalus::FrameId{"frame_depth_sidecar_0001"};
    sidecar_frame.has_source_frame = true;
    sidecar_frame.sensor_name = "front_center";
    sidecar_frame.map_frame_id = dedalus::MapFrameId{"map_local_0001"};
    sidecar_frame.width = 16;
    sidecar_frame.height = 9;
    sidecar_frame.depth_m.assign(
        static_cast<std::size_t>(sidecar_frame.width) * static_cast<std::size_t>(sidecar_frame.height),
        0.0F);
    sidecar_frame.depth_m[0] = 10.0F;
    sidecar_frame.depth_m[1] = 11.0F;
    sidecar_frame.depth_m[static_cast<std::size_t>(sidecar_frame.width) + 1U] = 12.0F;
    sidecar_frame.depth_m[static_cast<std::size_t>(sidecar_frame.width) * 4U + 8U] = 13.0F;

    dedalus::AirSimDepthObstacleDetectorConfig default_config;
    default_config.min_depth_m = 0.5F;
    default_config.max_depth_m = 30.0F;
    default_config.max_evidence = 32U;
    default_config.coalesce_surface_patches = false;

    const auto sidecar_evidence =
        dedalus::detect_airsim_depth_obstacles(sidecar_frame, volume, default_config);
    if (sidecar_evidence.size() != 4U) {
        std::cerr << "default detector config should consume every sidecar depth sample; got "
                  << sidecar_evidence.size() << "\n";
        return 1;
    }

    dedalus::AirSimDepthFrame coalesce_frame;
    coalesce_frame.timestamp = dedalus::TimePoint{4000000};
    coalesce_frame.source_frame_id = dedalus::FrameId{"frame_depth_coalesce_0001"};
    coalesce_frame.has_source_frame = true;
    coalesce_frame.sensor_name = "front_center";
    coalesce_frame.map_frame_id = dedalus::MapFrameId{"map_local_0001"};
    coalesce_frame.width = 3;
    coalesce_frame.height = 3;
    coalesce_frame.depth_m.assign(9, 1.0F);

    dedalus::AirSimDepthObstacleDetectorConfig coalesce_config;
    coalesce_config.pixel_stride = 1;
    coalesce_config.min_depth_m = 0.5F;
    coalesce_config.max_depth_m = 30.0F;
    coalesce_config.voxel_size_m = 20.0F;
    coalesce_config.confidence = 0.8F;
    coalesce_config.max_evidence = 32U;
    coalesce_config.coalesce_surface_patches = true;

    const auto coalesced =
        dedalus::detect_airsim_depth_obstacles(coalesce_frame, volume, coalesce_config);
    if (coalesced.size() != 1U) {
        std::cerr << "coalesced neighboring surface samples should emit one patch, got "
                  << coalesced.size() << "\n";
        return 1;
    }
    const auto& coalesced_item = coalesced.front();
    if (!coalesced_item.has_surface_normal) {
        std::cerr << "coalesced surface patch should preserve averaged depth-derived local normal\n";
        return 1;
    }
    const double coalesced_len2 =
        coalesced_item.surface_normal_local.x * coalesced_item.surface_normal_local.x +
        coalesced_item.surface_normal_local.y * coalesced_item.surface_normal_local.y +
        coalesced_item.surface_normal_local.z * coalesced_item.surface_normal_local.z;
    if (!near(coalesced_len2, 1.0)) {
        std::cerr << "coalesced depth-derived normal should remain unit length\n";
        return 1;
    }
    if (!near(coalesced_item.size_m.x, 20.0) ||
        !near(coalesced_item.size_m.y, 20.0) ||
        !near(coalesced_item.size_m.z, 5.0)) {
        std::cerr << "coalesced patch size should derive from voxel_size_m\n";
        return 1;
    }

    return 0;
}
