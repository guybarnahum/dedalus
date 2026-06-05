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

    dedalus::AirSimDepthObstacleDetectorConfig config;
    config.pixel_stride = 1;
    config.min_depth_m = 0.5F;
    config.max_depth_m = 30.0F;
    config.voxel_size_m = 0.5F;
    config.confidence = 0.8F;
    config.max_evidence = 32U;

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

    return 0;
}
