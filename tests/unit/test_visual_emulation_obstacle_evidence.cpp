#include <cmath>
#include <iostream>
#include <vector>

#include "dedalus/perception/perception_pipeline.hpp"
#include "dedalus/world_model/in_memory_world_model.hpp"

namespace {

bool near(double lhs, double rhs) {
    return std::abs(lhs - rhs) < 1.0e-9;
}

constexpr double kPi = 3.14159265358979323846;

dedalus::ObstacleSensingVolume explicit_front_coverage(
    dedalus::TimePoint timestamp,
    const dedalus::MapFrameId& map_frame_id,
    const dedalus::FrameId& frame_id) {
    dedalus::ObstacleSensingVolume volume;
    volume.timestamp = timestamp;
    volume.sensor_name = "front_center";
    volume.provider_name = "configured_camera_coverage";
    volume.map_frame_id = map_frame_id;
    volume.origin_local = dedalus::Vec3{0.0, 0.0, -8.0};
    volume.forward_axis_local = dedalus::Vec3{1.0, 0.0, 0.0};
    volume.right_axis_local = dedalus::Vec3{0.0, 1.0, 0.0};
    volume.up_axis_local = dedalus::Vec3{0.0, 0.0, -1.0};
    volume.near_range_m = 0.5F;
    volume.far_range_m = 80.0F;
    volume.horizontal_fov_rad = 1.57079632679F;
    volume.vertical_fov_rad = 1.0471975512F;
    volume.min_reliable_range_m = 1.0F;
    volume.max_reliable_range_m = 60.0F;
    volume.min_surface_area_m2 = 0.25F;
    volume.min_angular_size_rad = 0.01F;
    volume.min_confidence = 0.30F;
    volume.source_frame_id = frame_id;
    volume.has_source_frame = true;
    return volume;
}

dedalus::ObstacleSensingVolume explicit_pitched_down_coverage(
    dedalus::TimePoint timestamp,
    const dedalus::MapFrameId& map_frame_id,
    const dedalus::FrameId& frame_id,
    double pitch_rad) {
    dedalus::ObstacleSensingVolume volume = explicit_front_coverage(timestamp, map_frame_id, frame_id);
    const auto c = std::cos(pitch_rad);
    const auto s = std::sin(pitch_rad);

    volume.provider_name = "configured_camera_coverage";
    volume.forward_axis_local = dedalus::Vec3{c, 0.0, -s};
    volume.right_axis_local = dedalus::Vec3{0.0, 1.0, 0.0};
    volume.up_axis_local = dedalus::Vec3{-s, 0.0, -c};
    return volume;
}

bool has_evidence_center_near(
    const dedalus::WorldSnapshot& snapshot,
    const dedalus::Vec3& expected_center) {
    for (const auto& evidence : snapshot.obstacle_evidence) {
        if (near(evidence.center_local.x, expected_center.x) &&
            near(evidence.center_local.y, expected_center.y) &&
            near(evidence.center_local.z, expected_center.z)) {
            return true;
        }
    }
    return false;
}

}  // namespace

int main() {
    const dedalus::TimePoint timestamp{3000000000};
    const dedalus::MapFrameId map_frame_id{"map_local_0001"};
    const dedalus::FrameId frame_id{"frame_0001"};

    dedalus::EgoState ego;
    ego.timestamp = timestamp;
    ego.map_frame_id = map_frame_id;
    ego.local_T_body.position = dedalus::Vec3{0.0, 0.0, -8.0};
    ego.height_m = 8.0;
    ego.height_valid = true;
    ego.flight_status = dedalus::EgoFlightStatus::Airborne;
    ego.confidence = 0.9F;

    dedalus::PerceptionPipelineOutput output;

    dedalus::Observation3D forward_observation;
    forward_observation.track_id = dedalus::TrackId{"obstacle_forward"};
    forward_observation.source_frame_id = frame_id;
    forward_observation.has_source_frame = true;
    forward_observation.timestamp = timestamp;
    forward_observation.position_local = dedalus::Vec3{5.0, 0.0, -8.0};
    forward_observation.map_frame_id = map_frame_id;
    forward_observation.class_label = dedalus::ClassLabel::Unknown;
    forward_observation.confidence = 0.9F;
    output.observations.push_back(forward_observation);

    auto outside_fov_observation = forward_observation;
    outside_fov_observation.track_id = dedalus::TrackId{"obstacle_outside_sensing_volume"};
    outside_fov_observation.position_local = dedalus::Vec3{5.0, 7.0, -8.0};
    outside_fov_observation.confidence = 0.8F;
    output.observations.push_back(outside_fov_observation);

    auto beyond_range_observation = forward_observation;
    beyond_range_observation.track_id = dedalus::TrackId{"obstacle_beyond_sensing_range"};
    beyond_range_observation.position_local = dedalus::Vec3{100.0, 0.0, -8.0};
    beyond_range_observation.confidence = 0.7F;
    output.observations.push_back(beyond_range_observation);

    auto forward_down_observation = forward_observation;
    forward_down_observation.track_id = dedalus::TrackId{"obstacle_forward_down"};
    forward_down_observation.position_local = dedalus::Vec3{5.0, 0.0, -13.0};
    forward_down_observation.confidence = 0.88F;
    output.observations.push_back(forward_down_observation);

    dedalus::InMemoryWorldModelConfig config;
    config.map_frame_id = map_frame_id;
    config.occupancy_source_kind = dedalus::OccupancySourceKind::AirSimGroundTruth;

    dedalus::InMemoryWorldModel no_coverage_world_model{config};
    no_coverage_world_model.update_ego(ego);
    no_coverage_world_model.ingest(output);
    const auto no_coverage_snapshot = no_coverage_world_model.snapshot();
    if (!no_coverage_snapshot.has_ego_occupancy ||
        no_coverage_snapshot.ego_occupancy.source_kind != dedalus::OccupancySourceKind::AirSimGroundTruth ||
        no_coverage_snapshot.ego_occupancy.occupied_count != 4U) {
        std::cerr << "ground-truth world model should retain global occupancy without visual coverage\n";
        return 1;
    }
    if (!no_coverage_snapshot.obstacle_sensing_volumes.empty() || !no_coverage_snapshot.obstacle_evidence.empty()) {
        std::cerr << "AirSim visual-emulation should not fake sensing coverage when none is supplied\n";
        return 1;
    }

    dedalus::InMemoryWorldModel world_model{config};
    world_model.update_ego(ego);
    world_model.update_obstacle_sensing_volumes({explicit_front_coverage(timestamp, map_frame_id, frame_id)});
    world_model.ingest(output);

    const auto snapshot = world_model.snapshot();
    if (!snapshot.has_ego_occupancy || snapshot.ego_occupancy.source_kind != dedalus::OccupancySourceKind::AirSimGroundTruth) {
        std::cerr << "ground-truth world model did not expose global occupancy oracle\n";
        return 1;
    }
    if (snapshot.ego_occupancy.occupied_count != 4U || snapshot.ego_occupancy.debug_cells.size() != 4U) {
        std::cerr << "global occupancy oracle should retain all named-object observations despite visual clipping\n";
        return 1;
    }
    if (snapshot.obstacle_sensing_volumes.size() != 1U) {
        std::cerr << "visual-emulation path did not expose the explicit sensing volume\n";
        return 1;
    }
    const auto& sensing_volume = snapshot.obstacle_sensing_volumes.front();
    if (sensing_volume.provider_name != "configured_camera_coverage" || !sensing_volume.has_source_frame ||
        sensing_volume.source_frame_id.value != frame_id.value) {
        std::cerr << "visual-emulation sensing volume did not preserve explicit provider/source-frame metadata\n";
        return 1;
    }
    if (snapshot.obstacle_evidence.size() != 1U) {
        std::cerr << "visual-emulation evidence should include only the in-volume observation\n";
        return 1;
    }
    const auto& evidence = snapshot.obstacle_evidence.front();
    if (evidence.source_kind != dedalus::OccupancySourceKind::AirSimGroundTruthVisualEmulation ||
        evidence.source_provider != "airsim_gt_vd") {
        std::cerr << "visual-emulation evidence did not use visual-emulation provenance\n";
        return 1;
    }
    if (!evidence.inside_sensing_volume || !evidence.inside_swept_volume) {
        std::cerr << "forward visual-emulation evidence should intersect sensing and swept volumes\n";
        return 1;
    }
    if (!near(evidence.center_local.x, 5.0) || !near(evidence.center_local.y, 0.0) || evidence.confidence < 0.89F) {
        std::cerr << "visual-emulation evidence did not preserve the forward observation geometry\n";
        return 1;
    }

    dedalus::InMemoryWorldModel pitched_world_model{config};
    pitched_world_model.update_ego(ego);
    pitched_world_model.update_obstacle_sensing_volumes({
        explicit_pitched_down_coverage(timestamp, map_frame_id, frame_id, kPi / 4.0)});
    pitched_world_model.ingest(output);

    const auto pitched_snapshot = pitched_world_model.snapshot();
    if (!pitched_snapshot.has_ego_occupancy ||
        pitched_snapshot.ego_occupancy.source_kind != dedalus::OccupancySourceKind::AirSimGroundTruth ||
        pitched_snapshot.ego_occupancy.occupied_count != 4U) {
        std::cerr << "pitched visual-emulation should preserve global AirSim GT occupancy oracle\n";
        return 1;
    }
    if (pitched_snapshot.obstacle_sensing_volumes.size() != 1U) {
        std::cerr << "pitched visual-emulation should publish the explicit pitched sensing volume\n";
        return 1;
    }
    const auto& pitched_volume = pitched_snapshot.obstacle_sensing_volumes.front();
    if (!near(pitched_volume.forward_axis_local.x, std::sqrt(0.5)) ||
        !near(pitched_volume.forward_axis_local.y, 0.0) ||
        !near(pitched_volume.forward_axis_local.z, -std::sqrt(0.5))) {
        std::cerr << "pitched sensing volume did not preserve downward camera optical axis\n";
        return 1;
    }
    if (pitched_snapshot.obstacle_evidence.size() != 1U) {
        std::cerr << "pitched visual-emulation evidence should include only the forward-down observation\n";
        return 1;
    }
    if (has_evidence_center_near(pitched_snapshot, forward_observation.position_local)) {
        std::cerr << "pitched visual-emulation should exclude forward-level object outside camera coverage\n";
        return 1;
    }
    if (!has_evidence_center_near(pitched_snapshot, forward_down_observation.position_local)) {
        std::cerr << "pitched visual-emulation should include forward-down object inside camera coverage\n";
        return 1;
    }
    const auto& pitched_evidence = pitched_snapshot.obstacle_evidence.front();
    if (!pitched_evidence.inside_sensing_volume || !pitched_evidence.inside_swept_volume) {
        std::cerr << "pitched forward-down evidence should intersect sensing and swept volumes\n";
        return 1;
    }

    return 0;
}
