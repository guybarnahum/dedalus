#include <cmath>
#include <iostream>

#include "dedalus/perception/perception_pipeline.hpp"
#include "dedalus/world_model/in_memory_world_model.hpp"

namespace {

bool near(double lhs, double rhs) {
    return std::abs(lhs - rhs) < 1.0e-9;
}

}  // namespace

int main() {
    const dedalus::TimePoint timestamp{3000000000};
    const dedalus::MapFrameId map_frame_id{"map_local_0001"};

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
    forward_observation.source_frame_id = dedalus::FrameId{"frame_0001"};
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

    dedalus::InMemoryWorldModelConfig config;
    config.map_frame_id = map_frame_id;
    config.occupancy_source_kind = dedalus::OccupancySourceKind::AirSimGroundTruth;
    dedalus::InMemoryWorldModel world_model{config};
    world_model.update_ego(ego);
    world_model.ingest(output);

    const auto snapshot = world_model.snapshot();
    if (!snapshot.has_ego_occupancy || snapshot.ego_occupancy.source_kind != dedalus::OccupancySourceKind::AirSimGroundTruth) {
        std::cerr << "ground-truth world model did not expose global occupancy oracle\n";
        return 1;
    }
    if (snapshot.ego_occupancy.occupied_count != 3U || snapshot.ego_occupancy.debug_cells.size() != 3U) {
        std::cerr << "global occupancy oracle should retain all named-object observations\n";
        return 1;
    }
    if (snapshot.obstacle_sensing_volumes.size() != 1U) {
        std::cerr << "visual-emulation path did not expose one sensing volume\n";
        return 1;
    }
    const auto& sensing_volume = snapshot.obstacle_sensing_volumes.front();
    if (sensing_volume.provider_name != "airsim_gt_visual_emulation" || !sensing_volume.has_source_frame ||
        sensing_volume.source_frame_id.value != "frame_0001") {
        std::cerr << "visual-emulation sensing volume did not preserve provider/source-frame metadata\n";
        return 1;
    }
    if (snapshot.obstacle_evidence.size() != 1U) {
        std::cerr << "visual-emulation evidence should include only the in-volume observation\n";
        return 1;
    }
    const auto& evidence = snapshot.obstacle_evidence.front();
    if (evidence.source_kind != dedalus::OccupancySourceKind::AirSimGroundTruthVisualEmulation ||
        evidence.source_provider != "airsim_gt_visual_emulation") {
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

    return 0;
}
