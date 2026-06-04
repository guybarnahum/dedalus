#include <iostream>
#include <string>

#include "dedalus/world_model/world_snapshot.hpp"

namespace {

bool contains(const std::string& haystack, const std::string& needle) {
    return haystack.find(needle) != std::string::npos;
}

}  // namespace

int main() {
    dedalus::WorldSnapshot snapshot;
    snapshot.timestamp = dedalus::TimePoint{123456789};
    snapshot.active_map_frame_id = dedalus::MapFrameId{"map_local_0001"};
    snapshot.ego.timestamp = snapshot.timestamp;
    snapshot.ego.map_frame_id = snapshot.active_map_frame_id;
    snapshot.ego.local_T_body.position = dedalus::Vec3{1.0, 2.0, -8.0};
    snapshot.ego.local_T_body.rotation_rpy = dedalus::Vec3{0.1, 0.2, 0.3};
    snapshot.ego.velocity_local = dedalus::Vec3{0.4, 0.5, 0.6};
    snapshot.ego.height_m = 8.0;
    snapshot.ego.height_valid = true;
    snapshot.ego.flight_status = dedalus::EgoFlightStatus::Airborne;
    snapshot.ego.confidence = 0.85F;
    snapshot.ego.home_T_body = dedalus::Pose3{};
    snapshot.ego.home_timestamp = dedalus::TimePoint{123456789};

    snapshot.has_ego_occupancy = true;
    snapshot.ego_occupancy.timestamp = snapshot.timestamp;
    snapshot.ego_occupancy.map_frame_id = snapshot.active_map_frame_id;
    snapshot.ego_occupancy.source_kind = dedalus::OccupancySourceKind::SyntheticFixture;
    snapshot.ego_occupancy.source_provider = "synthetic_track4_fixture";
    snapshot.ego_occupancy.resolution_m = 1.0F;
    snapshot.ego_occupancy.size_m = dedalus::Vec3{12.0, 8.0, 4.0};
    snapshot.ego_occupancy.occupied_count = 1U;
    snapshot.ego_occupancy.free_count = 1U;
    snapshot.ego_occupancy.unknown_count = 1U;
    snapshot.ego_occupancy.nearest_obstacle_distance_m = 5.0F;
    snapshot.ego_occupancy.forward_corridor_clearance_m = 4.0F;
    snapshot.ego_occupancy.has_valid_occupancy = true;
    dedalus::OccupancyCellSummary occupied_cell;
    occupied_cell.center_local = dedalus::Vec3{6.0, 2.0, -8.0};
    occupied_cell.size_m = dedalus::Vec3{1.0, 1.0, 1.0};
    occupied_cell.state = dedalus::OccupancyCellState::Occupied;
    occupied_cell.confidence = 0.85F;
    occupied_cell.source_provider = "synthetic_track4_fixture";
    occupied_cell.source_object_name = "synthetic_forward_obstacle";
    snapshot.ego_occupancy.debug_cells.push_back(occupied_cell);

    snapshot.has_latest_swept_volume = true;
    snapshot.latest_swept_volume.timestamp = snapshot.timestamp;
    snapshot.latest_swept_volume.map_frame_id = snapshot.active_map_frame_id;
    snapshot.latest_swept_volume.status = dedalus::SweptVolumeStatus::OccupiedBlocked;
    snapshot.latest_swept_volume.source_provider = "synthetic_track4_swept_volume";
    snapshot.latest_swept_volume.reason = "synthetic_occupied_cell_intersects_forward_swept_volume";
    snapshot.latest_swept_volume.start_local = dedalus::Vec3{1.0, 2.0, -8.0};
    snapshot.latest_swept_volume.end_local = dedalus::Vec3{9.0, 2.0, -8.0};
    snapshot.latest_swept_volume.radius_m = 1.0F;
    snapshot.latest_swept_volume.horizon_s = 4.0F;
    snapshot.latest_swept_volume.nominal_speed_mps = 2.0F;
    snapshot.latest_swept_volume.min_clearance_m = -1.5F;
    snapshot.latest_swept_volume.time_to_collision_s = 2.5F;
    snapshot.latest_swept_volume.has_valid_query = true;
    snapshot.latest_swept_volume.blocking_cell_centers.push_back(dedalus::Vec3{6.0, 2.0, -8.0});

    dedalus::ObstacleSensingVolume sensing_volume;
    sensing_volume.timestamp = snapshot.timestamp;
    sensing_volume.source_frame_id = dedalus::FrameId{"frame_0007"};
    sensing_volume.has_source_frame = true;
    sensing_volume.sensor_name = "front_center";
    sensing_volume.provider_name = "airsim_gt_visual_emulation";
    sensing_volume.map_frame_id = snapshot.active_map_frame_id;
    sensing_volume.origin_local = dedalus::Vec3{1.0, 2.0, -8.0};
    sensing_volume.forward_axis_local = dedalus::Vec3{1.0, 0.0, 0.0};
    sensing_volume.right_axis_local = dedalus::Vec3{0.0, 1.0, 0.0};
    sensing_volume.up_axis_local = dedalus::Vec3{0.0, 0.0, -1.0};
    sensing_volume.near_range_m = 0.5F;
    sensing_volume.far_range_m = 80.0F;
    sensing_volume.horizontal_fov_rad = 1.5708F;
    sensing_volume.vertical_fov_rad = 1.0472F;
    sensing_volume.min_reliable_range_m = 1.0F;
    sensing_volume.max_reliable_range_m = 60.0F;
    sensing_volume.min_surface_area_m2 = 0.25F;
    sensing_volume.min_angular_size_rad = 0.01F;
    sensing_volume.min_confidence = 0.30F;
    snapshot.obstacle_sensing_volumes.push_back(sensing_volume);

    dedalus::ObstacleEvidence evidence;
    evidence.timestamp = snapshot.timestamp;
    evidence.source_frame_id = dedalus::FrameId{"frame_0007"};
    evidence.has_source_frame = true;
    evidence.sensor_name = "front_center";
    evidence.source_provider = "airsim_gt_visual_emulation";
    evidence.source_kind = dedalus::OccupancySourceKind::AirSimGroundTruthVisualEmulation;
    evidence.map_frame_id = snapshot.active_map_frame_id;
    evidence.state = dedalus::ObstacleEvidenceState::ThinStructureRisk;
    evidence.shape = dedalus::ObstacleEvidenceShape::Capsule;
    evidence.center_local = dedalus::Vec3{6.0, 2.0, -8.0};
    evidence.size_m = dedalus::Vec3{1.0, 1.0, 1.0};
    evidence.endpoint_a_local = dedalus::Vec3{6.0, 1.5, -8.0};
    evidence.endpoint_b_local = dedalus::Vec3{6.0, 2.5, -8.0};
    evidence.radius_m = 0.10F;
    evidence.occupancy_probability = 0.90F;
    evidence.free_probability = 0.05F;
    evidence.confidence = 0.88F;
    evidence.range_m = 5.0F;
    evidence.bearing_rad = 0.0F;
    evidence.elevation_rad = 0.0F;
    evidence.inside_sensing_volume = true;
    evidence.inside_swept_volume = true;
    evidence.is_static_hint = true;
    evidence.is_thin_structure_hint = true;
    snapshot.obstacle_evidence.push_back(evidence);

    dedalus::AgentState agent;
    agent.agent_id = dedalus::AgentId{"agent_track_0007"};
    agent.identity_id = dedalus::IdentityId{"identity_track_0007"};
    agent.source_track_id = dedalus::TrackId{"track_0007"};
    agent.class_label = dedalus::ClassLabel::Person;
    agent.lifecycle = dedalus::AgentLifecycle::Active;
    agent.position_local = dedalus::Vec3{3.0, 4.0, -1.0};
    agent.velocity_local = dedalus::Vec3{0.1, 0.2, 0.0};
    agent.confidence = 0.91F;
    snapshot.agents.push_back(agent);

    dedalus::MapFrame map_frame;
    map_frame.map_frame_id = snapshot.active_map_frame_id;
    map_frame.scale_confidence = 0.5F;
    map_frame.orientation_confidence = 0.5F;
    snapshot.map_frames.push_back(map_frame);

    const std::string json = dedalus::to_json(snapshot);

    if (!contains(json, "\"map_frames\"")) {
        std::cerr << "missing map_frames array\n";
        return 1;
    }

    if (!contains(json, "\"map_frame_id\": \"map_local_0001\"")) {
        std::cerr << "missing serialized map_frame_id\n";
        return 1;
    }

    const std::string required_ego_tokens[] = {
        "\"ego\"",
        "\"timestamp_ns\": 123456789",
        "\"position_local\": [1,2,-8]",
        "\"rotation_rpy\": [0.1,0.2,0.3]",
        "\"velocity_local\": [0.4,0.5,0.6]",
        "\"height_m\": 8",
        "\"height_valid\": true",
        "\"flight_status\": \"airborne\"",
        "\"confidence\": 0.85",
        "\"home_pose\"",
        "\"home_timestamp_ns\": 123456789"};

    for (const auto& token : required_ego_tokens) {
        if (!contains(json, token)) {
            std::cerr << "missing serialized ego token: " << token << "\n";
            std::cerr << json << "\n";
            return 1;
        }
    }

    const std::string required_occupancy_tokens[] = {
        "\"ego_occupancy\"",
        "\"source_kind\": \"synthetic_fixture\"",
        "\"source_provider\": \"synthetic_track4_fixture\"",
        "\"resolution_m\": 1",
        "\"size_m\": [12,8,4]",
        "\"occupied_count\": 1",
        "\"free_count\": 1",
        "\"unknown_count\": 1",
        "\"nearest_obstacle_distance_m\": 5",
        "\"forward_corridor_clearance_m\": 4",
        "\"has_valid_occupancy\": true",
        "\"debug_cells\"",
        "\"center_local\": [6,2,-8]",
        "\"state\": \"occupied\"",
        "\"source_object_name\": \"synthetic_forward_obstacle\""};

    for (const auto& token : required_occupancy_tokens) {
        if (!contains(json, token)) {
            std::cerr << "missing serialized occupancy token: " << token << "\n";
            std::cerr << json << "\n";
            return 1;
        }
    }

    const std::string required_swept_tokens[] = {
        "\"latest_swept_volume\"",
        "\"status\": \"occupied_blocked\"",
        "\"source_provider\": \"synthetic_track4_swept_volume\"",
        "\"reason\": \"synthetic_occupied_cell_intersects_forward_swept_volume\"",
        "\"start_local\": [1,2,-8]",
        "\"end_local\": [9,2,-8]",
        "\"radius_m\": 1",
        "\"horizon_s\": 4",
        "\"nominal_speed_mps\": 2",
        "\"min_clearance_m\": -1.5",
        "\"time_to_collision_s\": 2.5",
        "\"has_valid_query\": true",
        "\"blocking_cell_centers\": [[6,2,-8]]"};

    for (const auto& token : required_swept_tokens) {
        if (!contains(json, token)) {
            std::cerr << "missing serialized swept-volume token: " << token << "\n";
            std::cerr << json << "\n";
            return 1;
        }
    }

    const std::string required_obstacle_sensing_tokens[] = {
        "\"obstacle_sensing_volumes\"",
        "\"sensor_name\": \"front_center\"",
        "\"provider_name\": \"airsim_gt_visual_emulation\"",
        "\"source_frame_id\": \"frame_0007\"",
        "\"origin_local\": [1,2,-8]",
        "\"forward_axis_local\": [1,0,0]",
        "\"right_axis_local\": [0,1,0]",
        "\"up_axis_local\": [0,0,-1]",
        "\"near_range_m\": 0.5",
        "\"far_range_m\": 80",
        "\"horizontal_fov_rad\": 1.5708",
        "\"vertical_fov_rad\": 1.0472",
        "\"min_reliable_range_m\": 1",
        "\"max_reliable_range_m\": 60",
        "\"min_surface_area_m2\": 0.25",
        "\"min_angular_size_rad\": 0.01",
        "\"min_confidence\": 0.3"};

    for (const auto& token : required_obstacle_sensing_tokens) {
        if (!contains(json, token)) {
            std::cerr << "missing serialized obstacle-sensing token: " << token << "\n";
            std::cerr << json << "\n";
            return 1;
        }
    }

    const std::string required_obstacle_evidence_tokens[] = {
        "\"obstacle_evidence\"",
        "\"source_provider\": \"airsim_gt_visual_emulation\"",
        "\"source_kind\": \"airsim_gt_visual_emulation\"",
        "\"state\": \"thin_structure_risk\"",
        "\"shape\": \"capsule\"",
        "\"endpoint_a_local\": [6,1.5,-8]",
        "\"endpoint_b_local\": [6,2.5,-8]",
        "\"radius_m\": 0.1",
        "\"occupancy_probability\": 0.9",
        "\"free_probability\": 0.05",
        "\"confidence\": 0.88",
        "\"range_m\": 5",
        "\"bearing_rad\": 0",
        "\"elevation_rad\": 0",
        "\"inside_sensing_volume\": true",
        "\"inside_swept_volume\": true",
        "\"is_static_hint\": true",
        "\"is_thin_structure_hint\": true"};

    for (const auto& token : required_obstacle_evidence_tokens) {
        if (!contains(json, token)) {
            std::cerr << "missing serialized obstacle-evidence token: " << token << "\n";
            std::cerr << json << "\n";
            return 1;
        }
    }

    const std::string required_agent_tokens[] = {
        "\"agents\"",
        "\"agent_id\": \"agent_track_0007\"",
        "\"identity_id\": \"identity_track_0007\"",
        "\"source_track_id\": \"track_0007\"",
        "\"class\": \"person\"",
        "\"lifecycle\": \"active\"",
        "\"position_local\": [3,4,-1]",
        "\"velocity_local\": [0.1,0.2,0]",
        "\"confidence\": 0.91"};

    for (const auto& token : required_agent_tokens) {
        if (!contains(json, token)) {
            std::cerr << "missing serialized agent token: " << token << "\n";
            std::cerr << json << "\n";
            return 1;
        }
    }

    return 0;
}
