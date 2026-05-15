#include "dedalus/world_model/in_memory_world_model.hpp"

#include <cmath>

namespace dedalus {
namespace {

EgoState mission_ready_ego(EgoState ego) {
    if (!ego.height_valid) {
        ego.height_m = -ego.local_T_body.position.z;
        ego.height_valid = true;
    }
    if (ego.confidence <= 0.0F) {
        ego.confidence = 0.85F;
    }
    if (ego.flight_status == EgoFlightStatus::Unknown && ego.height_valid) {
        ego.flight_status = ego.height_m > 0.25 ? EgoFlightStatus::Airborne : EgoFlightStatus::Landed;
    }
    return ego;
}

}  // namespace

InMemoryWorldModel::InMemoryWorldModel(MapFrameId map_frame_id) {
    snapshot_.timestamp = TimePoint{0};
    snapshot_.active_map_frame_id = map_frame_id;
    snapshot_.ego.map_frame_id = map_frame_id;
    snapshot_.ego.home_T_body = snapshot_.ego.local_T_body;
    snapshot_.ego.home_timestamp = snapshot_.timestamp;
    snapshot_.ego.height_m = 0.0;
    snapshot_.ego.height_valid = true;
    snapshot_.ego.flight_status = EgoFlightStatus::Landed;

    MapFrame frame;
    frame.map_frame_id = map_frame_id;
    frame.scale_confidence = 0.5F;
    frame.orientation_confidence = 0.5F;
    frame.created_at = snapshot_.timestamp;
    frame.last_used = snapshot_.timestamp;
    snapshot_.map_frames.push_back(frame);
}

void InMemoryWorldModel::update_ego(const EgoState& ego) {
    snapshot_.timestamp = ego.timestamp;

    auto updated_ego = mission_ready_ego(ego);
    if (!updated_ego.home_T_body.has_value()) {
        if (snapshot_.ego.home_T_body.has_value()) {
            updated_ego.home_T_body = snapshot_.ego.home_T_body;
            updated_ego.home_timestamp = snapshot_.ego.home_timestamp;
        } else {
            updated_ego.home_T_body = updated_ego.local_T_body;
            updated_ego.home_timestamp = updated_ego.timestamp;
        }
    }

    snapshot_.ego = updated_ego;
    snapshot_.active_map_frame_id = updated_ego.map_frame_id;

    if (!snapshot_.map_frames.empty()) {
        snapshot_.map_frames.front().map_frame_id = updated_ego.map_frame_id;
        snapshot_.map_frames.front().last_used = updated_ego.timestamp;
    }
}

void InMemoryWorldModel::update_appearance(const AppearanceCondition& appearance_condition) {
    snapshot_.appearance_condition = appearance_condition;
}

void InMemoryWorldModel::ingest(const PerceptionPipelineOutput& perception_output) {
    snapshot_.agents.clear();

    for (const auto& observation : perception_output.observations) {
        AgentState agent;
        agent.agent_id = AgentId{"agent_0001"};
        agent.identity_id = IdentityId{"identity_unknown_0001"};
        agent.source_track_id = observation.track_id;
        agent.last_seen = observation.timestamp;
        agent.position_local = observation.position_local;
        agent.velocity_local = Vec3{2.0, 0.4, 0.0};
        agent.map_frame_id = observation.map_frame_id;
        agent.class_label = observation.class_label;
        agent.faction = observation.faction;
        agent.lifecycle = AgentLifecycle::Active;
        agent.confidence = observation.confidence;
        snapshot_.agents.push_back(agent);
    }

    snapshot_.tactical_exclusion_zones = cone_exclusion_mapper_.map(
        perception_output.observations,
        snapshot_.timestamp,
        snapshot_.active_map_frame_id);

    const auto flight_map_update = rough_flight_map_builder_.build(
        perception_output.observations,
        snapshot_.ego,
        snapshot_.timestamp,
        snapshot_.active_map_frame_id);
    snapshot_.static_structures = flight_map_update.static_structures;
    snapshot_.flight_corridors = flight_map_update.flight_corridors;
    snapshot_.landmarks = flight_map_update.landmarks;

    snapshot_.containers.clear();
    ContainerState car;
    car.container_id = AgentId{"agent_car_0001"};
    car.type = ContainerType::Car;
    car.local_T_container.position = Vec3{20.0, -4.0, -10.5};
    car.velocity_local = Vec3{0.0, 0.0, 0.0};
    car.map_frame_id = snapshot_.active_map_frame_id;
    car.capacity_estimate = 4.0F;
    car.confidence = 0.65F;
    snapshot_.containers.push_back(car);

    snapshot_.uncertain_regions.clear();
    UncertainRegion region;
    region.region_id = "unknown_forward_sector";
    region.bounds.min = Vec3{8.0, -6.0, -14.0};
    region.bounds.max = Vec3{30.0, 6.0, -4.0};
    region.map_frame_id = snapshot_.active_map_frame_id;
    region.uncertainty = 0.6F;
    region.reason = "synthetic_placeholder_depth_uncertainty";
    snapshot_.uncertain_regions.push_back(region);
}

WorldSnapshot InMemoryWorldModel::snapshot() const {
    return snapshot_;
}

EffectiveWorldView InMemoryWorldModel::effective_view() const {
    EffectiveWorldView view;
    view.actual = snapshot_;
    view.memory.confidence = 0.0F;
    view.uncertain_regions = snapshot_.uncertain_regions;
    return view;
}

}  // namespace dedalus
