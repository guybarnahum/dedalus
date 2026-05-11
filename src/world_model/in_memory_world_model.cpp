#include "dedalus/world_model/in_memory_world_model.hpp"

namespace dedalus {

InMemoryWorldModel::InMemoryWorldModel(MapFrameId map_frame_id) {
    snapshot_.timestamp = TimePoint{0};
    snapshot_.active_map_frame_id = map_frame_id;

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
    snapshot_.ego = ego;
    snapshot_.active_map_frame_id = ego.map_frame_id;

    if (!snapshot_.map_frames.empty()) {
        snapshot_.map_frames.front().map_frame_id = ego.map_frame_id;
        snapshot_.map_frames.front().last_used = ego.timestamp;
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
}

WorldSnapshot InMemoryWorldModel::snapshot() const {
    return snapshot_;
}

}  // namespace dedalus
