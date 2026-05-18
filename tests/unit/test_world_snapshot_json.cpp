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
