#include "dedalus/world_model/world_snapshot.hpp"

#include <sstream>
#include <string_view>

namespace dedalus {
namespace {

std::string escape_json(std::string_view value) {
    std::ostringstream out;
    for (const char ch : value) {
        switch (ch) {
            case '"':
                out << "\\\"";
                break;
            case '\\':
                out << "\\\\";
                break;
            case '\n':
                out << "\\n";
                break;
            case '\r':
                out << "\\r";
                break;
            case '\t':
                out << "\\t";
                break;
            default:
                out << ch;
                break;
        }
    }
    return out.str();
}

const char* to_string(ClassLabel value) {
    switch (value) {
        case ClassLabel::Person:
            return "person";
        case ClassLabel::Drone:
            return "drone";
        case ClassLabel::Car:
            return "car";
        case ClassLabel::Boat:
            return "boat";
        case ClassLabel::House:
            return "house";
        case ClassLabel::Building:
            return "building";
        case ClassLabel::Tree:
            return "tree";
        case ClassLabel::Road:
            return "road";
        case ClassLabel::River:
            return "river";
        case ClassLabel::Terrain:
            return "terrain";
        case ClassLabel::Unknown:
        default:
            return "unknown";
    }
}

const char* to_string(FactionLabel value) {
    switch (value) {
        case FactionLabel::Friendly:
            return "friendly";
        case FactionLabel::Neutral:
            return "neutral";
        case FactionLabel::Hostile:
            return "hostile";
        case FactionLabel::Unknown:
        default:
            return "unknown";
    }
}

const char* to_string(AgentLifecycle value) {
    switch (value) {
        case AgentLifecycle::New:
            return "new";
        case AgentLifecycle::Active:
            return "active";
        case AgentLifecycle::Occluded:
            return "occluded";
        case AgentLifecycle::Contained:
            return "contained";
        case AgentLifecycle::Stale:
            return "stale";
        case AgentLifecycle::Retired:
            return "retired";
        default:
            return "unknown";
    }
}

const char* to_string(ContainerType value) {
    switch (value) {
        case ContainerType::Car:
            return "car";
        case ContainerType::Boat:
            return "boat";
        case ContainerType::House:
            return "house";
        case ContainerType::Building:
            return "building";
        case ContainerType::Garage:
            return "garage";
        case ContainerType::Room:
            return "room";
        case ContainerType::Unknown:
        default:
            return "unknown";
    }
}

const char* to_string(ZoneType value) {
    switch (value) {
        case ZoneType::Cone:
            return "cone";
        case ZoneType::Cylinder:
            return "cylinder";
        case ZoneType::Box:
            return "box";
        case ZoneType::VoxelCluster:
            return "voxel_cluster";
        default:
            return "unknown";
    }
}

const char* to_string(LightingMode value) {
    switch (value) {
        case LightingMode::Day:
            return "day";
        case LightingMode::Night:
            return "night";
        case LightingMode::Dawn:
            return "dawn";
        case LightingMode::Dusk:
            return "dusk";
        case LightingMode::Artificial:
            return "artificial";
        case LightingMode::Unknown:
        default:
            return "unknown";
    }
}

const char* to_string(WeatherMode value) {
    switch (value) {
        case WeatherMode::Clear:
            return "clear";
        case WeatherMode::Rain:
            return "rain";
        case WeatherMode::Fog:
            return "fog";
        case WeatherMode::Dust:
            return "dust";
        case WeatherMode::Snow:
            return "snow";
        case WeatherMode::Unknown:
        default:
            return "unknown";
    }
}

const char* to_string(EgoFlightStatus value) {
    switch (value) {
        case EgoFlightStatus::Landed:
            return "landed";
        case EgoFlightStatus::Airborne:
            return "airborne";
        case EgoFlightStatus::Unknown:
        default:
            return "unknown";
    }
}

const char* to_string(FlightControlArmState value) {
    switch (value) {
        case FlightControlArmState::Disarmed:
            return "disarmed";
        case FlightControlArmState::ArmRequested:
            return "arm_requested";
        case FlightControlArmState::ArmedConfirmed:
            return "armed_confirmed";
        case FlightControlArmState::DisarmRequested:
            return "disarm_requested";
        case FlightControlArmState::DisarmedConfirmed:
            return "disarmed_confirmed";
        case FlightControlArmState::ArmFailed:
            return "arm_failed";
        case FlightControlArmState::DisarmFailed:
            return "disarm_failed";
        case FlightControlArmState::Unknown:
        default:
            return "unknown";
    }
}

const char* bool_string(bool value) {
    return value ? "true" : "false";
}

void write_vec2(std::ostringstream& out, const Vec2& value) {
    out << "[" << value.x << "," << value.y << "]";
}

void write_vec3(std::ostringstream& out, const Vec3& value) {
    out << "[" << value.x << "," << value.y << "," << value.z << "]";
}

void write_rect2(std::ostringstream& out, const Rect2& value) {
    out << "{\"x\":" << value.x << ",\"y\":" << value.y << ",\"width\":" << value.width
        << ",\"height\":" << value.height << "}";
}

void write_pose3(std::ostringstream& out, const Pose3& value) {
    out << "{\"position_local\":";
    write_vec3(out, value.position);
    out << ",\"rotation_rpy\":";
    write_vec3(out, value.rotation_rpy);
    out << "}";
}

void write_bounds3(std::ostringstream& out, const Bounds3& value) {
    out << "{\"min\":";
    write_vec3(out, value.min);
    out << ",\"max\":";
    write_vec3(out, value.max);
    out << "}";
}

void write_agent_view_evidence(std::ostringstream& out, const AgentViewEvidence& evidence) {
    out << "{\n";
    out << "        \"timestamp_ns\": " << evidence.timestamp.timestamp_ns;
    if (evidence.has_source_frame) {
        out << ",\n        \"source_frame_id\": \"" << escape_json(evidence.source_frame_id.value) << "\"";
    }
    if (evidence.has_source_detection) {
        out << ",\n        \"source_detection_id\": \"" << escape_json(evidence.source_detection_id.value) << "\"";
    }
    if (evidence.has_source_bbox) {
        out << ",\n        \"source_bbox_px\": ";
        write_rect2(out, evidence.source_bbox_px);
    }
    if (evidence.has_source_center) {
        out << ",\n        \"source_center_px\": ";
        write_vec2(out, evidence.source_center_px);
    }
    if (!evidence.camera_name.empty()) {
        out << ",\n        \"camera_name\": \"" << escape_json(evidence.camera_name) << "\"";
    }
    out << "\n      }";
}

}  // namespace

std::string to_json(const WorldSnapshot& snapshot) {
    std::ostringstream out;

    out << "{\n";
    out << "  \"timestamp_ns\": " << snapshot.timestamp.timestamp_ns << ",\n";
    out << "  \"active_map_frame_id\": \""
        << escape_json(snapshot.active_map_frame_id.value) << "\",\n";

    out << "  \"appearance_condition\": {\n";
    out << "    \"lighting_mode\": \""
        << to_string(snapshot.appearance_condition.lighting_mode) << "\",\n";
    out << "    \"weather_mode\": \""
        << to_string(snapshot.appearance_condition.weather_mode) << "\",\n";
    out << "    \"confidence\": "
        << snapshot.appearance_condition.confidence << "\n";
    out << "  },\n";

    out << "  \"ego\": {\n";
    out << "    \"timestamp_ns\": " << snapshot.ego.timestamp.timestamp_ns << ",\n";
    out << "    \"map_frame_id\": \"" << escape_json(snapshot.ego.map_frame_id.value) << "\",\n";
    out << "    \"position_local\": ";
    write_vec3(out, snapshot.ego.local_T_body.position);
    out << ",\n";
    out << "    \"rotation_rpy\": ";
    write_vec3(out, snapshot.ego.local_T_body.rotation_rpy);
    out << ",\n";
    out << "    \"velocity_local\": ";
    write_vec3(out, snapshot.ego.velocity_local);
    out << ",\n";
    out << "    \"angular_velocity_body\": ";
    write_vec3(out, snapshot.ego.angular_velocity_body);
    out << ",\n";
    out << "    \"height_m\": " << snapshot.ego.height_m << ",\n";
    out << "    \"height_valid\": " << bool_string(snapshot.ego.height_valid) << ",\n";
    out << "    \"armed\": " << bool_string(snapshot.ego.armed) << ",\n";
    out << "    \"armed_valid\": " << bool_string(snapshot.ego.armed_valid) << ",\n";
    out << "    \"flight_status\": \"" << to_string(snapshot.ego.flight_status) << "\",\n";
    out << "    \"confidence\": " << snapshot.ego.confidence;
    if (snapshot.ego.home_T_body.has_value()) {
        out << ",\n    \"home_pose\": ";
        write_pose3(out, *snapshot.ego.home_T_body);
    }
    if (snapshot.ego.home_timestamp.has_value()) {
        out << ",\n    \"home_timestamp_ns\": " << snapshot.ego.home_timestamp->timestamp_ns;
    }
    out << "\n";
    out << "  },\n";

    out << "  \"flight_control\": {\n";
    out << "    \"arm_state\": \"" << to_string(snapshot.flight_control.arm_state) << "\",\n";
    out << "    \"updated_at_ns\": " << snapshot.flight_control.updated_at.timestamp_ns << ",\n";
    out << "    \"last_arm_request_at_ns\": " << snapshot.flight_control.last_arm_request_at.timestamp_ns << ",\n";
    out << "    \"last_disarm_request_at_ns\": " << snapshot.flight_control.last_disarm_request_at.timestamp_ns << ",\n";
    out << "    \"status\": \"" << escape_json(snapshot.flight_control.status) << "\"\n";
    out << "  },\n";

    out << "  \"agents\": [";
    for (std::size_t i = 0; i < snapshot.agents.size(); ++i) {
        const auto& agent = snapshot.agents[i];
        if (i != 0) {
            out << ",";
        }
        out << "\n";
        out << "    {\n";
        out << "      \"agent_id\": \"" << escape_json(agent.agent_id.value) << "\",\n";
        out << "      \"identity_id\": \"" << escape_json(agent.identity_id.value) << "\",\n";
        out << "      \"source_track_id\": \"" << escape_json(agent.source_track_id.value) << "\",\n";
        if (agent.has_source_detection) {
            out << "      \"source_detection_id\": \"" << escape_json(agent.source_detection_id.value) << "\",\n";
        }
        if (agent.has_source_frame) {
            out << "      \"source_frame_id\": \"" << escape_json(agent.source_frame_id.value) << "\",\n";
        }
        if (agent.has_source_bbox) {
            out << "      \"source_bbox_px\": ";
            write_rect2(out, agent.source_bbox_px);
            out << ",\n";
        }
        if (agent.has_latest_view_evidence) {
            out << "      \"latest_view_evidence\": ";
            write_agent_view_evidence(out, agent.latest_view_evidence);
            out << ",\n";
        }
        out << "      \"class\": \"" << to_string(agent.class_label) << "\",\n";
        out << "      \"faction\": \"" << to_string(agent.faction) << "\",\n";
        out << "      \"lifecycle\": \"" << to_string(agent.lifecycle) << "\",\n";
        out << "      \"position_local\": ";
        write_vec3(out, agent.position_local);
        out << ",\n";
        out << "      \"velocity_local\": ";
        write_vec3(out, agent.velocity_local);
        out << ",\n";
        out << "      \"confidence\": " << agent.confidence << "\n";
        out << "    }";
    }
    if (!snapshot.agents.empty()) {
        out << "\n  ";
    }
    out << "],\n";

    out << "  \"containers\": [";
    for (std::size_t i = 0; i < snapshot.containers.size(); ++i) {
        const auto& container = snapshot.containers[i];
        if (i != 0) {
            out << ",";
        }
        out << "\n";
        out << "    {\n";
        out << "      \"container_id\": \"" << escape_json(container.container_id.value) << "\",\n";
        out << "      \"type\": \"" << to_string(container.type) << "\",\n";
        out << "      \"confidence\": " << container.confidence << "\n";
        out << "    }";
    }
    if (!snapshot.containers.empty()) {
        out << "\n  ";
    }
    out << "],\n";

    out << "  \"containment_events\": [],\n";

    out << "  \"tactical_exclusion_zones\": [";
    for (std::size_t i = 0; i < snapshot.tactical_exclusion_zones.size(); ++i) {
        const auto& zone = snapshot.tactical_exclusion_zones[i];
        if (i != 0) {
            out << ",";
        }
        out << "\n";
        out << "    {\n";
        out << "      \"zone_id\": \"" << escape_json(zone.zone_id.value) << "\",\n";
        out << "      \"type\": \"" << to_string(zone.type) << "\",\n";
        out << "      \"position_local\": ";
        write_vec3(out, zone.local_T_zone.position);
        out << ",\n";
        out << "      \"dimensions\": ";
        write_vec3(out, zone.dimensions);
        out << ",\n";
        out << "      \"inflation_radius_m\": " << zone.inflation_radius_m << ",\n";
        out << "      \"reason\": \"" << escape_json(zone.reason) << "\",\n";
        out << "      \"confidence\": " << zone.confidence << "\n";
        out << "    }";
    }
    if (!snapshot.tactical_exclusion_zones.empty()) {
        out << "\n  ";
    }
    out << "],\n";

    out << "  \"flight_corridors\": [";
    for (std::size_t i = 0; i < snapshot.flight_corridors.size(); ++i) {
        const auto& corridor = snapshot.flight_corridors[i];
        if (i != 0) {
            out << ",";
        }
        out << "\n";
        out << "    {\n";
        out << "      \"corridor_id\": \"" << escape_json(corridor.corridor_id.value) << "\",\n";
        out << "      \"radius_m\": " << corridor.radius_m << ",\n";
        out << "      \"min_altitude_m\": " << corridor.min_altitude_m << ",\n";
        out << "      \"max_altitude_m\": " << corridor.max_altitude_m << ",\n";
        out << "      \"confidence\": " << corridor.confidence << "\n";
        out << "    }";
    }
    if (!snapshot.flight_corridors.empty()) {
        out << "\n  ";
    }
    out << "],\n";

    out << "  \"static_structures\": [";
    for (std::size_t i = 0; i < snapshot.static_structures.size(); ++i) {
        const auto& structure = snapshot.static_structures[i];
        if (i != 0) {
            out << ",";
        }
        out << "\n";
        out << "    {\n";
        out << "      \"structure_id\": \"" << escape_json(structure.structure_id.value) << "\",\n";
        out << "      \"type\": \"" << escape_json(structure.type) << "\",\n";
        out << "      \"bounds\": ";
        write_bounds3(out, structure.bounds);
        out << ",\n";
        out << "      \"confidence\": " << structure.confidence << "\n";
        out << "    }";
    }
    if (!snapshot.static_structures.empty()) {
        out << "\n  ";
    }
    out << "],\n";

    out << "  \"landmarks\": [";
    for (std::size_t i = 0; i < snapshot.landmarks.size(); ++i) {
        const auto& landmark = snapshot.landmarks[i];
        if (i != 0) {
            out << ",";
        }
        out << "\n";
        out << "    {\n";
        out << "      \"landmark_id\": \"" << escape_json(landmark.landmark_id.value) << "\",\n";
        out << "      \"type\": \"" << escape_json(landmark.type) << "\",\n";
        out << "      \"position_local\": ";
        write_vec3(out, landmark.position_local);
        out << ",\n";
        out << "      \"confidence\": " << landmark.confidence << "\n";
        out << "    }";
    }
    if (!snapshot.landmarks.empty()) {
        out << "\n  ";
    }
    out << "],\n";

    out << "  \"uncertain_regions\": [";
    for (std::size_t i = 0; i < snapshot.uncertain_regions.size(); ++i) {
        const auto& region = snapshot.uncertain_regions[i];
        if (i != 0) {
            out << ",";
        }
        out << "\n";
        out << "    {\n";
        out << "      \"region_id\": \"" << escape_json(region.region_id) << "\",\n";
        out << "      \"bounds\": ";
        write_bounds3(out, region.bounds);
        out << ",\n";
        out << "      \"uncertainty\": " << region.uncertainty << ",\n";
        out << "      \"reason\": \"" << escape_json(region.reason) << "\"\n";
        out << "    }";
    }
    if (!snapshot.uncertain_regions.empty()) {
        out << "\n  ";
    }
    out << "],\n";

    out << "  \"map_frames\": [";
    for (std::size_t i = 0; i < snapshot.map_frames.size(); ++i) {
        const auto& frame = snapshot.map_frames[i];
        if (i != 0) {
            out << ",";
        }
        out << "\n";
        out << "    {\n";
        out << "      \"map_frame_id\": \"" << escape_json(frame.map_frame_id.value) << "\",\n";
        out << "      \"scale_confidence\": " << frame.scale_confidence << ",\n";
        out << "      \"orientation_confidence\": " << frame.orientation_confidence << "\n";
        out << "    }";
    }
    if (!snapshot.map_frames.empty()) {
        out << "\n  ";
    }
    out << "]\n";
    out << "}\n";

    return out.str();
}

}  // namespace dedalus
