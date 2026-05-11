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

void write_vec3(std::ostringstream& out, const Vec3& value) {
    out << "[" << value.x << "," << value.y << "," << value.z << "]";
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
    out << "    \"position_local\": ";
    write_vec3(out, snapshot.ego.local_T_body.position);
    out << ",\n";
    out << "    \"velocity_local\": ";
    write_vec3(out, snapshot.ego.velocity_local);
    out << "\n";
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
    out << "  \"tactical_exclusion_zones\": [],\n";
    out << "  \"flight_corridors\": [],\n";
    out << "  \"static_structures\": [],\n";
    out << "  \"landmarks\": [],\n";
    out << "  \"uncertain_regions\": [],\n";

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
