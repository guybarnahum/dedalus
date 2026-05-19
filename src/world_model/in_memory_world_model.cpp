#include "dedalus/world_model/in_memory_world_model.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

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

std::string sanitized_id_suffix(const std::string& value) {
    std::string suffix;
    suffix.reserve(value.size());
    for (const char ch : value) {
        const auto uch = static_cast<unsigned char>(ch);
        if (std::isalnum(uch) || ch == '_' || ch == '-') {
            suffix.push_back(ch);
        } else {
            suffix.push_back('_');
        }
    }
    return suffix;
}

std::string fallback_observation_suffix(std::size_t index) {
    std::ostringstream out;
    out << "observation_" << std::setw(4) << std::setfill('0') << (index + 1U);
    return out.str();
}

std::string track_suffix_or_fallback(const TrackId& track_id, std::size_t observation_index) {
    const auto suffix = sanitized_id_suffix(track_id.value);
    if (!suffix.empty()) {
        return suffix;
    }
    return fallback_observation_suffix(observation_index);
}

Vec2 bbox_center(const Rect2& bbox) {
    return Vec2{bbox.x + bbox.width * 0.5, bbox.y + bbox.height * 0.5};
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

    std::size_t observation_index = 0;
    for (const auto& observation : perception_output.observations) {
        const auto id_suffix = track_suffix_or_fallback(observation.track_id, observation_index);

        AgentState agent;
        agent.agent_id = AgentId{"agent_" + id_suffix};
        agent.identity_id = IdentityId{"identity_" + id_suffix};
        agent.source_track_id = observation.track_id;
        agent.source_detection_id = observation.source_detection_id;
        agent.has_source_detection = observation.has_source_detection;
        agent.source_bbox_px = observation.source_bbox_px;
        agent.has_source_bbox = observation.has_source_bbox;
        agent.source_frame_id = observation.source_frame_id;
        agent.has_source_frame = observation.has_source_frame;
        if (observation.has_source_detection || observation.has_source_bbox || observation.has_source_frame) {
            agent.has_latest_view_evidence = true;
            agent.latest_view_evidence.source_frame_id = observation.source_frame_id;
            agent.latest_view_evidence.has_source_frame = observation.has_source_frame;
            agent.latest_view_evidence.source_detection_id = observation.source_detection_id;
            agent.latest_view_evidence.has_source_detection = observation.has_source_detection;
            agent.latest_view_evidence.source_bbox_px = observation.source_bbox_px;
            agent.latest_view_evidence.has_source_bbox = observation.has_source_bbox;
            if (observation.has_source_bbox) {
                agent.latest_view_evidence.source_center_px = bbox_center(observation.source_bbox_px);
                agent.latest_view_evidence.has_source_center = true;
            }
            agent.latest_view_evidence.timestamp = observation.timestamp;
        }
        agent.last_seen = observation.timestamp;
        agent.position_local = observation.position_local;
        agent.velocity_local = Vec3{2.0, 0.4, 0.0};
        agent.map_frame_id = observation.map_frame_id;
        agent.class_label = observation.class_label;
        agent.faction = observation.faction;
        agent.lifecycle = AgentLifecycle::Active;
        agent.confidence = observation.confidence;
        snapshot_.agents.push_back(agent);
        ++observation_index;
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
