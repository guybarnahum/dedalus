#include "dedalus/world_model/in_memory_world_model.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <limits>
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

float swept_path_lateral_distance_m(const Vec3& point, const Vec3& path_start, const Vec3& path_end) {
    const auto ax = static_cast<float>(path_start.x);
    const auto ay = static_cast<float>(path_start.y);
    const auto bx = static_cast<float>(path_end.x);
    const auto by = static_cast<float>(path_end.y);
    const auto px = static_cast<float>(point.x);
    const auto py = static_cast<float>(point.y);
    const float vx = bx - ax;
    const float vy = by - ay;
    const float len2 = vx * vx + vy * vy;
    if (len2 <= 1.0e-6F) {
        const float dx = px - ax;
        const float dy = py - ay;
        return std::sqrt(dx * dx + dy * dy);
    }
    const float t = std::clamp(((px - ax) * vx + (py - ay) * vy) / len2, 0.0F, 1.0F);
    const float cx = ax + t * vx;
    const float cy = ay + t * vy;
    const float dx = px - cx;
    const float dy = py - cy;
    return std::sqrt(dx * dx + dy * dy);
}

OccupancyCellSummary synthetic_cell(
    Vec3 center,
    Vec3 size,
    OccupancyCellState state,
    float confidence,
    float distance_to_nearest_occupied_m) {
    OccupancyCellSummary cell;
    cell.center_local = center;
    cell.size_m = size;
    cell.state = state;
    cell.confidence = confidence;
    cell.age_s = 0.0F;
    cell.distance_to_nearest_occupied_m = distance_to_nearest_occupied_m;
    cell.source_provider = "synthetic_track4_fixture";
    return cell;
}

EgoOccupancyMapSnapshot build_synthetic_ego_occupancy(
    const EgoState& ego,
    TimePoint timestamp,
    const MapFrameId& map_frame_id) {
    EgoOccupancyMapSnapshot occupancy;
    occupancy.map_frame_id = map_frame_id;
    occupancy.timestamp = timestamp;
    occupancy.source_kind = OccupancySourceKind::SyntheticFixture;
    occupancy.source_provider = "synthetic_track4_fixture";
    occupancy.resolution_m = 1.0F;
    occupancy.size_m = Vec3{12.0, 8.0, 4.0};
    occupancy.has_valid_occupancy = true;

    const auto& p = ego.local_T_body.position;
    const Vec3 cell_size{1.0, 1.0, 1.0};

    occupancy.debug_cells.push_back(synthetic_cell(
        Vec3{p.x + 5.0, p.y, p.z},
        cell_size,
        OccupancyCellState::Occupied,
        0.85F,
        0.0F));
    occupancy.debug_cells.back().source_object_name = "synthetic_forward_obstacle";

    occupancy.debug_cells.push_back(synthetic_cell(
        Vec3{p.x + 3.0, p.y + 2.0, p.z},
        cell_size,
        OccupancyCellState::Free,
        0.70F,
        2.8F));

    occupancy.debug_cells.push_back(synthetic_cell(
        Vec3{p.x + 7.0, p.y - 2.5, p.z},
        Vec3{2.0, 2.0, 1.5},
        OccupancyCellState::Unknown,
        0.55F,
        2.5F));
    occupancy.debug_cells.back().source_object_name = "synthetic_unknown_sector";

    occupancy.occupied_count = 1U;
    occupancy.free_count = 1U;
    occupancy.unknown_count = 1U;
    occupancy.stale_count = 0U;
    occupancy.nearest_obstacle_distance_m = 5.0F;
    occupancy.forward_corridor_clearance_m = 4.0F;
    return occupancy;
}

SweptVolumeDebug build_synthetic_swept_volume(
    const EgoState& ego,
    const EgoOccupancyMapSnapshot& occupancy,
    TimePoint timestamp,
    const MapFrameId& map_frame_id) {
    SweptVolumeDebug swept;
    swept.map_frame_id = map_frame_id;
    swept.timestamp = timestamp;
    swept.source_provider = "synthetic_track4_swept_volume";
    swept.start_local = ego.local_T_body.position;
    swept.end_local = Vec3{ego.local_T_body.position.x + 8.0, ego.local_T_body.position.y, ego.local_T_body.position.z};
    swept.radius_m = 1.0F;
    swept.horizon_s = 4.0F;
    swept.nominal_speed_mps = 2.0F;
    swept.has_valid_query = occupancy.has_valid_occupancy;

    float min_clearance = std::numeric_limits<float>::infinity();
    bool has_unknown_risk = false;
    bool has_blocking_occupied = false;
    for (const auto& cell : occupancy.debug_cells) {
        const auto dx = static_cast<float>(cell.center_local.x - swept.start_local.x);
        if (dx < 0.0F || dx > static_cast<float>(swept.end_local.x - swept.start_local.x)) {
            continue;
        }
        const float lateral = swept_path_lateral_distance_m(cell.center_local, swept.start_local, swept.end_local);
        const float inflated_clearance = lateral - swept.radius_m - static_cast<float>(std::max(cell.size_m.x, cell.size_m.y)) * 0.5F;
        min_clearance = std::min(min_clearance, inflated_clearance);
        if (cell.state == OccupancyCellState::Occupied && inflated_clearance <= 0.0F) {
            has_blocking_occupied = true;
            swept.blocking_cell_centers.push_back(cell.center_local);
            swept.time_to_collision_s = std::max(0.0F, dx / std::max(0.01F, swept.nominal_speed_mps));
        } else if (cell.state == OccupancyCellState::Unknown && inflated_clearance <= 0.5F) {
            has_unknown_risk = true;
        }
    }

    if (!std::isfinite(min_clearance)) {
        min_clearance = occupancy.forward_corridor_clearance_m;
    }
    swept.min_clearance_m = min_clearance;
    if (!swept.has_valid_query) {
        swept.status = SweptVolumeStatus::StaleMap;
        swept.reason = "occupancy_not_valid";
    } else if (has_blocking_occupied) {
        swept.status = SweptVolumeStatus::OccupiedBlocked;
        swept.reason = "synthetic_occupied_cell_intersects_forward_swept_volume";
    } else if (has_unknown_risk) {
        swept.status = SweptVolumeStatus::UnknownRisk;
        swept.reason = "synthetic_unknown_cell_near_forward_swept_volume";
    } else {
        swept.status = SweptVolumeStatus::Clear;
        swept.reason = "synthetic_forward_swept_volume_clear";
        swept.time_to_collision_s = 0.0F;
    }
    return swept;
}

void refresh_track4_synthetic_products(WorldSnapshot& snapshot) {
    snapshot.has_ego_occupancy = true;
    snapshot.ego_occupancy = build_synthetic_ego_occupancy(
        snapshot.ego,
        snapshot.timestamp,
        snapshot.active_map_frame_id);
    snapshot.has_latest_swept_volume = true;
    snapshot.latest_swept_volume = build_synthetic_swept_volume(
        snapshot.ego,
        snapshot.ego_occupancy,
        snapshot.timestamp,
        snapshot.active_map_frame_id);
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
    refresh_track4_synthetic_products(snapshot_);

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
    refresh_track4_synthetic_products(snapshot_);

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
        agent.velocity_local = observation.velocity_local;
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
    refresh_track4_synthetic_products(snapshot_);

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
