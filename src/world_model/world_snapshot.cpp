#include "dedalus/world_model/world_snapshot.hpp"

#include <algorithm>
#include <cmath>
#include <vector>
#include <sstream>
#include <string_view>

namespace dedalus {
namespace {

std::string escape_json(std::string_view value) {
    std::ostringstream out;
    for (const char ch : value) {
        switch (ch) {
            case '"': out << "\\\""; break;
            case '\\': out << "\\\\"; break;
            case '\n': out << "\\n"; break;
            case '\r': out << "\\r"; break;
            case '\t': out << "\\t"; break;
            default: out << ch; break;
        }
    }
    return out.str();
}

const char* to_string(ClassLabel value) {
    switch (value) {
        case ClassLabel::Person: return "person";
        case ClassLabel::Drone: return "drone";
        case ClassLabel::Car: return "car";
        case ClassLabel::Boat: return "boat";
        case ClassLabel::Animal: return "animal";
        case ClassLabel::House: return "house";
        case ClassLabel::Building: return "building";
        case ClassLabel::Tree: return "tree";
        case ClassLabel::Road: return "road";
        case ClassLabel::River: return "river";
        case ClassLabel::Terrain: return "terrain";
        case ClassLabel::Unknown:
        default: return "unknown";
    }
}

const char* to_string(FactionLabel value) {
    switch (value) {
        case FactionLabel::Friendly: return "friendly";
        case FactionLabel::Neutral: return "neutral";
        case FactionLabel::Hostile: return "hostile";
        case FactionLabel::Unknown:
        default: return "unknown";
    }
}

const char* to_string(AgentLifecycle value) {
    switch (value) {
        case AgentLifecycle::New: return "new";
        case AgentLifecycle::Active: return "active";
        case AgentLifecycle::Occluded: return "occluded";
        case AgentLifecycle::Contained: return "contained";
        case AgentLifecycle::Stale: return "stale";
        case AgentLifecycle::Retired: return "retired";
        default: return "unknown";
    }
}

const char* to_string(ContainerType value) {
    switch (value) {
        case ContainerType::Car: return "car";
        case ContainerType::Boat: return "boat";
        case ContainerType::House: return "house";
        case ContainerType::Building: return "building";
        case ContainerType::Garage: return "garage";
        case ContainerType::Room: return "room";
        case ContainerType::Unknown:
        default: return "unknown";
    }
}

const char* to_string(ZoneType value) {
    switch (value) {
        case ZoneType::Cone: return "cone";
        case ZoneType::Cylinder: return "cylinder";
        case ZoneType::Box: return "box";
        case ZoneType::VoxelCluster: return "voxel_cluster";
        default: return "unknown";
    }
}

const char* to_string(LightingMode value) {
    switch (value) {
        case LightingMode::Day: return "day";
        case LightingMode::Night: return "night";
        case LightingMode::Dawn: return "dawn";
        case LightingMode::Dusk: return "dusk";
        case LightingMode::Artificial: return "artificial";
        case LightingMode::Unknown:
        default: return "unknown";
    }
}

const char* to_string(WeatherMode value) {
    switch (value) {
        case WeatherMode::Clear: return "clear";
        case WeatherMode::Rain: return "rain";
        case WeatherMode::Fog: return "fog";
        case WeatherMode::Dust: return "dust";
        case WeatherMode::Snow: return "snow";
        case WeatherMode::Unknown:
        default: return "unknown";
    }
}

const char* to_string(EgoFlightStatus value) {
    switch (value) {
        case EgoFlightStatus::Landed: return "landed";
        case EgoFlightStatus::Airborne: return "airborne";
        case EgoFlightStatus::Unknown:
        default: return "unknown";
    }
}

const char* to_string(FlightControlArmState value) {
    switch (value) {
        case FlightControlArmState::Disarmed: return "disarmed";
        case FlightControlArmState::ArmRequested: return "arm_requested";
        case FlightControlArmState::ArmedConfirmed: return "armed_confirmed";
        case FlightControlArmState::DisarmRequested: return "disarm_requested";
        case FlightControlArmState::DisarmedConfirmed: return "disarmed_confirmed";
        case FlightControlArmState::ArmFailed: return "arm_failed";
        case FlightControlArmState::DisarmFailed: return "disarm_failed";
        case FlightControlArmState::Unknown:
        default: return "unknown";
    }
}

const char* to_string(OccupancyCellState value) {
    switch (value) {
        case OccupancyCellState::Free: return "free";
        case OccupancyCellState::Occupied: return "occupied";
        case OccupancyCellState::Unknown:
        default: return "unknown";
    }
}

const char* to_string(OccupancySourceKind value) {
    switch (value) {
        case OccupancySourceKind::AirSimGroundTruth: return "airsim_gt";
        case OccupancySourceKind::AirSimGroundTruthVisualEmulation: return "airsim_gt_vd";
        case OccupancySourceKind::VisualObstacleDetector: return "visual_obstacle_detector";
        case OccupancySourceKind::DepthProvider: return "depth_provider";
        case OccupancySourceKind::Fused: return "fused";
        case OccupancySourceKind::SyntheticFixture:
        default: return "synthetic_fixture";
    }
}

const char* to_string(ObstacleEvidenceState value) {
    switch (value) {
        case ObstacleEvidenceState::Free: return "free";
        case ObstacleEvidenceState::Occupied: return "occupied";
        case ObstacleEvidenceState::ThinStructureRisk: return "thin_structure_risk";
        case ObstacleEvidenceState::Unknown:
        default: return "unknown";
    }
}

const char* to_string(ObstacleEvidenceShape value) {
    switch (value) {
        case ObstacleEvidenceShape::FrustumBin: return "frustum_bin";
        case ObstacleEvidenceShape::RaySegment: return "ray_segment";
        case ObstacleEvidenceShape::SurfacePatch: return "surface_patch";
        case ObstacleEvidenceShape::LineSegment: return "line_segment";
        case ObstacleEvidenceShape::Capsule: return "capsule";
        case ObstacleEvidenceShape::Voxel:
        default: return "voxel";
    }
}

const char* to_string(SweptVolumeStatus value) {
    switch (value) {
        case SweptVolumeStatus::Clear: return "clear";
        case SweptVolumeStatus::OccupiedBlocked: return "occupied_blocked";
        case SweptVolumeStatus::UnknownRisk: return "unknown_risk";
        case SweptVolumeStatus::StaleMap: return "stale_map";
        case SweptVolumeStatus::Unknown:
        default: return "unknown";
    }
}

const char* bool_string(bool value) { return value ? "true" : "false"; }

void write_vec2(std::ostringstream& out, const Vec2& value) { out << "[" << value.x << "," << value.y << "]"; }
void write_vec3(std::ostringstream& out, const Vec3& value) { out << "[" << value.x << "," << value.y << "," << value.z << "]"; }

void write_rect2(std::ostringstream& out, const Rect2& value) {
    out << "{\"x\":" << value.x << ",\"y\":" << value.y << ",\"width\":" << value.width << ",\"height\":" << value.height << "}";
}

void write_pose3(std::ostringstream& out, const Pose3& value) {
    out << "{\"position_local\":";
    write_vec3(out, value.position);
    out << ",\"rotation_rpy\":";
    write_vec3(out, value.rotation_rpy);
    out << "}";
}

void write_float_or_null(std::ostringstream& out, const float value) {
    if (std::isfinite(value)) {
        out << value;
    } else {
        out << "null";
    }
}

void write_bool_field(std::ostringstream& out, std::string_view name, const bool value) {
    out << "        \"" << name << "\": " << bool_string(value);
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
    if (evidence.has_source_frame) out << ",\n        \"source_frame_id\": \"" << escape_json(evidence.source_frame_id.value) << "\"";
    if (evidence.has_source_detection) out << ",\n        \"source_detection_id\": \"" << escape_json(evidence.source_detection_id.value) << "\"";
    if (evidence.has_source_bbox) {
        out << ",\n        \"source_bbox_px\": ";
        write_rect2(out, evidence.source_bbox_px);
    }
    if (evidence.has_source_center) {
        out << ",\n        \"source_center_px\": ";
        write_vec2(out, evidence.source_center_px);
    }
    if (!evidence.camera_name.empty()) out << ",\n        \"camera_name\": \"" << escape_json(evidence.camera_name) << "\"";
    out << "\n      }";
}

void write_ego_occupancy(std::ostringstream& out, const EgoOccupancyMapSnapshot& occupancy) {
    out << "  \"ego_occupancy\": {\n";
    out << "    \"timestamp_ns\": " << occupancy.timestamp.timestamp_ns << ",\n";
    out << "    \"map_frame_id\": \"" << escape_json(occupancy.map_frame_id.value) << "\",\n";
    out << "    \"source_kind\": \"" << to_string(occupancy.source_kind) << "\",\n";
    out << "    \"source_provider\": \"" << escape_json(occupancy.source_provider) << "\",\n";
    out << "    \"resolution_m\": " << occupancy.resolution_m << ",\n";
    out << "    \"size_m\": "; write_vec3(out, occupancy.size_m); out << ",\n";
    out << "    \"occupied_count\": " << occupancy.occupied_count << ",\n";
    out << "    \"free_count\": " << occupancy.free_count << ",\n";
    out << "    \"unknown_count\": " << occupancy.unknown_count << ",\n";
    out << "    \"stale_count\": " << occupancy.stale_count << ",\n";
    out << "    \"nearest_obstacle_distance_m\": " << occupancy.nearest_obstacle_distance_m << ",\n";
    out << "    \"forward_corridor_clearance_m\": " << occupancy.forward_corridor_clearance_m << ",\n";
    out << "    \"has_valid_occupancy\": " << bool_string(occupancy.has_valid_occupancy) << ",\n";
    out << "    \"debug_cells\": [";
    for (std::size_t i = 0; i < occupancy.debug_cells.size(); ++i) {
        const auto& cell = occupancy.debug_cells[i];
        if (i != 0) out << ",";
        out << "\n      {\n";
        out << "        \"center_local\": "; write_vec3(out, cell.center_local);
        out << ",\n        \"size_m\": "; write_vec3(out, cell.size_m);
        out << ",\n        \"state\": \"" << to_string(cell.state) << "\",\n";
        out << "        \"confidence\": " << cell.confidence << ",\n";
        out << "        \"age_s\": " << cell.age_s << ",\n";
        out << "        \"distance_to_nearest_occupied_m\": " << cell.distance_to_nearest_occupied_m;
        if (!cell.source_provider.empty()) out << ",\n        \"source_provider\": \"" << escape_json(cell.source_provider) << "\"";
        if (!cell.source_object_name.empty()) out << ",\n        \"source_object_name\": \"" << escape_json(cell.source_object_name) << "\"";
        out << "\n      }";
    }
    if (!occupancy.debug_cells.empty()) out << "\n    ";
    out << "]\n";
    out << "  },\n";
}


void write_mission_local_obstacle_map(
    std::ostringstream& out,
    const MissionLocalObstacleMapSnapshot& map) {
    constexpr std::size_t kMaxDebugCells = 128U;

    out << "  \"mission_local_obstacle_map\": {\n";
    out << "    \"map_frame_id\": \"" << escape_json(map.summary.map_frame_id.value) << "\",\n";
    out << "    \"cell_size_m\": " << map.config.cell_size_m << ",\n";
    out << "    \"vertical_cell_size_m\": " << map.config.vertical_cell_size_m << ",\n";
    out << "    \"observed_cell_count\": " << map.summary.observed_cell_count << ",\n";
    out << "    \"occupied_cell_count\": " << map.summary.occupied_cell_count << ",\n";
    out << "    \"free_cell_count\": " << map.summary.free_cell_count << ",\n";
    out << "    \"update_count\": " << map.summary.update_count << ",\n";
    out << "    \"last_update_timestamp_ns\": " << map.summary.last_update_timestamp_ns << ",\n";
    out << "    \"raw_evidence_count\": " << map.summary.raw_evidence_count << ",\n";
    out << "    \"accepted_evidence_count\": " << map.summary.accepted_evidence_count << ",\n";
    out << "    \"compacted_evidence_count\": " << map.summary.compacted_evidence_count << ",\n";
    out << "    \"duplicate_evidence_count\": " << map.summary.duplicate_evidence_count << ",\n";
    out << "    \"dropped_evidence_count\": " << map.summary.dropped_evidence_count << ",\n";
    out << "    \"new_cell_count\": " << map.summary.new_cell_count << ",\n";
    out << "    \"updated_cell_count\": " << map.summary.updated_cell_count << ",\n";

    std::vector<const MissionLocalObstacleCell*> debug_cells;
    debug_cells.reserve(std::min<std::size_t>(map.cells.size(), kMaxDebugCells));
    for (const auto& cell : map.cells) {
        if (!cell.observed) {
            continue;
        }
        debug_cells.push_back(&cell);
    }

    std::sort(debug_cells.begin(), debug_cells.end(), [](const auto* a, const auto* b) {
        if (a->occupied != b->occupied) return a->occupied > b->occupied;
        if (a->occupied_score != b->occupied_score) return a->occupied_score > b->occupied_score;
        return a->last_observed_timestamp_ns > b->last_observed_timestamp_ns;
    });

    if (debug_cells.size() > kMaxDebugCells) {
        debug_cells.resize(kMaxDebugCells);
    }

    out << "    \"debug_cell_limit\": " << kMaxDebugCells << ",\n";
    out << "    \"debug_cells\": [";
    for (std::size_t i = 0; i < debug_cells.size(); ++i) {
        const auto& cell = *debug_cells[i];
        if (i != 0) out << ",";
        out << "\n      {\n";
        out << "        \"center_map\": "; write_vec3(out, cell.center_map); out << ",\n";
        out << "        \"size_m\": "; write_vec3(out, cell.size_m); out << ",\n";
        write_bool_field(out, "observed", cell.observed); out << ",\n";
        write_bool_field(out, "occupied", cell.occupied); out << ",\n";
        write_bool_field(out, "free", cell.free); out << ",\n";
        out << "        \"occupied_score\": " << cell.occupied_score << ",\n";
        out << "        \"free_score\": " << cell.free_score << ",\n";
        out << "        \"risk_score\": " << cell.risk_score << ",\n";
        out << "        \"confidence\": " << cell.confidence << ",\n";
        out << "        \"first_observed_timestamp_ns\": " << cell.first_observed_timestamp_ns << ",\n";
        out << "        \"last_observed_timestamp_ns\": " << cell.last_observed_timestamp_ns << ",\n";
        out << "        \"min_z_m\": " << cell.min_z_m << ",\n";
        out << "        \"max_z_m\": " << cell.max_z_m << ",\n";
        out << "        \"positive_observation_count\": " << cell.positive_observation_count << ",\n";
        out << "        \"negative_observation_count\": " << cell.negative_observation_count << ",\n";
        out << "        \"same_update_duplicate_count\": " << cell.same_update_duplicate_count << ",\n";
        out << "        \"last_confirmed_occupied_timestamp_ns\": " << cell.last_confirmed_occupied_timestamp_ns << ",\n";
        out << "        \"last_observed_free_timestamp_ns\": " << cell.last_observed_free_timestamp_ns << ",\n";
        out << "        \"last_source_kind\": \"" << to_string(cell.last_source_kind) << "\",\n";
        out << "        \"last_source_provider\": \"" << escape_json(cell.last_source_provider) << "\"\n";
        out << "      }";
    }
    if (!debug_cells.empty()) out << "\n    ";
    out << "]\n";
    out << "  },\n";
}

void write_local_flight_map(std::ostringstream& out, const LocalFlightMapSnapshot& map) {
    constexpr std::size_t kMaxDebugCells = 96U;

    out << "  \"local_flight_map\": {\n";
    out << "    \"timestamp_ns\": " << map.timestamp.timestamp_ns << ",\n";
    if (map.has_source_frame) {
        out << "    \"source_frame_id\": \"" << escape_json(map.source_frame_id.value) << "\",\n";
    }
    out << "    \"cell_size_m\": " << map.cell_size_m << ",\n";
    out << "    \"x_cells\": " << map.x_cells << ",\n";
    out << "    \"y_cells\": " << map.y_cells << ",\n";
    out << "    \"forward_range_m\": " << map.forward_range_m << ",\n";
    out << "    \"rear_range_m\": " << map.rear_range_m << ",\n";
    out << "    \"lateral_range_m\": " << map.lateral_range_m << ",\n";
    out << "    \"occupied_count\": " << map.occupied_count << ",\n";
    out << "    \"inflated_blocked_count\": " << map.inflated_blocked_count << ",\n";
    out << "    \"nearest_obstacle_m\": ";
    write_float_or_null(out, map.nearest_obstacle_m);
    out << ",\n";
    out << "    \"source_mission_cell_count\": " << map.source_mission_cell_count << ",\n";
    out << "    \"projected_mission_cell_count\": " << map.projected_mission_cell_count << ",\n";
    out << "    \"projected_local_cell_update_count\": " << map.projected_local_cell_update_count << ",\n";
    out << "    \"exclusion_inflation_radius_m\": " << map.exclusion_inflation_radius_m << ",\n";

    std::vector<const LocalFlightMapCell*> debug_cells;
    debug_cells.reserve(std::min<std::size_t>(map.cells.size(), kMaxDebugCells));
    for (const auto& cell : map.cells) {
        if (!cell.occupied && !cell.inflated_blocked) {
            continue;
        }
        debug_cells.push_back(&cell);
    }

    std::sort(debug_cells.begin(), debug_cells.end(), [](const auto* a, const auto* b) {
        if (a->occupied != b->occupied) return a->occupied > b->occupied;
        if (a->inflated_blocked != b->inflated_blocked) return a->inflated_blocked > b->inflated_blocked;
        if (a->nearest_range_m != b->nearest_range_m) return a->nearest_range_m < b->nearest_range_m;
        const auto ar = (a->center_local.x * a->center_local.x) + (a->center_local.y * a->center_local.y);
        const auto br = (b->center_local.x * b->center_local.x) + (b->center_local.y * b->center_local.y);
        return ar < br;
    });

    if (debug_cells.size() > kMaxDebugCells) {
        debug_cells.resize(kMaxDebugCells);
    }

    out << "    \"debug_cell_limit\": " << kMaxDebugCells << ",\n";
    out << "    \"debug_cells\": [";
    for (std::size_t i = 0; i < debug_cells.size(); ++i) {
        const auto& cell = *debug_cells[i];
        if (i != 0) out << ",";
        out << "\n      {\n";
        out << "        \"center_local\": "; write_vec3(out, cell.center_local); out << ",\n";
        out << "        \"size_m\": [" << map.cell_size_m << "," << map.cell_size_m << ",0.12],\n";
        write_bool_field(out, "occupied", cell.occupied); out << ",\n";
        write_bool_field(out, "inflated_blocked", cell.inflated_blocked); out << ",\n";
        write_bool_field(out, "recently_observed", cell.recently_observed); out << ",\n";
        out << "        \"occupied_score\": " << cell.occupied_score << ",\n";
        out << "        \"free_score\": " << cell.free_score << ",\n";
        out << "        \"risk_score\": " << cell.risk_score << ",\n";
        out << "        \"nearest_range_m\": ";
        write_float_or_null(out, cell.nearest_range_m);
        out << ",\n";
        out << "        \"min_z_m\": " << cell.min_z_m << ",\n";
        out << "        \"max_z_m\": " << cell.max_z_m << "\n";
        out << "      }";
    }
    if (!debug_cells.empty()) out << "\n    ";
    out << "],\n";

    // ── L0 polar risk sectors ────────────────────────────────────────────────
    out << "    \"ego_speed_mps\": " << map.ego_speed_mps << ",\n";
    out << "    \"global_min_ttc_s\": ";
    write_float_or_null(out, map.global_min_ttc_s);
    out << ",\n";
    out << "    \"escape_direction_body\": "; write_vec3(out, map.escape_direction_body); out << ",\n";
    out << "    \"polar_risk_sectors\": [";
    for (std::size_t i = 0; i < map.polar_risk_sectors.size(); ++i) {
        const auto& s = map.polar_risk_sectors[i];
        if (i != 0) out << ",";
        out << "\n      {";
        out << "\"az\":" << s.azimuth_deg;
        out << ",\"vr\":" << s.max_closing_speed_mps;
        out << ",\"ttc\":"; write_float_or_null(out, s.min_ttc_s);
        out << ",\"nr\":";  write_float_or_null(out, s.nearest_range_m);
        out << ",\"obs\":" << (s.has_obstacle ? "true" : "false");
        out << "}";
    }
    if (!map.polar_risk_sectors.empty()) out << "\n    ";
    out << "],\n";

    // ── 2-D spherical risk bins ───────────────────────────────────────────────
    out << "    \"spherical_num_az\": " << map.spherical_num_az << ",\n";
    out << "    \"spherical_num_el\": " << map.spherical_num_el << ",\n";
    out << "    \"spherical_risk_bins\": [";
    {
        bool first_bin = true;
        for (const auto& b : map.spherical_risk_bins) {
            if (!b.has_obstacle) { continue; }   // omit empty bins — viewer treats gap as safe
            if (!first_bin) { out << ","; }
            first_bin = false;
            out << "\n      {";
            out << "\"az\":" << b.az_centre_deg;
            out << ",\"el\":" << b.el_centre_deg;
            out << ",\"ttc\":"; write_float_or_null(out, b.min_ttc_s);
            out << ",\"vr\":" << b.max_closing_speed_mps;
            out << ",\"nr\":";  write_float_or_null(out, b.nearest_range_m);
            out << ",\"sm\":" << static_cast<int>(b.source_mask);
            out << "}";
        }
        if (!first_bin) { out << "\n    "; }
    }
    out << "],\n";

    // ── Authoritative sensor scope metadata ──────────────────────────────────
    out << "    \"sensor_az_half_rad\": " << map.sensor_az_half_rad << ",\n";
    out << "    \"sensor_el_half_rad\": " << map.sensor_el_half_rad << ",\n";
    out << "    \"sensor_grid_cols\": "   << map.sensor_grid_cols   << ",\n";
    out << "    \"sensor_grid_rows\": "   << map.sensor_grid_rows   << ",\n";

    // ── Per-sensor observations (source-tagged, body-frame spherical) ─────────
    out << "    \"sensor_observations\": [";
    for (std::size_t i = 0; i < map.sensor_observations.size(); ++i) {
        const auto& o = map.sensor_observations[i];
        if (i != 0) out << ",";
        out << "\n      {";
        out << "\"az\":" << o.az_body_rad;
        out << ",\"el\":" << o.el_body_rad;
        out << ",\"r\":";  write_float_or_null(out, o.range_m);
        out << ",\"vr\":" << o.closing_speed_mps;
        out << ",\"ttc\":"; write_float_or_null(out, o.ttc_s);
        out << ",\"src\":" << static_cast<int>(o.source_kind);
        out << "}";
    }
    if (!map.sensor_observations.empty()) out << "\n    ";
    out << "]\n";
    out << "  },\n";
}

void write_trajectory_safety(std::ostringstream& out, const TrajectorySafetyResult& safety) {
    out << "  \"trajectory_safety\": {\n";
    out << "    \"clear\": " << bool_string(safety.clear) << ",\n";
    out << "    \"blocked\": " << bool_string(safety.blocked) << ",\n";
    out << "    \"has_valid_query\": " << bool_string(safety.has_valid_query) << ",\n";
    out << "    \"sample_count\": " << safety.sample_count << ",\n";
    out << "    \"blocked_sample_count\": " << safety.blocked_sample_count << ",\n";
    out << "    \"first_blocked_sample_index\": " << safety.first_blocked_sample_index << ",\n";
    out << "    \"first_blocked_position_local\": "; write_vec3(out, safety.first_blocked_position_local); out << ",\n";
    out << "    \"minimum_clearance_m\": "; write_float_or_null(out, safety.minimum_clearance_m); out << ",\n";
    out << "    \"nearest_obstacle_m\": "; write_float_or_null(out, safety.nearest_obstacle_m); out << "\n";
    out << "  },\n";
}

void write_swept_volume(std::ostringstream& out, const SweptVolumeDebug& swept) {
    out << "  \"latest_swept_volume\": {\n";
    out << "    \"timestamp_ns\": " << swept.timestamp.timestamp_ns << ",\n";
    out << "    \"map_frame_id\": \"" << escape_json(swept.map_frame_id.value) << "\",\n";
    out << "    \"status\": \"" << to_string(swept.status) << "\",\n";
    out << "    \"source_provider\": \"" << escape_json(swept.source_provider) << "\",\n";
    out << "    \"reason\": \"" << escape_json(swept.reason) << "\",\n";
    out << "    \"start_local\": "; write_vec3(out, swept.start_local); out << ",\n";
    out << "    \"end_local\": "; write_vec3(out, swept.end_local); out << ",\n";
    out << "    \"radius_m\": " << swept.radius_m << ",\n";
    out << "    \"horizon_s\": " << swept.horizon_s << ",\n";
    out << "    \"nominal_speed_mps\": " << swept.nominal_speed_mps << ",\n";
    out << "    \"min_clearance_m\": " << swept.min_clearance_m << ",\n";
    out << "    \"time_to_collision_s\": " << swept.time_to_collision_s << ",\n";
    out << "    \"has_valid_query\": " << bool_string(swept.has_valid_query) << ",\n";
    out << "    \"blocking_cell_centers\": [";
    for (std::size_t i = 0; i < swept.blocking_cell_centers.size(); ++i) {
        if (i != 0) out << ",";
        write_vec3(out, swept.blocking_cell_centers[i]);
    }
    out << "]\n";
    out << "  },\n";
}

void write_obstacle_sensing_volumes(std::ostringstream& out, const std::vector<ObstacleSensingVolume>& volumes) {
    out << "  \"obstacle_sensing_volumes\": [";
    for (std::size_t i = 0; i < volumes.size(); ++i) {
        const auto& volume = volumes[i];
        if (i != 0) out << ",";
        out << "\n    {\n";
        out << "      \"timestamp_ns\": " << volume.timestamp.timestamp_ns << ",\n";
        out << "      \"sensor_name\": \"" << escape_json(volume.sensor_name) << "\",\n";
        out << "      \"provider_name\": \"" << escape_json(volume.provider_name) << "\",\n";
        if (volume.has_source_frame) {
            out << "      \"source_frame_id\": \"" << escape_json(volume.source_frame_id.value) << "\",\n";
        }
        out << "      \"map_frame_id\": \"" << escape_json(volume.map_frame_id.value) << "\",\n";
        out << "      \"origin_local\": "; write_vec3(out, volume.origin_local); out << ",\n";
        out << "      \"forward_axis_local\": "; write_vec3(out, volume.forward_axis_local); out << ",\n";
        out << "      \"right_axis_local\": "; write_vec3(out, volume.right_axis_local); out << ",\n";
        out << "      \"up_axis_local\": "; write_vec3(out, volume.up_axis_local); out << ",\n";
        out << "      \"near_range_m\": " << volume.near_range_m << ",\n";
        out << "      \"far_range_m\": " << volume.far_range_m << ",\n";
        out << "      \"horizontal_fov_rad\": " << volume.horizontal_fov_rad << ",\n";
        out << "      \"vertical_fov_rad\": " << volume.vertical_fov_rad << ",\n";
        out << "      \"min_reliable_range_m\": " << volume.min_reliable_range_m << ",\n";
        out << "      \"max_reliable_range_m\": " << volume.max_reliable_range_m << ",\n";
        out << "      \"min_surface_area_m2\": " << volume.min_surface_area_m2 << ",\n";
        out << "      \"min_angular_size_rad\": " << volume.min_angular_size_rad << ",\n";
        out << "      \"min_confidence\": " << volume.min_confidence << "\n";
        out << "    }";
    }
    if (!volumes.empty()) out << "\n  ";
    out << "],\n";
}

void write_obstacle_evidence(std::ostringstream& out, const std::vector<ObstacleEvidence>& evidence_list) {
    out << "  \"obstacle_evidence\": [";
    for (std::size_t i = 0; i < evidence_list.size(); ++i) {
        const auto& evidence = evidence_list[i];
        if (i != 0) out << ",";
        out << "\n    {\n";
        out << "      \"timestamp_ns\": " << evidence.timestamp.timestamp_ns << ",\n";
        out << "      \"sensor_name\": \"" << escape_json(evidence.sensor_name) << "\",\n";
        out << "      \"source_provider\": \"" << escape_json(evidence.source_provider) << "\",\n";
        out << "      \"source_kind\": \"" << to_string(evidence.source_kind) << "\",\n";
        if (evidence.has_source_frame) {
            out << "      \"source_frame_id\": \"" << escape_json(evidence.source_frame_id.value) << "\",\n";
        }
        out << "      \"map_frame_id\": \"" << escape_json(evidence.map_frame_id.value) << "\",\n";
        out << "      \"state\": \"" << to_string(evidence.state) << "\",\n";
        out << "      \"shape\": \"" << to_string(evidence.shape) << "\",\n";
        out << "      \"center_local\": "; write_vec3(out, evidence.center_local); out << ",\n";
        out << "      \"size_m\": "; write_vec3(out, evidence.size_m); out << ",\n";
        out << "      \"endpoint_a_local\": "; write_vec3(out, evidence.endpoint_a_local); out << ",\n";
        out << "      \"endpoint_b_local\": "; write_vec3(out, evidence.endpoint_b_local); out << ",\n";
        out << "      \"radius_m\": " << evidence.radius_m << ",\n";
        out << "      \"has_surface_normal\": " << bool_string(evidence.has_surface_normal) << ",\n";
        out << "      \"surface_normal_local\": "; write_vec3(out, evidence.surface_normal_local); out << ",\n";
        out << "      \"normal_confidence\": " << evidence.normal_confidence << ",\n";
        out << "      \"occupancy_probability\": " << evidence.occupancy_probability << ",\n";
        out << "      \"free_probability\": " << evidence.free_probability << ",\n";
        out << "      \"confidence\": " << evidence.confidence << ",\n";
        out << "      \"range_m\": " << evidence.range_m << ",\n";
        out << "      \"bearing_rad\": " << evidence.bearing_rad << ",\n";
        out << "      \"elevation_rad\": " << evidence.elevation_rad << ",\n";
        out << "      \"inside_sensing_volume\": " << bool_string(evidence.inside_sensing_volume) << ",\n";
        out << "      \"inside_swept_volume\": " << bool_string(evidence.inside_swept_volume) << ",\n";
        out << "      \"is_static_hint\": " << bool_string(evidence.is_static_hint) << ",\n";
        out << "      \"is_thin_structure_hint\": " << bool_string(evidence.is_thin_structure_hint) << ",\n";
        out << "      \"is_surface_hint\": " << bool_string(evidence.is_surface_hint) << "\n";
        out << "    }";
    }
    if (!evidence_list.empty()) out << "\n  ";
    out << "],\n";
}

void write_perch_candidates(std::ostringstream& out, const std::vector<PerchCandidate>& candidates) {
    out << "  \"perch_candidates\": [";
    for (std::size_t i = 0; i < candidates.size(); ++i) {
        const auto& c = candidates[i];
        if (i != 0) out << ",";
        out << "\n    {\n";
        out << "      \"position_local\": "; write_vec3(out, c.position_local); out << ",\n";
        out << "      \"normal_local\": "; write_vec3(out, c.normal_local); out << ",\n";
        out << "      \"score\": " << c.score << "\n";
        out << "    }";
    }
    if (!candidates.empty()) out << "\n  ";
    out << "]\n";
}

}  // namespace

std::string to_json(const WorldSnapshot& snapshot) {
    std::ostringstream out;
    out << "{\n";
    out << "  \"timestamp_ns\": " << snapshot.timestamp.timestamp_ns << ",\n";
    out << "  \"active_map_frame_id\": \"" << escape_json(snapshot.active_map_frame_id.value) << "\",\n";
    out << "  \"depth_source_name\": \"" << escape_json(snapshot.depth_source_name) << "\",\n";
    out << "  \"pipeline\": {\n";
    out << "    \"ego\": \""               << escape_json(snapshot.ego_provider_name)      << "\",\n";
    out << "    \"depth\": \""             << escape_json(snapshot.depth_source_name)      << "\",\n";
    out << "    \"detector\": \""          << escape_json(snapshot.detector_name)          << "\",\n";
    out << "    \"camera_stabilizer\": \"" << escape_json(snapshot.camera_stabilizer_name) << "\",\n";
    out << "    \"tracker\": \""           << escape_json(snapshot.tracker_name)           << "\",\n";
    out << "    \"identity_resolver\": \"" << escape_json(snapshot.identity_resolver_name) << "\",\n";
    out << "    \"projector\": \""         << escape_json(snapshot.projector_name)         << "\"\n";
    out << "  },\n";
    out << "  \"appearance_condition\": {\n";
    out << "    \"lighting_mode\": \"" << to_string(snapshot.appearance_condition.lighting_mode) << "\",\n";
    out << "    \"weather_mode\": \"" << to_string(snapshot.appearance_condition.weather_mode) << "\",\n";
    out << "    \"confidence\": " << snapshot.appearance_condition.confidence << "\n";
    out << "  },\n";

    out << "  \"ego\": {\n";
    out << "    \"timestamp_ns\": " << snapshot.ego.timestamp.timestamp_ns << ",\n";
    out << "    \"map_frame_id\": \"" << escape_json(snapshot.ego.map_frame_id.value) << "\",\n";
    out << "    \"position_local\": "; write_vec3(out, snapshot.ego.local_T_body.position); out << ",\n";
    out << "    \"rotation_rpy\": "; write_vec3(out, snapshot.ego.local_T_body.rotation_rpy); out << ",\n";
    out << "    \"velocity_local\": "; write_vec3(out, snapshot.ego.velocity_local); out << ",\n";
    out << "    \"angular_velocity_body\": "; write_vec3(out, snapshot.ego.angular_velocity_body); out << ",\n";
    out << "    \"height_m\": " << snapshot.ego.height_m << ",\n";
    out << "    \"height_valid\": " << bool_string(snapshot.ego.height_valid) << ",\n";
    out << "    \"armed\": " << bool_string(snapshot.ego.armed) << ",\n";
    out << "    \"armed_valid\": " << bool_string(snapshot.ego.armed_valid) << ",\n";
    out << "    \"flight_status\": \"" << to_string(snapshot.ego.flight_status) << "\",\n";
    out << "    \"confidence\": " << snapshot.ego.confidence;
    if (snapshot.ego.home_T_body.has_value()) { out << ",\n    \"home_pose\": "; write_pose3(out, *snapshot.ego.home_T_body); }
    if (snapshot.ego.home_timestamp.has_value()) out << ",\n    \"home_timestamp_ns\": " << snapshot.ego.home_timestamp->timestamp_ns;
    out << "\n  },\n";

    out << "  \"flight_control\": {\n";
    out << "    \"arm_state\": \"" << to_string(snapshot.flight_control.arm_state) << "\",\n";
    out << "    \"updated_at_ns\": " << snapshot.flight_control.updated_at.timestamp_ns << ",\n";
    out << "    \"last_arm_request_at_ns\": " << snapshot.flight_control.last_arm_request_at.timestamp_ns << ",\n";
    out << "    \"last_disarm_request_at_ns\": " << snapshot.flight_control.last_disarm_request_at.timestamp_ns << ",\n";
    out << "    \"status\": \"" << escape_json(snapshot.flight_control.status) << "\"\n";
    out << "  },\n";

    if (snapshot.has_ego_occupancy) write_ego_occupancy(out, snapshot.ego_occupancy);
    if (snapshot.has_latest_swept_volume) write_swept_volume(out, snapshot.latest_swept_volume);
    if (snapshot.has_mission_local_obstacle_map) write_mission_local_obstacle_map(out, snapshot.mission_local_obstacle_map);
    if (snapshot.has_local_flight_map) write_local_flight_map(out, snapshot.local_flight_map);
    if (snapshot.has_trajectory_safety) write_trajectory_safety(out, snapshot.trajectory_safety);
    write_obstacle_sensing_volumes(out, snapshot.obstacle_sensing_volumes);
    write_obstacle_evidence(out, snapshot.obstacle_evidence);

    out << "  \"agents\": [";
    for (std::size_t i = 0; i < snapshot.agents.size(); ++i) {
        const auto& agent = snapshot.agents[i];
        if (i != 0) out << ",";
        out << "\n    {\n";
        out << "      \"agent_id\": \"" << escape_json(agent.agent_id.value) << "\",\n";
        out << "      \"identity_id\": \"" << escape_json(agent.identity_id.value) << "\",\n";
        out << "      \"source_track_id\": \"" << escape_json(agent.source_track_id.value) << "\",\n";
        if (agent.has_source_detection) out << "      \"source_detection_id\": \"" << escape_json(agent.source_detection_id.value) << "\",\n";
        if (agent.has_source_frame) out << "      \"source_frame_id\": \"" << escape_json(agent.source_frame_id.value) << "\",\n";
        if (agent.has_source_bbox) { out << "      \"source_bbox_px\": "; write_rect2(out, agent.source_bbox_px); out << ",\n"; }
        if (agent.has_latest_view_evidence) { out << "      \"latest_view_evidence\": "; write_agent_view_evidence(out, agent.latest_view_evidence); out << ",\n"; }
        out << "      \"class\": \"" << to_string(agent.class_label) << "\",\n";
        out << "      \"faction\": \"" << to_string(agent.faction) << "\",\n";
        out << "      \"lifecycle\": \"" << to_string(agent.lifecycle) << "\",\n";
        out << "      \"position_local\": "; write_vec3(out, agent.position_local); out << ",\n";
        out << "      \"velocity_local\": "; write_vec3(out, agent.velocity_local); out << ",\n";
        out << "      \"confidence\": " << agent.confidence << "\n";
        out << "    }";
    }
    if (!snapshot.agents.empty()) out << "\n  ";
    out << "],\n";

    out << "  \"containers\": [";
    for (std::size_t i = 0; i < snapshot.containers.size(); ++i) {
        const auto& container = snapshot.containers[i];
        if (i != 0) out << ",";
        out << "\n    {\n";
        out << "      \"container_id\": \"" << escape_json(container.container_id.value) << "\",\n";
        out << "      \"type\": \"" << to_string(container.type) << "\",\n";
        out << "      \"confidence\": " << container.confidence << "\n";
        out << "    }";
    }
    if (!snapshot.containers.empty()) out << "\n  ";
    out << "],\n";

    out << "  \"containment_events\": [],\n";

    out << "  \"tactical_exclusion_zones\": [";
    for (std::size_t i = 0; i < snapshot.tactical_exclusion_zones.size(); ++i) {
        const auto& zone = snapshot.tactical_exclusion_zones[i];
        if (i != 0) out << ",";
        out << "\n    {\n";
        out << "      \"zone_id\": \"" << escape_json(zone.zone_id.value) << "\",\n";
        out << "      \"type\": \"" << to_string(zone.type) << "\",\n";
        out << "      \"position_local\": "; write_vec3(out, zone.local_T_zone.position); out << ",\n";
        out << "      \"dimensions\": "; write_vec3(out, zone.dimensions); out << ",\n";
        out << "      \"inflation_radius_m\": " << zone.inflation_radius_m << ",\n";
        out << "      \"reason\": \"" << escape_json(zone.reason) << "\",\n";
        out << "      \"confidence\": " << zone.confidence << "\n";
        out << "    }";
    }
    if (!snapshot.tactical_exclusion_zones.empty()) out << "\n  ";
    out << "],\n";

    out << "  \"flight_corridors\": [";
    for (std::size_t i = 0; i < snapshot.flight_corridors.size(); ++i) {
        const auto& corridor = snapshot.flight_corridors[i];
        if (i != 0) out << ",";
        out << "\n    {\n";
        out << "      \"corridor_id\": \"" << escape_json(corridor.corridor_id.value) << "\",\n";
        out << "      \"radius_m\": " << corridor.radius_m << ",\n";
        out << "      \"min_altitude_m\": " << corridor.min_altitude_m << ",\n";
        out << "      \"max_altitude_m\": " << corridor.max_altitude_m << ",\n";
        out << "      \"confidence\": " << corridor.confidence << "\n";
        out << "    }";
    }
    if (!snapshot.flight_corridors.empty()) out << "\n  ";
    out << "],\n";

    out << "  \"static_structures\": [";
    for (std::size_t i = 0; i < snapshot.static_structures.size(); ++i) {
        const auto& structure = snapshot.static_structures[i];
        if (i != 0) out << ",";
        out << "\n    {\n";
        out << "      \"structure_id\": \"" << escape_json(structure.structure_id.value) << "\",\n";
        out << "      \"type\": \"" << escape_json(structure.type) << "\",\n";
        out << "      \"bounds\": "; write_bounds3(out, structure.bounds); out << ",\n";
        out << "      \"confidence\": " << structure.confidence << "\n";
        out << "    }";
    }
    if (!snapshot.static_structures.empty()) out << "\n  ";
    out << "],\n";

    out << "  \"landmarks\": [";
    for (std::size_t i = 0; i < snapshot.landmarks.size(); ++i) {
        const auto& landmark = snapshot.landmarks[i];
        if (i != 0) out << ",";
        out << "\n    {\n";
        out << "      \"landmark_id\": \"" << escape_json(landmark.landmark_id.value) << "\",\n";
        out << "      \"type\": \"" << escape_json(landmark.type) << "\",\n";
        out << "      \"position_local\": "; write_vec3(out, landmark.position_local); out << ",\n";
        out << "      \"confidence\": " << landmark.confidence << "\n";
        out << "    }";
    }
    if (!snapshot.landmarks.empty()) out << "\n  ";
    out << "],\n";

    out << "  \"uncertain_regions\": [";
    for (std::size_t i = 0; i < snapshot.uncertain_regions.size(); ++i) {
        const auto& region = snapshot.uncertain_regions[i];
        if (i != 0) out << ",";
        out << "\n    {\n";
        out << "      \"region_id\": \"" << escape_json(region.region_id) << "\",\n";
        out << "      \"bounds\": "; write_bounds3(out, region.bounds); out << ",\n";
        out << "      \"uncertainty\": " << region.uncertainty << ",\n";
        out << "      \"reason\": \"" << escape_json(region.reason) << "\"\n";
        out << "    }";
    }
    if (!snapshot.uncertain_regions.empty()) out << "\n  ";
    out << "],\n";

    out << "  \"map_frames\": [";
    for (std::size_t i = 0; i < snapshot.map_frames.size(); ++i) {
        const auto& frame = snapshot.map_frames[i];
        if (i != 0) out << ",";
        out << "\n    {\n";
        out << "      \"map_frame_id\": \"" << escape_json(frame.map_frame_id.value) << "\",\n";
        out << "      \"scale_confidence\": " << frame.scale_confidence << ",\n";
        out << "      \"orientation_confidence\": " << frame.orientation_confidence << "\n";
        out << "    }";
    }
    if (!snapshot.map_frames.empty()) out << "\n  ";
    out << "],\n";

    write_perch_candidates(out, snapshot.perch_candidates);

    out << "}\n";
    return out.str();
}

}  // namespace dedalus
