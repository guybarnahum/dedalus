#include "dedalus/world_model/obstacle_products.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace dedalus {
namespace {

constexpr float kPi = 3.14159265358979323846F;
constexpr float kDefaultSensingNearRangeM = 0.5F;
constexpr float kDefaultSensingFarRangeM = 80.0F;
constexpr float kDefaultReliableMinRangeM = 1.0F;
constexpr float kDefaultReliableMaxRangeM = 60.0F;
constexpr float kDefaultHorizontalFovRad = kPi * 0.5F;
constexpr float kDefaultVerticalFovRad = kPi / 3.0F;
constexpr double kEpsilon = 1.0e-9;

float clamp01(float value) {
    return std::max(0.0F, std::min(1.0F, value));
}

float distance_xy(const Vec3& a, const Vec3& b) {
    const auto dx = static_cast<float>(a.x - b.x);
    const auto dy = static_cast<float>(a.y - b.y);
    return std::sqrt(dx * dx + dy * dy);
}

float distance_3d(const Vec3& a, const Vec3& b) {
    const auto dx = static_cast<float>(a.x - b.x);
    const auto dy = static_cast<float>(a.y - b.y);
    const auto dz = static_cast<float>(a.z - b.z);
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double dot(const Vec3& lhs, const Vec3& rhs) {
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

double norm(const Vec3& value) {
    return std::sqrt(dot(value, value));
}

Vec3 normalize_or(const Vec3& value, const Vec3& fallback) {
    const double length = norm(value);
    if (length <= kEpsilon) {
        return fallback;
    }
    return Vec3{value.x / length, value.y / length, value.z / length};
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

OccupancyCellSummary occupancy_cell(
    Vec3 center,
    Vec3 size,
    OccupancyCellState state,
    float confidence,
    float distance_to_nearest_occupied_m,
    std::string source_provider,
    std::string source_object_name = {}) {
    OccupancyCellSummary cell;
    cell.center_local = center;
    cell.size_m = size;
    cell.state = state;
    cell.confidence = confidence;
    cell.age_s = 0.0F;
    cell.distance_to_nearest_occupied_m = distance_to_nearest_occupied_m;
    cell.source_provider = std::move(source_provider);
    cell.source_object_name = std::move(source_object_name);
    return cell;
}

Vec3 class_default_size(ClassLabel label) {
    switch (label) {
        case ClassLabel::Person: return Vec3{0.8, 0.8, 1.8};
        case ClassLabel::Car: return Vec3{4.5, 2.0, 1.7};
        case ClassLabel::Boat: return Vec3{5.0, 2.2, 2.0};
        case ClassLabel::Animal: return Vec3{1.2, 0.8, 1.0};
        case ClassLabel::Tree: return Vec3{1.5, 1.5, 6.0};
        case ClassLabel::Building:
        case ClassLabel::House: return Vec3{8.0, 8.0, 6.0};
        default: return Vec3{1.0, 1.0, 1.0};
    }
}

ObstacleEvidenceState evidence_state_from_occupancy(OccupancyCellState state) {
    switch (state) {
        case OccupancyCellState::Free: return ObstacleEvidenceState::Free;
        case OccupancyCellState::Occupied: return ObstacleEvidenceState::Occupied;
        case OccupancyCellState::Unknown:
        default: return ObstacleEvidenceState::Unknown;
    }
}

ObstacleSensingVolume build_forward_sensing_volume(
    const EgoState& ego,
    TimePoint timestamp,
    const MapFrameId& map_frame_id,
    std::string provider_name) {
    ObstacleSensingVolume volume;
    volume.timestamp = timestamp;
    volume.sensor_name = "front_center";
    volume.provider_name = std::move(provider_name);
    volume.map_frame_id = map_frame_id;
    volume.origin_local = ego.local_T_body.position;
    volume.forward_axis_local = Vec3{1.0, 0.0, 0.0};
    volume.right_axis_local = Vec3{0.0, 1.0, 0.0};
    volume.up_axis_local = Vec3{0.0, 0.0, -1.0};
    volume.near_range_m = kDefaultSensingNearRangeM;
    volume.far_range_m = kDefaultSensingFarRangeM;
    volume.horizontal_fov_rad = kDefaultHorizontalFovRad;
    volume.vertical_fov_rad = kDefaultVerticalFovRad;
    volume.min_reliable_range_m = kDefaultReliableMinRangeM;
    volume.max_reliable_range_m = kDefaultReliableMaxRangeM;
    volume.min_surface_area_m2 = 0.25F;
    volume.min_angular_size_rad = 0.01F;
    volume.min_confidence = 0.30F;
    return volume;
}

bool is_inside_sensing_volume(const Vec3& point, const ObstacleSensingVolume& volume, float* range_out, float* bearing_out, float* elevation_out) {
    const Vec3 delta{
        point.x - volume.origin_local.x,
        point.y - volume.origin_local.y,
        point.z - volume.origin_local.z};
    const Vec3 forward = normalize_or(volume.forward_axis_local, Vec3{1.0, 0.0, 0.0});
    const Vec3 right = normalize_or(volume.right_axis_local, Vec3{0.0, 1.0, 0.0});
    const Vec3 up = normalize_or(volume.up_axis_local, Vec3{0.0, 0.0, -1.0});

    const float forward_m = static_cast<float>(dot(delta, forward));
    const float right_m = static_cast<float>(dot(delta, right));
    const float up_m = static_cast<float>(dot(delta, up));
    const float range = static_cast<float>(norm(delta));
    const float lateral_norm = std::sqrt(forward_m * forward_m + right_m * right_m);
    const float bearing = std::atan2(right_m, forward_m);
    const float elevation = std::atan2(up_m, lateral_norm);
    if (range_out != nullptr) *range_out = range;
    if (bearing_out != nullptr) *bearing_out = bearing;
    if (elevation_out != nullptr) *elevation_out = elevation;
    if (forward_m <= 1.0e-6F || range < volume.near_range_m || range > volume.far_range_m) {
        return false;
    }
    const float half_width_at_forward = std::tan(volume.horizontal_fov_rad * 0.5F) * forward_m;
    const float half_height_at_forward = std::tan(volume.vertical_fov_rad * 0.5F) * forward_m;
    return std::fabs(right_m) <= half_width_at_forward && std::fabs(up_m) <= half_height_at_forward;
}

bool intersects_swept_volume(const Vec3& point, const Vec3& size_m, const SweptVolumeDebug& swept) {
    if (!swept.has_valid_query) {
        return false;
    }
    const auto dx = static_cast<float>(point.x - swept.start_local.x);
    const auto path_dx = static_cast<float>(swept.end_local.x - swept.start_local.x);
    if (dx < 0.0F || dx > path_dx) {
        return false;
    }
    const float lateral = swept_path_lateral_distance_m(point, swept.start_local, swept.end_local);
    const float inflated_clearance = lateral - swept.radius_m - static_cast<float>(std::max(size_m.x, size_m.y)) * 0.5F;
    return inflated_clearance <= 0.0F;
}

ObstacleEvidence evidence_from_cell(
    const OccupancyCellSummary& cell,
    const ObstacleSensingVolume& volume,
    const SweptVolumeDebug& swept,
    OccupancySourceKind source_kind,
    TimePoint timestamp,
    const MapFrameId& map_frame_id,
    std::string provider_name) {
    float range = 0.0F;
    float bearing = 0.0F;
    float elevation = 0.0F;
    const bool inside_sensing = is_inside_sensing_volume(cell.center_local, volume, &range, &bearing, &elevation);
    ObstacleEvidence evidence;
    evidence.timestamp = timestamp;
    evidence.sensor_name = volume.sensor_name;
    evidence.source_provider = std::move(provider_name);
    evidence.source_kind = source_kind;
    evidence.map_frame_id = map_frame_id;
    evidence.state = evidence_state_from_occupancy(cell.state);
    evidence.shape = ObstacleEvidenceShape::Voxel;
    evidence.center_local = cell.center_local;
    evidence.size_m = cell.size_m;
    evidence.radius_m = static_cast<float>(std::max({cell.size_m.x, cell.size_m.y, cell.size_m.z})) * 0.5F;
    evidence.occupancy_probability = cell.state == OccupancyCellState::Occupied ? clamp01(cell.confidence) : 0.0F;
    evidence.free_probability = cell.state == OccupancyCellState::Free ? clamp01(cell.confidence) : 0.0F;
    evidence.confidence = clamp01(cell.confidence);
    evidence.range_m = range;
    evidence.bearing_rad = bearing;
    evidence.elevation_rad = elevation;
    evidence.inside_sensing_volume = inside_sensing;
    evidence.inside_swept_volume = intersects_swept_volume(cell.center_local, cell.size_m, swept);
    evidence.is_static_hint = true;
    evidence.is_thin_structure_hint = false;
    return evidence;
}

ObstacleEvidence evidence_from_observation(
    const Observation3D& observation,
    const ObstacleSensingVolume& volume,
    const SweptVolumeDebug& swept,
    TimePoint timestamp,
    const MapFrameId& map_frame_id) {
    const auto size = class_default_size(observation.class_label);
    float range = 0.0F;
    float bearing = 0.0F;
    float elevation = 0.0F;
    const bool inside_sensing = is_inside_sensing_volume(observation.position_local, volume, &range, &bearing, &elevation);
    ObstacleEvidence evidence;
    evidence.timestamp = timestamp;
    evidence.source_frame_id = observation.source_frame_id;
    evidence.has_source_frame = observation.has_source_frame;
    evidence.sensor_name = volume.sensor_name;
    evidence.source_provider = "airsim_gt_vd";
    evidence.source_kind = OccupancySourceKind::AirSimGroundTruthVisualEmulation;
    evidence.map_frame_id = map_frame_id;
    evidence.state = ObstacleEvidenceState::Occupied;
    evidence.shape = ObstacleEvidenceShape::Voxel;
    evidence.center_local = observation.position_local;
    evidence.size_m = size;
    evidence.radius_m = static_cast<float>(std::max({size.x, size.y, size.z})) * 0.5F;
    evidence.occupancy_probability = clamp01(observation.confidence);
    evidence.free_probability = 0.0F;
    evidence.confidence = clamp01(observation.confidence);
    evidence.range_m = range;
    evidence.bearing_rad = bearing;
    evidence.elevation_rad = elevation;
    evidence.inside_sensing_volume = inside_sensing;
    evidence.inside_swept_volume = intersects_swept_volume(observation.position_local, size, swept);
    evidence.is_static_hint = true;
    evidence.is_thin_structure_hint = false;
    return evidence;
}

EgoOccupancyMapSnapshot build_synthetic_ego_occupancy(
    const EgoState& ego,
    TimePoint timestamp,
    const MapFrameId& map_frame_id) {
    EgoOccupancyMapSnapshot occupancy;
    occupancy.map_frame_id = map_frame_id;
    occupancy.timestamp = timestamp;
    occupancy.source_kind = OccupancySourceKind::SyntheticFixture;
    occupancy.source_provider = "synthetic_obstacle_fixture";
    occupancy.resolution_m = 1.0F;
    occupancy.size_m = Vec3{12.0, 8.0, 4.0};
    occupancy.has_valid_occupancy = true;

    const auto& p = ego.local_T_body.position;
    const Vec3 cell_size{1.0, 1.0, 1.0};
    occupancy.debug_cells.push_back(occupancy_cell(
        Vec3{p.x + 5.0, p.y, p.z}, cell_size, OccupancyCellState::Occupied, 0.85F, 0.0F,
        "synthetic_obstacle_fixture", "synthetic_forward_obstacle"));
    occupancy.debug_cells.push_back(occupancy_cell(
        Vec3{p.x + 3.0, p.y + 2.0, p.z}, cell_size, OccupancyCellState::Free, 0.70F, 2.8F,
        "synthetic_obstacle_fixture"));
    occupancy.debug_cells.push_back(occupancy_cell(
        Vec3{p.x + 7.0, p.y - 2.5, p.z}, Vec3{2.0, 2.0, 1.5}, OccupancyCellState::Unknown, 0.55F, 2.5F,
        "synthetic_obstacle_fixture", "synthetic_unknown_sector"));
    occupancy.occupied_count = 1U;
    occupancy.free_count = 1U;
    occupancy.unknown_count = 1U;
    occupancy.stale_count = 0U;
    occupancy.nearest_obstacle_distance_m = 5.0F;
    occupancy.forward_corridor_clearance_m = 4.0F;
    return occupancy;
}

EgoOccupancyMapSnapshot build_airsim_ground_truth_occupancy(
    const EgoState& ego,
    const PerceptionPipelineOutput& perception_output,
    TimePoint timestamp,
    const MapFrameId& map_frame_id) {
    EgoOccupancyMapSnapshot occupancy;
    occupancy.map_frame_id = map_frame_id;
    occupancy.timestamp = timestamp;
    occupancy.source_kind = OccupancySourceKind::AirSimGroundTruth;
    occupancy.source_provider = "airsim_ground_truth_named_objects";
    occupancy.resolution_m = 1.0F;
    occupancy.size_m = Vec3{60.0, 60.0, 20.0};
    occupancy.has_valid_occupancy = !perception_output.observations.empty();

    float nearest = std::numeric_limits<float>::infinity();
    float forward_clearance = std::numeric_limits<float>::infinity();
    for (const auto& observation : perception_output.observations) {
        const auto size = class_default_size(observation.class_label);
        const float range = distance_3d(observation.position_local, ego.local_T_body.position);
        nearest = std::min(nearest, range);
        const auto dx = static_cast<float>(observation.position_local.x - ego.local_T_body.position.x);
        if (dx >= 0.0F) {
            const float lateral = distance_xy(
                Vec3{observation.position_local.x, observation.position_local.y, 0.0},
                Vec3{observation.position_local.x, ego.local_T_body.position.y, 0.0});
            forward_clearance = std::min(forward_clearance, lateral);
        }
        occupancy.debug_cells.push_back(occupancy_cell(
            observation.position_local,
            size,
            OccupancyCellState::Occupied,
            clamp01(observation.confidence),
            0.0F,
            "airsim_ground_truth_named_objects",
            observation.track_id.value));
    }

    occupancy.occupied_count = static_cast<std::uint32_t>(occupancy.debug_cells.size());
    occupancy.free_count = 0U;
    occupancy.unknown_count = 0U;
    occupancy.stale_count = 0U;
    occupancy.nearest_obstacle_distance_m = std::isfinite(nearest) ? nearest : 0.0F;
    occupancy.forward_corridor_clearance_m = std::isfinite(forward_clearance) ? forward_clearance : occupancy.nearest_obstacle_distance_m;
    return occupancy;
}

SweptVolumeDebug build_swept_volume(
    const EgoState& ego,
    const EgoOccupancyMapSnapshot& occupancy,
    TimePoint timestamp,
    const MapFrameId& map_frame_id) {
    SweptVolumeDebug swept;
    swept.map_frame_id = map_frame_id;
    swept.timestamp = timestamp;
    swept.source_provider = occupancy.source_provider + "_swept_volume";
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
        swept.reason = "occupied_cell_intersects_forward_swept_volume";
    } else if (has_unknown_risk) {
        swept.status = SweptVolumeStatus::UnknownRisk;
        swept.reason = "unknown_cell_near_forward_swept_volume";
    } else {
        swept.status = SweptVolumeStatus::Clear;
        swept.reason = "forward_swept_volume_clear";
        swept.time_to_collision_s = 0.0F;
    }
    return swept;
}

std::vector<ObstacleSensingVolume> visual_emulation_volumes_for(
    const WorldSnapshot& snapshot,
    const PerceptionPipelineOutput& output,
    const std::vector<ObstacleSensingVolume>& configured_volumes) {
    std::vector<ObstacleSensingVolume> volumes = configured_volumes;
    for (auto& volume : volumes) {
        volume.timestamp = snapshot.timestamp;
        volume.map_frame_id = snapshot.active_map_frame_id;
        if (volume.provider_name.empty()) {
            volume.provider_name = "configured_camera_coverage";
        }
        for (const auto& observation : output.observations) {
            if (observation.has_source_frame && !volume.has_source_frame) {
                volume.source_frame_id = observation.source_frame_id;
                volume.has_source_frame = true;
            }
        }
    }
    return volumes;
}

}  // namespace

void refresh_synthetic_obstacle_products(WorldSnapshot& snapshot) {
    snapshot.has_ego_occupancy = true;
    snapshot.ego_occupancy = build_synthetic_ego_occupancy(snapshot.ego, snapshot.timestamp, snapshot.active_map_frame_id);
    snapshot.has_latest_swept_volume = true;
    snapshot.latest_swept_volume = build_swept_volume(snapshot.ego, snapshot.ego_occupancy, snapshot.timestamp, snapshot.active_map_frame_id);
    snapshot.obstacle_sensing_volumes.clear();
    snapshot.obstacle_evidence.clear();
    snapshot.obstacle_sensing_volumes.push_back(build_forward_sensing_volume(
        snapshot.ego, snapshot.timestamp, snapshot.active_map_frame_id, "synthetic_obstacle_fixture"));
    const auto& volume = snapshot.obstacle_sensing_volumes.back();
    for (const auto& cell : snapshot.ego_occupancy.debug_cells) {
        snapshot.obstacle_evidence.push_back(evidence_from_cell(
            cell,
            volume,
            snapshot.latest_swept_volume,
            OccupancySourceKind::SyntheticFixture,
            snapshot.timestamp,
            snapshot.active_map_frame_id,
            "synthetic_obstacle_fixture"));
    }
}

void refresh_ground_truth_obstacle_products(
    WorldSnapshot& snapshot,
    const PerceptionPipelineOutput& output,
    const std::vector<ObstacleSensingVolume>& configured_volumes) {
    snapshot.has_ego_occupancy = true;
    snapshot.ego_occupancy = build_airsim_ground_truth_occupancy(snapshot.ego, output, snapshot.timestamp, snapshot.active_map_frame_id);
    snapshot.has_latest_swept_volume = true;
    snapshot.latest_swept_volume = build_swept_volume(snapshot.ego, snapshot.ego_occupancy, snapshot.timestamp, snapshot.active_map_frame_id);
    snapshot.obstacle_sensing_volumes = visual_emulation_volumes_for(snapshot, output, configured_volumes);
    snapshot.obstacle_evidence.clear();
    for (const auto& sensing_volume : snapshot.obstacle_sensing_volumes) {
        for (const auto& observation : output.observations) {
            auto evidence = evidence_from_observation(
                observation,
                sensing_volume,
                snapshot.latest_swept_volume,
                snapshot.timestamp,
                snapshot.active_map_frame_id);
            if (evidence.inside_sensing_volume && evidence.confidence >= sensing_volume.min_confidence) {
                snapshot.obstacle_evidence.push_back(evidence);
            }
        }
    }
}

}  // namespace dedalus
