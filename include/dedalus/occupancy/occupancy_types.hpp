#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "dedalus/core/types.hpp"

namespace dedalus {

enum class OccupancyCellState {
    Unknown,
    Free,
    Occupied,
};

enum class OccupancySourceKind {
    SyntheticFixture,
    AirSimGroundTruth,
    AirSimGroundTruthVisualEmulation,
    VisualObstacleDetector,
    DepthProvider,
    Fused,
};

enum class ObstacleEvidenceState {
    Unknown,
    Free,
    Occupied,
    ThinStructureRisk,
};

enum class ObstacleEvidenceShape {
    Voxel,
    FrustumBin,
    RaySegment,
    SurfacePatch,
    LineSegment,
    Capsule,
};

enum class SweptVolumeStatus {
    Unknown,
    Clear,
    OccupiedBlocked,
    UnknownRisk,
    StaleMap,
};

struct ObstacleSensingVolume {
    TimePoint timestamp;
    FrameId source_frame_id;
    bool has_source_frame{false};
    std::string sensor_name{"front_center"};
    std::string provider_name{"none"};
    MapFrameId map_frame_id{"map_unknown"};

    Vec3 origin_local;
    Vec3 forward_axis_local{1.0, 0.0, 0.0};
    Vec3 right_axis_local{0.0, 1.0, 0.0};
    Vec3 up_axis_local{0.0, 0.0, -1.0};

    float near_range_m{0.0F};
    float far_range_m{0.0F};
    float horizontal_fov_rad{0.0F};
    float vertical_fov_rad{0.0F};
    float min_reliable_range_m{0.0F};
    float max_reliable_range_m{0.0F};

    float min_surface_area_m2{0.0F};
    float min_angular_size_rad{0.0F};
    float min_confidence{0.0F};
};

struct ObstacleEvidence {
    TimePoint timestamp;
    FrameId source_frame_id;
    bool has_source_frame{false};
    std::string sensor_name{"front_center"};
    std::string source_provider{"none"};
    OccupancySourceKind source_kind{OccupancySourceKind::SyntheticFixture};
    MapFrameId map_frame_id{"map_unknown"};

    ObstacleEvidenceState state{ObstacleEvidenceState::Unknown};
    ObstacleEvidenceShape shape{ObstacleEvidenceShape::Voxel};

    Vec3 center_local;
    Vec3 size_m;
    Vec3 endpoint_a_local;
    Vec3 endpoint_b_local;
    float radius_m{0.0F};

    bool has_surface_normal{false};
    Vec3 surface_normal_local;
    float normal_confidence{0.0F};

    float occupancy_probability{0.0F};
    float free_probability{0.0F};
    float confidence{0.0F};
    float range_m{0.0F};
    float bearing_rad{0.0F};
    float elevation_rad{0.0F};

    bool inside_sensing_volume{false};
    bool inside_swept_volume{false};
    bool is_static_hint{false};
    bool is_thin_structure_hint{false};
    bool is_surface_hint{false};
};

struct OccupancyCellSummary {
    Vec3 center_local;
    Vec3 size_m;
    OccupancyCellState state{OccupancyCellState::Unknown};
    float confidence{0.0F};
    float age_s{0.0F};
    float distance_to_nearest_occupied_m{0.0F};
    std::string source_provider;
    std::string source_object_name;
};

struct EgoOccupancyMapSnapshot {
    MapFrameId map_frame_id{"map_unknown"};
    TimePoint timestamp;
    OccupancySourceKind source_kind{OccupancySourceKind::SyntheticFixture};
    std::string source_provider{"none"};

    float resolution_m{0.0F};
    Vec3 size_m;

    std::uint32_t occupied_count{0U};
    std::uint32_t free_count{0U};
    std::uint32_t unknown_count{0U};
    std::uint32_t stale_count{0U};

    float nearest_obstacle_distance_m{0.0F};
    float forward_corridor_clearance_m{0.0F};
    bool has_valid_occupancy{false};

    std::vector<OccupancyCellSummary> debug_cells;
};

struct SweptVolumeDebug {
    MapFrameId map_frame_id{"map_unknown"};
    TimePoint timestamp;
    SweptVolumeStatus status{SweptVolumeStatus::Unknown};
    std::string source_provider{"none"};
    std::string reason;

    Vec3 start_local;
    Vec3 end_local;
    float radius_m{0.0F};
    float horizon_s{0.0F};
    float nominal_speed_mps{0.0F};
    float min_clearance_m{0.0F};
    float time_to_collision_s{0.0F};
    bool has_valid_query{false};

    std::vector<Vec3> blocking_cell_centers;
};

}  // namespace dedalus
