#pragma once

#include <optional>
#include <string>
#include <vector>

#include "dedalus/core/types.hpp"
#include "dedalus/occupancy/occupancy_types.hpp"
#include "dedalus/avoidance/local_flight_map.hpp"
#include "dedalus/avoidance/mission_local_obstacle_map.hpp"
#include "dedalus/avoidance/trajectory_safety_evaluator.hpp"

namespace dedalus {

struct GeoAnchor {
    double latitude_deg{0.0};
    double longitude_deg{0.0};
    double altitude_m{0.0};
};

struct AppearanceCondition {
    TimePoint timestamp;
    LightingMode lighting_mode{LightingMode::Unknown};
    WeatherMode weather_mode{WeatherMode::Unknown};
    SeasonMode season_mode{SeasonMode::Unknown};
    SensorMode sensor_mode{SensorMode::Unknown};
    float confidence{0.0F};
};

enum class EgoFlightStatus {
    Unknown,
    Landed,
    Airborne,
};

enum class FlightControlArmState {
    Unknown,
    Disarmed,
    ArmRequested,
    ArmedConfirmed,
    DisarmRequested,
    DisarmedConfirmed,
    ArmFailed,
    DisarmFailed,
};

struct FlightControlState {
    FlightControlArmState arm_state{FlightControlArmState::Unknown};
    TimePoint updated_at;
    TimePoint last_arm_request_at;
    TimePoint last_disarm_request_at;
    std::string status;
};

struct EgoState {
    TimePoint timestamp;
    Pose3 local_T_body;
    Vec3 velocity_local;
    Vec3 angular_velocity_body;
    Covariance6 covariance{};
    MapFrameId map_frame_id{"map_unknown"};

    double height_m{0.0};
    bool height_valid{false};
    bool armed{false};
    bool armed_valid{false};
    EgoFlightStatus flight_status{EgoFlightStatus::Unknown};
    float confidence{0.0F};

    std::optional<Pose3> home_T_body;
    std::optional<TimePoint> home_timestamp;
};

struct AgentViewEvidence {
    FrameId source_frame_id;
    bool has_source_frame{false};
    DetectionId source_detection_id;
    bool has_source_detection{false};
    Rect2 source_bbox_px;
    bool has_source_bbox{false};
    Vec2 source_center_px;
    bool has_source_center{false};
    TimePoint timestamp;
    std::string camera_name;
};

struct AgentState {
    AgentId agent_id;
    IdentityId identity_id;
    TrackId source_track_id;
    DetectionId source_detection_id;
    bool has_source_detection{false};
    Rect2 source_bbox_px;
    bool has_source_bbox{false};
    FrameId source_frame_id;
    bool has_source_frame{false};
    AgentViewEvidence latest_view_evidence;
    bool has_latest_view_evidence{false};
    TimePoint last_seen;
    Vec3 position_local;
    Vec3 velocity_local;
    MapFrameId map_frame_id;
    Covariance6 covariance{};
    ClassLabel class_label{ClassLabel::Unknown};
    FactionLabel faction{FactionLabel::Unknown};
    AgentLifecycle lifecycle{AgentLifecycle::New};
    float confidence{0.0F};
};

struct ContainerState {
    AgentId container_id;
    ContainerType type{ContainerType::Unknown};
    Pose3 local_T_container;
    Vec3 velocity_local;
    MapFrameId map_frame_id;
    float capacity_estimate{0.0F};
    float confidence{0.0F};
};

struct ContainmentEvent {
    EventId event_id;
    TimePoint timestamp;
    ContainmentEventType type{ContainmentEventType::PossibleEnter};
    AgentId subject_agent_id;
    AgentId container_agent_id;
    float confidence{0.0F};
    std::vector<std::string> evidence;
};

struct ExclusionZone {
    ZoneId zone_id;
    TimePoint timestamp;
    ZoneType type{ZoneType::Cone};
    Pose3 local_T_zone;
    Vec3 dimensions;
    MapFrameId map_frame_id;
    float confidence{0.0F};
    float inflation_radius_m{0.0F};
    TimePoint expires_at;
    std::string reason;
};

struct MapFrame {
    MapFrameId map_frame_id;
    std::optional<GeoAnchor> geo_anchor;
    Pose3 local_T_anchor;
    float scale_confidence{0.0F};
    float orientation_confidence{0.0F};
    TimePoint created_at;
    TimePoint last_used;
};

struct StaticStructure {
    StructureId structure_id;
    std::string type;
    Bounds3 bounds;
    MapFrameId map_frame_id;
    FeatureVector feature_signature;
    float confidence{0.0F};
    TimePoint first_seen;
    TimePoint last_confirmed;
};

struct FlightCorridor {
    CorridorId corridor_id;
    std::vector<Vec3> centerline;
    MapFrameId map_frame_id;
    float radius_m{0.0F};
    float min_altitude_m{0.0F};
    float max_altitude_m{0.0F};
    float confidence{0.0F};
};

struct Landmark {
    LandmarkId landmark_id;
    std::string type;
    Vec3 position_local;
    MapFrameId map_frame_id;
    FeatureVector feature_signature;
    float confidence{0.0F};
};

struct UncertainRegion {
    std::string region_id;
    Bounds3 bounds;
    MapFrameId map_frame_id;
    float uncertainty{1.0F};
    std::string reason;
};

// A ranked landing-zone candidate produced by PerchCandidateEvaluator.
struct PerchCandidate {
    Vec3  position_local;       // center_local of the surface patch (map frame)
    Vec3  normal_local;         // surface normal (should face toward world-up)
    float score{0.0F};          // composite: flatness × area × confidence, [0, 1]
};

struct WorldSnapshot {
    TimePoint timestamp;
    EgoState ego;
    FlightControlState flight_control;
    MapFrameId active_map_frame_id{"map_unknown"};
    std::string depth_source_name;        // provider_name() of active depth slot A; empty = inactive
    // Config-layer names for each active (slot A) pipeline provider.
    // Empty = not set (e.g. pipeline built without full ProviderRegistry).
    std::string ego_provider_name;
    std::string detector_name;
    std::string camera_stabilizer_name;
    std::string tracker_name;
    std::string identity_resolver_name;
    std::string projector_name;
    AppearanceCondition appearance_condition;

    std::vector<AgentState> agents;
    std::vector<ContainerState> containers;
    std::vector<ContainmentEvent> containment_events;
    std::vector<ExclusionZone> tactical_exclusion_zones;
    std::vector<FlightCorridor> flight_corridors;
    std::vector<StaticStructure> static_structures;
    std::vector<Landmark> landmarks;
    std::vector<UncertainRegion> uncertain_regions;
    std::vector<MapFrame> map_frames;

    bool has_ego_occupancy{false};
    EgoOccupancyMapSnapshot ego_occupancy;
    bool has_latest_swept_volume{false};
    SweptVolumeDebug latest_swept_volume;

    bool has_mission_local_obstacle_map{false};
    MissionLocalObstacleMapSnapshot mission_local_obstacle_map;

    bool has_local_flight_map{false};
    LocalFlightMapSnapshot local_flight_map;

    bool has_trajectory_safety{false};
    TrajectorySafetyResult trajectory_safety;

    std::vector<ObstacleSensingVolume> obstacle_sensing_volumes;
    std::vector<ObstacleEvidence> obstacle_evidence;
    std::vector<PerchCandidate> perch_candidates;
};

std::string to_json(const WorldSnapshot& snapshot);

}  // namespace dedalus
