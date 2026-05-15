#pragma once

#include <optional>
#include <string>
#include <vector>

#include "dedalus/core/types.hpp"

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

struct AgentState {
    AgentId agent_id;
    IdentityId identity_id;
    TrackId source_track_id;
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

struct WorldSnapshot {
    TimePoint timestamp;
    EgoState ego;
    MapFrameId active_map_frame_id{"map_unknown"};
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
};

std::string to_json(const WorldSnapshot& snapshot);

}  // namespace dedalus
