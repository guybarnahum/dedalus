#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace dedalus {

using Nanoseconds = std::int64_t;

struct TimePoint {
    Nanoseconds timestamp_ns{0};
};

struct FrameId {
    std::string value;
};

struct CameraId {
    std::string value;
};

struct DetectionId {
    std::string value;
};

struct TrackId {
    std::string value;
};

struct AgentId {
    std::string value;
};

struct IdentityId {
    std::string value;
};

struct ZoneId {
    std::string value;
};

struct EventId {
    std::string value;
};

struct MapFrameId {
    std::string value;
};

struct StructureId {
    std::string value;
};

struct CorridorId {
    std::string value;
};

struct LandmarkId {
    std::string value;
};

struct Vec2 {
    double x{0.0};
    double y{0.0};
};

struct Vec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

struct Rect2 {
    double x{0.0};
    double y{0.0};
    double width{0.0};
    double height{0.0};
};

struct Pose3 {
    Vec3 position;
    Vec3 rotation_rpy;
};

struct Bounds3 {
    Vec3 min;
    Vec3 max;
};

using Covariance3 = std::array<double, 9>;
using Covariance6 = std::array<double, 36>;
using FeatureVector = std::vector<float>;

enum class ClassLabel {
    Unknown,
    Person,
    Drone,
    Car,
    Boat,
    House,
    Building,
    Tree,
    Road,
    River,
    Terrain
};

enum class FactionLabel {
    Unknown,
    Friendly,
    Neutral,
    Hostile
};

enum class AgentLifecycle {
    New,
    Active,
    Occluded,
    Contained,
    Stale,
    Retired
};

enum class ZoneType {
    Cone,
    Cylinder,
    Box,
    VoxelCluster
};

enum class ContainerType {
    Unknown,
    Car,
    Boat,
    House,
    Building,
    Garage,
    Room
};

enum class ContainmentEventType {
    Enter,
    Exit,
    PossibleEnter,
    PossibleExit,
    Transfer
};

enum class LightingMode {
    Unknown,
    Day,
    Night,
    Dawn,
    Dusk,
    Artificial
};

enum class WeatherMode {
    Unknown,
    Clear,
    Rain,
    Fog,
    Dust,
    Snow
};

enum class SeasonMode {
    Unknown,
    Spring,
    Summer,
    Fall,
    Winter
};

enum class SensorMode {
    Unknown,
    Rgb,
    Mono,
    Infrared,
    Thermal
};

}  // namespace dedalus
