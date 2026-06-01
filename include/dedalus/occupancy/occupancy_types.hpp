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
    VisualObstacleDetector,
    DepthProvider,
    Fused,
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

}  // namespace dedalus
