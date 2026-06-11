#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "dedalus/core/types.hpp"
#include "dedalus/occupancy/occupancy_types.hpp"

namespace dedalus {

// MissionLocalObstacleMap is the stable, takeoff/mission-relative accumulation
// layer for classless obstacle evidence.
//
// This class intentionally stores evidence in the same mission-local/map frame
// used by ObstacleEvidence::center_local and EgoState::local_T_body. It does
// not derive an ego-local crop; that is the responsibility of the local flight
// map slice.
struct MissionLocalObstacleMapConfig {
    double cell_size_m{0.5};
    double vertical_cell_size_m{0.5};

    double occupied_hit_score{1.0};
    double free_hit_score{1.0};

    double occupied_threshold{1.0};
    double free_threshold{1.0};

    double score_decay_per_second{0.0};
    double max_score{20.0};

    std::size_t max_evidence_per_update{4096U};
};

struct MissionLocalObstacleCell {
    Vec3 center_map;
    Vec3 size_m;

    double occupied_score{0.0};
    double free_score{0.0};
    double risk_score{0.0};

    double confidence{0.0};

    std::uint64_t first_observed_timestamp_ns{0U};
    std::uint64_t last_observed_timestamp_ns{0U};

    float min_z_m{0.0F};
    float max_z_m{0.0F};

    bool observed{false};
    bool occupied{false};
    bool free{false};

    OccupancySourceKind last_source_kind{};
    std::string last_source_provider;
};

struct MissionLocalObstacleMapSummary {
    MapFrameId map_frame_id;

    std::size_t observed_cell_count{0U};
    std::size_t occupied_cell_count{0U};
    std::size_t free_cell_count{0U};

    std::uint64_t update_count{0U};
    std::uint64_t last_update_timestamp_ns{0U};
};

struct MissionLocalObstacleMapSnapshot {
    MissionLocalObstacleMapConfig config;
    MissionLocalObstacleMapSummary summary;
    std::vector<MissionLocalObstacleCell> cells;
};

class MissionLocalObstacleMap {
public:
    explicit MissionLocalObstacleMap(MissionLocalObstacleMapConfig config = {});

    const MissionLocalObstacleMapConfig& config() const;
    MissionLocalObstacleMapSnapshot snapshot(std::size_t max_cells = 0U) const;

    MissionLocalObstacleMapSnapshot update(
        const std::vector<ObstacleEvidence>& evidence,
        TimePoint now,
        const MapFrameId& fallback_map_frame_id = {});

    void reset();

private:
    struct CellKey {
        int x{0};
        int y{0};
        int z{0};

        bool operator==(const CellKey& other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct CellKeyHash {
        std::size_t operator()(const CellKey& key) const;
    };

    CellKey key_for_point(const Vec3& point) const;
    Vec3 center_for_key(const CellKey& key) const;

    void decay_to(const TimePoint& now);
    void refresh_summary();

    MissionLocalObstacleMapConfig config_;
    MissionLocalObstacleMapSummary summary_;

    struct StoredCell {
        CellKey key;
        MissionLocalObstacleCell cell;
    };

    std::vector<StoredCell> cells_;
    bool has_last_update_{false};
    TimePoint last_update_{};
};

}  // namespace dedalus
