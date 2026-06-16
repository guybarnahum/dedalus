#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include "dedalus/core/types.hpp"
#include "dedalus/occupancy/occupancy_types.hpp"

namespace dedalus {

struct WorldSnapshot;
struct MissionLocalObstacleMapSnapshot;

struct LocalFlightMapConfig {
    float cell_size_m{0.5F};

    float forward_range_m{30.0F};
    float rear_range_m{6.0F};
    float lateral_range_m{15.0F};

    float occupied_score_hit{1.0F};
    float thin_risk_score_hit{1.0F};
    float free_score_hit{0.25F};
    float decay_half_life_s{1.0F};

    float occupied_threshold{0.65F};
    float risk_threshold{0.65F};

    float vehicle_radius_m{0.45F};
    float safety_margin_m{1.0F};
    float max_evidence_range_m{35.0F};

    std::size_t max_evidence_per_update{512U};
};

struct LocalFlightMapCell {
    Vec3 center_local;

    float occupied_score{0.0F};
    float free_score{0.0F};
    float risk_score{0.0F};

    float nearest_range_m{std::numeric_limits<float>::infinity()};
    float min_z_m{0.0F};
    float max_z_m{0.0F};

    std::uint64_t last_observed_ns{0U};

    bool occupied{false};
    bool inflated_blocked{false};
    bool recently_observed{false};
};

struct LocalFlightMapSnapshot {
    TimePoint timestamp;
    FrameId source_frame_id;
    bool has_source_frame{false};

    float cell_size_m{0.0F};
    int x_cells{0};
    int y_cells{0};

    float forward_range_m{0.0F};
    float rear_range_m{0.0F};
    float lateral_range_m{0.0F};

    std::vector<LocalFlightMapCell> cells;

    std::size_t occupied_count{0U};
    std::size_t inflated_blocked_count{0U};
    float nearest_obstacle_m{std::numeric_limits<float>::infinity()};

    // Diagnostics for the mission-local -> ego-local exclusion-zone derivation.
    // These counters are observational only; they do not imply command blocking.
    std::size_t source_mission_cell_count{0U};
    std::size_t projected_mission_cell_count{0U};
    std::size_t projected_local_cell_update_count{0U};
    float exclusion_inflation_radius_m{0.0F};
};

struct LocalFlightMapIndex {
    int ix{0};
    int iy{0};
};

class LocalFlightMapAccumulator {
public:
    explicit LocalFlightMapAccumulator(LocalFlightMapConfig config = {});

    const LocalFlightMapConfig& config() const noexcept { return config_; }

    LocalFlightMapSnapshot update(const WorldSnapshot& snapshot);

    // Derive the current ego-local flight map from a stable mission-local map.
    //
    // map_T_body is the current ego pose in the mission-local map frame.
    // This method resets the ego-local grid each tick because the local map is
    // a crop/view of accumulated mission-local evidence, not the storage layer.
    LocalFlightMapSnapshot update_from_mission_local_map(
        const MissionLocalObstacleMapSnapshot& mission_map,
        const Pose3& map_T_body,
        TimePoint timestamp);

    const LocalFlightMapSnapshot& latest() const noexcept { return latest_; }

    std::optional<LocalFlightMapIndex> local_to_grid(const Vec3& local) const;

    const LocalFlightMapCell* cell_at(int ix, int iy) const;
    LocalFlightMapCell* mutable_cell_at(int ix, int iy);

private:
    void reset_cells();
    void decay_scores(TimePoint timestamp);
    void ingest_obstacle_evidence(const WorldSnapshot& snapshot);
    void ingest_single_evidence(const ObstacleEvidence& evidence, TimePoint snapshot_time);
    void splat_occupied_evidence(const ObstacleEvidence& evidence, TimePoint snapshot_time);
    void splat_free_evidence(const ObstacleEvidence& evidence, TimePoint snapshot_time);
    void classify_cells();
    void inflate_blocked_cells();
    void update_summary();

    int flat_index(int ix, int iy) const noexcept;
    Vec3 cell_center_local(int ix, int iy) const noexcept;
    float evidence_footprint_radius_m(const ObstacleEvidence& evidence) const noexcept;
    int footprint_radius_cells(float radius_m) const noexcept;
    bool evidence_is_usable(const ObstacleEvidence& evidence) const noexcept;

    LocalFlightMapConfig config_;
    LocalFlightMapSnapshot latest_;
    std::uint64_t last_update_ns_{0U};
    bool has_last_update_{false};
};

}  // namespace dedalus
