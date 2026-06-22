#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include "dedalus/avoidance/mission_local_obstacle_map.hpp"
#include "dedalus/core/types.hpp"

namespace dedalus {

enum class TraversabilityCellState {
    Unknown,
    ObservedFree,
    Occupied,
    Mixed,
    Stale,
};

struct MissionLocalTraversabilityMapConfig {
    double cell_size_m{0.5};
    double vertical_cell_size_m{0.5};

    double occupied_threshold{1.0};
    double free_threshold{1.0};
    double max_score{20.0};

    // Decay rates — evidence fades when a region is no longer observed.
    // At 0.05/s an occupied cell decays from score 1.5 → 0.1 in ~28 s without
    // re-observation, giving reasonable persistence for slow-moving platforms.
    double occupied_score_decay_per_second{0.05};
    double free_score_decay_per_second{0.02};
    double confidence_decay_per_second{0.0};

    double stale_after_seconds{300.0};

    // Pruning — evict cells whose occupied_score has decayed below this floor.
    // prune_interval_ticks controls how often the O(N) sweep runs; 0 disables pruning.
    double prune_min_occupied_score{0.1};
    std::uint32_t prune_interval_ticks{10U};

    double required_clearance_m{1.5};
    double soft_clearance_m{3.0};
    double clearance_search_radius_m{6.0};

    double occupied_weight{1.0};
    double proximity_weight{0.7};
    double unknown_weight{0.35};
    double stale_weight{0.25};
    double overhead_weight{0.5};
    double thin_structure_weight{0.4};

    std::uint32_t max_evidence_count{65535U};
};

struct MissionLocalTraversabilityCell {
    Vec3 center_map;
    Vec3 size_m;

    TraversabilityCellState state{TraversabilityCellState::Unknown};

    double occupied_score{0.0};
    double free_score{0.0};
    double confidence{0.0};
    double unknown_score{1.0};

    std::uint32_t occupied_hits_capped{0U};
    std::uint32_t free_rays_capped{0U};
    std::uint32_t conflict_count_capped{0U};
    std::uint32_t refresh_count_capped{0U};

    std::uint64_t first_observed_timestamp_ns{0U};
    std::uint64_t last_observed_timestamp_ns{0U};

    double age_score{0.0};
    double stability_score{0.0};
    double volatility_score{0.0};

    double nearest_obstacle_distance_m{std::numeric_limits<double>::infinity()};
    double clearance_margin_m{std::numeric_limits<double>::infinity()};
    double vertical_clearance_up_m{std::numeric_limits<double>::infinity()};
    double vertical_clearance_down_m{std::numeric_limits<double>::infinity()};

    double occupied_cost{0.0};
    double proximity_cost{0.0};
    double unknown_cost{0.0};
    double stale_cost{0.0};
    double overhead_cost{0.0};
    double thin_structure_cost{0.0};
    double total_traversability_cost{0.0};

    bool stale{false};
};

struct MissionLocalTraversabilityMapSummary {
    MapFrameId map_frame_id;

    std::uint64_t update_count{0U};
    std::uint64_t last_update_timestamp_ns{0U};

    std::size_t source_obstacle_cell_count{0U};
    std::size_t accepted_source_cell_count{0U};
    std::size_t new_cell_count{0U};
    std::size_t updated_cell_count{0U};

    std::size_t cell_count{0U};
    std::size_t occupied_cell_count{0U};
    std::size_t free_cell_count{0U};
    std::size_t mixed_cell_count{0U};
    std::size_t stale_cell_count{0U};
    std::size_t low_clearance_cell_count{0U};
    std::size_t overhead_risk_cell_count{0U};
    std::size_t volatile_cell_count{0U};

    double minimum_clearance_m{std::numeric_limits<double>::infinity()};
    double minimum_vertical_clearance_up_m{std::numeric_limits<double>::infinity()};
};

struct MissionLocalTraversabilityMapSnapshot {
    MissionLocalTraversabilityMapConfig config;
    MissionLocalTraversabilityMapSummary summary;
    std::vector<MissionLocalTraversabilityCell> cells;
    // false → full snapshot (client clears + rebuilds)
    // true  → delta (client merges changed cells only)
    bool is_delta{false};
};

struct TraversabilityQueryResult {
    bool known{false};
    bool occupied{false};
    bool observed_free{false};
    bool stale{false};

    double nearest_obstacle_distance_m{std::numeric_limits<double>::infinity()};
    double clearance_margin_m{std::numeric_limits<double>::infinity()};
    double occupied_fraction{0.0};
    double unknown_fraction{1.0};
    double total_cost{1.0};
};

class MissionLocalTraversabilityMap {
public:
    explicit MissionLocalTraversabilityMap(MissionLocalTraversabilityMapConfig config = {});

    const MissionLocalTraversabilityMapConfig& config() const noexcept { return config_; }
    const MissionLocalTraversabilityMapSummary& summary() const noexcept { return summary_; }

    MissionLocalTraversabilityMapSnapshot update_from_mission_obstacle_map(
        const MissionLocalObstacleMapSnapshot& obstacle_map,
        TimePoint now,
        bool include_clearance = true);

    MissionLocalTraversabilityMapSnapshot snapshot(std::size_t max_cells = 0U) const;

    TraversabilityQueryResult query_sphere(const Vec3& center_map, double radius_m) const;

    void reset();

private:
    struct CellKey {
        int x{0};
        int y{0};
        int z{0};

        bool operator==(const CellKey& other) const noexcept {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct CellKeyHash {
        std::size_t operator()(const CellKey& key) const noexcept;
    };

    struct StoredCell {
        CellKey key;
        MissionLocalTraversabilityCell cell;
    };

    CellKey key_for_point(const Vec3& point) const noexcept;
    Vec3 center_for_key(const CellKey& key) const noexcept;

    MissionLocalTraversabilityCell& ensure_cell(const CellKey& key);
    const MissionLocalTraversabilityCell* cell_at_key(const CellKey& key) const;

    void apply_aging(TimePoint now);
    void recompute_derived_fields(TimePoint now, bool include_clearance = true);
    void refresh_summary();
    // Evict cells whose occupied_score is below prune_min_occupied_score.
    // Rebuilds cell_index_ after compacting the cells_ vector.
    void prune_weak_cells();

    MissionLocalTraversabilityMapConfig config_;
    MissionLocalTraversabilityMapSummary summary_;

    std::vector<StoredCell> cells_;
    std::unordered_map<CellKey, std::size_t, CellKeyHash> cell_index_;

    bool has_last_update_{false};
    TimePoint last_update_{};
};

}  // namespace dedalus
