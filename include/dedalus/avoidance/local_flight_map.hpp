#pragma once

#include <cmath>
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

// One azimuth bucket of the L0 polar risk map.
// Sectors span 360° starting from body-forward (+X), increasing clockwise (body right = +Y).
// sector_index 0 → [−180°, −180° + sector_width_deg), increasing toward +180°.
// The canonical layout uses 36 sectors of 10° each (index 0 = centre-forward −5°…+5°).
struct L0PolarRiskSector {
    // Azimuth of the sector centre in body frame (degrees, −180…+180).
    // 0° = forward (+X), 90° = right (+Y), ±180° = aft.
    float azimuth_deg{0.0F};

    // Worst-case closing speed in this sector (m/s, positive = approaching ego).
    // 0 when no occupied cell is converging (receding or no obstacle).
    float max_closing_speed_mps{0.0F};

    // Time-to-collision for the most dangerous occupied cell (s).
    // +∞ when no converging obstacle is present.
    float min_ttc_s{std::numeric_limits<float>::infinity()};

    // Nearest occupied cell range in this sector (m).
    float nearest_range_m{std::numeric_limits<float>::infinity()};

    // True if at least one occupied cell falls in this sector.
    bool has_obstacle{false};
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

    // ── L0 polar risk map ───────────────────────────────────────────────────────
    // Populated by compute_l0_polar_risk() after each map update.
    // Consumers (flight response subscribers) read this without re-deriving it.

    // Ego body-frame speed at the time the risk map was computed (m/s).
    float ego_speed_mps{0.0F};

    // Per-sector risk, ordered by sector_index 0…N−1.
    // Populated only when ego_speed_mps is known; empty until first call.
    std::vector<L0PolarRiskSector> polar_risk_sectors;

    // Worst TTC across all sectors (convenience accessor for flight response).
    float global_min_ttc_s{std::numeric_limits<float>::infinity()};

    // Closing-speed-weighted escape direction in body frame (unit vector).
    // (0,0,0) when no converging obstacles exist.
    Vec3 escape_direction_body;
};

struct LocalFlightMapIndex {
    int ix{0};
    int iy{0};
};

// Compute per-sector closing-speed risk and populate LocalFlightMapSnapshot::polar_risk_sectors.
//
// velocity_body_mps: ego velocity in body frame (x = forward, y = right, z = down NED).
//   Pass {0,0,0} when hovering — risk collapses to proximity weighting only.
// num_sectors: number of azimuth buckets spanning 360°. Default 36 (10° each).
//
// This function is a pure computation on the already-built cell list; it does not
// mutate the cell scores.  Call it after each update_from_mission_local_map().
void compute_l0_polar_risk(LocalFlightMapSnapshot& snap,
                           Vec3 velocity_body_mps,
                           int num_sectors = 36);

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
