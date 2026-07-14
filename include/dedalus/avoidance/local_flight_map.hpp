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

// Per raw-evidence sensor observation, preserved before L1 aggregation.
// Populated by collect_l0_sensor_observations() from the WorldSnapshot evidence list.
// These carry the original sensor-frame angles and source tag — they are NOT
// derived from the L1→L0 cell projection.
struct L0SensorObservation {
    // Body-frame spherical angles at observation time.
    // az: atan2(right, fwd) → −π…+π; el: atan2(up, sqrt(fwd²+right²)) → −π/2…+π/2.
    float az_body_rad{0.0F};
    float el_body_rad{0.0F};

    float range_m{std::numeric_limits<float>::infinity()};
    float closing_speed_mps{0.0F};  // radial closing speed (m/s, positive = converging)
    float ttc_s{std::numeric_limits<float>::infinity()};

    OccupancySourceKind source_kind{OccupancySourceKind::DepthProvider};
};

// One cell of the 2-D azimuth × elevation spherical risk grid.
// Populated by compute_l0_polar_risk(); consumers read without re-deriving.
struct L0SphericalRiskBin {
    float az_centre_deg{0.0F};  // bin centre azimuth, body frame (deg)
    float el_centre_deg{0.0F};  // bin centre elevation, body frame (deg)

    float min_ttc_s{std::numeric_limits<float>::infinity()};
    float max_closing_speed_mps{0.0F};
    float nearest_range_m{std::numeric_limits<float>::infinity()};

    bool has_obstacle{false};

    // Bitmask of sensor sources contributing to this bin.
    // Bit 0 = AirSimGroundTruth/AirSimGroundTruthVisualEmulation
    // Bit 1 = DepthProvider
    // Bit 2 = VisualObstacleDetector
    // Bit 3 = other / unknown
    std::uint8_t source_mask{0U};
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

    // ── Spherical sensor view ───────────────────────────────────────────────────
    // Per-evidence observations from raw sensor evidence (before L1 merging).
    // Populated by collect_l0_sensor_observations(); empty when not called.
    std::vector<L0SensorObservation> sensor_observations;

    // 2-D az×el risk grid (num_az_bins × num_el_bins cells).
    // Layout: row-major, az index varies fastest.
    // Populated by compute_l0_polar_risk() alongside polar_risk_sectors.
    std::vector<L0SphericalRiskBin> spherical_risk_bins;
    int spherical_num_az{0};  // number of azimuth bins used
    int spherical_num_el{0};  // number of elevation bins used

    // Authoritative sensor scope metadata — emitted by the depth-slot detector,
    // stamped by the runner after the depth-detect loop, and forwarded in the SSE
    // stream so the viewer cone scope panel requires no estimation.
    float sensor_az_half_rad{0.0F};  // half-width of the sensor's horizontal FOV (rad)
    float sensor_el_half_rad{0.0F};  // half-height of the sensor's vertical FOV (rad)
    int   sensor_grid_cols{0};       // depth grid column count (az sampling resolution)
    int   sensor_grid_rows{0};       // depth grid row count   (el sampling resolution)
};

struct LocalFlightMapIndex {
    int ix{0};
    int iy{0};
};

// Compute per-sector and az×el-bin risk from the L0 cell list.
//
// Populates: polar_risk_sectors (1-D az), spherical_risk_bins (2-D az×el),
//            global_min_ttc_s, escape_direction_body, ego_speed_mps.
//
// velocity_body_mps: ego velocity in body frame (x=fwd, y=right, z=down NED).
// num_az: azimuth bins spanning 360°. Default 36 (10° each).
// num_el: elevation bins spanning ±(num_el/2 * el_step)°. Default 9 (10° each → ±45°).
void compute_l0_polar_risk(LocalFlightMapSnapshot& snap,
                           Vec3 velocity_body_mps,
                           int num_az = 36,
                           int num_el = 9);

// Populate LocalFlightMapSnapshot::sensor_observations from raw obstacle evidence.
//
// Call this after compute_l0_polar_risk(), passing the WorldSnapshot evidence list.
// Uses each occupied evidence's bearing_rad / elevation_rad / range_m / source_kind
// to build source-tagged TTC observations without re-projecting through the L1 map.
//
// max_obs: cap on number of observations stored (default 256).
void collect_l0_sensor_observations(LocalFlightMapSnapshot& snap,
                                    const std::vector<ObstacleEvidence>& evidence,
                                    Vec3 velocity_body_mps,
                                    std::size_t max_obs = 256U);

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
