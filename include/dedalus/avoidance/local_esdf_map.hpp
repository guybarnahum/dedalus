#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <unordered_map>
#include <vector>

#include "dedalus/core/types.hpp"

namespace dedalus {

class MissionLocalPlanningMap;

// ─── L3: Local ESDF Map ───────────────────────────────────────────────────────
//
// Sparse Euclidean Signed Distance Field derived from a window of L2.
// Only "shell" cells (|d| < d0_m) are stored — interior occupied cells use
// a clamped value of −0.5 m and exterior cells beyond d0 are omitted.
//
// Lifecycle:
//   1. Call slide_window() on the L2 map so the relevant region is in memory.
//   2. Call compute_esdf(l2, centre, half, d0) to get a LocalESDFMap.
//   3. Use query() / repulsion() / is_clear() for planning queries.
//   Recompute when the drone moves > half the window half-extent.
//
// compute_esdf uses a 3-phase separable EDT (Felzenszwalb & Huttenlocher,
// TPAMI 2012), O(N) in the number of voxels, with anisotropic cell sizes
// inherited from the L2 configuration.

struct LocalESDFConfig {
    double cell_size_m{1.0};           // XY voxel size (copied from L2)
    double vertical_cell_size_m{2.0};  // Z voxel size  (copied from L2)
    double d0_m{5.0};                  // truncation radius
};

struct LocalESDFCell {
    Vec3  centre;         // world-frame centre
    float d{0.0F};        // signed distance (< 0 inside occupied, clamped at −0.5 m)
    Vec3  grad;           // unit gradient ∇d — exact EDT central-difference (points away from nearest obstacle)
    Vec3  sgrad;          // smoothed gradient — 6-connected neighbour average of grad, renormalized.
                          // More normal-to-surface than grad; use for APF direction to avoid
                          // discontinuities at edges and corners.
};

struct LocalESDFQueryResult {
    float d{0.0F};        // signed distance at query pos; d0 if outside window
    Vec3  grad;           // unit gradient; zero if outside window
};

// Snapshot of the ESDF map for SSE streaming (Stage 6).
struct LocalESDFMapSnapshot {
    LocalESDFConfig config;
    std::size_t cell_count{0U};
    std::vector<LocalESDFCell> cells;  // all stored shell cells
    Vec3  net_repulsion;               // APF repulsion force at a query point (world frame)
    std::uint64_t seq{0U};
    bool  is_delta{false};
};

class LocalESDFMap {
public:
    explicit LocalESDFMap(LocalESDFConfig cfg = {}) : config_(cfg) {}

    const LocalESDFConfig& config()     const noexcept { return config_; }
    std::size_t            cell_count() const noexcept { return cells_.size(); }

    // Signed distance + gradient at world-frame pos.
    // Returns {d0, zero-grad} when pos falls outside the stored window.
    [[nodiscard]] LocalESDFQueryResult query(const Vec3& pos) const noexcept;

    // APF repulsion force at pos, active when 0 < d(pos) < d0.
    // F = k * (1/d − 1/d0) / d² · ∇d,  zero otherwise.
    // Uses cell.sgrad (1-hop smoothed gradient) for direction.
    // Suitable for quasi-static safety queries and hover mode.
    [[nodiscard]] Vec3 repulsion(const Vec3& pos, double d0, double k) const noexcept;

    // Velocity-aware APF repulsion.  Identical APF formula to repulsion(), but the
    // gradient direction is a Gaussian-weighted spatial average over a neighbourhood
    // of radius R(v) = clamp(|v|²/(2·a_max), R_min, d0).
    //
    // Rationale: at speed v the drone's stopping distance is v²/(2·a_max).  The
    // planner cannot react to field features smaller than that distance, so averaging
    // over that radius prevents chattering and gives smooth, globally-coherent
    // deflections around obstacle surfaces.  The stored distance d remains exact
    // (from EDT); only the repulsion direction is spatially smoothed.
    //
    //   vel        — current velocity in world frame (m/s).
    //   d0         — truncation radius (m); cells beyond this contribute nothing.
    //   k          — APF gain.
    //   a_max_mps2 — assumed maximum deceleration (m/s²); typical drone: 3.0.
    //
    // At |v|≈0 the kernel collapses to a single cell (= repulsion() with sgrad).
    // At high speed the kernel widens up to d0, trading local sharpness for global
    // smoothness — ideal for fast-transit trajectory optimisation.
    [[nodiscard]] Vec3 repulsion_smoothed(const Vec3& pos,
                                          const Vec3& vel,
                                          double d0,
                                          double k,
                                          double a_max_mps2) const noexcept;

    // True if the sphere of radius r centred at pos is entirely in free space
    // (d(pos) ≥ r).
    [[nodiscard]] bool is_clear(const Vec3& pos, double r) const noexcept;

    // ── Incremental updates (Stage 4) ───────────────────────────────────────
    //
    // update_incremental: after a set of L2 cells change, recompute only the
    // ESDF sub-window that covers those cells + d0_m margin.  Cells outside
    // the sub-window are unchanged.  Equivalent to a full recompute within
    // the affected region (< 0.01 m error); typically < 1 ms for 50 dirty cells.
    //
    // dirty_world_positions: world-frame centres of L2 cells that changed.
    // d0_m: truncation radius for this update (use the same value as the
    //        original compute_esdf call).
    void update_incremental(const MissionLocalPlanningMap& l2,
                            const std::vector<Vec3>& dirty_world_positions,
                            double d0_m);

    // Snapshot all shell cells + compute net repulsion at query_pos.
    // Repulsion scale k: APF gain (default 1.0).
    // is_delta=false on a full recompute; the caller sets it.
    [[nodiscard]] LocalESDFMapSnapshot snapshot(
        const Vec3& query_pos, double repulsion_k = 1.0) const;

    // update_tube: compute or refresh the ESDF along a proposed trajectory.
    // Only the bounding volume of the waypoints, expanded by tube_radius_m + d0,
    // is (re-)computed — cells outside this volume are unchanged.  Intended for
    // JIT path validation by the trajectory optimizer (Stage 7).
    // Uses config_.d0_m as the truncation radius.
    void update_tube(const MissionLocalPlanningMap& l2,
                     const std::vector<Vec3>& waypoints,
                     double tube_radius_m);

    // ── Persistence ────────────────────────────────────────────────────────────
    //
    // save(): write all shell cells to a binary file (magic 'ESDF', version 1).
    //         Returns false on I/O error; does not throw.
    // load(): restore cells from a previously saved file.  Replaces all current
    //         cells.  Returns false if the file is missing, corrupt, or versioned
    //         differently.  On failure the map is left empty.
    [[nodiscard]] bool save(const std::filesystem::path& path) const;
    [[nodiscard]] bool load(const std::filesystem::path& path);

    // compute_esdf populates cells_ directly.
    friend LocalESDFMap compute_esdf(const MissionLocalPlanningMap& l2,
                                     const Vec3& centre_world,
                                     double horiz_half_m,
                                     double vert_half_m,
                                     double d0_m);

private:
    struct CellKey {
        int x{0}, y{0}, z{0};
        bool operator==(const CellKey& o) const noexcept {
            return x == o.x && y == o.y && z == o.z;
        }
    };
    struct CellKeyHash {
        std::size_t operator()(const CellKey& k) const noexcept;
    };

    CellKey key_for_point(const Vec3& pos) const noexcept;

    LocalESDFConfig config_;
    std::unordered_map<CellKey, LocalESDFCell, CellKeyHash> cells_;
};

// ─── Free function ─────────────────────────────────────────────────────────
//
// Compute an ESDF from the in-memory L2 window centred on centre_world.
//   horiz_half_m : half-extent in X and Y (world metres).
//   vert_half_m  : half-extent in Z  (world metres).
//                  Separate parameter because L2's vertical cell size (2 m) is
//                  coarser than XY (1 m); a 80×80×20 m window needs horiz=40, vert=10.
//   d0_m         : truncation radius — only cells with |d| < d0 are stored.
//
// l2.slide_window() should be called before this to ensure the region is loaded.
// Cell sizes are inherited from l2.config() — no separate config needed.
[[nodiscard]] LocalESDFMap compute_esdf(const MissionLocalPlanningMap& l2,
                                         const Vec3& centre_world,
                                         double horiz_half_m,
                                         double vert_half_m,
                                         double d0_m);

}  // namespace dedalus
