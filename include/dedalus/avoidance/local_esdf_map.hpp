#pragma once

#include <cstddef>
#include <functional>
#include <unordered_map>

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
    Vec3  grad;           // unit gradient ∇d (points away from nearest obstacle)
};

struct LocalESDFQueryResult {
    float d{0.0F};        // signed distance at query pos; d0 if outside window
    Vec3  grad;           // unit gradient; zero if outside window
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
    [[nodiscard]] Vec3 repulsion(const Vec3& pos, double d0, double k) const noexcept;

    // True if the sphere of radius r centred at pos is entirely in free space
    // (d(pos) ≥ r).
    [[nodiscard]] bool is_clear(const Vec3& pos, double r) const noexcept;

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
