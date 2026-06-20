#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <unordered_map>
#include <vector>

#include "dedalus/avoidance/mission_local_traversability_map.hpp"
#include "dedalus/core/types.hpp"

namespace dedalus {

// ─── Two-level obstacle map — Level 2: persistent planning map ────────────────
//
// Architecture overview
// ─────────────────────
// Level 1  MissionLocalTraversabilityMap  (accumulator)
//   • 0.5 m voxels, all observed states (Occupied / Free / Unknown / Stale).
//   • Time-based score decay (0.05/s default) — evidence fades when a region
//     leaves the sensor FOV.
//   • Score-floor pruning — cells evicted once score drops below 0.1.
//   • Lifetime: per-flight.  Reset at mission start if desired.
//
// Level 2  MissionLocalPlanningMap  (this class)
//   • Coarser voxels (1 m XY / 2 m Z default) — ~16× fewer cells than L1.
//   • Persistent across missions: saved to disk at landing, loaded at startup.
//   • NO time-based decay.  Cells only leave L2 when free-space evidence
//     explicitly clears them.
//   • Evidence rule (applied per L1 update):
//       L1 Occupied  → max-merge occupied_score into L2 voxel
//       L1 Free      → multiplicative reduction on L2 voxel score;
//                      evict if score drops below min_occupied_score
//       L1 Unknown / absent → no change  (absence ≠ free space)
//
// This separation gives the flight-planning layer a stable, persistent obstacle
// map that survives power cycles, while the accumulator layer stays fresh and
// bounded in memory during flight.

struct MissionLocalPlanningMapConfig {
    // Coarser grid than L1.  At 1.0 m XY / 2.0 m Z vs L1's 0.5 m³, each
    // planning voxel covers 16 L1 sub-voxels.
    double cell_size_m{1.0};
    double vertical_cell_size_m{2.0};

    // Only L1 cells at or above this score are projected into L2.
    // Should be >= MissionLocalTraversabilityMapConfig::occupied_threshold (1.0).
    double min_occupied_score{1.0};

    // How much one L1 free-space observation reduces an L2 voxel's occupied_score.
    // Applied multiplicatively: new_score = old_score * (1 - free_evidence_weight).
    // At 0.5 a cell at score 1.5 clears after one observation; a cell at 15.0
    // (max evidence) takes ~4 free observations to drop below min_occupied_score.
    double free_evidence_weight{0.5};
};

struct MissionLocalPlanningCell {
    Vec3 center_map;                    // world-frame centre of the planning voxel
    float occupied_score{0.0F};         // accumulated occupied evidence (max-merged from L1)
    float confidence{0.0F};             // max confidence across contributing L1 cells
    std::uint32_t source_cell_count{0U}; // cumulative L1 hits
};

struct MissionLocalPlanningMapUpdateStats {
    std::size_t l1_input_cells{0U};     // total L1 cells considered this tick
    std::size_t l1_occupied_merged{0U}; // L1 cells that contributed positive evidence
    std::size_t l1_free_applied{0U};    // L1 free cells that reduced L2 scores
    std::size_t cells_evicted{0U};      // L2 cells evicted due to free-space evidence
};

struct MissionLocalPlanningMapSnapshot {
    MissionLocalPlanningMapConfig config;
    std::size_t cell_count{0U};
    MissionLocalPlanningMapUpdateStats last_update_stats;
    std::vector<MissionLocalPlanningCell> cells;
};

// ─── class ───────────────────────────────────────────────────────────────────

class MissionLocalPlanningMap {
public:
    explicit MissionLocalPlanningMap(MissionLocalPlanningMapConfig config = {});

    const MissionLocalPlanningMapConfig& config() const noexcept { return config_; }
    std::size_t cell_count() const noexcept { return cells_.size(); }
    const MissionLocalPlanningMapUpdateStats& last_update_stats() const noexcept {
        return last_update_stats_;
    }

    // Incrementally update L2 from a Level 1 traversability snapshot.
    //
    //   Occupied L1 cells  → add or strengthen the corresponding L2 voxel.
    //   ObservedFree L1 cells → reduce the L2 voxel score; evict if below floor.
    //   Unknown / Stale / absent  → no change.
    //
    // L2 cells that are neither reinforced nor cleared this tick are untouched.
    void update_from_traversability(const MissionLocalTraversabilityMapSnapshot& source);

    // Materialise a snapshot for publishing / inspection.  O(N) copy.
    MissionLocalPlanningMapSnapshot snapshot() const;

    // Discard all L2 cells.
    void reset();

    // Persistence.  Format: versioned text, one cell per line.
    // Returns true on success.
    bool save_to_file(const std::filesystem::path& path) const;
    bool load_from_file(const std::filesystem::path& path);

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
        MissionLocalPlanningCell cell;
    };

    CellKey key_for_point(const Vec3& point) const noexcept;
    Vec3 center_for_key(const CellKey& key) const noexcept;

    // Evict cells whose score has been driven below min_occupied_score.
    // Rebuilds cell_index_ after compacting cells_.
    void evict_cleared_cells();

    MissionLocalPlanningMapConfig config_;
    MissionLocalPlanningMapUpdateStats last_update_stats_{};

    std::vector<StoredCell> cells_;
    std::unordered_map<CellKey, std::size_t, CellKeyHash> cell_index_;
};

}  // namespace dedalus
