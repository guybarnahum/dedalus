#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

#include "dedalus/avoidance/mission_local_traversability_map.hpp"
#include "dedalus/core/types.hpp"

namespace dedalus {

// ─── Level 2: compressed planning map ────────────────────────────────────────
//
// The planning map is the second level of the two-level obstacle-map
// architecture:
//
//   Level 1 (MissionLocalTraversabilityMap)
//     • Raw-accumulator: full 0.5 m voxels, with score decay and pruning.
//     • All observed states (Occupied / ObservedFree / Unknown / Stale).
//     • Source of truth for evidence persistence.
//
//   Level 2 (MissionLocalPlanningMap — this class)
//     • Compressed projection of Level 1 occupied cells into a coarser grid.
//     • Retains only cells above a minimum occupied-score threshold.
//     • Much smaller cell count → faster path-planning queries.
//     • Rebuilt from scratch each time Level 1 is updated.
//
// At the default 1.0 m XY / 2.0 m Z resolution, each planning cell covers
// 16× the volume of a Level 1 cell (1.0 × 1.0 × 2.0 m³ vs 0.5³ m³).

struct MissionLocalPlanningMapConfig {
    // Grid resolution — should be coarser than Level 1's cell_size_m.
    double cell_size_m{1.0};
    double vertical_cell_size_m{2.0};

    // Minimum Level 1 occupied_score required to project a cell into Level 2.
    // Should be >= MissionLocalTraversabilityMapConfig::occupied_threshold (1.0).
    double min_occupied_score{1.0};
};

struct MissionLocalPlanningCell {
    Vec3 center_map;                  // world-frame centre of this planning voxel
    float max_occupied_score{0.0F};   // max occupied_score across all contributing L1 cells
    float confidence{0.0F};           // max confidence across all contributing L1 cells
    std::uint32_t source_cell_count{0U};  // number of L1 cells that contributed
};

struct MissionLocalPlanningMapSnapshot {
    MissionLocalPlanningMapConfig config;

    std::size_t l1_input_cell_count{0U};    // total L1 cells considered
    std::size_t l1_occupied_cell_count{0U}; // L1 cells that passed min_occupied_score
    std::size_t cell_count{0U};             // planning voxels produced

    std::vector<MissionLocalPlanningCell> cells;
};

// ─── class ───────────────────────────────────────────────────────────────────

class MissionLocalPlanningMap {
public:
    explicit MissionLocalPlanningMap(MissionLocalPlanningMapConfig config = {});

    const MissionLocalPlanningMapConfig& config() const noexcept { return config_; }
    std::size_t cell_count() const noexcept { return snapshot_.cell_count; }

    // Rebuild the planning map from a Level 1 traversability snapshot.
    // All prior planning cells are discarded; the map is rebuilt from scratch.
    void update_from_traversability(const MissionLocalTraversabilityMapSnapshot& source);

    const MissionLocalPlanningMapSnapshot& snapshot() const noexcept { return snapshot_; }

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

    CellKey key_for_point(const Vec3& point) const noexcept;
    Vec3 center_for_key(const CellKey& key) const noexcept;

    MissionLocalPlanningMapConfig config_;
    MissionLocalPlanningMapSnapshot snapshot_;
};

}  // namespace dedalus
