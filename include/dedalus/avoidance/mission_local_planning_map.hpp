#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <mutex>
#include <optional>
#include <unordered_map>
#include <unordered_set>
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

    // Sliding-window horizon (Stage 2).  Cells beyond 2×horizon_m from the
    // drone are evicted from memory (they remain in the DB for re-entry).
    // slide_window() is a no-op when no DB is open.
    // Slide is triggered when the drone moves more than horizon_m/4.
    double horizon_m{150.0};
};

struct MissionLocalPlanningCell {
    Vec3 center_map;                    // world-frame centre of the planning voxel
    float occupied_score{0.0F};         // accumulated occupied evidence (max-merged from L1)
    float confidence{0.0F};             // max confidence across contributing L1 cells
    std::uint32_t source_cell_count{0U}; // cumulative L1 hits
    // Wall-clock timestamp (nanoseconds since Unix epoch) of the last live L1
    // update that reinforced this cell.  0 for cells loaded from the DB that
    // have not yet been observed this session.
    // Viewer uses age = (now - last_updated_ns/1e9) to dim stale cells.
    std::int64_t last_updated_ns{0};
};

// Axis-aligned bounding box of all in-memory L2 cells.
// Returned by MissionLocalPlanningMap::extent() — nullopt when the map is empty.
struct MissionLocalPlanningMapExtent {
    Vec3 min;  // world-frame minimum corner (cell centres)
    Vec3 max;  // world-frame maximum corner (cell centres)
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
    // Stage 5: incremental streaming.
    // seq      — monotonically increasing map version at snapshot time.
    // is_delta — true when cells contains only cells changed since a prior seq.
    std::uint64_t seq{0U};
    bool is_delta{false};
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
    // since_seq == 0  → full snapshot (all cells); snap.is_delta = false.
    // since_seq  > 0  → delta snapshot (only cells written after since_seq);
    //                   snap.is_delta = true.  snap.seq is always map_seq_.
    MissionLocalPlanningMapSnapshot snapshot(std::uint64_t since_seq = 0U) const;

    // Current map sequence number (monotonically increasing, one per update call).
    std::uint64_t current_seq() const noexcept { return map_seq_; }

    // Bounding box of all currently in-memory cells (cell centres).
    // Returns nullopt when the map is empty.  O(N).
    [[nodiscard]] std::optional<MissionLocalPlanningMapExtent> extent() const noexcept;

    // Discard all L2 cells.
    void reset();

    // ── Text persistence (legacy / debug) ────────────────────────────────────
    // Returns true on success.
    bool save_to_file(const std::filesystem::path& path) const;
    bool load_from_file(const std::filesystem::path& path);

    // ── SQLite persistence (Stage 1+) ────────────────────────────────────────
    // open_db(): open or create a SQLite DB at path (WAL mode, R-tree schema).
    //   Loads all cells with score >= min_occupied_score into memory.
    //   Returns true on success.
    bool open_db(const std::filesystem::path& path);

    // flush_dirty_to_db(): UPSERT dirty cells + DELETE evicted cells from the
    //   open DB.  Clears both sets.  Safe to call from a background thread.
    //   No-op and returns true if no DB is open.
    bool flush_dirty_to_db();

    // close_db(): final flush_dirty_to_db() then sqlite3_close().
    //   Called at finalize / destructor.  Returns true on success.
    bool close_db();

    // ── Sliding window (Stage 2) ─────────────────────────────────────────────
    // slide_window(): call every tick with the current drone world-frame position.
    //   No-op if no DB is open or the drone has not moved > horizon_m/4.
    //   When triggered:
    //     1. Evicts in-memory cells beyond 2×horizon_m (still in DB for re-entry).
    //     2. Loads cells from DB within drone_pos ± horizon_m not yet in memory.
    void slide_window(const Vec3& drone_pos);

    // ── L2 planning query API (Stage 2.5) ────────────────────────────────────
    // Pure queries — no side effects on L2 state.  Operate on the in-memory
    // window only; call slide_window() first to ensure the relevant region is
    // loaded.

    // ray_cast(): march a ray from origin in direction dir (need not be
    //   normalised) up to max_range_m.  Returns the world-frame centre of the
    //   first occupied L2 cell hit, or nullopt if the ray reaches max_range_m
    //   through free / unmapped space.
    [[nodiscard]] std::optional<Vec3> ray_cast(const Vec3& origin,
                                               const Vec3& dir,
                                               double max_range_m) const noexcept;

    // query_occupied_in_box(): return the world-frame centres of all occupied
    //   L2 cells whose centres fall within the axis-aligned box [bbox.min,
    //   bbox.max].  Result order is unspecified.
    [[nodiscard]] std::vector<Vec3> query_occupied_in_box(
        const Bounds3& bbox) const;

    // query_occupied_ts_in_box(): same as query_occupied_in_box but also
    //   returns the last_updated_ns timestamp for each cell.
    //   Used by compute_esdf to propagate L2 age into L3 arrows.
    [[nodiscard]] std::vector<std::pair<Vec3, std::int64_t>>
        query_occupied_ts_in_box(const Bounds3& bbox) const;

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
        std::uint64_t write_seq{0U};  // map_seq_ at last write (Stage 5)
    };

    CellKey key_for_point(const Vec3& point) const noexcept;
    Vec3 center_for_key(const CellKey& key) const noexcept;

    // Evict cells whose score has been driven below min_occupied_score.
    // Rebuilds cell_index_ after compacting cells_.
    void evict_cleared_cells();

    // Remove cells beyond evict_radius_m from centre from the in-memory store.
    // Does NOT add to evicted_keys_ — evicted cells remain in the DB.
    void evict_far_cells(const Vec3& centre, double evict_radius_m);

    // Load cells from the open DB within centre ± horizon_m that are not yet
    // in cell_index_.  If a cell is also in dirty_cells_, its dirty value takes
    // precedence over the (stale) DB value.
    void load_window_from_db(const Vec3& centre);

    MissionLocalPlanningMapConfig config_;
    MissionLocalPlanningMapUpdateStats last_update_stats_{};

    std::vector<StoredCell> cells_;
    std::unordered_map<CellKey, std::size_t, CellKeyHash> cell_index_;

    // SQLite persistence — opaque sqlite3* stored as void* so sqlite3.h stays
    // out of this header.  Null when no DB is open.
    void* db_{nullptr};

    // Mutex protecting dirty_cells_ and evicted_keys_ — modified by the main
    // loop (update_from_traversability) and drained by the flush thread.
    mutable std::mutex db_mutex_;

    // Cells modified or created since the last flush (key → cell value at mark
    // time).  Cell value is captured in update_from_traversability() so the
    // flush thread never reads cells_ directly, avoiding a data race.
    std::unordered_map<CellKey, MissionLocalPlanningCell, CellKeyHash> dirty_cells_;
    // Keys evicted since the last flush (to be DELETEd from the DB).
    std::unordered_set<CellKey, CellKeyHash> evicted_keys_;

    // Sliding-window state (Stage 2).
    Vec3 last_slide_pos_{};
    bool slide_initialized_{false};

    // Stage 5: monotonically increasing version counter.
    // Incremented once per update_from_traversability() call.
    std::uint64_t map_seq_{0U};
};

}  // namespace dedalus
