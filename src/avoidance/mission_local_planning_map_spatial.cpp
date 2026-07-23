// mission_local_planning_map_spatial.cpp
//
// Stage 2: sliding in-memory window for MissionLocalPlanningMap.
// Implements: slide_window(), evict_far_cells(), load_window_from_db().
//
// Design
// ──────
// The in-memory store (cells_ / cell_index_) is capped to a sphere of radius
// horizon_m around the drone.  When the drone moves > horizon_m/4 from the
// last slide position, slide_window() is triggered:
//
//   1. evict_far_cells(drone_pos, 2×horizon_m)
//        Removes cells beyond the eviction radius from cells_/cell_index_.
//        Does NOT touch dirty_cells_ or evicted_keys_:
//          • Cells that were dirty before eviction retain their dirty entry;
//            the flush thread will write them to the DB as normal.
//          • This means re-entry (load_window_from_db) correctly picks up the
//            dirty value instead of the stale DB row.
//
//   2. load_window_from_db(drone_pos)
//        Bbox SELECT on cells table; skip cells already in cell_index_.
//        For cells also in dirty_cells_, the dirty value (fresher) replaces
//        the DB row value — snapshotted under db_mutex_ before the SELECT.
//
// Thread-safety
// ─────────────
// slide_window(), evict_far_cells(), and load_window_from_db() run on the
// main loop thread and do not touch dirty_cells_/evicted_keys_ (except for
// one lock acquisition in load_window_from_db to snapshot dirty_cells_).
// SQLite is compiled in serialised mode on macOS so concurrent access from the
// flush thread is safe without an additional application mutex.

#include "dedalus/avoidance/mission_local_planning_map.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>

#include <sqlite3.h>

namespace dedalus {

namespace {

inline sqlite3* as_db(void* p) noexcept { return static_cast<sqlite3*>(p); }

double dist3_sq(const Vec3& a, const Vec3& b) noexcept {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return dx * dx + dy * dy + dz * dz;
}

}  // namespace

// ─── evict_far_cells ─────────────────────────────────────────────────────────

void MissionLocalPlanningMap::evict_far_cells(const Vec3& centre,
                                               const double evict_radius_m) {
    const double r2 = evict_radius_m * evict_radius_m;

    const auto keep_end = std::stable_partition(
        cells_.begin(), cells_.end(),
        [&](const StoredCell& sc) {
            return dist3_sq(sc.cell.center_map, centre) <= r2;
        });

    if (keep_end == cells_.end()) {
        return;  // nothing to evict
    }

    cells_.erase(keep_end, cells_.end());

    // Rebuild index.
    cell_index_.clear();
    cell_index_.reserve(cells_.size());
    for (std::size_t i = 0U; i < cells_.size(); ++i) {
        cell_index_.emplace(cells_[i].key, i);
    }
}

// ─── load_window_from_db ─────────────────────────────────────────────────────

void MissionLocalPlanningMap::load_window_from_db(const Vec3& centre) {
    if (!db_) {
        return;
    }
    sqlite3* db = as_db(db_);

    const double hz = config_.horizon_m;

    // Snapshot dirty_cells_ under lock so we can use the fresher values for
    // cells that were modified in memory but not yet flushed to the DB.
    std::unordered_map<CellKey, MissionLocalPlanningCell, CellKeyHash> dirty_snap;
    {
        std::lock_guard<std::mutex> lk(db_mutex_);
        dirty_snap = dirty_cells_;
    }

    // Convert world bbox to cell-key ranges.
    const int xi_lo = static_cast<int>(std::floor((centre.x - hz) / config_.cell_size_m));
    const int xi_hi = static_cast<int>(std::floor((centre.x + hz) / config_.cell_size_m));
    const int yi_lo = static_cast<int>(std::floor((centre.y - hz) / config_.cell_size_m));
    const int yi_hi = static_cast<int>(std::floor((centre.y + hz) / config_.cell_size_m));
    const int zi_lo = static_cast<int>(
        std::floor((centre.z - hz) / config_.vertical_cell_size_m));
    const int zi_hi = static_cast<int>(
        std::floor((centre.z + hz) / config_.vertical_cell_size_m));

    const char* sql =
        "SELECT xi,yi,zi,score,confidence,count"
        "  FROM cells"
        "  WHERE xi BETWEEN ? AND ?"
        "    AND yi BETWEEN ? AND ?"
        "    AND zi BETWEEN ? AND ?"
        "    AND score >= ?;";

    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK) {
        return;
    }
    sqlite3_bind_int(stmt,    1, xi_lo);
    sqlite3_bind_int(stmt,    2, xi_hi);
    sqlite3_bind_int(stmt,    3, yi_lo);
    sqlite3_bind_int(stmt,    4, yi_hi);
    sqlite3_bind_int(stmt,    5, zi_lo);
    sqlite3_bind_int(stmt,    6, zi_hi);
    sqlite3_bind_double(stmt, 7, config_.min_occupied_score);

    while (sqlite3_step(stmt) == SQLITE_ROW) {
        CellKey key{
            sqlite3_column_int(stmt, 0),
            sqlite3_column_int(stmt, 1),
            sqlite3_column_int(stmt, 2),
        };
        if (cell_index_.count(key)) {
            continue;  // already in memory
        }

        MissionLocalPlanningCell cell;
        cell.center_map        = center_for_key(key);
        cell.occupied_score    = static_cast<float>(sqlite3_column_double(stmt, 3));
        cell.confidence        = static_cast<float>(sqlite3_column_double(stmt, 4));
        cell.source_cell_count = static_cast<std::uint32_t>(sqlite3_column_int(stmt, 5));

        // If this cell was modified in memory but not yet flushed, use the
        // fresher in-memory value rather than the stale DB row.
        const auto dirty_it = dirty_snap.find(key);
        if (dirty_it != dirty_snap.end()) {
            cell = dirty_it->second;
        }

        cells_.push_back(StoredCell{key, cell});
        cell_index_.emplace(key, cells_.size() - 1U);
    }
    sqlite3_finalize(stmt);
}

// ─── slide_window ────────────────────────────────────────────────────────────

bool MissionLocalPlanningMap::slide_window(const Vec3& drone_pos) {
    // No DB → evicted cells can't be reloaded; don't slide.
    if (!db_) {
        return false;
    }

    const double slide_threshold_m = config_.horizon_m / 4.0;

    if (slide_initialized_) {
        const double dist = std::sqrt(dist3_sq(drone_pos, last_slide_pos_));
        if (dist < slide_threshold_m) {
            return false;  // not moved enough
        }
    }

    last_slide_pos_    = drone_pos;
    slide_initialized_ = true;

    evict_far_cells(drone_pos, 2.0 * config_.horizon_m);
    load_window_from_db(drone_pos);
    return true;
}

}  // namespace dedalus
