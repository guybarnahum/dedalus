// mission_local_planning_map_sqlite.cpp
//
// SQLite persistence for MissionLocalPlanningMap (Stage 1).
// Implements: open_db(), flush_dirty_to_db(), close_db().
//
// Schema
// ──────
//   cells(xi, yi, zi  INTEGER PK,
//         score, confidence  REAL,
//         count  INTEGER,
//         updated_ns  INTEGER)
//
// The R-tree index and sync triggers are added in Stage 2 (spatial eviction).
// Stage 1 keeps only the cells table to meet the < 50 ms flush latency target.
//
// Thread-safety
// ─────────────
//   dirty_cells_ and evicted_keys_ are accessed by both the main loop
//   (update_from_traversability) and the flush thread.  They are protected by
//   db_mutex_.  flush_dirty_to_db() snapshots the sets under the lock, then
//   does all SQLite I/O outside the lock.

#include "dedalus/avoidance/mission_local_planning_map.hpp"

#include <chrono>
#include <mutex>
#include <unordered_set>
#include <utility>
#include <vector>

#include <sqlite3.h>

namespace dedalus {

namespace {

// ─── helpers ─────────────────────────────────────────────────────────────────

inline sqlite3* as_db(void* p) noexcept { return static_cast<sqlite3*>(p); }

// Execute a statement that returns no rows.  Ignores return code.
void exec(sqlite3* db, const char* sql) noexcept {
    sqlite3_exec(db, sql, nullptr, nullptr, nullptr);
}

// DDL executed once on open.
// R-tree and sync triggers are deferred to Stage 2.
const char* kSchema = R"(
PRAGMA journal_mode=WAL;
PRAGMA synchronous=NORMAL;

CREATE TABLE IF NOT EXISTS cells (
    xi         INTEGER NOT NULL,
    yi         INTEGER NOT NULL,
    zi         INTEGER NOT NULL,
    score      REAL    NOT NULL,
    confidence REAL    NOT NULL,
    count      INTEGER NOT NULL,
    updated_ns INTEGER NOT NULL DEFAULT 0,
    PRIMARY KEY (xi, yi, zi)
);

-- Config params written by the C++ runtime so viewers can reconstruct
-- cell centres correctly without needing CLI flags.
CREATE TABLE IF NOT EXISTS params (
    key   TEXT PRIMARY KEY,
    value REAL NOT NULL
);

-- L3 ESDF shell cells: derived from L2, full-replaced after each L2 flush.
-- Viewer uses these to render the repulsion vector field when no runtime
-- is connected.  Cells outside d0_m (stored in params) are not kept.
CREATE TABLE IF NOT EXISTS esdf_cells (
    cx     REAL NOT NULL,
    cy     REAL NOT NULL,
    cz     REAL NOT NULL,
    dist_m REAL NOT NULL,
    gx     REAL NOT NULL,
    gy     REAL NOT NULL,
    gz     REAL NOT NULL,
    sgx    REAL NOT NULL,
    sgy    REAL NOT NULL,
    sgz    REAL NOT NULL
);
)";

const char* kUpsert =
    "INSERT INTO cells(xi,yi,zi,score,confidence,count,updated_ns)"
    "  VALUES(?,?,?,?,?,?,?)"
    "  ON CONFLICT(xi,yi,zi) DO UPDATE SET"
    "    score=excluded.score,"
    "    confidence=excluded.confidence,"
    "    count=excluded.count,"
    "    updated_ns=excluded.updated_ns;";

const char* kDelete =
    "DELETE FROM cells WHERE xi=? AND yi=? AND zi=?;";

const char* kSelectAll =
    "SELECT xi,yi,zi,score,confidence,count,updated_ns"
    "  FROM cells"
    "  WHERE score>=?;";

}  // namespace

// ─── open_db ─────────────────────────────────────────────────────────────────

bool MissionLocalPlanningMap::open_db(const std::filesystem::path& path) {
    if (db_) {
        close_db();
    }

    sqlite3* db = nullptr;
    if (sqlite3_open(path.c_str(), &db) != SQLITE_OK) {
        sqlite3_close(db);
        return false;
    }
    db_ = db;

    exec(db, kSchema);

    // Persist config so the viewer can reconstruct cell centres without CLI flags.
    {
        const char* kUpsertParam =
            "INSERT INTO params(key,value) VALUES(?,?)"
            " ON CONFLICT(key) DO UPDATE SET value=excluded.value;";
        sqlite3_stmt* ps = nullptr;
        if (sqlite3_prepare_v2(db, kUpsertParam, -1, &ps, nullptr) == SQLITE_OK) {
            auto bind_kv = [&](const char* k, double v) {
                sqlite3_bind_text(ps, 1, k, -1, SQLITE_STATIC);
                sqlite3_bind_double(ps, 2, v);
                sqlite3_step(ps);
                sqlite3_reset(ps);
            };
            bind_kv("cell_size_m",          config_.cell_size_m);
            bind_kv("vertical_cell_size_m", config_.vertical_cell_size_m);
            sqlite3_finalize(ps);
        }
    }

    // Load all cells with score >= min_occupied_score into memory.
    reset();

    sqlite3_stmt* sel = nullptr;
    if (sqlite3_prepare_v2(db, kSelectAll, -1, &sel, nullptr) != SQLITE_OK) {
        return false;  // DB open succeeded but schema may be empty.
    }
    sqlite3_bind_double(sel, 1, config_.min_occupied_score);

    while (sqlite3_step(sel) == SQLITE_ROW) {
        CellKey key{
            sqlite3_column_int(sel, 0),
            sqlite3_column_int(sel, 1),
            sqlite3_column_int(sel, 2),
        };
        if (cell_index_.find(key) != cell_index_.end()) {
            continue;  // duplicate (shouldn't happen with PK)
        }
        MissionLocalPlanningCell cell;
        cell.center_map        = center_for_key(key);
        cell.occupied_score    = static_cast<float>(sqlite3_column_double(sel, 3));
        cell.confidence        = static_cast<float>(sqlite3_column_double(sel, 4));
        cell.source_cell_count = static_cast<std::uint32_t>(sqlite3_column_int(sel, 5));
        cell.last_updated_ns   = sqlite3_column_int64(sel, 6);  // 0 if never flushed
        cells_.push_back(StoredCell{key, cell});
        cell_index_.emplace(key, cells_.size() - 1U);
    }
    sqlite3_finalize(sel);

    return true;
}

// ─── flush_dirty_to_db ───────────────────────────────────────────────────────

bool MissionLocalPlanningMap::flush_dirty_to_db() {
    if (!db_) {
        return true;  // no DB open — nothing to do
    }
    sqlite3* db = as_db(db_);

    // ── snapshot under lock ──────────────────────────────────────────────────
    // dirty_cells_ stores (key, cell-value-at-mark-time) — no access to cells_
    // needed here, so there is no data race with the main loop.
    std::vector<std::pair<CellKey, MissionLocalPlanningCell>> to_upsert;
    std::vector<CellKey> to_delete;

    {
        std::lock_guard<std::mutex> lk(db_mutex_);

        to_upsert.reserve(dirty_cells_.size());
        for (const auto& [key, cell] : dirty_cells_) {
            to_upsert.emplace_back(key, cell);
        }

        to_delete.assign(evicted_keys_.begin(), evicted_keys_.end());

        dirty_cells_.clear();
        evicted_keys_.clear();
    }

    if (to_upsert.empty() && to_delete.empty()) {
        return true;
    }

    // ── SQLite I/O outside the lock ──────────────────────────────────────────

    // system_clock so updated_ns is a Unix wall-clock timestamp that viewers
    // can compare against Date.now().  steady_clock would give nanoseconds
    // since boot — not comparable to a Unix epoch.
    const auto now_ns = static_cast<sqlite3_int64>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());

    exec(db, "BEGIN;");

    // Upserts.
    sqlite3_stmt* ups = nullptr;
    sqlite3_prepare_v2(db, kUpsert, -1, &ups, nullptr);
    for (const auto& [key, cell] : to_upsert) {
        sqlite3_bind_int(ups,    1, key.x);
        sqlite3_bind_int(ups,    2, key.y);
        sqlite3_bind_int(ups,    3, key.z);
        sqlite3_bind_double(ups, 4, static_cast<double>(cell.occupied_score));
        sqlite3_bind_double(ups, 5, static_cast<double>(cell.confidence));
        sqlite3_bind_int64(ups,  6,
            static_cast<sqlite3_int64>(cell.source_cell_count));
        sqlite3_bind_int64(ups,  7, now_ns);
        sqlite3_step(ups);
        sqlite3_reset(ups);
    }
    sqlite3_finalize(ups);

    // Deletes.
    sqlite3_stmt* del = nullptr;
    sqlite3_prepare_v2(db, kDelete, -1, &del, nullptr);
    for (const auto& key : to_delete) {
        sqlite3_bind_int(del, 1, key.x);
        sqlite3_bind_int(del, 2, key.y);
        sqlite3_bind_int(del, 3, key.z);
        sqlite3_step(del);
        sqlite3_reset(del);
    }
    sqlite3_finalize(del);

    exec(db, "COMMIT;");

    return true;
}

// ─── close_db ────────────────────────────────────────────────────────────────

bool MissionLocalPlanningMap::close_db() {
    if (!db_) {
        return true;
    }
    const bool ok = flush_dirty_to_db();
    sqlite3_close(as_db(db_));
    db_ = nullptr;
    return ok;
}

// ─── flush_esdf_to_db ─────────────────────────────────────────────────────────
//
// Full replace of the esdf_cells table: DELETE all existing rows then INSERT
// the current ESDF shell-cell snapshot in one transaction.  Also upserts
// esdf_d0_m into the params table so the viewer knows the truncation radius.

bool MissionLocalPlanningMap::flush_esdf_to_db(
    const std::vector<ESDFCellRecord>& cells, double d0_m)
{
    if (!db_) return true;
    sqlite3* db = as_db(db_);

    // Persist d0_m for the viewer.
    {
        const char* kUpsertParam =
            "INSERT INTO params(key,value) VALUES(?,?)"
            " ON CONFLICT(key) DO UPDATE SET value=excluded.value;";
        sqlite3_stmt* ps = nullptr;
        if (sqlite3_prepare_v2(db, kUpsertParam, -1, &ps, nullptr) == SQLITE_OK) {
            sqlite3_bind_text(ps, 1, "esdf_d0_m", -1, SQLITE_STATIC);
            sqlite3_bind_double(ps, 2, d0_m);
            sqlite3_step(ps);
            sqlite3_finalize(ps);
        }
    }

    exec(db, "BEGIN;");
    exec(db, "DELETE FROM esdf_cells;");

    if (!cells.empty()) {
        const char* kInsert =
            "INSERT INTO esdf_cells(cx,cy,cz,dist_m,gx,gy,gz,sgx,sgy,sgz)"
            " VALUES(?,?,?,?,?,?,?,?,?,?);";
        sqlite3_stmt* ins = nullptr;
        sqlite3_prepare_v2(db, kInsert, -1, &ins, nullptr);
        for (const auto& c : cells) {
            sqlite3_bind_double(ins,  1, c.cx);
            sqlite3_bind_double(ins,  2, c.cy);
            sqlite3_bind_double(ins,  3, c.cz);
            sqlite3_bind_double(ins,  4, c.dist_m);
            sqlite3_bind_double(ins,  5, c.gx);
            sqlite3_bind_double(ins,  6, c.gy);
            sqlite3_bind_double(ins,  7, c.gz);
            sqlite3_bind_double(ins,  8, c.sgx);
            sqlite3_bind_double(ins,  9, c.sgy);
            sqlite3_bind_double(ins, 10, c.sgz);
            sqlite3_step(ins);
            sqlite3_reset(ins);
        }
        sqlite3_finalize(ins);
    }

    exec(db, "COMMIT;");
    return true;
}

}  // namespace dedalus
