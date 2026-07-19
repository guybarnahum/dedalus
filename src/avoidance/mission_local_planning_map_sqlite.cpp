// mission_local_planning_map_sqlite.cpp
//
// SQLite persistence for MissionLocalPlanningMap.
//
// Schema (v3)
// ──────────────────────────────────────────────────────────────────────────
//   methods(id PK, name UNIQUE)          — detection method names → int IDs
//
//   cells(xi, yi, zi PK,
//         score, confidence REAL,
//         log_odds REAL,                 — primary evidence accumulator
//         count INTEGER,
//         mission_count INTEGER,         — distinct missions that observed this cell
//         updated_ns INTEGER)            — last-write wall-clock ns
//
//   cell_votes(xi, yi, zi, mission_id PK,
//              method_id INTEGER → methods.id,
//              hit_count INTEGER,        — how many ticks this mission touched this cell
//              first_ns, last_ns INTEGER)
//
//   params(key TEXT PK, value REAL)      — config scalars for viewers
//
//   esdf_cells(cx,cy,cz,dist_m,gx,gy,gz,sgx,sgy,sgz)
//
// Triggers trg_cv_insert / trg_cv_delete keep cells.mission_count in sync
// whenever cell_votes rows are inserted or deleted.
//
// Schema migration
// ──────────────────────────────────────────────────────────────────────────
//   v1/v2 databases (no log_odds column) are upgraded in-place via ALTER TABLE
//   (exec ignores errors on already-existing columns).  Old cells start with
//   log_odds = 0 (DEFAULT 0) and accrue evidence as the current mission updates
//   them — occupied_score is recomputed as sigmoid(log_odds) on each write.
//
// Thread-safety
// ──────────────────────────────────────────────────────────────────────────
//   dirty_cells_ and evicted_keys_ are protected by db_mutex_.
//   flush_dirty_to_db() snapshots under the lock then does all SQLite I/O
//   outside the lock.  remove_mission() must be called from a single thread
//   (editor context only); it is not safe to call concurrently with the
//   flush thread.

#include "dedalus/avoidance/mission_local_planning_map.hpp"

#include <chrono>
#include <cmath>
#include <mutex>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>

#include <sqlite3.h>

namespace dedalus {

namespace {

inline sqlite3* as_db(void* p) noexcept { return static_cast<sqlite3*>(p); }

inline float sigmoid(const float x) noexcept {
    return 1.0F / (1.0F + std::exp(-x));
}

// Execute a statement that returns no rows.  Ignores all errors.
void exec(sqlite3* db, const char* sql) noexcept {
    sqlite3_exec(db, sql, nullptr, nullptr, nullptr);
}

// ─── base DDL ────────────────────────────────────────────────────────────────
// CREATE TABLE IF NOT EXISTS keeps this idempotent on both new and v1 databases.
// mission_count column is also added via ALTER TABLE below for v1 databases.

const char* kSchema = R"(
PRAGMA journal_mode=WAL;
PRAGMA synchronous=NORMAL;

CREATE TABLE IF NOT EXISTS methods (
    id   INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT UNIQUE NOT NULL
);

CREATE TABLE IF NOT EXISTS cells (
    xi            INTEGER NOT NULL,
    yi            INTEGER NOT NULL,
    zi            INTEGER NOT NULL,
    score         REAL    NOT NULL,
    confidence    REAL    NOT NULL,
    log_odds      REAL    NOT NULL DEFAULT 0,
    count         INTEGER NOT NULL,
    mission_count INTEGER NOT NULL DEFAULT 0,
    updated_ns    INTEGER NOT NULL DEFAULT 0,
    PRIMARY KEY (xi, yi, zi)
);

CREATE TABLE IF NOT EXISTS cell_votes (
    xi         INTEGER NOT NULL,
    yi         INTEGER NOT NULL,
    zi         INTEGER NOT NULL,
    mission_id TEXT    NOT NULL,
    method_id  INTEGER NOT NULL,
    hit_count  INTEGER NOT NULL DEFAULT 1,
    first_ns   INTEGER NOT NULL,
    last_ns    INTEGER NOT NULL,
    PRIMARY KEY (xi, yi, zi, mission_id),
    FOREIGN KEY (method_id) REFERENCES methods(id)
);

CREATE TABLE IF NOT EXISTS params (
    key   TEXT PRIMARY KEY,
    value REAL NOT NULL
);

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

// Upsert a cell (does not touch mission_count — that is trigger-managed).
const char* kUpsert =
    "INSERT INTO cells(xi,yi,zi,score,confidence,log_odds,count,updated_ns)"
    "  VALUES(?,?,?,?,?,?,?,?)"
    "  ON CONFLICT(xi,yi,zi) DO UPDATE SET"
    "    score=excluded.score,"
    "    confidence=excluded.confidence,"
    "    log_odds=excluded.log_odds,"
    "    count=excluded.count,"
    "    updated_ns=excluded.updated_ns;";

// Upsert a vote row.  On conflict: increment hit_count, update last_ns.
// The AFTER INSERT trigger trg_cv_insert fires only on genuine new rows (not
// on the ON CONFLICT UPDATE path), so mission_count increments exactly once
// per new (cell, mission) pair.
const char* kVoteUpsert =
    "INSERT INTO cell_votes(xi,yi,zi,mission_id,method_id,hit_count,first_ns,last_ns)"
    "  VALUES(?,?,?,?,?,1,?,?)"
    "  ON CONFLICT(xi,yi,zi,mission_id) DO UPDATE SET"
    "    hit_count = hit_count + 1,"
    "    last_ns = excluded.last_ns;";

const char* kDelete =
    "DELETE FROM cells WHERE xi=? AND yi=? AND zi=?;";

const char* kSelectAll =
    "SELECT xi,yi,zi,score,confidence,count,updated_ns,log_odds"
    "  FROM cells"
    "  WHERE score>=?;";

}  // namespace

// ─── set_mission_context ─────────────────────────────────────────────────────

void MissionLocalPlanningMap::set_mission_context(std::string_view mission_id,
                                                   std::string_view method) {
    mission_id_ = std::string{mission_id};
    method_     = std::string{method};
    // method_id_ is resolved in open_db() once the DB is open.
}

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

    // Create / ensure tables.
    exec(db, kSchema);

    // ── v1 → v2 migration ────────────────────────────────────────────────────
    // Add mission_count if this is a v1 database (exec ignores error if it
    // already exists).
    exec(db, "ALTER TABLE cells ADD COLUMN mission_count INTEGER NOT NULL DEFAULT 0;");

    // ── v2 → v3 migration ────────────────────────────────────────────────────
    // Add log_odds column if this is a v1/v2 database (exec ignores error if
    // it already exists).  Old cells start with log_odds = 0 and accrue
    // evidence from the current mission going forward.
    exec(db, "ALTER TABLE cells ADD COLUMN log_odds REAL NOT NULL DEFAULT 0;");

    // ── triggers (recreate each open to handle v1 databases) ─────────────────
    exec(db, "DROP TRIGGER IF EXISTS trg_cv_insert;");
    exec(db, R"(
CREATE TRIGGER trg_cv_insert AFTER INSERT ON cell_votes BEGIN
    UPDATE cells SET mission_count = (
        SELECT COUNT(*) FROM cell_votes
        WHERE xi=NEW.xi AND yi=NEW.yi AND zi=NEW.zi
    ) WHERE xi=NEW.xi AND yi=NEW.yi AND zi=NEW.zi;
END;)");

    exec(db, "DROP TRIGGER IF EXISTS trg_cv_delete;");
    exec(db, R"(
CREATE TRIGGER trg_cv_delete AFTER DELETE ON cell_votes BEGIN
    UPDATE cells SET mission_count = (
        SELECT COUNT(*) FROM cell_votes
        WHERE xi=OLD.xi AND yi=OLD.yi AND zi=OLD.zi
    ) WHERE xi=OLD.xi AND yi=OLD.yi AND zi=OLD.zi;
END;)");

    // ── persist config scalars ────────────────────────────────────────────────
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

    // ── resolve method_id ─────────────────────────────────────────────────────
    // Insert the current session's method name into methods if not present,
    // then read back its id.
    {
        sqlite3_stmt* ps = nullptr;
        const char* kInsert = "INSERT OR IGNORE INTO methods(name) VALUES(?);";
        if (sqlite3_prepare_v2(db, kInsert, -1, &ps, nullptr) == SQLITE_OK) {
            sqlite3_bind_text(ps, 1, method_.c_str(), -1, SQLITE_TRANSIENT);
            sqlite3_step(ps);
            sqlite3_finalize(ps);
        }
        const char* kSelect = "SELECT id FROM methods WHERE name=?;";
        if (sqlite3_prepare_v2(db, kSelect, -1, &ps, nullptr) == SQLITE_OK) {
            sqlite3_bind_text(ps, 1, method_.c_str(), -1, SQLITE_TRANSIENT);
            if (sqlite3_step(ps) == SQLITE_ROW) {
                method_id_ = sqlite3_column_int64(ps, 0);
            }
            sqlite3_finalize(ps);
        }
    }

    // ── load cells ───────────────────────────────────────────────────────────
    reset();

    sqlite3_stmt* sel = nullptr;
    if (sqlite3_prepare_v2(db, kSelectAll, -1, &sel, nullptr) != SQLITE_OK) {
        return false;
    }
    sqlite3_bind_double(sel, 1, config_.min_occupied_score);

    while (sqlite3_step(sel) == SQLITE_ROW) {
        CellKey key{
            sqlite3_column_int(sel, 0),
            sqlite3_column_int(sel, 1),
            sqlite3_column_int(sel, 2),
        };
        if (cell_index_.find(key) != cell_index_.end()) {
            continue;
        }
        MissionLocalPlanningCell cell;
        cell.center_map        = center_for_key(key);
        // col 3: score (legacy; ignored — recomputed from log_odds below)
        cell.confidence        = static_cast<float>(sqlite3_column_double(sel, 4));
        cell.source_cell_count = static_cast<std::uint32_t>(sqlite3_column_int(sel, 5));
        cell.last_updated_ns   = sqlite3_column_int64(sel, 6);
        cell.log_odds          = static_cast<float>(sqlite3_column_double(sel, 7));
        cell.occupied_score    = sigmoid(cell.log_odds);
        cells_.push_back(StoredCell{key, cell});
        cell_index_.emplace(key, cells_.size() - 1U);
    }
    sqlite3_finalize(sel);

    return true;
}

// ─── flush_dirty_to_db ───────────────────────────────────────────────────────

bool MissionLocalPlanningMap::flush_dirty_to_db() {
    if (!db_) {
        return true;
    }
    sqlite3* db = as_db(db_);

    // ── snapshot under lock ──────────────────────────────────────────────────
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
    const auto now_ns = static_cast<sqlite3_int64>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());

    exec(db, "BEGIN;");

    // Cell upserts.
    sqlite3_stmt* ups = nullptr;
    sqlite3_prepare_v2(db, kUpsert, -1, &ups, nullptr);
    for (const auto& [key, cell] : to_upsert) {
        sqlite3_bind_int(ups,    1, key.x);
        sqlite3_bind_int(ups,    2, key.y);
        sqlite3_bind_int(ups,    3, key.z);
        sqlite3_bind_double(ups, 4, static_cast<double>(cell.occupied_score));
        sqlite3_bind_double(ups, 5, static_cast<double>(cell.confidence));
        sqlite3_bind_double(ups, 6, static_cast<double>(cell.log_odds));
        sqlite3_bind_int64(ups,  7, static_cast<sqlite3_int64>(cell.source_cell_count));
        sqlite3_bind_int64(ups,  8, now_ns);
        sqlite3_step(ups);
        sqlite3_reset(ups);
    }
    sqlite3_finalize(ups);

    // Vote upserts (only when the session has a resolved method_id).
    if (method_id_ >= 0 && !mission_id_.empty()) {
        sqlite3_stmt* vps = nullptr;
        sqlite3_prepare_v2(db, kVoteUpsert, -1, &vps, nullptr);
        for (const auto& [key, cell] : to_upsert) {
            sqlite3_bind_int(vps,    1, key.x);
            sqlite3_bind_int(vps,    2, key.y);
            sqlite3_bind_int(vps,    3, key.z);
            sqlite3_bind_text(vps,   4, mission_id_.c_str(), -1, SQLITE_STATIC);
            sqlite3_bind_int64(vps,  5, method_id_);
            sqlite3_bind_int64(vps,  6, now_ns);  // first_ns (ignored on conflict)
            sqlite3_bind_int64(vps,  7, now_ns);  // last_ns
            sqlite3_step(vps);
            sqlite3_reset(vps);
        }
        sqlite3_finalize(vps);
    }

    // Cell deletes (also removes orphan vote rows).
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

    // Prune orphaned vote rows for deleted cells (FK cascade not guaranteed).
    if (!to_delete.empty()) {
        exec(db, "DELETE FROM cell_votes WHERE NOT EXISTS ("
                 "  SELECT 1 FROM cells WHERE cells.xi=cell_votes.xi"
                 "    AND cells.yi=cell_votes.yi AND cells.zi=cell_votes.zi);");
    }

    exec(db, "COMMIT;");
    return true;
}

// ─── remove_mission ──────────────────────────────────────────────────────────
//
// Deletes all cell_votes for mission_id from the open DB.  Triggers update
// mission_count per affected cell.  Cells whose count reaches 0 are then
// deleted.  Operates DB-only — does not evict from the in-memory window.
// Caller should reload (reset() + open_db()) to synchronise in-memory state.

std::size_t MissionLocalPlanningMap::remove_mission(std::string_view mission_id) {
    if (!db_) return 0U;
    sqlite3* db = as_db(db_);

    exec(db, "BEGIN;");

    // Count cells before deletion so we can report how many were orphaned.
    int cells_before = 0;
    {
        sqlite3_stmt* ps = nullptr;
        if (sqlite3_prepare_v2(db, "SELECT COUNT(*) FROM cells;", -1, &ps, nullptr) == SQLITE_OK) {
            if (sqlite3_step(ps) == SQLITE_ROW) {
                cells_before = sqlite3_column_int(ps, 0);
            }
            sqlite3_finalize(ps);
        }
    }

    // Delete votes (triggers decrement mission_count per affected cell).
    sqlite3_stmt* ps = nullptr;
    if (sqlite3_prepare_v2(db, "DELETE FROM cell_votes WHERE mission_id=?;",
                            -1, &ps, nullptr) == SQLITE_OK) {
        sqlite3_bind_text(ps, 1, mission_id.data(),
                          static_cast<int>(mission_id.size()), SQLITE_TRANSIENT);
        sqlite3_step(ps);
        sqlite3_finalize(ps);
    }

    // Delete cells with no remaining votes.
    exec(db, "DELETE FROM cells WHERE mission_count = 0;");

    exec(db, "COMMIT;");

    // Count how many cells were removed.
    int cells_after = 0;
    {
        if (sqlite3_prepare_v2(db, "SELECT COUNT(*) FROM cells;", -1, &ps, nullptr) == SQLITE_OK) {
            if (sqlite3_step(ps) == SQLITE_ROW) {
                cells_after = sqlite3_column_int(ps, 0);
            }
            sqlite3_finalize(ps);
        }
    }

    return static_cast<std::size_t>(cells_before - cells_after);
}

// ─── close_db ────────────────────────────────────────────────────────────────

bool MissionLocalPlanningMap::close_db() {
    if (!db_) {
        return true;
    }
    const bool ok = flush_dirty_to_db();
    sqlite3_close(as_db(db_));
    db_        = nullptr;
    method_id_ = -1;
    return ok;
}

// ─── flush_esdf_to_db ────────────────────────────────────────────────────────

bool MissionLocalPlanningMap::flush_esdf_to_db(
    const std::vector<ESDFCellRecord>& cells, double d0_m)
{
    if (!db_) return true;
    sqlite3* db = as_db(db_);

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
