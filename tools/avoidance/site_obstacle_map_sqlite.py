#!/usr/bin/env python3
"""SQLite persistent obstacle site-map prototype.

5L.1: efficient persistent site-memory backend prototype.

This tool is intentionally outside the flight hot path. It provides an efficient
backend for persistent site memory while keeping JSON as debug/import/export.

Subcommands:
  import-json    Import a debug site_obstacle_map.json into SQLite.
  score          Recompute derived scores/status in SQLite.
  summary        Print summary statistics from SQLite.
  query-region   Query cells by site/cell bounds/status.
  export-json    Export SQLite store back to debug JSON.

Runtime design rule:
  Flight code should use in-memory runtime maps. SQLite is for preload,
  persistence, scoring, post-mission merge, and debug conversion.
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import os
import sqlite3
import tempfile
import time
from pathlib import Path
from typing import Any


SCHEMA = "dedalus.site_obstacle_map.sqlite.v1"
DEBUG_JSON_SCHEMA = "dedalus.site_obstacle_map.v1"
TIME_UNIT = "unix_ns"


def now_unix_ns() -> int:
    return time.time_ns()


def error(message: str) -> int:
    print(f"ERROR: {message}")
    return 1


def load_json(path: Path) -> dict[str, Any] | None:
    try:
        with path.open("r", encoding="utf-8") as f:
            data = json.load(f)
    except OSError as exc:
        print(f"ERROR: failed to read {path}: {exc}")
        return None
    except json.JSONDecodeError as exc:
        print(f"ERROR: failed to parse JSON {path}: {exc}")
        return None

    if not isinstance(data, dict):
        print(f"ERROR: JSON root is not an object: {path}")
        return None
    return data


def atomic_write_json(path: Path, data: dict[str, Any]) -> bool:
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        encoded = json.dumps(data, indent=2, sort_keys=True) + "\n"
        with tempfile.NamedTemporaryFile(
            "w",
            encoding="utf-8",
            dir=str(path.parent),
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as f:
            tmp = Path(f.name)
            f.write(encoded)
        os.replace(tmp, path)
        return True
    except OSError as exc:
        print(f"ERROR: failed to write {path}: {exc}")
        return False


def as_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def as_float(value: Any, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def sigmoid(log_odds: float) -> float:
    if log_odds >= 0.0:
        z = math.exp(-log_odds)
        return 1.0 / (1.0 + z)
    z = math.exp(log_odds)
    return z / (1.0 + z)


def logit(probability: float) -> float:
    p = clamp(probability, 1e-6, 1.0 - 1e-6)
    return math.log(p / (1.0 - p))


def parse_point(value: Any) -> tuple[float, float, float]:
    if isinstance(value, dict):
        return (
            as_float(value.get("x")),
            as_float(value.get("y")),
            as_float(value.get("z")),
        )
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        return (as_float(value[0]), as_float(value[1]), as_float(value[2]))
    return (0.0, 0.0, 0.0)


def cell_key_from_json(
    cell: dict[str, Any],
    *,
    site_id: str,
    cell_size_m: float,
    vertical_cell_size_m: float,
) -> tuple[str, int, int, int]:
    key = cell.get("key")
    if isinstance(key, dict):
        return (
            str(key.get("site_id") or site_id),
            as_int(key.get("ix")),
            as_int(key.get("iy")),
            as_int(key.get("iz")),
        )

    for candidate in ("cell_key", "grid_key", "index", "grid_index"):
        value = cell.get(candidate)
        if isinstance(value, dict):
            return (
                str(value.get("site_id") or site_id),
                as_int(value.get("ix")),
                as_int(value.get("iy")),
                as_int(value.get("iz")),
            )

    if all(k in cell for k in ("ix", "iy", "iz")):
        return (site_id, as_int(cell.get("ix")), as_int(cell.get("iy")), as_int(cell.get("iz")))

    if isinstance(key, str):
        import re

        nums = [int(x) for x in re.findall(r"-?\d+", key)]
        if len(nums) >= 3:
            return (site_id, nums[-3], nums[-2], nums[-1])

    center = parse_point(cell.get("center_site") or cell.get("center_mission") or cell.get("center"))
    cs = cell_size_m if cell_size_m > 0 else 0.5
    vz = vertical_cell_size_m if vertical_cell_size_m > 0 else cs
    return (site_id, round(center[0] / cs), round(center[1] / cs), round(center[2] / vz))


def occupied_log_odds_from_json(cell: dict[str, Any]) -> float:
    if "occupied_log_odds" in cell:
        return as_float(cell.get("occupied_log_odds"))

    if "occupancy_score" in cell:
        return logit(as_float(cell.get("occupancy_score")))

    derived = cell.get("derived")
    if isinstance(derived, dict) and "occupancy_score" in derived:
        return logit(as_float(derived.get("occupancy_score")))

    if "normalized_occupied_score" in cell:
        return logit(as_float(cell.get("normalized_occupied_score")))

    if "occupied_probability" in cell:
        return logit(as_float(cell.get("occupied_probability")))

    if cell.get("occupied") is True:
        return logit(0.75)

    return 0.0


def source_from_json(cell: dict[str, Any]) -> tuple[str, str]:
    source = cell.get("source")
    if isinstance(source, dict):
        return (
            str(source.get("last_source_kind") or source.get("source_kind") or source.get("kind") or ""),
            str(source.get("last_source_provider") or source.get("source_provider") or source.get("provider") or ""),
        )

    return (
        str(cell.get("last_source_kind") or cell.get("source_kind") or ""),
        str(cell.get("last_source_provider") or cell.get("source_provider") or ""),
    )


def connect(path: Path) -> sqlite3.Connection | None:
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        conn = sqlite3.connect(str(path))
        conn.row_factory = sqlite3.Row
        conn.execute("PRAGMA journal_mode=WAL")
        conn.execute("PRAGMA synchronous=NORMAL")
        conn.execute("PRAGMA temp_store=MEMORY")
        conn.execute("PRAGMA foreign_keys=ON")
        return conn
    except sqlite3.Error as exc:
        print(f"ERROR: failed to open sqlite database {path}: {exc}")
        return None


def create_schema(conn: sqlite3.Connection) -> None:
    conn.executescript(
        """
        CREATE TABLE IF NOT EXISTS site_metadata (
          key TEXT PRIMARY KEY,
          value TEXT NOT NULL
        ) WITHOUT ROWID;

        CREATE TABLE IF NOT EXISTS missions (
          mission_id TEXT PRIMARY KEY,
          site_id TEXT NOT NULL,
          mission_map_frame_id TEXT,
          mission_start_unix_ns INTEGER,
          mission_end_unix_ns INTEGER,
          merged_at_unix_ns INTEGER,
          source_artifact TEXT
        ) WITHOUT ROWID;

        CREATE TABLE IF NOT EXISTS cells (
          site_id TEXT NOT NULL,
          ix INTEGER NOT NULL,
          iy INTEGER NOT NULL,
          iz INTEGER NOT NULL,

          center_x REAL NOT NULL,
          center_y REAL NOT NULL,
          center_z REAL NOT NULL,

          occupied_log_odds REAL NOT NULL,
          free_log_odds REAL NOT NULL,

          positive_observation_count INTEGER NOT NULL,
          free_observation_count INTEGER NOT NULL,

          first_seen_unix_ns INTEGER NOT NULL,
          last_seen_unix_ns INTEGER NOT NULL,
          last_confirmed_occupied_unix_ns INTEGER NOT NULL,
          last_observed_free_unix_ns INTEGER NOT NULL,

          last_source_kind TEXT,
          last_source_provider TEXT,

          PRIMARY KEY(site_id, ix, iy, iz)
        ) WITHOUT ROWID;

        CREATE TABLE IF NOT EXISTS cell_missions (
          site_id TEXT NOT NULL,
          ix INTEGER NOT NULL,
          iy INTEGER NOT NULL,
          iz INTEGER NOT NULL,
          mission_id TEXT NOT NULL,
          PRIMARY KEY(site_id, ix, iy, iz, mission_id)
        ) WITHOUT ROWID;

        CREATE TABLE IF NOT EXISTS derived_scores (
          site_id TEXT NOT NULL,
          ix INTEGER NOT NULL,
          iy INTEGER NOT NULL,
          iz INTEGER NOT NULL,

          occupancy_score REAL NOT NULL,
          freshness_score REAL NOT NULL,
          active_score REAL NOT NULL,
          cell_age_seconds REAL NOT NULL,
          site_staleness_seconds REAL NOT NULL,
          relative_gap_seconds REAL NOT NULL,
          site_relative_age_percentile REAL NOT NULL,
          status TEXT NOT NULL,
          scored_at_unix_ns INTEGER NOT NULL,

          PRIMARY KEY(site_id, ix, iy, iz)
        ) WITHOUT ROWID;

        CREATE INDEX IF NOT EXISTS cells_last_seen_idx
          ON cells(site_id, last_seen_unix_ns);

        CREATE INDEX IF NOT EXISTS scores_status_idx
          ON derived_scores(site_id, status);

        CREATE INDEX IF NOT EXISTS scores_active_idx
          ON derived_scores(site_id, active_score);

        CREATE INDEX IF NOT EXISTS cell_missions_mission_idx
          ON cell_missions(mission_id);
        """
    )


def set_meta(conn: sqlite3.Connection, key: str, value: Any) -> None:
    conn.execute(
        "INSERT INTO site_metadata(key, value) VALUES(?, ?) "
        "ON CONFLICT(key) DO UPDATE SET value=excluded.value",
        (key, json.dumps(value, sort_keys=True)),
    )


def get_meta(conn: sqlite3.Connection, key: str, default: Any = None) -> Any:
    row = conn.execute("SELECT value FROM site_metadata WHERE key = ?", (key,)).fetchone()
    if row is None:
        return default
    try:
        return json.loads(row["value"])
    except json.JSONDecodeError:
        return default


def import_json(args: argparse.Namespace) -> int:
    input_path = Path(args.input)
    db_path = Path(args.db)

    data = load_json(input_path)
    if data is None:
        return 1

    if args.replace and db_path.exists():
        try:
            db_path.unlink()
            for suffix in ("-wal", "-shm"):
                sidecar = Path(str(db_path) + suffix)
                if sidecar.exists():
                    sidecar.unlink()
        except OSError as exc:
            return error(f"failed to remove existing database {db_path}: {exc}")

    if data.get("schema") != DEBUG_JSON_SCHEMA:
        print(f"warning: importing non-standard debug schema: {data.get('schema')}", flush=True)

    site_id = str(args.site_id or data.get("site_id") or "unknown_site")
    site_frame_id = str(args.site_frame_id or data.get("site_frame_id") or "")
    cell_size_m = as_float(data.get("cell_size_m"), as_float(args.cell_size_m, 0.5))
    vertical_cell_size_m = as_float(data.get("vertical_cell_size_m"), as_float(args.vertical_cell_size_m, cell_size_m))
    now_ns = now_unix_ns()

    cells = data.get("cells")
    if not isinstance(cells, list):
        return error("input JSON has no cells array")

    conn = connect(db_path)
    if conn is None:
        return 1

    try:
        create_schema(conn)

        with conn:
            set_meta(conn, "schema", SCHEMA)
            set_meta(conn, "debug_source_schema", data.get("schema"))
            set_meta(conn, "site_id", site_id)
            set_meta(conn, "site_frame_id", site_frame_id)
            set_meta(conn, "time_unit", TIME_UNIT)
            set_meta(conn, "cell_size_m", cell_size_m)
            set_meta(conn, "vertical_cell_size_m", vertical_cell_size_m)
            set_meta(conn, "created_at_unix_ns", data.get("created_at_unix_ns", now_ns))
            set_meta(conn, "updated_at_unix_ns", now_ns)
            set_meta(conn, "site_last_visited_unix_ns", data.get("site_last_visited_unix_ns", 0))
            set_meta(conn, "merge_summary", data.get("merge_summary", {}))
            set_meta(conn, "decay_policy", data.get("decay_policy", {}))

            merge_summary = data.get("merge_summary")
            if isinstance(merge_summary, dict):
                for mission_id in merge_summary.get("merged_mission_ids", []) or []:
                    conn.execute(
                        "INSERT OR IGNORE INTO missions("
                        "mission_id, site_id, mission_map_frame_id, mission_start_unix_ns, "
                        "mission_end_unix_ns, merged_at_unix_ns, source_artifact"
                        ") VALUES (?, ?, ?, ?, ?, ?, ?)",
                        (
                            str(mission_id),
                            site_id,
                            "",
                            0,
                            as_int(merge_summary.get("last_merged_mission_end_unix_ns")),
                            now_ns,
                            str(input_path),
                        ),
                    )

            insert_cell = (
                "INSERT INTO cells("
                "site_id, ix, iy, iz, center_x, center_y, center_z, "
                "occupied_log_odds, free_log_odds, positive_observation_count, free_observation_count, "
                "first_seen_unix_ns, last_seen_unix_ns, last_confirmed_occupied_unix_ns, last_observed_free_unix_ns, "
                "last_source_kind, last_source_provider"
                ") VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?) "
                "ON CONFLICT(site_id, ix, iy, iz) DO UPDATE SET "
                "center_x=excluded.center_x, center_y=excluded.center_y, center_z=excluded.center_z, "
                "occupied_log_odds=excluded.occupied_log_odds, free_log_odds=excluded.free_log_odds, "
                "positive_observation_count=excluded.positive_observation_count, free_observation_count=excluded.free_observation_count, "
                "first_seen_unix_ns=excluded.first_seen_unix_ns, last_seen_unix_ns=excluded.last_seen_unix_ns, "
                "last_confirmed_occupied_unix_ns=excluded.last_confirmed_occupied_unix_ns, "
                "last_observed_free_unix_ns=excluded.last_observed_free_unix_ns, "
                "last_source_kind=excluded.last_source_kind, last_source_provider=excluded.last_source_provider"
            )

            insert_score = (
                "INSERT INTO derived_scores("
                "site_id, ix, iy, iz, occupancy_score, freshness_score, active_score, "
                "cell_age_seconds, site_staleness_seconds, relative_gap_seconds, site_relative_age_percentile, "
                "status, scored_at_unix_ns"
                ") VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?) "
                "ON CONFLICT(site_id, ix, iy, iz) DO UPDATE SET "
                "occupancy_score=excluded.occupancy_score, freshness_score=excluded.freshness_score, "
                "active_score=excluded.active_score, cell_age_seconds=excluded.cell_age_seconds, "
                "site_staleness_seconds=excluded.site_staleness_seconds, relative_gap_seconds=excluded.relative_gap_seconds, "
                "site_relative_age_percentile=excluded.site_relative_age_percentile, status=excluded.status, "
                "scored_at_unix_ns=excluded.scored_at_unix_ns"
            )

            for i, cell in enumerate(cells, start=1):
                if not isinstance(cell, dict):
                    continue

                sid, ix, iy, iz = cell_key_from_json(
                    cell,
                    site_id=site_id,
                    cell_size_m=cell_size_m,
                    vertical_cell_size_m=vertical_cell_size_m,
                )
                center = parse_point(cell.get("center_site") or cell.get("center_mission") or cell.get("center"))
                source_kind, source_provider = source_from_json(cell)
                occupied_log_odds = occupied_log_odds_from_json(cell)

                positive_count = as_int(cell.get("positive_observation_count"))
                if positive_count <= 0 and (as_float(cell.get("max_raw_occupied_score")) > 0 or cell.get("occupied") is True):
                    positive_count = 1
                if positive_count <= 0 and occupied_log_odds > 0:
                    positive_count = 1

                first_seen = as_int(cell.get("first_seen_unix_ns"))
                last_seen = as_int(cell.get("last_seen_unix_ns"))
                last_occ = as_int(cell.get("last_confirmed_occupied_unix_ns"), last_seen)
                last_free = as_int(cell.get("last_observed_free_unix_ns"))

                conn.execute(
                    insert_cell,
                    (
                        sid,
                        ix,
                        iy,
                        iz,
                        center[0],
                        center[1],
                        center[2],
                        occupied_log_odds,
                        as_float(cell.get("free_log_odds")),
                        positive_count,
                        as_int(cell.get("free_observation_count")),
                        first_seen,
                        last_seen,
                        last_occ,
                        last_free,
                        source_kind,
                        source_provider,
                    ),
                )

                derived = cell.get("derived")
                if isinstance(derived, dict):
                    score_summary = data.get("score_summary")
                    scored_at = (
                        as_int(score_summary.get("now_unix_ns"), now_ns)
                        if isinstance(score_summary, dict)
                        else now_ns
                    )
                    conn.execute(
                        insert_score,
                        (
                            sid,
                            ix,
                            iy,
                            iz,
                            as_float(derived.get("occupancy_score")),
                            as_float(derived.get("freshness_score")),
                            as_float(derived.get("active_score")),
                            as_float(derived.get("cell_age_seconds")),
                            as_float(derived.get("site_staleness_seconds")),
                            as_float(derived.get("relative_gap_seconds")),
                            as_float(derived.get("site_relative_age_percentile")),
                            str(derived.get("status") or "probationary"),
                            scored_at,
                        ),
                    )

                source_missions = cell.get("source_missions") or cell.get("mission_ids") or []
                if isinstance(source_missions, list):
                    for mission_id in source_missions:
                        conn.execute(
                            "INSERT OR IGNORE INTO cell_missions(site_id, ix, iy, iz, mission_id) "
                            "VALUES (?, ?, ?, ?, ?)",
                            (sid, ix, iy, iz, str(mission_id)),
                        )

                if i == 1 or i % 10000 == 0 or i == len(cells):
                    print(f"[sqlite-site-map] imported {i}/{len(cells)} cells", flush=True)

        print(f"Wrote {db_path}")
        print(f"Imported cells: {len(cells)}")
        return 0
    except sqlite3.Error as exc:
        print(f"ERROR: sqlite import failed: {exc}")
        return 1
    finally:
        conn.close()


def status_for(
    *,
    occupancy_score: float,
    active_score: float,
    freshness_score: float,
    has_positive_evidence: bool,
    free_after_occupied: bool,
    active_threshold: float,
    stale_threshold: float,
    probationary_threshold: float,
) -> str:
    if free_after_occupied:
        return "suppressed"
    if active_score >= active_threshold:
        return "active"
    if occupancy_score >= stale_threshold and freshness_score < 1.0:
        return "stale"
    if has_positive_evidence and occupancy_score >= probationary_threshold:
        return "probationary"
    if has_positive_evidence:
        return "probationary"
    return "retired"


def score(args: argparse.Namespace) -> int:
    db_path = Path(args.db)
    now_ns = args.now_unix_ns or now_unix_ns()

    conn = connect(db_path)
    if conn is None:
        return 1

    try:
        create_schema(conn)

        site_last_visited = as_int(get_meta(conn, "site_last_visited_unix_ns", 0))
        site_staleness_seconds = max(0.0, (now_ns - site_last_visited) / 1e9) if site_last_visited else 0.0
        half_life_seconds = max(1.0, args.freshness_half_life_days * 86400.0)
        freshness_floor = clamp(args.freshness_floor, 0.0, 1.0)

        rows = conn.execute(
            "SELECT site_id, ix, iy, iz, occupied_log_odds, positive_observation_count, "
            "last_seen_unix_ns, last_confirmed_occupied_unix_ns, last_observed_free_unix_ns "
            "FROM cells"
        ).fetchall()

        ages: list[float] = []
        for row in rows:
            last_seen = as_int(row["last_seen_unix_ns"])
            ages.append(max(0.0, (now_ns - last_seen) / 1e9) if last_seen else 0.0)
        sorted_ages = sorted(ages)

        def percentile(age: float) -> float:
            if not sorted_ages:
                return 0.0
            return bisect.bisect_right(sorted_ages, age) / len(sorted_ages)

        counts = {"active": 0, "stale": 0, "probationary": 0, "suppressed": 0, "retired": 0}
        active_scores: list[float] = []
        occupancy_scores: list[float] = []
        freshness_scores: list[float] = []

        with conn:
            for i, row in enumerate(rows, start=1):
                age_seconds = ages[i - 1]
                relative_gap_seconds = max(0.0, age_seconds - site_staleness_seconds)
                occupancy_score = sigmoid(as_float(row["occupied_log_odds"]))
                freshness_score = math.exp(-relative_gap_seconds / half_life_seconds)
                freshness_score = clamp(freshness_score, freshness_floor, 1.0)
                active_score = occupancy_score * freshness_score

                last_free = as_int(row["last_observed_free_unix_ns"])
                last_occ = as_int(row["last_confirmed_occupied_unix_ns"])
                free_after_occupied = last_free > last_occ and last_free > 0
                if free_after_occupied:
                    active_score *= clamp(args.contradiction_multiplier, 0.0, 1.0)

                has_positive_evidence = as_int(row["positive_observation_count"]) > 0 or occupancy_score > 0.5

                status = status_for(
                    occupancy_score=occupancy_score,
                    active_score=active_score,
                    freshness_score=freshness_score,
                    has_positive_evidence=has_positive_evidence,
                    free_after_occupied=free_after_occupied,
                    active_threshold=args.active_threshold,
                    stale_threshold=args.stale_threshold,
                    probationary_threshold=args.probationary_threshold,
                )
                counts[status] += 1
                active_scores.append(active_score)
                occupancy_scores.append(occupancy_score)
                freshness_scores.append(freshness_score)

                conn.execute(
                    "INSERT INTO derived_scores("
                    "site_id, ix, iy, iz, occupancy_score, freshness_score, active_score, "
                    "cell_age_seconds, site_staleness_seconds, relative_gap_seconds, site_relative_age_percentile, "
                    "status, scored_at_unix_ns"
                    ") VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?) "
                    "ON CONFLICT(site_id, ix, iy, iz) DO UPDATE SET "
                    "occupancy_score=excluded.occupancy_score, freshness_score=excluded.freshness_score, "
                    "active_score=excluded.active_score, cell_age_seconds=excluded.cell_age_seconds, "
                    "site_staleness_seconds=excluded.site_staleness_seconds, relative_gap_seconds=excluded.relative_gap_seconds, "
                    "site_relative_age_percentile=excluded.site_relative_age_percentile, status=excluded.status, "
                    "scored_at_unix_ns=excluded.scored_at_unix_ns",
                    (
                        row["site_id"],
                        row["ix"],
                        row["iy"],
                        row["iz"],
                        occupancy_score,
                        freshness_score,
                        active_score,
                        age_seconds,
                        site_staleness_seconds,
                        relative_gap_seconds,
                        percentile(age_seconds),
                        status,
                        now_ns,
                    ),
                )

                if i == 1 or i % 10000 == 0 or i == len(rows):
                    print(f"[sqlite-site-map] scored {i}/{len(rows)} cells", flush=True)

            score_summary = {
                "now_unix_ns": now_ns,
                "site_staleness_seconds": site_staleness_seconds,
                "cell_count": len(rows),
                "active_cell_count": counts["active"],
                "stale_cell_count": counts["stale"],
                "probationary_cell_count": counts["probationary"],
                "suppressed_cell_count": counts["suppressed"],
                "retired_cell_count": counts["retired"],
                "mean_occupancy_score": sum(occupancy_scores) / len(occupancy_scores) if occupancy_scores else 0.0,
                "mean_freshness_score": sum(freshness_scores) / len(freshness_scores) if freshness_scores else 0.0,
                "mean_active_score": sum(active_scores) / len(active_scores) if active_scores else 0.0,
                "max_active_score": max(active_scores) if active_scores else 0.0,
            }
            set_meta(conn, "updated_at_unix_ns", now_ns)
            set_meta(conn, "score_summary", score_summary)
            set_meta(
                conn,
                "decay_policy",
                {
                    "relative_gap_seconds_formula": "max(0, cell_age_seconds - site_staleness_seconds)",
                    "freshness_half_life_days": args.freshness_half_life_days,
                    "freshness_floor": args.freshness_floor,
                    "active_threshold": args.active_threshold,
                    "stale_threshold": args.stale_threshold,
                    "probationary_threshold": args.probationary_threshold,
                    "contradiction_multiplier": args.contradiction_multiplier,
                    "calendar_age_is_not_erasure": True,
                },
            )

        print(json.dumps(get_meta(conn, "score_summary", {}), indent=2, sort_keys=True))
        return 0
    except sqlite3.Error as exc:
        print(f"ERROR: sqlite scoring failed: {exc}")
        return 1
    finally:
        conn.close()


def summary(args: argparse.Namespace) -> int:
    conn = connect(Path(args.db))
    if conn is None:
        return 1

    try:
        create_schema(conn)
        cell_count = conn.execute("SELECT COUNT(*) AS n FROM cells").fetchone()["n"]
        status_rows = conn.execute(
            "SELECT status, COUNT(*) AS n FROM derived_scores GROUP BY status ORDER BY status"
        ).fetchall()
        db_path = Path(args.db)
        db_summary = {
            "schema": get_meta(conn, "schema", SCHEMA),
            "site_id": get_meta(conn, "site_id", ""),
            "site_frame_id": get_meta(conn, "site_frame_id", ""),
            "cell_count": cell_count,
            "status_counts": {row["status"]: row["n"] for row in status_rows},
            "score_summary": get_meta(conn, "score_summary", {}),
            "db_size_bytes": db_path.stat().st_size if db_path.exists() else 0,
        }
        print(json.dumps(db_summary, indent=2, sort_keys=True))
        return 0
    except sqlite3.Error as exc:
        print(f"ERROR: sqlite summary failed: {exc}")
        return 1
    finally:
        conn.close()


def query_region(args: argparse.Namespace) -> int:
    conn = connect(Path(args.db))
    if conn is None:
        return 1

    try:
        create_schema(conn)

        statuses = args.status or []
        params: list[Any] = [
            args.site_id,
            args.min_ix,
            args.max_ix,
            args.min_iy,
            args.max_iy,
            args.min_iz,
            args.max_iz,
        ]

        status_sql = ""
        if statuses:
            status_sql = " AND COALESCE(s.status, 'probationary') IN ({})".format(
                ",".join("?" for _ in statuses)
            )
            params.extend(statuses)

        limit_sql = ""
        if args.limit and args.limit > 0:
            limit_sql = " LIMIT ?"
            params.append(args.limit)

        rows = conn.execute(
            "SELECT c.*, "
            "COALESCE(s.occupancy_score, 0.0) AS occupancy_score, "
            "COALESCE(s.freshness_score, 0.0) AS freshness_score, "
            "COALESCE(s.active_score, 0.0) AS active_score, "
            "COALESCE(s.status, 'probationary') AS status "
            "FROM cells c "
            "LEFT JOIN derived_scores s "
            "ON c.site_id=s.site_id AND c.ix=s.ix AND c.iy=s.iy AND c.iz=s.iz "
            "WHERE c.site_id=? "
            "AND c.ix BETWEEN ? AND ? "
            "AND c.iy BETWEEN ? AND ? "
            "AND c.iz BETWEEN ? AND ? "
            f"{status_sql} "
            "ORDER BY active_score DESC"
            f"{limit_sql}",
            params,
        ).fetchall()

        out = {
            "schema": "dedalus.site_obstacle_map.query_result.v1",
            "site_id": args.site_id,
            "count": len(rows),
            "cells": [
                {
                    "key": {
                        "site_id": row["site_id"],
                        "ix": row["ix"],
                        "iy": row["iy"],
                        "iz": row["iz"],
                    },
                    "center_site": {
                        "x": row["center_x"],
                        "y": row["center_y"],
                        "z": row["center_z"],
                    },
                    "occupied_log_odds": row["occupied_log_odds"],
                    "free_log_odds": row["free_log_odds"],
                    "positive_observation_count": row["positive_observation_count"],
                    "free_observation_count": row["free_observation_count"],
                    "derived": {
                        "occupancy_score": row["occupancy_score"],
                        "freshness_score": row["freshness_score"],
                        "active_score": row["active_score"],
                        "status": row["status"],
                    },
                }
                for row in rows
            ],
        }

        if args.output:
            if not atomic_write_json(Path(args.output), out):
                return 1
            print(f"Wrote {args.output}")
        else:
            print(json.dumps(out, indent=2, sort_keys=True))

        return 0
    except sqlite3.Error as exc:
        print(f"ERROR: sqlite query failed: {exc}")
        return 1
    finally:
        conn.close()


def export_json(args: argparse.Namespace) -> int:
    conn = connect(Path(args.db))
    if conn is None:
        return 1

    try:
        create_schema(conn)

        rows = conn.execute(
            "SELECT c.*, "
            "COALESCE(s.occupancy_score, 0.0) AS occupancy_score, "
            "COALESCE(s.freshness_score, 0.0) AS freshness_score, "
            "COALESCE(s.active_score, 0.0) AS active_score, "
            "COALESCE(s.cell_age_seconds, 0.0) AS cell_age_seconds, "
            "COALESCE(s.site_staleness_seconds, 0.0) AS site_staleness_seconds, "
            "COALESCE(s.relative_gap_seconds, 0.0) AS relative_gap_seconds, "
            "COALESCE(s.site_relative_age_percentile, 0.0) AS site_relative_age_percentile, "
            "COALESCE(s.status, 'probationary') AS status "
            "FROM cells c "
            "LEFT JOIN derived_scores s "
            "ON c.site_id=s.site_id AND c.ix=s.ix AND c.iy=s.iy AND c.iz=s.iz "
            "ORDER BY c.site_id, c.ix, c.iy, c.iz"
        ).fetchall()

        data = {
            "schema": DEBUG_JSON_SCHEMA,
            "debug_export_source": SCHEMA,
            "time_unit": TIME_UNIT,
            "site_id": get_meta(conn, "site_id", ""),
            "site_frame_id": get_meta(conn, "site_frame_id", ""),
            "cell_size_m": get_meta(conn, "cell_size_m", 0.0),
            "vertical_cell_size_m": get_meta(conn, "vertical_cell_size_m", 0.0),
            "created_at_unix_ns": get_meta(conn, "created_at_unix_ns", 0),
            "updated_at_unix_ns": get_meta(conn, "updated_at_unix_ns", 0),
            "site_last_visited_unix_ns": get_meta(conn, "site_last_visited_unix_ns", 0),
            "merge_summary": get_meta(conn, "merge_summary", {}),
            "score_summary": get_meta(conn, "score_summary", {}),
            "decay_policy": get_meta(conn, "decay_policy", {}),
            "cells": [],
        }

        for row in rows:
            data["cells"].append(
                {
                    "key": {
                        "site_id": row["site_id"],
                        "ix": row["ix"],
                        "iy": row["iy"],
                        "iz": row["iz"],
                    },
                    "center_site": {
                        "x": row["center_x"],
                        "y": row["center_y"],
                        "z": row["center_z"],
                    },
                    "occupied_log_odds": row["occupied_log_odds"],
                    "free_log_odds": row["free_log_odds"],
                    "positive_observation_count": row["positive_observation_count"],
                    "free_observation_count": row["free_observation_count"],
                    "first_seen_unix_ns": row["first_seen_unix_ns"],
                    "last_seen_unix_ns": row["last_seen_unix_ns"],
                    "last_confirmed_occupied_unix_ns": row["last_confirmed_occupied_unix_ns"],
                    "last_observed_free_unix_ns": row["last_observed_free_unix_ns"],
                    "source": {
                        "last_source_kind": row["last_source_kind"] or "",
                        "last_source_provider": row["last_source_provider"] or "",
                    },
                    "derived": {
                        "occupancy_score": row["occupancy_score"],
                        "freshness_score": row["freshness_score"],
                        "active_score": row["active_score"],
                        "cell_age_seconds": row["cell_age_seconds"],
                        "site_staleness_seconds": row["site_staleness_seconds"],
                        "relative_gap_seconds": row["relative_gap_seconds"],
                        "site_relative_age_percentile": row["site_relative_age_percentile"],
                        "status": row["status"],
                    },
                }
            )

        if not atomic_write_json(Path(args.output), data):
            return 1

        print(f"Wrote {args.output}")
        print(f"Exported cells: {len(rows)}")
        return 0
    except sqlite3.Error as exc:
        print(f"ERROR: sqlite export failed: {exc}")
        return 1
    finally:
        conn.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    p_import = sub.add_parser("import-json", help="Import debug JSON site map into SQLite")
    p_import.add_argument("input")
    p_import.add_argument("--db", required=True)
    p_import.add_argument("--replace", action="store_true")
    p_import.add_argument("--site-id")
    p_import.add_argument("--site-frame-id")
    p_import.add_argument("--cell-size-m", type=float, default=0.5)
    p_import.add_argument("--vertical-cell-size-m", type=float, default=0.5)
    p_import.set_defaults(func=import_json)

    p_score = sub.add_parser("score", help="Recompute derived scores in SQLite")
    p_score.add_argument("--db", required=True)
    p_score.add_argument("--now-unix-ns", type=int)
    p_score.add_argument("--freshness-half-life-days", type=float, default=30.0)
    p_score.add_argument("--freshness-floor", type=float, default=0.35)
    p_score.add_argument("--active-threshold", type=float, default=0.55)
    p_score.add_argument("--stale-threshold", type=float, default=0.55)
    p_score.add_argument("--probationary-threshold", type=float, default=0.05)
    p_score.add_argument("--contradiction-multiplier", type=float, default=0.25)
    p_score.set_defaults(func=score)

    p_summary = sub.add_parser("summary", help="Print SQLite site map summary")
    p_summary.add_argument("--db", required=True)
    p_summary.set_defaults(func=summary)

    p_query = sub.add_parser("query-region", help="Query a cell-index region")
    p_query.add_argument("--db", required=True)
    p_query.add_argument("--site-id", required=True)
    p_query.add_argument("--min-ix", type=int, required=True)
    p_query.add_argument("--max-ix", type=int, required=True)
    p_query.add_argument("--min-iy", type=int, required=True)
    p_query.add_argument("--max-iy", type=int, required=True)
    p_query.add_argument("--min-iz", type=int, required=True)
    p_query.add_argument("--max-iz", type=int, required=True)
    p_query.add_argument(
        "--status",
        action="append",
        choices=["active", "probationary", "stale", "suppressed", "retired"],
    )
    p_query.add_argument("--limit", type=int, default=100)
    p_query.add_argument("--output")
    p_query.set_defaults(func=query_region)

    p_export = sub.add_parser("export-json", help="Export SQLite site map to debug JSON")
    p_export.add_argument("--db", required=True)
    p_export.add_argument("--output", required=True)
    p_export.set_defaults(func=export_json)

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    main()
