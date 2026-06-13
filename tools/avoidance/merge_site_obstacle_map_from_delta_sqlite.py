#!/usr/bin/env python3
"""Merge compact mission obstacle delta SQLite logs into persistent site SQLite.

5O: Tier-B compact delta SQLite -> Tier-C persistent site SQLite.

This is an offline/post-mission compaction step. It does not touch the flight
hot path and does not require the full mission_obstacle_map_full.json artifact.
"""

from __future__ import annotations

import argparse
import json
import math
import sqlite3
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any


SITE_SCHEMA = "dedalus.site_obstacle_map.sqlite.v1"
TIME_UNIT = "unix_ns"


@dataclass
class SiteCellAggregate:
    site_id: str
    ix: int
    iy: int
    iz: int
    center_x: float
    center_y: float
    center_z: float
    max_occupied_score: float = 0.0
    max_free_score: float = 0.0
    confidence: float = 0.0
    positive_observation_count: int = 0
    free_observation_count: int = 0
    first_seen_unix_ns: int = 0
    last_seen_unix_ns: int = 0
    last_confirmed_occupied_unix_ns: int = 0
    last_observed_free_unix_ns: int = 0
    last_source_kind: str = ""
    last_source_provider: str = ""


def now_unix_ns() -> int:
    return time.time_ns()


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def logit(probability: float) -> float:
    p = clamp(probability, 1.0e-6, 1.0 - 1.0e-6)
    return math.log(p / (1.0 - p))


def connect(path: Path) -> sqlite3.Connection:
    path.parent.mkdir(parents=True, exist_ok=True)
    conn = sqlite3.connect(str(path))
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA journal_mode=WAL")
    conn.execute("PRAGMA synchronous=NORMAL")
    conn.execute("PRAGMA temp_store=MEMORY")
    conn.execute("PRAGMA foreign_keys=ON")
    return conn


def create_site_schema(conn: sqlite3.Connection) -> None:
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


def read_delta_meta(conn: sqlite3.Connection) -> dict[str, Any]:
    rows = conn.execute("SELECT key, value FROM metadata").fetchall()
    out: dict[str, Any] = {}
    for row in rows:
        try:
            out[str(row["key"])] = json.loads(row["value"])
        except json.JSONDecodeError:
            out[str(row["key"])] = row["value"]
    return out


def round_cell_key(
    *,
    site_id: str,
    x: float,
    y: float,
    z: float,
    cell_size_m: float,
    vertical_cell_size_m: float,
) -> tuple[str, int, int, int]:
    cs = cell_size_m if cell_size_m > 0.0 else 0.5
    vz = vertical_cell_size_m if vertical_cell_size_m > 0.0 else cs
    return (site_id, round(x / cs), round(y / cs), round(z / vz))


def aggregate_delta_cells(
    delta_conn: sqlite3.Connection,
    *,
    site_id: str,
    cell_size_m: float,
    vertical_cell_size_m: float,
) -> tuple[dict[tuple[str, int, int, int], SiteCellAggregate], list[sqlite3.Row]]:
    mission_rows = delta_conn.execute(
        """
        SELECT
          mission_id,
          site_id AS delta_site_id,
          site_frame_id,
          mission_map_frame_id,
          MIN(mission_start_unix_ns) AS mission_start_unix_ns,
          MAX(batch_unix_ns) AS mission_end_unix_ns,
          MAX(update_count) AS last_update_count,
          COUNT(*) AS batch_count,
          COALESCE(SUM(changed_cell_count), 0) AS declared_changed_cells
        FROM batches
        GROUP BY mission_id, delta_site_id, site_frame_id, mission_map_frame_id
        ORDER BY mission_id
        """
    ).fetchall()

    rows = delta_conn.execute(
        """
        SELECT
          b.mission_id,
          b.batch_unix_ns,
          b.update_count,
          c.center_x,
          c.center_y,
          c.center_z,
          c.occupied_score,
          c.free_score,
          c.confidence,
          c.first_seen_unix_ns,
          c.last_seen_unix_ns,
          c.source_kind,
          c.source_provider
        FROM cells c
        JOIN batches b ON b.batch_id = c.batch_id
        ORDER BY b.batch_unix_ns, b.update_count, c.cell_index
        """
    ).fetchall()

    aggregates: dict[tuple[str, int, int, int], SiteCellAggregate] = {}

    for row in rows:
        x = float(row["center_x"])
        y = float(row["center_y"])
        z = float(row["center_z"])
        key = round_cell_key(
            site_id=site_id,
            x=x,
            y=y,
            z=z,
            cell_size_m=cell_size_m,
            vertical_cell_size_m=vertical_cell_size_m,
        )

        _, ix, iy, iz = key
        center_x = ix * cell_size_m
        center_y = iy * cell_size_m
        center_z = iz * vertical_cell_size_m

        agg = aggregates.get(key)
        if agg is None:
            agg = SiteCellAggregate(
                site_id=site_id,
                ix=ix,
                iy=iy,
                iz=iz,
                center_x=center_x,
                center_y=center_y,
                center_z=center_z,
            )
            aggregates[key] = agg

        occupied_score = float(row["occupied_score"] or 0.0)
        free_score = float(row["free_score"] or 0.0)
        confidence = float(row["confidence"] or 0.0)
        first_seen = int(row["first_seen_unix_ns"] or 0)
        last_seen = int(row["last_seen_unix_ns"] or row["batch_unix_ns"] or 0)

        agg.max_occupied_score = max(agg.max_occupied_score, occupied_score)
        agg.max_free_score = max(agg.max_free_score, free_score)
        agg.confidence = max(agg.confidence, confidence)

        if occupied_score > 0.0:
            agg.positive_observation_count += 1
            agg.last_confirmed_occupied_unix_ns = max(agg.last_confirmed_occupied_unix_ns, last_seen)

        if free_score > 0.0:
            agg.free_observation_count += 1
            agg.last_observed_free_unix_ns = max(agg.last_observed_free_unix_ns, last_seen)

        if agg.first_seen_unix_ns == 0 or (first_seen > 0 and first_seen < agg.first_seen_unix_ns):
            agg.first_seen_unix_ns = first_seen
        agg.last_seen_unix_ns = max(agg.last_seen_unix_ns, last_seen)

        if last_seen >= agg.last_seen_unix_ns:
            agg.last_source_kind = str(row["source_kind"] or "")
            agg.last_source_provider = str(row["source_provider"] or "")

    return aggregates, mission_rows


def remove_existing_site_db(db_path: Path) -> None:
    for path in (db_path, Path(str(db_path) + "-wal"), Path(str(db_path) + "-shm")):
        if path.exists():
            path.unlink()


def already_merged(site_conn: sqlite3.Connection, mission_ids: list[str]) -> bool:
    if not mission_ids:
        return False
    rows = site_conn.execute(
        "SELECT mission_id FROM missions WHERE mission_id IN ({})".format(
            ",".join("?" for _ in mission_ids)
        ),
        mission_ids,
    ).fetchall()
    return {str(row["mission_id"]) for row in rows} == set(mission_ids)


def run_site_score(site_db: Path, repo_root: Path) -> None:
    tool = repo_root / "tools" / "avoidance" / "site_obstacle_map_sqlite.py"
    if not tool.exists():
        print(f"warning: missing scoring tool; derived_scores not recomputed: {tool}", flush=True)
        return

    subprocess.run(
        [sys.executable, str(tool), "score", "--db", str(site_db)],
        check=False,
    )


def merge(args: argparse.Namespace) -> int:
    repo_root = Path(__file__).resolve().parents[2]
    delta_db = Path(args.delta_db)
    site_db = Path(args.site_db)

    if not delta_db.exists():
        print(f"ERROR: delta SQLite database does not exist: {delta_db}")
        return 1

    if args.replace:
        try:
            remove_existing_site_db(site_db)
        except OSError as exc:
            print(f"ERROR: failed removing existing site database {site_db}: {exc}")
            return 1

    delta_conn = connect(delta_db)
    site_conn = connect(site_db)

    try:
        create_site_schema(site_conn)
        delta_meta = read_delta_meta(delta_conn)

        site_id = args.site_id or str(delta_meta.get("site_id") or "unknown_site")
        site_frame_id = args.site_frame_id or str(delta_meta.get("site_frame_id") or "")
        cell_size_m = float(args.cell_size_m or delta_meta.get("cell_size_m") or 0.5)
        vertical_cell_size_m = float(
            args.vertical_cell_size_m or delta_meta.get("vertical_cell_size_m") or cell_size_m
        )
        merged_at = now_unix_ns()

        aggregates, mission_rows = aggregate_delta_cells(
            delta_conn,
            site_id=site_id,
            cell_size_m=cell_size_m,
            vertical_cell_size_m=vertical_cell_size_m,
        )

        mission_ids = [str(row["mission_id"]) for row in mission_rows]
        if not args.replace and already_merged(site_conn, mission_ids):
            result = {
                "site_db": str(site_db),
                "delta_db": str(delta_db),
                "site_id": site_id,
                "skipped_already_merged": True,
                "mission_ids": mission_ids,
                "merged_cell_count": 0,
            }
            print(json.dumps(result, indent=2, sort_keys=True))
            return 0

        max_occupied = max((cell.max_occupied_score for cell in aggregates.values()), default=1.0)
        max_free = max((cell.max_free_score for cell in aggregates.values()), default=1.0)
        max_occupied = max(max_occupied, 1.0)
        max_free = max(max_free, 1.0)

        with site_conn:
            set_meta(site_conn, "schema", SITE_SCHEMA)
            set_meta(site_conn, "site_id", site_id)
            set_meta(site_conn, "site_frame_id", site_frame_id)
            set_meta(site_conn, "time_unit", TIME_UNIT)
            set_meta(site_conn, "cell_size_m", cell_size_m)
            set_meta(site_conn, "vertical_cell_size_m", vertical_cell_size_m)
            set_meta(site_conn, "updated_at_unix_ns", merged_at)
            set_meta(site_conn, "site_last_visited_unix_ns", max((int(r["mission_end_unix_ns"] or 0) for r in mission_rows), default=0))

            for row in mission_rows:
                site_conn.execute(
                    """
                    INSERT INTO missions(
                      mission_id,
                      site_id,
                      mission_map_frame_id,
                      mission_start_unix_ns,
                      mission_end_unix_ns,
                      merged_at_unix_ns,
                      source_artifact
                    ) VALUES (?, ?, ?, ?, ?, ?, ?)
                    ON CONFLICT(mission_id) DO UPDATE SET
                      site_id=excluded.site_id,
                      mission_map_frame_id=excluded.mission_map_frame_id,
                      mission_start_unix_ns=excluded.mission_start_unix_ns,
                      mission_end_unix_ns=excluded.mission_end_unix_ns,
                      merged_at_unix_ns=excluded.merged_at_unix_ns,
                      source_artifact=excluded.source_artifact
                    """,
                    (
                        str(row["mission_id"]),
                        site_id,
                        str(row["mission_map_frame_id"] or ""),
                        int(row["mission_start_unix_ns"] or 0),
                        int(row["mission_end_unix_ns"] or 0),
                        merged_at,
                        str(delta_db),
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
                "positive_observation_count=excluded.positive_observation_count, "
                "free_observation_count=excluded.free_observation_count, "
                "first_seen_unix_ns=MIN(cells.first_seen_unix_ns, excluded.first_seen_unix_ns), "
                "last_seen_unix_ns=MAX(cells.last_seen_unix_ns, excluded.last_seen_unix_ns), "
                "last_confirmed_occupied_unix_ns=MAX(cells.last_confirmed_occupied_unix_ns, excluded.last_confirmed_occupied_unix_ns), "
                "last_observed_free_unix_ns=MAX(cells.last_observed_free_unix_ns, excluded.last_observed_free_unix_ns), "
                "last_source_kind=excluded.last_source_kind, last_source_provider=excluded.last_source_provider"
            )

            sorted_cells = sorted(aggregates.values(), key=lambda c: (c.ix, c.iy, c.iz))
            for i, cell in enumerate(sorted_cells, start=1):
                occupied_probability = clamp(cell.max_occupied_score / max_occupied, 1.0e-6, 1.0 - 1.0e-6)
                free_probability = clamp(cell.max_free_score / max_free, 1.0e-6, 1.0 - 1.0e-6)

                site_conn.execute(
                    insert_cell,
                    (
                        cell.site_id,
                        cell.ix,
                        cell.iy,
                        cell.iz,
                        cell.center_x,
                        cell.center_y,
                        cell.center_z,
                        logit(occupied_probability),
                        logit(free_probability),
                        cell.positive_observation_count,
                        cell.free_observation_count,
                        cell.first_seen_unix_ns,
                        cell.last_seen_unix_ns,
                        cell.last_confirmed_occupied_unix_ns,
                        cell.last_observed_free_unix_ns,
                        cell.last_source_kind,
                        cell.last_source_provider,
                    ),
                )

                for mission_id in mission_ids:
                    site_conn.execute(
                        """
                        INSERT OR IGNORE INTO cell_missions(site_id, ix, iy, iz, mission_id)
                        VALUES (?, ?, ?, ?, ?)
                        """,
                        (cell.site_id, cell.ix, cell.iy, cell.iz, mission_id),
                    )

                if i == 1 or i % 10000 == 0 or i == len(sorted_cells):
                    print(f"[delta->site-sqlite] merged {i}/{len(sorted_cells)} site cells", flush=True)

            merge_summary = {
                "source": "delta_sqlite",
                "source_delta_db": str(delta_db),
                "merged_mission_ids": mission_ids,
                "mission_merge_count": len(mission_ids),
                "input_delta_cell_count": int(sum(int(r["declared_changed_cells"] or 0) for r in mission_rows)),
                "merged_site_cell_count": len(sorted_cells),
                "merged_at_unix_ns": merged_at,
                "cell_size_m": cell_size_m,
                "vertical_cell_size_m": vertical_cell_size_m,
                "score_normalization": {
                    "max_occupied_score": max_occupied,
                    "max_free_score": max_free,
                    "probability": "clamp(max_raw_score / max_raw_score_over_delta_db, 1e-6, 1-1e-6)",
                },
            }
            set_meta(site_conn, "merge_summary", merge_summary)

        if not args.no_score:
            run_site_score(site_db, repo_root)

        result = {
            "site_db": str(site_db),
            "delta_db": str(delta_db),
            "site_id": site_id,
            "site_frame_id": site_frame_id,
            "mission_ids": mission_ids,
            "input_delta_cell_count": int(sum(int(r["declared_changed_cells"] or 0) for r in mission_rows)),
            "merged_site_cell_count": len(aggregates),
            "site_db_size_bytes": site_db.stat().st_size if site_db.exists() else 0,
        }
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0
    except Exception as exc:
        print(f"ERROR: {exc}")
        return 1
    finally:
        delta_conn.close()
        site_conn.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Merge compact mission delta SQLite logs into persistent site SQLite."
    )
    parser.add_argument("--delta-db", required=True, help="Input mission_obstacle_map_deltas.sqlite")
    parser.add_argument("--site-db", required=True, help="Output/target site_obstacle_map.sqlite")
    parser.add_argument("--site-id", default="", help="Persistent site id.")
    parser.add_argument("--site-frame-id", default="", help="Persistent site frame id.")
    parser.add_argument("--cell-size-m", type=float, default=0.0)
    parser.add_argument("--vertical-cell-size-m", type=float, default=0.0)
    parser.add_argument("--replace", action="store_true", help="Replace existing site database.")
    parser.add_argument("--no-score", action="store_true", help="Do not recompute derived_scores after merge.")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    return merge(args)


if __name__ == "__main__":
    main()
