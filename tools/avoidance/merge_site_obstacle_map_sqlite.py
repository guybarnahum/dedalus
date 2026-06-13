#!/usr/bin/env python3
"""Merge mission obstacle map artifacts directly into SQLite site memory.

5L.2: efficient post-mission merge path.

This tool consumes a full debug mission artifact:

    out/<run>/mission_obstacle_map_full.json

and updates the efficient persistent store:

    maps/<site_id>/site_obstacle_map.sqlite

It intentionally does not update the large debug JSON site map. Use
site_obstacle_map_sqlite.py export-json when a human-readable artifact is needed.
"""

from __future__ import annotations

import argparse
import json
import math
import sqlite3
import time
from pathlib import Path
from typing import Any

import site_obstacle_map_sqlite as store


MISSION_SCHEMA = "dedalus.mission_obstacle_map.v1"
SQLITE_SCHEMA = "dedalus.site_obstacle_map.sqlite.v1"
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


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def as_int(value: Any, default: int = 0) -> int:
    return store.as_int(value, default)


def as_float(value: Any, default: float = 0.0) -> float:
    return store.as_float(value, default)


def logit(probability: float) -> float:
    p = clamp(probability, 1e-6, 1.0 - 1e-6)
    return math.log(p / (1.0 - p))


def occupied_log_odds_from_mission_cell(cell: dict[str, Any]) -> float:
    if "occupied_log_odds" in cell:
        return as_float(cell.get("occupied_log_odds"))

    if "occupancy_score" in cell:
        return logit(as_float(cell.get("occupancy_score")))

    derived = cell.get("derived")
    if isinstance(derived, dict):
        for key in ("occupancy_score", "active_score", "persistent_score"):
            if key in derived:
                return logit(as_float(derived.get(key)))

    if "normalized_occupied_score" in cell:
        return logit(as_float(cell.get("normalized_occupied_score")))

    score_scale = as_float(cell.get("score_scale"), 20.0)
    if score_scale > 0 and "occupied_score" in cell:
        return logit(clamp(as_float(cell.get("occupied_score")) / score_scale, 0.0, 1.0))

    if cell.get("occupied") is True:
        return logit(0.75)

    return 0.0


def free_log_odds_from_mission_cell(cell: dict[str, Any]) -> float:
    if "free_log_odds" in cell:
        return as_float(cell.get("free_log_odds"))
    return 0.0


def parse_center_site(cell: dict[str, Any]) -> tuple[float, float, float]:
    # 5L.2 assumes identity site_T_mission for AirSim validation. The contract
    # keeps site storage separate so a future 5M can add explicit transforms.
    return store.parse_point(
        cell.get("center_site")
        or cell.get("center_mission")
        or cell.get("center_map")
        or cell.get("center")
    )


def key_from_center(
    *,
    site_id: str,
    center: tuple[float, float, float],
    cell_size_m: float,
    vertical_cell_size_m: float,
) -> tuple[str, int, int, int]:
    cs = cell_size_m if cell_size_m > 0 else 0.5
    vz = vertical_cell_size_m if vertical_cell_size_m > 0 else cs
    return (
        site_id,
        round(center[0] / cs),
        round(center[1] / cs),
        round(center[2] / vz),
    )


def cell_key_from_mission_cell(
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

    center = parse_center_site(cell)
    return key_from_center(
        site_id=site_id,
        center=center,
        cell_size_m=cell_size_m,
        vertical_cell_size_m=vertical_cell_size_m,
    )


def source_from_mission_cell(cell: dict[str, Any]) -> tuple[str, str]:
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


def existing_merged_missions(conn: sqlite3.Connection) -> set[str]:
    rows = conn.execute("SELECT mission_id FROM missions").fetchall()
    return {str(row["mission_id"]) for row in rows}


def merged_mission_ids_from_meta(conn: sqlite3.Connection) -> list[str]:
    merge_summary = store.get_meta(conn, "merge_summary", {})
    if isinstance(merge_summary, dict):
        ids = merge_summary.get("merged_mission_ids", [])
        if isinstance(ids, list):
            return [str(x) for x in ids]
    return []


def set_merge_summary(
    conn: sqlite3.Connection,
    *,
    mission_id: str,
    merged_cell_count: int,
    mission_end_unix_ns: int,
) -> None:
    existing_ids = set(merged_mission_ids_from_meta(conn))
    existing_ids.update(existing_merged_missions(conn))
    existing_ids.add(mission_id)
    merged_ids = sorted(existing_ids)

    summary = {
        "last_merged_cell_count": merged_cell_count,
        "last_merged_mission_end_unix_ns": mission_end_unix_ns,
        "last_merged_mission_id": mission_id,
        "merged_mission_ids": merged_ids,
        "mission_merge_count": len(merged_ids),
    }
    store.set_meta(conn, "merge_summary", summary)


def merge_one_mission(args: argparse.Namespace, conn: sqlite3.Connection, mission_path: Path) -> tuple[int, bool]:
    data = load_json(mission_path)
    if data is None:
        return (0, False)

    if data.get("schema") != MISSION_SCHEMA:
        print(f"warning: importing non-standard mission schema from {mission_path}: {data.get('schema')}", flush=True)

    cells = data.get("cells")
    if not isinstance(cells, list):
        print(f"ERROR: mission artifact has no cells array: {mission_path}")
        return (0, False)

    site_id = str(args.site_id or data.get("site_id") or "unknown_site")
    site_frame_id = str(args.site_frame_id or data.get("site_frame_id") or "")
    mission_id = str(args.mission_id or data.get("mission_id") or mission_path.parent.name or mission_path.stem)
    mission_map_frame_id = str(data.get("mission_map_frame_id") or "")
    mission_start_unix_ns = as_int(data.get("mission_start_unix_ns"))
    mission_end_unix_ns = as_int(data.get("mission_end_unix_ns"), mission_start_unix_ns)
    cell_size_m = as_float(args.cell_size_m, as_float(data.get("cell_size_m"), 0.5))
    vertical_cell_size_m = as_float(args.vertical_cell_size_m, as_float(data.get("vertical_cell_size_m"), cell_size_m))
    merged_at_unix_ns = now_unix_ns()

    if mission_id in existing_merged_missions(conn):
        print(f"[sqlite-site-map] skipping already-merged mission {mission_id}", flush=True)
        return (0, True)

    print(f"[sqlite-site-map] merging mission {mission_id}: {len(cells)} cells", flush=True)

    with conn:
        store.set_meta(conn, "schema", SQLITE_SCHEMA)
        store.set_meta(conn, "site_id", site_id)
        store.set_meta(conn, "site_frame_id", site_frame_id)
        store.set_meta(conn, "time_unit", TIME_UNIT)
        store.set_meta(conn, "cell_size_m", cell_size_m)
        store.set_meta(conn, "vertical_cell_size_m", vertical_cell_size_m)
        store.set_meta(conn, "updated_at_unix_ns", merged_at_unix_ns)

        if store.get_meta(conn, "created_at_unix_ns", 0) == 0:
            store.set_meta(conn, "created_at_unix_ns", merged_at_unix_ns)

        prev_last_visited = as_int(store.get_meta(conn, "site_last_visited_unix_ns", 0))
        store.set_meta(conn, "site_last_visited_unix_ns", max(prev_last_visited, mission_end_unix_ns))

        conn.execute(
            "INSERT INTO missions("
            "mission_id, site_id, mission_map_frame_id, mission_start_unix_ns, "
            "mission_end_unix_ns, merged_at_unix_ns, source_artifact"
            ") VALUES (?, ?, ?, ?, ?, ?, ?)",
            (
                mission_id,
                site_id,
                mission_map_frame_id,
                mission_start_unix_ns,
                mission_end_unix_ns,
                merged_at_unix_ns,
                str(mission_path),
            ),
        )

        for i, cell in enumerate(cells, start=1):
            if not isinstance(cell, dict):
                continue

            center = parse_center_site(cell)
            sid, ix, iy, iz = cell_key_from_mission_cell(
                cell,
                site_id=site_id,
                cell_size_m=cell_size_m,
                vertical_cell_size_m=vertical_cell_size_m,
            )
            source_kind, source_provider = source_from_mission_cell(cell)

            occupied_log_odds = occupied_log_odds_from_mission_cell(cell)
            free_log_odds = free_log_odds_from_mission_cell(cell)

            positive_count = as_int(cell.get("positive_observation_count"))
            if positive_count <= 0:
                positive_count = as_int(cell.get("mission_observation_count"))
            if positive_count <= 0:
                positive_count = 1 if occupied_log_odds > 0 else 0

            free_count = as_int(cell.get("free_observation_count"))

            first_seen = as_int(cell.get("first_seen_unix_ns"), mission_start_unix_ns)
            last_seen = as_int(cell.get("last_seen_unix_ns"), mission_end_unix_ns)
            last_occ = as_int(cell.get("last_confirmed_occupied_unix_ns"), last_seen)
            last_free = as_int(cell.get("last_observed_free_unix_ns"))

            row = conn.execute(
                "SELECT occupied_log_odds, free_log_odds, positive_observation_count, free_observation_count, "
                "first_seen_unix_ns, last_seen_unix_ns, last_confirmed_occupied_unix_ns, last_observed_free_unix_ns "
                "FROM cells WHERE site_id=? AND ix=? AND iy=? AND iz=?",
                (sid, ix, iy, iz),
            ).fetchone()

            if row is None:
                conn.execute(
                    "INSERT INTO cells("
                    "site_id, ix, iy, iz, center_x, center_y, center_z, "
                    "occupied_log_odds, free_log_odds, positive_observation_count, free_observation_count, "
                    "first_seen_unix_ns, last_seen_unix_ns, last_confirmed_occupied_unix_ns, last_observed_free_unix_ns, "
                    "last_source_kind, last_source_provider"
                    ") VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                    (
                        sid,
                        ix,
                        iy,
                        iz,
                        center[0],
                        center[1],
                        center[2],
                        clamp(occupied_log_odds, -20.0, 20.0),
                        clamp(free_log_odds, -20.0, 20.0),
                        positive_count,
                        free_count,
                        first_seen,
                        last_seen,
                        last_occ,
                        last_free,
                        source_kind,
                        source_provider,
                    ),
                )
            else:
                conn.execute(
                    "UPDATE cells SET "
                    "center_x=?, center_y=?, center_z=?, "
                    "occupied_log_odds=?, free_log_odds=?, "
                    "positive_observation_count=?, free_observation_count=?, "
                    "first_seen_unix_ns=?, last_seen_unix_ns=?, "
                    "last_confirmed_occupied_unix_ns=?, last_observed_free_unix_ns=?, "
                    "last_source_kind=?, last_source_provider=? "
                    "WHERE site_id=? AND ix=? AND iy=? AND iz=?",
                    (
                        center[0],
                        center[1],
                        center[2],
                        clamp(as_float(row["occupied_log_odds"]) + occupied_log_odds, -20.0, 20.0),
                        clamp(as_float(row["free_log_odds"]) + free_log_odds, -20.0, 20.0),
                        as_int(row["positive_observation_count"]) + positive_count,
                        as_int(row["free_observation_count"]) + free_count,
                        min(as_int(row["first_seen_unix_ns"]), first_seen) if as_int(row["first_seen_unix_ns"]) else first_seen,
                        max(as_int(row["last_seen_unix_ns"]), last_seen),
                        max(as_int(row["last_confirmed_occupied_unix_ns"]), last_occ),
                        max(as_int(row["last_observed_free_unix_ns"]), last_free),
                        source_kind,
                        source_provider,
                        sid,
                        ix,
                        iy,
                        iz,
                    ),
                )

            conn.execute(
                "INSERT OR IGNORE INTO cell_missions(site_id, ix, iy, iz, mission_id) "
                "VALUES (?, ?, ?, ?, ?)",
                (sid, ix, iy, iz, mission_id),
            )

            if i == 1 or i % 10000 == 0 or i == len(cells):
                print(f"[sqlite-site-map]   merged {i}/{len(cells)} cells", flush=True)

        set_merge_summary(
            conn,
            mission_id=mission_id,
            merged_cell_count=len(cells),
            mission_end_unix_ns=mission_end_unix_ns,
        )

    return (len(cells), True)


def score_database(db_path: Path, args: argparse.Namespace) -> int:
    score_args = argparse.Namespace(
        db=str(db_path),
        now_unix_ns=args.now_unix_ns,
        freshness_half_life_days=args.freshness_half_life_days,
        freshness_floor=args.freshness_floor,
        active_threshold=args.active_threshold,
        stale_threshold=args.stale_threshold,
        probationary_threshold=args.probationary_threshold,
        contradiction_multiplier=args.contradiction_multiplier,
    )
    return store.score(score_args)


def summary_database(db_path: Path) -> dict[str, Any] | None:
    conn = store.connect(db_path)
    if conn is None:
        return None
    try:
        store.create_schema(conn)
        cell_count = conn.execute("SELECT COUNT(*) AS n FROM cells").fetchone()["n"]
        status_rows = conn.execute(
            "SELECT status, COUNT(*) AS n FROM derived_scores GROUP BY status ORDER BY status"
        ).fetchall()
        return {
            "schema": store.get_meta(conn, "schema", SQLITE_SCHEMA),
            "site_id": store.get_meta(conn, "site_id", ""),
            "site_frame_id": store.get_meta(conn, "site_frame_id", ""),
            "cell_count": cell_count,
            "status_counts": {row["status"]: row["n"] for row in status_rows},
            "merge_summary": store.get_meta(conn, "merge_summary", {}),
            "score_summary": store.get_meta(conn, "score_summary", {}),
            "db_size_bytes": db_path.stat().st_size if db_path.exists() else 0,
        }
    except sqlite3.Error as exc:
        print(f"ERROR: sqlite summary failed: {exc}")
        return None
    finally:
        conn.close()


def merge(args: argparse.Namespace) -> int:
    db_path = Path(args.db)
    mission_paths = [Path(path) for path in args.mission_maps]

    conn = store.connect(db_path)
    if conn is None:
        return 1

    try:
        store.create_schema(conn)
        total_input_cells = 0

        for path in mission_paths:
            merged_count, ok = merge_one_mission(args, conn, path)
            if not ok:
                return 1
            total_input_cells += merged_count

        if not args.no_score:
            score_result = score_database(db_path, args)
            if score_result != 0:
                return score_result

        summary = summary_database(db_path)
        if summary is not None:
            print(json.dumps(summary, indent=2, sort_keys=True))

        print(f"Wrote {db_path}")
        print(f"Input mission maps: {len(mission_paths)}")
        print(f"Newly merged input cells: {total_input_cells}")
        return 0
    except sqlite3.Error as exc:
        print(f"ERROR: sqlite merge failed: {exc}")
        return 1
    finally:
        conn.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mission_maps", nargs="+", help="Full mission obstacle map JSON artifact(s)")
    parser.add_argument("--db", required=True, help="SQLite site map path")
    parser.add_argument("--site-id", default=None)
    parser.add_argument("--site-frame-id", default=None)
    parser.add_argument("--mission-id", default=None, help="Override mission id; only valid for one input artifact")
    parser.add_argument("--cell-size-m", type=float, default=0.5)
    parser.add_argument("--vertical-cell-size-m", type=float, default=0.5)
    parser.add_argument("--no-score", action="store_true", help="Skip derived score recomputation after merge")
    parser.add_argument("--now-unix-ns", type=int, default=None)
    parser.add_argument("--freshness-half-life-days", type=float, default=30.0)
    parser.add_argument("--freshness-floor", type=float, default=0.35)
    parser.add_argument("--active-threshold", type=float, default=0.55)
    parser.add_argument("--stale-threshold", type=float, default=0.55)
    parser.add_argument("--probationary-threshold", type=float, default=0.05)
    parser.add_argument("--contradiction-multiplier", type=float, default=0.25)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    if args.mission_id and len(args.mission_maps) != 1:
        return error("--mission-id override is only valid with one input artifact")

    return merge(args)


if __name__ == "__main__":
    main()
