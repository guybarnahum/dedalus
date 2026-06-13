#!/usr/bin/env python3
"""Import and query compact mission obstacle delta JSONL streams in SQLite.

This is a Tier-B persistence/log utility. It stores the append-only mission
delta stream in a compact, queryable SQLite database without mutating the
persistent site map.

Input:
  mission_obstacle_map_deltas.jsonl
  mission_obstacle_map_deltas.jsonl.meta.json

Output:
  mission_obstacle_map_deltas.sqlite
"""

from __future__ import annotations

import argparse
import json
import sqlite3
from pathlib import Path
from typing import Any, Iterable


SCHEMA_VERSION = "dedalus.mission_obstacle_delta_sqlite.v1"


def connect(db_path: Path) -> sqlite3.Connection:
    db_path.parent.mkdir(parents=True, exist_ok=True)
    conn = sqlite3.connect(str(db_path))
    conn.execute("PRAGMA journal_mode=WAL")
    conn.execute("PRAGMA synchronous=NORMAL")
    conn.execute("PRAGMA foreign_keys=ON")
    return conn


def init_schema(conn: sqlite3.Connection) -> None:
    conn.executescript(
        """
        CREATE TABLE IF NOT EXISTS metadata (
            key TEXT PRIMARY KEY,
            value TEXT NOT NULL
        ) WITHOUT ROWID;

        CREATE TABLE IF NOT EXISTS batches (
            batch_id INTEGER PRIMARY KEY AUTOINCREMENT,
            schema_name TEXT NOT NULL,
            time_unit TEXT NOT NULL,
            site_id TEXT NOT NULL,
            site_frame_id TEXT NOT NULL,
            mission_id TEXT NOT NULL,
            mission_map_frame_id TEXT NOT NULL,
            mission_start_unix_ns INTEGER NOT NULL,
            batch_unix_ns INTEGER NOT NULL,
            update_count INTEGER NOT NULL,
            previous_snapshot_timestamp_ns INTEGER NOT NULL,
            changed_cell_count INTEGER NOT NULL,
            UNIQUE(mission_id, update_count, batch_unix_ns)
        );

        CREATE TABLE IF NOT EXISTS cells (
            batch_id INTEGER NOT NULL,
            cell_index INTEGER NOT NULL,
            center_x REAL NOT NULL,
            center_y REAL NOT NULL,
            center_z REAL NOT NULL,
            occupied_score REAL NOT NULL,
            free_score REAL NOT NULL,
            confidence REAL NOT NULL,
            first_seen_unix_ns INTEGER NOT NULL,
            last_seen_unix_ns INTEGER NOT NULL,
            source_kind TEXT NOT NULL,
            source_provider TEXT NOT NULL,
            PRIMARY KEY(batch_id, cell_index),
            FOREIGN KEY(batch_id) REFERENCES batches(batch_id) ON DELETE CASCADE
        ) WITHOUT ROWID;

        CREATE INDEX IF NOT EXISTS idx_batches_mission_update
            ON batches(mission_id, update_count);

        CREATE INDEX IF NOT EXISTS idx_batches_time
            ON batches(batch_unix_ns);

        CREATE INDEX IF NOT EXISTS idx_cells_center_xyz
            ON cells(center_x, center_y, center_z);

        CREATE INDEX IF NOT EXISTS idx_cells_source
            ON cells(source_kind, source_provider);
        """
    )
    conn.execute(
        "INSERT OR REPLACE INTO metadata(key, value) VALUES (?, ?)",
        ("schema_version", SCHEMA_VERSION),
    )


def insert_metadata(conn: sqlite3.Connection, metadata: dict[str, Any]) -> None:
    for key, value in metadata.items():
        conn.execute(
            "INSERT OR REPLACE INTO metadata(key, value) VALUES (?, ?)",
            (str(key), json.dumps(value, sort_keys=True)),
        )


def load_meta(meta_path: Path) -> dict[str, Any]:
    if not meta_path.exists():
        return {}
    with meta_path.open("r", encoding="utf-8") as f:
        value = json.load(f)
    if not isinstance(value, dict):
        return {}
    return value


def iter_jsonl(path: Path) -> Iterable[dict[str, Any]]:
    with path.open("r", encoding="utf-8") as f:
        for line_no, line in enumerate(f, start=1):
            stripped = line.strip()
            if not stripped:
                continue
            value = json.loads(stripped)
            if not isinstance(value, dict):
                raise ValueError(f"{path}:{line_no}: expected JSON object")
            yield value


def require_number(obj: dict[str, Any], key: str, default: float | int | None = None) -> float | int:
    value = obj.get(key, default)
    if value is None:
        raise ValueError(f"missing numeric field: {key}")
    if not isinstance(value, (int, float)):
        raise ValueError(f"field {key} is not numeric: {value!r}")
    return value


def require_string(obj: dict[str, Any], key: str, default: str | None = None) -> str:
    value = obj.get(key, default)
    if value is None:
        raise ValueError(f"missing string field: {key}")
    if not isinstance(value, str):
        raise ValueError(f"field {key} is not a string: {value!r}")
    return value


def require_center(cell: dict[str, Any]) -> tuple[float, float, float]:
    center = cell.get("center_mission")
    if not isinstance(center, dict):
        raise ValueError("cell missing center_mission object")
    return (
        float(require_number(center, "x")),
        float(require_number(center, "y")),
        float(require_number(center, "z")),
    )


def import_jsonl(args: argparse.Namespace) -> int:
    jsonl_path = Path(args.jsonl)
    if not jsonl_path.exists():
        print(f"ERROR: delta JSONL does not exist: {jsonl_path}")
        return 1

    db_path = Path(args.db)
    if args.replace and db_path.exists():
        db_path.unlink()
        wal_path = Path(str(db_path) + "-wal")
        shm_path = Path(str(db_path) + "-shm")
        if wal_path.exists():
            wal_path.unlink()
        if shm_path.exists():
            shm_path.unlink()

    meta_path = Path(args.meta) if args.meta else Path(str(jsonl_path) + ".meta.json")

    conn = connect(db_path)
    try:
        init_schema(conn)

        metadata = load_meta(meta_path)
        metadata.setdefault("source_jsonl_path", str(jsonl_path))
        metadata.setdefault("source_meta_path", str(meta_path))
        insert_metadata(conn, metadata)

        imported_batches = 0
        skipped_batches = 0
        imported_cells = 0

        with conn:
            for batch in iter_jsonl(jsonl_path):
                cells = batch.get("cells", [])
                if not isinstance(cells, list):
                    raise ValueError("batch field cells must be an array")

                cursor = conn.execute(
                    """
                    INSERT OR IGNORE INTO batches(
                        schema_name,
                        time_unit,
                        site_id,
                        site_frame_id,
                        mission_id,
                        mission_map_frame_id,
                        mission_start_unix_ns,
                        batch_unix_ns,
                        update_count,
                        previous_snapshot_timestamp_ns,
                        changed_cell_count
                    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                    """,
                    (
                        require_string(batch, "schema", "dedalus.mission_obstacle_map_delta_batch.v2"),
                        require_string(batch, "time_unit", "unix_ns"),
                        require_string(batch, "site_id", "unknown_site"),
                        require_string(batch, "site_frame_id", "site_local"),
                        require_string(batch, "mission_id", "unknown_mission"),
                        require_string(batch, "mission_map_frame_id", "unknown_mission_frame"),
                        int(require_number(batch, "mission_start_unix_ns", 0)),
                        int(require_number(batch, "batch_unix_ns")),
                        int(require_number(batch, "update_count")),
                        int(require_number(batch, "previous_snapshot_timestamp_ns", 0)),
                        int(require_number(batch, "changed_cell_count", len(cells))),
                    ),
                )

                if cursor.rowcount == 0:
                    skipped_batches += 1
                    continue

                batch_id = conn.execute("SELECT last_insert_rowid()").fetchone()[0]
                imported_batches += 1

                for cell_index, cell in enumerate(cells):
                    if not isinstance(cell, dict):
                        raise ValueError("cell entry must be an object")

                    x, y, z = require_center(cell)
                    conn.execute(
                        """
                        INSERT INTO cells(
                            batch_id,
                            cell_index,
                            center_x,
                            center_y,
                            center_z,
                            occupied_score,
                            free_score,
                            confidence,
                            first_seen_unix_ns,
                            last_seen_unix_ns,
                            source_kind,
                            source_provider
                        ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                        """,
                        (
                            batch_id,
                            cell_index,
                            x,
                            y,
                            z,
                            float(require_number(cell, "occupied_score", 0.0)),
                            float(require_number(cell, "free_score", 0.0)),
                            float(require_number(cell, "confidence", 0.0)),
                            int(require_number(cell, "first_seen_unix_ns", 0)),
                            int(require_number(cell, "last_seen_unix_ns", 0)),
                            require_string(cell, "source_kind", "unknown"),
                            require_string(cell, "source_provider", "unknown"),
                        ),
                    )
                    imported_cells += 1

        result = {
            "db": str(db_path),
            "source_jsonl": str(jsonl_path),
            "source_meta": str(meta_path),
            "imported_batches": imported_batches,
            "skipped_existing_batches": skipped_batches,
            "imported_cells": imported_cells,
            "db_size_bytes": db_path.stat().st_size if db_path.exists() else 0,
        }
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0
    except Exception as exc:
        print(f"ERROR: {exc}")
        return 1
    finally:
        conn.close()


def summary(args: argparse.Namespace) -> int:
    db_path = Path(args.db)
    if not db_path.exists():
        print(f"ERROR: SQLite delta db does not exist: {db_path}")
        return 1

    conn = connect(db_path)
    try:
        init_schema(conn)
        row = conn.execute(
            """
            SELECT
                COUNT(*) AS batch_count,
                COALESCE(SUM(changed_cell_count), 0) AS declared_changed_cells,
                COALESCE(MIN(batch_unix_ns), 0) AS first_batch_unix_ns,
                COALESCE(MAX(batch_unix_ns), 0) AS last_batch_unix_ns,
                COALESCE(MIN(update_count), 0) AS first_update_count,
                COALESCE(MAX(update_count), 0) AS last_update_count
            FROM batches
            """
        ).fetchone()
        cell_count = conn.execute("SELECT COUNT(*) FROM cells").fetchone()[0]
        missions = [
            r[0]
            for r in conn.execute(
                "SELECT DISTINCT mission_id FROM batches ORDER BY mission_id"
            ).fetchall()
        ]
        sources = [
            {"source_kind": r[0], "source_provider": r[1], "cell_count": r[2]}
            for r in conn.execute(
                """
                SELECT source_kind, source_provider, COUNT(*)
                FROM cells
                GROUP BY source_kind, source_provider
                ORDER BY COUNT(*) DESC, source_kind, source_provider
                """
            ).fetchall()
        ]

        result = {
            "schema": SCHEMA_VERSION,
            "db": str(db_path),
            "db_size_bytes": db_path.stat().st_size,
            "batch_count": row[0],
            "declared_changed_cells": row[1],
            "stored_cell_count": cell_count,
            "first_batch_unix_ns": row[2],
            "last_batch_unix_ns": row[3],
            "first_update_count": row[4],
            "last_update_count": row[5],
            "mission_ids": missions,
            "sources": sources,
        }
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0
    except Exception as exc:
        print(f"ERROR: {exc}")
        return 1
    finally:
        conn.close()


def query_region(args: argparse.Namespace) -> int:
    db_path = Path(args.db)
    if not db_path.exists():
        print(f"ERROR: SQLite delta db does not exist: {db_path}")
        return 1

    conn = connect(db_path)
    try:
        rows = conn.execute(
            """
            SELECT
                b.mission_id,
                b.update_count,
                b.batch_unix_ns,
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
            WHERE c.center_x BETWEEN ? AND ?
              AND c.center_y BETWEEN ? AND ?
              AND c.center_z BETWEEN ? AND ?
            ORDER BY b.batch_unix_ns, c.cell_index
            LIMIT ?
            """,
            (
                args.x_min,
                args.x_max,
                args.y_min,
                args.y_max,
                args.z_min,
                args.z_max,
                args.limit,
            ),
        ).fetchall()

        result = {
            "db": str(db_path),
            "region": {
                "x_min": args.x_min,
                "x_max": args.x_max,
                "y_min": args.y_min,
                "y_max": args.y_max,
                "z_min": args.z_min,
                "z_max": args.z_max,
            },
            "returned_cell_count": len(rows),
            "cells": [
                {
                    "mission_id": r[0],
                    "update_count": r[1],
                    "batch_unix_ns": r[2],
                    "center_mission": {"x": r[3], "y": r[4], "z": r[5]},
                    "occupied_score": r[6],
                    "free_score": r[7],
                    "confidence": r[8],
                    "first_seen_unix_ns": r[9],
                    "last_seen_unix_ns": r[10],
                    "source_kind": r[11],
                    "source_provider": r[12],
                }
                for r in rows
            ],
        }
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0
    except Exception as exc:
        print(f"ERROR: {exc}")
        return 1
    finally:
        conn.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Import/query compact mission obstacle delta JSONL streams in SQLite."
    )
    sub = parser.add_subparsers(dest="command", required=True)

    import_parser = sub.add_parser("import-jsonl", help="Import a compact delta JSONL stream.")
    import_parser.add_argument("jsonl", help="Path to mission_obstacle_map_deltas.jsonl")
    import_parser.add_argument("--meta", default="", help="Optional metadata JSON path.")
    import_parser.add_argument("--db", required=True, help="Output SQLite database path.")
    import_parser.add_argument("--replace", action="store_true", help="Replace existing database.")
    import_parser.set_defaults(func=import_jsonl)

    summary_parser = sub.add_parser("summary", help="Print database summary as JSON.")
    summary_parser.add_argument("--db", required=True, help="SQLite database path.")
    summary_parser.set_defaults(func=summary)

    query_parser = sub.add_parser("query-region", help="Query cells in mission-frame bounding box.")
    query_parser.add_argument("--db", required=True, help="SQLite database path.")
    query_parser.add_argument("--x-min", type=float, required=True)
    query_parser.add_argument("--x-max", type=float, required=True)
    query_parser.add_argument("--y-min", type=float, required=True)
    query_parser.add_argument("--y-max", type=float, required=True)
    query_parser.add_argument("--z-min", type=float, required=True)
    query_parser.add_argument("--z-max", type=float, required=True)
    query_parser.add_argument("--limit", type=int, default=25)
    query_parser.set_defaults(func=query_region)

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    main()
