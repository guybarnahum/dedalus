#!/usr/bin/env python3
"""Export a persistent mission obstacle map artifact from Dedalus snapshots.

This is a post-process tool for 5H.

Input:
  - one snapshot_*.json, or
  - a directory/glob containing snapshot_*.json files.

Output:
  - mission_obstacle_map.json
  - mission_obstacle_map.meta.json

The artifact records explicit time units (`unix_ns`) and primitive timestamp/count
fields so later tools can experiment with age/freshness/decay formulas.

Current limitation:
  WorldSnapshot publishes mission_local_obstacle_map.debug_cells, which are capped
  diagnostics. This exporter therefore exports the cells available in the snapshot.
  A later runtime artifact writer should export the full mission-local map.
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import tempfile
import time
from pathlib import Path
from typing import Any


SCHEMA = "dedalus.mission_obstacle_map.v1"
TIME_UNIT = "unix_ns"


def unix_ns_now() -> int:
    return time.time_ns()


def file_mtime_unix_ns(path: Path) -> int:
    return int(path.stat().st_mtime_ns)


def looks_like_unix_ns(timestamp_ns: int) -> bool:
    # Anything after year 2000 in ns is absolute wall-clock time for our use.
    # Mission-relative clocks are expected to be much smaller.
    return timestamp_ns >= 946684800000000000


def normalized_score(raw_score: float, score_scale: float) -> float:
    if score_scale <= 0.0:
        return 0.0
    value = raw_score / score_scale
    if not math.isfinite(value):
        return 0.0
    return min(1.0, max(0.0, value))


def as_float(value: Any, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def as_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def vec3(value: Any, default: dict[str, float] | None = None) -> dict[str, float]:
    if default is None:
        default = {"x": 0.0, "y": 0.0, "z": 0.0}
    if isinstance(value, dict):
        return {
            "x": as_float(value.get("x"), default["x"]),
            "y": as_float(value.get("y"), default["y"]),
            "z": as_float(value.get("z"), default["z"]),
        }
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        return {"x": as_float(value[0]), "y": as_float(value[1]), "z": as_float(value[2])}
    return dict(default)


def identity_pose() -> dict[str, dict[str, float]]:
    return {
        "position": {"x": 0.0, "y": 0.0, "z": 0.0},
        "rotation_rpy": {"x": 0.0, "y": 0.0, "z": 0.0},
    }


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise SystemExit(f"JSON root is not an object: {path}")
    return data


def atomic_write_json(path: Path, data: dict[str, Any]) -> None:
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


def snapshot_index(path: Path) -> int:
    digits = "".join(ch if ch.isdigit() else " " for ch in path.name).split()
    return int(digits[-1]) if digits else -1


def discover_snapshots(input_path: str) -> list[Path]:
    p = Path(input_path)

    if p.is_file():
        return [p]

    if p.is_dir():
        matches = sorted(p.glob("snapshot_*.json"), key=snapshot_index)
        return list(matches)

    matches = sorted((Path(x) for x in glob.glob(input_path, recursive=True)), key=snapshot_index)
    return [m for m in matches if m.is_file()]


def select_snapshot(paths: list[Path]) -> tuple[Path, dict[str, Any]]:
    if not paths:
        raise SystemExit("No snapshot_*.json files found")

    for path in reversed(paths):
        try:
            snapshot = load_json(path)
        except Exception:
            continue
        mission = snapshot.get("mission_local_obstacle_map")
        if isinstance(mission, dict) and isinstance(mission.get("debug_cells"), list):
            return path, snapshot

    raise SystemExit("No snapshot contains mission_local_obstacle_map.debug_cells")


def align_mission_ns_to_unix_ns(
    mission_timestamp_ns: int,
    mission_end_timestamp_ns: int,
    mission_end_unix_ns: int,
) -> int:
    if mission_timestamp_ns <= 0:
        return mission_end_unix_ns

    # 5F snapshots currently carry absolute unix_ns values from TimePoint.
    # Preserve those directly instead of treating them as mission-relative.
    if looks_like_unix_ns(mission_timestamp_ns):
        return mission_timestamp_ns

    if mission_end_timestamp_ns <= 0:
        return mission_end_unix_ns

    if looks_like_unix_ns(mission_end_timestamp_ns):
        # Mixed case: a relative cell timestamp with an absolute mission end.
        # Align relative value against wall-clock mission end as best effort.
        return max(0, mission_end_unix_ns - max(0, mission_end_timestamp_ns - mission_timestamp_ns))

    delta = max(0, mission_end_timestamp_ns - mission_timestamp_ns)
    return max(0, mission_end_unix_ns - delta)


def cell_to_artifact_cell(
    raw: dict[str, Any],
    *,
    mission_end_timestamp_ns: int,
    mission_end_unix_ns: int,
    score_scale: float,
) -> dict[str, Any]:
    first_mission_ns = as_int(raw.get("first_observed_timestamp_ns"))
    last_mission_ns = as_int(raw.get("last_observed_timestamp_ns"))

    first_seen_unix_ns = align_mission_ns_to_unix_ns(
        first_mission_ns,
        mission_end_timestamp_ns,
        mission_end_unix_ns,
    )
    last_seen_unix_ns = align_mission_ns_to_unix_ns(
        last_mission_ns,
        mission_end_timestamp_ns,
        mission_end_unix_ns,
    )

    occupied = bool(raw.get("occupied", False))
    free = bool(raw.get("free", False))
    occupied_score = as_float(raw.get("occupied_score"))
    free_score = as_float(raw.get("free_score"))
    confidence = as_float(raw.get("confidence"))
    normalized_occupied_score = normalized_score(occupied_score, score_scale)
    normalized_free_score = normalized_score(free_score, score_scale)

    last_confirmed_occupied_unix_ns = last_seen_unix_ns if occupied else 0
    last_observed_free_unix_ns = last_seen_unix_ns if free else 0

    # Approximate log-odds from normalized score when possible. This is a
    # derived field; raw score and timestamps are kept so formulas can change.
    clamped = min(0.999, max(0.001, normalized_occupied_score))
    occupied_log_odds = math.log(clamped / (1.0 - clamped))

    return {
        "center_mission": vec3(raw.get("center_map")),
        "size_m": vec3(raw.get("size_m"), {"x": 0.5, "y": 0.5, "z": 0.5}),
        "occupied": occupied,
        "free": free,
        "occupied_score": occupied_score,
        "free_score": free_score,
        "normalized_occupied_score": normalized_occupied_score,
        "normalized_free_score": normalized_free_score,
        "score_scale": score_scale,
        "risk_score": as_float(raw.get("risk_score")),
        "confidence": confidence,
        "occupied_log_odds": occupied_log_odds,
        "first_seen_unix_ns": first_seen_unix_ns,
        "last_seen_unix_ns": last_seen_unix_ns,
        "last_confirmed_occupied_unix_ns": last_confirmed_occupied_unix_ns,
        "last_observed_free_unix_ns": last_observed_free_unix_ns,
        "last_in_sensor_frustum_unix_ns": last_seen_unix_ns,
        "positive_observation_count": 1 if occupied else 0,
        "negative_observation_count": 1 if free else 0,
        "mission_observation_count": 1,
        "source": {
            "last_source_kind": str(raw.get("last_source_kind", "")),
            "last_source_provider": str(raw.get("last_source_provider", "")),
        },
        "derived": {
            "persistent_score": normalized_occupied_score,
            "freshness_score": 1.0,
            "active_score": normalized_occupied_score,
            "status": "active" if occupied else "free" if free else "unknown",
        },
    }


def build_artifact(
    *,
    snapshot_path: Path,
    snapshot: dict[str, Any],
    site_id: str,
    mission_id: str,
    site_frame_id: str,
    site_t_mission: dict[str, Any],
    mission_start_unix_ns: int | None,
    mission_end_unix_ns: int | None,
) -> tuple[dict[str, Any], dict[str, Any]]:
    mission = snapshot.get("mission_local_obstacle_map")
    if not isinstance(mission, dict):
        raise SystemExit("Snapshot does not contain mission_local_obstacle_map")

    raw_cells = mission.get("debug_cells")
    if not isinstance(raw_cells, list):
        raw_cells = []

    mission_end_timestamp_ns = as_int(mission.get("last_update_timestamp_ns"))
    if mission_end_unix_ns is None:
        mission_end_unix_ns = file_mtime_unix_ns(snapshot_path)
    if mission_start_unix_ns is None:
        # Prefer the earliest exported cell timestamp. If snapshots carry unix_ns,
        # this preserves real wall-clock mission time. Fall back conservatively.
        cell_first_seen = [
            as_int(raw.get("first_observed_timestamp_ns"))
            for raw in raw_cells
            if isinstance(raw, dict) and as_int(raw.get("first_observed_timestamp_ns")) > 0
        ]
        unix_cell_first_seen = [t for t in cell_first_seen if looks_like_unix_ns(t)]
        if unix_cell_first_seen:
            mission_start_unix_ns = min(unix_cell_first_seen)
        elif mission_end_timestamp_ns > 0 and not looks_like_unix_ns(mission_end_timestamp_ns):
            mission_start_unix_ns = max(0, mission_end_unix_ns - mission_end_timestamp_ns)
        else:
            mission_start_unix_ns = mission_end_unix_ns

    raw_score_values = [
        max(as_float(raw.get("occupied_score")), as_float(raw.get("free_score")))
        for raw in raw_cells
        if isinstance(raw, dict)
    ]
    score_scale = max([1.0] + raw_score_values)

    cells: list[dict[str, Any]] = []
    for raw in raw_cells:
        if not isinstance(raw, dict):
            continue
        cells.append(
            cell_to_artifact_cell(
                raw,
                mission_end_timestamp_ns=mission_end_timestamp_ns,
                mission_end_unix_ns=mission_end_unix_ns,
                score_scale=score_scale,
            )
        )

    map_frame_id = str(mission.get("map_frame_id", ""))
    debug_cell_limit = as_int(mission.get("debug_cell_limit"))
    observed_cell_count = as_int(mission.get("observed_cell_count"))
    occupied_cell_count = as_int(mission.get("occupied_cell_count"))

    artifact = {
        "schema": SCHEMA,
        "time_unit": TIME_UNIT,
        "site_id": site_id,
        "site_frame_id": site_frame_id,
        "mission_id": mission_id,
        "mission_map_frame_id": map_frame_id,
        "site_T_mission": site_t_mission,
        "mission_start_unix_ns": mission_start_unix_ns,
        "mission_end_unix_ns": mission_end_unix_ns,
        "created_at_unix_ns": unix_ns_now(),
        "source_snapshot": str(snapshot_path),
        "source_snapshot_mtime_unix_ns": file_mtime_unix_ns(snapshot_path),
        "cell_size_m": as_float(mission.get("cell_size_m")),
        "vertical_cell_size_m": as_float(mission.get("vertical_cell_size_m")),
        "score_scale": score_scale,
        "mission_summary": {
            "observed_cell_count": observed_cell_count,
            "occupied_cell_count": occupied_cell_count,
            "free_cell_count": as_int(mission.get("free_cell_count")),
            "update_count": as_int(mission.get("update_count")),
            "last_update_timestamp_ns": mission_end_timestamp_ns,
        },
        "export_summary": {
            "exported_cell_count": len(cells),
            "source_cells_are_debug_capped": debug_cell_limit > 0 and len(cells) <= debug_cell_limit,
            "source_debug_cell_limit": debug_cell_limit,
            "coverage_note": (
                "This artifact was exported from WorldSnapshot mission_local_obstacle_map.debug_cells. "
                "It may be capped and is not guaranteed to contain the full mission-local obstacle map."
            ),
        },
        "decay_policy_note": {
            "calendar_age_is_not_erasure": True,
            "recommended_relative_gap_seconds": (
                "max(0, cell_age_seconds - site_staleness_seconds)"
            ),
            "strong_decay_requires": "contradiction or revisits without reconfirmation",
        },
        "cells": cells,
    }

    meta = {
        "schema": f"{SCHEMA}.meta",
        "time_unit": TIME_UNIT,
        "site_id": site_id,
        "site_frame_id": site_frame_id,
        "mission_id": mission_id,
        "mission_map_frame_id": map_frame_id,
        "source_snapshot": str(snapshot_path),
        "mission_start_unix_ns": mission_start_unix_ns,
        "mission_end_unix_ns": mission_end_unix_ns,
        "created_at_unix_ns": artifact["created_at_unix_ns"],
        "cell_size_m": artifact["cell_size_m"],
        "vertical_cell_size_m": artifact["vertical_cell_size_m"],
        "score_scale": artifact["score_scale"],
        "observed_cell_count": observed_cell_count,
        "occupied_cell_count": occupied_cell_count,
        "exported_cell_count": len(cells),
        "source_cells_are_debug_capped": artifact["export_summary"]["source_cells_are_debug_capped"],
        "source_debug_cell_limit": debug_cell_limit,
    }

    return artifact, meta


def parse_site_t_mission(value: str | None) -> dict[str, Any]:
    if not value:
        return identity_pose()

    path = Path(value)
    if path.exists():
        data = load_json(path)
    else:
        data = json.loads(value)

    if not isinstance(data, dict):
        raise SystemExit("--site-T-mission must be a JSON object or path to one")

    position = vec3(data.get("position"))
    rotation = vec3(data.get("rotation_rpy"))
    return {"position": position, "rotation_rpy": rotation}


def default_mission_id(snapshot_path: Path) -> str:
    parent = snapshot_path.parent.name or "mission"
    stem = snapshot_path.stem
    return f"{parent}_{stem}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "input",
        help="Snapshot file, snapshot directory, or glob. Example: out/run or out/run/snapshot_*.json",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Directory for mission_obstacle_map.json. Defaults to selected snapshot directory.",
    )
    parser.add_argument("--site-id", default="unknown_site")
    parser.add_argument("--site-frame-id", default="site_local")
    parser.add_argument("--mission-id", default=None)
    parser.add_argument(
        "--site-T-mission",
        default=None,
        help="JSON object or JSON file path with {position, rotation_rpy}. Defaults to identity.",
    )
    parser.add_argument(
        "--mission-start-unix-ns",
        type=int,
        default=None,
        help="Optional absolute mission start time in unix ns.",
    )
    parser.add_argument(
        "--mission-end-unix-ns",
        type=int,
        default=None,
        help="Optional absolute mission end time in unix ns. Defaults to selected snapshot mtime.",
    )
    parser.add_argument(
        "--allow-debug-capped",
        action="store_true",
        help="Allow export from capped debug cells. Currently required for snapshot-based export.",
    )
    args = parser.parse_args()

    paths = discover_snapshots(args.input)
    snapshot_path, snapshot = select_snapshot(paths)

    mission = snapshot.get("mission_local_obstacle_map")
    debug_limit = as_int(mission.get("debug_cell_limit")) if isinstance(mission, dict) else 0
    debug_cells = mission.get("debug_cells") if isinstance(mission, dict) else []
    is_debug_capped = debug_limit > 0 and isinstance(debug_cells, list) and len(debug_cells) <= debug_limit
    if is_debug_capped and not args.allow_debug_capped:
        raise SystemExit(
            "Refusing to export capped debug cells without --allow-debug-capped. "
            "This protects against mistaking diagnostics for the full map."
        )

    output_dir = Path(args.output_dir) if args.output_dir else snapshot_path.parent
    mission_id = args.mission_id or default_mission_id(snapshot_path)

    artifact, meta = build_artifact(
        snapshot_path=snapshot_path,
        snapshot=snapshot,
        site_id=args.site_id,
        mission_id=mission_id,
        site_frame_id=args.site_frame_id,
        site_t_mission=parse_site_t_mission(args.site_T_mission),
        mission_start_unix_ns=args.mission_start_unix_ns,
        mission_end_unix_ns=args.mission_end_unix_ns,
    )

    map_path = output_dir / "mission_obstacle_map.json"
    meta_path = output_dir / "mission_obstacle_map.meta.json"
    atomic_write_json(map_path, artifact)
    atomic_write_json(meta_path, meta)

    print(f"Wrote {map_path}")
    print(f"Wrote {meta_path}")
    print(f"Snapshot: {snapshot_path}")
    print(f"Site: {args.site_id}")
    print(f"Mission: {mission_id}")
    print(f"Exported cells: {len(artifact['cells'])}")
    if artifact["export_summary"]["source_cells_are_debug_capped"]:
        print(
            "Warning: exported from capped debug_cells; this is not the full mission-local map."
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
