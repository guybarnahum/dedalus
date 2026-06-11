#!/usr/bin/env python3
"""Merge mission obstacle map artifacts into a persistent site obstacle map.

5I: post-process persistent map merge.

Input:
  - one or more mission_obstacle_map_full.json files

Output:
  - maps/<site_id>/site_obstacle_map.json
  - maps/<site_id>/site_obstacle_map.meta.json

The persistent map is site-local. It is not necessarily geodetic/global.
A mission artifact provides site_T_mission so mission-local cells can be
transformed into the site frame before quantization and merge.

Decay/freshness policy:
  Calendar age alone does not erase obstacles. Freshness uses site-relative age:

    relative_gap_seconds = max(0, cell_age_seconds - site_staleness_seconds)

This means that if an entire site has not been visited recently, cells do not
collapse simply because wall-clock time passed.
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


SCHEMA = "dedalus.site_obstacle_map.v1"
MISSION_SCHEMA = "dedalus.mission_obstacle_map.v1"
TIME_UNIT = "unix_ns"


def unix_ns_now() -> int:
    return time.time_ns()


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


def add(a: dict[str, float], b: dict[str, float]) -> dict[str, float]:
    return {"x": a["x"] + b["x"], "y": a["y"] + b["y"], "z": a["z"] + b["z"]}


def rotate_rpy(point: dict[str, float], rpy: dict[str, float]) -> dict[str, float]:
    """Rotate point by roll/pitch/yaw.

    Matches the existing repo's RPY convention closely enough for persistence
    tooling. In current AirSim validation site_T_mission is identity.
    """
    x, y, z = point["x"], point["y"], point["z"]
    cr, sr = math.cos(rpy["x"]), math.sin(rpy["x"])
    cp, sp = math.cos(rpy["y"]), math.sin(rpy["y"])
    cy, sy = math.cos(rpy["z"]), math.sin(rpy["z"])

    # Rz * Ry * Rx
    x1 = x
    y1 = cr * y - sr * z
    z1 = sr * y + cr * z

    x2 = cp * x1 + sp * z1
    y2 = y1
    z2 = -sp * x1 + cp * z1

    return {
        "x": cy * x2 - sy * y2,
        "y": sy * x2 + cy * y2,
        "z": z2,
    }


def transform_point(pose: dict[str, Any], point: dict[str, float]) -> dict[str, float]:
    position = vec3(pose.get("position"))
    rotation = vec3(pose.get("rotation_rpy"))
    return add(rotate_rpy(point, rotation), position)


def quantize(value: float, cell_size: float) -> int:
    if cell_size <= 0.0:
        raise SystemExit("cell size must be positive")
    return math.floor(value / cell_size)


def cell_key(center_site: dict[str, float], cell_size_m: float, vertical_cell_size_m: float) -> str:
    ix = quantize(center_site["x"], cell_size_m)
    iy = quantize(center_site["y"], cell_size_m)
    iz = quantize(center_site["z"], vertical_cell_size_m)
    return f"{ix}:{iy}:{iz}"


def key_center(key: str, cell_size_m: float, vertical_cell_size_m: float) -> dict[str, float]:
    ix_s, iy_s, iz_s = key.split(":")
    ix, iy, iz = int(ix_s), int(iy_s), int(iz_s)
    return {
        "x": (ix + 0.5) * cell_size_m,
        "y": (iy + 0.5) * cell_size_m,
        "z": (iz + 0.5) * vertical_cell_size_m,
    }


def sigmoid(log_odds: float) -> float:
    if log_odds >= 0:
        z = math.exp(-log_odds)
        return 1.0 / (1.0 + z)
    z = math.exp(log_odds)
    return z / (1.0 + z)


def logit(probability: float) -> float:
    p = min(0.999, max(0.001, probability))
    return math.log(p / (1.0 - p))


def discover_mission_maps(patterns: list[str]) -> list[Path]:
    paths: list[Path] = []
    for pattern in patterns:
        p = Path(pattern)
        if p.is_file():
            paths.append(p)
            continue
        if p.is_dir():
            paths.extend(sorted(p.glob("mission_obstacle_map_full.json")))
            paths.extend(sorted(p.glob("mission_obstacle_map.json")))
            continue
        paths.extend(Path(x) for x in sorted(glob.glob(pattern, recursive=True)))

    seen: set[str] = set()
    unique: list[Path] = []
    for path in paths:
        if not path.is_file():
            continue
        resolved = str(path.resolve())
        if resolved in seen:
            continue
        seen.add(resolved)
        unique.append(path)
    return unique


def default_site_map_path(site_id: str) -> Path:
    return Path("maps") / site_id / "site_obstacle_map.json"


def load_or_create_site_map(
    path: Path,
    *,
    site_id: str,
    site_frame_id: str,
    cell_size_m: float,
    vertical_cell_size_m: float,
    now_unix_ns: int,
) -> dict[str, Any]:
    if path.exists():
        site_map = load_json(path)
        if site_map.get("schema") != SCHEMA:
            raise SystemExit(f"Unsupported site map schema in {path}: {site_map.get('schema')}")
        return site_map

    return {
        "schema": SCHEMA,
        "time_unit": TIME_UNIT,
        "site_id": site_id,
        "site_frame_id": site_frame_id,
        "created_at_unix_ns": now_unix_ns,
        "updated_at_unix_ns": now_unix_ns,
        "site_last_visited_unix_ns": 0,
        "cell_size_m": cell_size_m,
        "vertical_cell_size_m": vertical_cell_size_m,
        "merge_summary": {
            "mission_merge_count": 0,
            "last_merged_mission_id": "",
            "last_merged_mission_end_unix_ns": 0,
        },
        "decay_policy": {
            "relative_gap_seconds_formula": "max(0, cell_age_seconds - site_staleness_seconds)",
            "freshness_half_life_days": 30.0,
            "freshness_floor": 0.35,
            "calendar_age_is_not_erasure": True,
        },
        "cells": [],
    }


def source_key(source: dict[str, Any]) -> str:
    provider = str(source.get("last_source_provider", ""))
    kind = str(source.get("last_source_kind", ""))
    if provider and kind:
        return f"{kind}:{provider}"
    return provider or kind or "unknown"


def update_source_stats(cell: dict[str, Any], mission_id: str, source: dict[str, Any]) -> None:
    source_stats = cell.setdefault("source_stats", {})
    key = source_key(source)
    source_stats[key] = as_int(source_stats.get(key)) + 1

    missions = cell.setdefault("source_missions", [])
    if mission_id not in missions:
        missions.append(mission_id)

    cell["source"] = {
        "last_source_kind": str(source.get("last_source_kind", "")),
        "last_source_provider": str(source.get("last_source_provider", "")),
    }


def merge_cell(
    existing: dict[str, Any] | None,
    *,
    key: str,
    center_site: dict[str, float],
    mission_cell: dict[str, Any],
    mission: dict[str, Any],
    cell_size_m: float,
    vertical_cell_size_m: float,
) -> dict[str, Any]:
    mission_id = str(mission.get("mission_id", "unknown_mission"))

    incoming_raw_occupied = as_float(mission_cell.get("occupied_score"))
    incoming_raw_free = as_float(mission_cell.get("free_score"))
    incoming_score = as_float(
        mission_cell.get(
            "normalized_occupied_score",
            mission_cell.get("derived", {}).get("persistent_score", 0.0)
            if isinstance(mission_cell.get("derived"), dict)
            else 0.0,
        )
    )
    incoming_free = as_float(mission_cell.get("normalized_free_score"))

    incoming_log_odds = as_float(mission_cell.get("occupied_log_odds"), logit(incoming_score))
    incoming_confidence = as_float(mission_cell.get("confidence"))

    first_seen = as_int(mission_cell.get("first_seen_unix_ns"))
    last_seen = as_int(mission_cell.get("last_seen_unix_ns"))
    last_occupied = as_int(mission_cell.get("last_confirmed_occupied_unix_ns"))
    last_free = as_int(mission_cell.get("last_observed_free_unix_ns"))
    last_frustum = as_int(mission_cell.get("last_in_sensor_frustum_unix_ns"))

    positive_count = as_int(mission_cell.get("positive_observation_count"))
    negative_count = as_int(mission_cell.get("negative_observation_count"))
    mission_observation_count = as_int(mission_cell.get("mission_observation_count"), 1)

    if existing is None:
        cell = {
            "key": key,
            "center_site": center_site,
            "size_m": {
                "x": cell_size_m,
                "y": cell_size_m,
                "z": vertical_cell_size_m,
            },
            "occupied_log_odds": incoming_log_odds,
            "free_log_odds": logit(incoming_free) if incoming_free > 0.0 else 0.0,
            "raw_occupied_score": incoming_raw_occupied,
            "raw_free_score": incoming_raw_free,
            "max_raw_occupied_score": incoming_raw_occupied,
            "max_raw_free_score": incoming_raw_free,
            "confidence": incoming_confidence,
            "first_seen_unix_ns": first_seen or last_seen,
            "last_seen_unix_ns": last_seen,
            "last_confirmed_occupied_unix_ns": last_occupied,
            "last_observed_free_unix_ns": last_free,
            "last_in_sensor_frustum_unix_ns": last_frustum,
            "positive_observation_count": positive_count,
            "negative_observation_count": negative_count,
            "mission_observation_count": mission_observation_count,
            "source": {},
            "source_stats": {},
            "source_missions": [],
            "derived": {},
        }
    else:
        cell = existing
        previous_count = max(1, as_int(cell.get("mission_observation_count"), 1))
        incoming_count = max(1, mission_observation_count)

        # Merge in log-odds space. This keeps repeated missions reinforcing evidence
        # while still preserving raw/max scores for later formula experiments.
        cell["occupied_log_odds"] = as_float(cell.get("occupied_log_odds")) + incoming_log_odds
        if incoming_free > 0.0:
            cell["free_log_odds"] = as_float(cell.get("free_log_odds")) + logit(incoming_free)

        cell["raw_occupied_score"] = as_float(cell.get("raw_occupied_score")) + incoming_raw_occupied
        cell["raw_free_score"] = as_float(cell.get("raw_free_score")) + incoming_raw_free
        cell["max_raw_occupied_score"] = max(as_float(cell.get("max_raw_occupied_score")), incoming_raw_occupied)
        cell["max_raw_free_score"] = max(as_float(cell.get("max_raw_free_score")), incoming_raw_free)

        cell["confidence"] = (
            as_float(cell.get("confidence")) * previous_count
            + incoming_confidence * incoming_count
        ) / (previous_count + incoming_count)

        if first_seen:
            existing_first = as_int(cell.get("first_seen_unix_ns"), first_seen)
            cell["first_seen_unix_ns"] = min(existing_first, first_seen)

        cell["last_seen_unix_ns"] = max(as_int(cell.get("last_seen_unix_ns")), last_seen)
        cell["last_confirmed_occupied_unix_ns"] = max(
            as_int(cell.get("last_confirmed_occupied_unix_ns")), last_occupied
        )
        cell["last_observed_free_unix_ns"] = max(
            as_int(cell.get("last_observed_free_unix_ns")), last_free
        )
        cell["last_in_sensor_frustum_unix_ns"] = max(
            as_int(cell.get("last_in_sensor_frustum_unix_ns")), last_frustum
        )
        cell["positive_observation_count"] = as_int(cell.get("positive_observation_count")) + positive_count
        cell["negative_observation_count"] = as_int(cell.get("negative_observation_count")) + negative_count
        cell["mission_observation_count"] = previous_count + incoming_count

    update_source_stats(cell, mission_id, mission_cell.get("source", {}) if isinstance(mission_cell.get("source"), dict) else {})
    return cell


def score_site_map(site_map: dict[str, Any], now_unix_ns: int) -> None:
    cells = site_map.get("cells", [])
    if not isinstance(cells, list):
        cells = []
        site_map["cells"] = cells

    site_last_visited = as_int(site_map.get("site_last_visited_unix_ns"))
    site_staleness_seconds = max(0.0, (now_unix_ns - site_last_visited) / 1e9) if site_last_visited else 0.0

    policy = site_map.get("decay_policy", {})
    if not isinstance(policy, dict):
        policy = {}
    half_life_days = as_float(policy.get("freshness_half_life_days"), 30.0)
    freshness_floor = as_float(policy.get("freshness_floor"), 0.35)
    half_life_seconds = max(1.0, half_life_days * 86400.0)

    ages: list[float] = []
    for cell in cells:
        if not isinstance(cell, dict):
            continue
        last_seen = as_int(cell.get("last_seen_unix_ns"))
        age_seconds = max(0.0, (now_unix_ns - last_seen) / 1e9) if last_seen else 0.0
        ages.append(age_seconds)

    sorted_ages = sorted(ages)

    def percentile(age: float) -> float:
        if not sorted_ages:
            return 0.0
        # fraction of cells no older than this cell
        count = 0
        for value in sorted_ages:
            if value <= age:
                count += 1
            else:
                break
        return count / len(sorted_ages)

    active_count = 0
    stale_count = 0
    suppressed_count = 0
    retired_count = 0

    for cell in cells:
        if not isinstance(cell, dict):
            continue

        last_seen = as_int(cell.get("last_seen_unix_ns"))
        cell_age_seconds = max(0.0, (now_unix_ns - last_seen) / 1e9) if last_seen else 0.0
        relative_gap_seconds = max(0.0, cell_age_seconds - site_staleness_seconds)

        occupancy_score = sigmoid(as_float(cell.get("occupied_log_odds")))
        freshness_score = math.exp(-relative_gap_seconds / half_life_seconds)
        freshness_score = max(freshness_floor, min(1.0, freshness_score))

        active_score = occupancy_score * freshness_score

        last_free = as_int(cell.get("last_observed_free_unix_ns"))
        last_occupied = as_int(cell.get("last_confirmed_occupied_unix_ns"))
        free_after_occupied = last_free > last_occupied and last_free > 0

        if free_after_occupied:
            active_score *= 0.25
            status = "suppressed"
            suppressed_count += 1
        elif active_score >= 0.55:
            status = "active"
            active_count += 1
        elif occupancy_score >= 0.55:
            status = "stale"
            stale_count += 1
        else:
            status = "retired"
            retired_count += 1

        cell["derived"] = {
            "occupancy_score": occupancy_score,
            "freshness_score": freshness_score,
            "active_score": active_score,
            "cell_age_seconds": cell_age_seconds,
            "site_staleness_seconds": site_staleness_seconds,
            "relative_gap_seconds": relative_gap_seconds,
            "site_relative_age_percentile": percentile(cell_age_seconds),
            "status": status,
        }

    site_map["score_summary"] = {
        "now_unix_ns": now_unix_ns,
        "site_staleness_seconds": site_staleness_seconds,
        "cell_count": len(cells),
        "active_cell_count": active_count,
        "stale_cell_count": stale_count,
        "suppressed_cell_count": suppressed_count,
        "retired_cell_count": retired_count,
    }


def merge_mission_into_site(
    site_map: dict[str, Any],
    mission: dict[str, Any],
    *,
    now_unix_ns: int,
) -> None:
    if mission.get("schema") != MISSION_SCHEMA:
        raise SystemExit(f"Unsupported mission map schema: {mission.get('schema')}")

    site_id = str(mission.get("site_id", site_map.get("site_id", "unknown_site")))
    site_frame_id = str(mission.get("site_frame_id", site_map.get("site_frame_id", "site_local")))
    mission_id = str(mission.get("mission_id", "unknown_mission"))
    mission_end_unix_ns = as_int(mission.get("mission_end_unix_ns"), now_unix_ns)

    if str(site_map.get("site_id")) != site_id:
        raise SystemExit(f"site_id mismatch: site map={site_map.get('site_id')} mission={site_id}")
    if str(site_map.get("site_frame_id")) != site_frame_id:
        raise SystemExit(
            f"site_frame_id mismatch: site map={site_map.get('site_frame_id')} mission={site_frame_id}"
        )

    cell_size_m = as_float(site_map.get("cell_size_m"), as_float(mission.get("cell_size_m"), 0.5))
    vertical_cell_size_m = as_float(
        site_map.get("vertical_cell_size_m"),
        as_float(mission.get("vertical_cell_size_m"), cell_size_m),
    )

    site_t_mission = mission.get("site_T_mission")
    if not isinstance(site_t_mission, dict):
        site_t_mission = {"position": {"x": 0, "y": 0, "z": 0}, "rotation_rpy": {"x": 0, "y": 0, "z": 0}}

    cells_by_key = {
        str(cell.get("key")): cell
        for cell in site_map.get("cells", [])
        if isinstance(cell, dict) and cell.get("key") is not None
    }

    mission_cells = mission.get("cells", [])
    if not isinstance(mission_cells, list):
        mission_cells = []

    merged_count = 0
    for mission_cell in mission_cells:
        if not isinstance(mission_cell, dict):
            continue
        center_mission = vec3(mission_cell.get("center_mission"))
        center_site = transform_point(site_t_mission, center_mission)
        key = cell_key(center_site, cell_size_m, vertical_cell_size_m)
        center_site_quantized = key_center(key, cell_size_m, vertical_cell_size_m)

        cells_by_key[key] = merge_cell(
            cells_by_key.get(key),
            key=key,
            center_site=center_site_quantized,
            mission_cell=mission_cell,
            mission=mission,
            cell_size_m=cell_size_m,
            vertical_cell_size_m=vertical_cell_size_m,
        )
        merged_count += 1

    site_map["cells"] = sorted(cells_by_key.values(), key=lambda cell: str(cell.get("key")))
    site_map["updated_at_unix_ns"] = now_unix_ns
    site_map["site_last_visited_unix_ns"] = max(
        as_int(site_map.get("site_last_visited_unix_ns")),
        mission_end_unix_ns,
    )
    site_map["cell_size_m"] = cell_size_m
    site_map["vertical_cell_size_m"] = vertical_cell_size_m

    merge_summary = site_map.setdefault("merge_summary", {})
    if not isinstance(merge_summary, dict):
        merge_summary = {}
        site_map["merge_summary"] = merge_summary

    merge_summary["mission_merge_count"] = as_int(merge_summary.get("mission_merge_count")) + 1
    merge_summary["last_merged_mission_id"] = mission_id
    merge_summary["last_merged_mission_end_unix_ns"] = mission_end_unix_ns
    merge_summary["last_merged_cell_count"] = merged_count


def build_meta(site_map: dict[str, Any]) -> dict[str, Any]:
    return {
        "schema": f"{SCHEMA}.meta",
        "time_unit": TIME_UNIT,
        "site_id": site_map.get("site_id", ""),
        "site_frame_id": site_map.get("site_frame_id", ""),
        "created_at_unix_ns": site_map.get("created_at_unix_ns", 0),
        "updated_at_unix_ns": site_map.get("updated_at_unix_ns", 0),
        "site_last_visited_unix_ns": site_map.get("site_last_visited_unix_ns", 0),
        "cell_size_m": site_map.get("cell_size_m", 0),
        "vertical_cell_size_m": site_map.get("vertical_cell_size_m", 0),
        "merge_summary": site_map.get("merge_summary", {}),
        "score_summary": site_map.get("score_summary", {}),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "mission_maps",
        nargs="+",
        help="Mission map file(s), directories, or globs. Prefer mission_obstacle_map_full.json.",
    )
    parser.add_argument("--site-map", default=None, help="Output site map path. Default: maps/<site_id>/site_obstacle_map.json")
    parser.add_argument("--site-id", default=None, help="Override/validate site id. Defaults to first mission artifact site_id.")
    parser.add_argument("--site-frame-id", default=None, help="Override/validate site frame id. Defaults to first mission artifact site_frame_id.")
    parser.add_argument("--now-unix-ns", type=int, default=None)
    parser.add_argument("--freshness-half-life-days", type=float, default=30.0)
    parser.add_argument("--freshness-floor", type=float, default=0.35)
    args = parser.parse_args()

    now_unix_ns = args.now_unix_ns or unix_ns_now()

    paths = discover_mission_maps(args.mission_maps)
    if not paths:
        raise SystemExit("No mission obstacle map artifacts found")

    missions = [load_json(path) for path in paths]
    first = missions[0]
    site_id = args.site_id or str(first.get("site_id", "unknown_site"))
    site_frame_id = args.site_frame_id or str(first.get("site_frame_id", "site_local"))

    site_map_path = Path(args.site_map) if args.site_map else default_site_map_path(site_id)

    site_map = load_or_create_site_map(
        site_map_path,
        site_id=site_id,
        site_frame_id=site_frame_id,
        cell_size_m=as_float(first.get("cell_size_m"), 0.5),
        vertical_cell_size_m=as_float(first.get("vertical_cell_size_m"), as_float(first.get("cell_size_m"), 0.5)),
        now_unix_ns=now_unix_ns,
    )

    site_map["decay_policy"]["freshness_half_life_days"] = args.freshness_half_life_days
    site_map["decay_policy"]["freshness_floor"] = args.freshness_floor

    for mission in missions:
        # Apply CLI site override/validation to mission records without mutating the file.
        mission = dict(mission)
        mission["site_id"] = args.site_id or mission.get("site_id", site_id)
        mission["site_frame_id"] = args.site_frame_id or mission.get("site_frame_id", site_frame_id)
        merge_mission_into_site(site_map, mission, now_unix_ns=now_unix_ns)

    score_site_map(site_map, now_unix_ns)

    atomic_write_json(site_map_path, site_map)
    atomic_write_json(site_map_path.with_suffix(site_map_path.suffix + ".meta.json"), build_meta(site_map))

    print(f"Wrote {site_map_path}")
    print(f"Wrote {site_map_path.with_suffix(site_map_path.suffix + '.meta.json')}")
    print(f"Merged mission maps: {len(missions)}")
    print(f"Site cells: {len(site_map.get('cells', []))}")
    print(f"Active cells: {site_map.get('score_summary', {}).get('active_cell_count')}")
    print(f"Stale cells: {site_map.get('score_summary', {}).get('stale_cell_count')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
