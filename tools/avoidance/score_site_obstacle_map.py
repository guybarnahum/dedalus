#!/usr/bin/env python3
"""Re-score a persistent site obstacle map without merging new mission evidence.

5J: standalone scoring / aging experiment tool.

This tool recomputes derived fields only:

  - occupancy_score
  - freshness_score
  - active_score
  - cell_age_seconds
  - site_staleness_seconds
  - relative_gap_seconds
  - site_relative_age_percentile
  - status

It intentionally preserves raw evidence primitives:

  - occupied_log_odds
  - free_log_odds
  - first_seen_unix_ns
  - last_seen_unix_ns
  - last_confirmed_occupied_unix_ns
  - last_observed_free_unix_ns
  - observation counts
  - source stats / missions

Core policy:
  Calendar age alone is not erasure. Age is normalized against whole-site
  staleness:

    relative_gap_seconds = max(0, cell_age_seconds - site_staleness_seconds)

So if the entire site has not been revisited, cells become stale/freshness-adjusted
only mildly. Strong suppression comes from contradiction/free evidence.
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import os
import tempfile
import time
from pathlib import Path
from typing import Any


SCHEMA = "dedalus.site_obstacle_map.v1"
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


def sigmoid(log_odds: float) -> float:
    if log_odds >= 0:
        z = math.exp(-log_odds)
        return 1.0 / (1.0 + z)
    z = math.exp(log_odds)
    return z / (1.0 + z)


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


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


def score_site_map(
    site_map: dict[str, Any],
    *,
    now_unix_ns: int,
    freshness_half_life_days: float,
    freshness_floor: float,
    active_threshold: float,
    stale_threshold: float,
    probationary_threshold: float,
    contradiction_multiplier: float,
) -> None:
    if site_map.get("schema") != SCHEMA:
        raise SystemExit(f"Unsupported site map schema: {site_map.get('schema')}")

    if site_map.get("time_unit") != TIME_UNIT:
        raise SystemExit(f"Unsupported time_unit: {site_map.get('time_unit')}")

    cells = site_map.get("cells")
    if not isinstance(cells, list):
        raise SystemExit("site map does not contain a cells array")

    site_last_visited = as_int(site_map.get("site_last_visited_unix_ns"))
    site_staleness_seconds = (
        max(0.0, (now_unix_ns - site_last_visited) / 1e9)
        if site_last_visited
        else 0.0
    )

    half_life_seconds = max(1.0, freshness_half_life_days * 86400.0)
    freshness_floor = clamp(freshness_floor, 0.0, 1.0)

    ages: list[float] = []
    for cell in cells:
        if not isinstance(cell, dict):
            continue
        last_seen = as_int(cell.get("last_seen_unix_ns"))
        age_seconds = max(0.0, (now_unix_ns - last_seen) / 1e9) if last_seen else 0.0
        ages.append(age_seconds)

    sorted_ages = sorted(ages)

    def percentile(age_seconds: float) -> float:
        if not sorted_ages:
            return 0.0
        return bisect.bisect_right(sorted_ages, age_seconds) / len(sorted_ages)

    counts = {
        "active": 0,
        "stale": 0,
        "probationary": 0,
        "suppressed": 0,
        "retired": 0,
    }

    active_scores: list[float] = []
    freshness_scores: list[float] = []
    occupancy_scores: list[float] = []

    for cell in cells:
        if not isinstance(cell, dict):
            continue

        last_seen = as_int(cell.get("last_seen_unix_ns"))
        cell_age_seconds = max(0.0, (now_unix_ns - last_seen) / 1e9) if last_seen else 0.0
        relative_gap_seconds = max(0.0, cell_age_seconds - site_staleness_seconds)

        occupancy_score = sigmoid(as_float(cell.get("occupied_log_odds")))
        freshness_score = math.exp(-relative_gap_seconds / half_life_seconds)
        freshness_score = clamp(freshness_score, freshness_floor, 1.0)

        active_score = occupancy_score * freshness_score

        last_free = as_int(cell.get("last_observed_free_unix_ns"))
        last_occupied = as_int(cell.get("last_confirmed_occupied_unix_ns"))
        free_after_occupied = last_free > last_occupied and last_free > 0

        if free_after_occupied:
            active_score *= clamp(contradiction_multiplier, 0.0, 1.0)

        has_positive_evidence = (
            as_int(cell.get("positive_observation_count")) > 0
            or as_float(cell.get("max_raw_occupied_score")) > 0.0
            or as_float(cell.get("raw_occupied_score")) > 0.0
        )

        status = status_for(
            occupancy_score=occupancy_score,
            active_score=active_score,
            freshness_score=freshness_score,
            has_positive_evidence=has_positive_evidence,
            free_after_occupied=free_after_occupied,
            active_threshold=active_threshold,
            stale_threshold=stale_threshold,
            probationary_threshold=probationary_threshold,
        )
        counts[status] += 1

        active_scores.append(active_score)
        freshness_scores.append(freshness_score)
        occupancy_scores.append(occupancy_score)

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

    site_map["updated_at_unix_ns"] = now_unix_ns

    policy = site_map.setdefault("decay_policy", {})
    if not isinstance(policy, dict):
        policy = {}
        site_map["decay_policy"] = policy

    policy.update(
        {
            "relative_gap_seconds_formula": "max(0, cell_age_seconds - site_staleness_seconds)",
            "freshness_half_life_days": freshness_half_life_days,
            "freshness_floor": freshness_floor,
            "active_threshold": active_threshold,
            "stale_threshold": stale_threshold,
            "probationary_threshold": probationary_threshold,
            "contradiction_multiplier": contradiction_multiplier,
            "calendar_age_is_not_erasure": True,
        }
    )

    def mean(values: list[float]) -> float:
        return sum(values) / len(values) if values else 0.0

    site_map["score_summary"] = {
        "now_unix_ns": now_unix_ns,
        "site_staleness_seconds": site_staleness_seconds,
        "cell_count": len(cells),
        "active_cell_count": counts["active"],
        "stale_cell_count": counts["stale"],
        "probationary_cell_count": counts["probationary"],
        "suppressed_cell_count": counts["suppressed"],
        "retired_cell_count": counts["retired"],
        "mean_occupancy_score": mean(occupancy_scores),
        "mean_freshness_score": mean(freshness_scores),
        "mean_active_score": mean(active_scores),
        "max_active_score": max(active_scores) if active_scores else 0.0,
    }


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
        "decay_policy": site_map.get("decay_policy", {}),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("site_map", help="Input site_obstacle_map.json")
    parser.add_argument(
        "--output",
        default=None,
        help="Output path. Defaults to in-place update of site_map.",
    )
    parser.add_argument(
        "--now-unix-ns",
        type=int,
        default=None,
        help="Override current wall-clock time for aging experiments.",
    )
    parser.add_argument("--freshness-half-life-days", type=float, default=30.0)
    parser.add_argument("--freshness-floor", type=float, default=0.35)
    parser.add_argument("--active-threshold", type=float, default=0.55)
    parser.add_argument("--stale-threshold", type=float, default=0.55)
    parser.add_argument("--probationary-threshold", type=float, default=0.05)
    parser.add_argument(
        "--contradiction-multiplier",
        type=float,
        default=0.25,
        help="Multiplier applied when free evidence is newer than occupied evidence.",
    )
    args = parser.parse_args()

    input_path = Path(args.site_map)
    output_path = Path(args.output) if args.output else input_path
    now_unix_ns = args.now_unix_ns or unix_ns_now()

    site_map = load_json(input_path)
    score_site_map(
        site_map,
        now_unix_ns=now_unix_ns,
        freshness_half_life_days=args.freshness_half_life_days,
        freshness_floor=args.freshness_floor,
        active_threshold=args.active_threshold,
        stale_threshold=args.stale_threshold,
        probationary_threshold=args.probationary_threshold,
        contradiction_multiplier=args.contradiction_multiplier,
    )

    atomic_write_json(output_path, site_map)
    atomic_write_json(output_path.with_suffix(output_path.suffix + ".meta.json"), build_meta(site_map))

    summary = site_map.get("score_summary", {})
    print(f"Wrote {output_path}")
    print(f"Wrote {output_path.with_suffix(output_path.suffix + '.meta.json')}")
    print(f"Cells: {summary.get('cell_count')}")
    print(f"Active: {summary.get('active_cell_count')}")
    print(f"Stale: {summary.get('stale_cell_count')}")
    print(f"Probationary: {summary.get('probationary_cell_count')}")
    print(f"Suppressed: {summary.get('suppressed_cell_count')}")
    print(f"Retired: {summary.get('retired_cell_count')}")
    print(f"Mean active score: {summary.get('mean_active_score')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
