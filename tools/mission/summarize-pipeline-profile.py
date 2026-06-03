#!/usr/bin/env python3
"""Summarize Dedalus pipeline profiler JSONL output.

The profiler writes one JSON object per processed frame with a `stages` object
mapping stage names to microsecond durations. This tool aggregates those stage
samples so EFF-0 baselines can be compared before and after runtime changes.
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
from pathlib import Path
from typing import Any


def percentile(values: list[int], pct: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    if len(ordered) == 1:
        return float(ordered[0])
    rank = (pct / 100.0) * (len(ordered) - 1)
    low = int(rank)
    high = min(low + 1, len(ordered) - 1)
    fraction = rank - low
    return float(ordered[low]) * (1.0 - fraction) + float(ordered[high]) * fraction


def read_profiles(path: Path) -> list[dict[str, Any]]:
    profiles: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line_number, raw in enumerate(handle, start=1):
            raw = raw.strip()
            if not raw:
                continue
            try:
                data = json.loads(raw)
            except json.JSONDecodeError as exc:
                raise SystemExit(f"{path}:{line_number}: invalid JSON: {exc}") from exc
            if not isinstance(data, dict):
                raise SystemExit(f"{path}:{line_number}: profile record is not an object")
            profiles.append(data)
    return profiles


def summarize(path: Path, top: int) -> int:
    if not path.exists():
        print(f"profile not found: {path}", file=sys.stderr)
        return 1
    profiles = read_profiles(path)
    if not profiles:
        print(f"profile is empty: {path}", file=sys.stderr)
        return 1

    totals = [int(profile.get("total_us", 0)) for profile in profiles if isinstance(profile.get("total_us"), int)]
    accounting_deltas = [
        int(profile.get("accounting_delta_us", 0))
        for profile in profiles
        if isinstance(profile.get("accounting_delta_us"), int)
    ]
    stages: dict[str, list[int]] = {}
    for profile in profiles:
        raw_stages = profile.get("stages")
        if not isinstance(raw_stages, dict):
            continue
        for name, value in raw_stages.items():
            if isinstance(name, str) and isinstance(value, int):
                stages.setdefault(name, []).append(value)

    print("Pipeline profile summary:")
    print(f"  path: {path}")
    print(f"  frames: {len(profiles)}")
    if totals:
        print(
            "  total_us: "
            f"min={min(totals)} median={statistics.median(totals):.1f} "
            f"p95={percentile(totals, 95):.1f} max={max(totals)}"
        )
    if accounting_deltas:
        print(
            "  accounting_delta_us: "
            f"median={statistics.median(accounting_deltas):.1f} "
            f"p95={percentile(accounting_deltas, 95):.1f}"
        )

    rows: list[tuple[float, str, list[int]]] = []
    for name, samples in stages.items():
        rows.append((percentile(samples, 95), name, samples))
    rows.sort(reverse=True)

    print("  stages:")
    for _, name, samples in rows[:top]:
        print(
            f"    {name}: "
            f"n={len(samples)} min={min(samples)} median={statistics.median(samples):.1f} "
            f"p95={percentile(samples, 95):.1f} max={max(samples)}"
        )
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("profile_jsonl", type=Path)
    parser.add_argument("--top", type=int, default=30, help="Number of slowest p95 stages to print")
    args = parser.parse_args()
    if args.top <= 0:
        parser.error("--top must be > 0")
    return summarize(args.profile_jsonl, args.top)


if __name__ == "__main__":
    raise SystemExit(main())
