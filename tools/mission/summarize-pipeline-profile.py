#!/usr/bin/env python3
"""Summarize Dedalus pipeline profiler JSONL output.

The profiler writes one JSON object per processed frame with a `stages` object
mapping stage names to microsecond durations. This tool aggregates those stage
samples so profiling baselines can be compared before and after runtime changes.

Some stages are aggregate/overlap stages, for example `runtime.post_frame_compute`
or `runtime.frame_source_reported_io`. These are useful for diagnosis, but they
must not be included in leaf-stage accounting checks because they intentionally
overlap with lower-level stages.
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
from pathlib import Path
from typing import Any

DEFAULT_AGGREGATE_PREFIXES = (
    "runtime.",
)

DEFAULT_AGGREGATE_STAGE_NAMES = {
    "frame_source.next_frame_wait",
}


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


def is_aggregate_stage(name: str) -> bool:
    if name in DEFAULT_AGGREGATE_STAGE_NAMES:
        return True
    return any(name.startswith(prefix) for prefix in DEFAULT_AGGREGATE_PREFIXES)


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


def summarize_samples(label: str, samples: list[int]) -> None:
    print(
        f"  {label}: "
        f"min={min(samples)} median={statistics.median(samples):.1f} "
        f"p95={percentile(samples, 95):.1f} max={max(samples)}"
    )


def summarize(path: Path, top: int, include_aggregates_in_accounting: bool, show_aggregates: bool) -> int:
    if not path.exists():
        print(f"profile not found: {path}", file=sys.stderr)
        return 1
    profiles = read_profiles(path)
    if not profiles:
        print(f"profile is empty: {path}", file=sys.stderr)
        return 1

    totals = [int(profile.get("total_us", 0)) for profile in profiles if isinstance(profile.get("total_us"), int)]
    recorded_accounting_deltas = [
        int(profile.get("accounting_delta_us", 0))
        for profile in profiles
        if isinstance(profile.get("accounting_delta_us"), int)
    ]
    stages: dict[str, list[int]] = {}
    leaf_accounting_deltas: list[int] = []
    aggregate_accounting_deltas: list[int] = []
    for profile in profiles:
        raw_stages = profile.get("stages")
        if not isinstance(raw_stages, dict):
            continue
        leaf_sum = 0
        aggregate_sum = 0
        for name, value in raw_stages.items():
            if isinstance(name, str) and isinstance(value, int):
                stages.setdefault(name, []).append(value)
                if is_aggregate_stage(name):
                    aggregate_sum += value
                else:
                    leaf_sum += value
        total = profile.get("total_us")
        if isinstance(total, int):
            leaf_accounting_deltas.append(total - leaf_sum)
            aggregate_accounting_deltas.append(total - leaf_sum - aggregate_sum)

    print("Pipeline profile summary:")
    print(f"  path: {path}")
    print(f"  frames: {len(profiles)}")
    if totals:
        summarize_samples("total_us", totals)
    if leaf_accounting_deltas:
        summarize_samples("leaf_accounting_delta_us", leaf_accounting_deltas)
    if recorded_accounting_deltas and include_aggregates_in_accounting:
        summarize_samples("recorded_accounting_delta_us", recorded_accounting_deltas)
    elif recorded_accounting_deltas:
        print("  recorded_accounting_delta_us: hidden because aggregate stages intentionally overlap leaf stages; pass --include-aggregates-in-accounting to show")
    if aggregate_accounting_deltas and include_aggregates_in_accounting:
        summarize_samples("aggregate_inclusive_accounting_delta_us", aggregate_accounting_deltas)

    rows: list[tuple[float, str, list[int], bool]] = []
    for name, samples in stages.items():
        aggregate = is_aggregate_stage(name)
        if aggregate and not show_aggregates:
            continue
        rows.append((percentile(samples, 95), name, samples, aggregate))
    rows.sort(reverse=True)

    print("  stages:")
    for _, name, samples, aggregate in rows[:top]:
        tag = " aggregate" if aggregate else ""
        print(
            f"    {name}{tag}: "
            f"n={len(samples)} min={min(samples)} median={statistics.median(samples):.1f} "
            f"p95={percentile(samples, 95):.1f} max={max(samples)}"
        )
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("profile_jsonl", type=Path)
    parser.add_argument("--top", type=int, default=30, help="Number of slowest p95 stages to print")
    parser.add_argument("--hide-aggregates", action="store_true", help="Hide aggregate/overlapping stages from the stage list")
    parser.add_argument("--include-aggregates-in-accounting", action="store_true", help="Also print the old aggregate-inclusive accounting deltas")
    args = parser.parse_args()
    if args.top <= 0:
        parser.error("--top must be > 0")
    return summarize(
        args.profile_jsonl,
        args.top,
        args.include_aggregates_in_accounting,
        not args.hide_aggregates,
    )


if __name__ == "__main__":
    raise SystemExit(main())
