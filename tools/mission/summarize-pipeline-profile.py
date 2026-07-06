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
    "frame_source.detail.",  # attribution-only sub-stages of frame_source.next_frame_wait
)

DEFAULT_AGGREGATE_STAGE_NAMES = {
    "frame_source.next_frame_wait",
}

# Stage grouping for the friendly summary view.
# Each entry: (group label, list of name-prefixes that belong to it).
# A stage is placed in the first matching group; ungrouped stages fall into "Other".
STAGE_GROUPS: list[tuple[str, list[str]]] = [
    ("Frame IO", [
        "frame_source.",
    ]),
    ("GPU / Depth", [
        "depth_slot_a.",
        "depth_slot_b.",
    ]),
    ("ESDF (L3)", [
        "esdf.",
    ]),
    ("World Model / Map", [
        "mission_local_obstacle_map.",
        "mission_map_assimilator.",
        "planning_map.",
        "local_flight_map.",
        "world_model.",
    ]),
    ("Publishing", [
        "snapshot_publisher.",
        "traversability_map_publisher.",
        "ghost_detections.",
        "mission_obstacle_map_delta_writer.",
    ]),
    ("Perception", [
        "perception_pipeline.",
        "sensing_coverage.",
        "ghost_targets.",
        "ego_provider.",
    ]),
    ("Safety / Annotation", [
        "trajectory_safety.",
        "frame_annotator.",
        "perch_candidate.",
    ]),
]


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


def ms(us: float) -> str:
    """Format microseconds as a right-aligned ms string."""
    return f"{us / 1000.0:6.1f}ms"


def pct_of(part: float, whole: float) -> str:
    if whole <= 0:
        return "  -  "
    return f"{100.0 * part / whole:4.0f}%"


def stage_group(name: str) -> str:
    for group_label, prefixes in STAGE_GROUPS:
        if any(name.startswith(p) for p in prefixes):
            return group_label
    return "Other"


def summarize_samples(label: str, samples: list[int]) -> None:
    print(
        f"  {label}: "
        f"min={min(samples)} median={statistics.median(samples):.1f} "
        f"p95={percentile(samples, 95):.1f} max={max(samples)}"
    )


def print_friendly(
    path: Path,
    profiles: list[dict[str, Any]],
    stages: dict[str, list[int]],
    totals: list[int],
) -> None:
    W = 72  # line width

    def rule(char: str = "━") -> None:
        print(char * W)

    def header_line(text: str) -> None:
        print(f" {text}")

    rule()
    header_line(f"PIPELINE PROFILE  ·  {path.name}")
    header_line(f"{len(profiles)} frames")
    rule()

    # ── Throughput ────────────────────────────────────────────────────────────
    med_total = statistics.median(totals) if totals else 0.0
    fps_approx = 1_000_000.0 / med_total if med_total > 0 else 0.0

    io_samples   = stages.get("frame_source.next_frame_wait", [])
    comp_samples = stages.get("runtime.post_frame_compute", [])
    gpu_samples  = stages.get("depth_slot_a.detect", [])

    # Derive per-frame CPU time = post_frame_compute - depth_slot_a.detect.
    # Only include frames where both stages were recorded.
    cpu_samples: list[int] = []
    for profile in profiles:
        raw = profile.get("stages")
        if not isinstance(raw, dict):
            continue
        comp_v = raw.get("runtime.post_frame_compute")
        gpu_v  = raw.get("depth_slot_a.detect")
        if isinstance(comp_v, int) and isinstance(gpu_v, int):
            cpu_samples.append(max(0, comp_v - gpu_v))

    med_io   = statistics.median(io_samples)   if io_samples   else 0.0
    med_comp = statistics.median(comp_samples) if comp_samples else 0.0
    med_gpu  = statistics.median(gpu_samples)  if gpu_samples  else 0.0
    med_cpu  = statistics.median(cpu_samples)  if cpu_samples  else 0.0

    p95_total = percentile(totals, 95)       if totals        else 0.0
    p95_io    = percentile(io_samples, 95)   if io_samples    else 0.0
    p95_comp  = percentile(comp_samples, 95) if comp_samples  else 0.0
    p95_gpu   = percentile(gpu_samples, 95)  if gpu_samples   else 0.0
    p95_cpu   = percentile(cpu_samples, 95)  if cpu_samples   else 0.0

    print()
    print(f"  THROUGHPUT    ≈ {fps_approx:.1f} fps   (1 / {med_total/1000:.1f}ms median frame time)")
    print()

    # Frame budget tree
    col = 38
    hdr = f"  {'FRAME BUDGET':<{col}}{'p50':>8}  {'p95':>8}  {'max':>8}"
    print(hdr)
    print(f"  {'-'*W}")

    max_total = max(totals) if totals else 0

    def budget_row(label: str, med: float, p95: float, mx: float, pct_str: str = "") -> None:
        suffix = f"  {pct_str}" if pct_str else ""
        print(f"  {label:<{col}}{ms(med):>8}  {ms(p95):>8}  {ms(mx):>8}{suffix}")

    budget_row("total", med_total, p95_total, max_total)
    if io_samples:
        budget_row(
            f"  ├─ io wait (camera)", med_io, p95_io,
            max(io_samples), pct_of(med_io, med_total),
        )
    if comp_samples:
        budget_row(
            f"  └─ compute", med_comp, p95_comp,
            max(comp_samples), pct_of(med_comp, med_total),
        )
    if gpu_samples:
        budget_row(
            f"       ├─ GPU (depth_slot_a)", med_gpu, p95_gpu,
            max(gpu_samples), pct_of(med_gpu, med_comp),
        )
    if cpu_samples:
        budget_row(
            f"       └─ CPU residual (compute − GPU)", med_cpu, p95_cpu,
            max(cpu_samples), pct_of(med_cpu, med_comp),
        )

    # ── Stage breakdown by group ───────────────────────────────────────────────
    print()
    rule("─")

    # Assign each stage to a group, collect rows
    grouped: dict[str, list[tuple[float, str, list[int], bool]]] = {}
    for name, samples in stages.items():
        g = stage_group(name)
        agg = is_aggregate_stage(name)
        grouped.setdefault(g, []).append((percentile(samples, 95), name, samples, agg))

    # Print groups in STAGE_GROUPS order, then "Other", then "runtime.*"
    group_order = [g for g, _ in STAGE_GROUPS] + ["Other"]

    col_name = 42
    print(f"  {'STAGE':<{col_name}}{'p50':>8}  {'p95':>8}  {'max':>8}  {'n':>5}")

    for group_label in group_order:
        rows = grouped.get(group_label)
        if not rows:
            continue
        rows.sort(reverse=True)  # sort by p95 descending within group

        print(f"\n  ── {group_label} {'─' * max(0, W - len(group_label) - 7)}")
        for _, name, samples, agg in rows:
            tag = "†" if agg else " "
            n = len(samples)
            med = statistics.median(samples)
            p95v = percentile(samples, 95)
            mx = max(samples)
            # For count-only stages (e.g. evidence_count whose values are counts not durations),
            # the numbers are already small integers; still format the same way.
            freq = ""
            if n < len(profiles):
                freq = f"  every ~{len(profiles)//n} frames"
            print(
                f"  {tag}{name:<{col_name}}{ms(med):>8}  {ms(p95v):>8}  {ms(mx):>8}  {n:>5}{freq}"
            )

    # Runtime aggregates at the end (they overlap leaf stages — for reference only)
    runtime_rows = [
        (percentile(s, 95), n, s, True)
        for n, s in stages.items()
        if n.startswith("runtime.")
    ]
    if runtime_rows:
        runtime_rows.sort(reverse=True)
        print(f"\n  ── Runtime aggregates (overlap leaf stages — reference only) {'─'*4}")
        for _, name, samples, _ in runtime_rows:
            n = len(samples)
            print(
                f"  †{name:<{col_name}}{ms(statistics.median(samples)):>8}"
                f"  {ms(percentile(samples, 95)):>8}  {ms(max(samples)):>8}  {n:>5}"
            )

    print()
    print("  † aggregate stage (overlaps leaf stages; excluded from accounting)")
    rule()


def summarize(
    path: Path,
    top: int,
    include_aggregates_in_accounting: bool,
    show_aggregates: bool,
    friendly: bool,
) -> int:
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

    if friendly:
        print_friendly(path, profiles, stages, totals)
        return 0

    # ── Legacy raw format ─────────────────────────────────────────────────────
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
    parser.add_argument("--raw", action="store_true",
                        help="Raw output: flat stage list sorted by p95 (legacy format)")
    parser.add_argument("--top", type=int, default=30,
                        help="(--raw only) number of slowest p95 stages to print")
    parser.add_argument("--hide-aggregates", action="store_true",
                        help="(--raw only) hide aggregate/overlapping stages")
    parser.add_argument("--include-aggregates-in-accounting", action="store_true",
                        help="(--raw only) also print aggregate-inclusive accounting deltas")
    args = parser.parse_args()
    if args.top <= 0:
        parser.error("--top must be > 0")
    return summarize(
        args.profile_jsonl,
        args.top,
        args.include_aggregates_in_accounting,
        not args.hide_aggregates,
        friendly=not args.raw,
    )


if __name__ == "__main__":
    raise SystemExit(main())
