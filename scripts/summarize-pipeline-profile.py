#!/usr/bin/env python3
"""Summarize Dedalus pipeline_profile.jsonl stage timings.

The input is produced by `pipeline_timing_enabled: true` in core-stack configs.
This script intentionally has no third-party dependencies so it can be used on
CI, EC2 validation machines, and ad-hoc profiling runs.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "profile_jsonl",
        type=Path,
        help="Path to pipeline_profile.jsonl.",
    )
    parser.add_argument(
        "--stage",
        action="append",
        dest="stages",
        help="Stage name to include. May be repeated. Defaults to all stages.",
    )
    parser.add_argument(
        "--sort",
        choices=["name", "mean", "p95", "p99", "max"],
        default="name",
        help="Sort order. Default: name.",
    )
    parser.add_argument(
        "--descending",
        action="store_true",
        help="Sort descending. Useful with --sort p95 or --sort mean.",
    )
    parser.add_argument(
        "--no-frames-line",
        action="store_true",
        help="Do not print the leading frames: N line.",
    )
    return parser.parse_args()


def percentile(values: list[int], p: float) -> int:
    ordered = sorted(values)
    if not ordered:
        raise ValueError("percentile requires at least one value")
    return ordered[round((len(ordered) - 1) * p)]


def load_rows(path: Path) -> list[dict]:
    if not path.exists():
        raise FileNotFoundError(f"missing pipeline profile JSONL: {path}")

    rows: list[dict] = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        stripped = line.strip()
        if not stripped:
            continue
        try:
            row = json.loads(stripped)
        except json.JSONDecodeError as exc:
            raise ValueError(f"bad JSON on line {line_number}: {exc}") from exc
        stages = row.get("stages")
        if not isinstance(stages, dict):
            raise ValueError(f"line {line_number} missing stages object")
        rows.append(row)

    if not rows:
        raise ValueError(f"pipeline profile JSONL has no rows: {path}")
    return rows


def collect_stage_values(rows: list[dict]) -> dict[str, list[int]]:
    values: dict[str, list[int]] = {}
    for row_index, row in enumerate(rows, start=1):
        stages = row["stages"]
        for name, value in stages.items():
            if not isinstance(name, str) or not isinstance(value, int):
                raise ValueError(f"row {row_index} has invalid stage timing: {name}={value!r}")
            if value < 0:
                raise ValueError(f"row {row_index} has negative stage timing: {name}={value}")
            values.setdefault(name, []).append(value)
    return values


def metric_ms(values: list[int], metric: str) -> float:
    if metric == "mean":
        return sum(values) / len(values) / 1000.0
    if metric == "p95":
        return percentile(values, 0.95) / 1000.0
    if metric == "p99":
        return percentile(values, 0.99) / 1000.0
    if metric == "max":
        return max(values) / 1000.0
    raise ValueError(f"unknown metric: {metric}")


def main() -> int:
    args = parse_args()
    rows = load_rows(args.profile_jsonl)
    stage_values = collect_stage_values(rows)

    selected = set(args.stages or stage_values.keys())
    missing = sorted(selected - set(stage_values.keys()))
    if missing:
        raise ValueError(f"requested missing stage(s): {', '.join(missing)}")

    names = [name for name in stage_values if name in selected]
    if args.sort == "name":
        names.sort(reverse=args.descending)
    else:
        names.sort(
            key=lambda name: metric_ms(stage_values[name], args.sort),
            reverse=args.descending,
        )

    if not args.no_frames_line:
        print(f"frames: {len(rows)}")

    for name in names:
        values = stage_values[name]
        print(
            f"{name}: "
            f"mean_ms={metric_ms(values, 'mean'):.3f} "
            f"p95_ms={metric_ms(values, 'p95'):.3f} "
            f"p99_ms={metric_ms(values, 'p99'):.3f} "
            f"max_ms={metric_ms(values, 'max'):.3f}"
        )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:  # noqa: BLE001 - keep dependency-free CLI concise.
        print(f"summarize-pipeline-profile: {exc}", file=sys.stderr)
        raise SystemExit(1)
