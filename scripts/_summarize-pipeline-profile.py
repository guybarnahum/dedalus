#!/usr/bin/env python3
"""Summarize Dedalus pipeline_profile.jsonl stage timings."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

DETAIL_PREFIX = "frame_source.detail."
FRAME_SOURCE_PARENT = "frame_source.next_frame"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("profile_jsonl", type=Path, help="Path to pipeline_profile.jsonl.")
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
    parser.add_argument("--descending", action="store_true", help="Sort descending.")
    parser.add_argument("--flat", action="store_true", help="Print all stages as a flat list.")
    parser.add_argument("--no-frames-line", action="store_true", help="Do not print the leading frames line.")
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
        if not isinstance(row.get("stages"), dict):
            raise ValueError(f"line {line_number} missing stages object")
        rows.append(row)
    if not rows:
        raise ValueError(f"pipeline profile JSONL has no rows: {path}")
    return rows


def collect_stage_values(rows: list[dict]) -> dict[str, list[int]]:
    values: dict[str, list[int]] = {}
    for row_index, row in enumerate(rows, start=1):
        for name, value in row["stages"].items():
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


def sorted_names(names: list[str], stage_values: dict[str, list[int]], sort_by: str, descending: bool) -> list[str]:
    ordered = list(names)
    if sort_by == "name":
        ordered.sort(reverse=descending)
    else:
        ordered.sort(key=lambda name: metric_ms(stage_values[name], sort_by), reverse=descending)
    return ordered


def format_stage(name: str, values: list[int], indent: str = "") -> str:
    return (
        f"{indent}{name}: "
        f"mean_ms={metric_ms(values, 'mean'):.3f} "
        f"p95_ms={metric_ms(values, 'p95'):.3f} "
        f"p99_ms={metric_ms(values, 'p99'):.3f} "
        f"max_ms={metric_ms(values, 'max'):.3f}"
    )


def print_flat(names: list[str], stage_values: dict[str, list[int]], sort_by: str, descending: bool) -> None:
    for name in sorted_names(names, stage_values, sort_by, descending):
        print(format_stage(name, stage_values[name]))


def print_hierarchical(names: list[str], stage_values: dict[str, list[int]], sort_by: str, descending: bool) -> None:
    detail_names = [name for name in names if name.startswith(DETAIL_PREFIX)]
    top_level = [name for name in names if not name.startswith(DETAIL_PREFIX)]
    emitted: set[str] = set()

    for name in sorted_names(top_level, stage_values, sort_by, descending):
        print(format_stage(name, stage_values[name]))
        emitted.add(name)
        if name == FRAME_SOURCE_PARENT:
            for detail in sorted_names(detail_names, stage_values, sort_by, descending):
                short = detail[len(DETAIL_PREFIX):]
                print(format_stage(short, stage_values[detail], indent="\t"))
                emitted.add(detail)

    for name in sorted_names([name for name in names if name not in emitted], stage_values, sort_by, descending):
        if name.startswith(DETAIL_PREFIX):
            print(format_stage(name[len(DETAIL_PREFIX):], stage_values[name], indent="\t"))
        else:
            print(format_stage(name, stage_values[name]))


def main() -> int:
    args = parse_args()
    rows = load_rows(args.profile_jsonl)
    stage_values = collect_stage_values(rows)
    selected = set(args.stages or stage_values.keys())
    missing = sorted(selected - set(stage_values.keys()))
    if missing:
        raise ValueError(f"requested missing stage(s): {', '.join(missing)}")
    names = [name for name in stage_values if name in selected]
    if not args.no_frames_line:
        print(f"frames: {len(rows)}")
    if args.flat:
        print_flat(names, stage_values, args.sort, args.descending)
    else:
        print_hierarchical(names, stage_values, args.sort, args.descending)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"_summarize-pipeline-profile: {exc}", file=sys.stderr)
        raise SystemExit(1)
