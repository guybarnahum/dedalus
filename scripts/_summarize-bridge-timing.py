#!/usr/bin/env python3
"""Summarize AirSim bridge-internal timing JSONL.

Input is produced by simulation/airsim-stream-frames-binary.py --timing-jsonl.
The script intentionally has no third-party dependencies.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

DEFAULT_FIELDS = [
    "sim_get_images_ms",
    "ego_sample_ms",
    "rgb_convert_ms",
    "stdout_write_ms",
    "sleep_ms",
    "total_loop_ms",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("timing_jsonl", type=Path)
    parser.add_argument(
        "--field",
        action="append",
        dest="fields",
        help="Timing field to include. May be repeated. Defaults to core bridge timing fields.",
    )
    parser.add_argument(
        "--sort",
        choices=["name", "mean", "p95", "p99", "max"],
        default="name",
    )
    parser.add_argument("--descending", action="store_true")
    return parser.parse_args()


def percentile(values: list[float], p: float) -> float:
    ordered = sorted(values)
    if not ordered:
        raise ValueError("percentile requires values")
    return ordered[round((len(ordered) - 1) * p)]


def load_rows(path: Path) -> list[dict]:
    if not path.exists():
        raise FileNotFoundError(f"missing bridge timing JSONL: {path}")
    rows = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        stripped = line.strip()
        if not stripped:
            continue
        try:
            rows.append(json.loads(stripped))
        except json.JSONDecodeError as exc:
            raise ValueError(f"bad JSON on line {line_number}: {exc}") from exc
    if not rows:
        raise ValueError(f"bridge timing JSONL has no rows: {path}")
    return rows


def metric(values: list[float], name: str) -> float:
    if name == "mean":
        return sum(values) / len(values)
    if name == "p95":
        return percentile(values, 0.95)
    if name == "p99":
        return percentile(values, 0.99)
    if name == "max":
        return max(values)
    raise ValueError(f"unknown metric: {name}")


def main() -> int:
    args = parse_args()
    rows = load_rows(args.timing_jsonl)
    fields = args.fields or DEFAULT_FIELDS

    values_by_field: dict[str, list[float]] = {}
    for field in fields:
        values = []
        for row in rows:
            if field in row:
                value = row[field]
                if not isinstance(value, (int, float)):
                    raise ValueError(f"field {field} has non-numeric value: {value!r}")
                values.append(float(value))
        if values:
            values_by_field[field] = values

    missing = [field for field in fields if field not in values_by_field]
    if missing:
        raise ValueError(f"requested missing field(s): {', '.join(missing)}")

    names = list(values_by_field)
    if args.sort == "name":
        names.sort(reverse=args.descending)
    else:
        names.sort(key=lambda field: metric(values_by_field[field], args.sort), reverse=args.descending)

    print(f"frames: {len(rows)}")
    for name in names:
        values = values_by_field[name]
        print(
            f"{name}: "
            f"mean_ms={metric(values, 'mean'):.3f} "
            f"p95_ms={metric(values, 'p95'):.3f} "
            f"p99_ms={metric(values, 'p99'):.3f} "
            f"max_ms={metric(values, 'max'):.3f}"
        )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:  # noqa: BLE001 - dependency-free CLI.
        print(f"_summarize-bridge-timing: {exc}", file=sys.stderr)
        raise SystemExit(1)
