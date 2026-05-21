#!/usr/bin/env python3
"""List AirSim scene objects and poses.

This is a discovery tool for binding existing AirSim environment objects to
Dedalus ghost detections later. It does not move objects, publish events, or
modify the simulator.

Examples:

  python3 simulation/airsim-list-objects.py

  python3 simulation/airsim-list-objects.py \
    --name-regex '.*(Car|Vehicle|Person|Human|Pedestrian).*' \
    --class-pattern car='(?i)(car|vehicle)' \
    --class-pattern person='(?i)(person|human|pedestrian)' \
    --format table

  python3 simulation/airsim-list-objects.py \
    --only-matched \
    --output out/airsim_scene_objects.json
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

try:
    import airsim
except ImportError as exc:
    raise SystemExit("airsim Python package is required: pip install airsim") from exc


DEFAULT_CLASS_PATTERNS = {
    "car": r"(?i)(car|vehicle|truck|suv|sedan|van)",
    "person": r"(?i)(person|human|pedestrian|character|mannequin)",
}


@dataclass(frozen=True)
class ClassPattern:
    class_name: str
    regex: re.Pattern[str]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1", help="AirSim RPC host")
    parser.add_argument("--rpc-port", type=int, default=41451, help="AirSim RPC port")
    parser.add_argument("--name-regex", default=".*", help="Regex passed to simListSceneObjects")
    parser.add_argument(
        "--class-pattern",
        action="append",
        default=[],
        metavar="CLASS=REGEX",
        help="Class matcher. May be repeated, e.g. --class-pattern car='(?i)car|vehicle'.",
    )
    parser.add_argument(
        "--no-default-class-patterns",
        action="store_true",
        help="Disable built-in car/person name matchers.",
    )
    parser.add_argument(
        "--only-matched",
        action="store_true",
        help="Only print objects matching at least one class pattern.",
    )
    parser.add_argument("--limit", type=int, default=0, help="Maximum number of objects to print; 0 means unlimited.")
    parser.add_argument("--format", choices=["json", "table", "names"], default="json")
    parser.add_argument("--output", type=Path, default=None, help="Write JSON output to this path instead of stdout.")
    parser.add_argument("--sort", choices=["name", "class", "distance"], default="name")
    parser.add_argument("--indent", type=int, default=2, help="JSON indentation; use 0 for compact JSON.")
    return parser.parse_args()


def compile_class_patterns(args: argparse.Namespace) -> list[ClassPattern]:
    raw_patterns: list[tuple[str, str]] = []
    if not args.no_default_class_patterns:
        raw_patterns.extend(DEFAULT_CLASS_PATTERNS.items())
    for raw in args.class_pattern:
        if "=" not in raw:
            raise ValueError(f"--class-pattern must be CLASS=REGEX, got: {raw!r}")
        class_name, pattern = raw.split("=", 1)
        class_name = class_name.strip()
        pattern = pattern.strip()
        if not class_name or not pattern:
            raise ValueError(f"--class-pattern must be CLASS=REGEX, got: {raw!r}")
        raw_patterns.append((class_name, pattern))

    compiled: list[ClassPattern] = []
    for class_name, pattern in raw_patterns:
        try:
            compiled.append(ClassPattern(class_name=class_name, regex=re.compile(pattern)))
        except re.error as exc:
            raise ValueError(f"invalid regex for class {class_name!r}: {pattern!r}: {exc}") from exc
    return compiled


def pose_to_dict(pose: Any) -> dict[str, Any]:
    position = pose.position
    orientation = pose.orientation
    roll = pitch = yaw = None
    try:
        roll, pitch, yaw = airsim.to_eularian_angles(orientation)
    except Exception:
        pass

    result: dict[str, Any] = {
        "position_ned_m": [round(float(position.x_val), 6), round(float(position.y_val), 6), round(float(position.z_val), 6)],
        "orientation_quat_xyzw": [
            round(float(orientation.x_val), 8),
            round(float(orientation.y_val), 8),
            round(float(orientation.z_val), 8),
            round(float(orientation.w_val), 8),
        ],
    }
    if roll is not None and pitch is not None and yaw is not None:
        result["orientation_rpy_deg"] = [
            round(math.degrees(float(roll)), 4),
            round(math.degrees(float(pitch)), 4),
            round(math.degrees(float(yaw)), 4),
        ]
    return result


def distance_from_origin(position_ned_m: list[float]) -> float:
    return math.sqrt(sum(float(component) * float(component) for component in position_ned_m))


def match_classes(name: str, patterns: list[ClassPattern]) -> list[str]:
    matches: list[str] = []
    for pattern in patterns:
        if pattern.regex.search(name):
            matches.append(pattern.class_name)
    return matches


def list_objects(args: argparse.Namespace) -> dict[str, Any]:
    class_patterns = compile_class_patterns(args)
    client = airsim.MultirotorClient(ip=args.host, port=args.rpc_port)
    client.confirmConnection()

    names = list(client.simListSceneObjects(args.name_regex))
    rows: list[dict[str, Any]] = []
    for name in names:
        matched_classes = match_classes(name, class_patterns)
        if args.only_matched and not matched_classes:
            continue
        try:
            pose = client.simGetObjectPose(name)
            pose_json = pose_to_dict(pose)
            pose_available = True
            error = None
        except Exception as exc:
            pose_json = {}
            pose_available = False
            error = str(exc)

        position = pose_json.get("position_ned_m", [0.0, 0.0, 0.0])
        row = {
            "name": name,
            "matched_classes": matched_classes,
            "pose_available": pose_available,
            **pose_json,
            "distance_from_origin_m": round(distance_from_origin(position), 6) if pose_available else None,
        }
        if error is not None:
            row["error"] = error
        rows.append(row)

    if args.sort == "class":
        rows.sort(key=lambda row: (",".join(row["matched_classes"]) if row["matched_classes"] else "~", row["name"]))
    elif args.sort == "distance":
        rows.sort(key=lambda row: (row["distance_from_origin_m"] is None, row["distance_from_origin_m"] or 0.0, row["name"]))
    else:
        rows.sort(key=lambda row: row["name"])

    if args.limit > 0:
        rows = rows[: args.limit]

    class_counts: dict[str, int] = {}
    unmatched = 0
    for row in rows:
        if not row["matched_classes"]:
            unmatched += 1
        for class_name in row["matched_classes"]:
            class_counts[class_name] = class_counts.get(class_name, 0) + 1

    return {
        "schema_version": 1,
        "source": "airsim_scene_objects",
        "host": args.host,
        "rpc_port": args.rpc_port,
        "name_regex": args.name_regex,
        "count": len(rows),
        "class_counts": class_counts,
        "unmatched_count": unmatched,
        "objects": rows,
    }


def emit_table(payload: dict[str, Any]) -> str:
    rows = payload["objects"]
    headers = ["class", "name", "x", "y", "z", "yaw_deg", "dist_m"]
    data: list[list[str]] = []
    for row in rows:
        classes = ",".join(row.get("matched_classes", [])) or "-"
        position = row.get("position_ned_m") or [None, None, None]
        rpy = row.get("orientation_rpy_deg") or [None, None, None]
        data.append(
            [
                classes,
                str(row.get("name", "")),
                "-" if position[0] is None else f"{float(position[0]):.2f}",
                "-" if position[1] is None else f"{float(position[1]):.2f}",
                "-" if position[2] is None else f"{float(position[2]):.2f}",
                "-" if rpy[2] is None else f"{float(rpy[2]):.1f}",
                "-" if row.get("distance_from_origin_m") is None else f"{float(row['distance_from_origin_m']):.2f}",
            ]
        )

    widths = [len(header) for header in headers]
    for row in data:
        for index, value in enumerate(row):
            widths[index] = max(widths[index], len(value))

    def fmt(row: list[str]) -> str:
        return "  ".join(value.ljust(widths[index]) for index, value in enumerate(row))

    lines = [fmt(headers), fmt(["-" * width for width in widths])]
    lines.extend(fmt(row) for row in data)
    lines.append(f"\ncount={payload['count']} class_counts={payload['class_counts']} unmatched={payload['unmatched_count']}")
    return "\n".join(lines) + "\n"


def emit_names(payload: dict[str, Any]) -> str:
    return "\n".join(row["name"] for row in payload["objects"]) + ("\n" if payload["objects"] else "")


def main() -> int:
    args = parse_args()
    payload = list_objects(args)

    if args.format == "table":
        text = emit_table(payload)
    elif args.format == "names":
        text = emit_names(payload)
    else:
        indent = None if args.indent == 0 else args.indent
        text = json.dumps(payload, indent=indent, sort_keys=False) + "\n"

    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    else:
        print(text, end="")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nairsim-list-objects: interrupted", file=sys.stderr)
        raise SystemExit(130)
    except Exception as exc:
        print(f"airsim-list-objects: {exc}", file=sys.stderr)
        raise SystemExit(1)
