#!/usr/bin/env python3
"""List AirSim scene objects and poses.

This is a discovery and inventory tool for binding existing AirSim environment
objects to Dedalus ground-truth obstacle evidence. It does not move objects,
publish events, or modify the simulator.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

try:
    import airsim
except ImportError as exc:
    raise SystemExit("airsim Python package is required: pip install airsim") from exc


DEFAULT_CLASS_PATTERNS = {
    "person": r"(?i)(person|human|pedestrian|character|mannequin|brplayer|player)",
    "car": r"(?i)(car|vehicle|truck|suv|sedan|van)",
    "drone": r"(?i)(drone|uav|multirotor)",
    "animal": r"(?i)(animal|animalaicontroller)",
    "tree": r"(?i)tree",
    "wall": r"(?i)wall",
    "fence": r"(?i)fence",
    "cable": r"(?i)(cable|wire|power[_-]?line|power|line)",
    "pole": r"(?i)(pole|post)",
    "rock": r"(?i)rock",
    "building": r"(?i)(building|house|structure)",
}

GEOMETRY_CLASS_BY_CANONICAL_CLASS = {
    "person": "volume",
    "car": "volume",
    "truck": "volume",
    "drone": "volume",
    "animal": "volume",
    "tree": "vertical_thin",
    "wall": "surface",
    "fence": "surface",
    "cable": "linear_thin",
    "pole": "vertical_thin",
    "rock": "volume",
    "building": "surface",
    "terrain": "surface",
    "unknown_obstacle": "unknown",
}

RECOMMENDED_SIZE_M_BY_CANONICAL_CLASS = {
    "person": [0.8, 0.8, 1.8],
    "car": [4.5, 2.0, 1.7],
    "truck": [6.0, 2.5, 2.5],
    "drone": [0.8, 0.8, 0.3],
    "animal": [1.2, 0.8, 1.0],
    "tree": [1.5, 1.5, 6.0],
    "wall": [4.0, 0.5, 2.5],
    "fence": [4.0, 0.25, 1.8],
    "cable": [0.15, 0.15, 4.0],
    "pole": [0.35, 0.35, 5.0],
    "rock": [1.5, 1.5, 1.0],
    "building": [8.0, 8.0, 6.0],
    "terrain": [4.0, 4.0, 0.25],
    "unknown_obstacle": [1.0, 1.0, 1.0],
}

THIN_GEOMETRY_CLASSES = {"linear_thin", "vertical_thin"}


@dataclass(frozen=True)
class ClassPattern:
    class_name: str
    regex_text: str
    regex: re.Pattern[str]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1", help="AirSim RPC host")
    parser.add_argument("--rpc-port", type=int, default=41451, help="AirSim RPC port")
    parser.add_argument("--vehicle-name", default="PX4", help="Vehicle name recorded in inventory metadata")
    parser.add_argument("--scene-id", default="airsim_scene", help="Stable scene id used in inventory output")
    parser.add_argument("--name-regex", default=".*", help="Regex passed to simListSceneObjects")
    parser.add_argument(
        "--class-pattern",
        action="append",
        default=[],
        metavar="CLASS=REGEX",
        help="Class matcher. May be repeated, e.g. --class-pattern car='(?i)car|vehicle'.",
    )
    parser.add_argument("--no-default-class-patterns", action="store_true", help="Disable built-in canonical class matchers")
    parser.add_argument("--only-matched", action="store_true", help="Only print objects matching at least one class pattern")
    parser.add_argument("--match-class", action="append", default=[], metavar="CLASS", help="Only print objects matched as this class. May be repeated. Implies --only-matched.")
    parser.add_argument("--limit", type=int, default=0, help="Maximum number of objects to print; 0 means unlimited")
    parser.add_argument("--format", choices=["json", "table", "names"], default="json")
    parser.add_argument("--inventory-schema", action="store_true", help="Emit canonical scene-inventory JSON schema instead of legacy discovery JSON")
    parser.add_argument("--output", type=Path, default=None, help="Write JSON output to this path instead of stdout")
    parser.add_argument("--sort", choices=["name", "class", "distance"], default="name")
    parser.add_argument("--indent", type=int, default=2, help="JSON indentation; use 0 for compact JSON")
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
            compiled.append(ClassPattern(class_name=class_name, regex_text=pattern, regex=re.compile(pattern)))
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


def canonical_class_from_matches(matched_classes: list[str]) -> str:
    return matched_classes[0] if matched_classes else "unknown_obstacle"


def geometry_class_for(canonical_class: str) -> str:
    return GEOMETRY_CLASS_BY_CANONICAL_CLASS.get(canonical_class, "unknown")


def recommended_size_for(canonical_class: str) -> list[float]:
    return RECOMMENDED_SIZE_M_BY_CANONICAL_CLASS.get(canonical_class, RECOMMENDED_SIZE_M_BY_CANONICAL_CLASS["unknown_obstacle"])


def class_pattern_map(patterns: list[ClassPattern]) -> dict[str, str]:
    out: dict[str, str] = {}
    for pattern in patterns:
        out[pattern.class_name] = pattern.regex_text
    return out


def list_objects(args: argparse.Namespace) -> dict[str, Any]:
    class_patterns = compile_class_patterns(args)
    requested_classes = {str(class_name).strip() for class_name in args.match_class if str(class_name).strip()}
    client = airsim.MultirotorClient(ip=args.host, port=args.rpc_port)
    client.confirmConnection()

    names = list(client.simListSceneObjects(args.name_regex))
    rows: list[dict[str, Any]] = []
    for name in names:
        matched_classes = match_classes(name, class_patterns)
        if requested_classes and requested_classes.isdisjoint(set(matched_classes)):
            continue
        if (args.only_matched or requested_classes) and not matched_classes:
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
        canonical_class = canonical_class_from_matches(matched_classes)
        geometry_class = geometry_class_for(canonical_class)
        row = {
            "name": name,
            "matched_classes": matched_classes,
            "canonical_class": canonical_class,
            "geometry_class": geometry_class,
            "recommended_size_m": recommended_size_for(canonical_class),
            "thin_obstacle": geometry_class in THIN_GEOMETRY_CLASSES,
            "source_pattern": class_pattern_map(class_patterns).get(canonical_class),
            "pose_available": pose_available,
            **pose_json,
            "distance_from_origin_m": round(distance_from_origin(position), 6) if pose_available else None,
        }
        if error is not None:
            row["error"] = error
        rows.append(row)

    if args.sort == "class":
        rows.sort(key=lambda row: (str(row.get("canonical_class") or "~"), row["name"]))
    elif args.sort == "distance":
        rows.sort(key=lambda row: (row["distance_from_origin_m"] is None, row["distance_from_origin_m"] or 0.0, row["name"]))
    else:
        rows.sort(key=lambda row: row["name"])

    if args.limit > 0:
        rows = rows[: args.limit]

    class_counts: dict[str, int] = {}
    unmatched = 0
    for row in rows:
        canonical_class = str(row.get("canonical_class") or "unknown_obstacle")
        class_counts[canonical_class] = class_counts.get(canonical_class, 0) + 1
        if canonical_class == "unknown_obstacle":
            unmatched += 1

    legacy_payload = {
        "schema_version": 1,
        "source": "airsim_scene_objects",
        "host": args.host,
        "rpc_port": args.rpc_port,
        "name_regex": args.name_regex,
        "requested_classes": sorted(requested_classes),
        "count": len(rows),
        "class_counts": class_counts,
        "unmatched_count": unmatched,
        "objects": rows,
    }
    if not args.inventory_schema:
        return legacy_payload

    return {
        "schema_version": 1,
        "scene_id": args.scene_id,
        "generated_at_unix_ns": time.time_ns(),
        "source": {
            "kind": "airsim_scene_inventory",
            "host": args.host,
            "rpc_port": args.rpc_port,
            "vehicle_name": args.vehicle_name,
            "generator": "simulation/airsim/scripts/airsim-list-objects.py",
            "name_regex": args.name_regex,
        },
        "class_patterns": class_pattern_map(class_patterns),
        "class_counts": class_counts,
        "unmatched_count": unmatched,
        "object_count": len(rows),
        "objects": rows,
    }


def emit_table(payload: dict[str, Any]) -> str:
    rows = payload["objects"]
    headers = ["class", "geom", "name", "x", "y", "z", "yaw_deg", "dist_m"]
    data: list[list[str]] = []
    for row in rows:
        canonical_class = str(row.get("canonical_class") or ",".join(row.get("matched_classes", [])) or "-")
        geometry_class = str(row.get("geometry_class") or "-")
        position = row.get("position_ned_m") or [None, None, None]
        rpy = row.get("orientation_rpy_deg") or [None, None, None]
        data.append(
            [
                canonical_class,
                geometry_class,
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
    count_key = "object_count" if "object_count" in payload else "count"
    lines.append(f"\ncount={payload[count_key]} class_counts={payload['class_counts']} unmatched={payload['unmatched_count']}")
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
