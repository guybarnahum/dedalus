#!/usr/bin/env python3
"""Validate obstacle sensing and obstacle evidence fields in mission snapshots.

This is an operator-facing validation helper for live or synthetic mission output
written by dedalus_mission_loop / ArtifactSnapshotWriter. It accepts either a
snapshot directory, an artifact root containing one or more snapshot directories,
or no path at all, in which case it searches common local output roots.

Examples:

    python3 tools/mission/validate-obstacle-sensing-evidence-snapshots.py
    python3 tools/mission/validate-obstacle-sensing-evidence-snapshots.py out/object_behavior_airsim_existing_object_circle
    python3 tools/mission/validate-obstacle-sensing-evidence-snapshots.py out --min-evidence 10

The validator intentionally checks the normalized world-model obstacle sensing
contract rather than specific object classes: obstacle_sensing_volumes and
obstacle_evidence.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys
from collections import Counter
from typing import Any


REQUIRED_VOLUME_KEYS = (
    "sensor_name",
    "provider_name",
    "source_frame_id",
    "has_source_frame",
    "origin_local",
    "forward_axis_local",
    "right_axis_local",
    "up_axis_local",
    "near_range_m",
    "far_range_m",
    "min_reliable_range_m",
    "max_reliable_range_m",
    "horizontal_fov_rad",
    "vertical_fov_rad",
)

REQUIRED_EVIDENCE_KEYS = (
    "center_local",
    "source_kind",
    "source_provider",
    "state",
    "shape",
    "confidence",
    "range_m",
    "bearing_rad",
    "elevation_rad",
)

INVALID_VOLUME_PROVIDERS = {"airsim_gt_visual_emulation"}
GT_VISUAL_SOURCE_KIND = "airsim_gt_visual_emulation"


def load_json(path: pathlib.Path) -> dict[str, Any]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:  # pragma: no cover - exercised by operator use.
        raise RuntimeError(f"failed to read {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise RuntimeError(f"snapshot is not a JSON object: {path}")
    return data


def snapshot_files(snapshot_dir: pathlib.Path) -> list[pathlib.Path]:
    manifest = snapshot_dir / "snapshot_manifest.txt"
    if manifest.exists():
        files: list[pathlib.Path] = []
        for line in manifest.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            # Current ArtifactSnapshotWriter writes: index path timestamp map_frame.
            # Some older tests wrote only the snapshot path. Support both.
            candidate = parts[1] if len(parts) >= 2 and parts[0].isdigit() else parts[0]
            files.append(snapshot_dir / candidate)
        files = [path for path in files if path.exists()]
        if files:
            return files
    return sorted(snapshot_dir.glob("snapshot_*.json"))


def find_snapshot_dirs(root: pathlib.Path) -> list[pathlib.Path]:
    manifests = sorted(root.rglob("snapshot_manifest.txt"))
    dirs = [path.parent for path in manifests]
    if root.is_dir() and list(root.glob("snapshot_*.json")):
        dirs.insert(0, root)

    seen: set[pathlib.Path] = set()
    unique_dirs: list[pathlib.Path] = []
    for path in dirs:
        if path not in seen:
            unique_dirs.append(path)
            seen.add(path)
    return unique_dirs


def common_roots() -> list[pathlib.Path]:
    return [pathlib.Path("out"), pathlib.Path("simulation"), pathlib.Path(".")]


def vector3(value: Any) -> list[float] | None:
    if not isinstance(value, list) or len(value) != 3:
        return None
    try:
        return [float(value[0]), float(value[1]), float(value[2])]
    except (TypeError, ValueError):
        return None


def positive_float(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    if result <= 0.0:
        return None
    return result


def finite_float(value: Any) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def validate_dir(
    snapshot_dir: pathlib.Path,
    min_evidence: int,
    require_gt_visual: bool,
    require_inside_swept: bool,
    max_errors: int,
) -> tuple[bool, list[str]]:
    files = snapshot_files(snapshot_dir)
    if not files:
        return False, [f"no snapshot_*.json files in {snapshot_dir}"]

    errors: list[str] = []
    volume_frames = 0
    evidence_frames = 0
    total_volumes = 0
    total_evidence = 0
    source_kinds: Counter[str] = Counter()
    volume_providers: Counter[str] = Counter()
    evidence_states: Counter[str] = Counter()
    occupancy_source_kinds: Counter[str] = Counter()
    in_sense = 0
    in_sweep = 0
    examples: list[tuple[str, dict[str, Any]]] = []

    def add_error(message: str) -> None:
        if len(errors) < max_errors:
            errors.append(message)

    for path in files:
        try:
            snap = load_json(path)
        except RuntimeError as exc:
            add_error(str(exc))
            continue

        occupancy = snap.get("ego_occupancy")
        if isinstance(occupancy, dict):
            occupancy_source_kinds[str(occupancy.get("source_kind", "unknown"))] += 1

        volumes = snap.get("obstacle_sensing_volumes", [])
        evidence = snap.get("obstacle_evidence", [])
        if not isinstance(volumes, list):
            add_error(f"{path}: obstacle_sensing_volumes is not a list")
            volumes = []
        if not isinstance(evidence, list):
            add_error(f"{path}: obstacle_evidence is not a list")
            evidence = []

        if volumes:
            volume_frames += 1
        if evidence:
            evidence_frames += 1
        total_volumes += len(volumes)
        total_evidence += len(evidence)

        volume_sensor_names: set[str] = set()
        for volume in volumes:
            if not isinstance(volume, dict):
                add_error(f"{path}: sensing volume item is not an object")
                continue
            volume_providers[str(volume.get("provider_name", ""))] += 1
            for key in REQUIRED_VOLUME_KEYS:
                if key not in volume:
                    add_error(f"{path}: sensing volume missing {key}")
            sensor_name = str(volume.get("sensor_name", "")).strip()
            provider_name = str(volume.get("provider_name", "")).strip()
            if sensor_name:
                volume_sensor_names.add(sensor_name)
            if provider_name in INVALID_VOLUME_PROVIDERS:
                add_error(
                    f"{path}: sensing volume provider_name={provider_name!r} is an implicit/fake visual-emulation provider"
                )
            if volume.get("has_source_frame") is not True:
                add_error(f"{path}: sensing volume must have has_source_frame=true")
            if not str(volume.get("source_frame_id", "")).strip():
                add_error(f"{path}: sensing volume must preserve source_frame_id")
            if vector3(volume.get("origin_local")) is None:
                add_error(f"{path}: sensing volume origin_local is not a vec3")
            if vector3(volume.get("forward_axis_local")) is None:
                add_error(f"{path}: sensing volume forward_axis_local is not a vec3")
            if vector3(volume.get("right_axis_local")) is None:
                add_error(f"{path}: sensing volume right_axis_local is not a vec3")
            if vector3(volume.get("up_axis_local")) is None:
                add_error(f"{path}: sensing volume up_axis_local is not a vec3")
            near = finite_float(volume.get("near_range_m"))
            far = finite_float(volume.get("far_range_m"))
            if near is None or far is None or near < 0.0 or far <= near:
                add_error(f"{path}: sensing volume range must satisfy 0 <= near_range_m < far_range_m")
            horizontal_fov = positive_float(volume.get("horizontal_fov_rad"))
            vertical_fov = positive_float(volume.get("vertical_fov_rad"))
            if horizontal_fov is None or vertical_fov is None:
                add_error(f"{path}: sensing volume FOV values must be positive")
            min_reliable = finite_float(volume.get("min_reliable_range_m"))
            max_reliable = finite_float(volume.get("max_reliable_range_m"))
            if min_reliable is None or max_reliable is None or min_reliable < 0.0 or max_reliable < min_reliable:
                add_error(
                    f"{path}: sensing volume reliable range must satisfy 0 <= min_reliable_range_m <= max_reliable_range_m"
                )

        for item in evidence:
            if not isinstance(item, dict):
                add_error(f"{path}: obstacle evidence item is not an object")
                continue
            source_kind = str(item.get("source_kind", ""))
            source_kinds[source_kind] += 1
            evidence_states[str(item.get("state", ""))] += 1
            in_sense += 1 if item.get("inside_sensing_volume") is True else 0
            in_sweep += 1 if item.get("inside_swept_volume") is True else 0
            for key in REQUIRED_EVIDENCE_KEYS:
                if key not in item:
                    add_error(f"{path}: obstacle evidence missing {key}")
            if vector3(item.get("center_local")) is None:
                add_error(f"{path}: obstacle evidence center_local is not a vec3")
            if source_kind == GT_VISUAL_SOURCE_KIND:
                if item.get("source_provider") != GT_VISUAL_SOURCE_KIND:
                    add_error(
                        f"{path}: AirSim visual-emulation evidence must use source_provider={GT_VISUAL_SOURCE_KIND}"
                    )
                if item.get("inside_sensing_volume") is not True:
                    add_error(
                        f"{path}: AirSim visual-emulation evidence must be inside the explicit sensing volume"
                    )
                evidence_sensor = str(item.get("sensor_name", "")).strip()
                if volume_sensor_names and evidence_sensor not in volume_sensor_names:
                    add_error(
                        f"{path}: evidence sensor_name={evidence_sensor!r} did not match any sensing volume sensor"
                    )
            if len(examples) < 3:
                examples.append((path.name, item))

    if volume_frames == 0:
        add_error("no frames had obstacle_sensing_volumes")
    if evidence_frames == 0:
        add_error("no frames had obstacle_evidence")
    if total_evidence < min_evidence:
        add_error(f"only {total_evidence} evidence records found, expected at least {min_evidence}")
    if require_gt_visual and total_evidence > 0 and GT_VISUAL_SOURCE_KIND not in source_kinds:
        add_error(f"expected at least one obstacle_evidence.source_kind == {GT_VISUAL_SOURCE_KIND}")
    if require_inside_swept and in_sweep <= 0:
        add_error("expected at least one obstacle_evidence item with inside_swept_volume=true")

    print(f"OBSTACLE_SENSING_EVIDENCE snapshot dir: {snapshot_dir}")
    print(f"  snapshots:        {len(files)}")
    print(f"  volume frames:    {volume_frames}")
    print(f"  evidence frames:  {evidence_frames}")
    print(f"  total volumes:    {total_volumes}")
    print(f"  total evidence:   {total_evidence}")
    print(f"  occupancy kinds:  {dict(occupancy_source_kinds)}")
    print(f"  source_kinds:     {dict(source_kinds)}")
    print(f"  volume providers: {dict(volume_providers)}")
    print(f"  evidence states:  {dict(evidence_states)}")
    print(f"  inside sensing:   {in_sense}")
    print(f"  inside swept:     {in_sweep}")
    if examples:
        print("  examples:")
        for name, item in examples:
            print(
                "    {name}: state={state} kind={kind} provider={provider} pos={pos} in_sweep={in_sweep}".format(
                    name=name,
                    state=item.get("state"),
                    kind=item.get("source_kind"),
                    provider=item.get("source_provider"),
                    pos=item.get("center_local"),
                    in_sweep=item.get("inside_swept_volume"),
                )
            )
    return not errors, errors


def candidate_dirs(paths: list[str]) -> list[pathlib.Path]:
    roots = [pathlib.Path(path) for path in paths] if paths else common_roots()
    candidates: list[pathlib.Path] = []
    for root in roots:
        if not root.exists():
            continue
        if root.is_dir() and (root / "snapshot_manifest.txt").exists():
            candidates.append(root)
        elif root.is_dir() and list(root.glob("snapshot_*.json")):
            candidates.append(root)
        elif root.is_dir():
            candidates.extend(find_snapshot_dirs(root))

    unique = sorted(set(candidates), key=lambda path: path.stat().st_mtime, reverse=True)
    return unique


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "paths",
        nargs="*",
        help="Snapshot dir(s) or artifact root(s). Omit to auto-search out/, simulation/, and repo root.",
    )
    parser.add_argument("--min-evidence", type=int, default=1)
    parser.add_argument("--no-require-gt-visual", action="store_true")
    parser.add_argument(
        "--require-inside-swept",
        action="store_true",
        help="Require at least one evidence item to intersect the current swept volume; useful for avoidance scenarios, not normal circle smoke.",
    )
    parser.add_argument("--max-candidates", type=int, default=5)
    parser.add_argument("--max-errors", type=int, default=20)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    candidates = candidate_dirs(args.paths)
    if not candidates:
        print("No snapshot_manifest.txt or snapshot_*.json found.", file=sys.stderr)
        print("Check mission stdout for: Wrote N snapshot(s) to <dir>", file=sys.stderr)
        return 2

    failures: list[tuple[pathlib.Path, list[str]]] = []
    for candidate in candidates[: max(1, args.max_candidates)]:
        ok, errors = validate_dir(
            candidate,
            args.min_evidence,
            not args.no_require_gt_visual,
            args.require_inside_swept,
            max(1, args.max_errors),
        )
        if ok:
            print("PASS")
            return 0
        failures.append((candidate, errors))
        print("FAIL candidate:", candidate)
        for error in errors:
            print("  -", error)
        print()

    print("No candidate passed.", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
