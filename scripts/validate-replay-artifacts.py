#!/usr/bin/env python3
"""Validate replay/capture artifacts produced by Dedalus core-stack runs.

This script is intentionally dependency-free. It validates the artifact contract
around snapshot manifests, optional PPM annotation manifests, and optional
pipeline timing JSONL output. It is meant for recorded-frame CI paths and live
AirSim validation output directories.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


@dataclass(frozen=True)
class SnapshotRow:
    index: int
    path: str
    timestamp_ns: int
    active_map_frame_id: str


@dataclass(frozen=True)
class AnnotationRow:
    index: int
    frame_id: str
    timestamp_ns: int
    path: str
    output_fps: str


@dataclass(frozen=True)
class ProfileRow:
    index: int
    frame_id: str
    timestamp_ns: int
    total_us: int
    stages: dict[str, int]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate Dedalus replay snapshot/annotation/timing artifacts."
    )
    parser.add_argument("--snapshot-dir", required=True, type=Path)
    parser.add_argument("--annotation-dir", type=Path)
    parser.add_argument("--profile-jsonl", type=Path)
    parser.add_argument("--expect-frames", type=int)
    parser.add_argument("--expect-map-frame")
    parser.add_argument("--timestamp-soft-threshold-ms", type=float)
    parser.add_argument(
        "--require-agent",
        action="store_true",
        help="Require each snapshot JSON to contain at least one agent.",
    )
    parser.add_argument(
        "--require-world-keys",
        action="store_true",
        help="Require stable world snapshot arrays to be present in each snapshot.",
    )
    return parser.parse_args()


def fail(message: str) -> None:
    raise AssertionError(message)


def read_snapshot_manifest(snapshot_dir: Path) -> list[SnapshotRow]:
    manifest_path = snapshot_dir / "snapshot_manifest.txt"
    if not manifest_path.exists():
        fail(f"missing snapshot manifest: {manifest_path}")

    rows: list[SnapshotRow] = []
    for line_number, line in enumerate(manifest_path.read_text(encoding="utf-8").splitlines(), start=1):
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        parts = stripped.split()
        if len(parts) < 4:
            fail(f"bad snapshot manifest line {line_number}: {line}")
        rows.append(
            SnapshotRow(
                index=int(parts[0]),
                path=parts[1],
                timestamp_ns=int(parts[2]),
                active_map_frame_id=parts[3],
            )
        )
    if not rows:
        fail(f"snapshot manifest has no rows: {manifest_path}")
    return rows


def read_annotation_manifest(annotation_dir: Path) -> list[AnnotationRow]:
    manifest_path = annotation_dir / "manifest.txt"
    if not manifest_path.exists():
        fail(f"missing annotation manifest: {manifest_path}")

    rows: list[AnnotationRow] = []
    with manifest_path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        required = {"frame_index", "frame_id", "timestamp_ns", "path", "output_fps"}
        if set(reader.fieldnames or []) < required:
            fail(f"annotation manifest missing required columns: {manifest_path}")
        for row in reader:
            rows.append(
                AnnotationRow(
                    index=int(row["frame_index"]),
                    frame_id=row["frame_id"],
                    timestamp_ns=int(row["timestamp_ns"]),
                    path=row["path"],
                    output_fps=row["output_fps"],
                )
            )
    if not rows:
        fail(f"annotation manifest has no rows: {manifest_path}")
    return rows


def read_profile_jsonl(path: Path) -> list[ProfileRow]:
    if not path.exists():
        fail(f"missing pipeline timing JSONL: {path}")

    rows: list[ProfileRow] = []
    for index, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        stripped = line.strip()
        if not stripped:
            continue
        data = json.loads(stripped)
        stages = data.get("stages")
        if not isinstance(stages, dict) or not stages:
            fail(f"profile row {index} missing non-empty stages")
        for name, value in stages.items():
            if not isinstance(name, str) or not isinstance(value, int) or value < 0:
                fail(f"profile row {index} has invalid stage timing: {name}={value}")
        rows.append(
            ProfileRow(
                index=index,
                frame_id=str(data.get("frame_id", "")),
                timestamp_ns=int(data.get("timestamp_ns")),
                total_us=int(data.get("total_us")),
                stages=stages,
            )
        )
    if not rows:
        fail(f"pipeline timing JSONL has no rows: {path}")
    return rows


def ensure_contiguous_indices(indices: Iterable[int], label: str) -> None:
    values = list(indices)
    expected = list(range(1, len(values) + 1))
    if values != expected:
        fail(f"{label} indices are not contiguous: got {values}, expected {expected}")


def validate_snapshot_files(
    snapshot_dir: Path,
    rows: list[SnapshotRow],
    expect_map_frame: str | None,
    require_agent: bool,
    require_world_keys: bool,
) -> None:
    stable_array_keys = [
        "agents",
        "tactical_exclusion_zones",
        "flight_corridors",
        "static_structures",
        "landmarks",
        "uncertain_regions",
        "containers",
        "containment_events",
        "map_frames",
    ]

    for row in rows:
        snapshot_path = snapshot_dir / row.path
        if not snapshot_path.exists():
            fail(f"snapshot manifest references missing file: {snapshot_path}")
        data = json.loads(snapshot_path.read_text(encoding="utf-8"))
        if data.get("timestamp_ns") != row.timestamp_ns:
            fail(
                f"snapshot timestamp mismatch for {snapshot_path}: "
                f"json={data.get('timestamp_ns')} manifest={row.timestamp_ns}"
            )
        if data.get("active_map_frame_id") != row.active_map_frame_id:
            fail(
                f"snapshot map frame mismatch for {snapshot_path}: "
                f"json={data.get('active_map_frame_id')} manifest={row.active_map_frame_id}"
            )
        if expect_map_frame and row.active_map_frame_id != expect_map_frame:
            fail(
                f"unexpected active map frame for row {row.index}: "
                f"{row.active_map_frame_id} != {expect_map_frame}"
            )
        if require_world_keys:
            for key in stable_array_keys:
                if key not in data or not isinstance(data[key], list):
                    fail(f"snapshot {snapshot_path} missing stable array key: {key}")
            if "ego" not in data or not isinstance(data["ego"], dict):
                fail(f"snapshot {snapshot_path} missing ego object")
        if require_agent and not data.get("agents"):
            fail(f"snapshot {snapshot_path} has no agents")


def resolve_annotation_frame_path(annotation_dir: Path, manifest_path_value: str) -> Path:
    raw_path = Path(manifest_path_value)
    if raw_path.is_absolute():
        return raw_path

    candidates = [
        Path.cwd() / raw_path,
        annotation_dir / raw_path,
        annotation_dir / raw_path.name,
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


def validate_annotation_files(annotation_dir: Path, rows: list[AnnotationRow]) -> None:
    for row in rows:
        frame_path = resolve_annotation_frame_path(annotation_dir, row.path)
        if not frame_path.exists():
            fail(f"annotation manifest references missing PPM frame: {row.path}")
        with frame_path.open("rb") as handle:
            magic = handle.read(2)
        if magic != b"P6":
            fail(f"annotation frame is not P6 PPM: {frame_path}")


def validate_timestamp_delta(
    label: str,
    index: int,
    left_timestamp_ns: int,
    right_timestamp_ns: int,
    threshold_ms: float | None,
) -> float:
    delta_ms = abs(left_timestamp_ns - right_timestamp_ns) / 1_000_000.0
    if threshold_ms is None and delta_ms != 0.0:
        fail(f"{label} timestamp mismatch at row {index}: delta_ms={delta_ms:.3f}")
    if threshold_ms is not None and delta_ms > threshold_ms:
        fail(
            f"{label} timestamp delta too large at row {index}: "
            f"{delta_ms:.3f} ms > {threshold_ms:.3f} ms"
        )
    return delta_ms


def validate_alignment(
    snapshot_rows: list[SnapshotRow],
    annotation_rows: list[AnnotationRow] | None,
    profile_rows: list[ProfileRow] | None,
    timestamp_soft_threshold_ms: float | None,
) -> None:
    ensure_contiguous_indices((row.index for row in snapshot_rows), "snapshot")

    if annotation_rows is not None:
        ensure_contiguous_indices((row.index for row in annotation_rows), "annotation")
        if len(annotation_rows) != len(snapshot_rows):
            fail(
                f"annotation/snapshot row count mismatch: "
                f"annotations={len(annotation_rows)} snapshots={len(snapshot_rows)}"
            )
        max_delta_ms = 0.0
        for snapshot, annotation in zip(snapshot_rows, annotation_rows):
            if snapshot.index != annotation.index:
                fail(f"annotation/snapshot index mismatch: {annotation.index} != {snapshot.index}")
            max_delta_ms = max(
                max_delta_ms,
                validate_timestamp_delta(
                    "annotation/snapshot",
                    snapshot.index,
                    annotation.timestamp_ns,
                    snapshot.timestamp_ns,
                    timestamp_soft_threshold_ms,
                ),
            )
        print(f"annotation/snapshot max_abs_delta_ms: {max_delta_ms:.3f}")

    if profile_rows is not None:
        if len(profile_rows) != len(snapshot_rows):
            fail(
                f"profile/snapshot row count mismatch: "
                f"profile={len(profile_rows)} snapshots={len(snapshot_rows)}"
            )
        max_delta_ms = 0.0
        for snapshot, profile in zip(snapshot_rows, profile_rows):
            if snapshot.index != profile.index:
                fail(f"profile/snapshot index mismatch: {profile.index} != {snapshot.index}")
            max_delta_ms = max(
                max_delta_ms,
                validate_timestamp_delta(
                    "profile/snapshot",
                    snapshot.index,
                    profile.timestamp_ns,
                    snapshot.timestamp_ns,
                    timestamp_soft_threshold_ms,
                ),
            )
        print(f"profile/snapshot max_abs_delta_ms: {max_delta_ms:.3f}")


def main() -> int:
    args = parse_args()
    if args.expect_frames is not None and args.expect_frames <= 0:
        fail("--expect-frames must be positive")
    if args.timestamp_soft_threshold_ms is not None and args.timestamp_soft_threshold_ms < 0:
        fail("--timestamp-soft-threshold-ms must be non-negative")

    snapshot_dir = args.snapshot_dir.resolve()
    snapshot_rows = read_snapshot_manifest(snapshot_dir)

    annotation_rows = None
    if args.annotation_dir is not None:
        annotation_rows = read_annotation_manifest(args.annotation_dir.resolve())
        validate_annotation_files(args.annotation_dir.resolve(), annotation_rows)

    profile_rows = None
    if args.profile_jsonl is not None:
        profile_rows = read_profile_jsonl(args.profile_jsonl.resolve())

    if args.expect_frames is not None and len(snapshot_rows) != args.expect_frames:
        fail(f"expected {args.expect_frames} snapshots, got {len(snapshot_rows)}")

    validate_snapshot_files(
        snapshot_dir=snapshot_dir,
        rows=snapshot_rows,
        expect_map_frame=args.expect_map_frame,
        require_agent=args.require_agent,
        require_world_keys=args.require_world_keys,
    )
    validate_alignment(
        snapshot_rows=snapshot_rows,
        annotation_rows=annotation_rows,
        profile_rows=profile_rows,
        timestamp_soft_threshold_ms=args.timestamp_soft_threshold_ms,
    )

    print("OK: replay artifacts validated")
    print(f"snapshots: {len(snapshot_rows)}")
    if annotation_rows is not None:
        print(f"annotations: {len(annotation_rows)}")
    if profile_rows is not None:
        print(f"profile rows: {len(profile_rows)}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except AssertionError as exc:
        print(f"validate-replay-artifacts: {exc}", file=sys.stderr)
        raise SystemExit(1)
