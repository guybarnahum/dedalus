#!/usr/bin/env python3
"""Smoke tests for tools/mission/validate-obstacle-sensing-evidence-snapshots.py."""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
from pathlib import Path


def write_snapshot(path: Path, *, with_evidence: bool = True, inside_swept: bool = True) -> None:
    evidence = []
    if with_evidence:
        evidence.append(
            {
                "timestamp_ns": 123,
                "sensor_name": "front_center",
                "source_provider": "airsim_gt_vd",
                "source_kind": "airsim_gt_vd",
                "map_frame_id": "map_local_0001",
                "state": "occupied",
                "shape": "voxel",
                "center_local": [5.0, 0.0, -8.0],
                "size_m": [1.0, 1.0, 1.0],
                "endpoint_a_local": [0.0, 0.0, 0.0],
                "endpoint_b_local": [0.0, 0.0, 0.0],
                "radius_m": 0.5,
                "occupancy_probability": 0.9,
                "free_probability": 0.0,
                "confidence": 0.9,
                "range_m": 5.0,
                "bearing_rad": 0.0,
                "elevation_rad": 0.0,
                "inside_sensing_volume": True,
                "inside_swept_volume": inside_swept,
                "is_static_hint": True,
                "is_thin_structure_hint": False,
            }
        )
    snapshot = {
        "timestamp_ns": 123,
        "active_map_frame_id": "map_local_0001",
        "ego_occupancy": {
            "timestamp_ns": 123,
            "map_frame_id": "map_local_0001",
            "source_kind": "airsim_ground_truth",
            "source_provider": "airsim_ground_truth_named_objects",
            "resolution_m": 1.0,
            "size_m": [60.0, 60.0, 20.0],
            "occupied_count": 1,
            "free_count": 0,
            "unknown_count": 0,
            "stale_count": 0,
            "nearest_obstacle_distance_m": 5.0,
            "forward_corridor_clearance_m": 0.0,
            "has_valid_occupancy": True,
            "debug_cells": [],
        },
        "obstacle_sensing_volumes": [
            {
                "timestamp_ns": 123,
                "sensor_name": "front_center",
                "provider_name": "configured_camera_coverage",
                "map_frame_id": "map_local_0001",
                "source_frame_id": "frame_0001",
                "has_source_frame": True,
                "origin_local": [0.0, 0.0, -8.0],
                "forward_axis_local": [1.0, 0.0, 0.0],
                "right_axis_local": [0.0, 1.0, 0.0],
                "up_axis_local": [0.0, 0.0, -1.0],
                "near_range_m": 0.5,
                "far_range_m": 80.0,
                "horizontal_fov_rad": 1.5708,
                "vertical_fov_rad": 1.0472,
                "min_reliable_range_m": 1.0,
                "max_reliable_range_m": 60.0,
                "min_surface_area_m2": 0.25,
                "min_angular_size_rad": 0.01,
                "min_confidence": 0.3,
            }
        ],
        "obstacle_evidence": evidence,
    }
    path.write_text(json.dumps(snapshot, separators=(",", ":")) + "\n", encoding="utf-8")


def seed_snapshot_dir(run_dir: Path, *, with_evidence: bool = True, inside_swept: bool = True) -> None:
    run_dir.mkdir()
    (run_dir / "snapshot_manifest.txt").write_text(
        "# index path timestamp_ns active_map_frame_id\n1 snapshot_0001.json 123 map_local_0001\n",
        encoding="utf-8",
    )
    write_snapshot(run_dir / "snapshot_0001.json", with_evidence=with_evidence, inside_swept=inside_swept)


def run_validator(repo_root: Path, run_dir: Path, *args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [
            sys.executable,
            str(repo_root / "tools" / "mission" / "validate-obstacle-sensing-evidence-snapshots.py"),
            str(run_dir),
            *args,
        ],
        cwd=repo_root,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )


def main() -> int:
    repo_root = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[2]
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)

        valid_dir = root / "valid_obstacle_sensing_evidence"
        seed_snapshot_dir(valid_dir)
        valid = run_validator(repo_root, valid_dir, "--require-inside-swept")
        if valid.returncode != 0:
            print(valid.stdout)
            print(valid.stderr, file=sys.stderr)
            return 1
        for token in (
            "OBSTACLE_SENSING_EVIDENCE snapshot dir:",
            "total volumes:    1",
            "total evidence:   1",
            "source_kinds:     {'airsim_gt_vd': 1}",
            "volume providers: {'configured_camera_coverage': 1}",
            "inside swept:     1",
            "PASS",
        ):
            if token not in valid.stdout:
                print(valid.stdout)
                print(f"missing expected validator token: {token}", file=sys.stderr)
                return 1

        missing_dir = root / "missing_obstacle_evidence"
        seed_snapshot_dir(missing_dir, with_evidence=False)
        missing = run_validator(repo_root, missing_dir)
        if missing.returncode == 0:
            print(missing.stdout)
            print("validator accepted missing obstacle_evidence", file=sys.stderr)
            return 1
        if "no frames had obstacle_evidence" not in missing.stdout:
            print(missing.stdout)
            print("validator did not explain missing obstacle_evidence", file=sys.stderr)
            return 1

        fake_volume_dir = root / "fake_visual_emulation_volume"
        seed_snapshot_dir(fake_volume_dir)
        fake_snapshot_path = fake_volume_dir / "snapshot_0001.json"
        fake_snapshot = json.loads(fake_snapshot_path.read_text(encoding="utf-8"))
        fake_snapshot["obstacle_sensing_volumes"][0]["provider_name"] = "airsim_gt_vd"
        fake_snapshot_path.write_text(json.dumps(fake_snapshot, separators=(",", ":")) + "\n", encoding="utf-8")
        fake_volume = run_validator(repo_root, fake_volume_dir)
        if fake_volume.returncode == 0:
            print(fake_volume.stdout)
            print("validator accepted implicit/fake AirSim visual-emulation sensing volume", file=sys.stderr)
            return 1
        if "implicit/fake visual-emulation provider" not in fake_volume.stdout:
            print(fake_volume.stdout)
            print("validator did not explain fake visual-emulation sensing volume", file=sys.stderr)
            return 1

        nonblocking_dir = root / "nonblocking_obstacle_evidence"
        seed_snapshot_dir(nonblocking_dir, inside_swept=False)
        nonblocking = run_validator(repo_root, nonblocking_dir, "--require-inside-swept")
        if nonblocking.returncode == 0:
            print(nonblocking.stdout)
            print("validator accepted missing inside_swept evidence", file=sys.stderr)
            return 1
        if "inside_swept_volume=true" not in nonblocking.stdout:
            print(nonblocking.stdout)
            print("validator did not explain missing swept-volume evidence", file=sys.stderr)
            return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
