#!/usr/bin/env python3
"""Smoke tests for tools/mission/validate-mission-artifacts.py."""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
from pathlib import Path


def write_jsonl(path: Path, records: list[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8") as output:
        for record in records:
            output.write(json.dumps(record, separators=(",", ":")) + "\n")


def write_occupancy_snapshot(path: Path) -> None:
    path.write_text(
        json.dumps(
            {
                "timestamp_ns": 123,
                "ego_occupancy": {
                    "timestamp_ns": 123,
                    "map_frame_id": "map_local_0001",
                    "source_kind": "synthetic_fixture",
                    "source_provider": "synthetic_track4_fixture",
                    "resolution_m": 1.0,
                    "size_m": [12.0, 8.0, 4.0],
                    "occupied_count": 1,
                    "free_count": 1,
                    "unknown_count": 1,
                    "stale_count": 0,
                    "nearest_obstacle_distance_m": 5.0,
                    "forward_corridor_clearance_m": 4.0,
                    "has_valid_occupancy": True,
                    "debug_cells": [
                        {
                            "center_local": [5.0, 0.0, 0.0],
                            "size_m": [1.0, 1.0, 1.0],
                            "state": "occupied",
                            "confidence": 0.85,
                            "age_s": 0.0,
                            "distance_to_nearest_occupied_m": 0.0,
                            "source_provider": "synthetic_track4_fixture",
                            "source_object_name": "synthetic_forward_obstacle",
                        }
                    ],
                },
            },
            separators=(",", ":"),
        )
        + "\n",
        encoding="utf-8",
    )


def write_occupancy_sidecar(path: Path) -> None:
    path.write_text(
        json.dumps(
            {
                "frame_index": 1,
                "occupancy": {
                    "present": True,
                    "timestamp_ns": 123,
                    "map_frame_id": "map_local_0001",
                    "source_kind": "synthetic_fixture",
                    "source_provider": "synthetic_track4_fixture",
                    "summary": {
                        "resolution_m": 1.0,
                        "size_m": [12.0, 8.0, 4.0],
                        "occupied_count": 1,
                        "free_count": 1,
                        "unknown_count": 1,
                        "stale_count": 0,
                        "nearest_obstacle_distance_m": 5.0,
                        "forward_corridor_clearance_m": 4.0,
                        "has_valid_occupancy": True,
                    },
                    "projected_cells": [
                        {
                            "state": "occupied",
                            "center_local": [5.0, 0.0, 0.0],
                            "size_m": [1.0, 1.0, 1.0],
                            "confidence": 0.85,
                            "age_s": 0.0,
                            "distance_to_nearest_occupied_m": 0.0,
                            "source_provider": "synthetic_track4_fixture",
                            "source_object_name": "synthetic_forward_obstacle",
                            "visible": True,
                            "u_px": 320.0,
                            "v_px": 240.0,
                            "depth_m": 5.0,
                            "range_m": 5.0,
                            "reason": "visible",
                        }
                    ],
                },
            },
            separators=(",", ":"),
        )
        + "\n",
        encoding="utf-8",
    )


def valid_records() -> list[dict[str, object]]:
    return [
        {"event": "runtime_start", "tick_hz": 10.0},
        {"event": "state_transition", "tick": 1, "from": "Idle", "to": "Prepare", "status": "arming", "ego_height_m": 0.0},
        {"event": "command_dispatch", "tick": 1, "state": "Prepare", "command": "Arm"},
        {"event": "command_result", "tick": 1, "state": "Prepare", "command": "Arm", "success": True},
        {"event": "state_transition", "tick": 2, "from": "Prepare", "to": "Takeoff", "status": "armed", "ego_height_m": 0.1},
        {"event": "command_dispatch", "tick": 2, "state": "Takeoff", "command": "Takeoff"},
        {"event": "command_result", "tick": 2, "state": "Takeoff", "command": "Takeoff", "success": True},
        {"event": "state_transition", "tick": 42, "from": "Takeoff", "to": "ExecuteMission", "status": "takeoff_complete", "ego_height_m": 16.2},
        {"event": "command_dispatch", "tick": 43, "state": "ExecuteMission", "command": "Velocity"},
        {"event": "command_result", "tick": 43, "state": "ExecuteMission", "command": "Velocity", "success": True},
        {"event": "state_transition", "tick": 70, "from": "ExecuteMission", "to": "GoHome", "status": "trajectory_complete", "ego_height_m": 16.0},
        {"event": "state_transition", "tick": 90, "from": "GoHome", "to": "Land", "status": "home_reached", "ego_height_m": 15.8},
        {"event": "command_dispatch", "tick": 91, "state": "Land", "command": "Land"},
        {"event": "command_result", "tick": 91, "state": "Land", "command": "Land", "success": True},
        {"event": "state_transition", "tick": 120, "from": "Land", "to": "Complete", "status": "landed", "ego_height_m": 0.1},
        {"event": "command_dispatch", "tick": 121, "state": "Complete", "command": "Disarm"},
        {"event": "command_result", "tick": 121, "state": "Complete", "command": "Disarm", "success": True},
        {"event": "runtime_stop", "tick_count": 122, "state": "Complete"},
    ]


def sequence_records() -> list[dict[str, object]]:
    records = valid_records()
    records[9:9] = [
        {"event": "target_selected", "tick": 43, "agent_id": "agent_ghost_person_001", "source_track_id": "ghost_person_001"},
        {"event": "behavior_start", "tick": 43, "behavior": "sequence", "mission": "sequence_test", "reason": "target_selected"},
        {"event": "behavior_sequence_step_start", "tick": 43, "behavior": "sequence", "step_index": 0, "step_behavior": "approach", "step_yaw_mode": "target", "step_camera_pointing_mode": "target", "mission": "sequence_test", "reason": "sequence_start"},
        {"event": "behavior_sequence_step_complete", "tick": 50, "behavior": "sequence", "step_index": 0, "step_behavior": "approach", "step_yaw_mode": "target", "step_camera_pointing_mode": "target", "mission": "sequence_test", "reason": "approach_standoff_reached"},
        {"event": "behavior_sequence_step_start", "tick": 50, "behavior": "sequence", "step_index": 1, "step_behavior": "circle", "step_yaw_mode": "target", "step_camera_pointing_mode": "target", "mission": "sequence_test", "reason": "previous_step_complete"},
        {"event": "behavior_complete", "tick": 68, "behavior": "sequence", "mission": "sequence_test", "reason": "sequence_complete"},
    ]
    return records


def run_validator(repo_root: Path, run_dir: Path, *args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(repo_root / "tools" / "mission" / "validate-mission-artifacts.py"), str(run_dir), *args],
        cwd=repo_root,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )


def seed_basic_run(run_dir: Path) -> None:
    run_dir.mkdir()
    write_jsonl(run_dir / "mission_events.jsonl", valid_records())
    (run_dir / "snapshot_manifest.txt").write_text("snapshot_0001.json\n", encoding="utf-8")
    (run_dir / "snapshot_0001.json").write_text("{}\n", encoding="utf-8")


def main() -> int:
    repo_root = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[2]

    with tempfile.TemporaryDirectory() as tmp:
        run_dir = Path(tmp) / "valid_run"
        seed_basic_run(run_dir)

        ok = run_validator(repo_root, run_dir, "--expect-complete", "--safe-height-m", "16", "--landed-height-m", "0.5")
        if ok.returncode != 0:
            print(ok.stdout)
            print(ok.stderr, file=sys.stderr)
            return 1
        if "final_state: Complete" not in ok.stdout or "failures: 0" not in ok.stdout:
            print(ok.stdout)
            print("validator did not report expected success summary", file=sys.stderr)
            return 1

        occupancy_dir = Path(tmp) / "occupancy_run"
        occupancy_dir.mkdir()
        write_jsonl(occupancy_dir / "mission_events.jsonl", valid_records())
        (occupancy_dir / "snapshot_manifest.txt").write_text("snapshot_0001.json\n", encoding="utf-8")
        write_occupancy_snapshot(occupancy_dir / "snapshot_0001.json")
        write_occupancy_sidecar(occupancy_dir / "frame_000001.world_overlay.json")
        occupancy = run_validator(repo_root, occupancy_dir, "--expect-complete", "--expect-occupancy", "--expect-occupancy-sidecars", "--safe-height-m", "16", "--landed-height-m", "0.5")
        if occupancy.returncode != 0:
            print(occupancy.stdout)
            print(occupancy.stderr, file=sys.stderr)
            return 1
        for token in ("occupancy_artifacts:", "snapshots_checked: 1", "sidecars_checked: 1", "projected_cells_checked: 1", "source_kind synthetic_fixture: 1", "failures: 0"):
            if token not in occupancy.stdout:
                print(occupancy.stdout)
                print(f"validator did not report expected occupancy token: {token}", file=sys.stderr)
                return 1

        missing_occupancy = run_validator(repo_root, run_dir, "--expect-complete", "--expect-occupancy", "--safe-height-m", "16", "--landed-height-m", "0.5")
        if missing_occupancy.returncode == 0:
            print(missing_occupancy.stdout)
            print("validator accepted missing ego_occupancy", file=sys.stderr)
            return 1
        if "missing ego_occupancy" not in missing_occupancy.stdout:
            print(missing_occupancy.stdout)
            print("validator did not explain missing ego_occupancy", file=sys.stderr)
            return 1

        sequence_dir = Path(tmp) / "sequence_run"
        sequence_dir.mkdir()
        write_jsonl(sequence_dir / "mission_events.jsonl", sequence_records())
        (sequence_dir / "snapshot_manifest.txt").write_text("snapshot_0001.json\n", encoding="utf-8")
        (sequence_dir / "snapshot_0001.json").write_text("{}\n", encoding="utf-8")
        sequence = run_validator(
            repo_root,
            sequence_dir,
            "--expect-complete",
            "--expect-behavior",
            "--expect-sequence",
            "--expect-sequence-steps",
            "approach,circle",
            "--expect-sequence-step-modes",
            "approach:target:target,circle:target:target",
            "--safe-height-m",
            "16",
            "--landed-height-m",
            "0.5",
        )
        if sequence.returncode != 0:
            print(sequence.stdout)
            print(sequence.stderr, file=sys.stderr)
            return 1
        if "sequence_started_steps: approach,circle" not in sequence.stdout or "failures: 0" not in sequence.stdout:
            print(sequence.stdout)
            print("validator did not report expected sequence success summary", file=sys.stderr)
            return 1
        if "approach: yaw=target camera=target" not in sequence.stdout or "circle: yaw=target camera=target" not in sequence.stdout:
            print(sequence.stdout)
            print("validator did not report expected sequence mode summary", file=sys.stderr)
            return 1

        low_height_dir = Path(tmp) / "low_height_run"
        low_height_dir.mkdir()
        records = valid_records()
        for record in records:
            if record.get("event") == "state_transition" and record.get("to") == "ExecuteMission":
                record["ego_height_m"] = 2.0
        write_jsonl(low_height_dir / "mission_events.jsonl", records)
        (low_height_dir / "snapshot_manifest.txt").write_text("snapshot_0001.json\n", encoding="utf-8")
        low_height = run_validator(repo_root, low_height_dir, "--expect-complete", "--safe-height-m", "16")
        if low_height.returncode == 0:
            print(low_height.stdout)
            print("validator accepted ExecuteMission below safe height", file=sys.stderr)
            return 1
        if "ExecuteMission reached below safe height" not in low_height.stdout:
            print(low_height.stdout)
            print("validator did not explain safe-height failure", file=sys.stderr)
            return 1

        behavior_missing = run_validator(repo_root, run_dir, "--expect-complete", "--expect-behavior")
        if behavior_missing.returncode == 0:
            print(behavior_missing.stdout)
            print("validator accepted missing object behavior events", file=sys.stderr)
            return 1
        if "target_selected" not in behavior_missing.stdout or "behavior_start" not in behavior_missing.stdout:
            print(behavior_missing.stdout)
            print("validator did not report missing behavior extension events", file=sys.stderr)
            return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
