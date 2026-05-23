#!/usr/bin/env python3
"""Integration tests for simulation/validate-circle-trajectory.py."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


def write_jsonl(path: Path, events: list[dict[str, object]]) -> None:
    path.write_text("\n".join(json.dumps(event, sort_keys=True) for event in events) + "\n")


def circle_debug(
    *,
    execute_tick: int,
    phase: str,
    latched: bool,
    actual_radius: float,
    radius_error: float,
    completed_orbits: float,
) -> dict[str, object]:
    return {
        "event": "behavior_debug",
        "debug_level": 2,
        "execute_tick": execute_tick,
        "source_track_id": "ghost_person_001",
        "circle_phase": phase,
        "orbit_mode_latched": latched,
        "orbit_radius_m": 10.0,
        "actual_radius_m": actual_radius,
        "radius_error_m": radius_error,
        "radial_correction_mps": -0.6 * radius_error,
        "tangent_velocity_mps": 1.396263,
        "target_velocity_mps": 0.0,
        "desired_velocity_mps": 1.5,
        "circle_completed_orbits": completed_orbits,
        "orbit_angle_rad": completed_orbits * 6.283185307179586,
        "vx": 1.0,
        "vy": 1.0,
        "vz": 0.0,
    }


def valid_events() -> list[dict[str, object]]:
    events: list[dict[str, object]] = [
        {
            "event": "target_selected",
            "tick": 7,
            "state": "ExecuteMission",
            "source_track_id": "ghost_person_001",
            "agent_id": "agent_ghost_person_001",
        },
        {
            "event": "behavior_tick_sample",
            "tick": 7,
            "circle_phase": "arriving",
            "orbit_mode_latched": False,
            "orbit_radius_m": 10.0,
            "actual_radius_m": 13.0,
            "radius_error_m": 3.0,
            "radial_correction_mps": -1.8,
            "tangent_velocity_mps": 1.396263,
            "target_velocity_mps": 0.0,
            "desired_velocity_mps": 2.2,
            "circle_completed_orbits": 0.0,
            "orbit_angle_rad": 0.0,
        },
    ]
    for index in range(7):
        completed = index * 0.5
        radius_error = 0.3 if index % 2 == 0 else -0.2
        events.append(circle_debug(
            execute_tick=20 + index,
            phase="circling",
            latched=True,
            actual_radius=10.0 + radius_error,
            radius_error=radius_error,
            completed_orbits=completed,
        ))
    events.extend([
        {
            "event": "behavior_complete",
            "tick": 1431,
            "state": "GoHome",
            "behavior": "circle",
            "reason": "orbit_count_elapsed",
        },
        {"event": "state_transition", "from": "ExecuteMission", "to": "GoHome"},
        {"event": "state_transition", "from": "GoHome", "to": "Land"},
        {"event": "state_transition", "from": "Land", "to": "Complete"},
        {"event": "runtime_stop", "terminal_settled": True, "state": "Complete"},
    ])
    return events


def invalid_events() -> list[dict[str, object]]:
    events = valid_events()
    for event in events:
        if "circle_completed_orbits" in event:
            event["circle_completed_orbits"] = min(float(event["circle_completed_orbits"]), 0.9)
        if event.get("event") == "behavior_complete":
            event["reason"] = "duration_elapsed"
    return events


def run_validator(repo_root: Path, events: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [
            sys.executable,
            str(repo_root / "simulation" / "validate-circle-trajectory.py"),
            "--events",
            str(events),
            "--min-orbits",
            "3.0",
            "--radius",
            "10.0",
            "--avg-radius-error-max",
            "1.0",
            "--max-radius-error-after-latch",
            "3.0",
            "--expect-complete-reason",
            "orbit_count_elapsed",
            "--require-terminal-settled",
            "--require-lifecycle",
        ],
        cwd=repo_root,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: test_circle_trajectory_validator.py <repo-root> <build-dir>", file=sys.stderr)
        return 2

    repo_root = Path(sys.argv[1]).resolve()
    build_dir = Path(sys.argv[2]).resolve()
    tmp_dir = build_dir / "circle_trajectory_validator_test"
    tmp_dir.mkdir(parents=True, exist_ok=True)

    valid_path = tmp_dir / "valid_mission_events.jsonl"
    invalid_path = tmp_dir / "invalid_mission_events.jsonl"
    write_jsonl(valid_path, valid_events())
    write_jsonl(invalid_path, invalid_events())

    valid = run_validator(repo_root, valid_path)
    if valid.returncode != 0:
        print(valid.stdout)
        print(valid.stderr, file=sys.stderr)
        raise AssertionError("valid circle trajectory fixture should pass")
    if "PASS completed_orbits" not in valid.stdout:
        raise AssertionError("valid output should include completed_orbits pass")

    invalid = run_validator(repo_root, invalid_path)
    if invalid.returncode == 0:
        print(invalid.stdout)
        raise AssertionError("invalid under-orbit fixture should fail")
    if "FAIL completed_orbits" not in invalid.stdout:
        print(invalid.stdout)
        raise AssertionError("invalid output should include completed_orbits failure")
    if "FAIL behavior_complete_reason" not in invalid.stdout:
        print(invalid.stdout)
        raise AssertionError("invalid output should include completion-reason failure")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
