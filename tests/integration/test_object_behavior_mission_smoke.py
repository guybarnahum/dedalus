#!/usr/bin/env python3
"""Synthetic smoke test for object-behavior mission lifecycle integration."""

from __future__ import annotations

import json
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any


def read_events(path: Path) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    for line_number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        if not raw.strip():
            continue
        try:
            event = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise AssertionError(f"invalid JSON event on line {line_number}: {exc}") from exc
        if not isinstance(event, dict):
            raise AssertionError(f"event on line {line_number} is not an object")
        events.append(event)
    return events


def require_event(events: list[dict[str, Any]], event_name: str) -> dict[str, Any]:
    for event in events:
        if event.get("event") == event_name:
            return event
    raise AssertionError(f"missing event {event_name}")


def require_command(events: list[dict[str, Any]], command: str) -> None:
    if not any(event.get("event") == "command_dispatch" and event.get("command") == command for event in events):
        raise AssertionError(f"missing command_dispatch for {command}")
    if not any(
        event.get("event") == "command_result" and event.get("command") == command and event.get("success") is True
        for event in events
    ):
        raise AssertionError(f"missing successful command_result for {command}")


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: test_object_behavior_mission_smoke.py <repo-root> <build-dir>", file=sys.stderr)
        return 2

    repo_root = Path(sys.argv[1]).resolve()
    build_dir = Path(sys.argv[2]).resolve()
    app = build_dir / "apps" / "dedalus_mission_loop"
    if not app.exists():
        raise AssertionError(f"missing app binary: {app}")

    output_dir = build_dir / "object-behavior-mission-smoke"
    if output_dir.exists():
        shutil.rmtree(output_dir)

    command = [
        str(app),
        "--config",
        str(repo_root / "config" / "ci" / "core_stack_object_behavior_mission.yaml"),
        "--output-dir",
        str(output_dir),
        "--max-frames",
        "220",
        "--shutdown-max-frames",
        "50",
        "--no-progress",
    ]
    result = subprocess.run(
        command,
        cwd=repo_root,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    if result.returncode != 0:
        print(result.stdout)
        raise AssertionError(f"dedalus_mission_loop returned {result.returncode}")

    if "Mission runtime: object_behavior" not in result.stdout:
        print(result.stdout)
        raise AssertionError("console output missing object_behavior runtime startup")
    if "Mission events:" not in result.stdout:
        print(result.stdout)
        raise AssertionError("console output missing Mission events artifact line")

    events_path = output_dir / "mission_events.jsonl"
    manifest_path = output_dir / "snapshot_manifest.txt"
    if not events_path.exists():
        raise AssertionError(f"missing mission events artifact: {events_path}")
    if not manifest_path.exists():
        raise AssertionError(f"missing snapshot manifest artifact: {manifest_path}")
    manifest_rows = [line for line in manifest_path.read_text(encoding="utf-8").splitlines() if line.strip()]
    if len(manifest_rows) < 2:
        raise AssertionError("snapshot manifest should contain header and at least one snapshot row")

    events = read_events(events_path)
    target_selected = require_event(events, "target_selected")
    behavior_start = require_event(events, "behavior_start")
    behavior_tick_sample = require_event(events, "behavior_tick_sample")
    behavior_complete = require_event(events, "behavior_complete")
    runtime_stop = require_event(events, "runtime_stop")

    if target_selected.get("source_track_id") != "ghost_person_001":
        raise AssertionError(f"expected selected source_track_id ghost_person_001, got {target_selected}")
    if target_selected.get("agent_id") != "agent_ghost_person_001":
        raise AssertionError(f"expected selected agent_id agent_ghost_person_001, got {target_selected}")
    if target_selected.get("class") != "person":
        raise AssertionError(f"expected selected class person, got {target_selected}")
    if any(event.get("event") == "target_selected" and event.get("source_track_id") == "ghost_person_002" for event in events):
        raise AssertionError("object behavior selected higher-confidence ghost_person_002 instead of requested track")

    if behavior_start.get("source_track_id") != "ghost_person_001":
        raise AssertionError(f"behavior_start should carry selected source_track_id: {behavior_start}")
    if behavior_complete.get("source_track_id") != "ghost_person_001":
        raise AssertionError(f"behavior_complete should carry selected source_track_id: {behavior_complete}")
    if behavior_tick_sample.get("source_track_id") != "ghost_person_001":
        raise AssertionError(f"behavior_tick_sample should carry selected source_track_id: {behavior_tick_sample}")
    if runtime_stop.get("state") != "Complete" or runtime_stop.get("terminal_settled") is not True:
        raise AssertionError(f"runtime_stop should be terminal Complete: {runtime_stop}")

    for command_name in ["Arm", "Takeoff", "Land", "Disarm"]:
        require_command(events, command_name)

    velocity_dispatches = [
        event for event in events
        if event.get("event") == "command_dispatch" and event.get("command") == "Velocity"
    ]
    if not velocity_dispatches:
        raise AssertionError("expected at least one hold Velocity command during object behavior")
    behavior_velocity_seen = False
    nonzero_behavior_velocity_seen = False
    for event in velocity_dispatches:
        if event.get("state") == "ExecuteMission":
            behavior_velocity_seen = True
            if any(abs(float(event.get(axis, 0.0))) > 1.0e-6 for axis in ["vx", "vy", "vz"]):
                nonzero_behavior_velocity_seen = True
            horizontal_speed = (float(event.get("vx", 0.0)) ** 2 + float(event.get("vy", 0.0)) ** 2) ** 0.5
            if horizontal_speed > 2.0001:
                raise AssertionError(f"follow horizontal velocity exceeded max_speed_mps: {event}")
            if abs(float(event.get("vz", 0.0))) > 1.0001:
                raise AssertionError(f"follow vertical velocity exceeded max_vertical_speed_mps: {event}")
    if not behavior_velocity_seen:
        raise AssertionError("expected at least one ExecuteMission hold Velocity command")
    if not nonzero_behavior_velocity_seen:
        raise AssertionError("expected at least one non-zero ExecuteMission follow Velocity command")

    state_path = [event.get("to") for event in events if event.get("event") == "state_transition"]
    for expected in ["Prepare", "Takeoff", "ExecuteMission", "GoHome", "Land", "Complete"]:
        if expected not in state_path:
            raise AssertionError(f"state transitions missing {expected}: {state_path}")

    complete_transitions = [event for event in events if event.get("event") == "state_transition" and event.get("to") == "Complete"]
    if not complete_transitions:
        raise AssertionError("missing Complete state transition")
    landed_height = complete_transitions[-1].get("ego_height_m")
    if not isinstance(landed_height, (int, float)) or float(landed_height) > 1.0:
        raise AssertionError(f"Complete should occur at landed height, got {landed_height}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
