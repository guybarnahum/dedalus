#!/usr/bin/env python3
"""Validate AirSim ghost object-behavior artifacts after an operator-run mission.

This script intentionally does not start AirSim or PX4. It validates the artifacts produced by
an AirSim ghost/object-behavior run so live simulator execution remains operator-controlled.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        raise AssertionError(f"missing JSONL artifact: {path}")
    rows: list[dict[str, Any]] = []
    for line_number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        if not raw.strip():
            continue
        try:
            value = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise AssertionError(f"invalid JSON on {path}:{line_number}: {exc}") from exc
        if not isinstance(value, dict):
            raise AssertionError(f"expected object on {path}:{line_number}")
        rows.append(value)
    if not rows:
        raise AssertionError(f"empty JSONL artifact: {path}")
    return rows


def find_event(events: list[dict[str, Any]], name: str) -> dict[str, Any]:
    for event in events:
        if event.get("event") == name:
            return event
    raise AssertionError(f"missing mission event: {name}")


def require_command(events: list[dict[str, Any]], command: str) -> None:
    if not any(event.get("event") == "command_dispatch" and event.get("command") == command for event in events):
        raise AssertionError(f"missing command_dispatch for {command}")
    if not any(
        event.get("event") == "command_result" and event.get("command") == command and event.get("success") is True
        for event in events
    ):
        raise AssertionError(f"missing successful command_result for {command}")


def validate_mission_events(output_dir: Path) -> None:
    events = read_jsonl(output_dir / "mission_events.jsonl")
    target_selected = find_event(events, "target_selected")
    behavior_start = find_event(events, "behavior_start")
    behavior_complete = find_event(events, "behavior_complete")
    runtime_stop = find_event(events, "runtime_stop")

    if target_selected.get("source_track_id") != "ghost_person_001":
        raise AssertionError(f"expected target_selected source_track_id ghost_person_001, got {target_selected}")
    if target_selected.get("agent_id") != "agent_ghost_person_001":
        raise AssertionError(f"expected target_selected agent_id agent_ghost_person_001, got {target_selected}")
    if target_selected.get("class") != "person":
        raise AssertionError(f"expected selected class person, got {target_selected}")
    if any(event.get("event") == "target_selected" and event.get("source_track_id") == "ghost_person_002" for event in events):
        raise AssertionError("selected higher-confidence ghost_person_002 instead of requested ghost_person_001")

    for event_name, event in [("behavior_start", behavior_start), ("behavior_complete", behavior_complete)]:
        if event.get("source_track_id") != "ghost_person_001":
            raise AssertionError(f"{event_name} should carry selected source_track_id: {event}")

    for command in ["Arm", "Takeoff", "Land", "Disarm"]:
        require_command(events, command)

    if runtime_stop.get("state") != "Complete" or runtime_stop.get("terminal_settled") is not True:
        raise AssertionError(f"expected terminal Complete runtime_stop, got {runtime_stop}")


def validate_snapshots(output_dir: Path) -> None:
    manifest = output_dir / "snapshot_manifest.txt"
    if not manifest.exists():
        raise AssertionError(f"missing snapshot manifest: {manifest}")
    snapshot_names = []
    for raw in manifest.read_text(encoding="utf-8").splitlines():
        if not raw.strip() or raw.startswith("#"):
            continue
        fields = raw.split()
        if len(fields) >= 2:
            snapshot_names.append(fields[1])
    if not snapshot_names:
        raise AssertionError("snapshot manifest has no snapshot rows")

    found = False
    for name in snapshot_names:
        path = output_dir / name
        if not path.exists():
            raise AssertionError(f"manifest references missing snapshot: {path}")
        snapshot = json.loads(path.read_text(encoding="utf-8"))
        agents = snapshot.get("agents", [])
        track_ids = {agent.get("source_track_id") for agent in agents}
        if {"ghost_person_001", "ghost_person_002", "ghost_car_001"} <= track_ids:
            found = True
            break
    if not found:
        raise AssertionError("no snapshot contained the expected ghost agents")


def validate_annotations(annotation_dir: Path) -> None:
    if not annotation_dir.exists():
        raise AssertionError(f"missing annotation directory: {annotation_dir}")
    frames = sorted(annotation_dir.glob("frame_*.ppm"))
    sidecars = sorted(annotation_dir.glob("frame_*.world_overlay.json"))
    if not frames:
        raise AssertionError(f"no annotated PPM frames found in {annotation_dir}")
    if not sidecars:
        raise AssertionError(f"no world overlay sidecars found in {annotation_dir}")

    saw_ghost = False
    for sidecar_path in sidecars[: min(len(sidecars), 20)]:
        sidecar = json.loads(sidecar_path.read_text(encoding="utf-8"))
        tracks = {agent.get("source_track_id") for agent in sidecar.get("agents", [])}
        if "ghost_person_001" in tracks:
            saw_ghost = True
            break
    if not saw_ghost:
        raise AssertionError("world overlay sidecars did not include ghost_person_001")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_dir", type=Path, help="Mission output directory containing mission_events.jsonl and snapshots")
    parser.add_argument(
        "--annotation-dir",
        type=Path,
        default=Path("out/object_behavior_airsim_ghost_annotation"),
        help="Annotation directory containing frame_XXXXXX.ppm and .world_overlay.json files",
    )
    args = parser.parse_args()

    validate_mission_events(args.output_dir)
    validate_snapshots(args.output_dir)
    validate_annotations(args.annotation_dir)
    print("AirSim ghost object-behavior artifacts validated")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
