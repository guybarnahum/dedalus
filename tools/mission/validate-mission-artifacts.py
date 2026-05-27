#!/usr/bin/env python3
"""Validate Dedalus live mission run artifact directories.

The validator intentionally stays independent from Dedalus C++ bindings. A live run
artifact directory is expected to contain mission_events.jsonl plus snapshot
artifacts. The event log is the authoritative compact timeline; snapshots are
checked as run evidence, not replay inputs.
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

COMPLETE_STATE_ORDER = ["Prepare", "Takeoff", "ExecuteMission", "GoHome", "Land", "Complete"]
ABORT_AFTER_FLIGHT_STATE_ORDER = ["Prepare", "Takeoff", "ExecuteMission", "GoHome", "Land", "Abort"]
OBJECT_BEHAVIOR_EVENTS = {
    "target_selected",
    "target_reacquired",
    "target_lost",
    "behavior_start",
    "behavior_tick_sample",
    "behavior_complete",
    "behavior_failed",
    "fallback_start",
    "fallback_complete",
    "behavior_sequence_step_start",
    "behavior_sequence_step_complete",
}
CAMERA_POINTING_EVENTS = {
    "camera_pointing_intent",
    "camera_pointing_dispatch",
    "camera_pointing_result",
}
SEQUENCE_EVENTS = {
    "behavior_sequence_step_start",
    "behavior_sequence_step_complete",
}


@dataclass
class ValidationResult:
    event_count: int = 0
    snapshot_count: int = 0
    final_state: str = "unknown"
    state_path: list[str] = field(default_factory=list)
    failures: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    behavior_events: dict[str, int] = field(default_factory=dict)
    sequence_events: dict[str, int] = field(default_factory=dict)
    sequence_started_steps: list[str] = field(default_factory=list)
    sequence_completed_steps: list[str] = field(default_factory=list)
    camera_pointing_events: dict[str, int] = field(default_factory=dict)
    camera_pointing_modes: dict[str, int] = field(default_factory=dict)
    camera_proof_frames: dict[str, int] = field(default_factory=dict)
    velocity_commands: int = 0
    safe_height_gate_height_m: float | None = None
    landed_gate_height_m: float | None = None
    abort_height_m: float | None = None

    @property
    def valid(self) -> bool:
        return not self.failures


def append_state_if_new(states: list[str], state: Any) -> None:
    if not isinstance(state, str) or not state:
        return
    if not states or states[-1] != state:
        states.append(state)


def event_tick(event: dict[str, Any]) -> str:
    tick = event.get("tick")
    return f"tick {tick}" if isinstance(tick, int) else "unknown tick"


def read_events(path: Path, result: ValidationResult) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    try:
        input_file = path.open("r", encoding="utf-8")
    except OSError as exc:
        result.failures.append(f"failed to open mission_events.jsonl: {exc}")
        return events

    with input_file:
        for line_number, raw_line in enumerate(input_file, start=1):
            line = raw_line.strip()
            if not line:
                continue
            try:
                event = json.loads(line)
            except json.JSONDecodeError as exc:
                result.failures.append(f"line {line_number}: invalid JSON: {exc}")
                continue
            if not isinstance(event, dict):
                result.failures.append(f"line {line_number}: event record is not an object")
                continue
            result.event_count += 1
            events.append(event)
    return events


def count_snapshots(run_dir: Path) -> int:
    manifest = run_dir / "snapshot_manifest.txt"
    if manifest.exists():
        try:
            return sum(1 for line in manifest.read_text(encoding="utf-8").splitlines() if line.strip())
        except OSError:
            return 0
    return len(list(run_dir.glob("snapshot_*.json")))


def camera_frame_counts(path: Path) -> Counter[str]:
    counts: Counter[str] = Counter()
    if not path.exists():
        return counts
    for file in path.glob("camera_pointing_*.png"):
        stem = file.stem
        parts = stem.split("_")
        # Expected: camera_pointing_00042_front_center_-074.95
        if len(parts) >= 5:
            camera = "_".join(parts[3:-1]) or "unknown"
        else:
            camera = "legacy_or_unknown"
        counts[camera] += 1
    return counts


def collect_timeline(events: list[dict[str, Any]], result: ValidationResult) -> None:
    for event in events:
        event_name = event.get("event")
        if event_name == "state_transition":
            append_state_if_new(result.state_path, event.get("from"))
            append_state_if_new(result.state_path, event.get("to"))
            if isinstance(event.get("to"), str):
                result.final_state = str(event["to"])
            height = event.get("ego_height_m")
            if event.get("to") == "Abort" and isinstance(height, (int, float)):
                result.abort_height_m = float(height)
        elif event_name == "runtime_stop":
            append_state_if_new(result.state_path, event.get("state"))
            if isinstance(event.get("state"), str):
                result.final_state = str(event["state"])
        elif event_name == "command_result":
            if not bool(event.get("success")):
                result.failures.append(
                    f"{event_tick(event)}: command {event.get('command', 'unknown')} failed: {event.get('status', '')}"
                )
        elif event_name == "command_exception":
            result.failures.append(
                f"{event_tick(event)}: command {event.get('command', 'unknown')} exception: {event.get('error', '')}"
            )

        if event_name == "command_dispatch" and event.get("command") == "Velocity":
            result.velocity_commands += 1
        if isinstance(event_name, str) and event_name in OBJECT_BEHAVIOR_EVENTS:
            result.behavior_events[event_name] = result.behavior_events.get(event_name, 0) + 1
        if isinstance(event_name, str) and event_name in SEQUENCE_EVENTS:
            result.sequence_events[event_name] = result.sequence_events.get(event_name, 0) + 1
            step = str(event.get("step_behavior") or "unknown")
            if event_name == "behavior_sequence_step_start":
                result.sequence_started_steps.append(step)
            elif event_name == "behavior_sequence_step_complete":
                result.sequence_completed_steps.append(step)
        if isinstance(event_name, str) and event_name in CAMERA_POINTING_EVENTS:
            result.camera_pointing_events[event_name] = result.camera_pointing_events.get(event_name, 0) + 1
        if event_name == "camera_pointing_intent":
            mode = str(event.get("camera_pointing_mode") or event.get("mode") or "unknown")
            result.camera_pointing_modes[mode] = result.camera_pointing_modes.get(mode, 0) + 1


def validate_state_order(result: ValidationResult, expected_order: list[str]) -> None:
    if not result.state_path:
        result.failures.append("no mission state transitions found")
        return

    search_from = 0
    for expected in expected_order:
        try:
            index = result.state_path.index(expected, search_from)
        except ValueError:
            result.failures.append(
                "missing required mission state in order: "
                f"{expected}; observed path: {' -> '.join(result.state_path)}"
            )
            return
        search_from = index + 1


def validate_height_gates(
    events: list[dict[str, Any]],
    result: ValidationResult,
    *,
    safe_height_m: float,
    landed_height_m: float,
    require_landed_height: bool,
) -> None:
    for event in events:
        if event.get("event") != "state_transition":
            continue
        to_state = event.get("to")
        height = event.get("ego_height_m")
        if to_state == "ExecuteMission" and isinstance(height, (int, float)):
            result.safe_height_gate_height_m = float(height)
        elif to_state == "Complete" and isinstance(height, (int, float)):
            result.landed_gate_height_m = float(height)

    if result.safe_height_gate_height_m is None:
        result.failures.append("missing ExecuteMission transition height gate evidence")
    elif result.safe_height_gate_height_m < safe_height_m:
        result.failures.append(
            "ExecuteMission reached below safe height: "
            f"height={result.safe_height_gate_height_m:.3f}m required>={safe_height_m:.3f}m"
        )

    if not require_landed_height:
        return
    if result.landed_gate_height_m is None:
        result.failures.append("missing Complete transition landed-height evidence")
    elif result.landed_gate_height_m > landed_height_m:
        result.failures.append(
            "Complete reached above landed height: "
            f"height={result.landed_gate_height_m:.3f}m required<={landed_height_m:.3f}m"
        )


def validate_expected_final_state(
    events: list[dict[str, Any]],
    result: ValidationResult,
    *,
    expected_final_state: str,
    safe_height_m: float,
    landed_height_m: float,
) -> None:
    if result.final_state != expected_final_state:
        result.failures.append(f"final_state is {result.final_state}; expected {expected_final_state}")

    if expected_final_state == "Complete":
        validate_state_order(result, COMPLETE_STATE_ORDER)
        if "Abort" in result.state_path:
            result.failures.append(f"Abort appeared in successful Complete lifecycle: {' -> '.join(result.state_path)}")
        validate_height_gates(
            events,
            result,
            safe_height_m=safe_height_m,
            landed_height_m=landed_height_m,
            require_landed_height=True,
        )
    elif expected_final_state == "Abort":
        validate_state_order(result, ABORT_AFTER_FLIGHT_STATE_ORDER)
        if "Complete" in result.state_path:
            result.failures.append(f"Complete appeared in expected Abort lifecycle: {' -> '.join(result.state_path)}")
        validate_height_gates(
            events,
            result,
            safe_height_m=safe_height_m,
            landed_height_m=landed_height_m,
            require_landed_height=False,
        )
    else:
        result.failures.append(f"unsupported expected final state: {expected_final_state}")


def validate_behavior_expectations(result: ValidationResult) -> None:
    if result.behavior_events.get("target_selected", 0) == 0:
        result.failures.append("expected object behavior event target_selected")
    if result.behavior_events.get("behavior_start", 0) == 0:
        result.failures.append("expected object behavior event behavior_start")
    if result.behavior_events.get("behavior_complete", 0) == 0:
        result.failures.append("expected object behavior event behavior_complete")
    if result.velocity_commands == 0:
        result.failures.append("expected velocity commands during behavior run")


def validate_sequence_expectations(result: ValidationResult, expected_steps: list[str]) -> None:
    if result.sequence_events.get("behavior_sequence_step_start", 0) == 0:
        result.failures.append("expected behavior_sequence_step_start events")
    if result.sequence_events.get("behavior_sequence_step_complete", 0) == 0:
        result.failures.append("expected behavior_sequence_step_complete events")
    for expected in expected_steps:
        if expected not in result.sequence_started_steps:
            result.failures.append(f"expected sequence step start for {expected}")
    if len(expected_steps) > 1:
        for expected in expected_steps[:-1]:
            if expected not in result.sequence_completed_steps:
                result.failures.append(f"expected sequence step complete for {expected}")
    if expected_steps:
        observed_prefix = result.sequence_started_steps[: len(expected_steps)]
        if observed_prefix != expected_steps:
            result.failures.append(
                "sequence step start order mismatch: "
                f"observed={observed_prefix} expected={expected_steps}"
            )


def validate_camera_pointing_expectations(
    result: ValidationResult,
    *,
    expected_modes: list[str],
    camera_frames_dir: Path | None,
    expect_camera_proof_frames: bool,
) -> None:
    if result.camera_pointing_events.get("camera_pointing_intent", 0) == 0:
        result.failures.append("expected camera_pointing_intent events")
    if result.camera_pointing_events.get("camera_pointing_dispatch", 0) == 0:
        result.failures.append("expected camera_pointing_dispatch events")
    if result.camera_pointing_events.get("camera_pointing_result", 0) == 0:
        result.failures.append("expected camera_pointing_result events")

    for mode in expected_modes:
        if result.camera_pointing_modes.get(mode, 0) == 0:
            result.failures.append(f"expected camera_pointing_mode {mode}")

    if camera_frames_dir is not None:
        result.camera_proof_frames = dict(camera_frame_counts(camera_frames_dir))

    if expect_camera_proof_frames:
        if camera_frames_dir is None:
            result.failures.append("--expect-camera-proof-frames requires --camera-frames-dir")
        elif not result.camera_proof_frames:
            result.failures.append(f"expected camera proof frames in {camera_frames_dir}")


def validate_run_dir(
    run_dir: Path,
    *,
    expected_final_state: str | None,
    expect_behavior: bool,
    expect_sequence: bool,
    expect_sequence_steps: list[str],
    expect_camera_pointing: bool,
    expect_camera_modes: list[str],
    camera_frames_dir: Path | None,
    expect_camera_proof_frames: bool,
    safe_height_m: float,
    landed_height_m: float,
    allow_missing_snapshots: bool,
) -> ValidationResult:
    result = ValidationResult()

    if not run_dir.exists() or not run_dir.is_dir():
        result.failures.append(f"run artifact directory not found: {run_dir}")
        return result

    events_path = run_dir / "mission_events.jsonl"
    if not events_path.exists():
        result.failures.append(f"mission_events.jsonl not found in: {run_dir}")
        return result

    result.snapshot_count = count_snapshots(run_dir)
    if result.snapshot_count == 0 and not allow_missing_snapshots:
        result.failures.append("no snapshot artifacts found; expected snapshot_manifest.txt or snapshot_*.json")

    events = read_events(events_path, result)
    collect_timeline(events, result)

    if expected_final_state is not None:
        validate_expected_final_state(
            events,
            result,
            expected_final_state=expected_final_state,
            safe_height_m=safe_height_m,
            landed_height_m=landed_height_m,
        )

    if expect_behavior:
        validate_behavior_expectations(result)

    if expect_sequence:
        validate_sequence_expectations(result, expect_sequence_steps)

    if expect_camera_pointing:
        validate_camera_pointing_expectations(
            result,
            expected_modes=expect_camera_modes,
            camera_frames_dir=camera_frames_dir,
            expect_camera_proof_frames=expect_camera_proof_frames,
        )

    return result


def print_result(result: ValidationResult) -> None:
    print("Mission artifact validation:")
    print(f"  final_state: {result.final_state}")
    print(f"  events: {result.event_count}")
    print(f"  snapshots: {result.snapshot_count}")
    if result.state_path:
        print(f"  state_path: {' -> '.join(result.state_path)}")
    print(f"  velocity_commands: {result.velocity_commands}")
    if result.safe_height_gate_height_m is not None:
        print(f"  safe_height_gate_height_m: {result.safe_height_gate_height_m:.3f}")
    if result.landed_gate_height_m is not None:
        print(f"  landed_gate_height_m: {result.landed_gate_height_m:.3f}")
    if result.abort_height_m is not None:
        print(f"  abort_height_m: {result.abort_height_m:.3f}")
    if result.behavior_events:
        print("  behavior_events:")
        for name in sorted(result.behavior_events):
            print(f"    {name}: {result.behavior_events[name]}")
    if result.sequence_events:
        print("  sequence_events:")
        for name in sorted(result.sequence_events):
            print(f"    {name}: {result.sequence_events[name]}")
        print(f"  sequence_started_steps: {','.join(result.sequence_started_steps)}")
        print(f"  sequence_completed_steps: {','.join(result.sequence_completed_steps)}")
    if result.camera_pointing_events:
        print("  camera_pointing_events:")
        for name in sorted(result.camera_pointing_events):
            print(f"    {name}: {result.camera_pointing_events[name]}")
    if result.camera_pointing_modes:
        print("  camera_pointing_modes:")
        for mode in sorted(result.camera_pointing_modes):
            print(f"    {mode}: {result.camera_pointing_modes[mode]}")
    if result.camera_proof_frames:
        print("  camera_proof_frames:")
        for camera in sorted(result.camera_proof_frames):
            print(f"    {camera}: {result.camera_proof_frames[camera]}")
    print(f"  failures: {len(result.failures)}")
    for failure in result.failures:
        print(f"    - {failure}")
    for warning in result.warnings:
        print(f"  warning: {warning}")


def parse_csv(value: str) -> list[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path, help="Mission run artifact directory")
    parser.add_argument("--expect-complete", action="store_true", help="Require a successful Complete mission lifecycle")
    parser.add_argument(
        "--expect-final-state",
        choices=["Complete", "Abort"],
        help="Require a specific final mission state",
    )
    parser.add_argument("--expect-behavior", action="store_true", help="Require M3 object-conditioned behavior events")
    parser.add_argument("--expect-sequence", action="store_true", help="Require behavior sequence step events")
    parser.add_argument(
        "--expect-sequence-steps",
        default="",
        help="Comma-separated step_behavior values expected in sequence start order, e.g. approach,circle",
    )
    parser.add_argument("--expect-camera-pointing", action="store_true", help="Require camera-pointing intent/dispatch/result events")
    parser.add_argument(
        "--expect-camera-modes",
        default="",
        help="Comma-separated camera_pointing_mode values required when --expect-camera-pointing is set",
    )
    parser.add_argument("--camera-frames-dir", type=Path, default=None, help="Directory containing camera_pointing_*.png proof frames")
    parser.add_argument("--expect-camera-proof-frames", action="store_true", help="Require at least one AirSim camera proof frame")
    parser.add_argument(
        "--safe-height-m",
        type=float,
        default=1.0,
        help="Minimum ego_height_m required on transition to ExecuteMission",
    )
    parser.add_argument(
        "--landed-height-m",
        type=float,
        default=1.0,
        help="Maximum ego_height_m allowed on transition to Complete",
    )
    parser.add_argument(
        "--allow-missing-snapshots",
        action="store_true",
        help="Validate event-only fixtures without requiring snapshot artifacts",
    )
    args = parser.parse_args()

    expected_final_state = args.expect_final_state
    if args.expect_complete:
        if expected_final_state is not None and expected_final_state != "Complete":
            print("--expect-complete conflicts with --expect-final-state", file=sys.stderr)
            return 2
        expected_final_state = "Complete"

    result = validate_run_dir(
        args.run_dir,
        expected_final_state=expected_final_state,
        expect_behavior=args.expect_behavior,
        expect_sequence=args.expect_sequence,
        expect_sequence_steps=parse_csv(args.expect_sequence_steps),
        expect_camera_pointing=args.expect_camera_pointing,
        expect_camera_modes=parse_csv(args.expect_camera_modes),
        camera_frames_dir=args.camera_frames_dir,
        expect_camera_proof_frames=args.expect_camera_proof_frames,
        safe_height_m=args.safe_height_m,
        landed_height_m=args.landed_height_m,
        allow_missing_snapshots=args.allow_missing_snapshots,
    )
    print_result(result)
    return 0 if result.valid else 1


if __name__ == "__main__":
    raise SystemExit(main())
