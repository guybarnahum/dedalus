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
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

EXPECTED_STATE_ORDER = ["Prepare", "Takeoff", "ExecuteMission", "GoHome", "Land", "Complete"]
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
    velocity_commands: int = 0
    safe_height_gate_height_m: float | None = None
    landed_gate_height_m: float | None = None

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


def collect_timeline(events: list[dict[str, Any]], result: ValidationResult) -> None:
    for event in events:
        event_name = event.get("event")
        if event_name == "state_transition":
            append_state_if_new(result.state_path, event.get("from"))
            append_state_if_new(result.state_path, event.get("to"))
            if isinstance(event.get("to"), str):
                result.final_state = str(event["to"])
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


def validate_state_order(result: ValidationResult) -> None:
    if not result.state_path:
        result.failures.append("no mission state transitions found")
        return

    search_from = 0
    for expected in EXPECTED_STATE_ORDER:
        try:
            index = result.state_path.index(expected, search_from)
        except ValueError:
            result.failures.append(
                "missing required mission state in order: "
                f"{expected}; observed path: {' -> '.join(result.state_path)}"
            )
            return
        search_from = index + 1

    complete_index = result.state_path.index("Complete")
    if "Abort" in result.state_path[: complete_index + 1]:
        result.failures.append(f"Abort appeared before Complete: {' -> '.join(result.state_path)}")


def validate_height_gates(
    events: list[dict[str, Any]],
    result: ValidationResult,
    *,
    safe_height_m: float,
    landed_height_m: float,
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

    if result.landed_gate_height_m is None:
        result.failures.append("missing Complete transition landed-height evidence")
    elif result.landed_gate_height_m > landed_height_m:
        result.failures.append(
            "Complete reached above landed height: "
            f"height={result.landed_gate_height_m:.3f}m required<={landed_height_m:.3f}m"
        )


def validate_behavior_expectations(result: ValidationResult) -> None:
    if result.behavior_events.get("target_selected", 0) == 0:
        result.failures.append("expected object behavior event target_selected")
    if result.behavior_events.get("behavior_start", 0) == 0:
        result.failures.append("expected object behavior event behavior_start")
    if result.behavior_events.get("behavior_complete", 0) == 0:
        result.failures.append("expected object behavior event behavior_complete")
    if result.velocity_commands == 0:
        result.failures.append("expected velocity commands during behavior run")


def validate_run_dir(
    run_dir: Path,
    *,
    expect_complete: bool,
    expect_behavior: bool,
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

    if expect_complete:
        if result.final_state != "Complete":
            result.failures.append(f"final_state is {result.final_state}; expected Complete")
        validate_state_order(result)
        validate_height_gates(
            events,
            result,
            safe_height_m=safe_height_m,
            landed_height_m=landed_height_m,
        )

    if expect_behavior:
        validate_behavior_expectations(result)

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
    if result.behavior_events:
        print("  behavior_events:")
        for name in sorted(result.behavior_events):
            print(f"    {name}: {result.behavior_events[name]}")
    print(f"  failures: {len(result.failures)}")
    for failure in result.failures:
        print(f"    - {failure}")
    for warning in result.warnings:
        print(f"  warning: {warning}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path, help="Mission run artifact directory")
    parser.add_argument("--expect-complete", action="store_true", help="Require a successful Complete mission lifecycle")
    parser.add_argument("--expect-behavior", action="store_true", help="Require M3 object-conditioned behavior events")
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

    result = validate_run_dir(
        args.run_dir,
        expect_complete=args.expect_complete,
        expect_behavior=args.expect_behavior,
        safe_height_m=args.safe_height_m,
        landed_height_m=args.landed_height_m,
        allow_missing_snapshots=args.allow_missing_snapshots,
    )
    print_result(result)
    return 0 if result.valid else 1


if __name__ == "__main__":
    raise SystemExit(main())
