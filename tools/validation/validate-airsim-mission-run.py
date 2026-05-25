#!/usr/bin/env python3
"""Validate and summarize a Dedalus AirSim mission run.

This validator is intentionally artifact-based. It can wait for
mission_events.jsonl to receive runtime_stop, then checks:

- terminal lifecycle settled
- behavior_complete reason
- camera pointing lifecycle modes
- AirSim camera proof frame counts
- circle/orbit validator result

It is designed for simulation/airsim/run_mission.sh post-run validation.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from collections import Counter
from pathlib import Path
from typing import Any


def load_events(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    events: list[dict[str, Any]] = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        stripped = line.strip()
        if not stripped:
            continue
        try:
            event = json.loads(stripped)
        except json.JSONDecodeError as exc:
            raise ValueError(f"{path}:{line_number}: invalid JSON: {exc}") from exc
        if isinstance(event, dict):
            events.append(event)
    return events


def as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in {"1", "true", "yes", "on"}
    return bool(value)


def last_event(events: list[dict[str, Any]], name: str) -> dict[str, Any] | None:
    for event in reversed(events):
        if event.get("event") == name:
            return event
    return None


def state_path(events: list[dict[str, Any]]) -> list[str]:
    path: list[str] = []
    for event in events:
        if event.get("event") != "state_transition":
            continue
        from_state = event.get("from")
        to_state = event.get("to")
        if from_state and not path:
            path.append(str(from_state))
        if to_state and (not path or path[-1] != str(to_state)):
            path.append(str(to_state))
    return path


def wait_for_runtime_stop(path: Path, timeout_s: float, poll_s: float) -> list[dict[str, Any]]:
    start = time.monotonic()
    last_count = -1
    while True:
        events = load_events(path)
        if len(events) != last_count:
            print(f"validate-airsim-mission-run: events={len(events)}", flush=True)
            last_count = len(events)
        if last_event(events, "runtime_stop") is not None:
            return events
        if timeout_s > 0.0 and time.monotonic() - start >= timeout_s:
            raise TimeoutError(f"timed out waiting for runtime_stop in {path}")
        time.sleep(max(0.1, poll_s))


def camera_modes(events: list[dict[str, Any]]) -> Counter[str]:
    modes: Counter[str] = Counter()
    for event in events:
        if event.get("event") == "camera_pointing_intent":
            mode = event.get("camera_pointing_mode") or event.get("mode") or "unknown"
            modes[str(mode)] += 1
    return modes


def camera_mode_states(events: list[dict[str, Any]]) -> list[tuple[str, str, float | None]]:
    rows: list[tuple[str, str, float | None]] = []
    last_key: tuple[str, str] | None = None
    for event in events:
        if event.get("event") != "camera_pointing_intent":
            continue
        state = str(event.get("state") or "?")
        mode = str(event.get("camera_pointing_mode") or event.get("mode") or "unknown")
        key = (state, mode)
        if key == last_key:
            continue
        last_key = key
        pitch = event.get("pitch_deg")
        try:
            pitch_value = float(pitch)
        except (TypeError, ValueError):
            pitch_value = None
        rows.append((state, mode, pitch_value))
    return rows


def camera_frame_counts(path: Path) -> Counter[str]:
    counts: Counter[str] = Counter()
    if not path.exists():
        return counts
    for file in path.glob("camera_pointing_*.png"):
        # Expected form: camera_pointing_00042_front_center_-074.95.png
        stem = file.stem
        parts = stem.split("_")
        if len(parts) < 4:
            counts["legacy_or_unknown"] += 1
            continue
        camera = "_".join(parts[3:-1]) or "unknown"
        counts[camera] += 1
    return counts


def run_circle_validator(args: argparse.Namespace) -> int:
    cmd = [
        sys.executable,
        str(args.circle_validator),
        "--events",
        str(args.events),
        "--min-orbits",
        str(args.min_orbits),
        "--radius",
        str(args.radius),
        "--avg-radius-error-max",
        str(args.avg_radius_error_max),
        "--max-radius-error-after-latch",
        str(args.max_radius_error_after_latch),
        "--expect-complete-reason",
        args.expect_complete_reason,
        "--require-terminal-settled",
        "--require-lifecycle",
    ]
    print("\nRunning circle validator:")
    print("  " + " ".join(cmd))
    completed = subprocess.run(cmd, check=False)
    return int(completed.returncode)


def stop_tmux_session(session: str) -> None:
    if not session:
        return
    subprocess.run(["tmux", "kill-session", "-t", session], check=False)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--events", type=Path, required=True)
    parser.add_argument("--camera-debug-json", type=Path, default=None)
    parser.add_argument("--camera-frames-dir", type=Path, default=None)
    parser.add_argument("--circle-validator", type=Path, default=Path("tools/validation/validate-circle-trajectory.py"))
    parser.add_argument("--wait", action="store_true", help="Wait until runtime_stop appears in mission_events.jsonl")
    parser.add_argument("--wait-timeout-s", type=float, default=0.0, help="0 means wait forever")
    parser.add_argument("--poll-s", type=float, default=2.0)
    parser.add_argument("--min-orbits", type=float, default=1.0)
    parser.add_argument("--radius", type=float, default=10.0)
    parser.add_argument("--avg-radius-error-max", type=float, default=1.0)
    parser.add_argument("--max-radius-error-after-latch", type=float, default=3.0)
    parser.add_argument("--expect-complete-reason", default="orbit_count_elapsed")
    parser.add_argument("--require-camera-modes", default="neutral,target,home,landing_area")
    parser.add_argument("--require-camera-frames", action="store_true")
    parser.add_argument("--stop-tmux-session", default="", help="Kill this tmux session after validation completes")
    parser.add_argument("--no-stop-on-fail", action="store_true", help="Do not stop tmux session if validation fails")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    events = wait_for_runtime_stop(args.events, args.wait_timeout_s, args.poll_s) if args.wait else load_events(args.events)

    runtime_stop = last_event(events, "runtime_stop")
    behavior_complete = last_event(events, "behavior_complete")
    modes = camera_modes(events)
    required_modes = [mode.strip() for mode in args.require_camera_modes.split(",") if mode.strip()]
    missing_modes = [mode for mode in required_modes if modes.get(mode, 0) <= 0]
    frame_counts = camera_frame_counts(args.camera_frames_dir) if args.camera_frames_dir is not None else Counter()

    print("\nAirSim mission run summary")
    print(f"  events: {len(events)}")
    print(f"  runtime_stop terminal_settled: {runtime_stop.get('terminal_settled') if runtime_stop else None}")
    print(f"  behavior_complete reason: {behavior_complete.get('reason') if behavior_complete else None}")
    path = state_path(events)
    print(f"  state path: {' -> '.join(path) if path else 'n/a'}")
    print("  camera modes:")
    for mode, count in sorted(modes.items()):
        print(f"    {mode}: {count}")
    print("  camera mode transitions:")
    for state, mode, pitch in camera_mode_states(events):
        pitch_text = "n/a" if pitch is None else f"{pitch:.2f} deg"
        print(f"    {state:14s} {mode:14s} {pitch_text}")
    if args.camera_frames_dir is not None:
        print("  camera proof frames:")
        if frame_counts:
            for camera, count in sorted(frame_counts.items()):
                print(f"    {camera}: {count}")
        else:
            print(f"    none found in {args.camera_frames_dir}")

    checks: list[tuple[str, bool, str]] = []
    checks.append((
        "terminal_settled",
        runtime_stop is not None and as_bool(runtime_stop.get("terminal_settled")),
        str(runtime_stop.get("terminal_settled") if runtime_stop else None),
    ))
    checks.append((
        "behavior_complete_reason",
        behavior_complete is not None and behavior_complete.get("reason") == args.expect_complete_reason,
        str(behavior_complete.get("reason") if behavior_complete else None),
    ))
    checks.append((
        "camera_modes",
        not missing_modes,
        "missing=" + ",".join(missing_modes) if missing_modes else "all required modes present",
    ))
    if args.require_camera_frames and args.camera_frames_dir is not None:
        checks.append((
            "camera_proof_frames",
            bool(frame_counts),
            f"total={sum(frame_counts.values())}",
        ))

    print("\nMission run checks")
    local_ok = True
    for name, passed, detail in checks:
        local_ok = local_ok and passed
        print(f"  {'PASS' if passed else 'FAIL'} {name}: {detail}")

    circle_rc = run_circle_validator(args)
    overall_ok = local_ok and circle_rc == 0
    print(f"\nAirSim mission validation: {'PASS' if overall_ok else 'FAIL'}")

    if args.stop_tmux_session and (overall_ok or not args.no_stop_on_fail):
        print(f"Stopping tmux session: {args.stop_tmux_session}")
        stop_tmux_session(args.stop_tmux_session)

    return 0 if overall_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
