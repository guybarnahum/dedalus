#!/usr/bin/env python3
"""Validate Dedalus circle/orbit behavior from mission_events.jsonl.

This script is intentionally artifact-only. It does not connect to AirSim, PX4,
or the runtime stream. It validates the behavior controller's durable mission
records so live runs can be checked after the fact and CI can use deterministic
fixtures.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable


@dataclass
class CheckResult:
    name: str
    passed: bool
    detail: str


def as_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in {"true", "1", "yes", "on"}
    return bool(value)


def load_events(path: Path) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    for line_number, line in enumerate(path.read_text().splitlines(), start=1):
        stripped = line.strip()
        if not stripped:
            continue
        try:
            decoded = json.loads(stripped)
        except json.JSONDecodeError as exc:
            raise ValueError(f"{path}:{line_number}: invalid JSON: {exc}") from exc
        if isinstance(decoded, dict):
            events.append(decoded)
    return events


def samples_with_circle_fields(events: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
    return [
        event
        for event in events
        if event.get("event") in {"behavior_debug", "behavior_tick_sample"}
        and "circle_phase" in event
    ]


def numeric_values(samples: Iterable[dict[str, Any]], key: str) -> list[float]:
    values: list[float] = []
    for sample in samples:
        if key in sample:
            values.append(as_float(sample[key]))
    return values


def min_avg_max(values: list[float]) -> tuple[float, float, float] | None:
    if not values:
        return None
    return min(values), statistics.fmean(values), max(values)


def fmt_triplet(values: tuple[float, float, float] | None) -> str:
    if values is None:
        return "n/a"
    return f"{values[0]:.3f} / {values[1]:.3f} / {values[2]:.3f}"


def last_event(events: Iterable[dict[str, Any]], event_name: str) -> dict[str, Any] | None:
    matched = [event for event in events if event.get("event") == event_name]
    return matched[-1] if matched else None


def ordered_state_path(events: Iterable[dict[str, Any]]) -> list[str]:
    states: list[str] = []
    for event in events:
        if event.get("event") != "state_transition":
            continue
        from_state = event.get("from")
        to_state = event.get("to")
        if from_state and not states:
            states.append(str(from_state))
        if to_state:
            to_state = str(to_state)
            if not states or states[-1] != to_state:
                states.append(to_state)
    return states


def has_subsequence(values: list[str], subsequence: list[str]) -> bool:
    cursor = 0
    for value in values:
        if value == subsequence[cursor]:
            cursor += 1
            if cursor == len(subsequence):
                return True
    return False


def build_checks(
    *,
    events: list[dict[str, Any]],
    circle_samples: list[dict[str, Any]],
    circling_samples: list[dict[str, Any]],
    post_latch_samples: list[dict[str, Any]],
    args: argparse.Namespace,
) -> list[CheckResult]:
    checks: list[CheckResult] = []

    checks.append(CheckResult(
        "target_selected",
        any(event.get("event") == "target_selected" for event in events),
        "target_selected event present",
    ))
    checks.append(CheckResult(
        "circle_samples",
        bool(circle_samples),
        f"circle samples={len(circle_samples)}",
    ))
    checks.append(CheckResult(
        "circling_samples",
        bool(circling_samples),
        f"circling samples={len(circling_samples)}",
    ))
    checks.append(CheckResult(
        "orbit_mode_latched",
        any(as_bool(sample.get("orbit_mode_latched")) for sample in circle_samples),
        "orbit_mode_latched=true observed",
    ))

    if args.radius is not None:
        target_radii = numeric_values(circle_samples, "orbit_radius_m")
        radius_seen = any(abs(value - args.radius) <= args.radius_epsilon for value in target_radii)
        checks.append(CheckResult(
            "orbit_radius",
            radius_seen,
            f"expected radius={args.radius:.3f} epsilon={args.radius_epsilon:.3f}",
        ))

    completed_orbits = numeric_values(circle_samples, "circle_completed_orbits")
    last_orbit = completed_orbits[-1] if completed_orbits else 0.0
    if args.min_orbits > 0.0:
        threshold = max(0.0, args.min_orbits - args.orbit_epsilon)
        checks.append(CheckResult(
            "completed_orbits",
            last_orbit >= threshold,
            f"last={last_orbit:.3f} required>={threshold:.3f}",
        ))

    if args.expect_complete_reason:
        complete = last_event(events, "behavior_complete")
        actual_reason = complete.get("reason") if complete else None
        checks.append(CheckResult(
            "behavior_complete_reason",
            actual_reason == args.expect_complete_reason,
            f"actual={actual_reason!r} expected={args.expect_complete_reason!r}",
        ))

    if args.avg_radius_error_max is not None:
        radius_errors = [abs(value) for value in numeric_values(circling_samples, "radius_error_m")]
        avg_error = statistics.fmean(radius_errors) if radius_errors else math.inf
        checks.append(CheckResult(
            "avg_abs_radius_error",
            avg_error <= args.avg_radius_error_max,
            f"actual={avg_error:.3f} max={args.avg_radius_error_max:.3f}",
        ))

    if args.max_radius_error_after_latch is not None:
        radius_errors = [abs(value) for value in numeric_values(post_latch_samples, "radius_error_m")]
        max_error = max(radius_errors) if radius_errors else math.inf
        checks.append(CheckResult(
            "max_radius_error_after_latch",
            max_error <= args.max_radius_error_after_latch,
            f"actual={max_error:.3f} max={args.max_radius_error_after_latch:.3f}",
        ))

    if args.require_terminal_settled:
        runtime_stop = last_event(events, "runtime_stop")
        checks.append(CheckResult(
            "terminal_settled",
            runtime_stop is not None and as_bool(runtime_stop.get("terminal_settled")),
            f"runtime_stop_terminal_settled={runtime_stop.get('terminal_settled') if runtime_stop else None}",
        ))

    if args.require_lifecycle:
        path = ordered_state_path(events)
        required = ["GoHome", "Land", "Complete"]
        checks.append(CheckResult(
            "lifecycle_go_home_land_complete",
            has_subsequence(path, required),
            "state_path=" + " -> ".join(path),
        ))

    return checks


def print_summary(
    *,
    events: list[dict[str, Any]],
    circle_samples: list[dict[str, Any]],
    circling_samples: list[dict[str, Any]],
    post_latch_samples: list[dict[str, Any]],
    checks: list[CheckResult],
) -> None:
    completed_orbits = numeric_values(circle_samples, "circle_completed_orbits")
    radii = numeric_values(circling_samples, "actual_radius_m")
    radius_errors = numeric_values(circling_samples, "radius_error_m")
    post_latch_errors = numeric_values(post_latch_samples, "radius_error_m")
    tangent = numeric_values(circling_samples, "tangent_velocity_mps")
    radial = numeric_values(circling_samples, "radial_correction_mps")
    complete = last_event(events, "behavior_complete")
    runtime_stop = last_event(events, "runtime_stop")

    print("Circle trajectory summary")
    print(f"  events: {len(events)}")
    print(f"  circle samples: {len(circle_samples)}")
    print(f"  circling samples: {len(circling_samples)}")
    print(f"  post-latch samples: {len(post_latch_samples)}")
    if circle_samples:
        print(f"  orbit radius target: {circle_samples[-1].get('orbit_radius_m')}")
        print(f"  orbit mode latched observed: {any(as_bool(s.get('orbit_mode_latched')) for s in circle_samples)}")
    print(f"  actual radius min/avg/max: {fmt_triplet(min_avg_max(radii))}")
    print(f"  radius error min/avg/max: {fmt_triplet(min_avg_max(radius_errors))}")
    if radius_errors:
        print(f"  avg abs radius error: {statistics.fmean(abs(v) for v in radius_errors):.3f}")
    print(f"  post-latch radius error min/avg/max: {fmt_triplet(min_avg_max(post_latch_errors))}")
    print(f"  tangent velocity min/avg/max: {fmt_triplet(min_avg_max(tangent))}")
    print(f"  radial correction min/avg/max: {fmt_triplet(min_avg_max(radial))}")
    if completed_orbits:
        print(f"  completed orbits first/last: {completed_orbits[0]:.3f} -> {completed_orbits[-1]:.3f}")
    else:
        print("  completed orbits first/last: n/a")
    print(f"  behavior_complete reason: {complete.get('reason') if complete else None}")
    print(f"  runtime_stop terminal_settled: {runtime_stop.get('terminal_settled') if runtime_stop else None}")
    path = ordered_state_path(events)
    print(f"  state path: {' -> '.join(path) if path else 'n/a'}")
    print()
    print("Checks")
    for check in checks:
        prefix = "PASS" if check.passed else "FAIL"
        print(f"  {prefix} {check.name}: {check.detail}")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--events", required=True, type=Path, help="mission_events.jsonl path")
    parser.add_argument("--min-orbits", type=float, default=0.0, help="minimum completed orbit count")
    parser.add_argument("--orbit-epsilon", type=float, default=0.05, help="allowed orbit-count shortfall")
    parser.add_argument("--radius", type=float, default=None, help="expected orbit radius")
    parser.add_argument("--radius-epsilon", type=float, default=0.25, help="allowed radius target mismatch")
    parser.add_argument("--avg-radius-error-max", type=float, default=None, help="maximum average absolute radius error while circling")
    parser.add_argument("--max-radius-error-after-latch", type=float, default=None, help="maximum absolute radius error after orbit latch")
    parser.add_argument("--expect-complete-reason", default=None, help="required behavior_complete reason")
    parser.add_argument("--require-terminal-settled", action="store_true", help="require runtime_stop terminal_settled=true")
    parser.add_argument("--require-lifecycle", action="store_true", help="require GoHome -> Land -> Complete in state_transition events")
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    events = load_events(args.events)
    circle_samples = samples_with_circle_fields(events)
    circling_samples = [sample for sample in circle_samples if sample.get("circle_phase") == "circling"]
    post_latch_samples = [sample for sample in circle_samples if as_bool(sample.get("orbit_mode_latched"))]
    checks = build_checks(
        events=events,
        circle_samples=circle_samples,
        circling_samples=circling_samples,
        post_latch_samples=post_latch_samples,
        args=args,
    )
    print_summary(
        events=events,
        circle_samples=circle_samples,
        circling_samples=circling_samples,
        post_latch_samples=post_latch_samples,
        checks=checks,
    )
    return 0 if all(check.passed for check in checks) else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
