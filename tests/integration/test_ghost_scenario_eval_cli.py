#!/usr/bin/env python3
"""Smoke-test the shared ghost scenario evaluator CLI."""

from __future__ import annotations

import json
import math
import subprocess
import sys
from pathlib import Path


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def require_close(actual: float, expected: float, tolerance: float, message: str) -> None:
    if abs(actual - expected) > tolerance:
        raise AssertionError(f"{message}: actual={actual} expected={expected} tolerance={tolerance}")


def find_detection(payload: dict, source_track_id: str) -> dict:
    for detection in payload.get("detections", []):
        if detection.get("source_track_id") == source_track_id:
            return detection
    raise AssertionError(f"missing detection: {source_track_id}")


def require_vec3(actual: list[float], expected: tuple[float, float, float], message: str) -> None:
    require(len(actual) == 3, f"{message}: expected 3-vector, got {actual}")
    for index, expected_value in enumerate(expected):
        require_close(float(actual[index]), expected_value, 1.0e-6, f"{message}[{index}]")


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: test_ghost_scenario_eval_cli.py <repo-root> <build-dir>", file=sys.stderr)
        return 2

    repo_root = Path(sys.argv[1]).resolve()
    build_dir = Path(sys.argv[2]).resolve()
    app = build_dir / "apps" / "dedalus_ghost_scenario_eval"
    scenario = repo_root / "simulation" / "ghost_detections" / "person_pair_crossing.json"

    require(app.exists(), f"missing evaluator binary: {app}")
    require(scenario.exists(), f"missing scenario fixture: {scenario}")

    result = subprocess.run(
        [str(app), "--scenario", str(scenario), "--time-s", "10"],
        cwd=repo_root,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    payload = json.loads(result.stdout)

    require(payload.get("scenario") == "person_pair_crossing", f"unexpected scenario: {payload.get('scenario')}")
    require(payload.get("map_frame_id") == "map_airsim_mission_0001", f"unexpected map_frame_id: {payload.get('map_frame_id')}")
    require_close(float(payload.get("time_s")), 10.0, 1.0e-9, "time_s")

    p1 = find_detection(payload, "ghost_person_001")
    p2 = find_detection(payload, "ghost_person_002")
    car = find_detection(payload, "ghost_car_001")

    require(p1.get("class") == "person", "p1 class")
    require(p2.get("class") == "person", "p2 class")
    require(car.get("class") == "car", "car class")

    require_vec3(p1.get("position_local_m", []), (15.0, -4.0, 0.0), "p1 position at 10s")
    require_vec3(p1.get("velocity_local_mps", []), (0.3, 0.0, 0.0), "p1 velocity")
    require_vec3(p2.get("position_local_m", []), (6.0, 4.0, 0.0), "p2 position at 10s")
    require_vec3(p2.get("velocity_local_mps", []), (-0.2, 0.0, 0.0), "p2 velocity")
    require_vec3(car.get("position_local_m", []), (4.0, 0.0, 0.0), "car static position")
    require_vec3(car.get("velocity_local_mps", []), (0.0, 0.0, 0.0), "car static velocity")

    # The CLI must fail cleanly for invalid input so downstream visualizers can
    # treat it as a hard scenario authoring error.
    failed = subprocess.run(
        [str(app), "--scenario", str(scenario), "--time-s", "-1"],
        cwd=repo_root,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    require(failed.returncode != 0, "negative time should fail")
    require("--time-s must be >= 0" in failed.stderr, f"unexpected failure text: {failed.stderr}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
