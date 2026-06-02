#!/usr/bin/env python3
"""Validate Dedalus live mission run artifact directories."""

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
    "target_selected", "target_reacquired", "target_lost", "behavior_start",
    "behavior_tick_sample", "behavior_complete", "behavior_failed", "fallback_start",
    "fallback_complete", "behavior_sequence_step_start", "behavior_sequence_step_complete",
}
CAMERA_POINTING_EVENTS = {"camera_pointing_intent", "camera_pointing_dispatch", "camera_pointing_result"}
SEQUENCE_EVENTS = {"behavior_sequence_step_start", "behavior_sequence_step_complete"}
VALID_OCCUPANCY_CELL_STATES = {"free", "occupied", "unknown"}
VALID_OCCUPANCY_SOURCE_KINDS = {
    "synthetic_fixture", "airsim_ground_truth", "visual_obstacle_detector", "depth_provider", "fused",
}
VALID_SWEPT_VOLUME_STATUSES = {"unknown", "clear", "occupied_blocked", "unknown_risk", "stale_map"}


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
    sequence_step_modes: list[tuple[str, str, str]] = field(default_factory=list)
    camera_pointing_events: dict[str, int] = field(default_factory=dict)
    camera_pointing_modes: dict[str, int] = field(default_factory=dict)
    camera_proof_frames: dict[str, int] = field(default_factory=dict)
    velocity_commands: int = 0
    safe_height_gate_height_m: float | None = None
    landed_gate_height_m: float | None = None
    abort_height_m: float | None = None
    occupancy_snapshots_checked: int = 0
    occupancy_debug_cells_checked: int = 0
    occupancy_sidecars_checked: int = 0
    occupancy_projected_cells_checked: int = 0
    occupancy_source_kinds: dict[str, int] = field(default_factory=dict)
    swept_volume_snapshots_checked: int = 0
    swept_volume_sidecars_checked: int = 0
    swept_volume_statuses: dict[str, int] = field(default_factory=dict)
    swept_volume_blocking_cells_checked: int = 0

    @property
    def valid(self) -> bool:
        return not self.failures


def append_state_if_new(states: list[str], state: Any) -> None:
    if isinstance(state, str) and state and (not states or states[-1] != state):
        states.append(state)


def event_tick(event: dict[str, Any]) -> str:
    tick = event.get("tick")
    return f"tick {tick}" if isinstance(tick, int) else "unknown tick"


def read_events(path: Path, result: ValidationResult) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        result.failures.append(f"failed to open mission_events.jsonl: {exc}")
        return events
    for line_number, raw_line in enumerate(lines, start=1):
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


def snapshot_path_from_manifest_line(line: str) -> Path | None:
    stripped = line.strip()
    if not stripped or stripped.startswith("#"):
        return None
    parts = stripped.split()
    if not parts:
        return None
    candidate = parts[1] if len(parts) >= 2 and parts[0].isdigit() else parts[0]
    return Path(candidate)


def snapshot_paths(run_dir: Path) -> list[Path]:
    manifest = run_dir / "snapshot_manifest.txt"
    if manifest.exists():
        paths: list[Path] = []
        try:
            for raw in manifest.read_text(encoding="utf-8").splitlines():
                entry = snapshot_path_from_manifest_line(raw)
                if entry is not None:
                    paths.append(entry if entry.is_absolute() else run_dir / entry)
        except OSError:
            return []
        return paths
    return sorted(run_dir.glob("snapshot_*.json"))


def count_snapshots(run_dir: Path) -> int:
    return len(snapshot_paths(run_dir))


def camera_frame_counts(path: Path) -> Counter[str]:
    counts: Counter[str] = Counter()
    if not path.exists():
        return counts
    for file in path.glob("camera_pointing_*.png"):
        parts = file.stem.split("_")
        camera = "_".join(parts[3:-1]) if len(parts) >= 5 else "legacy_or_unknown"
        counts[camera or "unknown"] += 1
    return counts


def read_json_object(path: Path, result: ValidationResult, label: str) -> dict[str, Any] | None:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except OSError as exc:
        result.failures.append(f"failed to read {label} {path}: {exc}")
        return None
    except json.JSONDecodeError as exc:
        result.failures.append(f"invalid JSON in {label} {path}: {exc}")
        return None
    if not isinstance(data, dict):
        result.failures.append(f"{label} {path} is not a JSON object")
        return None
    return data


def is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def is_vec3(value: Any) -> bool:
    return isinstance(value, list) and len(value) == 3 and all(is_number(item) for item in value)


def validate_occupancy_debug_cell(cell: Any, result: ValidationResult, *, path: Path, index: int) -> None:
    prefix = f"snapshot occupancy debug_cells[{index}] in {path}"
    if not isinstance(cell, dict):
        result.failures.append(f"{prefix} is not an object")
        return
    if cell.get("state") not in VALID_OCCUPANCY_CELL_STATES:
        result.failures.append(f"{prefix} has invalid state: {cell.get('state')!r}")
    for key in ("center_local", "size_m"):
        if not is_vec3(cell.get(key)):
            result.failures.append(f"{prefix} missing vec3 {key}")
    for key in ("confidence", "distance_to_nearest_occupied_m"):
        if not is_number(cell.get(key)):
            result.failures.append(f"{prefix} missing numeric {key}")
    result.occupancy_debug_cells_checked += 1


def validate_snapshot_occupancy(snapshot: dict[str, Any], result: ValidationResult, *, path: Path) -> None:
    occupancy = snapshot.get("ego_occupancy")
    if not isinstance(occupancy, dict):
        result.failures.append(f"snapshot {path} missing ego_occupancy object")
        return
    source_kind = occupancy.get("source_kind")
    if source_kind not in VALID_OCCUPANCY_SOURCE_KINDS:
        result.failures.append(f"snapshot {path} has invalid occupancy source_kind: {source_kind!r}")
    else:
        result.occupancy_source_kinds[source_kind] = result.occupancy_source_kinds.get(source_kind, 0) + 1
    if not occupancy.get("has_valid_occupancy"):
        result.failures.append(f"snapshot {path} occupancy has_valid_occupancy is not true")
    for key in ("source_provider", "map_frame_id"):
        if not isinstance(occupancy.get(key), str) or not occupancy.get(key):
            result.failures.append(f"snapshot {path} occupancy missing {key}")
    if not is_number(occupancy.get("resolution_m")):
        result.failures.append(f"snapshot {path} occupancy missing numeric resolution_m")
    if not is_vec3(occupancy.get("size_m")):
        result.failures.append(f"snapshot {path} occupancy missing vec3 size_m")
    for key in ("occupied_count", "free_count", "unknown_count", "stale_count"):
        if not isinstance(occupancy.get(key), int) or occupancy.get(key) < 0:
            result.failures.append(f"snapshot {path} occupancy missing nonnegative integer {key}")
    for key in ("nearest_obstacle_distance_m", "forward_corridor_clearance_m"):
        if not is_number(occupancy.get(key)):
            result.failures.append(f"snapshot {path} occupancy missing numeric {key}")
    debug_cells = occupancy.get("debug_cells")
    if not isinstance(debug_cells, list) or not debug_cells:
        result.failures.append(f"snapshot {path} occupancy missing non-empty debug_cells")
    else:
        for index, cell in enumerate(debug_cells[:32]):
            validate_occupancy_debug_cell(cell, result, path=path, index=index)
    result.occupancy_snapshots_checked += 1


def validate_swept_volume(snapshot: dict[str, Any], result: ValidationResult, *, path: Path) -> None:
    swept = snapshot.get("latest_swept_volume")
    if not isinstance(swept, dict):
        result.failures.append(f"snapshot {path} missing latest_swept_volume object")
        return
    status = swept.get("status")
    if status not in VALID_SWEPT_VOLUME_STATUSES:
        result.failures.append(f"snapshot {path} swept-volume invalid status: {status!r}")
    else:
        result.swept_volume_statuses[status] = result.swept_volume_statuses.get(status, 0) + 1
    if not swept.get("has_valid_query"):
        result.failures.append(f"snapshot {path} swept-volume has_valid_query is not true")
    for key in ("source_provider", "map_frame_id", "reason"):
        if not isinstance(swept.get(key), str) or not swept.get(key):
            result.failures.append(f"snapshot {path} swept-volume missing {key}")
    for key in ("start_local", "end_local"):
        if not is_vec3(swept.get(key)):
            result.failures.append(f"snapshot {path} swept-volume missing vec3 {key}")
    for key in ("radius_m", "horizon_s", "nominal_speed_mps", "min_clearance_m", "time_to_collision_s"):
        if not is_number(swept.get(key)):
            result.failures.append(f"snapshot {path} swept-volume missing numeric {key}")
    blocking = swept.get("blocking_cell_centers")
    if not isinstance(blocking, list):
        result.failures.append(f"snapshot {path} swept-volume missing blocking_cell_centers list")
    else:
        for index, center in enumerate(blocking[:32]):
            if not is_vec3(center):
                result.failures.append(f"snapshot {path} swept-volume blocking_cell_centers[{index}] is not vec3")
            else:
                result.swept_volume_blocking_cells_checked += 1
    result.swept_volume_snapshots_checked += 1


def validate_projected_cell(cell: Any, result: ValidationResult, *, path: Path, index: int) -> None:
    prefix = f"sidecar occupancy projected_cells[{index}] in {path}"
    if not isinstance(cell, dict):
        result.failures.append(f"{prefix} is not an object")
        return
    if cell.get("state") not in VALID_OCCUPANCY_CELL_STATES:
        result.failures.append(f"{prefix} has invalid state: {cell.get('state')!r}")
    for key in ("center_local", "size_m"):
        if not is_vec3(cell.get(key)):
            result.failures.append(f"{prefix} missing vec3 {key}")
    for key in ("confidence", "u_px", "v_px", "depth_m", "range_m"):
        if not is_number(cell.get(key)):
            result.failures.append(f"{prefix} missing numeric {key}")
    if not isinstance(cell.get("visible"), bool):
        result.failures.append(f"{prefix} missing bool visible")
    if not isinstance(cell.get("reason"), str):
        result.failures.append(f"{prefix} missing string reason")
    result.occupancy_projected_cells_checked += 1


def validate_occupancy_sidecar(sidecar: dict[str, Any], result: ValidationResult, *, path: Path) -> None:
    occupancy = sidecar.get("occupancy")
    if not isinstance(occupancy, dict):
        result.failures.append(f"sidecar {path} missing occupancy object")
        return
    if occupancy.get("present") is not True:
        result.failures.append(f"sidecar {path} occupancy.present is not true")
    summary = occupancy.get("summary")
    if not isinstance(summary, dict):
        result.failures.append(f"sidecar {path} occupancy missing summary object")
    projected_cells = occupancy.get("projected_cells")
    if not isinstance(projected_cells, list) or not projected_cells:
        result.failures.append(f"sidecar {path} occupancy missing non-empty projected_cells")
    else:
        for index, cell in enumerate(projected_cells[:32]):
            validate_projected_cell(cell, result, path=path, index=index)
    result.occupancy_sidecars_checked += 1


def validate_swept_volume_sidecar(sidecar: dict[str, Any], result: ValidationResult, *, path: Path) -> None:
    swept = sidecar.get("swept_volume")
    if not isinstance(swept, dict):
        result.failures.append(f"sidecar {path} missing swept_volume object")
        return
    if swept.get("present") is not True:
        result.failures.append(f"sidecar {path} swept_volume.present is not true")
    if swept.get("status") not in VALID_SWEPT_VOLUME_STATUSES:
        result.failures.append(f"sidecar {path} swept_volume invalid status: {swept.get('status')!r}")
    for key in ("start", "end"):
        projected = swept.get(f"projected_{key}")
        if not isinstance(projected, dict):
            result.failures.append(f"sidecar {path} swept_volume missing projected_{key}")
            continue
        for nkey in ("u_px", "v_px", "depth_m", "range_m"):
            if not is_number(projected.get(nkey)):
                result.failures.append(f"sidecar {path} swept_volume projected_{key} missing numeric {nkey}")
        if not isinstance(projected.get("visible"), bool):
            result.failures.append(f"sidecar {path} swept_volume projected_{key} missing bool visible")
    result.swept_volume_sidecars_checked += 1


def validate_occupancy_artifacts(run_dir: Path, result: ValidationResult, *, expect_sidecars: bool) -> None:
    snapshots = snapshot_paths(run_dir)
    if not snapshots:
        result.failures.append("expected occupancy snapshots but no snapshot artifacts were found")
        return
    readable = False
    for path in snapshots[: min(len(snapshots), 10)]:
        data = read_json_object(path, result, "snapshot")
        if data is None:
            continue
        readable = True
        validate_snapshot_occupancy(data, result, path=path)
    if not readable:
        result.failures.append("expected occupancy snapshots but no readable snapshot JSON was found")
    sidecars = sorted(run_dir.rglob("frame_*.world_overlay.json"))
    if not sidecars:
        msg = "no world overlay sidecar JSON found for occupancy projection validation"
        (result.failures if expect_sidecars else result.warnings).append(msg)
        return
    for path in sidecars[: min(len(sidecars), 10)]:
        data = read_json_object(path, result, "world overlay sidecar")
        if data is not None:
            validate_occupancy_sidecar(data, result, path=path)


def validate_swept_volume_artifacts(run_dir: Path, result: ValidationResult, *, expect_sidecars: bool) -> None:
    snapshots = snapshot_paths(run_dir)
    if not snapshots:
        result.failures.append("expected swept-volume snapshots but no snapshot artifacts were found")
        return
    readable = False
    for path in snapshots[: min(len(snapshots), 10)]:
        data = read_json_object(path, result, "snapshot")
        if data is None:
            continue
        readable = True
        validate_swept_volume(data, result, path=path)
    if not readable:
        result.failures.append("expected swept-volume snapshots but no readable snapshot JSON was found")
    sidecars = sorted(run_dir.rglob("frame_*.world_overlay.json"))
    if not sidecars:
        msg = "no world overlay sidecar JSON found for swept-volume projection validation"
        (result.failures if expect_sidecars else result.warnings).append(msg)
        return
    for path in sidecars[: min(len(sidecars), 10)]:
        data = read_json_object(path, result, "world overlay sidecar")
        if data is not None:
            validate_swept_volume_sidecar(data, result, path=path)


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
        elif event_name == "command_result" and not bool(event.get("success")):
            result.failures.append(f"{event_tick(event)}: command {event.get('command', 'unknown')} failed: {event.get('status', '')}")
        elif event_name == "command_exception":
            result.failures.append(f"{event_tick(event)}: command {event.get('command', 'unknown')} exception: {event.get('error', '')}")
        if event_name == "command_dispatch" and event.get("command") == "Velocity":
            result.velocity_commands += 1
        if isinstance(event_name, str) and event_name in OBJECT_BEHAVIOR_EVENTS:
            result.behavior_events[event_name] = result.behavior_events.get(event_name, 0) + 1
        if isinstance(event_name, str) and event_name in SEQUENCE_EVENTS:
            result.sequence_events[event_name] = result.sequence_events.get(event_name, 0) + 1
            step = str(event.get("step_behavior") or "unknown")
            if event_name == "behavior_sequence_step_start":
                result.sequence_started_steps.append(step)
                result.sequence_step_modes.append((step, str(event.get("step_yaw_mode") or "unknown"), str(event.get("step_camera_pointing_mode") or "unknown")))
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
            result.failures.append(f"missing required mission state in order: {expected}; observed path: {' -> '.join(result.state_path)}")
            return
        search_from = index + 1


def validate_height_gates(events: list[dict[str, Any]], result: ValidationResult, *, safe_height_m: float, landed_height_m: float, require_landed_height: bool) -> None:
    for event in events:
        if event.get("event") != "state_transition":
            continue
        height = event.get("ego_height_m")
        if event.get("to") == "ExecuteMission" and isinstance(height, (int, float)):
            result.safe_height_gate_height_m = float(height)
        elif event.get("to") == "Complete" and isinstance(height, (int, float)):
            result.landed_gate_height_m = float(height)
    if result.safe_height_gate_height_m is None:
        result.failures.append("missing ExecuteMission transition height gate evidence")
    elif result.safe_height_gate_height_m < safe_height_m:
        result.failures.append(f"ExecuteMission reached below safe height: height={result.safe_height_gate_height_m:.3f}m required>={safe_height_m:.3f}m")
    if require_landed_height:
        if result.landed_gate_height_m is None:
            result.failures.append("missing Complete transition landed-height evidence")
        elif result.landed_gate_height_m > landed_height_m:
            result.failures.append(f"Complete reached above landed height: height={result.landed_gate_height_m:.3f}m required<={landed_height_m:.3f}m")


def validate_expected_final_state(events: list[dict[str, Any]], result: ValidationResult, *, expected_final_state: str, safe_height_m: float, landed_height_m: float) -> None:
    if result.final_state != expected_final_state:
        result.failures.append(f"final_state is {result.final_state}; expected {expected_final_state}")
    if expected_final_state == "Complete":
        validate_state_order(result, COMPLETE_STATE_ORDER)
        if "Abort" in result.state_path:
            result.failures.append(f"Abort appeared in successful Complete lifecycle: {' -> '.join(result.state_path)}")
        validate_height_gates(events, result, safe_height_m=safe_height_m, landed_height_m=landed_height_m, require_landed_height=True)
    elif expected_final_state == "Abort":
        validate_state_order(result, ABORT_AFTER_FLIGHT_STATE_ORDER)
        if "Complete" in result.state_path:
            result.failures.append(f"Complete appeared in expected Abort lifecycle: {' -> '.join(result.state_path)}")
        validate_height_gates(events, result, safe_height_m=safe_height_m, landed_height_m=landed_height_m, require_landed_height=False)
    else:
        result.failures.append(f"unsupported expected final state: {expected_final_state}")


def validate_behavior_expectations(result: ValidationResult) -> None:
    for name in ("target_selected", "behavior_start", "behavior_complete"):
        if result.behavior_events.get(name, 0) == 0:
            result.failures.append(f"expected object behavior event {name}")
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
    if expected_steps and result.sequence_started_steps[: len(expected_steps)] != expected_steps:
        result.failures.append(f"sequence step start order mismatch: observed={result.sequence_started_steps[:len(expected_steps)]} expected={expected_steps}")


def validate_sequence_step_mode_expectations(result: ValidationResult, expected_modes: list[tuple[str, str, str]]) -> None:
    if not expected_modes:
        return
    for expected in expected_modes:
        if expected not in result.sequence_step_modes:
            result.failures.append(f"expected sequence step mode {expected[0]}:{expected[1]}:{expected[2]}; observed={result.sequence_step_modes}")
    if result.sequence_step_modes[: len(expected_modes)] != expected_modes:
        result.failures.append(f"sequence step mode order mismatch: observed={result.sequence_step_modes[:len(expected_modes)]} expected={expected_modes}")


def validate_camera_pointing_expectations(result: ValidationResult, *, expected_modes: list[str], camera_frames_dir: Path | None, expect_camera_proof_frames: bool) -> None:
    for name in ("camera_pointing_intent", "camera_pointing_dispatch", "camera_pointing_result"):
        if result.camera_pointing_events.get(name, 0) == 0:
            result.failures.append(f"expected {name} events")
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


def validate_run_dir(run_dir: Path, *, expected_final_state: str | None, expect_behavior: bool, expect_sequence: bool, expect_sequence_steps: list[str], expect_sequence_step_modes: list[tuple[str, str, str]], expect_camera_pointing: bool, expect_camera_modes: list[str], camera_frames_dir: Path | None, expect_camera_proof_frames: bool, expect_occupancy: bool, expect_occupancy_sidecars: bool, expect_swept_volume: bool, expect_swept_volume_sidecars: bool, safe_height_m: float, landed_height_m: float, allow_missing_snapshots: bool) -> ValidationResult:
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
        validate_expected_final_state(events, result, expected_final_state=expected_final_state, safe_height_m=safe_height_m, landed_height_m=landed_height_m)
    if expect_behavior:
        validate_behavior_expectations(result)
    if expect_sequence:
        validate_sequence_expectations(result, expect_sequence_steps)
        validate_sequence_step_mode_expectations(result, expect_sequence_step_modes)
    if expect_camera_pointing:
        validate_camera_pointing_expectations(result, expected_modes=expect_camera_modes, camera_frames_dir=camera_frames_dir, expect_camera_proof_frames=expect_camera_proof_frames)
    if expect_occupancy:
        validate_occupancy_artifacts(run_dir, result, expect_sidecars=expect_occupancy_sidecars)
    if expect_swept_volume:
        validate_swept_volume_artifacts(run_dir, result, expect_sidecars=expect_swept_volume_sidecars)
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
        if result.sequence_step_modes:
            print("  sequence_step_modes:")
            for step, yaw, camera in result.sequence_step_modes:
                print(f"    {step}: yaw={yaw} camera={camera}")
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
    if result.occupancy_snapshots_checked or result.occupancy_sidecars_checked:
        print("  occupancy_artifacts:")
        print(f"    snapshots_checked: {result.occupancy_snapshots_checked}")
        print(f"    debug_cells_checked: {result.occupancy_debug_cells_checked}")
        print(f"    sidecars_checked: {result.occupancy_sidecars_checked}")
        print(f"    projected_cells_checked: {result.occupancy_projected_cells_checked}")
        for source_kind in sorted(result.occupancy_source_kinds):
            print(f"    source_kind {source_kind}: {result.occupancy_source_kinds[source_kind]}")
    if result.swept_volume_snapshots_checked or result.swept_volume_sidecars_checked:
        print("  swept_volume_artifacts:")
        print(f"    snapshots_checked: {result.swept_volume_snapshots_checked}")
        print(f"    sidecars_checked: {result.swept_volume_sidecars_checked}")
        print(f"    blocking_cells_checked: {result.swept_volume_blocking_cells_checked}")
        for status in sorted(result.swept_volume_statuses):
            print(f"    status {status}: {result.swept_volume_statuses[status]}")
    print(f"  failures: {len(result.failures)}")
    for failure in result.failures:
        print(f"    - {failure}")
    for warning in result.warnings:
        print(f"  warning: {warning}")


def parse_csv(value: str) -> list[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


def parse_step_modes(value: str) -> list[tuple[str, str, str]]:
    out: list[tuple[str, str, str]] = []
    for item in parse_csv(value):
        parts = [part.strip() for part in item.split(":")]
        if len(parts) != 3 or not all(parts):
            raise ValueError("--expect-sequence-step-modes entries must be step:yaw_mode:camera_pointing_mode")
        out.append((parts[0], parts[1], parts[2]))
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--expect-complete", action="store_true")
    parser.add_argument("--expect-final-state", choices=["Complete", "Abort"])
    parser.add_argument("--expect-behavior", action="store_true")
    parser.add_argument("--expect-sequence", action="store_true")
    parser.add_argument("--expect-sequence-steps", default="")
    parser.add_argument("--expect-sequence-step-modes", default="")
    parser.add_argument("--expect-camera-pointing", action="store_true")
    parser.add_argument("--expect-camera-modes", default="")
    parser.add_argument("--camera-frames-dir", type=Path, default=None)
    parser.add_argument("--expect-camera-proof-frames", action="store_true")
    parser.add_argument("--expect-occupancy", action="store_true")
    parser.add_argument("--expect-occupancy-sidecars", action="store_true")
    parser.add_argument("--expect-swept-volume", action="store_true")
    parser.add_argument("--expect-swept-volume-sidecars", action="store_true")
    parser.add_argument("--safe-height-m", type=float, default=1.0)
    parser.add_argument("--landed-height-m", type=float, default=1.0)
    parser.add_argument("--allow-missing-snapshots", action="store_true")
    args = parser.parse_args()

    expected_final_state = args.expect_final_state
    if args.expect_complete:
        if expected_final_state is not None and expected_final_state != "Complete":
            print("--expect-complete conflicts with --expect-final-state", file=sys.stderr)
            return 2
        expected_final_state = "Complete"
    if args.expect_occupancy_sidecars and not args.expect_occupancy:
        print("--expect-occupancy-sidecars requires --expect-occupancy", file=sys.stderr)
        return 2
    if args.expect_swept_volume_sidecars and not args.expect_swept_volume:
        print("--expect-swept-volume-sidecars requires --expect-swept-volume", file=sys.stderr)
        return 2

    result = validate_run_dir(
        args.run_dir,
        expected_final_state=expected_final_state,
        expect_behavior=args.expect_behavior,
        expect_sequence=args.expect_sequence,
        expect_sequence_steps=parse_csv(args.expect_sequence_steps),
        expect_sequence_step_modes=parse_step_modes(args.expect_sequence_step_modes),
        expect_camera_pointing=args.expect_camera_pointing,
        expect_camera_modes=parse_csv(args.expect_camera_modes),
        camera_frames_dir=args.camera_frames_dir,
        expect_camera_proof_frames=args.expect_camera_proof_frames,
        expect_occupancy=args.expect_occupancy,
        expect_occupancy_sidecars=args.expect_occupancy_sidecars,
        expect_swept_volume=args.expect_swept_volume,
        expect_swept_volume_sidecars=args.expect_swept_volume_sidecars,
        safe_height_m=args.safe_height_m,
        landed_height_m=args.landed_height_m,
        allow_missing_snapshots=args.allow_missing_snapshots,
    )
    print_result(result)
    return 0 if result.valid else 1


if __name__ == "__main__":
    raise SystemExit(main())
