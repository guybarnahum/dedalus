#!/usr/bin/env python3
"""Validate Dedalus mission artifacts and optional world-overlay sidecars."""

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
VALID_OCCUPANCY_SOURCE_KINDS = {"synthetic_fixture", "airsim_ground_truth", "visual_obstacle_detector", "depth_provider", "fused"}
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
    occupancy_max_occupied_count: int = 0
    occupancy_source_object_prefix_counts: dict[str, int] = field(default_factory=dict)
    swept_volume_snapshots_checked: int = 0
    swept_volume_sidecars_checked: int = 0
    swept_volume_statuses: dict[str, int] = field(default_factory=dict)
    swept_volume_blocking_cells_checked: int = 0
    scene_inventory_path: str | None = None
    scene_inventory_scene_id: str | None = None
    scene_inventory_object_count: int = 0
    scene_inventory_class_counts: dict[str, int] = field(default_factory=dict)

    @property
    def valid(self) -> bool:
        return not self.failures


def is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def is_vec3(value: Any) -> bool:
    return isinstance(value, list) and len(value) == 3 and all(is_number(v) for v in value)


def read_json(path: Path, result: ValidationResult, label: str) -> dict[str, Any] | None:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        result.failures.append(f"failed to read {label} {path}: {exc}")
        return None
    if not isinstance(data, dict):
        result.failures.append(f"{label} {path} is not a JSON object")
        return None
    return data


def snapshot_path_from_manifest_line(line: str) -> Path | None:
    stripped = line.strip()
    if not stripped or stripped.startswith("#"):
        return None
    parts = stripped.split()
    return Path(parts[1] if len(parts) >= 2 and parts[0].isdigit() else parts[0])


def snapshot_paths(run_dir: Path) -> list[Path]:
    manifest = run_dir / "snapshot_manifest.txt"
    if manifest.exists():
        paths: list[Path] = []
        for raw in manifest.read_text(encoding="utf-8").splitlines():
            path = snapshot_path_from_manifest_line(raw)
            if path is not None:
                paths.append(path if path.is_absolute() else run_dir / path)
        return paths
    return sorted(run_dir.glob("snapshot_*.json"))


def sidecar_paths(run_dir: Path) -> list[Path]:
    return sorted(run_dir.rglob("frame_*.world_overlay.json"))


def read_events(path: Path, result: ValidationResult) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        result.failures.append(f"failed to open mission_events.jsonl: {exc}")
        return events
    for line_number, raw in enumerate(lines, start=1):
        if not raw.strip():
            continue
        try:
            event = json.loads(raw)
        except json.JSONDecodeError as exc:
            result.failures.append(f"line {line_number}: invalid JSON: {exc}")
            continue
        if not isinstance(event, dict):
            result.failures.append(f"line {line_number}: event record is not an object")
            continue
        result.event_count += 1
        events.append(event)
    return events


def append_state(states: list[str], state: Any) -> None:
    if isinstance(state, str) and state and (not states or states[-1] != state):
        states.append(state)


def collect_timeline(events: list[dict[str, Any]], result: ValidationResult) -> None:
    for event in events:
        name = event.get("event")
        if name == "state_transition":
            append_state(result.state_path, event.get("from"))
            append_state(result.state_path, event.get("to"))
            if isinstance(event.get("to"), str):
                result.final_state = str(event["to"])
            if event.get("to") == "Abort" and is_number(event.get("ego_height_m")):
                result.abort_height_m = float(event["ego_height_m"])
        elif name == "runtime_stop":
            append_state(result.state_path, event.get("state"))
            if isinstance(event.get("state"), str):
                result.final_state = str(event["state"])
        elif name == "command_result" and not bool(event.get("success")):
            result.failures.append(f"tick {event.get('tick', '?')}: command {event.get('command', 'unknown')} failed: {event.get('status', '')}")
        elif name == "command_exception":
            result.failures.append(f"tick {event.get('tick', '?')}: command {event.get('command', 'unknown')} exception: {event.get('error', '')}")
        if name == "command_dispatch" and event.get("command") == "Velocity":
            result.velocity_commands += 1
        if isinstance(name, str) and name in OBJECT_BEHAVIOR_EVENTS:
            result.behavior_events[name] = result.behavior_events.get(name, 0) + 1
        if isinstance(name, str) and name in SEQUENCE_EVENTS:
            result.sequence_events[name] = result.sequence_events.get(name, 0) + 1
            step = str(event.get("step_behavior") or "unknown")
            if name == "behavior_sequence_step_start":
                result.sequence_started_steps.append(step)
                result.sequence_step_modes.append((step, str(event.get("step_yaw_mode") or "unknown"), str(event.get("step_camera_pointing_mode") or "unknown")))
            elif name == "behavior_sequence_step_complete":
                result.sequence_completed_steps.append(step)
        if isinstance(name, str) and name in CAMERA_POINTING_EVENTS:
            result.camera_pointing_events[name] = result.camera_pointing_events.get(name, 0) + 1
        if name == "camera_pointing_intent":
            mode = str(event.get("camera_pointing_mode") or event.get("mode") or "unknown")
            result.camera_pointing_modes[mode] = result.camera_pointing_modes.get(mode, 0) + 1


def validate_order(result: ValidationResult, expected_order: list[str]) -> None:
    if not result.state_path:
        result.failures.append("no mission state transitions found")
        return
    search_from = 0
    for state in expected_order:
        try:
            index = result.state_path.index(state, search_from)
        except ValueError:
            result.failures.append(f"missing required mission state in order: {state}; observed path: {' -> '.join(result.state_path)}")
            return
        search_from = index + 1


def validate_heights(events: list[dict[str, Any]], result: ValidationResult, safe_height_m: float, landed_height_m: float, require_landed: bool) -> None:
    for event in events:
        if event.get("event") != "state_transition":
            continue
        height = event.get("ego_height_m")
        if event.get("to") == "ExecuteMission" and is_number(height):
            result.safe_height_gate_height_m = float(height)
        elif event.get("to") == "Complete" and is_number(height):
            result.landed_gate_height_m = float(height)
    if result.safe_height_gate_height_m is None:
        result.failures.append("missing ExecuteMission transition height gate evidence")
    elif result.safe_height_gate_height_m < safe_height_m:
        result.failures.append(f"ExecuteMission reached below safe height: height={result.safe_height_gate_height_m:.3f}m required>={safe_height_m:.3f}m")
    if require_landed:
        if result.landed_gate_height_m is None:
            result.failures.append("missing Complete transition landed-height evidence")
        elif result.landed_gate_height_m > landed_height_m:
            result.failures.append(f"Complete reached above landed height: height={result.landed_gate_height_m:.3f}m required<={landed_height_m:.3f}m")


def validate_lifecycle(events: list[dict[str, Any]], result: ValidationResult, final_state: str, safe_height_m: float, landed_height_m: float) -> None:
    if result.final_state != final_state:
        result.failures.append(f"final_state is {result.final_state}; expected {final_state}")
    if final_state == "Complete":
        validate_order(result, COMPLETE_STATE_ORDER)
        if "Abort" in result.state_path:
            result.failures.append(f"Abort appeared in successful Complete lifecycle: {' -> '.join(result.state_path)}")
        validate_heights(events, result, safe_height_m, landed_height_m, True)
    elif final_state == "Abort":
        validate_order(result, ABORT_AFTER_FLIGHT_STATE_ORDER)
        validate_heights(events, result, safe_height_m, landed_height_m, False)
    else:
        result.failures.append(f"unsupported expected final state: {final_state}")


def validate_behavior(result: ValidationResult) -> None:
    for name in ("target_selected", "behavior_start", "behavior_complete"):
        if result.behavior_events.get(name, 0) == 0:
            result.failures.append(f"expected object behavior event {name}")
    if result.velocity_commands == 0:
        result.failures.append("expected velocity commands during behavior run")


def validate_sequence(result: ValidationResult, expected_steps: list[str], expected_modes: list[tuple[str, str, str]]) -> None:
    if result.sequence_events.get("behavior_sequence_step_start", 0) == 0:
        result.failures.append("expected behavior_sequence_step_start events")
    if result.sequence_events.get("behavior_sequence_step_complete", 0) == 0:
        result.failures.append("expected behavior_sequence_step_complete events")
    for step in expected_steps:
        if step not in result.sequence_started_steps:
            result.failures.append(f"expected sequence step start for {step}")
    if expected_steps and result.sequence_started_steps[: len(expected_steps)] != expected_steps:
        result.failures.append(f"sequence step start order mismatch: observed={result.sequence_started_steps[:len(expected_steps)]} expected={expected_steps}")
    for expected in expected_modes:
        if expected not in result.sequence_step_modes:
            result.failures.append(f"expected sequence step mode {expected[0]}:{expected[1]}:{expected[2]}; observed={result.sequence_step_modes}")


def camera_frame_counts(path: Path) -> Counter[str]:
    counts: Counter[str] = Counter()
    if not path.exists():
        return counts
    for file in path.glob("camera_pointing_*.png"):
        parts = file.stem.split("_")
        camera = "_".join(parts[3:-1]) if len(parts) >= 5 else "legacy_or_unknown"
        counts[camera or "unknown"] += 1
    return counts


def validate_camera(result: ValidationResult, expected_modes: list[str], camera_frames_dir: Path | None, expect_frames: bool) -> None:
    for name in CAMERA_POINTING_EVENTS:
        if result.camera_pointing_events.get(name, 0) == 0:
            result.failures.append(f"expected {name} events")
    for mode in expected_modes:
        if result.camera_pointing_modes.get(mode, 0) == 0:
            result.failures.append(f"expected camera_pointing_mode {mode}")
    if camera_frames_dir is not None:
        result.camera_proof_frames = dict(camera_frame_counts(camera_frames_dir))
    if expect_frames and not result.camera_proof_frames:
        result.failures.append(f"expected camera proof frames in {camera_frames_dir}")


def record_source_object_prefix(result: ValidationResult, source_object_name: Any, expected_prefixes: list[str]) -> None:
    if not isinstance(source_object_name, str):
        return
    for prefix in expected_prefixes:
        if source_object_name.startswith(prefix):
            result.occupancy_source_object_prefix_counts[prefix] = result.occupancy_source_object_prefix_counts.get(prefix, 0) + 1


def validate_snapshot_occupancy(snapshot: dict[str, Any], result: ValidationResult, path: Path, *, expected_prefixes: list[str]) -> None:
    occ = snapshot.get("ego_occupancy")
    if not isinstance(occ, dict):
        result.failures.append(f"snapshot {path} missing ego_occupancy object")
        return
    source = occ.get("source_kind")
    if source not in VALID_OCCUPANCY_SOURCE_KINDS:
        result.failures.append(f"snapshot {path} has invalid occupancy source_kind: {source!r}")
    else:
        result.occupancy_source_kinds[source] = result.occupancy_source_kinds.get(source, 0) + 1
    if occ.get("has_valid_occupancy") is not True:
        result.failures.append(f"snapshot {path} occupancy has_valid_occupancy is not true")
    for key in ("source_provider", "map_frame_id"):
        if not isinstance(occ.get(key), str) or not occ.get(key):
            result.failures.append(f"snapshot {path} occupancy missing {key}")
    for key in ("resolution_m", "nearest_obstacle_distance_m", "forward_corridor_clearance_m"):
        if not is_number(occ.get(key)):
            result.failures.append(f"snapshot {path} occupancy missing numeric {key}")
    if not is_vec3(occ.get("size_m")):
        result.failures.append(f"snapshot {path} occupancy missing vec3 size_m")
    for key in ("occupied_count", "free_count", "unknown_count", "stale_count"):
        if not isinstance(occ.get(key), int) or occ.get(key) < 0:
            result.failures.append(f"snapshot {path} occupancy missing nonnegative integer {key}")
    if isinstance(occ.get("occupied_count"), int):
        result.occupancy_max_occupied_count = max(result.occupancy_max_occupied_count, int(occ["occupied_count"]))
    cells = occ.get("debug_cells")
    if not isinstance(cells, list) or not cells:
        result.failures.append(f"snapshot {path} occupancy missing non-empty debug_cells")
    else:
        for index, cell in enumerate(cells):
            if not isinstance(cell, dict):
                result.failures.append(f"snapshot {path} occupancy debug_cells[{index}] is not an object")
                continue
            record_source_object_prefix(result, cell.get("source_object_name"), expected_prefixes)
            if index >= 32:
                continue
            if cell.get("state") not in VALID_OCCUPANCY_CELL_STATES:
                result.failures.append(f"snapshot {path} occupancy debug_cells[{index}] invalid state")
            if not is_vec3(cell.get("center_local")) or not is_vec3(cell.get("size_m")):
                result.failures.append(f"snapshot {path} occupancy debug_cells[{index}] missing geometry")
            if not is_number(cell.get("confidence")) or not is_number(cell.get("distance_to_nearest_occupied_m")):
                result.failures.append(f"snapshot {path} occupancy debug_cells[{index}] missing numeric fields")
            result.occupancy_debug_cells_checked += 1
    result.occupancy_snapshots_checked += 1


def validate_snapshot_swept(snapshot: dict[str, Any], result: ValidationResult, path: Path) -> None:
    swept = snapshot.get("latest_swept_volume")
    if not isinstance(swept, dict):
        result.failures.append(f"snapshot {path} missing latest_swept_volume object")
        return
    status = swept.get("status")
    if status not in VALID_SWEPT_VOLUME_STATUSES:
        result.failures.append(f"snapshot {path} swept-volume invalid status: {status!r}")
    else:
        result.swept_volume_statuses[status] = result.swept_volume_statuses.get(status, 0) + 1
    if swept.get("has_valid_query") is not True:
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
    blockers = swept.get("blocking_cell_centers")
    if not isinstance(blockers, list):
        result.failures.append(f"snapshot {path} swept-volume missing blocking_cell_centers list")
    else:
        for index, center in enumerate(blockers[:32]):
            if not is_vec3(center):
                result.failures.append(f"snapshot {path} swept-volume blocking_cell_centers[{index}] is not vec3")
            else:
                result.swept_volume_blocking_cells_checked += 1
    result.swept_volume_snapshots_checked += 1


def validate_projected_point(point: Any, result: ValidationResult, path: Path, label: str) -> None:
    if not isinstance(point, dict):
        result.failures.append(f"sidecar {path} missing {label}")
        return
    for key in ("u_px", "v_px", "depth_m", "range_m"):
        if not is_number(point.get(key)):
            result.failures.append(f"sidecar {path} {label} missing numeric {key}")
    if not isinstance(point.get("visible"), bool):
        result.failures.append(f"sidecar {path} {label} missing bool visible")
    if not isinstance(point.get("reason"), str):
        result.failures.append(f"sidecar {path} {label} missing string reason")


def validate_sidecar_occupancy(sidecar: dict[str, Any], result: ValidationResult, path: Path) -> None:
    occ = sidecar.get("occupancy")
    if not isinstance(occ, dict):
        result.failures.append(f"sidecar {path} missing occupancy object")
        return
    if occ.get("present") is not True:
        result.failures.append(f"sidecar {path} occupancy.present is not true")
    if not isinstance(occ.get("summary"), dict):
        result.failures.append(f"sidecar {path} occupancy missing summary object")
    cells = occ.get("projected_cells")
    if not isinstance(cells, list) or not cells:
        result.failures.append(f"sidecar {path} occupancy missing non-empty projected_cells")
    else:
        for index, cell in enumerate(cells[:32]):
            if not isinstance(cell, dict):
                result.failures.append(f"sidecar {path} occupancy projected_cells[{index}] is not an object")
                continue
            if cell.get("state") not in VALID_OCCUPANCY_CELL_STATES:
                result.failures.append(f"sidecar {path} occupancy projected_cells[{index}] invalid state")
            if not is_vec3(cell.get("center_local")) or not is_vec3(cell.get("size_m")):
                result.failures.append(f"sidecar {path} occupancy projected_cells[{index}] missing geometry")
            for key in ("confidence", "u_px", "v_px", "depth_m", "range_m"):
                if not is_number(cell.get(key)):
                    result.failures.append(f"sidecar {path} occupancy projected_cells[{index}] missing numeric {key}")
            if not isinstance(cell.get("visible"), bool) or not isinstance(cell.get("reason"), str):
                result.failures.append(f"sidecar {path} occupancy projected_cells[{index}] missing projection metadata")
            result.occupancy_projected_cells_checked += 1
    result.occupancy_sidecars_checked += 1


def validate_sidecar_swept(sidecar: dict[str, Any], result: ValidationResult, path: Path) -> None:
    swept = sidecar.get("swept_volume")
    if not isinstance(swept, dict):
        result.failures.append(f"sidecar {path} missing swept_volume object")
        return
    if swept.get("present") is not True:
        result.failures.append(f"sidecar {path} swept_volume.present is not true")
    status = swept.get("status")
    if status not in VALID_SWEPT_VOLUME_STATUSES:
        result.failures.append(f"sidecar {path} swept_volume invalid status: {status!r}")
    else:
        result.swept_volume_statuses[status] = result.swept_volume_statuses.get(status, 0) + 1
    for key in ("source_provider", "map_frame_id", "reason"):
        if not isinstance(swept.get(key), str) or not swept.get(key):
            result.failures.append(f"sidecar {path} swept_volume missing {key}")
    for key in ("start_local", "end_local"):
        if not is_vec3(swept.get(key)):
            result.failures.append(f"sidecar {path} swept_volume missing vec3 {key}")
    for key in ("radius_m", "horizon_s", "nominal_speed_mps", "min_clearance_m", "time_to_collision_s"):
        if not is_number(swept.get(key)):
            result.failures.append(f"sidecar {path} swept_volume missing numeric {key}")
    if not isinstance(swept.get("has_valid_query"), bool):
        result.failures.append(f"sidecar {path} swept_volume missing bool has_valid_query")
    validate_projected_point(swept.get("projected_start"), result, path, "swept_volume projected_start")
    validate_projected_point(swept.get("projected_end"), result, path, "swept_volume projected_end")
    blockers = swept.get("projected_blocking_cells")
    if not isinstance(blockers, list):
        result.failures.append(f"sidecar {path} swept_volume missing projected_blocking_cells list")
    else:
        for index, blocker in enumerate(blockers[:32]):
            if not isinstance(blocker, dict) or not is_vec3(blocker.get("center_local")):
                result.failures.append(f"sidecar {path} swept_volume projected_blocking_cells[{index}] missing center_local")
            else:
                validate_projected_point(blocker.get("projection"), result, path, f"swept_volume projected_blocking_cells[{index}].projection")
                result.swept_volume_blocking_cells_checked += 1
    result.swept_volume_sidecars_checked += 1


def validate_scene_inventory(path: Path, result: ValidationResult) -> None:
    result.scene_inventory_path = str(path)
    data = read_json(path, result, "scene inventory")
    if data is None:
        return
    if data.get("schema_version") != 1:
        result.failures.append(f"scene inventory {path} schema_version is not 1")
    scene_id = data.get("scene_id")
    if not isinstance(scene_id, str) or not scene_id:
        result.failures.append(f"scene inventory {path} missing scene_id")
    else:
        result.scene_inventory_scene_id = scene_id
    source = data.get("source")
    if not isinstance(source, dict) or source.get("kind") != "airsim_scene_inventory":
        result.failures.append(f"scene inventory {path} missing source.kind=airsim_scene_inventory")
    class_counts = data.get("class_counts")
    if not isinstance(class_counts, dict):
        result.failures.append(f"scene inventory {path} missing class_counts object")
    else:
        result.scene_inventory_class_counts = {str(k): int(v) for k, v in class_counts.items() if isinstance(v, int)}
    objects = data.get("objects")
    if not isinstance(objects, list) or not objects:
        result.failures.append(f"scene inventory {path} missing non-empty objects list")
        return
    result.scene_inventory_object_count = len(objects)
    for index, obj in enumerate(objects[:32]):
        if not isinstance(obj, dict):
            result.failures.append(f"scene inventory {path} objects[{index}] is not an object")
            continue
        for key in ("name", "canonical_class", "geometry_class"):
            if not isinstance(obj.get(key), str) or not obj.get(key):
                result.failures.append(f"scene inventory {path} objects[{index}] missing {key}")
        if not isinstance(obj.get("pose_available"), bool):
            result.failures.append(f"scene inventory {path} objects[{index}] missing bool pose_available")
        if obj.get("pose_available") is True and not is_vec3(obj.get("position_ned_m")):
            result.failures.append(f"scene inventory {path} objects[{index}] missing vec3 position_ned_m")
        if not is_vec3(obj.get("recommended_size_m")):
            result.failures.append(f"scene inventory {path} objects[{index}] missing vec3 recommended_size_m")


def validate_snapshots(run_dir: Path, result: ValidationResult, *, occupancy: bool, swept: bool, allow_missing: bool, sidecars_expected: bool, expected_prefixes: list[str]) -> None:
    paths = snapshot_paths(run_dir)
    if not paths:
        if not allow_missing or not sidecars_expected:
            result.failures.append("expected snapshot artifacts but none were found")
        return
    readable = False
    for path in paths[: min(len(paths), 10)]:
        data = read_json(path, result, "snapshot")
        if data is None:
            continue
        readable = True
        if occupancy:
            validate_snapshot_occupancy(data, result, path, expected_prefixes=expected_prefixes)
        if swept:
            validate_snapshot_swept(data, result, path)
    if not readable:
        result.failures.append("expected snapshot artifacts but no readable snapshot JSON was found")


def validate_sidecars(run_dir: Path, result: ValidationResult, *, occupancy: bool, swept: bool, required: bool) -> None:
    paths = sidecar_paths(run_dir)
    if not paths:
        message = "no world overlay sidecar JSON found for validation"
        (result.failures if required else result.warnings).append(message)
        return
    for path in paths[: min(len(paths), 10)]:
        data = read_json(path, result, "world overlay sidecar")
        if data is None:
            continue
        if occupancy:
            validate_sidecar_occupancy(data, result, path)
        if swept:
            validate_sidecar_swept(data, result, path)


def validate_occupancy_evidence_expectations(result: ValidationResult, *, expected_source: str | None, min_occupied_cells: int, expected_prefixes: list[str]) -> None:
    if expected_source is not None and result.occupancy_source_kinds.get(expected_source, 0) == 0:
        observed = ", ".join(sorted(result.occupancy_source_kinds)) or "none"
        result.failures.append(f"expected occupancy source_kind {expected_source}; observed {observed}")
    if min_occupied_cells > 0 and result.occupancy_max_occupied_count < min_occupied_cells:
        result.failures.append(f"expected at least {min_occupied_cells} occupied occupancy cells; max observed {result.occupancy_max_occupied_count}")
    for prefix in expected_prefixes:
        if result.occupancy_source_object_prefix_counts.get(prefix, 0) == 0:
            result.failures.append(f"expected occupancy source_object_name prefix {prefix}")


def validate_run_dir(
    run_dir: Path,
    *,
    expected_final_state: str | None,
    expect_behavior: bool,
    expect_sequence: bool,
    expect_sequence_steps: list[str],
    expect_sequence_step_modes: list[tuple[str, str, str]],
    expect_camera_pointing: bool,
    expect_camera_modes: list[str],
    camera_frames_dir: Path | None,
    expect_camera_proof_frames: bool,
    expect_occupancy: bool,
    expect_occupancy_sidecars: bool,
    expect_swept_volume: bool,
    expect_swept_volume_sidecars: bool,
    expected_occupancy_source: str | None,
    min_occupied_cells: int,
    expected_source_object_prefixes: list[str],
    scene_inventory_path: Path | None,
    safe_height_m: float,
    landed_height_m: float,
    allow_missing_snapshots: bool,
) -> ValidationResult:
    result = ValidationResult()
    if not run_dir.exists() or not run_dir.is_dir():
        result.failures.append(f"run artifact directory not found: {run_dir}")
        return result
    result.snapshot_count = len(snapshot_paths(run_dir))
    if result.snapshot_count == 0 and not allow_missing_snapshots:
        result.failures.append("no snapshot artifacts found; expected snapshot_manifest.txt or snapshot_*.json")

    if scene_inventory_path is not None:
        validate_scene_inventory(scene_inventory_path, result)

    events_path = run_dir / "mission_events.jsonl"
    needs_events = expected_final_state is not None or expect_behavior or expect_sequence or expect_camera_pointing
    events: list[dict[str, Any]] = []
    if events_path.exists():
        events = read_events(events_path, result)
        collect_timeline(events, result)
    elif needs_events:
        result.failures.append(f"mission_events.jsonl not found in: {run_dir}")

    if events_path.exists():
        if expected_final_state is not None:
            validate_lifecycle(events, result, expected_final_state, safe_height_m, landed_height_m)
        if expect_behavior:
            validate_behavior(result)
        if expect_sequence:
            validate_sequence(result, expect_sequence_steps, expect_sequence_step_modes)
        if expect_camera_pointing:
            validate_camera(result, expect_camera_modes, camera_frames_dir, expect_camera_proof_frames)

    snapshot_requested = expect_occupancy or expect_swept_volume
    sidecar_requested = expect_occupancy_sidecars or expect_swept_volume_sidecars
    if snapshot_requested:
        validate_snapshots(run_dir, result, occupancy=expect_occupancy, swept=expect_swept_volume, allow_missing=allow_missing_snapshots, sidecars_expected=sidecar_requested, expected_prefixes=expected_source_object_prefixes)
    if sidecar_requested:
        validate_sidecars(run_dir, result, occupancy=expect_occupancy_sidecars, swept=expect_swept_volume_sidecars, required=True)
    elif expect_occupancy or expect_swept_volume:
        validate_sidecars(run_dir, result, occupancy=expect_occupancy, swept=expect_swept_volume, required=False)

    if expect_occupancy or expected_occupancy_source is not None or min_occupied_cells > 0 or expected_source_object_prefixes:
        validate_occupancy_evidence_expectations(result, expected_source=expected_occupancy_source, min_occupied_cells=min_occupied_cells, expected_prefixes=expected_source_object_prefixes)
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
    if result.scene_inventory_path is not None:
        print("  scene_inventory:")
        print(f"    path: {result.scene_inventory_path}")
        if result.scene_inventory_scene_id is not None:
            print(f"    scene_id: {result.scene_inventory_scene_id}")
        print(f"    object_count: {result.scene_inventory_object_count}")
        for class_name in sorted(result.scene_inventory_class_counts):
            print(f"    class {class_name}: {result.scene_inventory_class_counts[class_name]}")
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
        print(f"    max_occupied_count: {result.occupancy_max_occupied_count}")
        for source in sorted(result.occupancy_source_kinds):
            print(f"    source_kind {source}: {result.occupancy_source_kinds[source]}")
        for prefix in sorted(result.occupancy_source_object_prefix_counts):
            print(f"    source_object_prefix {prefix}: {result.occupancy_source_object_prefix_counts[prefix]}")
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
    parser.add_argument("--expect-occupancy-source", choices=sorted(VALID_OCCUPANCY_SOURCE_KINDS), default=None)
    parser.add_argument("--expect-min-occupied-cells", type=int, default=0)
    parser.add_argument("--expect-source-object-prefix", action="append", default=[])
    parser.add_argument("--expect-scene-inventory", type=Path, default=None)
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
    if args.expect_min_occupied_cells < 0:
        print("--expect-min-occupied-cells must be >= 0", file=sys.stderr)
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
        expect_occupancy=args.expect_occupancy or args.expect_occupancy_sidecars or args.expect_occupancy_source is not None or args.expect_min_occupied_cells > 0 or bool(args.expect_source_object_prefix),
        expect_occupancy_sidecars=args.expect_occupancy_sidecars,
        expect_swept_volume=args.expect_swept_volume or args.expect_swept_volume_sidecars,
        expect_swept_volume_sidecars=args.expect_swept_volume_sidecars,
        expected_occupancy_source=args.expect_occupancy_source,
        min_occupied_cells=args.expect_min_occupied_cells,
        expected_source_object_prefixes=args.expect_source_object_prefix,
        scene_inventory_path=args.expect_scene_inventory,
        safe_height_m=args.safe_height_m,
        landed_height_m=args.landed_height_m,
        allow_missing_snapshots=args.allow_missing_snapshots,
    )
    print_result(result)
    return 0 if result.valid else 1


if __name__ == "__main__":
    raise SystemExit(main())
