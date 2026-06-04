#!/usr/bin/env python3
"""Render live Dedalus runtime events into the AirSim/Unreal viewport.

The overlay is a subscriber/renderer only. It does not evaluate ghost scenarios,
read snapshot manifests, decide between source modes, or compute occupancy or
swept-volume results. Producers publish typed JSONL events on the runtime event
stream; this script renders whatever arrives.

Consumed event types today:
  ghost_detections -> PLAN / PLAN* markers
  world_snapshot   -> AG, EGO, optional OCC, SWEEP, SENSE, and EVID markers
  mission_event    -> SEL marker state from target_selected events
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import time
from typing import Any

try:
    import airsim
except ImportError as exc:
    raise SystemExit("airsim Python package is required for viewport overlay") from exc


SELECTED_COLOR = [0.0, 1.0, 0.2, 1.0]
WORLD_AGENT_COLOR = [1.0, 0.85, 0.0, 1.0]
PLANNED_GHOST_COLOR = [0.2, 0.6, 1.0, 0.35]
STATIC_GHOST_COLOR = [0.4, 0.8, 1.0, 0.55]
STALE_COLOR = [0.5, 0.5, 0.5, 1.0]
ORIGIN_COLOR = [1.0, 1.0, 1.0, 1.0]
EGO_COLOR = [1.0, 0.0, 1.0, 1.0]
EGO_VELOCITY_COLOR = [1.0, 0.2, 1.0, 1.0]
OCCUPIED_CELL_COLOR = [1.0, 0.25, 0.05, 0.85]
FREE_CELL_COLOR = [0.1, 0.8, 1.0, 0.45]
UNKNOWN_CELL_COLOR = [1.0, 0.8, 0.1, 0.45]
OCCUPANCY_TEXT_COLOR = [1.0, 0.8, 0.1, 1.0]
SWEEP_CLEAR_COLOR = [0.2, 1.0, 0.35, 0.85]
SWEEP_BLOCKED_COLOR = [1.0, 0.05, 0.05, 0.95]
SWEEP_UNKNOWN_COLOR = [1.0, 0.8, 0.1, 0.85]
SWEEP_STALE_COLOR = [0.55, 0.55, 0.55, 0.8]

SENSING_VOLUME_COLOR = [0.45, 0.65, 1.0, 0.55]
EVIDENCE_OCCUPIED_COLOR = [1.0, 0.25, 0.05, 0.90]
EVIDENCE_THIN_RISK_COLOR = [1.0, 0.0, 1.0, 0.95]
EVIDENCE_BLOCKING_COLOR = [1.0, 0.0, 0.0, 1.0]


class RuntimeEventStreamClient:
    def __init__(self, host: str, port: int, reconnect_s: float = 1.0) -> None:
        self.host = host
        self.port = port
        self.reconnect_s = reconnect_s
        self.sock: socket.socket | None = None
        self.buffer = ""
        self.last_connect_attempt_s = 0.0
        self.reported_wait = False
        self.latest_ghost_detections: dict[str, Any] | None = None
        self.latest_ghost_seq: int | None = None
        self.latest_world_snapshot: dict[str, Any] | None = None
        self.latest_world_seq: int | None = None
        self.latest_selected_target: dict[str, Any] | None = None
        self.latest_mission_event: dict[str, Any] | None = None
        self.latest_mission_seq: int | None = None
        self.last_message_s: float | None = None
        self.runtime_stop_event: dict[str, Any] | None = None
        self.terminal_settled: bool | None = None

    def close(self) -> None:
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None

    def connect_if_needed(self) -> None:
        if self.sock is not None:
            return
        now = time.monotonic()
        if now - self.last_connect_attempt_s < self.reconnect_s:
            return
        self.last_connect_attempt_s = now
        try:
            sock = socket.create_connection((self.host, self.port), timeout=0.2)
            sock.setblocking(False)
            self.sock = sock
            self.buffer = ""
            self.reported_wait = False
            print(f"airsim-world-overlay: connected to runtime event stream {self.host}:{self.port}", file=sys.stderr)
        except OSError as exc:
            if not self.reported_wait:
                print(f"airsim-world-overlay: waiting for runtime event stream {self.host}:{self.port} ({exc})", file=sys.stderr)
                self.reported_wait = True
            self.close()

    def poll(self) -> None:
        self.connect_if_needed()
        if self.sock is None:
            return
        while True:
            try:
                chunk = self.sock.recv(65536)
            except BlockingIOError:
                break
            except OSError as exc:
                print(f"airsim-world-overlay: runtime event stream closed; waiting for reconnect ({exc})", file=sys.stderr)
                self.close()
                break
            if not chunk:
                print("airsim-world-overlay: runtime event stream closed; waiting for reconnect", file=sys.stderr)
                self.close()
                break
            self.buffer += chunk.decode("utf-8", errors="replace")
        while "\n" in self.buffer:
            raw, self.buffer = self.buffer.split("\n", 1)
            raw = raw.strip()
            if raw:
                self.handle_line(raw)

    def handle_line(self, raw: str) -> None:
        try:
            message = json.loads(raw)
        except json.JSONDecodeError as exc:
            print(f"airsim-world-overlay: ignoring malformed runtime event line ({exc})", file=sys.stderr)
            return
        message_type = message.get("type")
        seq = message.get("seq")
        seq_value = int(seq) if isinstance(seq, int) else None
        now = time.monotonic()
        if message_type == "ghost_detections":
            payload = message.get("ghost_detections")
            if isinstance(payload, dict):
                self.latest_ghost_detections = payload
                self.latest_ghost_seq = seq_value
                self.last_message_s = now
        elif message_type == "world_snapshot":
            snapshot = message.get("snapshot")
            if isinstance(snapshot, dict):
                self.latest_world_snapshot = snapshot
                self.latest_world_seq = seq_value
                self.last_message_s = now
        elif message_type == "mission_event":
            event = message.get("mission_event")
            if isinstance(event, dict):
                self.latest_mission_event = event
                self.latest_mission_seq = seq_value
                if event.get("event") == "target_selected":
                    self.latest_selected_target = event
                elif event.get("event") == "runtime_stop":
                    self.runtime_stop_event = event
                    self.terminal_settled = bool(event.get("terminal_settled", False))
                self.last_message_s = now


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stream-host", "--world-snapshot-stream-host", dest="stream_host", default="127.0.0.1")
    parser.add_argument("--stream-port", "--world-snapshot-stream-port", dest="stream_port", type=int, required=True)
    parser.add_argument("--host", default="127.0.0.1", help="AirSim RPC host")
    parser.add_argument("--rpc-port", type=int, default=41451, help="AirSim RPC port")
    parser.add_argument("--rate-hz", type=float, default=5.0)
    parser.add_argument("--duration-s", type=float, default=0.0)
    parser.add_argument("--follow", action="store_true")
    parser.add_argument("--clear", action="store_true")
    parser.add_argument("--persistent", action="store_true", help="Force all markers to be persistent.")
    parser.add_argument("--persistent-static", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--label", action="store_true")
    parser.add_argument("--max-agents", type=int, default=20)
    parser.add_argument("--z-lift-m", type=float, default=1.5)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--debug-every-s", type=float, default=1.0)
    parser.add_argument("--debug-json", default=None)
    parser.add_argument("--osd", action="store_true", help="Show live flight stats in the AirSim viewport HUD.")
    parser.add_argument("--osd-rate-hz", type=float, default=2.0)
    parser.add_argument("--osd-name", default="DEDALUS")
    parser.add_argument("--osd-state-name", default="DEDALUS-STATE")
    parser.add_argument("--osd-occupancy-name", default="DEDALUS-OCC")
    parser.add_argument("--osd-swept-volume-name", default="DEDALUS-SWEEP")
    parser.add_argument("--osd-evidence-name", default="DEDALUS-EVID")
    parser.add_argument("--osd-severity", type=int, default=0)
    parser.add_argument("--osd-arrow", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--osd-arrow-scale", type=float, default=1.5, help="XY velocity arrow length in meters at 1 m/s.")
    parser.add_argument("--osd-arrow-min-speed-mps", type=float, default=0.1)
    parser.add_argument("--osd-arrow-z-lift-m", type=float, default=0.75)
    parser.add_argument("--osd-arrow-duration-s", type=float, default=0.18)
    parser.add_argument("--osd-arrow-thickness", type=float, default=3.0)
    parser.add_argument("--wait-for-airsim-s", type=float, default=0.0, help="0 means wait until Ctrl-C.")
    parser.add_argument("--exit-on-runtime-stop", action="store_true", help="Exit cleanly when the mission stream publishes runtime_stop.")
    parser.add_argument("--wait-for-stream-s", type=float, default=0.0, help="0 means wait until Ctrl-C.")
    parser.add_argument("--hide-planned", action="store_true")
    parser.add_argument("--hide-world", action="store_true")
    parser.add_argument("--hide-ego", action="store_true")
    parser.add_argument("--hide-origin", action="store_true")
    parser.add_argument("--hide-selected", action="store_true")
    parser.add_argument("--show-occupancy-summary", action="store_true")
    parser.add_argument("--show-occupancy-cells", action="store_true")
    parser.add_argument("--max-occupancy-cells", type=int, default=64)
    parser.add_argument("--occupancy-z-lift-m", type=float, default=0.15)
    parser.add_argument("--show-swept-volume", action="store_true")
    parser.add_argument("--swept-volume-z-lift-m", type=float, default=0.25)
    parser.add_argument("--swept-volume-line-thickness", type=float, default=5.0)
    parser.add_argument("--swept-volume-blocking-size", type=float, default=24.0)
    parser.add_argument("--show-sensing-volumes", action="store_true")
    parser.add_argument("--max-sensing-volumes", type=int, default=4)
    parser.add_argument("--sensing-volume-z-lift-m", type=float, default=0.05)
    parser.add_argument("--sensing-volume-line-thickness", type=float, default=2.0)
    parser.add_argument("--show-obstacle-evidence", action="store_true")
    parser.add_argument("--max-obstacle-evidence", type=int, default=96)
    parser.add_argument("--obstacle-evidence-z-lift-m", type=float, default=0.35)
    parser.add_argument("--obstacle-evidence-size", type=float, default=12.0)
    return parser.parse_args()


def vec3(value: Any) -> list[float] | None:
    if not isinstance(value, list) or len(value) != 3:
        return None
    try:
        return [float(value[0]), float(value[1]), float(value[2])]
    except Exception:
        return None


def vec_delta(a: list[float] | None, b: list[float] | None) -> list[float] | None:
    if a is None or b is None:
        return None
    return [a[index] - b[index] for index in range(3)]


def vec_norm(value: list[float] | None) -> float | None:
    if value is None:
        return None
    return math.sqrt(sum(component * component for component in value))


def vec_add(a: list[float], b: list[float]) -> list[float]:
    return [a[index] + b[index] for index in range(3)]


def vec_scale(value: list[float], scale: float) -> list[float]:
    return [component * scale for component in value]


def vec_lift(value: list[float], z_lift_m: float) -> list[float]:
    return [value[0], value[1], value[2] - z_lift_m]


def airsim_vec(value: list[float]) -> airsim.Vector3r:
    return airsim.Vector3r(value[0], value[1], value[2])


def unit_or_default(value: Any, default: list[float]) -> list[float]:
    parsed = vec3(value)
    if parsed is None:
        return default
    norm = vec_norm(parsed)
    return default if norm is None or norm <= 1.0e-6 else [component / norm for component in parsed]


def rounded(value: Any) -> Any:
    if isinstance(value, float):
        return round(value, 4)
    if isinstance(value, list):
        return [rounded(item) for item in value]
    if isinstance(value, dict):
        return {key: rounded(item) for key, item in value.items()}
    return value


def distance_xy(a: Any, b: Any) -> float:
    try:
        return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))
    except Exception:
        return float("inf")


def marker_duration(args: argparse.Namespace) -> float:
    if args.persistent:
        return 0.0
    return max(0.12, 0.85 / max(args.rate_hz, 0.1))


def planned_agents_from_event(event: dict[str, Any] | None) -> list[dict[str, Any]]:
    if event is None:
        return []
    agents: list[dict[str, Any]] = []
    for detection in event.get("detections", []):
        if not isinstance(detection, dict):
            continue
        velocity = detection.get("velocity_local_mps", [0.0, 0.0, 0.0])
        dynamic = any(abs(float(component)) > 1.0e-9 for component in velocity)
        agents.append({"source": "planned", "source_track_id": detection.get("source_track_id", ""), "class": detection.get("class", "unknown"), "confidence": float(detection.get("confidence", 0.0)), "position_local": detection.get("position_local_m", [0.0, 0.0, 0.0]), "velocity_local": velocity, "dynamic": dynamic, "selected": False})
    return agents


def selected_track_id(selected_target: dict[str, Any] | None) -> str | None:
    if selected_target is None:
        return None
    value = selected_target.get("source_track_id") or selected_target.get("agent_id")
    return str(value) if value else None


def world_agents_from_snapshot(snapshot: dict[str, Any] | None, max_agents: int, selected_target: dict[str, Any] | None) -> list[dict[str, Any]]:
    if snapshot is None:
        return []
    selected_track = selected_track_id(selected_target)
    agents = [dict(agent, source="world", selected=(selected_track is not None and (agent.get("source_track_id") == selected_track or agent.get("agent_id") == selected_track))) for agent in snapshot.get("agents", []) if isinstance(agent, dict)]
    ego_position = snapshot.get("ego", {}).get("position_local", [0.0, 0.0, 0.0])
    agents.sort(key=lambda agent: (not agent.get("selected", False), distance_xy(agent.get("position_local"), ego_position)))
    return agents[: max(0, max_agents)]


def occupancy_from_snapshot(snapshot: dict[str, Any] | None) -> dict[str, Any] | None:
    if snapshot is None:
        return None
    occupancy = snapshot.get("ego_occupancy")
    return occupancy if isinstance(occupancy, dict) and occupancy.get("has_valid_occupancy") else None


def swept_volume_from_snapshot(snapshot: dict[str, Any] | None) -> dict[str, Any] | None:
    if snapshot is None:
        return None
    swept = snapshot.get("latest_swept_volume")
    return swept if isinstance(swept, dict) and swept.get("has_valid_query") else None


def sensing_volumes_from_snapshot(snapshot: dict[str, Any] | None, max_volumes: int) -> list[dict[str, Any]]:
    if snapshot is None:
        return []
    volumes = [
        volume for volume in snapshot.get("obstacle_sensing_volumes", [])
        if isinstance(volume, dict) and vec3(volume.get("origin_local")) is not None
    ]
    return volumes[: max(0, max_volumes)]


def obstacle_evidence_from_snapshot(snapshot: dict[str, Any] | None, max_evidence: int) -> list[dict[str, Any]]:
    if snapshot is None:
        return []
    evidence = [
        item for item in snapshot.get("obstacle_evidence", [])
        if isinstance(item, dict) and vec3(item.get("center_local")) is not None
    ]
    state_rank = {"thin_structure_risk": 0, "occupied": 1, "unknown": 2, "free": 3}
    evidence.sort(key=lambda item: (
        not bool(item.get("inside_swept_volume", False)),
        state_rank.get(str(item.get("state", "unknown")), 4),
        -float(item.get("confidence", 0.0)),
    ))
    return evidence[: max(0, max_evidence)]


def occupancy_cells_from_snapshot(snapshot: dict[str, Any] | None, max_cells: int) -> list[dict[str, Any]]:
    occupancy = occupancy_from_snapshot(snapshot)
    if occupancy is None:
        return []
    cells = [cell for cell in occupancy.get("debug_cells", []) if isinstance(cell, dict) and vec3(cell.get("center_local")) is not None]
    state_rank = {"occupied": 0, "unknown": 1, "free": 2}
    cells.sort(key=lambda cell: (state_rank.get(str(cell.get("state", "unknown")), 3), -float(cell.get("confidence", 0.0))))
    return cells[: max(0, max_cells)]


def draw_vec_from_position(position: Any, z_lift_m: float, source: str) -> list[float] | None:
    position_vec = vec3(position)
    if position_vec is None:
        return None
    lift = z_lift_m * (0.55 if source == "planned" else 1.0)
    if source == "selected":
        lift = z_lift_m * 1.25
    return [position_vec[0], position_vec[1], position_vec[2] - lift]


def vector3_from_position(position: Any, z_lift_m: float, source: str) -> airsim.Vector3r:
    draw_position = draw_vec_from_position(position, z_lift_m, source)
    if draw_position is None:
        raise ValueError(f"invalid position_local: {position!r}")
    return airsim.Vector3r(draw_position[0], draw_position[1], draw_position[2])


def marker_color(agent: dict[str, Any]) -> list[float]:
    if agent.get("selected"):
        return SELECTED_COLOR
    if agent.get("source") == "planned":
        return STATIC_GHOST_COLOR if not agent.get("dynamic") else PLANNED_GHOST_COLOR
    if str(agent.get("lifecycle", "active")) not in {"new", "active"}:
        return STALE_COLOR
    return WORLD_AGENT_COLOR


def occupancy_cell_color(cell: dict[str, Any]) -> list[float]:
    state = str(cell.get("state", "unknown"))
    if state == "occupied":
        return OCCUPIED_CELL_COLOR
    if state == "free":
        return FREE_CELL_COLOR
    return UNKNOWN_CELL_COLOR


def swept_volume_color(swept: dict[str, Any]) -> list[float]:
    status = str(swept.get("status", "unknown"))
    if status == "clear":
        return SWEEP_CLEAR_COLOR
    if status == "occupied_blocked":
        return SWEEP_BLOCKED_COLOR
    if status == "stale_map":
        return SWEEP_STALE_COLOR
    return SWEEP_UNKNOWN_COLOR


def obstacle_evidence_color(evidence: dict[str, Any]) -> list[float]:
    if evidence.get("inside_swept_volume"):
        return EVIDENCE_BLOCKING_COLOR
    state = str(evidence.get("state", "unknown"))
    if state == "thin_structure_risk":
        return EVIDENCE_THIN_RISK_COLOR
    if state == "occupied":
        return EVIDENCE_OCCUPIED_COLOR
    if state == "free":
        return FREE_CELL_COLOR
    return UNKNOWN_CELL_COLOR


def label_for(agent: dict[str, Any]) -> str:
    if agent.get("selected"):
        prefix = "SEL"
    elif agent.get("source") == "planned":
        prefix = "PLAN*" if not agent.get("dynamic") else "PLAN"
    else:
        prefix = "AG"
    return f"{prefix} {agent.get('class', 'unknown')} {agent.get('source_track_id', '')} {float(agent.get('confidence', 0.0)):.2f}"


def connect_airsim(args: argparse.Namespace) -> Any | None:
    if args.dry_run:
        return None
    start = time.time()
    reported = False
    while True:
        try:
            client = airsim.MultirotorClient(ip=args.host, port=args.rpc_port)
            client.confirmConnection()
            if args.clear:
                client.simFlushPersistentMarkers()
            return client
        except KeyboardInterrupt:
            raise
        except Exception as exc:
            if not reported:
                print(f"airsim-world-overlay: waiting for AirSim RPC at {args.host}:{args.rpc_port} ({exc})", file=sys.stderr)
                reported = True
            if args.wait_for_airsim_s > 0.0 and time.time() - start >= args.wait_for_airsim_s:
                raise TimeoutError(f"timed out waiting for AirSim RPC at {args.host}:{args.rpc_port}") from exc
            time.sleep(1.0 / max(args.rate_hz, 0.1))


def draw_agents(client: Any, agents: list[dict[str, Any]], args: argparse.Namespace, static_once: bool = False) -> None:
    duration = marker_duration(args)
    if args.dry_run:
        for agent in agents:
            print(f"  {label_for(agent)} pos={agent.get('position_local')} vel={agent.get('velocity_local')}")
        return
    for agent in agents:
        source = "selected" if agent.get("selected") else str(agent.get("source", "world"))
        try:
            point = vector3_from_position(agent.get("position_local"), args.z_lift_m, source)
        except ValueError:
            continue
        is_static = agent.get("source") == "planned" and not agent.get("dynamic")
        persistent = args.persistent or (args.persistent_static and is_static) or static_once
        agent_duration = 0.0 if persistent else duration
        color = marker_color(agent)
        size = 28.0 if agent.get("selected") else (12.0 if agent.get("source") == "planned" else 22.0)
        client.simPlotPoints([point], color_rgba=color, size=size, duration=agent_duration, is_persistent=persistent)
        if args.label:
            label_point = airsim.Vector3r(point.x_val, point.y_val, point.z_val - (0.45 if agent.get("source") == "planned" else 0.8))
            client.simPlotStrings([label_for(agent)], [label_point], scale=1.25 if agent.get("selected") else (0.9 if agent.get("source") == "planned" else 1.2), color_rgba=color, duration=agent_duration)


def draw_occupancy(client: Any, snapshot: dict[str, Any] | None, args: argparse.Namespace) -> None:
    occupancy = occupancy_from_snapshot(snapshot)
    if occupancy is None:
        return
    cells = occupancy_cells_from_snapshot(snapshot, args.max_occupancy_cells)
    if args.dry_run:
        if args.show_occupancy_summary:
            print("  OCC src={src} occ={occ} free={free} unk={unk} clear={clear}".format(src=occupancy.get("source_kind", "unknown"), occ=occupancy.get("occupied_count", 0), free=occupancy.get("free_count", 0), unk=occupancy.get("unknown_count", 0), clear=occupancy.get("forward_corridor_clearance_m", "n/a")))
        if args.show_occupancy_cells:
            for cell in cells:
                print(f"    OCC {cell.get('state', 'unknown')} pos={cell.get('center_local')} conf={cell.get('confidence', 0.0)} src={cell.get('source_provider', '-')}")
        return
    if client is None or not args.show_occupancy_cells:
        return
    duration = marker_duration(args)
    for cell in cells:
        center = vec3(cell.get("center_local"))
        if center is None:
            continue
        point = airsim.Vector3r(center[0], center[1], center[2] - args.occupancy_z_lift_m)
        state = str(cell.get("state", "unknown"))
        size = 16.0 if state == "occupied" else (10.0 if state == "unknown" else 7.0)
        client.simPlotPoints([point], color_rgba=occupancy_cell_color(cell), size=size, duration=duration, is_persistent=args.persistent)
        if args.label and state == "occupied":
            label = f"OCC {cell.get('source_object_name') or cell.get('source_provider') or state}"
            client.simPlotStrings([label], [airsim.Vector3r(point.x_val, point.y_val, point.z_val - 0.4)], scale=0.85, color_rgba=occupancy_cell_color(cell), duration=duration)


def lifted_vector(position: Any, z_lift_m: float) -> airsim.Vector3r | None:
    pos = vec3(position)
    if pos is None:
        return None
    return airsim.Vector3r(pos[0], pos[1], pos[2] - z_lift_m)


def plot_line_segments(client: Any, points: list[Any], color: list[float], thickness: float, duration: float, persistent: bool = False) -> None:
    try:
        client.simPlotLineList(points, color_rgba=color, thickness=thickness, duration=duration, is_persistent=persistent)
    except AttributeError:
        for index in range(0, len(points), 2):
            client.simPlotLineStrip(points[index : index + 2], color_rgba=color, thickness=thickness, duration=duration, is_persistent=persistent)


def frustum_corners(volume: dict[str, Any], z_lift_m: float) -> list[list[float]] | None:
    origin = vec3(volume.get("origin_local"))
    if origin is None:
        return None
    forward = unit_or_default(volume.get("forward_axis_local"), [1.0, 0.0, 0.0])
    right = unit_or_default(volume.get("right_axis_local"), [0.0, 1.0, 0.0])
    up = unit_or_default(volume.get("up_axis_local"), [0.0, 0.0, -1.0])
    try:
        near_range = max(0.0, float(volume.get("near_range_m", 0.0)))
        far_range = max(near_range, float(volume.get("far_range_m", near_range)))
        h_fov = max(0.0, float(volume.get("horizontal_fov_rad", 0.0)))
        v_fov = max(0.0, float(volume.get("vertical_fov_rad", 0.0)))
    except (TypeError, ValueError):
        return None

    def plane_corners(range_m: float) -> list[list[float]]:
        half_w = math.tan(h_fov * 0.5) * range_m
        half_h = math.tan(v_fov * 0.5) * range_m
        center = vec_add(origin, vec_scale(forward, range_m))
        return [
            vec_lift(vec_add(vec_add(center, vec_scale(right, -half_w)), vec_scale(up, -half_h)), z_lift_m),
            vec_lift(vec_add(vec_add(center, vec_scale(right, half_w)), vec_scale(up, -half_h)), z_lift_m),
            vec_lift(vec_add(vec_add(center, vec_scale(right, half_w)), vec_scale(up, half_h)), z_lift_m),
            vec_lift(vec_add(vec_add(center, vec_scale(right, -half_w)), vec_scale(up, half_h)), z_lift_m),
        ]

    return plane_corners(near_range) + plane_corners(far_range)


def draw_sensing_volumes(client: Any, snapshot: dict[str, Any] | None, args: argparse.Namespace) -> None:
    if not args.show_sensing_volumes:
        return
    volumes = sensing_volumes_from_snapshot(snapshot, args.max_sensing_volumes)
    if args.dry_run:
        for volume in volumes:
            print(
                "  SENSE provider={provider} sensor={sensor} origin={origin} near={near} far={far} hfov={hfov} vfov={vfov}".format(
                    provider=volume.get("provider_name", "unknown"),
                    sensor=volume.get("sensor_name", "unknown"),
                    origin=volume.get("origin_local"),
                    near=volume.get("near_range_m"),
                    far=volume.get("far_range_m"),
                    hfov=volume.get("horizontal_fov_rad"),
                    vfov=volume.get("vertical_fov_rad"),
                )
            )
        return
    if client is None:
        return
    duration = marker_duration(args)
    for volume in volumes:
        corners = frustum_corners(volume, args.sensing_volume_z_lift_m)
        if corners is None or len(corners) != 8:
            continue
        near = [airsim_vec(point) for point in corners[0:4]]
        far = [airsim_vec(point) for point in corners[4:8]]
        segments: list[airsim.Vector3r] = []
        for ring in (near, far):
            segments.extend([ring[0], ring[1], ring[1], ring[2], ring[2], ring[3], ring[3], ring[0]])
        for index in range(4):
            segments.extend([near[index], far[index]])
        plot_line_segments(
            client,
            segments,
            SENSING_VOLUME_COLOR,
            args.sensing_volume_line_thickness,
            duration,
            args.persistent,
        )
        if args.label:
            label_point = far[0]
            label = f"SENSE {volume.get('provider_name', 'unknown')}"
            client.simPlotStrings(
                [label],
                [airsim.Vector3r(label_point.x_val, label_point.y_val, label_point.z_val - 0.5)],
                scale=0.8,
                color_rgba=SENSING_VOLUME_COLOR,
                duration=duration,
            )


def draw_obstacle_evidence(client: Any, snapshot: dict[str, Any] | None, args: argparse.Namespace) -> None:
    if not args.show_obstacle_evidence:
        return
    evidence_items = obstacle_evidence_from_snapshot(snapshot, args.max_obstacle_evidence)
    if args.dry_run:
        for evidence in evidence_items:
            print(
                "  EVID state={state} shape={shape} pos={pos} conf={conf} src={src} in_sense={sense} in_sweep={sweep}".format(
                    state=evidence.get("state", "unknown"),
                    shape=evidence.get("shape", "unknown"),
                    pos=evidence.get("center_local"),
                    conf=evidence.get("confidence", 0.0),
                    src=evidence.get("source_provider", "-"),
                    sense=evidence.get("inside_sensing_volume", False),
                    sweep=evidence.get("inside_swept_volume", False),
                )
            )
        return
    if client is None:
        return
    duration = marker_duration(args)
    points_by_color: dict[tuple[float, float, float, float], list[airsim.Vector3r]] = {}
    for evidence in evidence_items:
        center = vec3(evidence.get("center_local"))
        if center is None:
            continue
        point = airsim.Vector3r(center[0], center[1], center[2] - args.obstacle_evidence_z_lift_m)
        color = tuple(obstacle_evidence_color(evidence))
        points_by_color.setdefault(color, []).append(point)
        if args.label and evidence.get("inside_swept_volume"):
            label = f"EVID {evidence.get('state', 'unknown')} {evidence.get('source_provider', '-')}"
            client.simPlotStrings([label], [airsim.Vector3r(point.x_val, point.y_val, point.z_val - 0.45)], scale=0.8, color_rgba=list(color), duration=duration)
    for color, points in points_by_color.items():
        size = args.obstacle_evidence_size * (1.6 if list(color) == EVIDENCE_BLOCKING_COLOR else 1.0)
        client.simPlotPoints(points, color_rgba=list(color), size=size, duration=duration, is_persistent=args.persistent)


def draw_swept_volume(client: Any, snapshot: dict[str, Any] | None, args: argparse.Namespace) -> None:
    if not args.show_swept_volume:
        return
    swept = swept_volume_from_snapshot(snapshot)
    if swept is None:
        return
    if args.dry_run:
        print("  SWEEP status={status} ttc={ttc} clear={clear} start={start} end={end} blockers={blockers}".format(
            status=swept.get("status", "unknown"),
            ttc=swept.get("time_to_collision_s", "n/a"),
            clear=swept.get("min_clearance_m", "n/a"),
            start=swept.get("start_local"),
            end=swept.get("end_local"),
            blockers=len(swept.get("blocking_cell_centers", []) or []),
        ))
        return
    if client is None:
        return
    start = lifted_vector(swept.get("start_local"), args.swept_volume_z_lift_m)
    end = lifted_vector(swept.get("end_local"), args.swept_volume_z_lift_m)
    if start is None or end is None:
        return
    duration = marker_duration(args)
    color = swept_volume_color(swept)
    plot_line_segments(client, [start, end], color, args.swept_volume_line_thickness, duration, args.persistent)
    blockers = []
    for center in swept.get("blocking_cell_centers", []) or []:
        point = lifted_vector(center, args.swept_volume_z_lift_m + 0.15)
        if point is not None:
            blockers.append(point)
    if blockers:
        client.simPlotPoints(blockers, color_rgba=SWEEP_BLOCKED_COLOR, size=args.swept_volume_blocking_size, duration=duration, is_persistent=args.persistent)
    if args.label:
        label = f"SWEEP {swept.get('status', 'unknown')} ttc={fixed_osd_value(swept.get('time_to_collision_s'), '.1f', 'n/a')}s"
        client.simPlotStrings([label], [airsim.Vector3r(end.x_val, end.y_val, end.z_val - 0.55)], scale=0.95, color_rgba=color, duration=duration)


def draw_reference_markers(client: Any, snapshot: dict[str, Any] | None, args: argparse.Namespace) -> None:
    if client is None:
        return
    duration = marker_duration(args)
    if args.debug and not args.hide_origin:
        origin = airsim.Vector3r(0.0, 0.0, -0.25)
        client.simPlotPoints([origin], color_rgba=ORIGIN_COLOR, size=16.0, duration=duration, is_persistent=args.persistent)
        client.simPlotStrings(["ORIGIN [0,0,0]"], [airsim.Vector3r(0.0, 0.0, -1.0)], scale=1.0, color_rgba=ORIGIN_COLOR, duration=duration)
    if args.hide_ego or snapshot is None:
        return
    ego_position = vec3(snapshot.get("ego", {}).get("position_local"))
    if ego_position is None:
        return
    ego_point = airsim.Vector3r(ego_position[0], ego_position[1], ego_position[2] - 0.75)
    client.simPlotPoints([ego_point], color_rgba=EGO_COLOR, size=18.0, duration=duration, is_persistent=args.persistent)
    if args.label:
        client.simPlotStrings(["EGO"], [airsim.Vector3r(ego_point.x_val, ego_point.y_val, ego_point.z_val - 0.8)], scale=1.0, color_rgba=EGO_COLOR, duration=duration)


def snapshot_time_s(snapshot: dict[str, Any] | None) -> float | None:
    if snapshot is None:
        return None
    timestamp_ns = snapshot.get("timestamp_ns")
    if isinstance(timestamp_ns, int):
        return timestamp_ns / 1_000_000_000.0
    try:
        return float(timestamp_ns) / 1_000_000_000.0
    except (TypeError, ValueError):
        return None


def ego_motion_stats(snapshot: dict[str, Any] | None, state: dict[str, Any]) -> dict[str, Any] | None:
    if snapshot is None:
        return None
    ego = snapshot.get("ego", {})
    if not isinstance(ego, dict):
        return None
    position = vec3(ego.get("position_local"))
    if position is None:
        return None
    timestamp_s = snapshot_time_s(snapshot) or time.monotonic()
    previous_position = state.get("last_ego_position")
    previous_timestamp_s = state.get("last_ego_timestamp_s")
    if previous_timestamp_s is not None and timestamp_s <= float(previous_timestamp_s):
        cached = state.get("last_ego_motion_stats")
        if isinstance(cached, dict):
            return cached
    stats: dict[str, Any] = {"position": position, "height_m": ego.get("height_m"), "velocity": None, "vxy_mps": None, "vz_mps": None, "heading_deg": None}
    if previous_position is None or previous_timestamp_s is None:
        state["last_ego_position"] = position
        state["last_ego_timestamp_s"] = timestamp_s
        state["last_ego_motion_stats"] = stats
        return stats
    dt = timestamp_s - float(previous_timestamp_s)
    if dt <= 1.0e-6:
        return state.get("last_ego_motion_stats", stats)
    velocity = [(position[index] - previous_position[index]) / dt for index in range(3)]
    vxy_mps = math.hypot(velocity[0], velocity[1])
    heading_deg = math.degrees(math.atan2(velocity[1], velocity[0])) if vxy_mps > 1.0e-6 else None
    stats.update({"velocity": velocity, "vxy_mps": vxy_mps, "vz_mps": velocity[2], "heading_deg": heading_deg})
    state["last_ego_position"] = position
    state["last_ego_timestamp_s"] = timestamp_s
    state["last_ego_motion_stats"] = stats
    return stats


def fixed_osd_value(value: Any, fmt: str, blank: str) -> str:
    if value is None:
        return blank
    try:
        return format(float(value), fmt)
    except (TypeError, ValueError):
        return blank


def format_osd_line(stats: dict[str, Any]) -> str:
    h = fixed_osd_value(stats.get("height_m"), "7.1f", "    n/a")
    vz = fixed_osd_value(stats.get("vz_mps"), "+7.1f", "    n/a")
    vxy = fixed_osd_value(stats.get("vxy_mps"), "6.1f", "   n/a")
    hdg = fixed_osd_value(stats.get("heading_deg"), "7.1f", "    n/a")
    return f"h={h} m  vz={vz} m/s  vxy={vxy} m/s  hdg={hdg} deg"


def short_text(value: Any, width: int) -> str:
    text = "Unknown" if value is None else str(value)
    if len(text) > width:
        return text[: max(0, width - 1)] + "~"
    return f"{text:<{width}}"


STATE_DISPLAY = {"Prepare": ("Arm", "arming"), "Takeoff": ("Takeoff", "climbing"), "ExecuteMission": ("Mission", "-"), "GoHome": ("GoHome", "returning"), "Land": ("Land", "landing"), "Complete": ("Settled", "done"), "Abort": ("Failed", "abort")}
COMMAND_DISPLAY = {"Arm": ("Arm", "send"), "Takeoff": ("Takeoff", "send"), "Land": ("Land", "send"), "Disarm": ("Disarm", "send"), "Velocity": ("Mission", "-")}


def compact_status(value: Any) -> str:
    text = "" if value is None else str(value)
    lowered = text.lower()
    if not text:
        return "-"
    if any(token in lowered for token in ("fail", "error", "exception", "timeout", "abort")):
        return "failed"
    if "arm" in lowered:
        return "arming"
    if "takeoff" in lowered or "climb" in lowered:
        return "climbing"
    if "land" in lowered:
        return "landing"
    if "home" in lowered:
        return "returning"
    if "complete" in lowered:
        return "done"
    return text


def fallback_display_state(mission_event: dict[str, Any] | None) -> tuple[str, str] | None:
    if not isinstance(mission_event, dict):
        return None
    event = mission_event.get("event")
    state = mission_event.get("state") or mission_event.get("to") or mission_event.get("from")
    command = mission_event.get("command")
    status = mission_event.get("status")
    if event == "runtime_stop":
        settled = mission_event.get("terminal_settled")
        return ("Settled", "done") if settled is True else ("Failed", "stopped")
    if event == "command_exception" or (event == "command_result" and mission_event.get("success") is False):
        return "Failed", str(command or "command")
    if command in COMMAND_DISPLAY:
        primary, detail = COMMAND_DISPLAY[command]
        return (primary, "ok") if event == "command_result" and mission_event.get("success") is True else (primary, detail)
    if state in STATE_DISPLAY:
        primary, default_detail = STATE_DISPLAY[state]
        detail = compact_status(status)
        return primary, detail if detail != "-" else default_detail
    return "Unknown", compact_status(status)


def format_osd_state_line(stats: dict[str, Any], mission_event: dict[str, Any] | None) -> str:
    if isinstance(mission_event, dict):
        primary = mission_event.get("display_state")
        detail = mission_event.get("display_detail")
        if primary is None:
            fallback = fallback_display_state(mission_event)
            if fallback is not None:
                primary, detail = fallback
        if primary is not None:
            return f" {short_text(primary, 8)} {short_text(detail or '', 12)}"
    vz = stats.get("vz_mps")
    vxy = stats.get("vxy_mps")
    if vz is None or vxy is None:
        return "state=waiting-for-motion-sample"
    vertical = "desc" if float(vz) > 0.05 else ("climb" if float(vz) < -0.05 else "level")
    motion = "moving" if float(vxy) > 0.1 else "hover"
    return f"state={vertical:<5} xy={motion:<6}"


def format_osd_occupancy_line(snapshot: dict[str, Any] | None) -> str | None:
    occupancy = occupancy_from_snapshot(snapshot)
    if occupancy is None:
        return None
    clear = fixed_osd_value(occupancy.get("forward_corridor_clearance_m"), ".1f", "n/a")
    nearest = fixed_osd_value(occupancy.get("nearest_obstacle_distance_m"), ".1f", "n/a")
    src = str(occupancy.get("source_kind", "unknown")).replace("_", "-")
    return f"src={short_text(src, 18)} occ={occupancy.get('occupied_count', 0)} free={occupancy.get('free_count', 0)} unk={occupancy.get('unknown_count', 0)} near={nearest} clear={clear}"


def format_osd_swept_volume_line(snapshot: dict[str, Any] | None) -> str | None:
    swept = swept_volume_from_snapshot(snapshot)
    if swept is None:
        return None
    status = str(swept.get("status", "unknown")).replace("_", "-")
    ttc = fixed_osd_value(swept.get("time_to_collision_s"), ".1f", "n/a")
    clear = fixed_osd_value(swept.get("min_clearance_m"), ".1f", "n/a")
    radius = fixed_osd_value(swept.get("radius_m"), ".1f", "n/a")
    blockers = len(swept.get("blocking_cell_centers", []) or [])
    return f"status={short_text(status, 16)} ttc={ttc}s clear={clear}m r={radius} blockers={blockers}"


def format_osd_obstacle_evidence_line(snapshot: dict[str, Any] | None) -> str | None:
    if snapshot is None:
        return None
    evidence_items = [
        item for item in snapshot.get("obstacle_evidence", [])
        if isinstance(item, dict)
    ]
    if not evidence_items:
        return None
    counts = {"occupied": 0, "free": 0, "unknown": 0, "thin_structure_risk": 0}
    in_sense = 0
    in_sweep = 0
    source = "-"
    for item in evidence_items:
        state = str(item.get("state", "unknown"))
        counts[state] = counts.get(state, 0) + 1
        if item.get("inside_sensing_volume"):
            in_sense += 1
        if item.get("inside_swept_volume"):
            in_sweep += 1
        if source == "-" and item.get("source_provider"):
            source = str(item.get("source_provider"))
    return (
        f"src={short_text(source.replace('_', '-'), 18)} "
        f"occ={counts.get('occupied', 0)} free={counts.get('free', 0)} unk={counts.get('unknown', 0)} "
        f"thin={counts.get('thin_structure_risk', 0)} sense={in_sense} sweep={in_sweep}"
    )


def draw_ego_velocity_arrow(client: Any, stats: dict[str, Any], args: argparse.Namespace) -> None:
    if client is None or not args.osd_arrow:
        return
    velocity = stats.get("velocity")
    position = stats.get("position")
    vxy_mps = stats.get("vxy_mps")
    if velocity is None or position is None or vxy_mps is None or float(vxy_mps) < args.osd_arrow_min_speed_mps:
        return
    z = position[2] - args.osd_arrow_z_lift_m
    start = airsim.Vector3r(position[0], position[1], z)
    end = airsim.Vector3r(position[0] + velocity[0] * args.osd_arrow_scale, position[1] + velocity[1] * args.osd_arrow_scale, z)
    duration = max(0.1, args.osd_arrow_duration_s)
    points = [start, end]
    if float(vxy_mps) >= 0.5:
        heading = math.atan2(velocity[1], velocity[0])
        head_len = 0.25
        head_angle = math.radians(25.0)
        points.extend([
            end,
            airsim.Vector3r(end.x_val - head_len * math.cos(heading - head_angle), end.y_val - head_len * math.sin(heading - head_angle), z),
            end,
            airsim.Vector3r(end.x_val - head_len * math.cos(heading + head_angle), end.y_val - head_len * math.sin(heading + head_angle), z),
        ])
    plot_line_segments(client, points, EGO_VELOCITY_COLOR, args.osd_arrow_thickness, duration)


def maybe_draw_osd(client: Any, snapshot: dict[str, Any] | None, mission_event: dict[str, Any] | None, args: argparse.Namespace, state: dict[str, Any]) -> None:
    if not args.osd:
        return
    stats = ego_motion_stats(snapshot, state)
    if stats is None:
        return
    occupancy_line = format_osd_occupancy_line(snapshot) if args.show_occupancy_summary else None
    swept_volume_line = format_osd_swept_volume_line(snapshot) if args.show_swept_volume else None
    evidence_line = format_osd_obstacle_evidence_line(snapshot) if args.show_obstacle_evidence else None
    if args.dry_run:
        print(f"OSD {args.osd_name}: {format_osd_line(stats)}")
        print(f"OSD {args.osd_state_name}: {format_osd_state_line(stats, mission_event)}")
        if occupancy_line is not None:
            print(f"OSD {args.osd_occupancy_name}: {occupancy_line}")
        if swept_volume_line is not None:
            print(f"OSD {args.osd_swept_volume_name}: {swept_volume_line}")
        if evidence_line is not None:
            print(f"OSD {args.osd_evidence_name}: {evidence_line}")
        return
    if client is None:
        return
    now = time.time()
    last_osd_s = float(state.get("last_osd_s", 0.0))
    period_s = 1.0 / max(args.osd_rate_hz, 0.1)
    if now - last_osd_s >= period_s:
        state["last_osd_s"] = now
        client.simPrintLogMessage(args.osd_name, format_osd_line(stats), severity=args.osd_severity)
        client.simPrintLogMessage(args.osd_state_name, format_osd_state_line(stats, mission_event), severity=args.osd_severity)
        if occupancy_line is not None:
            client.simPrintLogMessage(args.osd_occupancy_name, occupancy_line, severity=args.osd_severity)
        if swept_volume_line is not None:
            client.simPrintLogMessage(args.osd_swept_volume_name, swept_volume_line, severity=args.osd_severity)
        if evidence_line is not None:
            client.simPrintLogMessage(args.osd_evidence_name, evidence_line, severity=args.osd_severity)
    draw_ego_velocity_arrow(client, stats, args)


def build_debug_report(ghost_event: dict[str, Any] | None, ghost_seq: int | None, snapshot: dict[str, Any] | None, snapshot_seq: int | None, mission_seq: int | None, selected_target: dict[str, Any] | None, planned_agents: list[dict[str, Any]], world_agents: list[dict[str, Any]]) -> dict[str, Any]:
    planned_by_track = {str(agent.get("source_track_id")): agent for agent in planned_agents if agent.get("source_track_id")}
    world_by_track = {str(agent.get("source_track_id")): agent for agent in world_agents if agent.get("source_track_id")}
    ego = snapshot.get("ego", {}) if snapshot is not None else {}
    ego_position = vec3(ego.get("position_local"))
    tracks: list[dict[str, Any]] = []
    for track in sorted(set(planned_by_track) | set(world_by_track)):
        planned = planned_by_track.get(track)
        world = world_by_track.get(track)
        planned_position = vec3(planned.get("position_local")) if planned else None
        world_position = vec3(world.get("position_local")) if world else None
        delta_plan_minus_world = vec_delta(planned_position, world_position)
        tracks.append({"source_track_id": track, "selected": bool(world.get("selected")) if world else False, "planned_position_local": planned_position, "world_position_local": world_position, "delta_plan_minus_world": delta_plan_minus_world, "delta_plan_minus_world_norm_m": vec_norm(delta_plan_minus_world), "world_minus_ego": vec_delta(world_position, ego_position)})
    return {"ghost_seq": ghost_seq, "world_seq": snapshot_seq, "mission_seq": mission_seq, "selected_target": selected_target, "ghost_timestamp_ns": ghost_event.get("timestamp_ns") if ghost_event else None, "world_timestamp_ns": snapshot.get("timestamp_ns") if snapshot else None, "ghost_elapsed_s": ghost_event.get("scenario_elapsed_s") if ghost_event else None, "ego": {"position_local": ego_position, "height_m": ego.get("height_m"), "map_frame_id": ego.get("map_frame_id")}, "occupancy": occupancy_from_snapshot(snapshot), "swept_volume": swept_volume_from_snapshot(snapshot), "obstacle_sensing_volumes": sensing_volumes_from_snapshot(snapshot, 8), "obstacle_evidence": obstacle_evidence_from_snapshot(snapshot, 128), "tracks": tracks}


def maybe_print_debug_report(report: dict[str, Any], args: argparse.Namespace, state: dict[str, float]) -> None:
    if not args.debug:
        return
    now = time.time()
    if now - state.get("last_debug_print_s", 0.0) < max(0.0, args.debug_every_s):
        return
    state["last_debug_print_s"] = now
    print("airsim-world-overlay debug:", file=sys.stderr)
    selected = report.get("selected_target") or {}
    print(f"  ghost_seq={report.get('ghost_seq')} world_seq={report.get('world_seq')} mission_seq={report.get('mission_seq')} selected={selected.get('source_track_id') or selected.get('agent_id') or '-'} ghost_ts={report.get('ghost_timestamp_ns')} world_ts={report.get('world_timestamp_ns')} ghost_elapsed_s={rounded(report.get('ghost_elapsed_s'))}", file=sys.stderr)
    ego = report.get("ego", {})
    print(f"  ego position_local={rounded(ego.get('position_local'))} height_m={rounded(ego.get('height_m'))} map={ego.get('map_frame_id')}", file=sys.stderr)
    occupancy = report.get("occupancy")
    if isinstance(occupancy, dict):
        print(f"  occupancy src={occupancy.get('source_kind')} occ={occupancy.get('occupied_count')} free={occupancy.get('free_count')} unk={occupancy.get('unknown_count')} clear={occupancy.get('forward_corridor_clearance_m')}", file=sys.stderr)
    swept = report.get("swept_volume")
    if isinstance(swept, dict):
        print(f"  swept_volume status={swept.get('status')} ttc={swept.get('time_to_collision_s')} clear={swept.get('min_clearance_m')} blockers={len(swept.get('blocking_cell_centers', []) or [])}", file=sys.stderr)
    sensing = report.get("obstacle_sensing_volumes", [])
    evidence = report.get("obstacle_evidence", [])
    if isinstance(sensing, list) or isinstance(evidence, list):
        in_sweep = sum(1 for item in evidence if isinstance(item, dict) and item.get("inside_swept_volume"))
        in_sense = sum(1 for item in evidence if isinstance(item, dict) and item.get("inside_sensing_volume"))
        thin = sum(1 for item in evidence if isinstance(item, dict) and item.get("state") == "thin_structure_risk")
        print(f"  obstacle_sensing_evidence volumes={len(sensing) if isinstance(sensing, list) else 0} evidence={len(evidence) if isinstance(evidence, list) else 0} in_sense={in_sense} in_sweep={in_sweep} thin={thin}", file=sys.stderr)
    for track in report.get("tracks", []):
        print("  track={track} selected={selected} plan={plan} ag={ag} delta={delta} norm={norm} ag_minus_ego={rel}".format(track=track.get("source_track_id"), selected=track.get("selected"), plan=rounded(track.get("planned_position_local")), ag=rounded(track.get("world_position_local")), delta=rounded(track.get("delta_plan_minus_world")), norm=rounded(track.get("delta_plan_minus_world_norm_m")), rel=rounded(track.get("world_minus_ego"))), file=sys.stderr)


def maybe_write_debug_json(report: dict[str, Any], args: argparse.Namespace) -> None:
    if args.debug_json is None:
        return
    from pathlib import Path
    path = Path(args.debug_json)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(rounded(report), indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    if args.rate_hz <= 0.0:
        raise ValueError("--rate-hz must be positive")
    if args.stream_port <= 0:
        raise ValueError("--stream-port must be positive")
    stream = RuntimeEventStreamClient(args.stream_host, args.stream_port)
    client = connect_airsim(args)
    start = time.time()
    deadline = None if args.duration_s <= 0.0 else start + args.duration_s
    waiting_since = time.time()
    waiting_reported = False
    static_drawn = False
    debug_state = {"last_debug_print_s": 0.0}
    osd_state: dict[str, Any] = {"last_osd_s": 0.0}
    while True:
        stream.poll()
        if args.exit_on_runtime_stop and stream.runtime_stop_event is not None:
            print(f"airsim-world-overlay: exiting on runtime_stop terminal_settled={stream.terminal_settled}", file=sys.stderr)
            break
        if stream.last_message_s is None:
            if not waiting_reported:
                print(f"airsim-world-overlay: waiting for runtime events on {args.stream_host}:{args.stream_port}", file=sys.stderr)
                waiting_reported = True
            if args.wait_for_stream_s > 0.0 and time.time() - waiting_since >= args.wait_for_stream_s:
                raise TimeoutError("timed out waiting for runtime events")
            time.sleep(1.0 / args.rate_hz)
            continue
        planned_agents = [] if args.hide_planned else planned_agents_from_event(stream.latest_ghost_detections)
        selected_target = None if args.hide_selected else stream.latest_selected_target
        world_agents = [] if args.hide_world else world_agents_from_snapshot(stream.latest_world_snapshot, args.max_agents, selected_target)
        static_planned = [agent for agent in planned_agents if not agent.get("dynamic")]
        dynamic_planned = [agent for agent in planned_agents if agent.get("dynamic")]
        if static_planned and not static_drawn:
            draw_agents(client, static_planned, args, static_once=True)
            static_drawn = True
        draw_agents(client, dynamic_planned, args)
        draw_agents(client, world_agents, args)
        draw_occupancy(client, stream.latest_world_snapshot, args)
        draw_sensing_volumes(client, stream.latest_world_snapshot, args)
        draw_obstacle_evidence(client, stream.latest_world_snapshot, args)
        draw_swept_volume(client, stream.latest_world_snapshot, args)
        draw_reference_markers(client, stream.latest_world_snapshot, args)
        maybe_draw_osd(client, stream.latest_world_snapshot, stream.latest_mission_event, args, osd_state)
        report = build_debug_report(stream.latest_ghost_detections, stream.latest_ghost_seq, stream.latest_world_snapshot, stream.latest_world_seq, stream.latest_mission_seq, selected_target, planned_agents, world_agents)
        maybe_print_debug_report(report, args, debug_state)
        maybe_write_debug_json(report, args)
        if not args.follow:
            break
        if deadline is not None and time.time() >= deadline:
            break
        time.sleep(1.0 / args.rate_hz)
    stream.close()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nairsim-world-overlay: interrupted", file=sys.stderr)
        raise SystemExit(130)
    except Exception as exc:
        print(f"\nairsim-world-overlay: {exc}", file=sys.stderr)
        raise SystemExit(1)
