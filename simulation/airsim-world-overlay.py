#!/usr/bin/env python3
"""Draw ghost detections and world-model state into the AirSim viewport.

Normal live modes are artifact-free:

  ghost_scenario:
    planned ghost markers from the shared C++ GhostScenario evaluator.

  world_snapshot:
    AG/EGO markers from the live WorldSnapshot TCP JSONL stream.

  combined:
    planned ghost markers + live WorldSnapshot AG/EGO markers.

Artifact modes remain explicit debug fallbacks only:

  artifact_snapshot
  artifact_combined
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import subprocess
import sys
import time
from pathlib import Path
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
DEFAULT_REQUIRED_TRACKS = ["ghost_person_001", "ghost_person_002", "ghost_car_001"]
DEFAULT_GHOST_SCENARIO = Path("simulation/ghost_detections/person_pair_crossing.json")
DEFAULT_GHOST_EVALUATOR = Path("build-staging/apps/dedalus_ghost_scenario_eval")
STREAM_SOURCES = {"world_snapshot", "combined"}
ARTIFACT_SOURCES = {"artifact_snapshot", "artifact_combined"}
PLANNED_SOURCES = {"ghost_scenario", "combined", "artifact_combined"}


class WorldSnapshotStreamClient:
    def __init__(self, host: str, port: int, reconnect_s: float = 1.0) -> None:
        self.host = host
        self.port = port
        self.reconnect_s = reconnect_s
        self.sock: socket.socket | None = None
        self.buffer = ""
        self.latest_snapshot: dict[str, Any] | None = None
        self.latest_seq: int | None = None
        self.last_connect_attempt_s = 0.0
        self.reported_wait = False

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
            print(f"airsim-world-overlay: connected to WorldSnapshot stream {self.host}:{self.port}", file=sys.stderr)
        except OSError as exc:
            if not self.reported_wait:
                print(f"airsim-world-overlay: waiting for WorldSnapshot stream {self.host}:{self.port} ({exc})", file=sys.stderr)
                self.reported_wait = True
            self.close()

    def poll(self) -> tuple[dict[str, Any] | None, int | None]:
        self.connect_if_needed()
        if self.sock is None:
            return self.latest_snapshot, self.latest_seq

        while True:
            try:
                chunk = self.sock.recv(65536)
            except BlockingIOError:
                break
            except OSError as exc:
                print(f"airsim-world-overlay: WorldSnapshot stream disconnected ({exc})", file=sys.stderr)
                self.close()
                break
            if not chunk:
                print("airsim-world-overlay: WorldSnapshot stream closed", file=sys.stderr)
                self.close()
                break
            self.buffer += chunk.decode("utf-8", errors="replace")

        while "\n" in self.buffer:
            raw, self.buffer = self.buffer.split("\n", 1)
            raw = raw.strip()
            if not raw:
                continue
            try:
                message = json.loads(raw)
            except json.JSONDecodeError as exc:
                print(f"airsim-world-overlay: ignoring malformed WorldSnapshot stream line ({exc})", file=sys.stderr)
                continue
            if message.get("type") != "world_snapshot":
                continue
            snapshot = message.get("snapshot")
            if not isinstance(snapshot, dict):
                continue
            seq = message.get("seq")
            self.latest_snapshot = snapshot
            self.latest_seq = int(seq) if isinstance(seq, int) else None

        return self.latest_snapshot, self.latest_seq


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot-dir", type=Path, default=None, help="Artifact snapshot directory for artifact_* debug modes only.")
    parser.add_argument("--mission-events", type=Path, default=None, help="Optional mission_events.jsonl for SEL labels until live event stream exists.")
    parser.add_argument("--ghost-scenario", type=Path, default=DEFAULT_GHOST_SCENARIO)
    parser.add_argument("--ghost-evaluator", type=Path, default=DEFAULT_GHOST_EVALUATOR)
    parser.add_argument(
        "--source",
        choices=["ghost_scenario", "world_snapshot", "combined", "artifact_snapshot", "artifact_combined"],
        default="ghost_scenario",
        help="world_snapshot/combined use live TCP stream; artifact_* modes are explicit debug fallbacks.",
    )
    parser.add_argument("--world-snapshot-stream-host", default="127.0.0.1")
    parser.add_argument("--world-snapshot-stream-port", type=int, default=0)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--rpc-port", type=int, default=41451)
    parser.add_argument("--rate-hz", type=float, default=2.0)
    parser.add_argument("--duration-s", type=float, default=0.0)
    parser.add_argument("--follow", action="store_true")
    parser.add_argument("--clear", action="store_true")
    parser.add_argument("--persistent", action="store_true", help="Force all markers to be persistent.")
    parser.add_argument("--persistent-static", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--label", action="store_true")
    parser.add_argument("--max-agents", type=int, default=20)
    parser.add_argument("--z-lift-m", type=float, default=1.5)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--debug", action="store_true", help="Print PLAN/AG coordinate comparisons and draw ORIGIN/EGO reference markers when available.")
    parser.add_argument("--debug-every-s", type=float, default=1.0)
    parser.add_argument("--debug-json", type=Path, default=None)
    parser.add_argument("--wait-for-airsim-s", type=float, default=0.0, help="0 means wait until Ctrl-C.")
    parser.add_argument("--wait-for-world-model-s", type=float, default=0.0, help="0 means wait until Ctrl-C in world_snapshot mode only.")
    parser.add_argument("--required-track", action="append", default=None)
    parser.add_argument("--allow-partial-tracks", action="store_true")
    return parser.parse_args()


def read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def evaluate_ghost_scenario(evaluator: Path, scenario: Path, time_s: float) -> list[dict[str, Any]]:
    result = subprocess.run(
        [str(evaluator), "--scenario", str(scenario), "--time-s", f"{max(0.0, time_s):.6f}"],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    payload = json.loads(result.stdout)
    detections = payload.get("detections", [])
    agents: list[dict[str, Any]] = []
    for detection in detections:
        if not isinstance(detection, dict):
            continue
        velocity = detection.get("velocity_local_mps", [0.0, 0.0, 0.0])
        is_dynamic = any(abs(float(component)) > 1.0e-9 for component in velocity)
        agents.append(
            {
                "source": "planned",
                "source_track_id": detection.get("source_track_id", ""),
                "class": detection.get("class", "unknown"),
                "confidence": float(detection.get("confidence", 0.0)),
                "position_local": detection.get("position_local_m", [0.0, 0.0, 0.0]),
                "velocity_local": velocity,
                "size_m": detection.get("size_m", [1.0, 1.0, 1.0]),
                "lifecycle": "dynamic" if is_dynamic else "static",
                "dynamic": is_dynamic,
            }
        )
    return agents


def snapshot_paths_from_manifest(snapshot_dir: Path) -> list[Path]:
    manifest = snapshot_dir / "snapshot_manifest.txt"
    if not manifest.exists():
        return []
    paths: list[Path] = []
    for raw in manifest.read_text(encoding="utf-8").splitlines():
        if not raw.strip() or raw.startswith("#"):
            continue
        fields = raw.split()
        if len(fields) >= 2:
            paths.append(snapshot_dir / fields[1])
    return paths


def first_existing_snapshot(snapshot_dir: Path) -> tuple[Path, dict[str, Any]] | None:
    for path in snapshot_paths_from_manifest(snapshot_dir):
        if path.exists():
            return path, read_json(path)
    return None


def latest_existing_snapshot(snapshot_dir: Path) -> tuple[Path, dict[str, Any]] | None:
    for path in reversed(snapshot_paths_from_manifest(snapshot_dir)):
        if path.exists():
            return path, read_json(path)
    return None


def snapshot_timestamp_ns(snapshot: dict[str, Any] | None) -> int | None:
    if snapshot is None:
        return None
    value = snapshot.get("timestamp_ns")
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        return int(value)
    return None


def snapshot_time_window(snapshot_dir: Path) -> tuple[float | None, Path | None, dict[str, Any] | None]:
    first = first_existing_snapshot(snapshot_dir)
    latest = latest_existing_snapshot(snapshot_dir)
    if first is None or latest is None:
        return None, None, None
    _, first_snapshot = first
    latest_path, latest_snapshot = latest
    first_ts = snapshot_timestamp_ns(first_snapshot)
    latest_ts = snapshot_timestamp_ns(latest_snapshot)
    elapsed_s = None
    if first_ts is not None and latest_ts is not None:
        elapsed_s = max(0.0, (latest_ts - first_ts) / 1_000_000_000.0)
    return elapsed_s, latest_path, latest_snapshot


def selected_source_track_id(events_path: Path | None, snapshot: dict[str, Any] | None) -> str | None:
    if snapshot is None or events_path is None or not events_path.exists():
        return None
    snapshot_ts = snapshot_timestamp_ns(snapshot)
    selected: str | None = None
    for raw in events_path.read_text(encoding="utf-8").splitlines():
        if not raw.strip():
            continue
        try:
            event = json.loads(raw)
        except json.JSONDecodeError:
            continue
        if event.get("event") != "target_selected" or not event.get("source_track_id"):
            continue
        event_ts = event.get("timestamp_ns")
        if snapshot_ts is not None and isinstance(event_ts, int) and event_ts > snapshot_ts:
            continue
        selected = str(event["source_track_id"])
    return selected


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
    return [a[i] - b[i] for i in range(3)]


def vec_norm(value: list[float] | None) -> float | None:
    if value is None:
        return None
    return math.sqrt(sum(component * component for component in value))


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


def agent_track_ids(snapshot: dict[str, Any]) -> set[str]:
    tracks: set[str] = set()
    for agent in snapshot.get("agents", []):
        if isinstance(agent, dict) and agent.get("source_track_id"):
            tracks.add(str(agent["source_track_id"]))
    return tracks


def snapshot_ready(snapshot: dict[str, Any], required_tracks: list[str], allow_partial: bool) -> tuple[bool, str]:
    tracks = agent_track_ids(snapshot)
    if allow_partial:
        return (bool(tracks), f"tracks={sorted(tracks)}")
    missing = [track for track in required_tracks if track not in tracks]
    if missing:
        return (False, f"waiting_for_tracks={missing} present={sorted(tracks)}")
    return (True, f"tracks={sorted(tracks)}")


def world_agents_to_draw(snapshot: dict[str, Any], max_agents: int, selected_track: str | None) -> list[dict[str, Any]]:
    agents = [dict(agent, source="world") for agent in snapshot.get("agents", []) if isinstance(agent, dict)]
    ego_position = snapshot.get("ego", {}).get("position_local", [0.0, 0.0, 0.0])
    agents.sort(key=lambda agent: (agent.get("source_track_id") != selected_track, distance_xy(agent.get("position_local"), ego_position)))
    return agents[: max(0, max_agents)]


def draw_vec_from_position(position: Any, z_lift_m: float, source: str) -> list[float] | None:
    position_vec = vec3(position)
    if position_vec is None:
        return None
    lift = z_lift_m * (0.55 if source == "planned" else 1.0)
    return [position_vec[0], position_vec[1], position_vec[2] - lift]


def vector3_from_position(position: Any, z_lift_m: float, source: str) -> airsim.Vector3r:
    draw_position = draw_vec_from_position(position, z_lift_m, source)
    if draw_position is None:
        raise ValueError(f"invalid position_local: {position!r}")
    return airsim.Vector3r(draw_position[0], draw_position[1], draw_position[2])


def marker_color(agent: dict[str, Any], selected_track: str | None) -> list[float]:
    if agent.get("source") == "planned":
        return STATIC_GHOST_COLOR if not agent.get("dynamic") else PLANNED_GHOST_COLOR
    if selected_track and agent.get("source_track_id") == selected_track:
        return SELECTED_COLOR
    if str(agent.get("lifecycle", "active")) not in {"new", "active"}:
        return STALE_COLOR
    return WORLD_AGENT_COLOR


def label_for(agent: dict[str, Any], selected_track: str | None) -> str:
    if agent.get("source") == "planned":
        prefix = "PLAN*" if not agent.get("dynamic") else "PLAN"
    else:
        prefix = "SEL" if selected_track and agent.get("source_track_id") == selected_track else "AG"
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


def draw_reference_markers(client: Any, snapshot: dict[str, Any] | None, args: argparse.Namespace) -> None:
    if not args.debug or client is None:
        return
    duration = 0.0 if args.persistent else max(0.5, 1.5 / max(args.rate_hz, 0.1))
    origin = airsim.Vector3r(0.0, 0.0, -0.25)
    client.simPlotPoints([origin], color_rgba=ORIGIN_COLOR, size=16.0, duration=duration, is_persistent=args.persistent)
    client.simPlotStrings(["ORIGIN [0,0,0]"], [airsim.Vector3r(0.0, 0.0, -1.0)], scale=1.0, color_rgba=ORIGIN_COLOR, duration=duration)
    if snapshot is None:
        return
    ego_position = vec3(snapshot.get("ego", {}).get("position_local"))
    if ego_position is None:
        return
    ego_point = airsim.Vector3r(ego_position[0], ego_position[1], ego_position[2] - 0.75)
    client.simPlotPoints([ego_point], color_rgba=EGO_COLOR, size=18.0, duration=duration, is_persistent=args.persistent)
    client.simPlotStrings(["EGO"], [airsim.Vector3r(ego_point.x_val, ego_point.y_val, ego_point.z_val - 0.8)], scale=1.0, color_rgba=EGO_COLOR, duration=duration)


def draw_agents(client: Any, label: str, agents: list[dict[str, Any]], selected_track: str | None, args: argparse.Namespace, static_once: bool = False) -> None:
    del label
    duration = 0.0 if args.persistent else max(0.5, 1.5 / max(args.rate_hz, 0.1))
    if args.dry_run:
        for agent in agents:
            print(f"  {label_for(agent, selected_track)} pos={agent.get('position_local')} vel={agent.get('velocity_local')}")
        return
    for agent in agents:
        try:
            point = vector3_from_position(agent.get("position_local"), args.z_lift_m, str(agent.get("source", "world")))
        except ValueError:
            continue
        is_static = agent.get("source") == "planned" and not agent.get("dynamic")
        persistent = args.persistent or (args.persistent_static and is_static) or static_once
        agent_duration = 0.0 if persistent else duration
        color = marker_color(agent, selected_track)
        size = 12.0 if agent.get("source") == "planned" else 22.0
        client.simPlotPoints([point], color_rgba=color, size=size, duration=agent_duration, is_persistent=persistent)
        if args.label:
            label_point = airsim.Vector3r(point.x_val, point.y_val, point.z_val - (0.45 if agent.get("source") == "planned" else 0.8))
            client.simPlotStrings([label_for(agent, selected_track)], [label_point], scale=0.9 if agent.get("source") == "planned" else 1.2, color_rgba=color, duration=agent_duration)


def build_debug_report(snapshot_path: str | None, snapshot: dict[str, Any] | None, planned_agents: list[dict[str, Any]], world_agents: list[dict[str, Any]], selected_track: str | None, elapsed_s: float, args: argparse.Namespace, stream_seq: int | None) -> dict[str, Any]:
    planned_by_track = {str(agent.get("source_track_id")): agent for agent in planned_agents if agent.get("source_track_id")}
    world_by_track = {str(agent.get("source_track_id")): agent for agent in world_agents if agent.get("source_track_id")}
    tracks = sorted(set(planned_by_track) | set(world_by_track))
    ego = snapshot.get("ego", {}) if snapshot is not None else {}
    ego_position = vec3(ego.get("position_local"))
    report_tracks: list[dict[str, Any]] = []
    for track in tracks:
        planned = planned_by_track.get(track)
        world = world_by_track.get(track)
        planned_position = vec3(planned.get("position_local")) if planned else None
        world_position = vec3(world.get("position_local")) if world else None
        delta_plan_minus_world = vec_delta(planned_position, world_position)
        world_minus_ego = vec_delta(world_position, ego_position)
        report_tracks.append({
            "source_track_id": track,
            "selected": track == selected_track,
            "planned_position_local": planned_position,
            "world_position_local": world_position,
            "delta_plan_minus_world": delta_plan_minus_world,
            "delta_plan_minus_world_norm_m": vec_norm(delta_plan_minus_world),
            "world_minus_ego": world_minus_ego,
            "world_minus_ego_norm_m": vec_norm(world_minus_ego),
            "planned_draw_position": draw_vec_from_position(planned_position, args.z_lift_m, "planned") if planned_position else None,
            "world_draw_position": draw_vec_from_position(world_position, args.z_lift_m, "world") if world_position else None,
            "planned_velocity_local": vec3(planned.get("velocity_local")) if planned else None,
            "world_velocity_local": vec3(world.get("velocity_local")) if world else None,
        })
    return {
        "snapshot": snapshot_path,
        "stream_seq": stream_seq,
        "snapshot_timestamp_ns": snapshot_timestamp_ns(snapshot),
        "elapsed_s": elapsed_s,
        "selected_source_track_id": selected_track,
        "ego": {"position_local": ego_position, "height_m": ego.get("height_m"), "map_frame_id": ego.get("map_frame_id")},
        "tracks": report_tracks,
    }


def maybe_print_debug_report(report: dict[str, Any], args: argparse.Namespace, state: dict[str, float]) -> None:
    if not args.debug:
        return
    now = time.time()
    if now - state.get("last_debug_print_s", 0.0) < max(0.0, args.debug_every_s):
        return
    state["last_debug_print_s"] = now
    print("airsim-world-overlay debug:", file=sys.stderr)
    print(f"  snapshot={report.get('snapshot')} seq={report.get('stream_seq')} ts={report.get('snapshot_timestamp_ns')} elapsed_s={report.get('elapsed_s'):.3f} selected={report.get('selected_source_track_id') or '-'}", file=sys.stderr)
    ego = report.get("ego", {})
    print(f"  ego position_local={rounded(ego.get('position_local'))} height_m={rounded(ego.get('height_m'))} map={ego.get('map_frame_id')}", file=sys.stderr)
    for track in report.get("tracks", []):
        print(
            "  track={track} selected={selected} plan={plan} ag={ag} delta={delta} norm={norm} ag_minus_ego={rel} draw_plan={draw_plan} draw_ag={draw_ag}".format(
                track=track.get("source_track_id"), selected=track.get("selected"), plan=rounded(track.get("planned_position_local")), ag=rounded(track.get("world_position_local")), delta=rounded(track.get("delta_plan_minus_world")), norm=rounded(track.get("delta_plan_minus_world_norm_m")), rel=rounded(track.get("world_minus_ego")), draw_plan=rounded(track.get("planned_draw_position")), draw_ag=rounded(track.get("world_draw_position"))
            ),
            file=sys.stderr,
        )


def maybe_write_debug_json(report: dict[str, Any], args: argparse.Namespace) -> None:
    if args.debug_json is None:
        return
    args.debug_json.parent.mkdir(parents=True, exist_ok=True)
    args.debug_json.write_text(json.dumps(rounded(report), indent=2, sort_keys=True) + "\n", encoding="utf-8")


def source_uses_stream(source: str) -> bool:
    return source in STREAM_SOURCES


def source_uses_artifacts(source: str) -> bool:
    return source in ARTIFACT_SOURCES


def source_uses_planned(source: str) -> bool:
    return source in PLANNED_SOURCES


def main() -> int:
    args = parse_args()
    if args.rate_hz <= 0.0:
        raise ValueError("--rate-hz must be positive")
    if source_uses_stream(args.source) and args.world_snapshot_stream_port <= 0:
        raise ValueError("--world-snapshot-stream-port is required for world_snapshot and combined live modes")
    if source_uses_artifacts(args.source) and args.snapshot_dir is None:
        raise ValueError("--snapshot-dir is required for artifact_snapshot and artifact_combined modes")

    required_tracks = args.required_track or DEFAULT_REQUIRED_TRACKS
    events_path = args.mission_events or ((args.snapshot_dir / "mission_events.jsonl") if args.snapshot_dir else None)
    stream_client = WorldSnapshotStreamClient(args.world_snapshot_stream_host, args.world_snapshot_stream_port) if source_uses_stream(args.source) else None
    client = connect_airsim(args)

    start = time.time()
    deadline = None if args.duration_s <= 0.0 else start + args.duration_s
    world_model_wait_start = time.time()
    waiting_reported = False
    ready_reported = False
    static_drawn = False
    debug_state = {"last_debug_print_s": 0.0}

    while True:
        if args.clear and client is not None and not static_drawn:
            client.simFlushPersistentMarkers()

        elapsed = time.time() - start
        snapshot_name: str | None = None
        snapshot = None
        stream_seq: int | None = None
        world_agents: list[dict[str, Any]] = []

        if source_uses_stream(args.source) and stream_client is not None:
            snapshot, stream_seq = stream_client.poll()
            if stream_seq is not None:
                snapshot_name = f"stream:{stream_seq}"
        elif source_uses_artifacts(args.source) and args.snapshot_dir is not None:
            snapshot_elapsed, snapshot_path, snapshot = snapshot_time_window(args.snapshot_dir)
            if snapshot_elapsed is not None:
                elapsed = snapshot_elapsed
            if snapshot_path is not None:
                snapshot_name = str(snapshot_path)

        planned_agents: list[dict[str, Any]] = []
        if source_uses_planned(args.source):
            planned_agents = evaluate_ghost_scenario(args.ghost_evaluator, args.ghost_scenario, elapsed)

        selected_track = selected_source_track_id(events_path, snapshot)

        if source_uses_planned(args.source):
            static_agents = [agent for agent in planned_agents if not agent.get("dynamic")]
            dynamic_agents = [agent for agent in planned_agents if agent.get("dynamic")]
            if args.clear and client is not None and dynamic_agents:
                client.simFlushPersistentMarkers()
                static_drawn = False
            if static_agents and not static_drawn:
                draw_agents(client, "planned_static", static_agents, selected_track, args, static_once=True)
                static_drawn = True
            draw_agents(client, "planned_dynamic", dynamic_agents, selected_track, args)
            if args.dry_run and not dynamic_agents and not args.follow:
                draw_agents(client, "planned_static", static_agents, selected_track, args, static_once=True)

        if source_uses_stream(args.source) or source_uses_artifacts(args.source):
            if snapshot is None:
                if args.source in {"world_snapshot", "artifact_snapshot"}:
                    if not waiting_reported:
                        source_description = f"stream {args.world_snapshot_stream_host}:{args.world_snapshot_stream_port}" if source_uses_stream(args.source) else f"snapshots under {args.snapshot_dir}"
                        print(f"airsim-world-overlay: waiting for WorldSnapshot from {source_description}", file=sys.stderr)
                        waiting_reported = True
                    if args.wait_for_world_model_s > 0.0 and time.time() - world_model_wait_start >= args.wait_for_world_model_s:
                        raise TimeoutError("timed out waiting for WorldSnapshot")
                    time.sleep(1.0 / args.rate_hz)
                    continue
                if args.source in {"combined", "artifact_combined"} and not waiting_reported:
                    source_description = f"stream {args.world_snapshot_stream_host}:{args.world_snapshot_stream_port}" if source_uses_stream(args.source) else f"snapshots under {args.snapshot_dir}"
                    print(f"airsim-world-overlay: drawing ghost scenario; waiting for WorldSnapshot from {source_description}", file=sys.stderr)
                    waiting_reported = True
            else:
                ready, reason = snapshot_ready(snapshot, required_tracks, args.allow_partial_tracks)
                if ready:
                    if not ready_reported:
                        print(f"airsim-world-overlay: WorldSnapshot ghost agents ready from {snapshot_name} ({reason})", file=sys.stderr)
                        ready_reported = True
                    world_agents = world_agents_to_draw(snapshot, args.max_agents, selected_track)
                    draw_agents(client, "world", world_agents, selected_track, args)
                elif args.source in {"world_snapshot", "artifact_snapshot"}:
                    if not waiting_reported:
                        print(f"airsim-world-overlay: waiting for ghost agents in WorldSnapshot ({reason})", file=sys.stderr)
                        waiting_reported = True
                    if args.wait_for_world_model_s > 0.0 and time.time() - world_model_wait_start >= args.wait_for_world_model_s:
                        raise TimeoutError(f"timed out waiting for ghost agents in WorldSnapshot: {reason}")
                    time.sleep(1.0 / args.rate_hz)
                    continue
                elif args.source in {"combined", "artifact_combined"} and not waiting_reported:
                    print(f"airsim-world-overlay: drawing ghost scenario; waiting for WorldSnapshot ghosts ({reason})", file=sys.stderr)
                    waiting_reported = True

        draw_reference_markers(client, snapshot, args)
        if args.debug:
            report = build_debug_report(snapshot_name, snapshot, planned_agents, world_agents, selected_track, elapsed, args, stream_seq)
            maybe_print_debug_report(report, args, debug_state)
            maybe_write_debug_json(report, args)

        if not args.follow:
            break
        if deadline is not None and time.time() >= deadline:
            break
        time.sleep(1.0 / args.rate_hz)

    if stream_client is not None:
        stream_client.close()
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
