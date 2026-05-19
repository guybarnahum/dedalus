#!/usr/bin/env python3
"""Draw Dedalus WorldSnapshot agents into the AirSim/Unreal viewport.

Visualization-only helper. Reads mission artifacts and mirrors WorldSnapshot
agents into AirSim using debug plot primitives. It does not feed perception,
modify autonomy state, or write to binary bridge stdout.

The helper deliberately waits for two independent readiness signals:

1. AirSim RPC is reachable, so viewport/debug drawing can be injected.
2. Dedalus mission artifacts contain ghost/world-model agents, proving the
   mission loop has injected ghost detections into the perception/world-model
   path that drives behavior.

Ctrl-C exits either wait without changing mission state.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

try:
    import airsim
except ImportError as exc:
    raise SystemExit("airsim Python package is required for viewport overlay") from exc


SELECTED_COLOR = [0.0, 1.0, 0.2, 1.0]
AGENT_COLOR = [1.0, 0.85, 0.0, 1.0]
STALE_COLOR = [0.5, 0.5, 0.5, 1.0]
DEFAULT_REQUIRED_TRACKS = ["ghost_person_001", "ghost_person_002", "ghost_car_001"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot-dir", type=Path, required=True)
    parser.add_argument("--mission-events", type=Path, default=None)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--rpc-port", type=int, default=41451)
    parser.add_argument("--rate-hz", type=float, default=2.0)
    parser.add_argument("--duration-s", type=float, default=0.0)
    parser.add_argument("--follow", action="store_true")
    parser.add_argument("--clear", action="store_true")
    parser.add_argument("--persistent", action="store_true")
    parser.add_argument("--label", action="store_true")
    parser.add_argument("--max-agents", type=int, default=20)
    parser.add_argument("--z-lift-m", type=float, default=1.5)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument(
        "--wait-for-airsim-s",
        type=float,
        default=0.0,
        help="How long to wait for AirSim RPC. 0 means wait until Ctrl-C.",
    )
    parser.add_argument(
        "--wait-for-world-model-s",
        type=float,
        default=0.0,
        help="How long to wait for snapshots containing required tracks. 0 means wait until Ctrl-C.",
    )
    parser.add_argument(
        "--required-track",
        action="append",
        default=None,
        help="Track that must appear in WorldSnapshot.agents before drawing. Repeatable. Defaults to the ghost scenario tracks.",
    )
    parser.add_argument(
        "--allow-partial-tracks",
        action="store_true",
        help="Draw as soon as any agent exists instead of waiting for all required ghost tracks.",
    )
    return parser.parse_args()


def read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


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


def latest_existing_snapshot(snapshot_dir: Path) -> tuple[Path, dict[str, Any]] | None:
    for path in reversed(snapshot_paths_from_manifest(snapshot_dir)):
        if path.exists():
            return path, read_json(path)
    return None


def selected_source_track_id(events_path: Path) -> str | None:
    if not events_path.exists():
        return None

    selected: str | None = None
    for raw in events_path.read_text(encoding="utf-8").splitlines():
        if not raw.strip():
            continue
        try:
            event = json.loads(raw)
        except json.JSONDecodeError:
            continue
        if event.get("event") == "target_selected" and event.get("source_track_id"):
            selected = str(event["source_track_id"])
    return selected


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


def agents_to_draw(snapshot: dict[str, Any], max_agents: int, selected_track: str | None) -> list[dict[str, Any]]:
    agents = [agent for agent in snapshot.get("agents", []) if isinstance(agent, dict)]
    ego_position = snapshot.get("ego", {}).get("position_local", [0.0, 0.0, 0.0])
    agents.sort(
        key=lambda agent: (
            agent.get("source_track_id") != selected_track,
            distance_xy(agent.get("position_local"), ego_position),
        )
    )
    return agents[: max(0, max_agents)]


def vector3_from_position(position: Any, z_lift_m: float) -> airsim.Vector3r:
    if not isinstance(position, list) or len(position) != 3:
        raise ValueError(f"invalid position_local: {position!r}")

    # AirSim uses NED-like coordinates: +x forward/north, +y right/east, +z down.
    # A negative z offset lifts the marker visually above the agent.
    return airsim.Vector3r(float(position[0]), float(position[1]), float(position[2]) - z_lift_m)


def marker_color(agent: dict[str, Any], selected_track: str | None) -> list[float]:
    if selected_track and agent.get("source_track_id") == selected_track:
        return SELECTED_COLOR

    if str(agent.get("lifecycle", "active")) not in {"new", "active"}:
        return STALE_COLOR

    return AGENT_COLOR


def label_for(agent: dict[str, Any], selected_track: str | None) -> str:
    prefix = "SEL" if selected_track and agent.get("source_track_id") == selected_track else "AG"
    return (
        f"{prefix} "
        f"{agent.get('class', 'unknown')} "
        f"{agent.get('source_track_id', '')} "
        f"{float(agent.get('confidence', 0.0)):.2f}"
    )


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
        except Exception as exc:  # noqa: BLE001 - AirSim RPC throws several exception types.
            if not reported:
                print(
                    f"airsim-world-overlay: waiting for AirSim RPC at {args.host}:{args.rpc_port} ({exc})",
                    file=sys.stderr,
                )
                reported = True
            if args.wait_for_airsim_s > 0.0 and time.time() - start >= args.wait_for_airsim_s:
                raise TimeoutError(f"timed out waiting for AirSim RPC at {args.host}:{args.rpc_port}") from exc
            time.sleep(1.0 / max(args.rate_hz, 0.1))


def draw_snapshot(
    client: Any,
    snapshot_path: Path,
    snapshot: dict[str, Any],
    selected_track: str | None,
    args: argparse.Namespace,
) -> None:
    agents = agents_to_draw(snapshot, args.max_agents, selected_track)
    duration = 0.0 if args.persistent else max(0.5, 1.5 / max(args.rate_hz, 0.1))

    if args.clear and client is not None:
        client.simFlushPersistentMarkers()

    if args.dry_run:
        print(f"snapshot={snapshot_path.name} selected={selected_track or '-'} agents={len(agents)}")
        for agent in agents:
            print(f"  {label_for(agent, selected_track)} pos={agent.get('position_local')}")
        return

    for agent in agents:
        try:
            point = vector3_from_position(agent.get("position_local"), args.z_lift_m)
        except ValueError:
            continue

        color = marker_color(agent, selected_track)

        client.simPlotPoints(
            [point],
            color_rgba=color,
            size=20.0,
            duration=duration,
            is_persistent=args.persistent,
        )

        if args.label:
            label_point = airsim.Vector3r(point.x_val, point.y_val, point.z_val - 0.8)
            client.simPlotStrings(
                [label_for(agent, selected_track)],
                [label_point],
                scale=1.2,
                color_rgba=color,
                duration=duration,
            )


def main() -> int:
    args = parse_args()

    if args.rate_hz <= 0.0:
        raise ValueError("--rate-hz must be positive")

    required_tracks = args.required_track or DEFAULT_REQUIRED_TRACKS
    events_path = args.mission_events or (args.snapshot_dir / "mission_events.jsonl")
    client = connect_airsim(args)

    start = time.time()
    deadline = None if args.duration_s <= 0.0 else start + args.duration_s
    world_model_wait_start = time.time()
    wait_reported = False
    ready_reported = False

    while True:
        latest = latest_existing_snapshot(args.snapshot_dir)
        if latest is None:
            if not wait_reported:
                print(
                    f"airsim-world-overlay: waiting for Dedalus snapshots under {args.snapshot_dir}",
                    file=sys.stderr,
                )
                wait_reported = True
            if args.wait_for_world_model_s > 0.0 and time.time() - world_model_wait_start >= args.wait_for_world_model_s:
                raise TimeoutError(f"timed out waiting for Dedalus snapshots under {args.snapshot_dir}")
            time.sleep(1.0 / args.rate_hz)
            continue

        snapshot_path, snapshot = latest
        ready, reason = snapshot_ready(snapshot, required_tracks, args.allow_partial_tracks)
        if not ready:
            if not wait_reported:
                print(
                    f"airsim-world-overlay: waiting for ghost agents in WorldSnapshot ({reason})",
                    file=sys.stderr,
                )
                wait_reported = True
            if args.wait_for_world_model_s > 0.0 and time.time() - world_model_wait_start >= args.wait_for_world_model_s:
                raise TimeoutError(f"timed out waiting for ghost agents in WorldSnapshot: {reason}")
            time.sleep(1.0 / args.rate_hz)
            continue

        if not ready_reported:
            print(
                f"airsim-world-overlay: WorldSnapshot ghost agents ready from {snapshot_path.name} ({reason})",
                file=sys.stderr,
            )
            ready_reported = True

        selected_track = selected_source_track_id(events_path)
        draw_snapshot(client, snapshot_path, snapshot, selected_track, args)

        if not args.follow:
            break

        if deadline is not None and time.time() >= deadline:
            break

        time.sleep(1.0 / args.rate_hz)

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("airsim-world-overlay: interrupted", file=sys.stderr)
        raise SystemExit(130)
    except Exception as exc:
        print(f"airsim-world-overlay: {exc}", file=sys.stderr)
        raise SystemExit(1)
