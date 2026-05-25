#!/usr/bin/env python3
"""Bridge Dedalus runtime target selection to AirSim camera pitch.

This is the first AirSim sink for the 2.28C target-stare work. It subscribes to
Dedalus' runtime JSONL stream, tracks the latest selected target and world
snapshot, computes target elevation from ego -> selected target geometry, and
commands AirSim camera pitch via simSetCameraPose(...).

It intentionally does not command PX4 flight control and does not own behavior
semantics. Vehicle yaw remains a PX4 velocity/yaw concern; this bridge only
adapts camera/gimbal pitch for the AirSim simulation target.

The default AirSim pitch sign is -1 because AirSim/Unreal camera pitch commonly
uses negative values for looking down. Use --airsim-pitch-sign 1 if validation
against saved images shows the opposite sign for a given vehicle/camera.
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import time
from pathlib import Path
from typing import Any

try:
    import airsim
except ImportError as exc:
    raise SystemExit("airsim Python package is required: pip install airsim") from exc


class RuntimeEventStreamClient:
    def __init__(self, host: str, port: int, reconnect_s: float = 1.0) -> None:
        self.host = host
        self.port = port
        self.reconnect_s = reconnect_s
        self.sock: socket.socket | None = None
        self.buffer = ""
        self.last_connect_attempt_s = 0.0
        self.reported_wait = False
        self.latest_world_snapshot: dict[str, Any] | None = None
        self.latest_selected_target: dict[str, Any] | None = None
        self.latest_mission_event: dict[str, Any] | None = None
        self.latest_seq: int | None = None
        self.last_message_s: float | None = None

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
            print(
                f"airsim-camera-pointing-bridge: connected to runtime event stream {self.host}:{self.port}",
                file=sys.stderr,
            )
        except OSError as exc:
            if not self.reported_wait:
                print(
                    f"airsim-camera-pointing-bridge: waiting for runtime event stream {self.host}:{self.port} ({exc})",
                    file=sys.stderr,
                )
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
                print(
                    f"airsim-camera-pointing-bridge: runtime event stream closed; waiting for reconnect ({exc})",
                    file=sys.stderr,
                )
                self.close()
                break
            if not chunk:
                print("airsim-camera-pointing-bridge: runtime event stream closed; waiting for reconnect", file=sys.stderr)
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
            print(
                f"airsim-camera-pointing-bridge: ignoring malformed runtime event line ({exc})",
                file=sys.stderr,
            )
            return

        message_type = message.get("type")
        seq = message.get("seq")
        self.latest_seq = int(seq) if isinstance(seq, int) else self.latest_seq
        self.last_message_s = time.monotonic()

        if message_type == "world_snapshot":
            snapshot = message.get("snapshot")
            if isinstance(snapshot, dict):
                self.latest_world_snapshot = snapshot
        elif message_type == "mission_event":
            event = message.get("mission_event")
            if isinstance(event, dict):
                self.latest_mission_event = event
                if event.get("event") == "target_selected":
                    self.latest_selected_target = event


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stream-host", "--world-snapshot-stream-host", dest="stream_host", default="127.0.0.1")
    parser.add_argument("--stream-port", "--world-snapshot-stream-port", dest="stream_port", type=int, required=True)
    parser.add_argument("--host", default="127.0.0.1", help="AirSim RPC host")
    parser.add_argument("--rpc-port", type=int, default=41451, help="AirSim RPC port")
    parser.add_argument("--vehicle-name", default="PX4", help="AirSim vehicle name")
    parser.add_argument("--camera", default="front_center", help="AirSim camera name/id to command")
    parser.add_argument("--rate-hz", type=float, default=10.0, help="Maximum camera command rate")
    parser.add_argument("--duration-s", type=float, default=0.0, help="0 means run until Ctrl-C")
    parser.add_argument("--pitch-min-deg", type=float, default=-80.0)
    parser.add_argument("--pitch-max-deg", type=float, default=35.0)
    parser.add_argument("--pitch-offset-deg", type=float, default=0.0)
    parser.add_argument(
        "--airsim-pitch-sign",
        type=float,
        default=-1.0,
        choices=[-1.0, 1.0],
        help="Maps target elevation to AirSim pitch. Default -1 means positive target elevation commands negative camera pitch.",
    )
    parser.add_argument("--deadband-deg", type=float, default=0.25, help="Minimum pitch change before sending a new command")
    parser.add_argument("--dry-run", action="store_true", help="Compute pitch but do not call simSetCameraPose")
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--debug-every-s", type=float, default=1.0)
    parser.add_argument("--debug-json", type=Path, default=None)
    parser.add_argument("--wait-for-airsim-s", type=float, default=0.0, help="0 means wait until Ctrl-C")
    return parser.parse_args()


def vec3(value: Any) -> list[float] | None:
    if not isinstance(value, list) or len(value) != 3:
        return None
    try:
        return [float(value[0]), float(value[1]), float(value[2])]
    except Exception:
        return None


def clamp(value: float, lo: float, hi: float) -> float:
    return min(max(value, lo), hi)


def find_selected_agent(snapshot: dict[str, Any], selected: dict[str, Any]) -> dict[str, Any] | None:
    agents = snapshot.get("agents")
    if not isinstance(agents, list):
        return None

    source_track_id = selected.get("source_track_id")
    agent_id = selected.get("agent_id")
    identity_id = selected.get("identity_id")

    for agent in agents:
        if not isinstance(agent, dict):
            continue
        if source_track_id and agent.get("source_track_id") == source_track_id:
            return agent
        if agent_id and agent.get("agent_id") == agent_id:
            return agent
        if identity_id and agent.get("identity_id") == identity_id:
            return agent
    return None


def compute_pitch(
    *,
    snapshot: dict[str, Any],
    selected: dict[str, Any],
    pitch_sign: float,
    pitch_offset_rad: float,
    pitch_min_rad: float,
    pitch_max_rad: float,
) -> dict[str, Any] | None:
    ego = snapshot.get("ego")
    if not isinstance(ego, dict):
        return None
    ego_position = vec3(ego.get("position_local"))
    if ego_position is None:
        return None

    agent = find_selected_agent(snapshot, selected)
    if agent is None:
        return None
    target_position = vec3(agent.get("position_local"))
    if target_position is None:
        return None

    dx = target_position[0] - ego_position[0]
    dy = target_position[1] - ego_position[1]
    dz = target_position[2] - ego_position[2]
    range_xy = max(math.hypot(dx, dy), 1.0e-6)
    target_elevation_rad = math.atan2(dz, range_xy)
    unclamped_pitch_rad = pitch_sign * target_elevation_rad + pitch_offset_rad
    pitch_rad = clamp(unclamped_pitch_rad, pitch_min_rad, pitch_max_rad)

    return {
        "camera": None,
        "source_track_id": selected.get("source_track_id"),
        "agent_id": selected.get("agent_id"),
        "identity_id": selected.get("identity_id"),
        "ego_position_local": ego_position,
        "target_position_local": target_position,
        "range_xy_m": range_xy,
        "delta_z_m": dz,
        "target_elevation_rad": target_elevation_rad,
        "target_elevation_deg": math.degrees(target_elevation_rad),
        "pitch_unclamped_rad": unclamped_pitch_rad,
        "pitch_rad": pitch_rad,
        "pitch_deg": math.degrees(pitch_rad),
        "pitch_clamped": abs(pitch_rad - unclamped_pitch_rad) > 1.0e-9,
    }


def connect_airsim(host: str, rpc_port: int, wait_for_airsim_s: float) -> Any:
    start = time.monotonic()
    reported_wait = False
    while True:
        try:
            client = airsim.MultirotorClient(ip=host, port=rpc_port)
            client.confirmConnection()
            return client
        except Exception as exc:
            if wait_for_airsim_s > 0.0 and time.monotonic() - start >= wait_for_airsim_s:
                raise SystemExit(f"airsim-camera-pointing-bridge: AirSim RPC unavailable at {host}:{rpc_port}: {exc}") from exc
            if not reported_wait:
                print(
                    f"airsim-camera-pointing-bridge: waiting for AirSim RPC at {host}:{rpc_port} ({exc})",
                    file=sys.stderr,
                )
                reported_wait = True
            time.sleep(1.0)


def write_debug(path: Path | None, payload: dict[str, Any]) -> None:
    if path is None:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    if args.rate_hz <= 0.0:
        raise SystemExit("--rate-hz must be positive")
    if args.pitch_min_deg > args.pitch_max_deg:
        raise SystemExit("--pitch-min-deg must be <= --pitch-max-deg")

    client = None if args.dry_run else connect_airsim(args.host, args.rpc_port, args.wait_for_airsim_s)
    stream = RuntimeEventStreamClient(args.stream_host, args.stream_port)

    pitch_min_rad = math.radians(args.pitch_min_deg)
    pitch_max_rad = math.radians(args.pitch_max_deg)
    pitch_offset_rad = math.radians(args.pitch_offset_deg)
    deadband_rad = math.radians(max(0.0, args.deadband_deg))
    period_s = 1.0 / args.rate_hz
    start_s = time.monotonic()
    last_command_s = 0.0
    last_debug_s = 0.0
    last_pitch_rad: float | None = None
    commands_sent = 0

    print(
        "airsim-camera-pointing-bridge: running "
        f"camera={args.camera!r} stream={args.stream_host}:{args.stream_port} "
        f"airsim={args.host}:{args.rpc_port} dry_run={args.dry_run}",
        file=sys.stderr,
    )

    try:
        while True:
            now = time.monotonic()
            if args.duration_s > 0.0 and now - start_s >= args.duration_s:
                break

            stream.poll()
            snapshot = stream.latest_world_snapshot
            selected = stream.latest_selected_target
            latest: dict[str, Any] = {
                "ok": False,
                "reason": "waiting_for_world_snapshot_and_target_selected",
                "commands_sent": commands_sent,
            }

            if snapshot is not None and selected is not None and now - last_command_s >= period_s:
                pointing = compute_pitch(
                    snapshot=snapshot,
                    selected=selected,
                    pitch_sign=args.airsim_pitch_sign,
                    pitch_offset_rad=pitch_offset_rad,
                    pitch_min_rad=pitch_min_rad,
                    pitch_max_rad=pitch_max_rad,
                )
                if pointing is None:
                    latest = {
                        "ok": False,
                        "reason": "selected_target_not_found_in_world_snapshot",
                        "selected_target": selected,
                        "commands_sent": commands_sent,
                    }
                else:
                    pointing["camera"] = args.camera
                    pointing["vehicle_name"] = args.vehicle_name
                    pointing["dry_run"] = args.dry_run
                    pointing["seq"] = stream.latest_seq
                    send = last_pitch_rad is None or abs(pointing["pitch_rad"] - last_pitch_rad) >= deadband_rad
                    if send:
                        if client is not None:
                            pose = airsim.Pose(
                                airsim.Vector3r(0.0, 0.0, 0.0),
                                airsim.to_quaternion(float(pointing["pitch_rad"]), 0.0, 0.0),
                            )
                            client.simSetCameraPose(args.camera, pose, vehicle_name=args.vehicle_name)
                        last_pitch_rad = float(pointing["pitch_rad"])
                        last_command_s = now
                        commands_sent += 1
                    latest = {
                        "ok": True,
                        "command_sent": send,
                        "commands_sent": commands_sent,
                        **pointing,
                    }

            if args.debug and now - last_debug_s >= args.debug_every_s:
                last_debug_s = now
                if latest.get("ok"):
                    print(
                        "airsim-camera-pointing-bridge: "
                        f"pitch={latest['pitch_deg']:.2f}deg "
                        f"elev={latest['target_elevation_deg']:.2f}deg "
                        f"range_xy={latest['range_xy_m']:.2f}m "
                        f"sent={latest['command_sent']}",
                        file=sys.stderr,
                    )
                else:
                    print(
                        "airsim-camera-pointing-bridge: "
                        f"{latest.get('reason', 'waiting')}",
                        file=sys.stderr,
                    )
            write_debug(args.debug_json, latest)
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("airsim-camera-pointing-bridge: interrupted", file=sys.stderr)
        return 130
    finally:
        stream.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
