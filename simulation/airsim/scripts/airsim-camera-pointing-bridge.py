#!/usr/bin/env python3
"""Bridge Dedalus runtime target selection to AirSim camera pitch.

This is the first AirSim sink for the 2.28C target-stare work. It subscribes to
Dedalus' runtime JSONL stream, tracks the latest selected target and world
snapshot, computes target elevation from ego -> selected target geometry, and
commands AirSim camera pitch via simSetCameraPose(...).

It intentionally does not command PX4 flight control and does not own behavior
semantics. Vehicle yaw remains a PX4 velocity/yaw concern; this bridge only
adapts camera/gimbal pitch for the AirSim simulation target.

The external AirSim follow camera is not the commanded vehicle camera. To verify
camera pitch, use the front-camera image/sensor view or enable --capture-dir and
--verify-pose.
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
    parser.add_argument("--rate-hz", type=float, default=10.0, help="Maximum camera compute/send loop rate")
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
    parser.add_argument("--deadband-deg", type=float, default=0.25, help="Pitch-change threshold for immediate sends")
    parser.add_argument("--resend-s", type=float, default=0.25, help="Re-send current camera pitch at this cadence; 0 disables resend")
    parser.add_argument("--verify-pose", action="store_true", help="Read simGetCameraInfo after sends and report accepted pose")
    parser.add_argument("--capture-dir", type=Path, default=None, help="Optional directory for front-camera proof images")
    parser.add_argument("--capture-every-s", type=float, default=1.0, help="Minimum interval between proof image captures")
    parser.add_argument("--image-type", default="Scene", help="AirSim ImageType name for --capture-dir")
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


def image_type_value(name: str) -> int:
    return int(getattr(airsim.ImageType, name))


def quat_to_list(quat: Any) -> list[float]:
    return [float(quat.x_val), float(quat.y_val), float(quat.z_val), float(quat.w_val)]


def pose_pitch_rad(info: Any) -> float | None:
    try:
        angles = airsim.to_eularian_angles(info.pose.orientation)
        return float(angles[0])
    except Exception:
        return None


def write_image(path: Path, data: Any) -> bool:
    if data is None:
        return False
    if isinstance(data, str):
        if data == "":
            return False
        path.write_bytes(data.encode("latin1"))
        return True
    if isinstance(data, (bytes, bytearray)):
        if not data:
            return False
        path.write_bytes(bytes(data))
        return True
    return False


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
    if args.resend_s < 0.0:
        raise SystemExit("--resend-s must be >= 0")

    client = None if args.dry_run else connect_airsim(args.host, args.rpc_port, args.wait_for_airsim_s)
    stream = RuntimeEventStreamClient(args.stream_host, args.stream_port)

    pitch_min_rad = math.radians(args.pitch_min_deg)
    pitch_max_rad = math.radians(args.pitch_max_deg)
    pitch_offset_rad = math.radians(args.pitch_offset_deg)
    deadband_rad = math.radians(max(0.0, args.deadband_deg))
    period_s = 1.0 / args.rate_hz
    start_s = time.monotonic()
    last_loop_s = 0.0
    last_send_s = 0.0
    last_debug_s = 0.0
    last_capture_s = 0.0
    last_pitch_rad: float | None = None
    commands_sent = 0
    image_type = image_type_value(args.image_type)
    latest: dict[str, Any] = {
        "ok": False,
        "reason": "waiting_for_world_snapshot_and_target_selected",
        "commands_sent": commands_sent,
    }

    if args.capture_dir is not None:
        args.capture_dir.mkdir(parents=True, exist_ok=True)

    print(
        "airsim-camera-pointing-bridge: running "
        f"camera={args.camera!r} stream={args.stream_host}:{args.stream_port} "
        f"airsim={args.host}:{args.rpc_port} dry_run={args.dry_run} "
        f"resend_s={args.resend_s} verify_pose={args.verify_pose}",
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

            if now - last_loop_s >= period_s:
                last_loop_s = now
                if snapshot is None or selected is None:
                    latest = {
                        "ok": False,
                        "reason": "waiting_for_world_snapshot_and_target_selected",
                        "has_snapshot": snapshot is not None,
                        "has_target_selected": selected is not None,
                        "commands_sent": commands_sent,
                    }
                else:
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
                        pitch_changed = last_pitch_rad is None or abs(pointing["pitch_rad"] - last_pitch_rad) >= deadband_rad
                        resend_due = args.resend_s > 0.0 and now - last_send_s >= args.resend_s
                        send = pitch_changed or resend_due
                        verify: dict[str, Any] = {}
                        image_path: str | None = None
                        if send:
                            if client is not None:
                                pose = airsim.Pose(
                                    airsim.Vector3r(0.0, 0.0, 0.0),
                                    airsim.to_quaternion(float(pointing["pitch_rad"]), 0.0, 0.0),
                                )
                                client.simSetCameraPose(args.camera, pose, vehicle_name=args.vehicle_name)
                                if args.verify_pose:
                                    info = client.simGetCameraInfo(args.camera, vehicle_name=args.vehicle_name)
                                    accepted_pitch_rad = pose_pitch_rad(info)
                                    verify = {
                                        "camera_info_available": True,
                                        "accepted_orientation_quat_xyzw": quat_to_list(info.pose.orientation),
                                        "accepted_pitch_rad": accepted_pitch_rad,
                                        "accepted_pitch_deg": math.degrees(accepted_pitch_rad) if accepted_pitch_rad is not None else None,
                                    }
                                if args.capture_dir is not None and now - last_capture_s >= max(0.0, args.capture_every_s):
                                    image = client.simGetImage(args.camera, image_type, vehicle_name=args.vehicle_name)
                                    image_file = args.capture_dir / f"camera_pointing_{commands_sent:05d}_{pointing['pitch_deg']:+07.2f}.png"
                                    if write_image(image_file, image):
                                        image_path = str(image_file)
                                        last_capture_s = now
                            last_pitch_rad = float(pointing["pitch_rad"])
                            last_send_s = now
                            commands_sent += 1
                        latest = {
                            "ok": True,
                            "command_sent": send,
                            "send_reason": "pitch_changed" if pitch_changed else ("resend" if resend_due else "deadband"),
                            "commands_sent": commands_sent,
                            "image_path": image_path,
                            **pointing,
                            **verify,
                        }

            if args.debug and now - last_debug_s >= args.debug_every_s:
                last_debug_s = now
                if latest.get("ok"):
                    accepted = latest.get("accepted_pitch_deg")
                    accepted_text = "" if accepted is None else f" accepted={accepted:.2f}deg"
                    print(
                        "airsim-camera-pointing-bridge: "
                        f"pitch={latest['pitch_deg']:.2f}deg "
                        f"elev={latest['target_elevation_deg']:.2f}deg "
                        f"range_xy={latest['range_xy_m']:.2f}m "
                        f"sent={latest['command_sent']} "
                        f"reason={latest['send_reason']} "
                        f"count={latest['commands_sent']}"
                        f"{accepted_text}",
                        file=sys.stderr,
                    )
                else:
                    print(
                        "airsim-camera-pointing-bridge: "
                        f"{latest.get('reason', 'waiting')} "
                        f"snapshot={latest.get('has_snapshot')} target={latest.get('has_target_selected')}",
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
