#!/usr/bin/env python3
"""Bridge Dedalus camera_pointing_intent to MAVLink Gimbal Protocol v2.

Policy remains in C++:
  ObjectBehaviorMissionController -> mission_event camera_pointing_intent

Transport is target-specific:
  AirSim sink  -> simSetCameraPose(...)
  MAVLink sink -> GIMBAL_MANAGER_SET_PITCHYAW or MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW

The bridge intentionally talks to the Gimbal Manager, not directly to a Gimbal
Device. This matches MAVLink Gimbal Protocol v2, where applications control
through the manager so ownership and multiple control sources can be deconflicted.
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

from pymavlink import mavutil

DEFAULT_ENDPOINTS = "udpin:127.0.0.1:14550"
DEFAULT_SOURCE_SYSTEM = 255
DEFAULT_SOURCE_COMPONENT = int(getattr(mavutil.mavlink, "MAV_COMP_ID_ONBOARD_COMPUTER", 191))
MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW = int(getattr(mavutil.mavlink, "MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW", 1000))
MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE = int(getattr(mavutil.mavlink, "MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE", 1001))
MAV_CMD_REQUEST_MESSAGE = int(getattr(mavutil.mavlink, "MAV_CMD_REQUEST_MESSAGE", 512))
MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION = int(getattr(mavutil.mavlink, "MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION", 280))
MAV_COMP_ID_AUTOPILOT1 = int(getattr(mavutil.mavlink, "MAV_COMP_ID_AUTOPILOT1", 1))
MAV_RESULT_ACCEPTED = int(getattr(mavutil.mavlink, "MAV_RESULT_ACCEPTED", 0))


class RuntimeEventStreamClient:
    def __init__(self, host: str, port: int, reconnect_s: float = 1.0) -> None:
        self.host = host
        self.port = port
        self.reconnect_s = reconnect_s
        self.sock: socket.socket | None = None
        self.buffer = ""
        self.last_connect_attempt_s = 0.0
        self.reported_wait = False
        self.latest_camera_pointing_intent: dict[str, Any] | None = None
        self.latest_seq: int | None = None

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
            print(f"mavlink-gimbal-pointing-bridge: connected to runtime event stream {self.host}:{self.port}", file=sys.stderr, flush=True)
        except OSError as exc:
            if not self.reported_wait:
                print(f"mavlink-gimbal-pointing-bridge: waiting for runtime event stream {self.host}:{self.port} ({exc})", file=sys.stderr, flush=True)
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
            except OSError:
                self.close()
                break
            if not chunk:
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
            print(f"mavlink-gimbal-pointing-bridge: ignoring malformed JSONL ({exc})", file=sys.stderr, flush=True)
            return
        seq = message.get("seq")
        self.latest_seq = int(seq) if isinstance(seq, int) else self.latest_seq
        if message.get("type") != "mission_event":
            return
        event = message.get("mission_event")
        if isinstance(event, dict) and event.get("event") == "camera_pointing_intent":
            self.latest_camera_pointing_intent = event


def parse_endpoint_csv(raw: str) -> list[str]:
    return [item.strip() for item in raw.split(",") if item.strip()]


def float_or_none(value: Any) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def finite_or_nan(value: float | None) -> float:
    if value is None or not math.isfinite(value):
        return math.nan
    return value


def connect_mavlink(endpoints: list[str], source_system: int, source_component: int, timeout_s: float):
    last_error: Exception | None = None
    for endpoint in endpoints:
        mav = None
        try:
            print(f"mavlink-gimbal-pointing-bridge: trying MAVLink endpoint {endpoint}", file=sys.stderr, flush=True)
            mav = mavutil.mavlink_connection(endpoint, autoreconnect=True, source_system=source_system, source_component=source_component)
            heartbeat = mav.wait_heartbeat(timeout=timeout_s)
            if heartbeat is None:
                raise TimeoutError("no heartbeat")
            print(f"mavlink-gimbal-pointing-bridge: heartbeat target_system={mav.target_system} target_component={mav.target_component}", file=sys.stderr, flush=True)
            return mav
        except Exception as exc:  # noqa: BLE001
            last_error = exc
            print(f"mavlink-gimbal-pointing-bridge: endpoint failed {endpoint}: {exc}", file=sys.stderr, flush=True)
            try:
                if mav is not None:
                    mav.close()
            except Exception:
                pass
    raise RuntimeError(f"No usable MAVLink endpoint found. Last error: {last_error}")


def wait_command_ack(mav, command: int, timeout_s: float) -> dict[str, Any] | None:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        msg = mav.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.25)
        if msg is None or int(getattr(msg, "command", -1)) != command:
            continue
        result = int(getattr(msg, "result", -1))
        return {"command": command, "result": result, "accepted": result == MAV_RESULT_ACCEPTED}
    return None


def request_gimbal_manager_information(mav, target_component: int, gimbal_device_id: int, timeout_s: float) -> dict[str, Any] | None:
    mav.mav.command_long_send(mav.target_system, target_component, MAV_CMD_REQUEST_MESSAGE, 0, float(MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION), float(gimbal_device_id), 0, 0, 0, 0, 0)
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        msg = mav.recv_match(type="GIMBAL_MANAGER_INFORMATION", blocking=True, timeout=0.25)
        if msg is not None:
            return msg.to_dict()
    return None


def configure_gimbal_manager(mav, target_component: int, gimbal_device_id: int, *, acquire: bool, timeout_s: float) -> dict[str, Any] | None:
    primary = -2.0 if acquire else -3.0
    mav.mav.command_long_send(mav.target_system, target_component, MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE, 0, primary, primary, 0, 0, 0, 0, float(gimbal_device_id))
    return wait_command_ack(mav, MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE, timeout_s)


def send_gimbal_pitchyaw_message(mav, target_component: int, gimbal_device_id: int, flags: int, pitch_rad: float, yaw_rad: float | None, pitch_rate_rad_s: float | None, yaw_rate_rad_s: float | None) -> None:
    if not hasattr(mav.mav, "gimbal_manager_set_pitchyaw_send"):
        raise RuntimeError("pymavlink dialect does not expose gimbal_manager_set_pitchyaw_send")
    mav.mav.gimbal_manager_set_pitchyaw_send(mav.target_system, target_component, int(flags), int(gimbal_device_id), float(pitch_rad), finite_or_nan(yaw_rad), finite_or_nan(pitch_rate_rad_s), finite_or_nan(yaw_rate_rad_s))


def send_gimbal_pitchyaw_command(mav, target_component: int, gimbal_device_id: int, flags: int, pitch_rad: float, yaw_rad: float | None, pitch_rate_rad_s: float | None, yaw_rate_rad_s: float | None) -> None:
    mav.mav.command_long_send(mav.target_system, target_component, MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW, 0, math.degrees(float(pitch_rad)), math.degrees(finite_or_nan(yaw_rad)), math.degrees(finite_or_nan(pitch_rate_rad_s)), math.degrees(finite_or_nan(yaw_rate_rad_s)), float(flags), 0, float(gimbal_device_id))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stream-host", "--world-snapshot-stream-host", dest="stream_host", default="127.0.0.1")
    parser.add_argument("--stream-port", "--world-snapshot-stream-port", dest="stream_port", type=int, required=True)
    parser.add_argument("--mavlink-endpoints", default=DEFAULT_ENDPOINTS)
    parser.add_argument("--source-system", type=int, default=DEFAULT_SOURCE_SYSTEM)
    parser.add_argument("--source-component", type=int, default=DEFAULT_SOURCE_COMPONENT)
    parser.add_argument("--target-component", type=int, default=MAV_COMP_ID_AUTOPILOT1)
    parser.add_argument("--gimbal-device-id", type=int, default=0, help="0 targets all gimbals managed by the target component")
    parser.add_argument("--manager-flags", type=int, default=0)
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--resend-s", type=float, default=0.25)
    parser.add_argument("--deadband-deg", type=float, default=0.25)
    parser.add_argument("--duration-s", type=float, default=0.0, help="0 means run until Ctrl-C")
    parser.add_argument("--mode", choices=["message", "command"], default="message", help="message uses GIMBAL_MANAGER_SET_PITCHYAW; command uses MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW")
    parser.add_argument("--yaw-rad", type=float, default=None, help="Optional yaw setpoint. Default NaN/ignored.")
    parser.add_argument("--pitch-rate-rad-s", type=float, default=None, help="Optional pitch rate. Default NaN/ignored.")
    parser.add_argument("--yaw-rate-rad-s", type=float, default=None, help="Optional yaw rate. Default NaN/ignored.")
    parser.add_argument("--request-info", action="store_true", help="Request and print GIMBAL_MANAGER_INFORMATION at startup")
    parser.add_argument("--configure-primary", action="store_true", help="Acquire primary control using MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE")
    parser.add_argument("--release-on-exit", action="store_true", help="Release primary control on exit when --configure-primary was used")
    parser.add_argument("--ack-timeout-s", type=float, default=2.0)
    parser.add_argument("--heartbeat-timeout-s", type=float, default=4.0)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--debug-every-s", type=float, default=1.0)
    parser.add_argument("--debug-json", type=Path, default=None)
    return parser.parse_args()


def write_debug(path: Path | None, payload: dict[str, Any]) -> None:
    if path is None:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    if args.rate_hz <= 0.0:
        raise SystemExit("--rate-hz must be positive")
    if args.resend_s < 0.0:
        raise SystemExit("--resend-s must be >= 0")

    mav = None
    if not args.dry_run:
        mav = connect_mavlink(parse_endpoint_csv(args.mavlink_endpoints), args.source_system, args.source_component, args.heartbeat_timeout_s)
        if args.request_info:
            info = request_gimbal_manager_information(mav, args.target_component, args.gimbal_device_id, args.ack_timeout_s)
            print("mavlink-gimbal-pointing-bridge: gimbal_manager_information=" + json.dumps(info, sort_keys=True), file=sys.stderr, flush=True)
        if args.configure_primary:
            ack = configure_gimbal_manager(mav, args.target_component, args.gimbal_device_id, acquire=True, timeout_s=args.ack_timeout_s)
            print("mavlink-gimbal-pointing-bridge: configure_primary_ack=" + json.dumps(ack, sort_keys=True), file=sys.stderr, flush=True)

    stream = RuntimeEventStreamClient(args.stream_host, args.stream_port)
    period_s = 1.0 / args.rate_hz
    deadband_rad = math.radians(max(0.0, args.deadband_deg))
    start_s = time.monotonic()
    last_loop_s = 0.0
    last_send_s = 0.0
    last_debug_s = 0.0
    last_pitch_rad: float | None = None
    commands_sent = 0
    latest: dict[str, Any] = {"ok": False, "reason": "waiting_for_camera_pointing_intent", "commands_sent": 0}

    print(f"mavlink-gimbal-pointing-bridge: running mode={args.mode} target_component={args.target_component} gimbal_device_id={args.gimbal_device_id} dry_run={args.dry_run}", file=sys.stderr, flush=True)

    try:
        while True:
            now = time.monotonic()
            if args.duration_s > 0.0 and now - start_s >= args.duration_s:
                break
            stream.poll()
            intent = stream.latest_camera_pointing_intent
            if now - last_loop_s >= period_s:
                last_loop_s = now
                if intent is None:
                    latest = {"ok": False, "reason": "waiting_for_camera_pointing_intent", "commands_sent": commands_sent}
                else:
                    pitch_rad = float_or_none(intent.get("pitch_rad"))
                    if pitch_rad is None or not math.isfinite(pitch_rad):
                        latest = {"ok": False, "reason": "camera_pointing_intent_missing_pitch_rad", "commands_sent": commands_sent, "intent": intent}
                    else:
                        pitch_changed = last_pitch_rad is None or abs(pitch_rad - last_pitch_rad) >= deadband_rad
                        resend_due = args.resend_s > 0.0 and now - last_send_s >= args.resend_s
                        send = pitch_changed or resend_due
                        send_reason = "pitch_changed" if pitch_changed else ("resend" if resend_due else "deadband")
                        ack = None
                        if send:
                            if mav is not None:
                                if args.mode == "message":
                                    send_gimbal_pitchyaw_message(mav, args.target_component, args.gimbal_device_id, args.manager_flags, pitch_rad, args.yaw_rad, args.pitch_rate_rad_s, args.yaw_rate_rad_s)
                                else:
                                    send_gimbal_pitchyaw_command(mav, args.target_component, args.gimbal_device_id, args.manager_flags, pitch_rad, args.yaw_rad, args.pitch_rate_rad_s, args.yaw_rate_rad_s)
                                    ack = wait_command_ack(mav, MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW, args.ack_timeout_s)
                            last_pitch_rad = pitch_rad
                            last_send_s = now
                            commands_sent += 1
                        latest = {
                            "ok": True,
                            "command_sent": send,
                            "send_reason": send_reason,
                            "commands_sent": commands_sent,
                            "mode": args.mode,
                            "target_component": args.target_component,
                            "gimbal_device_id": args.gimbal_device_id,
                            "pitch_rad": pitch_rad,
                            "pitch_deg": math.degrees(pitch_rad),
                            "yaw_rad": args.yaw_rad,
                            "source_track_id": intent.get("source_track_id"),
                            "agent_id": intent.get("agent_id"),
                            "identity_id": intent.get("identity_id"),
                            "seq": stream.latest_seq,
                            "dry_run": args.dry_run,
                            "ack": ack,
                        }
            if args.debug and now - last_debug_s >= args.debug_every_s:
                last_debug_s = now
                if latest.get("ok"):
                    print(f"mavlink-gimbal-pointing-bridge: pitch={latest['pitch_deg']:.2f}deg sent={latest['command_sent']} reason={latest['send_reason']} count={latest['commands_sent']} mode={latest['mode']} gimbal_device_id={latest['gimbal_device_id']}", file=sys.stderr, flush=True)
                else:
                    print(f"mavlink-gimbal-pointing-bridge: {latest.get('reason', 'waiting')} count={latest.get('commands_sent', 0)}", file=sys.stderr, flush=True)
            write_debug(args.debug_json, latest)
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("mavlink-gimbal-pointing-bridge: interrupted", file=sys.stderr, flush=True)
        return 130
    finally:
        stream.close()
        if mav is not None and args.configure_primary and args.release_on_exit:
            ack = configure_gimbal_manager(mav, args.target_component, args.gimbal_device_id, acquire=False, timeout_s=args.ack_timeout_s)
            print("mavlink-gimbal-pointing-bridge: release_primary_ack=" + json.dumps(ack, sort_keys=True), file=sys.stderr, flush=True)
        try:
            if mav is not None:
                mav.close()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
