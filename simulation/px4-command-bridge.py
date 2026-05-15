#!/usr/bin/env python3
"""Persistent PX4 command bridge for Dedalus live missions.

This bridge intentionally follows the known-good control path in
simulation/test-flight.py:

  - PX4 shell for lifecycle commands: arm, takeoff, land, disarm
  - pymavlink connection created with mavutil.mavlink_connection(...)
  - zero-velocity OFFBOARD priming before the first velocity command
  - MAV_CMD_DO_SET_MODE for PX4 OFFBOARD mode
  - SET_POSITION_TARGET_LOCAL_NED for velocity setpoints

Protocol: JSON lines on stdin/stdout.

Input examples:
  {"command":"arm"}
  {"command":"takeoff"}
  {"command":"velocity","vx":1.0,"vy":0.0,"vz":0.0,"duration":0.1}
  {"command":"land"}
  {"command":"disarm"}
  {"command":"shutdown"}

Output examples:
  {"ok":true,"command":"velocity","status":"sent"}
  {"ok":false,"command":"velocity","error":"..."}
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

from pymavlink import mavutil

SIMULATION_DIR = Path(__file__).resolve().parent
DEFAULT_ENDPOINTS = "udpin:127.0.0.1:14550,udpin:127.0.0.1:14540,udpin:127.0.0.1:14600"
DEFAULT_PX4_TMUX_TARGET = "dedalus-sim:px4"


def log(message: str) -> None:
    print(f"px4-command-bridge: {message}", file=sys.stderr, flush=True)


def parse_endpoint_csv(raw: str) -> list[str]:
    return [item.strip() for item in raw.split(",") if item.strip()]


def px4_shell(cmd: str, target: str) -> None:
    log(f"PX4 shell> {cmd}")
    subprocess.run(["tmux", "send-keys", "-t", target, cmd, "C-m"], check=True)


def connect_mavlink(endpoints: list[str], timeout_s: float = 4.0):
    last_error: Exception | None = None
    for endpoint in endpoints:
        log(f"Trying MAVLink endpoint: {endpoint}")
        mav = None
        try:
            mav = mavutil.mavlink_connection(
                endpoint,
                autoreconnect=True,
                source_system=255,
                source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
            )
            heartbeat = mav.wait_heartbeat(timeout=timeout_s)
            if heartbeat is None:
                raise TimeoutError("no heartbeat")
            log(
                "MAVLink heartbeat received "
                f"from system={mav.target_system}, component={mav.target_component}"
            )
            return mav
        except Exception as exc:  # noqa: BLE001 - try next endpoint.
            last_error = exc
            log(f"MAVLink endpoint failed: {endpoint}: {exc}")
            try:
                if mav is not None:
                    mav.close()
            except Exception:
                pass
    raise RuntimeError(f"No usable MAVLink endpoint found. Last error: {last_error}")


def mavlink_send_velocity_local_ned(mav, vx: float, vy: float, vz: float) -> None:
    """Send velocity-only local-NED setpoint exactly like test-flight.py."""
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )

    mav.mav.set_position_target_local_ned_send(
        int(time.time() * 1e3) & 0xFFFFFFFF,
        mav.target_system,
        mav.target_component or mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        0,
        0,
        0,
        vx,
        vy,
        vz,
        0,
        0,
        0,
        0,
        0,
    )


def prime_mavlink_offboard_velocity(mav, duration_s: float = 2.0, hz: float = 20.0) -> None:
    log("Priming PX4 Offboard velocity stream")
    dt = 1.0 / max(hz, 1.0)
    end = time.time() + duration_s
    while time.time() < end:
        mavlink_send_velocity_local_ned(mav, 0.0, 0.0, 0.0)
        time.sleep(dt)


def mavlink_set_px4_mode(mav, mode_name: str, timeout_s: float = 3.0) -> bool:
    """Set PX4 custom mode exactly like test-flight.py."""
    mapping = mav.mode_mapping()
    if not mapping or mode_name not in mapping:
        raise RuntimeError(f"PX4 mode '{mode_name}' not found. Known modes: {mapping}")

    mode = mapping[mode_name]
    if isinstance(mode, tuple):
        base_mode, main_mode, sub_mode = mode[:3]
    else:
        base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        main_mode = mode
        sub_mode = 0

    log(
        f"Setting PX4 mode {mode_name}: "
        f"base_mode={base_mode}, main_mode={main_mode}, sub_mode={sub_mode}"
    )

    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component or mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        float(base_mode),
        float(main_mode),
        float(sub_mode),
        0,
        0,
        0,
        0,
    )

    deadline = time.time() + timeout_s
    while time.time() < deadline:
        msg = mav.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
        if msg and msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            log(f"PX4 mode ACK: result={msg.result}")
            return msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED

    log("No PX4 mode ACK received; continuing and verifying by motion")
    return True


class Px4CommandBridge:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.mav = connect_mavlink(parse_endpoint_csv(args.mavlink_endpoints), timeout_s=args.mavlink_timeout)
        self.offboard_ready = False

    def close(self) -> None:
        try:
            if self.mav is not None:
                self.mav.close()
        except Exception:
            pass

    def ensure_offboard(self) -> None:
        if self.offboard_ready:
            return
        prime_mavlink_offboard_velocity(
            self.mav,
            duration_s=self.args.offboard_prime_s,
            hz=self.args.offboard_prime_hz,
        )
        mavlink_set_px4_mode(self.mav, "OFFBOARD")
        self.offboard_ready = True

    def handle(self, request: dict[str, object]) -> dict[str, object]:
        command = str(request.get("command", ""))
        if command == "arm":
            px4_shell("commander arm", self.args.px4_tmux_target)
            time.sleep(self.args.arm_settle_s)
            return {"ok": True, "command": command, "status": "px4_shell arm"}
        if command == "takeoff":
            px4_shell("commander takeoff", self.args.px4_tmux_target)
            time.sleep(self.args.takeoff_settle_s)
            return {"ok": True, "command": command, "status": "px4_shell takeoff"}
        if command == "velocity":
            self.ensure_offboard()
            vx = float(request.get("vx", 0.0))
            vy = float(request.get("vy", 0.0))
            vz = float(request.get("vz", 0.0))
            mavlink_send_velocity_local_ned(self.mav, vx, vy, vz)
            return {
                "ok": True,
                "command": command,
                "status": "mavlink velocity",
                "vx": vx,
                "vy": vy,
                "vz": vz,
            }
        if command == "land":
            px4_shell("commander land", self.args.px4_tmux_target)
            time.sleep(self.args.land_settle_s)
            self.offboard_ready = False
            return {"ok": True, "command": command, "status": "px4_shell land"}
        if command == "disarm":
            px4_shell("commander disarm", self.args.px4_tmux_target)
            return {"ok": True, "command": command, "status": "px4_shell disarm"}
        if command == "shutdown":
            return {"ok": True, "command": command, "status": "shutdown"}
        raise ValueError(f"unknown command: {command}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mavlink-endpoints", default=DEFAULT_ENDPOINTS)
    parser.add_argument("--mavlink-timeout", type=float, default=4.0)
    parser.add_argument("--px4-tmux-target", default=DEFAULT_PX4_TMUX_TARGET)
    parser.add_argument("--offboard-prime-s", type=float, default=2.0)
    parser.add_argument("--offboard-prime-hz", type=float, default=20.0)
    parser.add_argument("--arm-settle-s", type=float, default=2.0)
    parser.add_argument("--takeoff-settle-s", type=float, default=8.0)
    parser.add_argument("--land-settle-s", type=float, default=8.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    bridge = Px4CommandBridge(args)
    try:
        print(json.dumps({"ok": True, "command": "ready", "status": "bridge ready"}), flush=True)
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                request = json.loads(line)
                response = bridge.handle(request)
            except Exception as exc:  # noqa: BLE001 - protocol should report errors as JSON.
                response = {"ok": False, "command": "error", "error": str(exc)}
            print(json.dumps(response, separators=(",", ":")), flush=True)
            if response.get("command") == "shutdown":
                break
        return 0
    finally:
        bridge.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:  # noqa: BLE001 - startup errors before protocol is ready.
        print(json.dumps({"ok": False, "command": "startup", "error": str(exc)}), flush=True)
        log(f"startup failed: {exc}")
        raise SystemExit(1)
