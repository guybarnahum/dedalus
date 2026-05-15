#!/usr/bin/env python3
"""Persistent PX4 command bridge for Dedalus live missions.

This bridge intentionally follows the known-good control path in
simulation/test-flight.py:

  - PX4 shell for lifecycle commands: arm, takeoff, land, disarm
  - pymavlink connection created lazily after shell takeoff, before OFFBOARD
  - zero-velocity OFFBOARD priming before the first velocity command
  - MAV_CMD_DO_SET_MODE for PX4 OFFBOARD mode
  - LOCAL_POSITION_NED feedback climb to safe height
  - SET_POSITION_TARGET_LOCAL_NED for velocity setpoints

Protocol: JSON lines on stdin/stdout.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

from pymavlink import mavutil

SIMULATION_DIR = Path(__file__).resolve().parent
DEFAULT_ENDPOINTS = "udpin:127.0.0.1:14550"
DEFAULT_PX4_TMUX_TARGET = "dedalus-sim:px4"
DEFAULT_VERBOSITY = int(os.environ.get("DEDALUS_PX4_BRIDGE_VERBOSITY", "0"))


def log(message: str, *, level: int = 1) -> None:
    if DEFAULT_VERBOSITY < level:
        return
    print(f"px4-command-bridge: {message}", file=sys.stderr, flush=True)


def emit(response: dict[str, object]) -> None:
    print(json.dumps(response, separators=(",", ":")), flush=True)


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


def mavlink_get_local_position(mav, timeout_s: float = 2.0):
    msg = mav.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=timeout_s)
    if msg is None:
        raise TimeoutError("Timed out waiting for LOCAL_POSITION_NED sample")
    return msg


def mavlink_send_velocity_local_ned(mav, vx: float, vy: float, vz: float) -> None:
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


def climb_to_safe_height_mavlink(
    mav,
    safe_height_m: float,
    timeout_s: float = 20.0,
    min_margin_m: float = 0.35,
) -> float:
    target_z = -abs(safe_height_m)
    deadline = time.time() + timeout_s
    last_height = 0.0
    log(f"Climbing to safe height: {safe_height_m:.1f}m AGL target_z={target_z:.2f}")

    while time.time() < deadline:
        pos = mavlink_get_local_position(mav, timeout_s=1.0)
        last_height = -float(pos.z)
        remaining = float(pos.z) - target_z
        if remaining <= min_margin_m:
            log(f"Safe height reached: local_z={pos.z:.2f}, height≈{-pos.z:.2f}m")
            for _ in range(5):
                mavlink_send_velocity_local_ned(mav, 0.0, 0.0, 0.0)
                time.sleep(0.05)
            return max(0.0, last_height)

        climb_vz = -min(1.5, max(0.35, remaining * 0.4))
        mavlink_send_velocity_local_ned(mav, 0.0, 0.0, climb_vz)
        log(
            f"safe_height_progress local_z={pos.z:.2f} "
            f"height≈{-pos.z:.2f}m remaining={remaining:.2f}m vz={climb_vz:.2f}"
            , level=3)
        time.sleep(0.1)

    raise RuntimeError(
        f"Timed out climbing to safe height {safe_height_m:.1f}m; "
        f"last_height={last_height:.2f}m"
    )


class Px4CommandBridge:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.mav = None
        self.offboard_ready = False
        self.safe_height_reached = False

    def close(self) -> None:
        try:
            if self.mav is not None:
                self.mav.close()
                self.mav = None
        except Exception:
            pass

    def ensure_mavlink(self):
        if self.mav is None:
            self.mav = connect_mavlink(parse_endpoint_csv(self.args.mavlink_endpoints), timeout_s=self.args.mavlink_timeout)
        return self.mav

    def ensure_offboard(self) -> None:
        if self.offboard_ready:
            return
        mav = self.ensure_mavlink()
        prime_mavlink_offboard_velocity(
            mav,
            duration_s=self.args.offboard_prime_s,
            hz=self.args.offboard_prime_hz,
        )
        mavlink_set_px4_mode(mav, "OFFBOARD")
        self.offboard_ready = True

    def ensure_safe_height(self) -> None:
        if self.safe_height_reached or self.args.safe_height <= 0.0:
            return
        self.ensure_offboard()
        reached = climb_to_safe_height_mavlink(
            self.ensure_mavlink(),
            safe_height_m=self.args.safe_height,
            timeout_s=self.args.safe_height_timeout_s,
        )
        self.safe_height_reached = True
        log(f"safe_height_complete reached_height={reached:.2f}m")

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
            self.ensure_safe_height()
            vx = float(request.get("vx", 0.0))
            vy = float(request.get("vy", 0.0))
            vz = float(request.get("vz", 0.0))
            mavlink_send_velocity_local_ned(self.ensure_mavlink(), vx, vy, vz)
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
            self.safe_height_reached = False
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
    parser.add_argument("--safe-height", type=float, default=8.0)
    parser.add_argument("--safe-height-timeout-s", type=float, default=20.0)
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
        emit({"ok": True, "command": "ready", "status": "bridge ready"})
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                request = json.loads(line)
                response = bridge.handle(request)
            except Exception as exc:  # noqa: BLE001 - protocol should report errors as JSON.
                response = {"ok": False, "command": "error", "error": str(exc)}
            emit(response)
            if response.get("command") == "shutdown":
                break
        return 0
    finally:
        bridge.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:  # noqa: BLE001 - startup errors before protocol is ready.
        emit({"ok": False, "command": "startup", "error": str(exc)})
        log(f"startup failed: {exc}")
        raise SystemExit(1)