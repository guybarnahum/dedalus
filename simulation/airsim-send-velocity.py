#!/usr/bin/env python3
"""Send one explicit flight command to AirSim/PX4.

This helper keeps the C++ core dependency-light: the C++ AirSimVelocityCommandSink
shells out to this script instead of linking the AirSim Python/RPC stack.

Mission lifecycle ownership lives in the mission controller:
- Prepare emits an explicit arm command.
- Takeoff emits an explicit takeoff command and waits for ego height.
- Flight states emit velocity commands.
- Complete emits an explicit disarm command.

For the PX4-backed Colosseum/AirSim setup, arming/disarming/takeoff are dispatched
through the PX4 shell because this AirSim server rejects the Python client's
armDisarm RPC with a vehicle-name argument and AirSim velocity OK is not vehicle
state truth. The mission state machine confirms the result asynchronously from
ego/PX4 telemetry in the world model.
"""

from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys
import time

import airsim

DEFAULT_MAVLINK_ENDPOINTS = "udpin:127.0.0.1:14550,udpin:127.0.0.1:14540,udpin:127.0.0.1:14600"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--command", choices=("arm", "takeoff", "disarm", "velocity"), default="velocity")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--rpc-port", type=int, default=41451)
    parser.add_argument("--vehicle-name", default="PX4")
    parser.add_argument("--vx", type=float, default=0.0)
    parser.add_argument("--vy", type=float, default=0.0)
    parser.add_argument("--vz", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=0.1)
    parser.add_argument("--yaw-rate", type=float, default=0.0)
    parser.add_argument("--no-join", action="store_true", help="Do not block waiting for the async AirSim command to complete.")
    parser.add_argument("--no-enable-api-control", action="store_true", help="Do not call enableApiControl(True) for velocity commands.")
    parser.add_argument(
        "--prearm-warmup-count",
        type=int,
        default=8,
        help="Number of zero-velocity AirSim setpoints to send before PX4 shell arm.",
    )
    parser.add_argument(
        "--prearm-warmup-duration",
        type=float,
        default=0.25,
        help="Duration in seconds for each pre-arm zero-velocity warmup setpoint.",
    )
    parser.add_argument(
        "--safe-height",
        type=float,
        default=float(os.environ.get("DEDALUS_FLIGHT_SAFE_HEIGHT_M", "0") or 0),
        help="If >0, climb to this AGL height with MAVLink OFFBOARD velocity after shell takeoff.",
    )
    parser.add_argument(
        "--takeoff-settle-s",
        type=float,
        default=6.0,
        help="Seconds to let PX4 shell takeoff establish climb before MAVLink safe-height control.",
    )
    parser.add_argument(
        "--mavlink-endpoints",
        default=os.environ.get("DEDALUS_MAVLINK_EGO_ENDPOINTS", DEFAULT_MAVLINK_ENDPOINTS),
        help="Comma-separated pymavlink endpoints for takeoff safe-height control.",
    )
    parser.add_argument("--px4-tmux-target", default="dedalus-sim:px4")
    return parser.parse_args()


def validate_finite(name: str, value: float) -> None:
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite, got {value!r}")


def px4_shell(target: str, command: str) -> None:
    subprocess.run(["tmux", "send-keys", "-t", target, command, "C-m"], check=True)


def make_client(args: argparse.Namespace) -> airsim.MultirotorClient:
    client = airsim.MultirotorClient(ip=args.host, port=args.rpc_port)
    client.confirmConnection()
    return client


def enable_api_control(client: airsim.MultirotorClient, vehicle_name: str) -> bool:
    client.enableApiControl(True, vehicle_name=vehicle_name)
    try:
        return bool(client.isApiControlEnabled(vehicle_name=vehicle_name))
    except Exception:  # noqa: BLE001 - older AirSim builds may not expose this reliably.
        return True


def prearm_warmup(client: airsim.MultirotorClient, vehicle_name: str, count: int, duration_s: float) -> None:
    for _ in range(max(0, count)):
        client.moveByVelocityAsync(0, 0, 0, duration_s, vehicle_name=vehicle_name).join()
        time.sleep(0.1)


def parse_mavlink_endpoints(raw: str) -> list[str]:
    return [item.strip() for item in raw.split(",") if item.strip()]


def connect_mavlink(endpoints: list[str], timeout_s: float = 4.0):
    try:
        from pymavlink import mavutil  # type: ignore
    except Exception as exc:  # pragma: no cover - sim host dependency
        raise RuntimeError(f"pymavlink is required for safe-height takeoff climb: {exc}") from exc

    last_error: Exception | None = None
    for endpoint in endpoints:
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
            return mav, mavutil
        except Exception as exc:  # noqa: BLE001 - try next endpoint.
            last_error = exc
            try:
                if mav is not None:
                    mav.close()
            except Exception:
                pass
    raise RuntimeError(f"No usable MAVLink endpoint found. Last error: {last_error}")


def mavlink_send_velocity_local_ned(mav, mavutil, vx: float, vy: float, vz: float) -> None:
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


def mavlink_set_px4_offboard(mav, mavutil, timeout_s: float = 3.0) -> None:
    mapping = mav.mode_mapping()
    if not mapping or "OFFBOARD" not in mapping:
        raise RuntimeError(f"PX4 OFFBOARD mode not found. Known modes: {mapping}")
    mode = mapping["OFFBOARD"]
    if isinstance(mode, tuple):
        base_mode, main_mode, sub_mode = mode[:3]
    else:
        base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        main_mode = mode
        sub_mode = 0

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
            if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                raise RuntimeError(f"PX4 OFFBOARD mode rejected with result={msg.result}")
            return


def mavlink_get_local_position(mav, timeout_s: float = 2.0):
    msg = mav.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=timeout_s)
    if msg is None:
        raise TimeoutError("Timed out waiting for LOCAL_POSITION_NED sample")
    return msg


def climb_to_safe_height_mavlink(safe_height_m: float, endpoints: list[str], timeout_s: float = 25.0) -> float:
    mav, mavutil = connect_mavlink(endpoints)
    try:
        end_prime = time.time() + 2.0
        while time.time() < end_prime:
            mavlink_send_velocity_local_ned(mav, mavutil, 0.0, 0.0, 0.0)
            time.sleep(0.05)
        mavlink_set_px4_offboard(mav, mavutil)

        target_z = -abs(safe_height_m)
        deadline = time.time() + timeout_s
        last_height = 0.0
        while time.time() < deadline:
            pos = mavlink_get_local_position(mav, timeout_s=1.0)
            last_height = max(0.0, -float(pos.z))
            remaining = float(pos.z) - target_z
            if remaining <= 0.35:
                for _ in range(5):
                    mavlink_send_velocity_local_ned(mav, mavutil, 0.0, 0.0, 0.0)
                    time.sleep(0.05)
                return last_height
            climb_vz = -min(1.5, max(0.35, remaining * 0.4))
            mavlink_send_velocity_local_ned(mav, mavutil, 0.0, 0.0, climb_vz)
            time.sleep(0.1)
        raise RuntimeError(
            f"Timed out climbing to safe height {safe_height_m:.2f}m; last_height={last_height:.2f}m"
        )
    finally:
        try:
            mav.close()
        except Exception:
            pass


def main() -> int:
    args = parse_args()
    validate_finite("vx", args.vx)
    validate_finite("vy", args.vy)
    validate_finite("vz", args.vz)
    validate_finite("duration", args.duration)
    validate_finite("yaw_rate", args.yaw_rate)
    validate_finite("prearm_warmup_duration", args.prearm_warmup_duration)
    validate_finite("safe_height", args.safe_height)
    validate_finite("takeoff_settle_s", args.takeoff_settle_s)
    if args.duration <= 0:
        raise ValueError("duration must be positive")
    if args.prearm_warmup_duration <= 0:
        raise ValueError("prearm warmup duration must be positive")
    if args.takeoff_settle_s < 0:
        raise ValueError("takeoff settle seconds must be non-negative")

    if args.command == "arm":
        client = make_client(args)
        api_control_enabled = enable_api_control(client, args.vehicle_name)
        prearm_warmup(
            client,
            args.vehicle_name,
            args.prearm_warmup_count,
            args.prearm_warmup_duration,
        )
        px4_shell(args.px4_tmux_target, "commander arm")
        print(
            f"OK command=arm dispatch=px4_shell target={args.px4_tmux_target} "
            f"api_control={api_control_enabled} warmup_count={args.prearm_warmup_count}"
        )
        return 0

    if args.command == "takeoff":
        client = make_client(args)
        api_control_enabled = enable_api_control(client, args.vehicle_name)
        px4_shell(args.px4_tmux_target, "commander takeoff")
        reached_height = 0.0
        if args.safe_height > 0.0:
            time.sleep(args.takeoff_settle_s)
            reached_height = climb_to_safe_height_mavlink(
                args.safe_height,
                parse_mavlink_endpoints(args.mavlink_endpoints),
            )
        print(
            f"OK command=takeoff dispatch=px4_shell target={args.px4_tmux_target} "
            f"api_control={api_control_enabled} safe_height={args.safe_height:.3f} "
            f"reached_height={reached_height:.3f}"
        )
        return 0

    if args.command == "disarm":
        px4_shell(args.px4_tmux_target, "commander disarm")
        print(f"OK command=disarm dispatch=px4_shell target={args.px4_tmux_target}")
        return 0

    client = make_client(args)

    api_control_enabled = False
    if not args.no_enable_api_control:
        api_control_enabled = enable_api_control(client, args.vehicle_name)

    task = client.moveByVelocityAsync(
        args.vx,
        args.vy,
        args.vz,
        args.duration,
        yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=args.yaw_rate),
        vehicle_name=args.vehicle_name,
    )
    if not args.no_join:
        task.join()

    print(
        "OK "
        f"command=velocity "
        f"vehicle={args.vehicle_name} "
        f"api_control={api_control_enabled} "
        f"velocity=({args.vx:.3f},{args.vy:.3f},{args.vz:.3f}) "
        f"duration={args.duration:.3f} "
        f"yaw_rate={args.yaw_rate:.3f}"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:  # noqa: BLE001 - CLI helper should print concise errors.
        print(f"airsim-send-velocity: {exc}", file=sys.stderr)
        raise SystemExit(1)