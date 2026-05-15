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
import subprocess
import sys
import time

import airsim


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


def main() -> int:
    args = parse_args()
    validate_finite("vx", args.vx)
    validate_finite("vy", args.vy)
    validate_finite("vz", args.vz)
    validate_finite("duration", args.duration)
    validate_finite("yaw_rate", args.yaw_rate)
    validate_finite("prearm_warmup_duration", args.prearm_warmup_duration)
    if args.duration <= 0:
        raise ValueError("duration must be positive")
    if args.prearm_warmup_duration <= 0:
        raise ValueError("prearm warmup duration must be positive")

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
        print(
            f"OK command=takeoff dispatch=px4_shell target={args.px4_tmux_target} "
            f"api_control={api_control_enabled}"
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