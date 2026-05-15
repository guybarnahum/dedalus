#!/usr/bin/env python3
"""Send one bounded velocity command to AirSim.

This helper keeps the C++ core dependency-light: the C++ AirSimVelocityCommandSink
shells out to this script instead of linking the AirSim Python/RPC stack.
"""

from __future__ import annotations

import argparse
import math
import sys

import airsim


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--rpc-port", type=int, default=41451)
    parser.add_argument("--vehicle-name", default="PX4")
    parser.add_argument("--vx", type=float, required=True)
    parser.add_argument("--vy", type=float, required=True)
    parser.add_argument("--vz", type=float, required=True)
    parser.add_argument("--duration", type=float, default=0.1)
    parser.add_argument("--yaw-rate", type=float, default=0.0)
    parser.add_argument("--no-join", action="store_true", help="Do not block waiting for the async AirSim command to complete.")
    return parser.parse_args()


def validate_finite(name: str, value: float) -> None:
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite, got {value!r}")


def main() -> int:
    args = parse_args()
    validate_finite("vx", args.vx)
    validate_finite("vy", args.vy)
    validate_finite("vz", args.vz)
    validate_finite("duration", args.duration)
    validate_finite("yaw_rate", args.yaw_rate)
    if args.duration <= 0:
        raise ValueError("duration must be positive")

    client = airsim.MultirotorClient(ip=args.host, port=args.rpc_port)
    client.confirmConnection()

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
        f"vehicle={args.vehicle_name} "
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
