#!/usr/bin/env python3
"""Capture one AirSim ego-state sample and write flat JSON to stdout.

The C++ AirSimEgoStateProvider invokes this bridge command and parses stdout.
Keeping the AirSim Python dependency here avoids linking AirSim/rpclib into the
C++ core stack.
"""

from __future__ import annotations

import argparse
import json
import sys
import time

import airsim


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--rpc-port", type=int, default=41451)
    parser.add_argument("--vehicle-name", default="PX4")
    parser.add_argument("--camera-name", default="front_center")  # accepted for command-shape parity
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    client = airsim.MultirotorClient(ip=args.host, port=args.rpc_port)
    client.confirmConnection()

    state = client.getMultirotorState(vehicle_name=args.vehicle_name)
    pose = client.simGetVehiclePose(vehicle_name=args.vehicle_name)

    position = pose.position
    orientation = airsim.to_eularian_angles(pose.orientation)
    velocity = state.kinematics_estimated.linear_velocity
    angular_velocity = state.kinematics_estimated.angular_velocity

    payload = {
        "timestamp_ns": int(time.time_ns()),
        "position": [float(position.x_val), float(position.y_val), float(position.z_val)],
        "rotation_rpy": [float(orientation[0]), float(orientation[1]), float(orientation[2])],
        "velocity": [float(velocity.x_val), float(velocity.y_val), float(velocity.z_val)],
        "angular_velocity": [
            float(angular_velocity.x_val),
            float(angular_velocity.y_val),
            float(angular_velocity.z_val),
        ],
    }
    sys.stdout.write(json.dumps(payload, separators=(",", ":")) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
