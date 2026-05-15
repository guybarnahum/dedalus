#!/usr/bin/env python3
"""Prepare a deterministic AirSim/PX4 session before mission arming.

This mirrors the known-good startup sequence from simulation/test-flight.py:
  1. connect to AirSim
  2. confirm the requested vehicle exists
  3. wait for timestamped AirSim state
  4. wait for valid GPS
  5. enable API control
  6. send zero-velocity warmup setpoints
  7. verify MAVLink heartbeat and LOCAL_POSITION_NED

The helper intentionally does not arm. MissionRuntime/TrajectoryMissionController
still own the mission lifecycle; this script only makes the simulator/PX4 session
ready for the first Arm command.
"""

from __future__ import annotations

import argparse
import sys
import time

import airsim
from pymavlink import mavutil

DEFAULT_ENDPOINTS = "udpin:127.0.0.1:14550,udpin:127.0.0.1:14540,udpin:127.0.0.1:14600"


def parse_endpoints(raw: str) -> list[str]:
    return [item.strip() for item in raw.split(",") if item.strip()]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--rpc-port", type=int, default=41451)
    parser.add_argument("--vehicle-name", default="PX4")
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--warmup-count", type=int, default=8)
    parser.add_argument("--warmup-duration", type=float, default=0.25)
    parser.add_argument("--mavlink-endpoints", default=DEFAULT_ENDPOINTS)
    return parser.parse_args()


def choose_vehicle(client: airsim.MultirotorClient, requested: str) -> str:
    vehicles = client.listVehicles()
    if not vehicles:
        raise RuntimeError("No AirSim vehicles found")
    if requested in vehicles:
        return requested
    return vehicles[0]


def wait_for_state(client: airsim.MultirotorClient, vehicle_name: str, timeout_s: float) -> None:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        state = client.getMultirotorState(vehicle_name=vehicle_name)
        q = state.kinematics_estimated.orientation
        if state.timestamp > 0 and abs(q.w_val) > 0.1:
            return
        time.sleep(0.5)
    raise TimeoutError("Timed out waiting for timestamped AirSim state")


def wait_for_gps(client: airsim.MultirotorClient, vehicle_name: str, timeout_s: float) -> None:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        gps = client.getGpsData(vehicle_name=vehicle_name)
        if gps.is_valid and gps.gnss.fix_type >= 3:
            return
        time.sleep(0.5)
    raise TimeoutError("Timed out waiting for valid AirSim GPS")


def warmup_offboard(client: airsim.MultirotorClient, vehicle_name: str, count: int, duration_s: float) -> None:
    for _ in range(max(0, count)):
        client.moveByVelocityAsync(0, 0, 0, duration_s, vehicle_name=vehicle_name).join()
        time.sleep(0.1)


def verify_mavlink(endpoints: list[str], timeout_s: float) -> str:
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
            pos = mav.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=timeout_s)
            if pos is None:
                raise TimeoutError("no LOCAL_POSITION_NED")
            return endpoint
        except Exception as exc:  # noqa: BLE001 - try the next endpoint.
            last_error = exc
        finally:
            try:
                if mav is not None:
                    mav.close()
            except Exception:
                pass
    raise RuntimeError(f"No usable MAVLink endpoint found. Last error: {last_error}")


def main() -> int:
    args = parse_args()
    client = airsim.MultirotorClient(ip=args.host, port=args.rpc_port)
    client.confirmConnection()
    vehicle_name = choose_vehicle(client, args.vehicle_name)
    wait_for_state(client, vehicle_name, args.timeout)
    wait_for_gps(client, vehicle_name, args.timeout)
    client.enableApiControl(True, vehicle_name=vehicle_name)
    api_control = client.isApiControlEnabled(vehicle_name=vehicle_name)
    warmup_offboard(client, vehicle_name, args.warmup_count, args.warmup_duration)
    endpoint = verify_mavlink(parse_endpoints(args.mavlink_endpoints), timeout_s=min(args.timeout, 5.0))
    print(
        "OK prepare_session "
        f"vehicle={vehicle_name} api_control={api_control} "
        f"warmup_count={args.warmup_count} mavlink_endpoint={endpoint}"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:  # noqa: BLE001 - concise CLI helper failure.
        print(f"airsim-prepare-session: {exc}", file=sys.stderr)
        raise SystemExit(1)
