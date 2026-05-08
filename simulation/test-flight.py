#!/usr/bin/env python3
import airsim
import time
import argparse
import traceback


DEFAULT_VEHICLE = "PX4"


def fps_from_timestamp_delta(prev_ts, ts):
    if not prev_ts or ts <= prev_ts:
        return None
    dt = (ts - prev_ts) / 1e9
    if dt <= 0:
        return None
    return 1.0 / dt


def state_summary(state):
    k = state.kinematics_estimated
    q = k.orientation
    p = k.position
    gps = state.gps_location
    rc = state.rc_data
    return (
        f"ts={state.timestamp} "
        f"landed={state.landed_state} "
        f"pos=({p.x_val:.2f},{p.y_val:.2f},{p.z_val:.2f}) "
        f"q=({q.w_val:.3f},{q.x_val:.3f},{q.y_val:.3f},{q.z_val:.3f}) "
        f"state_gps=({gps.latitude},{gps.longitude},{gps.altitude}) "
        f"rc_valid={rc.is_valid}"
    )


def choose_vehicle(client, requested):
    vehicles = client.listVehicles()
    print(f"Vehicles: {vehicles}")

    if not vehicles:
        raise RuntimeError(
            "No AirSim vehicles found. Check ~/Documents/AirSim/settings.json "
            "and restart ./run.sh."
        )

    if requested in vehicles:
        return requested

    if requested:
        print(f"⚠️ Requested vehicle '{requested}' not found; using '{vehicles[0]}'")
    return vehicles[0]


def wait_for_airsim_state(client, vehicle_name, timeout_s):
    print(f"Waiting for AirSim state from '{vehicle_name}'...")
    deadline = time.time() + timeout_s
    last_ts = 0

    while time.time() < deadline:
        state = client.getMultirotorState(vehicle_name=vehicle_name)
        fps = fps_from_timestamp_delta(last_ts, state.timestamp)
        last_ts = state.timestamp

        if fps:
            print(f"{state_summary(state)} api_fps={fps:.2f}")
        else:
            print(state_summary(state))

        q = state.kinematics_estimated.orientation
        if state.timestamp > 0 and abs(q.w_val) > 0.1:
            print("✅ AirSim vehicle state is alive.")
            return state

        time.sleep(1)

    raise TimeoutError("Timed out waiting for timestamped AirSim vehicle state.")


def wait_for_gps_sensor(client, vehicle_name, timeout_s):
    print("Waiting for AirSim GPS sensor validity...")
    deadline = time.time() + timeout_s

    while time.time() < deadline:
        gps = client.getGpsData(vehicle_name=vehicle_name)
        fix = gps.gnss.fix_type
        point = gps.gnss.geo_point
        print(
            f"gps_valid={gps.is_valid} fix_type={fix} "
            f"lat={point.latitude} lon={point.longitude} alt={point.altitude}"
        )
        if gps.is_valid and fix >= 3:
            print("✅ AirSim GPS sensor is valid.")
            return gps
        time.sleep(1)

    raise TimeoutError(
        "Timed out waiting for valid AirSim GPS. If PX4 says 'waiting for GPS', "
        "also inspect the PX4 tmux window and logs/px4_*.log."
    )


def warmup_offboard(client, vehicle_name, count=8):
    print("Sending offboard warmup setpoints...")
    for i in range(count):
        client.moveByVelocityAsync(0, 0, 0, 0.25, vehicle_name=vehicle_name).join()
        print(f"  warmup {i + 1}/{count}")
        time.sleep(0.1)


def arm_with_retries(client, vehicle_name, attempts=3):
    for i in range(attempts):
        try:
            print(f"Arming attempt {i + 1}/{attempts}...")
            result = client.armDisarm(True, vehicle_name=vehicle_name)
            print(f"✅ armDisarm(True) returned: {result}")
            return True
        except Exception as exc:
            print(f"⚠️ armDisarm failed: {exc}")
            time.sleep(1)

    return False


def run_test_flight(vehicle_name, timeout_s, skip_arm):
    client = airsim.MultirotorClient()
    client.confirmConnection()

    v_name = choose_vehicle(client, vehicle_name)
    print(f"Using vehicle: {v_name}")

    try:
        wait_for_airsim_state(client, v_name, timeout_s)
        wait_for_gps_sensor(client, v_name, timeout_s)

        print(f"Requesting API control of '{v_name}'...")
        client.enableApiControl(True, vehicle_name=v_name)
        print(f"API control enabled: {client.isApiControlEnabled(vehicle_name=v_name)}")

        warmup_offboard(client, v_name)

        last_ts = 0
        print("Monitoring API timestamp-derived FPS...")

        start_wait = time.time()
        while time.time() - start_wait < 5:
            state = client.getMultirotorState(vehicle_name=v_name)
            fps = fps_from_timestamp_delta(last_ts, state.timestamp)
            if fps:
                print(f"API FPS: {fps:6.2f} | landed_state={state.landed_state}", end="\r")
            last_ts = state.timestamp
            time.sleep(0.1)
        print()

        print("Skipping arm RPC; testing direct velocity command...")
        client.moveByVelocityAsync(0, 0, -1, 5, vehicle_name=v_name).join()

    except Exception as e:
        print(f"\n❌ Test flight failed: {e}")
        traceback.print_exc()
    finally:
        try:
            client.armDisarm(False, vehicle_name=v_name)
        except Exception:
            pass
        try:
            client.enableApiControl(False, vehicle_name=v_name)
        except Exception:
            pass
        print("Test Concluded.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--vehicle", default=DEFAULT_VEHICLE)
    parser.add_argument("--timeout", type=int, default=30)
    parser.add_argument("--skip-arm", action="store_true")
    args = parser.parse_args()

    run_test_flight(
        vehicle_name=args.vehicle,
        timeout_s=args.timeout,
        skip_arm=args.skip_arm,
    )