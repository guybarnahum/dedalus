#!/usr/bin/env python3
import airsim
import time
import argparse
import traceback
import subprocess
import json
import math
from pathlib import Path

from pymavlink import mavutil


DEFAULT_VEHICLE = "PX4"
DEFAULT_PX4_TMUX_TARGET = "dedalus-sim:px4"

DEFAULT_TRAJECTORY_DICT = {
    "name": "default_takeoff_hover_land",
    "description": "Simple takeoff, hover, land sequence.",
    "rate_hz": 10,
    "segments": [
        {
            "type": "hold",
            "label": "hover",
            "duration_s": 10,
            "vx_mps": 0.0,
            "vy_mps": 0.0,
            "vz_mps": 0.0,
        }
    ],
}

DEFAULT_MAVLINK_ENDPOINTS = [
    "udpin:127.0.0.1:14550",
    "udpin:127.0.0.1:14540",
    "udpin:127.0.0.1:14600",
]


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


def sleep_with_status(seconds, label):
    print(label)
    time.sleep(seconds)


def px4_shell(cmd, target=DEFAULT_PX4_TMUX_TARGET):
    print(f"PX4 shell> {cmd}")
    subprocess.run(
        ["tmux", "send-keys", "-t", target, cmd, "C-m"],
        check=True,
    )


def arm_via_px4_shell(target):
    try:
        px4_shell("commander arm", target)
        time.sleep(2)
        return True
    except Exception as exc:
        print(f"⚠️ PX4 shell arm fallback failed: {exc}")
        return False


def disarm_via_px4_shell(target):
    try:
        px4_shell("commander disarm", target)
    except Exception:
        pass


def px4_takeoff_via_shell(target):
    px4_shell("commander takeoff", target)
    time.sleep(8)


def px4_land_via_shell(target):
    try:
        px4_shell("commander land", target)
    except Exception:
        pass


def validate_trajectory_file(path):
    """
    Validate that a trajectory file is valid JSON and contains 'segments'.
    Returns the path if valid; raises argparse.ArgumentTypeError if not.
    """
    trajectory_path = Path(path)
    if not trajectory_path.is_absolute():
        trajectory_path = Path(__file__).resolve().parent / trajectory_path

    if not trajectory_path.exists():
        raise argparse.ArgumentTypeError(f"Trajectory file not found: {trajectory_path}")

    try:
        with trajectory_path.open("r", encoding="utf-8") as f:
            data = json.load(f)
        if "segments" not in data:
            raise argparse.ArgumentTypeError(
                f"Trajectory file missing 'segments' key: {trajectory_path}"
            )
        return path
    except json.JSONDecodeError as e:
        raise argparse.ArgumentTypeError(
            f"Invalid JSON in trajectory file {trajectory_path}: {e}"
        )
    except Exception as e:
        raise argparse.ArgumentTypeError(f"Error reading trajectory file {trajectory_path}: {e}")


def load_trajectory(path):
    # If path is None or empty, use the default hardcoded trajectory.
    if not path:
        return DEFAULT_TRAJECTORY_DICT

    # Otherwise load from file.
    if not path.is_absolute():
        path = Path(__file__).resolve().parent / path

    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    if "segments" not in data:
        raise ValueError(f"Trajectory file missing 'segments': {path}")

    return data


def lerp(a, b, u):
    return a + (b - a) * u


def interpolate_keyframes(keyframes, t):
    if not keyframes:
        return 0.0, 0.0, 0.0

    if t <= keyframes[0]["t"]:
        k = keyframes[0]
        return k.get("vx_mps", 0.0), k.get("vy_mps", 0.0), k.get("vz_mps", 0.0)

    if t >= keyframes[-1]["t"]:
        k = keyframes[-1]
        return k.get("vx_mps", 0.0), k.get("vy_mps", 0.0), k.get("vz_mps", 0.0)

    for i in range(len(keyframes) - 1):
        a = keyframes[i]
        b = keyframes[i + 1]
        if a["t"] <= t <= b["t"]:
            span = max(b["t"] - a["t"], 1e-6)
            u = (t - a["t"]) / span
            return (
                lerp(a.get("vx_mps", 0.0), b.get("vx_mps", 0.0), u),
                lerp(a.get("vy_mps", 0.0), b.get("vy_mps", 0.0), u),
                lerp(a.get("vz_mps", 0.0), b.get("vz_mps", 0.0), u),
            )

    k = keyframes[-1]
    return k.get("vx_mps", 0.0), k.get("vy_mps", 0.0), k.get("vz_mps", 0.0)


def segment_velocity(segment, t):
    typ = segment.get("type")

    if typ == "hold":
        return (
            segment.get("vx_mps", 0.0),
            segment.get("vy_mps", 0.0),
            segment.get("vz_mps", 0.0),
        )

    if typ == "velocity_keyframes":
        return interpolate_keyframes(segment["keyframes"], t)

    if typ == "circle_velocity":
        duration = float(segment["duration_s"])
        speed = float(segment.get("speed_mps", 2.0))
        radius = float(segment.get("radius_m", 10.0))
        direction = segment.get("direction", "cw").lower()
        vz = float(segment.get("vz_mps", 0.0))

        omega = speed / max(radius, 1e-6)
        theta = omega * t
        sign = -1.0 if direction == "cw" else 1.0

        # NED horizontal velocity tangent to a circle.
        vx = speed * math.cos(theta)
        vy = sign * speed * math.sin(theta)

        # Make duration roughly match one full loop if user did not tune it exactly.
        _ = duration
        return vx, vy, vz

    if typ == "figure8_velocity":
        duration = float(segment["duration_s"])
        speed = float(segment.get("speed_mps", 2.0))
        scale = float(segment.get("scale_m", 10.0))
        vz = float(segment.get("vz_mps", 0.0))

        # Lemniscate-inspired velocity field.
        # x = A sin(theta), y = A sin(theta) cos(theta)
        # We scale derivatives so the speed remains near requested speed.
        theta = 2.0 * math.pi * t / max(duration, 1e-6)
        dx = scale * math.cos(theta)
        dy = scale * math.cos(2.0 * theta)
        norm = max(math.hypot(dx, dy), 1e-6)

        vx = speed * dx / norm
        vy = speed * dy / norm
        return vx, vy, vz

    raise ValueError(f"Unknown trajectory segment type: {typ}")


def play_velocity_trajectory(client, vehicle_name, trajectory_path):
    trajectory = load_trajectory(trajectory_path)
    rate_hz = float(trajectory.get("rate_hz", 10))
    dt = 1.0 / max(rate_hz, 1.0)

    print(f"Playing trajectory: {trajectory.get('name', trajectory_path)}")
    print(f"Description: {trajectory.get('description', '')}")
    print(f"Rate: {rate_hz:.1f} Hz")

    for seg_idx, segment in enumerate(trajectory["segments"], start=1):
        label = segment.get("label", segment.get("type", f"segment-{seg_idx}"))
        typ = segment.get("type")

        if typ == "velocity_keyframes":
            duration = float(segment["keyframes"][-1]["t"])
        else:
            duration = float(segment.get("duration_s", 0))

        if duration <= 0:
            print(f"Skipping empty segment: {label}")
            continue

        print(f"\nSegment {seg_idx}: {label} ({typ}, {duration:.1f}s)")
        start = time.time()
        next_tick = start

        while True:
            now = time.time()
            t = now - start
            if t >= duration:
                break

            vx, vy, vz = segment_velocity(segment, t)
            client.moveByVelocityAsync(vx, vy, vz, dt, vehicle_name=vehicle_name).join()

            print(
                f"\r  t={t:5.1f}/{duration:5.1f}s "
                f"v=({vx:+.2f},{vy:+.2f},{vz:+.2f}) m/s",
                end="",
                flush=True,
            )

            next_tick += dt
            sleep_s = next_tick - time.time()
            if sleep_s > 0:
                time.sleep(sleep_s)

        print()

    print("\nTrajectory complete. Sending zero velocity settle command.")
    client.moveByVelocityAsync(0, 0, 0, 2, vehicle_name=vehicle_name).join()


def px4_control_sequence(client, vehicle_name, target, trajectory_path):
    print("PX4 shell control sequence: arm → takeoff → velocity trajectory → land")
    px4_shell("commander arm", target)
    time.sleep(2)
    px4_shell("commander takeoff", target)
    print("Waiting for PX4 takeoff climb...")
    time.sleep(8)
    play_velocity_trajectory(client, vehicle_name, trajectory_path)
    px4_shell("commander land", target)
    time.sleep(8)
    disarm_via_px4_shell(target)


def parse_mavlink_endpoints(raw):
    endpoints = []
    for item in raw:
        for endpoint in item.split(","):
            endpoint = endpoint.strip()
            if endpoint:
                endpoints.append(endpoint)
    return endpoints


def connect_mavlink(endpoints, timeout_s=4):
    last_error = None

    for endpoint in endpoints:
        print(f"Trying MAVLink endpoint: {endpoint}")
        mav = None

        try:
            mav = mavutil.mavlink_connection(
                endpoint,
                autoreconnect=True,
                source_system=255,
                source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
            )

            hb = mav.wait_heartbeat(timeout=timeout_s)
            if hb is None:
                raise TimeoutError("no heartbeat")

            print(
                "✅ MAVLink heartbeat received "
                f"from system={mav.target_system}, component={mav.target_component}"
            )
            return mav

        except Exception as exc:
            last_error = exc
            print(f"⚠️ MAVLink endpoint failed: {endpoint}: {exc}")
            try:
                if mav:
                    mav.close()
            except Exception:
                pass

    raise RuntimeError(f"No usable MAVLink endpoint found. Last error: {last_error}")


def wait_for_command_ack(mav, command, timeout_s=5):
    deadline = time.time() + timeout_s

    while time.time() < deadline:
        msg = mav.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
        if msg is None:
            continue

        if msg.command != command:
            continue

        result = msg.result
        result_name = mavutil.mavlink.enums["MAV_RESULT"].get(result)
        result_text = result_name.name if result_name else str(result)
        print(f"MAVLink COMMAND_ACK command={command} result={result_text}")
        return result

    raise TimeoutError(f"Timed out waiting for COMMAND_ACK for command={command}")


def mavlink_arm(mav, force=False):
    param2 = 21196 if force else 0

    print(f"Sending MAVLink arm command force={force}...")
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component or mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        param2,
        0,
        0,
        0,
        0,
        0,
    )

    result = wait_for_command_ack(
        mav,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    )

    if result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        raise RuntimeError(f"MAVLink arm rejected with MAV_RESULT={result}")

    print("✅ MAVLink arm accepted.")
    return True


def mavlink_disarm(mav, force=False):
    param2 = 21196 if force else 0

    try:
        print(f"Sending MAVLink disarm command force={force}...")
        mav.mav.command_long_send(
            mav.target_system,
            mav.target_component or mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,
            param2,
            0,
            0,
            0,
            0,
            0,
        )
        wait_for_command_ack(
            mav,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            timeout_s=3,
        )
    except Exception as exc:
        print(f"⚠️ MAVLink disarm failed/ignored: {exc}")


def mavlink_takeoff(mav, relative_alt_m=3.0):
    """
    Use PX4's normal takeoff command instead of OFFBOARD.

    This matches the working PX4 shell path more closely:
      commander arm
      commander takeoff

    AirSim/PX4 Offboard mode is more sensitive and our previous set_mode()
    call was invalid for PX4 because pymavlink returned tuple mode mappings.
    """
    print(f"Sending MAVLink NAV_TAKEOFF relative_alt_m={relative_alt_m}...")
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component or mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        float("nan"),
        0,
        0,
        relative_alt_m,
    )

    result = wait_for_command_ack(mav, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)
    if result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        raise RuntimeError(f"MAVLink takeoff rejected with MAV_RESULT={result}")


def mavlink_land(mav):
    print("Sending MAVLink NAV_LAND...")
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component or mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0,
        0,
        0,
        float("nan"),
        0,
        0,
        0,
    )

    result = wait_for_command_ack(mav, mavutil.mavlink.MAV_CMD_NAV_LAND)
    if result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        raise RuntimeError(f"MAVLink land rejected with MAV_RESULT={result}")


def mavlink_wait_local_position(mav, timeout_s=10):
    print("Waiting for MAVLink LOCAL_POSITION_NED...")
    deadline = time.time() + timeout_s
    last = None

    while time.time() < deadline:
        msg = mav.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1)
        if msg:
            last = msg
            print(f"local_position: x={msg.x:.2f} y={msg.y:.2f} z={msg.z:.2f}")
            return msg

    raise TimeoutError("Timed out waiting for LOCAL_POSITION_NED.")


def mavlink_get_local_position(mav, timeout_s=2):
    msg = mav.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=timeout_s)
    if msg is None:
        raise TimeoutError("Timed out waiting for LOCAL_POSITION_NED sample.")
    return msg


def mavlink_verify_climb(mav, start_z, min_climb_m=0.5, timeout_s=8):
    """
    PX4 local NED uses negative z for altitude above home.
    A successful climb should make z decrease by at least min_climb_m.
    """
    print(f"Verifying climb: start_z={start_z:.2f}, required climb={min_climb_m:.2f}m")
    deadline = time.time() + timeout_s
    best_z = start_z

    while time.time() < deadline:
        pos = mavlink_get_local_position(mav, timeout_s=1)
        best_z = min(best_z, pos.z)
        climb = start_z - best_z
        print(
            f"\r  local_z={pos.z:.2f}, observed_climb={climb:.2f}m",
            end="",
            flush=True,
        )
        if climb >= min_climb_m:
            print()
            print("✅ MAVLink takeoff produced real climb.")
            return True

    print()
    raise RuntimeError(
        "MAVLink takeoff was ACKed by PX4, but no real climb was observed. "
        "Use --control px4 or --control auto for the confirmed working path."
    )


def mavlink_control_sequence(mav, force_arm=False):
    """
    PX4 MAVLink command sequence:
      1. Wait for local position.
      2. Arm.
      3. Send NAV_TAKEOFF.
      4. Hold.
      5. Send NAV_LAND.

    We intentionally avoid OFFBOARD here because the current PX4/pymavlink
    mode mapping returned tuples like (29, 6, 0), and mav.set_mode() rejected
    them with "required argument is not a float".
    """
    pos = mavlink_wait_local_position(mav)
    mavlink_arm(mav, force=force_arm)
    mavlink_takeoff(mav, relative_alt_m=3.0)
    mavlink_verify_climb(mav, start_z=pos.z, min_climb_m=0.5, timeout_s=8)
    print("Holding after MAVLink takeoff...")
    time.sleep(5)
    mavlink_land(mav)
    time.sleep(8)


def airsim_control_sequence(client, vehicle_name):
    print("AirSim control sequence: armDisarm -> takeoffAsync -> hover -> landAsync")
    if not arm_with_retries(client, vehicle_name):
        raise RuntimeError("AirSim RPC armDisarm() failed.")

    client.takeoffAsync(vehicle_name=vehicle_name).join()
    time.sleep(5)
    client.landAsync(vehicle_name=vehicle_name).join()


def auto_control_sequence(
    client,
    vehicle_name,
    endpoints,
    px4_tmux_target,
    force_mavlink_arm,
    trajectory_path,
):
    """
    Prefer PX4 shell because it is the only confirmed complete path so far:
    arm, takeoff, and land. MAVLink currently arms successfully and now tries
    NAV_TAKEOFF/NAV_LAND, but PX4 shell remains the safest default.
    """
    try:
        print("AUTO control: trying PX4 shell control sequence...")
        px4_control_sequence(client, vehicle_name, px4_tmux_target, trajectory_path)
        return "px4-shell", None
    except Exception as exc:
        print(f"⚠️ AUTO PX4 shell control failed: {exc}")

    try:
        print("AUTO control: trying MAVLink full control sequence...")
        mav = connect_mavlink(endpoints)
        mavlink_control_sequence(mav, force_arm=force_mavlink_arm)
        return "mavlink", mav
    except Exception as exc:
        print(f"⚠️ AUTO MAVLink control failed: {exc}")

    print("AUTO control: trying AirSim control sequence as last resort...")
    airsim_control_sequence(client, vehicle_name)
    return "airsim", None


def run_test_flight(
    vehicle_name,
    timeout_s,
    skip_arm,
    mavlink_endpoints,
    px4_tmux_target,
    control_mode,
    force_mavlink_arm,
    trajectory_path,
):
    client = airsim.MultirotorClient()
    client.confirmConnection()

    v_name = choose_vehicle(client, vehicle_name)
    print(f"Using vehicle: {v_name}")
    control_method = None
    mav = None

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

        if skip_arm:
            print("Skipping control sequence by request.")
        else:
            if control_mode == "mavlink":
                mav = connect_mavlink(mavlink_endpoints)
                mavlink_control_sequence(mav, force_arm=force_mavlink_arm)
                control_method = "mavlink"
            elif control_mode == "px4":
                px4_control_sequence(client, v_name, px4_tmux_target, trajectory_path)
                control_method = "px4-shell"
            elif control_mode == "airsim":
                airsim_control_sequence(client, v_name)
                control_method = "airsim"
            elif control_mode == "auto":
                control_method, mav = auto_control_sequence(
                    client,
                    v_name,
                    mavlink_endpoints,
                    px4_tmux_target,
                    force_mavlink_arm,
                    trajectory_path,
                )
            else:
                raise RuntimeError(f"Unknown control mode: {control_mode}")

            print(f"✅ Control sequence completed using: {control_method}")

    except Exception as e:
        print(f"\n❌ Test flight failed: {e}")
        traceback.print_exc()
    finally:
        if mav is not None:
            mavlink_disarm(mav, force=force_mavlink_arm)
            try:
                mav.close()
            except Exception:
                pass

        if control_method == "px4-shell":
            disarm_via_px4_shell(px4_tmux_target)

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
    parser.add_argument(
        "--mavlink-endpoint",
        action="append",
        default=[],
        help=(
            "MAVLink endpoint(s) to try. Can be repeated or comma-separated. "
            "Default: udpin:127.0.0.1:14550, udpin:127.0.0.1:14540, udpin:127.0.0.1:14600"
        ),
    )
    parser.add_argument("--px4-tmux-target", default=DEFAULT_PX4_TMUX_TARGET)
    parser.add_argument(
        "--trajectory",
        type=validate_trajectory_file,
        default=None,
        help=(
            "Path to a JSON trajectory file. Relative paths are resolved from "
            "the simulation directory. If not provided, uses hardcoded "
            "takeoff/hover/land sequence."
        ),
    )
    parser.add_argument(
        "--control",
        choices=["auto", "mavlink", "px4", "airsim"],
        default="auto",
        help=(
            "Control strategy for arm/takeoff/hover/land. "
            "Default: auto, currently prefers PX4 shell because it is the confirmed working path. "
            "MAVLink mode is experimental: PX4 may ACK takeoff without producing motion."
        ),
    )
    parser.add_argument("--force-mavlink-arm", action="store_true")
    args = parser.parse_args()

    endpoints = parse_mavlink_endpoints(args.mavlink_endpoint) or DEFAULT_MAVLINK_ENDPOINTS

    run_test_flight(
        vehicle_name=args.vehicle,
        timeout_s=args.timeout,
        skip_arm=args.skip_arm,
        mavlink_endpoints=endpoints,
        px4_tmux_target=args.px4_tmux_target,
        control_mode=args.control,
        force_mavlink_arm=args.force_mavlink_arm,
        trajectory_path=args.trajectory,
    )