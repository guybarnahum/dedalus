#!/usr/bin/env python3
import airsim
import time
import argparse
import traceback
import subprocess

from pymavlink import mavutil


DEFAULT_VEHICLE = "PX4"
DEFAULT_PX4_TMUX_TARGET = "dedalus-sim:px4"

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


def px4_control_sequence(target):
    print("PX4 shell control sequence: arm -> takeoff -> hold -> land")
    px4_shell("commander arm", target)
    time.sleep(2)
    px4_shell("commander takeoff", target)
    time.sleep(10)
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


def mavlink_set_mode(mav, mode_name):
    mode_mapping = mav.mode_mapping()
    if not mode_mapping or mode_name not in mode_mapping:
        raise RuntimeError(f"MAVLink mode '{mode_name}' not available. Known modes: {mode_mapping}")

    mode_id = mode_mapping[mode_name]
    print(f"Setting MAVLink mode: {mode_name} ({mode_id})")
    mav.set_mode(mode_id)

    deadline = time.time() + 5
    while time.time() < deadline:
        ack = mav.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
        if ack:
            print(f"Mode change ACK: command={ack.command} result={ack.result}")
            return True

    print("⚠️ No mode ACK received; continuing.")
    return False


def mavlink_send_position_target_local_ned(mav, x, y, z, vx, vy, vz, yaw=0.0):
    """
    Send a local NED setpoint. Uses position + velocity fields.

    type_mask ignores acceleration, yaw-rate. Position and velocity are active.
    """
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )

    mav.mav.set_position_target_local_ned_send(
        int(time.time() * 1e3) & 0xFFFFFFFF,
        mav.target_system,
        mav.target_component or mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        x,
        y,
        z,
        vx,
        vy,
        vz,
        0,
        0,
        0,
        yaw,
        0,
    )


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


def mavlink_stream_setpoint(mav, x, y, z, vx, vy, vz, duration_s, hz=20):
    interval = 1.0 / hz
    end = time.time() + duration_s

    while time.time() < end:
        mavlink_send_position_target_local_ned(mav, x, y, z, vx, vy, vz)
        time.sleep(interval)


def mavlink_control_sequence(mav, force_arm=False):
    """
    PX4 Offboard sequence:
      1. Wait for local position.
      2. Stream setpoints before mode switch.
      3. Set OFFBOARD.
      4. Arm.
      5. Command climb using local NED z.
      6. Hold.
      7. Land.
    """
    pos = mavlink_wait_local_position(mav)

    hold_x = pos.x
    hold_y = pos.y
    hold_z = pos.z
    takeoff_z = min(hold_z - 3.0, -3.0)

    print("Priming MAVLink Offboard setpoints...")
    mavlink_stream_setpoint(mav, hold_x, hold_y, hold_z, 0, 0, 0, duration_s=2)

    try:
        mavlink_set_mode(mav, "OFFBOARD")
    except Exception as exc:
        print(f"⚠️ OFFBOARD mode switch failed/uncertain: {exc}")

    mavlink_arm(mav, force=force_arm)

    print(f"Climbing to local NED z={takeoff_z:.2f}...")
    mavlink_stream_setpoint(mav, hold_x, hold_y, takeoff_z, 0, 0, -0.7, duration_s=6)

    print("Holding position...")
    mavlink_stream_setpoint(mav, hold_x, hold_y, takeoff_z, 0, 0, 0, duration_s=6)

    print("Landing via MAVLink mode LAND...")
    try:
        mavlink_set_mode(mav, "LAND")
    except Exception as exc:
        print(f"⚠️ LAND mode switch failed; falling back to disarm later: {exc}")
    time.sleep(8)


def airsim_control_sequence(client, vehicle_name):
    print("AirSim control sequence: armDisarm -> takeoffAsync -> hover -> landAsync")
    if not arm_with_retries(client, vehicle_name):
        raise RuntimeError("AirSim RPC armDisarm() failed.")

    client.takeoffAsync(vehicle_name=vehicle_name).join()
    time.sleep(5)
    client.landAsync(vehicle_name=vehicle_name).join()


def auto_control_sequence(client, vehicle_name, endpoints, px4_tmux_target, force_mavlink_arm):
    """
    Prefer MAVLink for PX4 because AirSim/Colosseum armDisarm() is broken in this setup.
    Fall back to PX4 shell. Use AirSim only as last resort.
    """
    try:
        print("AUTO control: trying MAVLink full control sequence...")
        mav = connect_mavlink(endpoints)
        mavlink_control_sequence(mav, force_arm=force_mavlink_arm)
        return "mavlink", mav
    except Exception as exc:
        print(f"⚠️ AUTO MAVLink control failed: {exc}")

    try:
        print("AUTO control: trying PX4 shell control sequence...")
        px4_control_sequence(px4_tmux_target)
        return "px4-shell", None
    except Exception as exc:
        print(f"⚠️ AUTO PX4 shell control failed: {exc}")

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
                px4_control_sequence(px4_tmux_target)
                control_method = "px4-shell"
            elif control_mode == "airsim":
                airsim_control_sequence(client, v_name)
                control_method = "airsim"
            elif control_mode == "auto":
                control_method, mav = auto_control_sequence(
                    client, v_name, mavlink_endpoints, px4_tmux_target, force_mavlink_arm
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
        "--control",
        choices=["auto", "mavlink", "px4", "airsim"],
        default="auto",
        help=(
            "Control strategy for arm/takeoff/hover/land. "
            "Default: auto."
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
    )