#!/usr/bin/env python3
"""Stream AirSim scene frames using a compact binary protocol.

Frame protocol, little-endian:

    magic[8]        = b'DEDFRM1\0'
    header_size     uint32, currently 56
    version         uint32, 1 for RGB-only, 2 for RGB + ego JSON
    sequence        uint64
    timestamp_ns    int64
    width           uint32
    height          uint32
    channels        uint32, currently 3
    pixel_format    uint32, currently 1 for RGB8
    payload_size    uint32
    reserved        uint32, version 2 uses this as ego_json_size
    payload         raw RGB bytes
    ego_json        optional UTF-8 JSON bytes when version == 2

This avoids JSON/base64/PPM overhead while keeping AirSim dependencies in the
Python bridge process instead of the C++ core stack.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import struct
import sys
import time
from typing import TextIO

import airsim

MAGIC = b"DEDFRM1\0"
VERSION_RGB_ONLY = 1
VERSION_RGB_EGO = 2
HEADER_SIZE = 56
PIXEL_FORMAT_RGB8 = 1
MAVLINK_SAFETY_ARMED_FLAG = 128


def elapsed_ms(start_ns: int, end_ns: int) -> float:
    return (end_ns - start_ns) / 1_000_000.0


class TimingJsonlWriter:
    """Optional dependency-free bridge-internal timing writer."""

    def __init__(self, path: str | None):
        self._file: TextIO | None = None
        if path:
            output_path = Path(path)
            output_path.parent.mkdir(parents=True, exist_ok=True)
            self._file = output_path.open("w", encoding="utf-8")

    def close(self) -> None:
        if self._file is not None:
            self._file.close()
            self._file = None

    def write(self, record: dict[str, object]) -> None:
        if self._file is None:
            return
        self._file.write(json.dumps(record, separators=(",", ":")) + "\n")
        self._file.flush()


class MavlinkArmedStateReader:
    """Best-effort non-blocking armed-state reader from MAVLink heartbeats."""

    def __init__(self, endpoints: list[str]):
        self._connections: list[object] = []
        self._last_armed: bool | None = None
        self._mavutil = None

        if not endpoints:
            return

        try:
            from pymavlink import mavutil  # type: ignore
        except Exception as exc:  # pragma: no cover - depends on sim host deps
            print(
                f"airsim-stream-frames-binary: pymavlink unavailable; "
                f"MAVLink armed telemetry disabled: {exc}",
                file=sys.stderr,
            )
            return

        self._mavutil = mavutil
        for endpoint in endpoints:
            try:
                connection = mavutil.mavlink_connection(
                    endpoint,
                    autoreconnect=True,
                    source_system=255,
                )
                self._connections.append(connection)
                print(
                    f"airsim-stream-frames-binary: MAVLink armed telemetry listening on {endpoint}",
                    file=sys.stderr,
                )
            except Exception as exc:  # pragma: no cover - depends on live ports
                print(
                    f"airsim-stream-frames-binary: failed to open MAVLink endpoint "
                    f"{endpoint}: {exc}",
                    file=sys.stderr,
                )

    def sample(self) -> tuple[bool, bool]:
        for connection in self._connections:
            while True:
                try:
                    msg = connection.recv_match(type="HEARTBEAT", blocking=False)
                except Exception as exc:  # pragma: no cover - live transport only
                    print(
                        f"airsim-stream-frames-binary: MAVLink heartbeat read failed: {exc}",
                        file=sys.stderr,
                    )
                    break
                if msg is None:
                    break
                base_mode = int(getattr(msg, "base_mode", 0))
                self._last_armed = bool(base_mode & MAVLINK_SAFETY_ARMED_FLAG)

        if self._last_armed is None:
            return False, False
        return self._last_armed, True


def parse_mavlink_endpoints(value: str) -> list[str]:
    return [endpoint.strip() for endpoint in value.split(",") if endpoint.strip()]


def rgb_bytes_from_response(response: object) -> bytes:
    width = int(response.width)
    height = int(response.height)
    raw = bytes(response.image_data_uint8)
    if width <= 0 or height <= 0:
        raise RuntimeError(f"invalid AirSim image dimensions: {width}x{height}")

    pixel_count = width * height
    if len(raw) == pixel_count * 3:
        return raw
    if len(raw) == pixel_count * 4:
        rgb_buffer = bytearray()
        for i in range(0, len(raw), 4):
            rgb_buffer.extend(raw[i : i + 3])
        return bytes(rgb_buffer)

    raise RuntimeError(
        f"unexpected AirSim image byte count: got {len(raw)}, "
        f"expected {pixel_count * 3} or {pixel_count * 4}"
    )


def ego_json_bytes(
    client: airsim.MultirotorClient,
    vehicle_name: str,
    timestamp_ns: int,
    mavlink_armed_reader: MavlinkArmedStateReader | None = None,
) -> bytes:
    # 2.14 optimization:
    # Use one AirSim RPC per frame for ego telemetry. MultirotorState already
    # carries kinematics_estimated position/orientation/velocity, so avoid an
    # extra simGetVehiclePose() call in stream_binary_ego mode.
    state = client.getMultirotorState(vehicle_name=vehicle_name)
    kin = state.kinematics_estimated

    position = kin.position
    orientation = airsim.to_eularian_angles(kin.orientation)
    velocity = kin.linear_velocity
    angular_velocity = kin.angular_velocity

    landed_state = int(getattr(state, "landed_state", 0))
    armed_valid = hasattr(state, "armed")
    armed = bool(getattr(state, "armed", False)) if armed_valid else False
    armed_source = "airsim" if armed_valid else "none"

    if mavlink_armed_reader is not None:
        mavlink_armed, mavlink_armed_valid = mavlink_armed_reader.sample()
        if mavlink_armed_valid:
            armed = mavlink_armed
            armed_valid = True
            armed_source = "mavlink_heartbeat"

    payload = {
        "timestamp_ns": int(timestamp_ns),
        "position": [float(position.x_val), float(position.y_val), float(position.z_val)],
        "rotation_rpy": [float(orientation[0]), float(orientation[1]), float(orientation[2])],
        "velocity": [float(velocity.x_val), float(velocity.y_val), float(velocity.z_val)],
        "angular_velocity": [
            float(angular_velocity.x_val),
            float(angular_velocity.y_val),
            float(angular_velocity.z_val),
        ],
        "landed_state": landed_state,
        "armed": armed,
        "armed_valid": armed_valid,
        "armed_source": armed_source,
    }
    return json.dumps(payload, separators=(",", ":")).encode("utf-8")


def write_frame(
    sequence: int,
    timestamp_ns: int,
    width: int,
    height: int,
    payload: bytes,
    ego_payload: bytes = b"",
) -> bool:
    version = VERSION_RGB_EGO if ego_payload else VERSION_RGB_ONLY
    header = struct.pack(
        "<8sIIQqIIIIII",
        MAGIC,
        HEADER_SIZE,
        version,
        sequence,
        timestamp_ns,
        width,
        height,
        3,
        PIXEL_FORMAT_RGB8,
        len(payload),
        len(ego_payload),
    )
    try:
        sys.stdout.buffer.write(header)
        sys.stdout.buffer.write(payload)
        if ego_payload:
            sys.stdout.buffer.write(ego_payload)
        sys.stdout.buffer.flush()
    except BrokenPipeError:
        return False
    return True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--rpc-port", type=int, default=41451)
    parser.add_argument("--vehicle-name", default="PX4")
    parser.add_argument("--camera-name", default="front_center")
    parser.add_argument("--count", type=int, default=0, help="0 means stream forever")
    parser.add_argument("--rate-hz", type=float, default=5.0)
    parser.add_argument(
        "--include-ego",
        action="store_true",
        help="Append ego telemetry JSON after each RGB payload using binary protocol version 2.",
    )
    parser.add_argument(
        "--mavlink-armed-endpoints",
        default=os.environ.get("DEDALUS_MAVLINK_ARMED_ENDPOINTS", ""),
        help=(
            "Comma-separated pymavlink endpoints used to derive armed state "
            "from HEARTBEAT base_mode. Example: "
            "udpin:127.0.0.1:14550,udpin:127.0.0.1:14540"
        ),
    )
    parser.add_argument(
        "--timing-jsonl",
        default="",
        help="Optional path for bridge-internal timing JSONL records.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    timing = TimingJsonlWriter(args.timing_jsonl or None)
    mavlink_armed_reader = MavlinkArmedStateReader(
        parse_mavlink_endpoints(args.mavlink_armed_endpoints)
    )
    try:
        client = airsim.MultirotorClient(ip=args.host, port=args.rpc_port)
        client.confirmConnection()

        period_s = 0.0 if args.rate_hz <= 0 else 1.0 / args.rate_hz
        sequence = 0
        while args.count == 0 or sequence < args.count:
            loop_start_ns = time.perf_counter_ns()

            image_start_ns = time.perf_counter_ns()
            responses = client.simGetImages(
                [airsim.ImageRequest(args.camera_name, airsim.ImageType.Scene, False, False)],
                vehicle_name=args.vehicle_name,
            )
            image_end_ns = time.perf_counter_ns()
            if not responses:
                raise RuntimeError("AirSim returned no image responses")

            response = responses[0]
            sequence += 1
            timestamp_ns = int(getattr(response, "time_stamp", 0) or time.time_ns())

            rgb_start_ns = time.perf_counter_ns()
            payload = rgb_bytes_from_response(response)
            rgb_end_ns = time.perf_counter_ns()

            ego_start_ns = time.perf_counter_ns()
            ego_payload = (
                ego_json_bytes(client, args.vehicle_name, timestamp_ns, mavlink_armed_reader)
                if args.include_ego
                else b""
            )
            ego_end_ns = time.perf_counter_ns()

            write_start_ns = time.perf_counter_ns()
            write_ok = write_frame(
                sequence,
                timestamp_ns,
                int(response.width),
                int(response.height),
                payload,
                ego_payload,
            )
            write_end_ns = time.perf_counter_ns()
            if not write_ok:
                return 0

            sleep_ms = 0.0
            if period_s > 0 and (args.count == 0 or sequence < args.count):
                sleep_start_ns = time.perf_counter_ns()
                time.sleep(period_s)
                sleep_end_ns = time.perf_counter_ns()
                sleep_ms = elapsed_ms(sleep_start_ns, sleep_end_ns)

            loop_end_ns = time.perf_counter_ns()
            timing.write(
                {
                    "sequence": sequence,
                    "timestamp_ns": timestamp_ns,
                    "width": int(response.width),
                    "height": int(response.height),
                    "payload_bytes": len(payload),
                    "ego_bytes": len(ego_payload),
                    "include_ego": bool(args.include_ego),
                    "sim_get_images_ms": elapsed_ms(image_start_ns, image_end_ns),
                    "rgb_convert_ms": elapsed_ms(rgb_start_ns, rgb_end_ns),
                    "ego_sample_ms": elapsed_ms(ego_start_ns, ego_end_ns),
                    "stdout_write_ms": elapsed_ms(write_start_ns, write_end_ns),
                    "sleep_ms": sleep_ms,
                    "total_loop_ms": elapsed_ms(loop_start_ns, loop_end_ns),
                }
            )

        return 0
    finally:
        timing.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    finally:
        # Avoid the interpreter printing a second BrokenPipeError while flushing
        # stdout during process teardown after the C++ consumer exits normally.
        try:
            sys.stdout.close()
        except BrokenPipeError:
            pass