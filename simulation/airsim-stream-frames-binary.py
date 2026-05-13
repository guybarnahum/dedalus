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
import struct
import sys
import time

import airsim

MAGIC = b"DEDFRM1\0"
VERSION_RGB_ONLY = 1
VERSION_RGB_EGO = 2
HEADER_SIZE = 56
PIXEL_FORMAT_RGB8 = 1


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


def ego_json_bytes(client: airsim.MultirotorClient, vehicle_name: str, timestamp_ns: int) -> bytes:
    state = client.getMultirotorState(vehicle_name=vehicle_name)
    pose = client.simGetVehiclePose(vehicle_name=vehicle_name)

    position = pose.position
    orientation = airsim.to_eularian_angles(pose.orientation)
    velocity = state.kinematics_estimated.linear_velocity
    angular_velocity = state.kinematics_estimated.angular_velocity

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
    }
    return json.dumps(payload, separators=(",", ":")).encode("utf-8")


def write_frame(
    sequence: int,
    timestamp_ns: int,
    width: int,
    height: int,
    payload: bytes,
    ego_payload: bytes = b"",
) -> None:
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
    sys.stdout.buffer.write(header)
    sys.stdout.buffer.write(payload)
    if ego_payload:
        sys.stdout.buffer.write(ego_payload)
    sys.stdout.buffer.flush()


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
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    client = airsim.MultirotorClient(ip=args.host, port=args.rpc_port)
    client.confirmConnection()

    period_s = 0.0 if args.rate_hz <= 0 else 1.0 / args.rate_hz
    sequence = 0
    while args.count == 0 or sequence < args.count:
        responses = client.simGetImages(
            [airsim.ImageRequest(args.camera_name, airsim.ImageType.Scene, False, False)],
            vehicle_name=args.vehicle_name,
        )
        if not responses:
            raise RuntimeError("AirSim returned no image responses")
        response = responses[0]
        sequence += 1
        timestamp_ns = int(getattr(response, "time_stamp", 0) or time.time_ns())
        payload = rgb_bytes_from_response(response)
        ego_payload = ego_json_bytes(client, args.vehicle_name, timestamp_ns) if args.include_ego else b""
        write_frame(sequence, timestamp_ns, int(response.width), int(response.height), payload, ego_payload)
        if period_s > 0 and (args.count == 0 or sequence < args.count):
            time.sleep(period_s)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
