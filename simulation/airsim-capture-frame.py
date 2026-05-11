#!/usr/bin/env python3
"""Capture one AirSim scene frame and write PPM bytes to stdout.

This script is the live AirSim RGB bridge for the dependency-free C++ core
stack. The C++ AirSimFrameSource invokes it as an external command and parses
the PPM output into a FramePacket.
"""

from __future__ import annotations

import argparse
import sys

import airsim


def ppm_bytes_from_response(response: object) -> bytes:
    width = int(response.width)
    height = int(response.height)
    raw = bytes(response.image_data_uint8)
    if width <= 0 or height <= 0:
        raise RuntimeError(f"invalid AirSim image dimensions: {width}x{height}")

    pixel_count = width * height
    if len(raw) == pixel_count * 3:
        rgb = raw
    elif len(raw) == pixel_count * 4:
        rgb_buffer = bytearray()
        for i in range(0, len(raw), 4):
            rgb_buffer.extend(raw[i : i + 3])
        rgb = bytes(rgb_buffer)
    else:
        raise RuntimeError(
            f"unexpected AirSim image byte count: got {len(raw)}, "
            f"expected {pixel_count * 3} or {pixel_count * 4}"
        )

    return f"P6\n{width} {height}\n255\n".encode("ascii") + rgb


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--rpc-port", type=int, default=41451)
    parser.add_argument("--vehicle-name", default="PX4")
    parser.add_argument("--camera-name", default="front_center")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    client = airsim.MultirotorClient(ip=args.host, port=args.rpc_port)
    client.confirmConnection()
    responses = client.simGetImages(
        [airsim.ImageRequest(args.camera_name, airsim.ImageType.Scene, False, False)],
        vehicle_name=args.vehicle_name,
    )
    if not responses:
        raise RuntimeError("AirSim returned no image responses")
    sys.stdout.buffer.write(ppm_bytes_from_response(responses[0]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
