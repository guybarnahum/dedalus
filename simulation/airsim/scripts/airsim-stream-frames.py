#!/usr/bin/env python3
"""Stream AirSim scene frames as JSON lines.

Each stdout line is a compact JSON object:

    {"frame_id":"airsim_stream_frame_0001","timestamp_ns":123,"camera_id":"front_center","ppm_b64":"..."}

The PPM payload is base64-encoded P6 bytes so the C++ core can read a
line-oriented persistent bridge without linking AirSim/rpclib.
"""

from __future__ import annotations

import argparse
import base64
import json
import sys
import time

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
    parser.add_argument("--count", type=int, default=0, help="0 means stream forever")
    parser.add_argument("--rate-hz", type=float, default=5.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    client = airsim.MultirotorClient(ip=args.host, port=args.rpc_port)
    client.confirmConnection()

    period_s = 0.0 if args.rate_hz <= 0 else 1.0 / args.rate_hz
    index = 0
    while args.count == 0 or index < args.count:
        responses = client.simGetImages(
            [airsim.ImageRequest(args.camera_name, airsim.ImageType.Scene, False, False)],
            vehicle_name=args.vehicle_name,
        )
        if not responses:
            raise RuntimeError("AirSim returned no image responses")
        response = responses[0]
        timestamp_ns = int(getattr(response, "time_stamp", 0) or time.time_ns())
        index += 1
        payload = {
            "frame_id": f"airsim_stream_frame_{index:04d}",
            "timestamp_ns": timestamp_ns,
            "camera_id": args.camera_name,
            "ppm_b64": base64.b64encode(ppm_bytes_from_response(response)).decode("ascii"),
        }
        sys.stdout.write(json.dumps(payload, separators=(",", ":")) + "\n")
        sys.stdout.flush()
        if period_s > 0 and (args.count == 0 or index < args.count):
            time.sleep(period_s)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
