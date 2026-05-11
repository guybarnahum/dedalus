#!/usr/bin/env python3
"""Fake persistent AirSim frame bridge used by CTest."""

from __future__ import annotations

import argparse
import base64
import json
import sys


def ppm_payload() -> bytes:
    return (
        b"P6\n2 2\n255\n"
        b"\xff\x00\x00"
        b"\x00\xff\x00"
        b"\x00\x00\xff"
        b"\xff\xff\xff"
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--rpc-port", type=int, default=41451)
    parser.add_argument("--vehicle-name", default="PX4")
    parser.add_argument("--camera-name", default="front_center")
    parser.add_argument("--count", type=int, default=2)
    parser.add_argument("--rate-hz", type=float, default=0.0)
    args = parser.parse_args()

    count = args.count if args.count > 0 else 2
    encoded = base64.b64encode(ppm_payload()).decode("ascii")
    for index in range(count):
        payload = {
            "frame_id": f"airsim_stream_frame_{index + 1:04d}",
            "timestamp_ns": 1000 + index,
            "camera_id": args.camera_name,
            "ppm_b64": encoded,
        }
        sys.stdout.write(json.dumps(payload, separators=(",", ":")) + "\n")
        sys.stdout.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
