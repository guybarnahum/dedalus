#!/usr/bin/env python3
"""Fake binary RGB plus vehicle-state frame bridge used by CTest."""

from __future__ import annotations

import argparse
import json
import struct
import sys

MAGIC = b"DEDFRM1\0"
HEADER_SIZE = 60
VERSION = 3
PIXEL_FORMAT_RGB8 = 1


def write_frame(sequence: int, timestamp_ns: int) -> None:
    payload = (
        b"\xff\x00\x00"
        b"\x00\xff\x00"
        b"\x00\x00\xff"
        b"\xff\xff\xff"
    )
    state = {
        "timestamp_ns": timestamp_ns,
        "position": [1.0 + sequence, 2.0 + sequence, 3.0 + sequence],
        "rotation_rpy": [0.1, 0.2, 0.3],
        "velocity": [4.0 + sequence, 5.0 + sequence, 6.0 + sequence],
        "angular_velocity": [0.01, 0.02, 0.03],
    }
    sidecar = json.dumps(state, separators=(",", ":")).encode("utf-8")
    header = struct.pack(
        "<8sIIQqIIIIIII",
        MAGIC,
        HEADER_SIZE,
        VERSION,
        sequence,
        timestamp_ns,
        2,
        2,
        3,
        PIXEL_FORMAT_RGB8,
        len(payload),
        len(sidecar),
        0,  # depth_size
    )
    sys.stdout.buffer.write(header)
    sys.stdout.buffer.write(payload)
    sys.stdout.buffer.write(sidecar)
    sys.stdout.buffer.flush()


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
    for index in range(count):
        write_frame(index + 1, 3000 + index)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
