#!/usr/bin/env python3
"""Fake binary frame bridge used by CTest."""

from __future__ import annotations

import argparse
import struct
import sys

MAGIC = b"DEDFRM1\0"
HEADER_SIZE = 56
VERSION = 1
PIXEL_FORMAT_RGB8 = 1


def write_frame(sequence: int, timestamp_ns: int) -> None:
    payload = (
        b"\xff\x00\x00"
        b"\x00\xff\x00"
        b"\x00\x00\xff"
        b"\xff\xff\xff"
    )
    header = struct.pack(
        "<8sIIQqIIIIII",
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
        0,
    )
    sys.stdout.buffer.write(header)
    sys.stdout.buffer.write(payload)
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
        write_frame(index + 1, 2000 + index)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
