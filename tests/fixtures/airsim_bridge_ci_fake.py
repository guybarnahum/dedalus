#!/usr/bin/env python3
"""Fake AirSim bridge used by CTest.

It accepts the same arguments as simulation/airsim/scripts/airsim-capture-frame.py and emits a
small binary PPM frame to stdout without importing AirSim.
"""

from __future__ import annotations

import argparse
import sys


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--rpc-port", type=int, default=41451)
    parser.add_argument("--vehicle-name", default="PX4")
    parser.add_argument("--camera-name", default="front_center")
    parser.parse_args()

    ppm = (
        b"P6\n2 2\n255\n"
        b"\xff\x00\x00"
        b"\x00\xff\x00"
        b"\x00\x00\xff"
        b"\xff\xff\xff"
    )
    sys.stdout.buffer.write(ppm)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
