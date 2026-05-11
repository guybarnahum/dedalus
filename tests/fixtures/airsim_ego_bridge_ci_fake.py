#!/usr/bin/env python3
"""Fake AirSim ego bridge used by CTest."""

from __future__ import annotations

import argparse
import json
import sys


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--rpc-port", type=int, default=41451)
    parser.add_argument("--vehicle-name", default="PX4")
    parser.add_argument("--camera-name", default="front_center")
    parser.parse_args()

    payload = {
        "timestamp_ns": 123456789,
        "position": [1.0, 2.0, -3.0],
        "rotation_rpy": [0.1, 0.2, 0.3],
        "velocity": [4.0, 5.0, 6.0],
        "angular_velocity": [0.01, 0.02, 0.03],
    }
    sys.stdout.write(json.dumps(payload, separators=(",", ":")) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
