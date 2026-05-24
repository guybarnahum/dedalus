#!/usr/bin/env python3
"""Return AirSim scene object poses as compact JSON.

This is a bridge utility used by the dependency-free C++ core to read selected
AirSim object poses without linking against the AirSim Python package directly.
It writes machine-readable JSON to stdout and human diagnostics to stderr.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from typing import Any

try:
    import airsim
except ImportError as exc:
    raise SystemExit("airsim Python package is required: pip install airsim") from exc


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1", help="AirSim RPC host")
    parser.add_argument("--rpc-port", type=int, default=41451, help="AirSim RPC port")
    parser.add_argument("--object", action="append", default=[], dest="objects", help="Object name to query. May be repeated.")
    parser.add_argument("--timestamp-ns", type=int, default=0, help="Optional caller timestamp to echo in the response.")
    parser.add_argument("--fail-on-missing", action=argparse.BooleanOptionalAction, default=True)
    return parser.parse_args()


def pose_to_json(name: str, pose: Any) -> dict[str, Any]:
    position = pose.position
    orientation = pose.orientation
    return {
        "name": name,
        "pose_available": True,
        "position_ned_m": [float(position.x_val), float(position.y_val), float(position.z_val)],
        "orientation_quat_xyzw": [
            float(orientation.x_val),
            float(orientation.y_val),
            float(orientation.z_val),
            float(orientation.w_val),
        ],
    }


def main() -> int:
    args = parse_args()
    if not args.objects:
        raise ValueError("at least one --object is required")

    client = airsim.MultirotorClient(ip=args.host, port=args.rpc_port)
    client.confirmConnection()

    objects: list[dict[str, Any]] = []
    errors: list[str] = []
    for name in args.objects:
        try:
            pose = client.simGetObjectPose(name)
            objects.append(pose_to_json(name, pose))
        except Exception as exc:
            error = f"{name}: {exc}"
            errors.append(error)
            objects.append({"name": name, "pose_available": False, "error": str(exc)})

    if errors and args.fail_on_missing:
        raise RuntimeError("failed to read AirSim object pose(s): " + "; ".join(errors))

    payload = {
        "schema_version": 1,
        "source": "airsim_object_poses",
        "timestamp_ns": args.timestamp_ns if args.timestamp_ns != 0 else time.time_ns(),
        "objects": objects,
    }
    print(json.dumps(payload, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("airsim-object-poses: interrupted", file=sys.stderr)
        raise SystemExit(130)
    except Exception as exc:
        print(f"airsim-object-poses: {exc}", file=sys.stderr)
        raise SystemExit(1)
