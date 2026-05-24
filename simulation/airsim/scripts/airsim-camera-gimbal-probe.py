#!/usr/bin/env python3
"""Probe AirSim camera pose / gimbal capability.

This utility answers a narrow question for Dedalus behavior planning:

    Can the current AirSim/Colosseum vehicle camera be pitched at runtime?

It does not command PX4 and does not participate in the mission runtime. It only
uses AirSim simulation APIs:

    simGetCameraInfo(...)
    simSetCameraPose(...)
    simGetImage(...)

The result helps decide whether vertical target stare can be implemented as a
simulator-only camera/gimbal sink, or should remain warning/debug-only.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

try:
    import airsim
except ImportError as exc:
    raise SystemExit("airsim Python package is required: pip install airsim") from exc


DEFAULT_CAMERAS = ["front_center", "0", ""]
DEFAULT_PITCH_DEG = [-45.0, 0.0, 25.0]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1", help="AirSim RPC host")
    parser.add_argument("--rpc-port", type=int, default=41451, help="AirSim RPC port")
    parser.add_argument("--vehicle-name", default="PX4", help="AirSim vehicle name")
    parser.add_argument(
        "--camera",
        action="append",
        dest="cameras",
        default=[],
        help="Camera name/id to test. May be repeated. Defaults to front_center, 0, and empty legacy id.",
    )
    parser.add_argument(
        "--pitch-deg",
        action="append",
        type=float,
        dest="pitch_deg",
        default=[],
        help="Pitch angle in degrees to test. May be repeated. Defaults to -45, 0, +25.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("out/airsim_camera_gimbal_probe"),
        help="Directory for captured Scene images and summary JSON.",
    )
    parser.add_argument(
        "--image-type",
        default="Scene",
        choices=["Scene", "DepthPerspective", "Segmentation", "SurfaceNormals", "Infrared"],
        help="AirSim image type to capture.",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print compact machine-readable JSON only.",
    )
    return parser.parse_args()


def quaternion_to_list(quat: Any) -> list[float]:
    return [
        float(quat.x_val),
        float(quat.y_val),
        float(quat.z_val),
        float(quat.w_val),
    ]


def vector_to_list(vec: Any) -> list[float]:
    return [float(vec.x_val), float(vec.y_val), float(vec.z_val)]


def camera_info_to_json(info: Any) -> dict[str, Any]:
    pose = info.pose
    return {
        "fov_deg": float(getattr(info, "fov", 0.0)),
        "position": vector_to_list(pose.position),
        "orientation_quat_xyzw": quaternion_to_list(pose.orientation),
    }


def quat_distance(a: list[float], b: list[float]) -> float:
    return math.sqrt(sum((lhs - rhs) * (lhs - rhs) for lhs, rhs in zip(a, b)))


def image_type_value(name: str) -> int:
    return int(getattr(airsim.ImageType, name))


def safe_filename_camera(camera: str) -> str:
    if camera == "":
        return "legacy_empty"
    return camera.replace("/", "_").replace(" ", "_")


def write_image(path: Path, data: Any) -> bool:
    if data is None:
        return False
    if isinstance(data, str):
        if data == "":
            return False
        path.write_bytes(data.encode("latin1"))
        return True
    if isinstance(data, (bytes, bytearray)):
        if not data:
            return False
        path.write_bytes(bytes(data))
        return True
    return False


def probe_camera(
    *,
    client: Any,
    vehicle_name: str,
    camera: str,
    pitch_values_deg: list[float],
    image_type: int,
    output_dir: Path,
) -> dict[str, Any]:
    result: dict[str, Any] = {
        "camera": camera,
        "ok": False,
        "camera_info_available": False,
        "pose_set_accepted": False,
        "orientation_changed": False,
        "images_saved": 0,
        "steps": [],
    }

    try:
        before = client.simGetCameraInfo(camera, vehicle_name=vehicle_name)
        before_json = camera_info_to_json(before)
        result["before"] = before_json
        result["camera_info_available"] = True
    except Exception as exc:
        result["error"] = f"simGetCameraInfo before failed: {exc}"
        return result

    before_orientation = result["before"]["orientation_quat_xyzw"]
    last_orientation = before_orientation

    for pitch_deg in pitch_values_deg:
        step: dict[str, Any] = {
            "pitch_deg": pitch_deg,
            "pose_set": False,
            "camera_info_available": False,
            "orientation_delta_from_before": None,
            "orientation_delta_from_previous": None,
            "image_saved": False,
            "image_path": None,
        }
        try:
            pose = airsim.Pose(
                airsim.Vector3r(0.0, 0.0, 0.0),
                airsim.to_quaternion(math.radians(pitch_deg), 0.0, 0.0),
            )
            client.simSetCameraPose(camera, pose, vehicle_name=vehicle_name)
            step["pose_set"] = True
            result["pose_set_accepted"] = True
        except Exception as exc:
            step["error"] = f"simSetCameraPose failed: {exc}"
            result["steps"].append(step)
            continue

        try:
            after = client.simGetCameraInfo(camera, vehicle_name=vehicle_name)
            after_json = camera_info_to_json(after)
            step["after"] = after_json
            step["camera_info_available"] = True
            orientation = after_json["orientation_quat_xyzw"]
            step["orientation_delta_from_before"] = quat_distance(before_orientation, orientation)
            step["orientation_delta_from_previous"] = quat_distance(last_orientation, orientation)
            if step["orientation_delta_from_before"] > 1e-6 or step["orientation_delta_from_previous"] > 1e-6:
                result["orientation_changed"] = True
            last_orientation = orientation
        except Exception as exc:
            step["camera_info_error"] = str(exc)

        try:
            image = client.simGetImage(camera, image_type, vehicle_name=vehicle_name)
            image_name = f"{safe_filename_camera(camera)}_pitch_{pitch_deg:+06.1f}.png"
            image_path = output_dir / image_name
            if write_image(image_path, image):
                step["image_saved"] = True
                step["image_path"] = str(image_path)
                result["images_saved"] += 1
        except Exception as exc:
            step["image_error"] = str(exc)

        result["steps"].append(step)

    result["ok"] = bool(result["pose_set_accepted"] and result["orientation_changed"] and result["images_saved"] > 0)
    return result


def print_human_summary(summary: dict[str, Any]) -> None:
    print("AirSim camera/gimbal probe")
    print(f"  host: {summary['host']}:{summary['rpc_port']}")
    print(f"  vehicle: {summary['vehicle_name']}")
    print(f"  output_dir: {summary['output_dir']}")
    print(f"  overall_ok: {summary['overall_ok']}")
    print()
    for camera in summary["cameras"]:
        label = camera["camera"] if camera["camera"] else "<empty legacy id>"
        print(f"camera {label!r}")
        print(f"  ok: {camera['ok']}")
        print(f"  camera_info_available: {camera['camera_info_available']}")
        print(f"  pose_set_accepted: {camera['pose_set_accepted']}")
        print(f"  orientation_changed: {camera['orientation_changed']}")
        print(f"  images_saved: {camera['images_saved']}")
        if "error" in camera:
            print(f"  error: {camera['error']}")
        for step in camera.get("steps", []):
            print(
                "  step pitch={pitch:+.1f}: pose_set={pose_set} info={info} "
                "d_before={db} image={image}".format(
                    pitch=step["pitch_deg"],
                    pose_set=step["pose_set"],
                    info=step["camera_info_available"],
                    db=step["orientation_delta_from_before"],
                    image=step["image_saved"],
                )
            )
        print()


def main() -> int:
    args = parse_args()
    cameras = args.cameras if args.cameras else DEFAULT_CAMERAS
    pitch_values = args.pitch_deg if args.pitch_deg else DEFAULT_PITCH_DEG
    args.output_dir.mkdir(parents=True, exist_ok=True)

    client = airsim.MultirotorClient(ip=args.host, port=args.rpc_port)
    client.confirmConnection()

    image_type = image_type_value(args.image_type)
    results = [
        probe_camera(
            client=client,
            vehicle_name=args.vehicle_name,
            camera=camera,
            pitch_values_deg=pitch_values,
            image_type=image_type,
            output_dir=args.output_dir,
        )
        for camera in cameras
    ]
    summary: dict[str, Any] = {
        "schema_version": 1,
        "source": "airsim_camera_gimbal_probe",
        "timestamp_ns": time.time_ns(),
        "host": args.host,
        "rpc_port": args.rpc_port,
        "vehicle_name": args.vehicle_name,
        "output_dir": str(args.output_dir),
        "image_type": args.image_type,
        "pitch_deg": pitch_values,
        "overall_ok": any(result["ok"] for result in results),
        "cameras": results,
    }

    summary_path = args.output_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")

    if args.json:
        print(json.dumps(summary, separators=(",", ":"), sort_keys=True))
    else:
        print_human_summary(summary)
        print(f"summary: {summary_path}")

    return 0 if summary["overall_ok"] else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("airsim-camera-gimbal-probe: interrupted", file=sys.stderr)
        raise SystemExit(130)
    except Exception as exc:
        print(f"airsim-camera-gimbal-probe: {exc}", file=sys.stderr)
        raise SystemExit(1)
