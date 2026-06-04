#!/usr/bin/env python3
"""Validate AirSim GT visual-emulation configs declare camera coverage.

AirSim GT global occupancy is an oracle. AirSim GT visual-emulation evidence is
not an oracle: it must be clipped by explicit camera sensing coverage. This test
prevents future live/AirSim configs from accidentally relying on fake forward
coverage.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path


CAMERA_KEY_RE = re.compile(r"^mission_options\.obstacle_sensing\.cameras\.(\d+)\.(.+)$")


def parse_flat_config(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        value = value.strip()
        if len(value) >= 2 and ((value[0] == value[-1] == '"') or (value[0] == value[-1] == "'")):
            value = value[1:-1]
        values[key.strip()] = value
    return values


def camera_blocks(config: dict[str, str]) -> dict[int, dict[str, str]]:
    cameras: dict[int, dict[str, str]] = {}
    for key, value in config.items():
        match = CAMERA_KEY_RE.match(key)
        if not match:
            continue
        index = int(match.group(1))
        field = match.group(2)
        cameras.setdefault(index, {})[field] = value
    return cameras


def has_any_fov(camera: dict[str, str], axis: str) -> bool:
    return f"{axis}_fov_deg" in camera or f"{axis}_fov_rad" in camera


def validate_air_sim_gt_config(path: Path, config: dict[str, str]) -> list[str]:
    errors: list[str] = []
    cameras = camera_blocks(config)

    if not cameras:
        return [
            f"{path}: occupancy_source=airsim_ground_truth requires explicit "
            "mission_options.obstacle_sensing.cameras.* coverage"
        ]

    vehicle_camera_name = config.get("vehicle_camera_name", "")
    declared_names = {
        value
        for camera in cameras.values()
        for key, value in camera.items()
        if key in {"name", "camera_name", "camera_id"} and value
    }
    if vehicle_camera_name and vehicle_camera_name not in declared_names:
        errors.append(
            f"{path}: vehicle_camera_name={vehicle_camera_name!r} is not declared "
            "as a sensing camera name/camera_id"
        )

    required_fields = {
        "role",
        "near_range_m",
        "far_range_m",
        "min_reliable_range_m",
        "max_reliable_range_m",
        "body_T_camera_xyz_m",
        "pointing_source",
    }

    for index, camera in sorted(cameras.items()):
        label = f"{path}: mission_options.obstacle_sensing.cameras.{index}"
        if not (camera.get("name") or camera.get("camera_name") or camera.get("camera_id")):
            errors.append(f"{label} requires name/camera_name/camera_id")
        for field in sorted(required_fields):
            if field not in camera:
                errors.append(f"{label}.{field} is required")
        if not has_any_fov(camera, "horizontal"):
            errors.append(f"{label} requires horizontal_fov_deg or horizontal_fov_rad")
        if not has_any_fov(camera, "vertical"):
            errors.append(f"{label} requires vertical_fov_deg or vertical_fov_rad")
        if "body_T_camera_rpy_deg" not in camera and "body_T_camera_rpy_rad" not in camera:
            errors.append(f"{label} requires body_T_camera_rpy_deg or body_T_camera_rpy_rad")

    return errors


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: test_airsim_visual_emulation_config_coverage.py <repo_root>", file=sys.stderr)
        return 2

    repo_root = Path(sys.argv[1])
    config_paths = sorted((repo_root / "config").glob("core_stack*.y*ml"))
    checked: list[Path] = []
    errors: list[str] = []

    for path in config_paths:
        config = parse_flat_config(path)
        if config.get("occupancy_source") != "airsim_ground_truth":
            continue
        checked.append(path.relative_to(repo_root))
        errors.extend(validate_air_sim_gt_config(path.relative_to(repo_root), config))

    if not checked:
        print("no AirSim ground-truth configs found", file=sys.stderr)
        return 1

    if errors:
        print("AirSim visual-emulation config coverage validation failed:", file=sys.stderr)
        for error in errors:
            print(f"  {error}", file=sys.stderr)
        return 1

    print("AirSim visual-emulation config coverage validation passed:")
    for path in checked:
        print(f"  {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
