#!/usr/bin/env python3
"""Reusable velocity trajectory loading and evaluation for simulation tools."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

DEFAULT_TRAJECTORY_DICT: dict[str, Any] = {
    "name": "default_takeoff_hover_land",
    "description": "Simple takeoff, hover, land sequence.",
    "rate_hz": 10,
    "segments": [
        {"type": "hold", "label": "hover", "duration_s": 10, "vx_mps": 0.0, "vy_mps": 0.0, "vz_mps": 0.0}
    ],
}


def resolve_trajectory_path(path: str | Path, base_dir: Path | None = None) -> Path:
    trajectory_path = Path(path)
    if trajectory_path.is_absolute():
        return trajectory_path
    return (base_dir or Path.cwd()) / trajectory_path


def validate_trajectory_file(path: str | Path, base_dir: Path | None = None) -> str:
    trajectory_path = resolve_trajectory_path(path, base_dir)
    if not trajectory_path.exists():
        raise ValueError(f"Trajectory file not found: {trajectory_path}")
    data = json.loads(trajectory_path.read_text(encoding="utf-8"))
    if "segments" not in data:
        raise ValueError(f"Trajectory file missing 'segments' key: {trajectory_path}")
    return str(path)


def load_trajectory(path: str | Path | None, base_dir: Path | None = None) -> dict[str, Any]:
    if not path:
        return DEFAULT_TRAJECTORY_DICT
    trajectory_path = resolve_trajectory_path(path, base_dir)
    data = json.loads(trajectory_path.read_text(encoding="utf-8"))
    if "segments" not in data:
        raise ValueError(f"Trajectory file missing 'segments': {trajectory_path}")
    return data


def lerp(a: float, b: float, u: float) -> float:
    return a + (b - a) * u


def interpolate_keyframes(keyframes: list[dict[str, Any]], t: float) -> tuple[float, float, float]:
    if not keyframes:
        return 0.0, 0.0, 0.0
    ordered = sorted(keyframes, key=lambda item: float(item.get("t", 0.0)))
    if t <= float(ordered[0]["t"]):
        k = ordered[0]
        return float(k.get("vx_mps", 0.0)), float(k.get("vy_mps", 0.0)), float(k.get("vz_mps", 0.0))
    if t >= float(ordered[-1]["t"]):
        k = ordered[-1]
        return float(k.get("vx_mps", 0.0)), float(k.get("vy_mps", 0.0)), float(k.get("vz_mps", 0.0))
    for i in range(len(ordered) - 1):
        a = ordered[i]
        b = ordered[i + 1]
        if float(a["t"]) <= t <= float(b["t"]):
            span = max(float(b["t"]) - float(a["t"]), 1e-6)
            u = (t - float(a["t"])) / span
            return (
                lerp(float(a.get("vx_mps", 0.0)), float(b.get("vx_mps", 0.0)), u),
                lerp(float(a.get("vy_mps", 0.0)), float(b.get("vy_mps", 0.0)), u),
                lerp(float(a.get("vz_mps", 0.0)), float(b.get("vz_mps", 0.0)), u),
            )
    k = ordered[-1]
    return float(k.get("vx_mps", 0.0)), float(k.get("vy_mps", 0.0)), float(k.get("vz_mps", 0.0))


def segment_duration(segment: dict[str, Any]) -> float:
    if segment.get("type") == "velocity_keyframes":
        keyframes = segment.get("keyframes", [])
        if not keyframes:
            return 0.0
        return float(sorted(keyframes, key=lambda item: float(item.get("t", 0.0)))[-1]["t"])
    return float(segment.get("duration_s", 0.0))


def segment_velocity(segment: dict[str, Any], t: float) -> tuple[float, float, float]:
    typ = segment.get("type")
    if typ == "hold":
        return float(segment.get("vx_mps", 0.0)), float(segment.get("vy_mps", 0.0)), float(segment.get("vz_mps", 0.0))
    if typ == "velocity_keyframes":
        return interpolate_keyframes(segment["keyframes"], t)
    if typ == "circle_velocity":
        speed = float(segment.get("speed_mps", 2.0))
        radius = float(segment.get("radius_m", 10.0))
        direction = str(segment.get("direction", "cw")).lower()
        vz = float(segment.get("vz_mps", 0.0))
        theta = speed / max(radius, 1e-6) * t
        sign = -1.0 if direction == "cw" else 1.0
        return speed * math.cos(theta), sign * speed * math.sin(theta), vz
    if typ == "figure8_velocity":
        duration = float(segment["duration_s"])
        speed = float(segment.get("speed_mps", 2.0))
        scale = float(segment.get("scale_m", 10.0))
        vz = float(segment.get("vz_mps", 0.0))
        theta = 2.0 * math.pi * t / max(duration, 1e-6)
        dx = scale * math.cos(theta)
        dy = scale * math.cos(2.0 * theta)
        norm = max(math.hypot(dx, dy), 1e-6)
        return speed * dx / norm, speed * dy / norm, vz
    raise ValueError(f"Unknown trajectory segment type: {typ}")
