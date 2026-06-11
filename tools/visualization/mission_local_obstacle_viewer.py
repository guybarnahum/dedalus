#!/usr/bin/env python3
"""Render mission-local obstacle map diagnostics from Dedalus snapshots.

Dependency-free: reads snapshot_*.json artifacts and writes a self-contained
HTML canvas viewer.
"""

from __future__ import annotations

import argparse
import glob
import html
import json
import math
from pathlib import Path
from typing import Any, Iterable


def as_float(value: Any, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def vec3(value: Any) -> dict[str, float] | None:
    if isinstance(value, dict):
        if {"x", "y", "z"}.issubset(value.keys()):
            return {"x": as_float(value["x"]), "y": as_float(value["y"]), "z": as_float(value["z"])}
        if {"north", "east", "down"}.issubset(value.keys()):
            return {
                "x": as_float(value["north"]),
                "y": as_float(value["east"]),
                "z": -as_float(value["down"]),
            }
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        return {"x": as_float(value[0]), "y": as_float(value[1]), "z": as_float(value[2])}
    return None


def get_path(obj: Any, path: Iterable[str]) -> Any:
    cur = obj
    for key in path:
        if not isinstance(cur, dict) or key not in cur:
            return None
        cur = cur[key]
    return cur


def first_vec3(obj: Any, paths: list[list[str]]) -> dict[str, float] | None:
    for path in paths:
        value = vec3(get_path(obj, path))
        if value is not None:
            return value
    return None


def first_number(obj: Any, paths: list[list[str]]) -> float | None:
    for path in paths:
        value = get_path(obj, path)
        if value is None:
            continue
        number = as_float(value, math.nan)
        if math.isfinite(number):
            return number
    return None


def snapshot_sort_key(path: str) -> tuple[str, int]:
    name = Path(path).name
    digits = "".join(ch if ch.isdigit() else " " for ch in name).split()
    return (str(Path(path).parent), int(digits[-1]) if digits else -1)


def latest_snapshot(pattern: str) -> Path:
    matches = sorted(glob.glob(pattern, recursive=True), key=snapshot_sort_key)
    if not matches:
        raise SystemExit(f"No snapshots matched: {pattern}")
    return Path(matches[-1])


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise SystemExit(f"Snapshot is not a JSON object: {path}")
    return data


def ego_position(snapshot: dict[str, Any]) -> dict[str, float] | None:
    return first_vec3(
        snapshot,
        [
            ["ego", "local_T_body", "position"],
            ["ego", "pose", "position"],
            ["ego", "position_local"],
            ["ego", "position"],
            ["ego", "local_position"],
        ],
    )


def ego_yaw(snapshot: dict[str, Any]) -> float | None:
    return first_number(
        snapshot,
        [
            ["ego", "local_T_body", "rotation_rpy", "z"],
            ["ego", "local_T_body", "rotation_rpy", "yaw"],
            ["ego", "pose", "rotation_rpy", "z"],
            ["ego", "pose", "rotation_rpy", "yaw"],
            ["ego", "yaw_rad"],
            ["ego", "heading_rad"],
        ],
    )


def trajectory_first_blocked(snapshot: dict[str, Any]) -> dict[str, float] | None:
    safety = snapshot.get("trajectory_safety")
    if not isinstance(safety, dict):
        return None
    return first_vec3(
        safety,
        [
            ["first_blocked_position_local"],
            ["first_blocked_position"],
            ["first_blocked_point"],
        ],
    )


def mission_cells(snapshot: dict[str, Any], max_cells: int) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    mission = snapshot.get("mission_local_obstacle_map")
    if not isinstance(mission, dict):
        raise SystemExit("Snapshot does not contain mission_local_obstacle_map")

    raw_cells = mission.get("debug_cells")
    if not isinstance(raw_cells, list):
        raw_cells = []

    cells: list[dict[str, Any]] = []
    for raw in raw_cells[:max_cells]:
        if not isinstance(raw, dict):
            continue
        center = vec3(raw.get("center_map"))
        size = vec3(raw.get("size_m"))
        if center is None:
            continue
        cells.append(
            {
                "center": center,
                "size": size or {"x": 0.5, "y": 0.5, "z": 0.5},
                "occupied": bool(raw.get("occupied", False)),
                "free": bool(raw.get("free", False)),
                "occupied_score": as_float(raw.get("occupied_score")),
                "free_score": as_float(raw.get("free_score")),
                "risk_score": as_float(raw.get("risk_score")),
                "confidence": as_float(raw.get("confidence")),
                "last_source_provider": str(raw.get("last_source_provider", "")),
                "last_source_kind": str(raw.get("last_source_kind", "")),
            }
        )

    return mission, cells


def history_points(pattern: str | None, max_history: int) -> list[dict[str, float]]:
    if not pattern:
        return []
    paths = sorted(glob.glob(pattern, recursive=True), key=snapshot_sort_key)
    if not paths:
        return []
    if len(paths) > max_history:
        step = max(1, len(paths) // max_history)
        paths = paths[::step][-max_history:]

    points: list[dict[str, float]] = []
    for path in paths:
        try:
            snapshot = load_json(Path(path))
        except Exception:
            continue
        position = ego_position(snapshot)
        if position is not None:
            points.append(position)
    return points


def bounds(points: list[dict[str, float]]) -> dict[str, float]:
    if not points:
        return {"min_x": -10.0, "max_x": 10.0, "min_y": -10.0, "max_y": 10.0, "min_z": -5.0, "max_z": 5.0}
    xs = [p["x"] for p in points]
    ys = [p["y"] for p in points]
    zs = [p["z"] for p in points]
    return {
        "min_x": min(xs),
        "max_x": max(xs),
        "min_y": min(ys),
        "max_y": max(ys),
        "min_z": min(zs),
        "max_z": max(zs),
    }


HTML_TEMPLATE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Dedalus mission-local obstacle map</title>
<style>
  body {{ margin: 0; background: #0f1117; color: #e8e8e8; font: 14px system-ui, sans-serif; }}
  #wrap {{ display: grid; grid-template-columns: 320px 1fr; height: 100vh; }}
  #side {{ padding: 16px; border-right: 1px solid #2b2f3a; overflow: auto; background: #151923; }}
  #canvas {{ width: 100%; height: 100%; display: block; background: radial-gradient(circle at center, #1b2130, #0f1117); }}
  code {{ color: #a7d4ff; word-break: break-all; }}
  .metric {{ display: grid; grid-template-columns: 1fr auto; gap: 8px; padding: 4px 0; border-bottom: 1px solid #252a36; }}
  .hint {{ color: #aab0bf; line-height: 1.4; }}
</style>
</head>
<body>
<div id="wrap">
  <aside id="side">
    <h2>Mission-local obstacle map</h2>
    <p class="hint">Drag to rotate. Wheel to zoom. Mission cells are rendered in map/takeoff-relative coordinates.</p>
    <div class="metric"><span>Snapshot</span><code>{snapshot_path}</code></div>
    <div class="metric"><span>Map frame</span><code>{map_frame}</code></div>
    <div class="metric"><span>Observed cells</span><b>{observed}</b></div>
    <div class="metric"><span>Occupied cells</span><b>{occupied}</b></div>
    <div class="metric"><span>Debug cells shown</span><b>{debug_cells}</b></div>
    <div class="metric"><span>Trajectory points</span><b>{trajectory_points}</b></div>
    <div class="metric"><span>Bounds</span><code>{bounds_text}</code></div>
    <h3>Legend</h3>
    <p class="hint">
      Red: occupied mission-local cells<br>
      Blue: free cells<br>
      Yellow: ego pose / path<br>
      Magenta: first blocked trajectory point, when present
    </p>
  </aside>
  <canvas id="canvas"></canvas>
</div>
<script>
const data = {data_json};

const canvas = document.getElementById("canvas");
const ctx = canvas.getContext("2d");
let yaw = -0.75;
let pitch = 0.75;
let zoom = 1.0;
let dragging = false;
let lastX = 0;
let lastY = 0;

function resize() {{
  const rect = canvas.getBoundingClientRect();
  canvas.width = Math.max(1, Math.floor(rect.width * window.devicePixelRatio));
  canvas.height = Math.max(1, Math.floor(rect.height * window.devicePixelRatio));
  draw();
}}

function center() {{
  const b = data.bounds;
  return {{
    x: 0.5 * (b.min_x + b.max_x),
    y: 0.5 * (b.min_y + b.max_y),
    z: 0.5 * (b.min_z + b.max_z)
  }};
}}

function radius() {{
  const b = data.bounds;
  return Math.max(1.0, b.max_x - b.min_x, b.max_y - b.min_y, b.max_z - b.min_z);
}}

function project(p) {{
  const c = center();
  let x = p.x - c.x;
  let y = p.y - c.y;
  // Viewer convention: flip Z so positive/down snapshot values render visually down,
  // and positive/up render visually up.
  let z = -(p.z - c.z);

  const cy = Math.cos(yaw), sy = Math.sin(yaw);
  const cp = Math.cos(pitch), sp = Math.sin(pitch);

  const x1 = cy * x - sy * y;
  const y1 = sy * x + cy * y;
  const z1 = z;

  const y2 = cp * y1 - sp * z1;
  const z2 = sp * y1 + cp * z1;

  const scale = (0.78 * Math.min(canvas.width, canvas.height) * zoom) / radius();
  return {{
    x: 0.5 * canvas.width + x1 * scale,
    y: 0.5 * canvas.height - z2 * scale,
    depth: y2,
    scale: scale
  }};
}}

function drawLine(a, b, color, width = 1) {{
  const pa = project(a);
  const pb = project(b);
  ctx.strokeStyle = color;
  ctx.lineWidth = width * window.devicePixelRatio;
  ctx.beginPath();
  ctx.moveTo(pa.x, pa.y);
  ctx.lineTo(pb.x, pb.y);
  ctx.stroke();
}}

function drawAxes() {{
  const length = 0.25 * radius();
  const origin = center();
  drawLine(origin, {{x: origin.x + length, y: origin.y, z: origin.z}}, "#ff6b6b", 2);
  drawLine(origin, {{x: origin.x, y: origin.y + length, z: origin.z}}, "#7bed9f", 2);
  drawLine(origin, {{x: origin.x, y: origin.y, z: origin.z + length}}, "#70a1ff", 2);
}}

function drawPoint(p, color, r) {{
  const pp = project(p);
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(pp.x, pp.y, r * window.devicePixelRatio, 0, Math.PI * 2);
  ctx.fill();
}}

function draw() {{
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  drawAxes();

  const cells = data.cells.slice().sort((a, b) => project(a.center).depth - project(b.center).depth);
  for (const cell of cells) {{
    const score = Math.max(cell.occupied_score || 0, cell.free_score || 0, cell.risk_score || 0);
    const r = Math.max(2, Math.min(8, 2 + score * 0.4));
    const color = cell.occupied ? "rgba(255, 83, 83, 0.78)" :
                  cell.free ? "rgba(80, 170, 255, 0.60)" :
                  "rgba(180, 180, 180, 0.45)";
    drawPoint(cell.center, color, r);
  }}

  if (data.trajectory.length > 1) {{
    for (let i = 1; i < data.trajectory.length; ++i) {{
      drawLine(data.trajectory[i - 1], data.trajectory[i], "rgba(255, 220, 90, 0.9)", 2);
    }}
  }}

  if (data.ego) {{
    drawPoint(data.ego, "rgba(255, 230, 80, 1.0)", 7);
    if (typeof data.ego_yaw === "number") {{
      const l = 2.0;
      drawLine(
        data.ego,
        {{x: data.ego.x + l * Math.cos(data.ego_yaw), y: data.ego.y + l * Math.sin(data.ego_yaw), z: data.ego.z}},
        "rgba(255, 230, 80, 1.0)",
        3
      );
    }}
  }}

  if (data.first_blocked) {{
    drawPoint(data.first_blocked, "rgba(255, 80, 255, 1.0)", 8);
  }}
}}

canvas.addEventListener("mousedown", (e) => {{ dragging = true; lastX = e.clientX; lastY = e.clientY; }});
window.addEventListener("mouseup", () => {{ dragging = false; }});
window.addEventListener("mousemove", (e) => {{
  if (!dragging) return;
  const dx = e.clientX - lastX;
  const dy = e.clientY - lastY;
  lastX = e.clientX;
  lastY = e.clientY;
  yaw += dx * 0.006;
  pitch += dy * 0.006;
  draw();
}});
canvas.addEventListener("wheel", (e) => {{
  e.preventDefault();
  zoom *= Math.exp(-e.deltaY * 0.001);
  zoom = Math.max(0.1, Math.min(20.0, zoom));
  draw();
}}, {{passive: false}});

window.addEventListener("resize", resize);
resize();
</script>
</body>
</html>
"""


def build_html(
    snapshot_path: Path,
    snapshot: dict[str, Any],
    mission: dict[str, Any],
    cells: list[dict[str, Any]],
    trajectory: list[dict[str, float]],
) -> str:
    ego = ego_position(snapshot)
    yaw = ego_yaw(snapshot)
    first_blocked = trajectory_first_blocked(snapshot)

    all_points = [cell["center"] for cell in cells]
    all_points.extend(trajectory)
    if ego is not None:
        all_points.append(ego)
    if first_blocked is not None:
        all_points.append(first_blocked)

    b = bounds(all_points)
    data = {
        "cells": cells,
        "trajectory": trajectory,
        "ego": ego,
        "ego_yaw": yaw,
        "first_blocked": first_blocked,
        "bounds": b,
    }

    return HTML_TEMPLATE.format(
        snapshot_path=html.escape(str(snapshot_path)),
        map_frame=html.escape(str(mission.get("map_frame_id", ""))),
        observed=html.escape(str(mission.get("observed_cell_count", 0))),
        occupied=html.escape(str(mission.get("occupied_cell_count", 0))),
        debug_cells=html.escape(str(len(cells))),
        trajectory_points=html.escape(str(len(trajectory))),
        bounds_text=html.escape(
            f"x[{b['min_x']:.1f},{b['max_x']:.1f}] "
            f"y[{b['min_y']:.1f},{b['max_y']:.1f}] "
            f"z[{b['min_z']:.1f},{b['max_z']:.1f}]"
        ),
        data_json=json.dumps(data),
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("snapshot", nargs="?", help="Snapshot JSON to visualize. Defaults to latest out/**/snapshot_*.json.")
    parser.add_argument("--snapshot-glob", default="out/**/snapshot_*.json")
    parser.add_argument("--history-glob", default=None)
    parser.add_argument("--max-cells", type=int, default=2048)
    parser.add_argument("--max-history", type=int, default=1000)
    parser.add_argument("--output", default="out/mission_local_obstacle_viewer.html")
    args = parser.parse_args()

    snapshot_path = Path(args.snapshot) if args.snapshot else latest_snapshot(args.snapshot_glob)
    snapshot = load_json(snapshot_path)
    mission, cells = mission_cells(snapshot, args.max_cells)

    history_glob = args.history_glob or str(snapshot_path.parent / "snapshot_*.json")
    trajectory = history_points(history_glob, args.max_history)

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(build_html(snapshot_path, snapshot, mission, cells, trajectory), encoding="utf-8")

    print(f"Wrote {output}")
    print(f"Snapshot: {snapshot_path}")
    print(f"Mission cells: {len(cells)}")
    print(f"Ego path points: {len(trajectory)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
