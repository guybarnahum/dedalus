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
    <div class="metric"><span>Observed cells</span><b id="observed-count">{observed}</b></div>
    <div class="metric"><span>Occupied cells</span><b id="occupied-count">{occupied}</b></div>
    <div class="metric"><span>Cells shown</span><b id="cell-count">{debug_cells}</b></div>
    <div class="metric"><span>Trajectory points</span><b id="trajectory-count">{trajectory_points}</b></div>
    <div class="metric"><span>Bounds</span><code id="bounds-text">{bounds_text}</code></div>
    <div class="metric"><span>Live stream</span><code id="live-status">offline</code></div>
    <div class="metric"><span>Live delta cells</span><b id="live-delta-count">0</b></div>
    <div class="metric"><span>World snapshots</span><b id="world-snapshot-count">0</b></div>
    <div class="metric"><span>Ego updates</span><b id="ego-update-count">0</b></div>
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
if (!Array.isArray(data.cells)) data.cells = [];
if (!Array.isArray(data.trajectory)) data.trajectory = [];

const cellsByKey = new Map();
for (const cell of data.cells) {
  cellsByKey.set(cellKey(cell.center), normalizeCell(cell));
}
data.cells = Array.from(cellsByKey.values());

const LIVE_DECAY_MS = 10000;
const LIVE_OBSTACLE_DIM = 0.45;
const LIVE_TRACK_DIM = 0.55;

const live = {
  enabled: false,
  connected: false,
  eventSource: null,
  eventCount: 0,
  deltaCellCount: 0,
  worldSnapshotCount: 0,
  egoUpdateCount: 0,
  lastSeq: null,
  error: ""
};

function el(id) {
  return document.getElementById(id);
}

function finiteNumber(value, fallback = 0) {
  const n = Number(value);
  return Number.isFinite(n) ? n : fallback;
}

function asVec3(value) {
  if (!value || typeof value !== "object") return null;
  if (["x", "y", "z"].every((k) => Object.prototype.hasOwnProperty.call(value, k))) {
    return {
      x: finiteNumber(value.x),
      y: finiteNumber(value.y),
      z: finiteNumber(value.z)
    };
  }
  if (["north", "east", "down"].every((k) => Object.prototype.hasOwnProperty.call(value, k))) {
    return {
      x: finiteNumber(value.north),
      y: finiteNumber(value.east),
      z: -finiteNumber(value.down)
    };
  }
  return null;
}

function getPath(obj, path) {
  let cur = obj;
  for (const key of path) {
    if (!cur || typeof cur !== "object" || !Object.prototype.hasOwnProperty.call(cur, key)) {
      return null;
    }
    cur = cur[key];
  }
  return cur;
}

function firstVec3(obj, paths) {
  for (const path of paths) {
    const v = asVec3(getPath(obj, path));
    if (v) return v;
  }
  return null;
}

function firstNumber(obj, paths) {
  for (const path of paths) {
    const raw = getPath(obj, path);
    if (raw === null || raw === undefined) continue;
    const n = Number(raw);
    if (Number.isFinite(n)) return n;
  }
  return null;
}

function pathContainsAny(path, words) {
  const lower = path.join(".").toLowerCase();
  return words.some((word) => lower.includes(word));
}

function recursiveVec3Search(obj, path = []) {
  if (!obj || typeof obj !== "object") return null;

  const direct = asVec3(obj);
  if (direct) {
    const key = path.length ? String(path[path.length - 1]).toLowerCase() : "";
    if (
      key.includes("position") ||
      key.includes("pose") ||
      key.includes("location") ||
      pathContainsAny(path, ["local_t_body.position", "local_t_body.position", "local_t_body"])
    ) {
      return direct;
    }
  }

  for (const [key, value] of Object.entries(obj)) {
    const nextPath = path.concat([key]);
    if (
      !pathContainsAny(nextPath, ["ego", "vehicle", "drone", "agent", "ownship", "local_t_body", "pose", "position"]) &&
      nextPath.length > 1
    ) {
      continue;
    }

    if (key.toLowerCase().includes("velocity")) {
      continue;
    }

    const found = recursiveVec3Search(value, nextPath);
    if (found) return found;
  }

  return null;
}

function recursiveYawSearch(obj, path = []) {
  if (!obj || typeof obj !== "object") return null;

  for (const [key, value] of Object.entries(obj)) {
    const lower = key.toLowerCase();
    const nextPath = path.concat([key]);

    if ((lower === "yaw" || lower === "yaw_rad" || lower === "heading" || lower === "heading_rad") && Number.isFinite(Number(value))) {
      return Number(value);
    }

    if ((lower === "z" || lower === "yaw") && pathContainsAny(path, ["rotation_rpy", "rpy", "orientation"]) && Number.isFinite(Number(value))) {
      return Number(value);
    }

    if (
      typeof value === "object" &&
      (pathContainsAny(nextPath, ["ego", "vehicle", "drone", "agent", "ownship", "local_t_body", "pose", "rotation", "orientation", "yaw", "heading"]) ||
       nextPath.length <= 1)
    ) {
      const found = recursiveYawSearch(value, nextPath);
      if (found !== null) return found;
    }
  }

  return null;
}

function snapshotEgoPosition(snapshot) {
  return firstVec3(snapshot, [
    ["ego_state", "local_T_body", "position"],
    ["ego_state", "pose", "position"],
    ["ego_state", "position_local"],
    ["ego_state", "position"],
    ["ego", "local_T_body", "position"],
    ["ego", "pose", "position"],
    ["ego", "position_local"],
    ["ego", "position"],
    ["ego", "local_position"],
    ["vehicle", "position"],
    ["drone", "position"],
    ["agent", "position"],
    ["ownship", "position"]
  ]) || recursiveVec3Search(snapshot);
}

function snapshotEgoYaw(snapshot) {
  const direct = firstNumber(snapshot, [
    ["ego_state", "local_T_body", "rotation_rpy", "z"],
    ["ego_state", "local_T_body", "rotation_rpy", "yaw"],
    ["ego_state", "pose", "rotation_rpy", "z"],
    ["ego_state", "pose", "rotation_rpy", "yaw"],
    ["ego_state", "yaw_rad"],
    ["ego_state", "heading_rad"],
    ["ego", "local_T_body", "rotation_rpy", "z"],
    ["ego", "local_T_body", "rotation_rpy", "yaw"],
    ["ego", "pose", "rotation_rpy", "z"],
    ["ego", "pose", "rotation_rpy", "yaw"],
    ["ego", "yaw_rad"],
    ["ego", "heading_rad"],
    ["vehicle", "yaw_rad"],
    ["drone", "yaw_rad"],
    ["agent", "yaw_rad"],
    ["ownship", "yaw_rad"]
  ]);
  return direct === null ? recursiveYawSearch(snapshot) : direct;
}

function snapshotFirstBlocked(snapshot) {
  const safety = snapshot && snapshot.trajectory_safety;
  if (!safety || typeof safety !== "object") return null;
  return firstVec3(safety, [
    ["first_blocked_position_local"],
    ["first_blocked_position"],
    ["first_blocked_point"]
  ]);
}

function cellKey(center) {
  if (!center) return "missing";
  return [
    finiteNumber(center.x).toFixed(3),
    finiteNumber(center.y).toFixed(3),
    finiteNumber(center.z).toFixed(3)
  ].join(",");
}

function normalizeCell(cell) {
  const center = asVec3(cell.center) || asVec3(cell.center_mission) || asVec3(cell.center_map);
  const size = asVec3(cell.size) || asVec3(cell.size_m) || {x: 0.5, y: 0.5, z: 0.5};
  const occupiedScore = finiteNumber(cell.occupied_score);
  const freeScore = finiteNumber(cell.free_score);
  const riskScore = finiteNumber(cell.risk_score);
  const occupied = cell.occupied === true || (occupiedScore > 0 && occupiedScore >= freeScore);
  const free = cell.free === true || (!occupied && freeScore > 0);
  return {
    center,
    size,
    occupied,
    free,
    occupied_score: occupiedScore,
    free_score: freeScore,
    risk_score: riskScore,
    confidence: finiteNumber(cell.confidence),
    last_source_provider: String(cell.last_source_provider || cell.source_provider || ""),
    last_source_kind: String(cell.last_source_kind || cell.source_kind || ""),
    first_seen_unix_ns: cell.first_seen_unix_ns || null,
    last_seen_unix_ns: cell.last_seen_unix_ns || null,
    live_seen_ms: cell.live_seen_ms || null
  };
}

function recomputeBounds() {
  const points = [];
  for (const cell of data.cells) {
    if (cell.center) points.push(cell.center);
  }
  for (const p of data.trajectory) {
    if (p) points.push(p);
  }
  if (data.ego) points.push(data.ego);
  if (data.first_blocked) points.push(data.first_blocked);

  if (points.length === 0) {
    data.bounds = {min_x: -10, max_x: 10, min_y: -10, max_y: 10, min_z: -5, max_z: 5};
    return;
  }

  const xs = points.map((p) => p.x);
  const ys = points.map((p) => p.y);
  const zs = points.map((p) => p.z);
  data.bounds = {
    min_x: Math.min(...xs),
    max_x: Math.max(...xs),
    min_y: Math.min(...ys),
    max_y: Math.max(...ys),
    min_z: Math.min(...zs),
    max_z: Math.max(...zs)
  };
}

function updateMetrics() {
  const observedCount = cellsByKey.size;
  const occupiedCount = data.cells.filter((cell) => cell.occupied).length;
  const b = data.bounds;
  if (el("observed-count")) el("observed-count").textContent = String(observedCount);
  if (el("occupied-count")) el("occupied-count").textContent = String(occupiedCount);
  if (el("cell-count")) el("cell-count").textContent = String(data.cells.length);
  if (el("trajectory-count")) el("trajectory-count").textContent = String(data.trajectory.length);
  if (el("live-delta-count")) el("live-delta-count").textContent = String(live.deltaCellCount);
  if (el("world-snapshot-count")) el("world-snapshot-count").textContent = String(live.worldSnapshotCount);
  if (el("ego-update-count")) el("ego-update-count").textContent = String(live.egoUpdateCount);
  if (el("bounds-text")) {
    el("bounds-text").textContent =
      `x[${b.min_x.toFixed(1)},${b.max_x.toFixed(1)}] ` +
      `y[${b.min_y.toFixed(1)},${b.max_y.toFixed(1)}] ` +
      `z[${b.min_z.toFixed(1)},${b.max_z.toFixed(1)}]`;
  }
  if (el("live-status")) {
    if (!live.enabled) {
      el("live-status").textContent = "offline";
    } else if (live.connected) {
      const suffix = live.lastSeq === null ? "" : ` seq=${live.lastSeq}`;
      el("live-status").textContent = `connected${suffix}`;
    } else if (live.error) {
      el("live-status").textContent = `error: ${live.error}`;
    } else {
      el("live-status").textContent = "connecting";
    }
  }
}

function applyMissionObstacleMapDelta(delta, seq) {
  if (!delta || typeof delta !== "object" || !Array.isArray(delta.cells)) return;
  let changed = 0;
  for (const raw of delta.cells) {
    const cell = normalizeCell(raw);
    if (!cell.center) continue;
    cell.live_seen_ms = Date.now();
    cellsByKey.set(cellKey(cell.center), cell);
    changed += 1;
  }
  data.cells = Array.from(cellsByKey.values());
  live.deltaCellCount += changed;
  live.lastSeq = seq;
  recomputeBounds();
  updateMetrics();
  draw();
}

function applyWorldSnapshot(snapshot, seq) {
  if (!snapshot || typeof snapshot !== "object") return;
  live.worldSnapshotCount += 1;
  const ego = snapshotEgoPosition(snapshot);
  if (ego) {
    live.egoUpdateCount += 1;
    data.ego = ego;
    const last = data.trajectory.length ? data.trajectory[data.trajectory.length - 1] : null;
    if (!last || Math.hypot(last.x - ego.x, last.y - ego.y, last.z - ego.z) > 0.05) {
      const trackSample = {...ego, live_seen_ms: Date.now()};
      data.trajectory.push(trackSample);
      if (data.trajectory.length > 5000) {
        data.trajectory.splice(0, data.trajectory.length - 5000);
      }
    }
  }
  const yawValue = snapshotEgoYaw(snapshot);
  if (typeof yawValue === "number") {
    data.ego_yaw = yawValue;
  }
  const blocked = snapshotFirstBlocked(snapshot);
  if (blocked) {
    data.first_blocked = blocked;
  }
  live.lastSeq = seq;
  recomputeBounds();
  updateMetrics();
  draw();
}

function eventUrlFromLocation() {
  const params = new URLSearchParams(window.location.search);
  if (params.get("live") === "0") return null;
  const explicit = params.get("events");
  if (explicit) return explicit;
  if (window.location.protocol === "http:" || window.location.protocol === "https:") {
    return "/events";
  }
  return null;
}

function startLiveStream() {
  const url = eventUrlFromLocation();
  if (!url || typeof EventSource === "undefined") {
    live.enabled = false;
    updateMetrics();
    return;
  }

  live.enabled = true;
  live.error = "";
  updateMetrics();

  const source = new EventSource(url);
  live.eventSource = source;

  source.onopen = () => {
    live.connected = true;
    live.error = "";
    updateMetrics();
  };

  source.onerror = () => {
    live.connected = false;
    live.error = "disconnected";
    updateMetrics();
  };

  source.addEventListener("mission_obstacle_map_delta", (event) => {
    try {
      const payload = JSON.parse(event.data);
      applyMissionObstacleMapDelta(payload.mission_obstacle_map_delta, payload.seq ?? null);
    } catch (err) {
      live.error = String(err);
      updateMetrics();
    }
  });

  source.addEventListener("world_snapshot", (event) => {
    try {
      const payload = JSON.parse(event.data);
      applyWorldSnapshot(payload.world_snapshot || payload.snapshot, payload.seq ?? null);
    } catch (err) {
      live.error = String(err);
      updateMetrics();
    }
  });
}



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

  const cells = data.cells.filter((cell) => cell.center).slice().sort((a, b) => project(a.center).depth - project(b.center).depth);
  for (const cell of cells) {{
    if (cell.live_seen_ms) {{
      continue;
    }}
    const score = Math.max(cell.occupied_score || 0, cell.free_score || 0, cell.risk_score || 0);
    const r = Math.max(2, Math.min(8, 2 + score * 0.4));
    const color = cell.occupied ? "rgba(130, 38, 38, 0.42)" :
                  cell.free ? "rgba(45, 95, 145, 0.38)" :
                  "rgba(120, 120, 120, 0.30)";
    drawPoint(cell.center, color, r);
  }}

  if (data.trajectory.length > 1) {{
    for (let i = 1; i < data.trajectory.length; ++i) {{
      const a = data.trajectory[i - 1];
      const b = data.trajectory[i];
      if ((a && a.live_seen_ms) || (b && b.live_seen_ms)) {{
        continue;
      }}
      drawLine(a, b, "rgba(150, 120, 35, 0.38)", 1.5);
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


function liveDecayLevel(seenMs, dimValue) {
  if (!seenMs) return null;
  const ageMs = Math.max(0, Date.now() - Number(seenMs));
  const t = Math.min(1, ageMs / LIVE_DECAY_MS);
  return 1 - ((1 - dimValue) * t);
}

function liveRed(level, alpha = 0.88) {
  const g = Math.round(24 * level);
  const b = Math.round(24 * level);
  return `rgba(255,${g},${b},${alpha})`;
}

function liveYellow(level, alpha = 0.92) {
  const gb = Math.round(255 * level);
  return `rgba(255,${gb},0,${alpha})`;
}

function drawLivePoint(center, color, radius) {
  if (!center) return;
  const p = project(center);
  if (!p || !Number.isFinite(p.x) || !Number.isFinite(p.y)) return;
  ctx.beginPath();
  ctx.arc(p.x, p.y, radius, 0, Math.PI * 2);
  ctx.fillStyle = color;
  ctx.fill();
}

function drawLiveAgingOverlay() {
  const now = Date.now();
  ctx.save();

  for (const cell of data.cells) {
    const level = liveDecayLevel(cell.live_seen_ms, LIVE_OBSTACLE_DIM);
    if (level === null) continue;
    const radius = Math.max(2.0, 5.5 * level);
    drawLivePoint(cell.center, liveRed(level), radius);
  }

  const liveTrack = data.trajectory.filter((point) => point && point.live_seen_ms);
  if (liveTrack.length >= 2) {
    for (let i = 1; i < liveTrack.length; ++i) {
      const a = project(liveTrack[i - 1]);
      const b = project(liveTrack[i]);
      if (!a || !b || !Number.isFinite(a.x) || !Number.isFinite(a.y) || !Number.isFinite(b.x) || !Number.isFinite(b.y)) {
        continue;
      }

      const level = liveDecayLevel(liveTrack[i].live_seen_ms, LIVE_TRACK_DIM) ?? LIVE_TRACK_DIM;
      ctx.beginPath();
      ctx.moveTo(a.x, a.y);
      ctx.lineTo(b.x, b.y);
      ctx.strokeStyle = liveYellow(level, 0.9);
      ctx.lineWidth = Math.max(1.5, 4.0 * level);
      ctx.stroke();
    }
  }

  if (data.ego) {
    drawLivePoint(data.ego, "rgba(255,255,64,1.0)", 6.5);
  }

  ctx.restore();

  // Keep live visuals moving through the 10-second fade even when no new events arrive.
  if (live.enabled && live.connected) {
    requestAnimationFrame(() => {
      const hasRecentObstacle = data.cells.some((cell) => cell.live_seen_ms && now - Number(cell.live_seen_ms) < LIVE_DECAY_MS);
      const hasRecentTrack = data.trajectory.some((point) => point.live_seen_ms && now - Number(point.live_seen_ms) < LIVE_DECAY_MS);
      if (hasRecentObstacle || hasRecentTrack) {
        draw();
      }
    });
  }
}

const baseDraw = draw;
draw = function() {
  baseDraw();
  drawLiveAgingOverlay();
};


window.addEventListener("resize", resize);
recomputeBounds();
updateMetrics();
resize();
startLiveStream();
</script>
</body>
</html>
"""



def render_html_template(template: str, **values: object) -> str:
    """Render HTML_TEMPLATE without treating JavaScript braces as format fields.

    The viewer template contains large JavaScript blocks. Python str.format()
    interprets every single `{...}` pair as a placeholder, which breaks as soon
    as the embedded JavaScript contains normal object/function braces. This
    helper replaces only the explicit top-level template tokens used by
    build_html(), while still supporting the older doubled-brace escapes already
    present in the template.

    Order matters:
      1. Replace known `{key}` placeholders with sentinels.
      2. Collapse existing `{{` / `}}` escapes in the template itself.
      3. Substitute the real values, so JSON payload braces are not modified.
    """

    rendered = template
    sentinels: dict[str, str] = {}

    for index, (key, value) in enumerate(values.items()):
        token = "{" + key + "}"
        sentinel = f"__DEDALUS_TEMPLATE_VALUE_{index}__"
        sentinels[sentinel] = str(value)
        rendered = rendered.replace(token, sentinel)

    rendered = rendered.replace("{{", "{").replace("}}", "}")

    for sentinel, value in sentinels.items():
        rendered = rendered.replace(sentinel, value)

    return rendered


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

    return render_html_template(HTML_TEMPLATE,
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
    main()
