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
                "min_z_m": as_float(raw.get("min_z_m"), math.nan),
                "max_z_m": as_float(raw.get("max_z_m"), math.nan),
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
  .view-controls {{ display: grid; grid-template-columns: 1fr 1fr; gap: 8px; margin: 12px 0 14px; }}
  .view-controls button {{ background: #222838; color: #e8e8e8; border: 1px solid #3a4358; border-radius: 6px; padding: 6px 8px; cursor: pointer; }}
  .view-controls button:hover {{ background: #2b3348; }}
  .height-legend {{ margin: 10px 0 12px; }}
  .height-ramp {{ height: 14px; border: 1px solid #3a4358; border-radius: 6px;
    background: linear-gradient(to right, #2641a8, #00a8d8, #3fbf6a, #e5d84c, #f08c2e, #d83b7d); }}
  .height-ticks {{ display: grid; grid-template-columns: repeat(5, 1fr); gap: 4px; color: #aab0bf; font-size: 12px; margin-top: 4px; }}
  .height-ticks span:nth-child(1) {{ text-align: left; }}
  .height-ticks span:nth-child(2), .height-ticks span:nth-child(3), .height-ticks span:nth-child(4) {{ text-align: center; }}
  .height-ticks span:nth-child(5) {{ text-align: right; }}
  .overlay-controls {{ display: grid; gap: 6px; margin: 10px 0 12px; color: #c8ccd8; }}
  .overlay-controls label {{ display: flex; align-items: center; gap: 8px; }}
  #hover-card {{ position: fixed; display: none; pointer-events: none; z-index: 10;
    max-width: 310px; padding: 9px 10px; border: 1px solid #3a4358; border-radius: 8px;
    background: rgba(15, 18, 27, 0.94); box-shadow: 0 6px 20px rgba(0,0,0,0.35);
    color: #e8e8e8; font-size: 12px; line-height: 1.35; }}
  #hover-card b {{ color: #ffffff; }}
  #hover-card code {{ color: #a7d4ff; word-break: break-word; }}
  #hover-card .muted {{ color: #aab0bf; }}
</style>
</head>
<body>
<div id="wrap">
  <aside id="side">
    <h2>Mission-local obstacle map</h2>
    <p class="hint">Drag to rotate. Wheel to zoom. Mission cells are rendered in map/takeoff-relative coordinates.</p>
    <div class="view-controls">
      <button id="view-center" type="button">Center</button>
      <button id="view-45" type="button">45°</button>
      <button id="view-side" type="button">Side</button>
      <button id="view-top" type="button">Top</button>
    </div>
    <div class="overlay-controls">
      <label><input id="toggle-sensing-overlay" type="checkbox" checked> Sensing direction overlay</label>
    </div>
    <div class="metric"><span>Snapshot</span><code>{snapshot_path}</code></div>
    <div class="metric"><span>Map frame</span><code>{map_frame}</code></div>
    <div class="metric"><span>Observed cells</span><b id="observed-count">{observed}</b></div>
    <div class="metric"><span>Occupied cells</span><b id="occupied-count">{occupied}</b></div>
    <div class="metric"><span>Cells shown</span><b id="cell-count">{debug_cells}</b></div>
    <div class="metric"><span>Raw evidence</span><b id="raw-evidence-count">{raw_evidence}</b></div>
    <div class="metric"><span>Compacted evidence</span><b id="compacted-evidence-count">{compacted_evidence}</b></div>
    <div class="metric"><span>Duplicate evidence</span><b id="duplicate-evidence-count">{duplicate_evidence}</b></div>
    <div class="metric"><span>Projected mission cells</span><b id="projected-mission-cell-count">{projected_mission_cells}</b></div>
    <div class="metric"><span>Exclusion cells</span><b id="exclusion-cell-count">{exclusion_cells}</b></div>
    <div class="metric"><span>Exclusion radius</span><b id="exclusion-radius">{exclusion_radius}</b></div>
    <div class="metric"><span>Trajectory points</span><b id="trajectory-count">{trajectory_points}</b></div>
    <div class="metric"><span>Bounds</span><code id="bounds-text">{bounds_text}</code></div>
    <div class="metric"><span>Live stream</span><code id="live-status">offline</code></div>
    <div class="metric"><span>Live delta cells</span><b id="live-delta-count">0</b></div>
    <div class="metric"><span>World snapshots</span><b id="world-snapshot-count">0</b></div>
    <div class="metric"><span>Ego updates</span><b id="ego-update-count">0</b></div>
    <div class="metric"><span>Sensing overlay</span><code id="sensing-overlay-status">{sensing_overlay_status}</code></div>
    <h3>Legend</h3>
    <div class="height-legend">
      <div class="hint">Obstacle color: height above takeoff / local origin</div>
      <div class="height-ramp" aria-label="Height color ramp from 0 to 40 meters"></div>
      <div class="height-ticks">
        <span>0m</span><span>10m</span><span>20m</span><span>30m</span><span>40m+</span>
      </div>
    </div>
    <p class="hint">
      Topo color: mission-cell / live evidence height, using AirSim/NED-style height = -Z<br>
      Opacity: confidence and live recency<br>
      Yellow: ego pose / path<br>
      White: true sensing volume, when published<br>
      Magenta: first blocked trajectory point, when present
    </p>
  </aside>
  <canvas id="canvas"></canvas>
  <div id="hover-card"></div>
</div>
<script>
const data = {data_json};
if (!Array.isArray(data.cells)) data.cells = [];
if (!Array.isArray(data.trajectory)) data.trajectory = [];
if (!Array.isArray(data.liveObstacleEvents)) data.liveObstacleEvents = [];
if (!Array.isArray(data.sensingOverlays)) data.sensingOverlays = [];
if (!data.diagnostics || typeof data.diagnostics !== "object") data.diagnostics = {};
data.showSensingOverlay = true;

const cellsByKey = new Map();
for (const cell of data.cells) {
  cellsByKey.set(cellKey(cell.center), normalizeCell(cell));
}
data.cells = Array.from(cellsByKey.values());

const LIVE_BRIGHT_HOLD_MS = 2000;
const LIVE_FADE_END_MS = 10000;
const LIVE_DECAY_MS = LIVE_FADE_END_MS;
const LIVE_OBSTACLE_DIM = 0.45;
const LIVE_TRACK_DIM = 0.55;
const LIVE_OBSTACLE_EVENT_MAX = 5000;
const LIVE_OBSTACLE_EVENT_GRID_M = 0.35;
const LIVE_PENDING_DELTA_CELL_MAX = 4096;

const liveObstacleEventsByKey = new Map();
for (const event of data.liveObstacleEvents) {
  if (event && event.key) liveObstacleEventsByKey.set(event.key, event);
}

const live = {
  enabled: false,
  connected: false,
  eventSource: null,
  eventCount: 0,
  deltaCellCount: 0,
  worldSnapshotCount: 0,
  egoUpdateCount: 0,
  lastSeq: null,
  error: "",
  pendingWorldSnapshot: null,
  pendingWorldSnapshotSeq: null,
  pendingDeltaCells: [],
  pendingDeltaSeq: null,
  processingScheduled: false,
  coalescedDeltaFrames: 0,
  droppedPendingDeltaCells: 0
};

function el(id) {
  return document.getElementById(id);
}

function escapeHtml(value) {
  return String(value)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#39;");
}

function finiteNumber(value, fallback = 0) {
  const n = Number(value);
  return Number.isFinite(n) ? n : fallback;
}

function metricNumber(value, fallback = 0) {
  const n = Number(value);
  return Number.isFinite(n) ? n : fallback;
}

function formatMetricNumber(value) {
  const n = metricNumber(value, 0);
  return Number.isInteger(n) ? String(n) : n.toFixed(2);
}

function formatMetricMeters(value) {
  const n = Number(value);
  if (!Number.isFinite(n)) return "n/a";
  return `${n.toFixed(2)} m`;
}

const HEIGHT_COLOR_MIN_M = 0.0;
const HEIGHT_COLOR_MAX_M = 40.0;
const HEIGHT_COLOR_BAND_M = 2.0;
const HEIGHT_COLOR_RAMP = [
  {v: 0.0, rgb: [38, 65, 168]},
  {v: 5.0, rgb: [0, 168, 216]},
  {v: 10.0, rgb: [63, 191, 106]},
  {v: 20.0, rgb: [229, 216, 76]},
  {v: 30.0, rgb: [240, 140, 46]},
  {v: 40.0, rgb: [216, 59, 125]}
];

function clampNumber(value, lo, hi) {
  return Math.max(lo, Math.min(hi, value));
}

function lerpNumber(a, b, t) {
  return a + ((b - a) * t);
}

function topoBandHeightM(heightM) {
  if (!Number.isFinite(heightM)) return HEIGHT_COLOR_MIN_M;
  return Math.round(heightM / HEIGHT_COLOR_BAND_M) * HEIGHT_COLOR_BAND_M;
}

function cellMapZ(cell) {
  const minZ = Number(cell.min_z_m);
  const maxZ = Number(cell.max_z_m);
  if (Number.isFinite(minZ) && Number.isFinite(maxZ)) {
    return 0.5 * (minZ + maxZ);
  }
  const center = asVec3(cell.center) || asVec3(cell.center_mission) || asVec3(cell.center_map);
  if (center && Number.isFinite(Number(center.z))) {
    return Number(center.z);
  }
  return 0.0;
}

function heightAboveTakeoffM(cell) {
  // AirSim / NED-style mission-local Z is negative above takeoff.
  return -cellMapZ(cell);
}

function rgbForHeight(heightM) {
  const h = clampNumber(topoBandHeightM(heightM), HEIGHT_COLOR_MIN_M, HEIGHT_COLOR_MAX_M);
  for (let i = 1; i < HEIGHT_COLOR_RAMP.length; ++i) {
    const lo = HEIGHT_COLOR_RAMP[i - 1];
    const hi = HEIGHT_COLOR_RAMP[i];
    if (h <= hi.v) {
      const span = Math.max(1e-6, hi.v - lo.v);
      const t = clampNumber((h - lo.v) / span, 0.0, 1.0);
      return [
        Math.round(lerpNumber(lo.rgb[0], hi.rgb[0], t)),
        Math.round(lerpNumber(lo.rgb[1], hi.rgb[1], t)),
        Math.round(lerpNumber(lo.rgb[2], hi.rgb[2], t))
      ];
    }
  }
  return HEIGHT_COLOR_RAMP[HEIGHT_COLOR_RAMP.length - 1].rgb;
}

function rgbaForHeight(heightM, alpha) {
  const rgb = rgbForHeight(heightM);
  return `rgba(${rgb[0]},${rgb[1]},${rgb[2]},${alpha})`;
}

function cellDisplayAlpha(cell) {
  if (cell.occupied) return 0.62;
  if (cell.free) return 0.30;
  return 0.38;
}

function asVec3(value) {
  if (Array.isArray(value)) {
    if (value.length < 2) return null;
    const x = finiteNumber(value[0], NaN);
    const y = finiteNumber(value[1], NaN);
    const z = finiteNumber(value.length >= 3 ? value[2] : 0, NaN);
    if (![x, y, z].every(Number.isFinite)) return null;
    return {x, y, z};
  }

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

function firstArrayNumber(obj, paths, index) {
  for (const path of paths) {
    const raw = getPath(obj, path);
    if (!Array.isArray(raw) || raw.length <= index) continue;
    const n = Number(raw[index]);
    if (Number.isFinite(n)) return n;
  }
  return null;
}

function normalizeVec3(v) {
  if (!v) return null;
  const length = Math.hypot(v.x, v.y, v.z);
  if (!Number.isFinite(length) || length <= 1e-6) return null;
  return {x: v.x / length, y: v.y / length, z: v.z / length};
}

function addVec3(a, b, scale = 1.0) {
  return {x: a.x + b.x * scale, y: a.y + b.y * scale, z: a.z + b.z * scale};
}

function snapshotSensingOverlays(snapshot) {
  if (!snapshot || typeof snapshot !== "object") return [];

  const candidates = [
    snapshot.obstacle_sensing_volumes,
    snapshot.sensing_volumes,
    snapshot.camera_sensing_volumes,
    snapshot.depth_sensing_volumes,
    snapshot.sensing_coverage && snapshot.sensing_coverage.volumes
  ];

  const overlays = [];
  for (const candidate of candidates) {
    if (!Array.isArray(candidate)) continue;
    for (const raw of candidate) {
      if (!raw || typeof raw !== "object") continue;
      const origin =
        asVec3(raw.origin_local) ||
        asVec3(raw.origin) ||
        asVec3(raw.camera_origin_local) ||
        asVec3(raw.position_local) ||
        asVec3(raw.position);
      const forward =
        normalizeVec3(asVec3(raw.forward_axis_local)) ||
        normalizeVec3(asVec3(raw.forward)) ||
        normalizeVec3(asVec3(raw.direction_local)) ||
        normalizeVec3(asVec3(raw.direction));
      if (!origin || !forward) continue;

      overlays.push({
        kind: "true_sensing_volume",
        label: String(raw.camera_id || raw.sensor_id || raw.id || "sensing volume"),
        origin,
        forward,
        range_m: Math.max(0.25, finiteNumber(raw.range_m || raw.max_range_m || raw.far_m, 8.0))
      });
    }
  }

  return overlays;
}

function sensingOverlayStatus() {
  const count = Array.isArray(data.sensingOverlays) ? data.sensingOverlays.length : 0;
  if (count > 0) return `${count} true volume${count === 1 ? "" : "s"}`;
  return "unavailable";
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
  if (direct !== null) return direct;

  const arrayYaw = firstArrayNumber(snapshot, [
    ["ego_state", "local_T_body", "rotation_rpy"],
    ["ego_state", "pose", "rotation_rpy"],
    ["ego_state", "rotation_rpy"],
    ["ego", "local_T_body", "rotation_rpy"],
    ["ego", "pose", "rotation_rpy"],
    ["ego", "rotation_rpy"],
    ["vehicle", "rotation_rpy"],
    ["drone", "rotation_rpy"],
    ["agent", "rotation_rpy"],
    ["ownship", "rotation_rpy"]
  ], 2);
  if (arrayYaw !== null) return arrayYaw;

  return recursiveYawSearch(snapshot);
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

function liveEventKey(center) {
  if (!center) return "missing";
  const g = LIVE_OBSTACLE_EVENT_GRID_M;
  return [
    Math.round(finiteNumber(center.x) / g),
    Math.round(finiteNumber(center.y) / g),
    Math.round(finiteNumber(center.z) / g)
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
    min_z_m: finiteNumber(cell.min_z_m, NaN),
    max_z_m: finiteNumber(cell.max_z_m, NaN),
    stable_color: cell.stable_color || null,
    live_color: cell.live_color || null,
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
  if (Array.isArray(data.sensingOverlays)) {
    for (const overlay of data.sensingOverlays) {
      if (overlay && overlay.origin) points.push(overlay.origin);
      if (overlay && overlay.origin && overlay.forward) {
        points.push(addVec3(overlay.origin, overlay.forward, finiteNumber(overlay.range_m, 8.0)));
      }
    }
  }
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

let drawScheduled = false;

function scheduleDraw() {
  if (drawScheduled) return;
  drawScheduled = true;
  requestAnimationFrame(() => {
    drawScheduled = false;
    draw();
  });
}

function updateMetrics() {
  const observedCount = cellsByKey.size;
  const occupiedCount = data.cells.filter((cell) => cell.occupied).length;
  const b = data.bounds;
  if (el("observed-count")) el("observed-count").textContent = String(observedCount);
  if (el("occupied-count")) el("occupied-count").textContent = String(occupiedCount);
  if (el("cell-count")) el("cell-count").textContent = String(data.cells.length);
  if (el("trajectory-count")) el("trajectory-count").textContent = String(data.trajectory.length);
  const d = data.diagnostics || {};
  if (el("raw-evidence-count")) el("raw-evidence-count").textContent = formatMetricNumber(d.raw_evidence_count);
  if (el("compacted-evidence-count")) el("compacted-evidence-count").textContent = formatMetricNumber(d.compacted_evidence_count);
  if (el("duplicate-evidence-count")) el("duplicate-evidence-count").textContent = formatMetricNumber(d.duplicate_evidence_count);
  if (el("projected-mission-cell-count")) {
    el("projected-mission-cell-count").textContent = formatMetricNumber(d.projected_mission_cell_count);
  }
  if (el("exclusion-cell-count")) el("exclusion-cell-count").textContent = formatMetricNumber(d.inflated_blocked_count);
  if (el("exclusion-radius")) el("exclusion-radius").textContent = formatMetricMeters(d.exclusion_inflation_radius_m);
  if (el("live-delta-count")) el("live-delta-count").textContent = String(live.deltaCellCount);
  if (el("world-snapshot-count")) el("world-snapshot-count").textContent = String(live.worldSnapshotCount);
  if (el("ego-update-count")) el("ego-update-count").textContent = String(live.egoUpdateCount);
  if (el("sensing-overlay-status")) el("sensing-overlay-status").textContent = sensingOverlayStatus();
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

function applyMissionObstacleMapDelta(delta, seq, options = {}) {
  if (!delta || typeof delta !== "object" || !Array.isArray(delta.cells)) return 0;

  let changed = 0;
  const nowMs = Date.now();

  for (const raw of delta.cells) {
    const cell = normalizeCell(raw);
    if (!cell.center) continue;

    cellsByKey.set(cellKey(cell.center), cell);

    const eventKey = liveEventKey(cell.center);
    const event = {
      key: eventKey,
      center: cell.center,
      occupied_score: cell.occupied_score,
      free_score: cell.free_score,
      risk_score: cell.risk_score,
      confidence: cell.confidence,
      min_z_m: cell.min_z_m,
      max_z_m: cell.max_z_m,
      stable_color: stableCellColor(cell),
      live_color: cell.live_color || null,
      live_seen_ms: nowMs
    };

    const existing = liveObstacleEventsByKey.get(eventKey);
    if (existing) {
      Object.assign(existing, event);
    } else {
      liveObstacleEventsByKey.set(eventKey, event);
      data.liveObstacleEvents.push(event);
    }

    changed += 1;
  }

  if (data.liveObstacleEvents.length > LIVE_OBSTACLE_EVENT_MAX) {
    const removed = data.liveObstacleEvents.splice(0, data.liveObstacleEvents.length - LIVE_OBSTACLE_EVENT_MAX);
    for (const event of removed) {
      if (event && event.key) liveObstacleEventsByKey.delete(event.key);
    }
  }

  data.cells = Array.from(cellsByKey.values());
  live.deltaCellCount += changed;
  live.lastSeq = seq;

  if (!options.deferRender) {
    updateMetrics();
    scheduleDraw();
  }

  return changed;
}

function applyWorldSnapshot(snapshot, seq, options = {}) {
  if (!snapshot || typeof snapshot !== "object") return false;

  let changed = false;
  const ego = snapshotEgoPosition(snapshot);
  if (ego) {
    live.egoUpdateCount += 1;
    data.ego = ego;
    changed = true;
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
    changed = true;
  }

  const sensingOverlays = snapshotSensingOverlays(snapshot);
  if (sensingOverlays.length > 0) {
    data.sensingOverlays = sensingOverlays;
    changed = true;
  }

  const blocked = snapshotFirstBlocked(snapshot);
  if (blocked) {
    data.first_blocked = blocked;
    changed = true;
  }

  const mission = snapshot.mission_local_obstacle_map;
  if (mission && typeof mission === "object") {
    data.diagnostics.raw_evidence_count = metricNumber(mission.raw_evidence_count);
    data.diagnostics.accepted_evidence_count = metricNumber(mission.accepted_evidence_count);
    data.diagnostics.compacted_evidence_count = metricNumber(mission.compacted_evidence_count);
    data.diagnostics.duplicate_evidence_count = metricNumber(mission.duplicate_evidence_count);
    data.diagnostics.dropped_evidence_count = metricNumber(mission.dropped_evidence_count);
    data.diagnostics.new_cell_count = metricNumber(mission.new_cell_count);
    data.diagnostics.updated_cell_count = metricNumber(mission.updated_cell_count);
    changed = true;
  }

  const localFlight = snapshot.local_flight_map;
  if (localFlight && typeof localFlight === "object") {
    data.diagnostics.source_mission_cell_count = metricNumber(localFlight.source_mission_cell_count);
    data.diagnostics.projected_mission_cell_count = metricNumber(localFlight.projected_mission_cell_count);
    data.diagnostics.projected_local_cell_update_count = metricNumber(localFlight.projected_local_cell_update_count);
    data.diagnostics.inflated_blocked_count = metricNumber(localFlight.inflated_blocked_count);
    data.diagnostics.exclusion_inflation_radius_m = metricNumber(localFlight.exclusion_inflation_radius_m, NaN);
    changed = true;
  }

  live.lastSeq = seq;

  if (!options.deferRender) {
    updateMetrics();
    scheduleDraw();
  }

  return changed;
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

function flushLiveEventRenderUpdates() {
  updateMetrics();
  scheduleDraw();
}

function scheduleLiveEventProcessing() {
  if (live.processingScheduled) return;
  live.processingScheduled = true;
  requestAnimationFrame(processPendingLiveEvents);
}

function processPendingLiveEvents() {
  live.processingScheduled = false;

  const pendingCells = live.pendingDeltaCells;
  const pendingDeltaSeq = live.pendingDeltaSeq;
  live.pendingDeltaCells = [];
  live.pendingDeltaSeq = null;

  const pendingSnapshot = live.pendingWorldSnapshot;
  const pendingSnapshotSeq = live.pendingWorldSnapshotSeq;
  live.pendingWorldSnapshot = null;
  live.pendingWorldSnapshotSeq = null;

  let changed = false;
  if (pendingCells.length > 0) {
    changed = applyMissionObstacleMapDelta({cells: pendingCells}, pendingDeltaSeq, {deferRender: true}) > 0 || changed;
  }
  if (pendingSnapshot) {
    changed = applyWorldSnapshot(pendingSnapshot, pendingSnapshotSeq, {deferRender: true}) || changed;
  }

  if (changed) {
    flushLiveEventRenderUpdates();
  } else {
    updateMetrics();
  }

  if ((live.pendingDeltaCells.length > 0 || live.pendingWorldSnapshot) && !live.processingScheduled) {
    scheduleLiveEventProcessing();
  }
}

function enqueueMissionObstacleMapDelta(payload) {
  const delta = payload && payload.mission_obstacle_map_delta;
  if (!delta || typeof delta !== "object" || !Array.isArray(delta.cells)) return;

  live.coalescedDeltaFrames += 1;
  live.pendingDeltaSeq = payload.seq ?? live.pendingDeltaSeq;

  for (const raw of delta.cells) {
    live.pendingDeltaCells.push(raw);
  }

  if (live.pendingDeltaCells.length > LIVE_PENDING_DELTA_CELL_MAX) {
    const dropped = live.pendingDeltaCells.length - LIVE_PENDING_DELTA_CELL_MAX;
    live.pendingDeltaCells.splice(0, dropped);
    live.droppedPendingDeltaCells += dropped;
  }

  scheduleLiveEventProcessing();
}

function enqueueWorldSnapshot(payload) {
  const snapshot = payload && (payload.world_snapshot || payload.snapshot);
  if (!snapshot || typeof snapshot !== "object") return;

  live.worldSnapshotCount += 1;
  live.pendingWorldSnapshot = snapshot;
  live.pendingWorldSnapshotSeq = payload.seq ?? null;
  scheduleLiveEventProcessing();
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
      enqueueMissionObstacleMapDelta(JSON.parse(event.data));
    } catch (err) {
      live.error = String(err);
      updateMetrics();
    }
  });

  source.addEventListener("world_snapshot", (event) => {
    try {
      enqueueWorldSnapshot(JSON.parse(event.data));
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
  // AirSim/NED-style local Y is mirrored in canvas space; flip it so orbit
  // handedness matches the simulator view.
  let y = -(p.y - c.y);
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

function drawDashedLine(a, b, color, width = 1) {{
  const pa = project(a);
  const pb = project(b);
  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = width * window.devicePixelRatio;
  ctx.setLineDash([8 * window.devicePixelRatio, 6 * window.devicePixelRatio]);
  ctx.beginPath();
  ctx.moveTo(pa.x, pa.y);
  ctx.lineTo(pb.x, pb.y);
  ctx.stroke();
  ctx.restore();
}}

function drawAxes() {{
  const length = 0.25 * radius();
  const origin = center();
  drawLine(origin, {{x: origin.x + length, y: origin.y, z: origin.z}}, "#ff6b6b", 2);
  drawLine(origin, {{x: origin.x, y: origin.y + length, z: origin.z}}, "#7bed9f", 2);
  drawLine(origin, {{x: origin.x, y: origin.y, z: origin.z + length}}, "#70a1ff", 2);
}}

function drawSensingOverlays() {{
  if (!data.showSensingOverlay) return;

  const overlays = Array.isArray(data.sensingOverlays) ? data.sensingOverlays : [];
  if (overlays.length > 0) {{
    for (const overlay of overlays) {{
      if (!overlay || !overlay.origin || !overlay.forward) continue;
      const range = Math.max(0.25, finiteNumber(overlay.range_m, 8.0));
      const tip = addVec3(overlay.origin, overlay.forward, range);
      drawLine(overlay.origin, tip, "rgba(0, 0, 0, 0.90)", 5.0);
      drawLine(overlay.origin, tip, "rgba(255, 255, 255, 0.96)", 2.8);
      drawPoint(overlay.origin, "rgba(0, 0, 0, 0.90)", 6.0);
      drawPoint(overlay.origin, "rgba(255, 255, 255, 0.96)", 3.8);
    }}
    return;
  }}

  // Do not draw a fallback camera/sensing vector from ego yaw.
  // Without true camera/frustum data, this duplicates the drone yaw vector and
  // can be mistaken for the actual depth-camera direction.
}}

function drawPoint(p, color, r) {{
  const pp = project(p);
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(pp.x, pp.y, r * window.devicePixelRatio, 0, Math.PI * 2);
  ctx.fill();
}}

function drawDroneMarker() {{
  if (!data.ego) return;

  drawPoint(data.ego, "rgba(0, 0, 0, 0.92)", 10.5);
  drawPoint(data.ego, "rgba(255, 255, 255, 1.0)", 8.5);
  drawPoint(data.ego, "rgba(255, 230, 64, 1.0)", 5.8);

  if (typeof data.ego_yaw === "number") {{
    const l = 2.4;
    const tip = {{
      x: data.ego.x + l * Math.cos(data.ego_yaw),
      y: data.ego.y + l * Math.sin(data.ego_yaw),
      z: data.ego.z
    }};
    drawLine(data.ego, tip, "rgba(0, 0, 0, 0.90)", 6.0);
    drawLine(data.ego, tip, "rgba(255, 255, 255, 1.0)", 3.2);
  }}
}}

function cellStateLabel(cell) {
  if (cell.occupied) return "occupied";
  if (cell.free) return "free";
  return "unknown";
}

function scoreText(value) {
  const n = Number(value);
  if (!Number.isFinite(n)) return "n/a";
  return n.toFixed(2);
}

function hoverCellHtml(cell) {
  const heightM = heightAboveTakeoffM(cell);
  const zM = cellMapZ(cell);
  const provider = cell.last_source_provider || "n/a";
  const kind = cell.last_source_kind || "n/a";
  const source = `${kind}${provider === "n/a" ? "" : " / " + provider}`;

  return [
    `<b>Mission-local obstacle cell</b>`,
    `<div>state: <code>${escapeHtml(cellStateLabel(cell))}</code></div>`,
    `<div>height above takeoff: <code>${scoreText(heightM)} m</code></div>`,
    `<div>map Z: <code>${scoreText(zM)} m</code> <span class="muted">(AirSim/NED height = -Z)</span></div>`,
    `<div>occupied score: <code>${scoreText(cell.occupied_score)}</code></div>`,
    `<div>free score: <code>${scoreText(cell.free_score)}</code></div>`,
    `<div>risk score: <code>${scoreText(cell.risk_score)}</code></div>`,
    `<div>confidence: <code>${scoreText(cell.confidence)}</code></div>`,
    `<div>source: <code>${escapeHtml(source)}</code></div>`,
    `<div class="muted">x=${scoreText(cell.center.x)}, y=${scoreText(cell.center.y)}, z=${scoreText(cell.center.z)}</div>`
  ].join("");
}

function canvasMousePoint(event) {
  const rect = canvas.getBoundingClientRect();
  return {
    x: (event.clientX - rect.left) * window.devicePixelRatio,
    y: (event.clientY - rect.top) * window.devicePixelRatio
  };
}

function nearestCellToCanvasPoint(point) {
  let best = null;
  let bestDistance = Infinity;
  const threshold = 14.0 * window.devicePixelRatio;

  for (const cell of data.cells) {
    if (!cell || !cell.center || cell.live_seen_ms) continue;
    const projected = project(cell.center);
    if (!projected || !Number.isFinite(projected.x) || !Number.isFinite(projected.y)) continue;
    const distance = Math.hypot(projected.x - point.x, projected.y - point.y);
    if (distance < bestDistance) {
      bestDistance = distance;
      best = cell;
    }
  }

  return bestDistance <= threshold ? best : null;
}

function hideHoverCard() {
  const card = el("hover-card");
  if (!card) return;
  card.style.display = "none";
}

function updateHoverCard(event) {
  const card = el("hover-card");
  if (!card || dragging) {
    hideHoverCard();
    return;
  }

  const cell = nearestCellToCanvasPoint(canvasMousePoint(event));
  if (!cell) {
    hideHoverCard();
    return;
  }

  card.innerHTML = hoverCellHtml(cell);
  card.style.left = `${event.clientX + 14}px`;
  card.style.top = `${event.clientY + 14}px`;
  card.style.display = "block";
}

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
    drawPoint(cell.center, stableCellColor(cell), r);
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

  drawDroneMarker();

  if (data.first_blocked) {{
    drawPoint(data.first_blocked, "rgba(255, 80, 255, 1.0)", 8);
  }}
}}

function setViewPreset(nextYaw, nextPitch, nextZoom = null) {{
  yaw = nextYaw;
  pitch = nextPitch;
  if (nextZoom !== null) zoom = nextZoom;
  draw();
}}

function installViewControls() {{
  const centerButton = el("view-center");
  const angleButton = el("view-45");
  const sideButton = el("view-side");
  const topButton = el("view-top");

  if (centerButton) centerButton.addEventListener("click", () => {{
    zoom = 1.0;
    draw();
  }});

  if (angleButton) angleButton.addEventListener("click", () => {{
    setViewPreset(-Math.PI / 4, Math.PI / 4, 1.0);
  }});

  if (sideButton) sideButton.addEventListener("click", () => {{
    setViewPreset(0, 0, 1.0);
  }});

  if (topButton) topButton.addEventListener("click", () => {{
    setViewPreset(0, Math.PI / 2, 1.0);
  }});

  const sensingToggle = el("toggle-sensing-overlay");
  if (sensingToggle) sensingToggle.addEventListener("change", () => {{
    data.showSensingOverlay = sensingToggle.checked;
    draw();
  }});
}}

canvas.addEventListener("mousedown", (e) => {{ dragging = true; lastX = e.clientX; lastY = e.clientY; hideHoverCard(); }});
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
canvas.addEventListener("mousemove", updateHoverCard);
canvas.addEventListener("mouseleave", hideHoverCard);
canvas.addEventListener("wheel", (e) => {{
  e.preventDefault();
  hideHoverCard();
  zoom *= Math.exp(-e.deltaY * 0.001);
  zoom = Math.max(0.1, Math.min(20.0, zoom));
  draw();
}}, {{passive: false}});


function liveDecayLevel(seenMs, dimValue) {
  if (!seenMs) return null;
  const ageMs = Math.max(0, Date.now() - Number(seenMs));
  if (ageMs <= LIVE_BRIGHT_HOLD_MS) return 1.0;
  if (ageMs >= LIVE_FADE_END_MS) return dimValue;
  const t = (ageMs - LIVE_BRIGHT_HOLD_MS) / (LIVE_FADE_END_MS - LIVE_BRIGHT_HOLD_MS);
  return 1 - ((1 - dimValue) * t);
}

function stableCellColor(cell) {
  if (!cell.stable_color) {
    cell.stable_color = rgbaForHeight(heightAboveTakeoffM(cell), cellDisplayAlpha(cell));
  }
  return cell.stable_color;
}

function stableLiveColor(event) {
  if (!event.live_color) {
    event.live_color = event.stable_color || rgbaForHeight(heightAboveTakeoffM(event), 0.88);
  }
  return event.live_color;
}

function liveYellow(level, alpha = 0.92) {
  const gb = Math.round(255 * level);
  return `rgba(255,${gb},0,${alpha})`;
}

function pruneLiveObstacleEvents() {
  const cutoffMs = Date.now() - 60000;
  if (!Array.isArray(data.liveObstacleEvents)) {
    data.liveObstacleEvents = [];
    return;
  }
  if (data.liveObstacleEvents.length > 12000) {
    data.liveObstacleEvents.splice(0, data.liveObstacleEvents.length - 12000);
  }
  data.liveObstacleEvents = data.liveObstacleEvents.filter((event) => {
    return event && event.center && Number(event.live_seen_ms) >= cutoffMs;
  });
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

  const obstacleEvents = Array.isArray(data.liveObstacleEvents) ? data.liveObstacleEvents : [];
  for (const event of obstacleEvents) {
    const level = liveDecayLevel(event.live_seen_ms, LIVE_OBSTACLE_DIM);
    if (level === null) continue;

    const score = Math.max(event.occupied_score || 0, event.free_score || 0, event.risk_score || 0);
    const radius = Math.max(2.0, Math.min(8.0, 3.0 + score * 0.5)) * Math.max(0.8, level);
    drawLivePoint(event.center, stableLiveColor(event), radius);
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

  if (live.enabled && live.connected) {
    requestAnimationFrame(() => {
      const hasRecentObstacle = data.liveObstacleEvents.some((event) => event.live_seen_ms && now - Number(event.live_seen_ms) < LIVE_FADE_END_MS);
      const hasRecentTrack = data.trajectory.some((point) => point.live_seen_ms && now - Number(point.live_seen_ms) < LIVE_FADE_END_MS);
      if (hasRecentObstacle || hasRecentTrack) {
        scheduleDraw();
      }
    });
  }
}

const baseDraw = draw;
draw = function() {
  baseDraw();
  drawLiveAgingOverlay();
  drawSensingOverlays();
  drawDroneMarker();
};


window.addEventListener("resize", resize);
installViewControls();
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


def sensing_overlays(snapshot: dict[str, Any]) -> list[dict[str, Any]]:
    """Extract optional true sensing/camera volumes if future snapshots publish them.

    Current snapshots at bb2d836 do not expose a first-class camera/frustum
    payload. This intentionally accepts several likely future keys, but returns
    an empty list unless true origin + forward-axis data exists.
    """

    candidate_arrays = [
        snapshot.get("obstacle_sensing_volumes"),
        snapshot.get("sensing_volumes"),
        snapshot.get("camera_sensing_volumes"),
        snapshot.get("depth_sensing_volumes"),
    ]

    sensing_coverage = snapshot.get("sensing_coverage")
    if isinstance(sensing_coverage, dict):
        candidate_arrays.append(sensing_coverage.get("volumes"))

    overlays: list[dict[str, Any]] = []
    for candidate in candidate_arrays:
        if not isinstance(candidate, list):
            continue
        for raw in candidate:
            if not isinstance(raw, dict):
                continue
            origin = (
                vec3(raw.get("origin_local"))
                or vec3(raw.get("origin"))
                or vec3(raw.get("camera_origin_local"))
                or vec3(raw.get("position_local"))
                or vec3(raw.get("position"))
            )
            forward = (
                vec3(raw.get("forward_axis_local"))
                or vec3(raw.get("forward"))
                or vec3(raw.get("direction_local"))
                or vec3(raw.get("direction"))
            )
            if origin is None or forward is None:
                continue
            overlays.append(
                {
                    "kind": "true_sensing_volume",
                    "label": str(raw.get("camera_id") or raw.get("sensor_id") or raw.get("id") or "sensing volume"),
                    "origin": origin,
                    "forward": forward,
                    "range_m": as_float(raw.get("range_m") or raw.get("max_range_m") or raw.get("far_m"), 8.0),
                }
            )

    return overlays


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
    local_flight_map = snapshot.get("local_flight_map")
    if not isinstance(local_flight_map, dict):
        local_flight_map = {}

    diagnostics = {
        "raw_evidence_count": mission.get("raw_evidence_count", 0),
        "accepted_evidence_count": mission.get("accepted_evidence_count", 0),
        "compacted_evidence_count": mission.get("compacted_evidence_count", 0),
        "duplicate_evidence_count": mission.get("duplicate_evidence_count", 0),
        "dropped_evidence_count": mission.get("dropped_evidence_count", 0),
        "new_cell_count": mission.get("new_cell_count", 0),
        "updated_cell_count": mission.get("updated_cell_count", 0),
        "source_mission_cell_count": local_flight_map.get("source_mission_cell_count", 0),
        "projected_mission_cell_count": local_flight_map.get("projected_mission_cell_count", 0),
        "projected_local_cell_update_count": local_flight_map.get("projected_local_cell_update_count", 0),
        "inflated_blocked_count": local_flight_map.get("inflated_blocked_count", 0),
        "exclusion_inflation_radius_m": local_flight_map.get("exclusion_inflation_radius_m", None),
    }

    overlays = sensing_overlays(snapshot)

    data = {
        "cells": cells,
        "trajectory": trajectory,
        "ego": ego,
        "ego_yaw": yaw,
        "first_blocked": first_blocked,
        "bounds": b,
        "diagnostics": diagnostics,
        "sensingOverlays": overlays,
    }

    return render_html_template(HTML_TEMPLATE,
        snapshot_path=html.escape(str(snapshot_path)),
        map_frame=html.escape(str(mission.get("map_frame_id", ""))),
        observed=html.escape(str(mission.get("observed_cell_count", 0))),
        occupied=html.escape(str(mission.get("occupied_cell_count", 0))),
        debug_cells=html.escape(str(len(cells))),
        raw_evidence=html.escape(str(diagnostics.get("raw_evidence_count", 0))),
        compacted_evidence=html.escape(str(diagnostics.get("compacted_evidence_count", 0))),
        duplicate_evidence=html.escape(str(diagnostics.get("duplicate_evidence_count", 0))),
        projected_mission_cells=html.escape(str(diagnostics.get("projected_mission_cell_count", 0))),
        exclusion_cells=html.escape(str(diagnostics.get("inflated_blocked_count", 0))),
        exclusion_radius=html.escape(
            "n/a" if diagnostics.get("exclusion_inflation_radius_m") is None
            else f"{as_float(diagnostics.get('exclusion_inflation_radius_m')):.2f} m"
        ),
        sensing_overlay_status=html.escape(
            f"{len(overlays)} true volume{'s' if len(overlays) != 1 else ''}"
            if overlays else "unavailable"
        ),
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
