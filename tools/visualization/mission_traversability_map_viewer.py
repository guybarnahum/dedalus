#!/usr/bin/env python3
"""Render foundational mission-local traversability-map artifacts.

The viewer consumes the S4A
`dedalus.mission_local_traversability_map.v1` JSON artifact and writes a
self-contained HTML canvas viewer. When the HTML is served from the same static
root as the artifact, `--artifact-url` enables polling so the display updates as
new live/final map artifacts are written.
"""

from __future__ import annotations

import argparse
import html
import json
import math
from pathlib import Path
from typing import Any


SCHEMA = "dedalus.mission_local_traversability_map.v1"


def as_float(value: Any, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def as_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def vec3(value: Any) -> dict[str, float] | None:
    if isinstance(value, dict) and {"x", "y", "z"}.issubset(value.keys()):
        return {"x": as_float(value["x"]), "y": as_float(value["y"]), "z": as_float(value["z"])}
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        return {"x": as_float(value[0]), "y": as_float(value[1]), "z": as_float(value[2])}
    return None


def finite_or_none(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def load_artifact(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"artifact is not a JSON object: {path}")
    schema = data.get("schema")
    if schema != SCHEMA:
        raise ValueError(f"unsupported traversability map schema {schema!r}; expected {SCHEMA!r}")
    return data


def normalize_cell(raw: dict[str, Any]) -> dict[str, Any] | None:
    center = vec3(raw.get("center_map"))
    if center is None:
        return None
    size = vec3(raw.get("size_m")) or {"x": 0.5, "y": 0.5, "z": 0.5}
    costs = raw.get("costs") if isinstance(raw.get("costs"), dict) else {}
    state = str(raw.get("state") or "unknown")
    total_cost = as_float(costs.get("total_traversability_cost"), 0.0)
    clearance = finite_or_none(raw.get("nearest_obstacle_distance_m"))
    clearance_margin = finite_or_none(raw.get("clearance_margin_m"))
    vertical_clearance_up = finite_or_none(raw.get("vertical_clearance_up_m"))
    overhead_cost = as_float(costs.get("overhead_cost"), 0.0)
    stale = bool(raw.get("stale", False)) or state == "stale"
    return {
        "center": center,
        "size": size,
        "state": state,
        "occupied_score": as_float(raw.get("occupied_score"), 0.0),
        "free_score": as_float(raw.get("free_score"), 0.0),
        "unknown_score": as_float(raw.get("unknown_score"), 0.0),
        "confidence": as_float(raw.get("confidence"), 0.0),
        "age_score": as_float(raw.get("age_score"), 0.0),
        "stability_score": as_float(raw.get("stability_score"), 0.0),
        "volatility_score": as_float(raw.get("volatility_score"), 0.0),
        "nearest_obstacle_distance_m": clearance,
        "clearance_margin_m": clearance_margin,
        "vertical_clearance_up_m": vertical_clearance_up,
        "vertical_clearance_down_m": finite_or_none(raw.get("vertical_clearance_down_m")),
        "occupied_cost": as_float(costs.get("occupied_cost"), 0.0),
        "proximity_cost": as_float(costs.get("proximity_cost"), 0.0),
        "unknown_cost": as_float(costs.get("unknown_cost"), 0.0),
        "stale_cost": as_float(costs.get("stale_cost"), 0.0),
        "overhead_cost": overhead_cost,
        "thin_structure_cost": as_float(costs.get("thin_structure_cost"), 0.0),
        "total_cost": total_cost,
        "stale": stale,
        "low_clearance": clearance_margin is not None and clearance_margin < 0.5,
        "overhead_risk": overhead_cost > 0.05,
        "occupied_hits_capped": as_int(raw.get("occupied_hits_capped"), 0),
        "free_rays_capped": as_int(raw.get("free_rays_capped"), 0),
        "conflict_count_capped": as_int(raw.get("conflict_count_capped"), 0),
        "refresh_count_capped": as_int(raw.get("refresh_count_capped"), 0),
        "first_observed_timestamp_ns": as_int(raw.get("first_observed_timestamp_ns"), 0),
        "last_observed_timestamp_ns": as_int(raw.get("last_observed_timestamp_ns"), 0),
    }


def artifact_payload(artifact: dict[str, Any], max_cells: int) -> dict[str, Any]:
    raw_cells = artifact.get("cells")
    if not isinstance(raw_cells, list):
        raw_cells = []
    cells: list[dict[str, Any]] = []
    for raw in raw_cells:
        if not isinstance(raw, dict):
            continue
        cell = normalize_cell(raw)
        if cell is not None:
            cells.append(cell)
    if max_cells > 0 and len(cells) > max_cells:
        cells = cells[:max_cells]

    summary = artifact.get("summary") if isinstance(artifact.get("summary"), dict) else {}
    export_summary = artifact.get("export_summary") if isinstance(artifact.get("export_summary"), dict) else {}
    config = artifact.get("config") if isinstance(artifact.get("config"), dict) else {}
    return {
        "schema": artifact.get("schema", ""),
        "site_id": artifact.get("site_id", ""),
        "site_frame_id": artifact.get("site_frame_id", ""),
        "mission_id": artifact.get("mission_id", ""),
        "map_frame_id": artifact.get("map_frame_id", ""),
        "first_update_timestamp_ns": artifact.get("first_update_timestamp_ns", 0),
        "last_update_timestamp_ns": artifact.get("last_update_timestamp_ns", 0),
        "config": config,
        "summary": summary,
        "export_summary": export_summary,
        "cells": cells,
        "source_cell_count": len(raw_cells),
        "shown_cell_count": len(cells),
    }


HTML_TEMPLATE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Dedalus foundational traversability map</title>
<style>
  body { margin: 0; background: #0f1117; color: #e8e8e8; font: 14px system-ui, sans-serif; }
  #wrap { display: grid; grid-template-columns: 340px 1fr; height: 100vh; }
  #side { padding: 16px; border-right: 1px solid #2b2f3a; overflow: auto; background: #151923; }
  #canvas { width: 100%; height: 100%; display: block; background: radial-gradient(circle at center, #1b2130, #0f1117); }
  code { color: #a7d4ff; word-break: break-all; }
  .metric { display: grid; grid-template-columns: 1fr auto; gap: 8px; padding: 4px 0; border-bottom: 1px solid #252a36; }
  .hint { color: #aab0bf; line-height: 1.4; }
  .controls { display: grid; gap: 6px; margin: 10px 0 14px; color: #c8ccd8; }
  .controls label { display: flex; align-items: center; gap: 8px; }
  .view-controls { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; margin: 12px 0 14px; }
  .view-controls button { background: #222838; color: #e8e8e8; border: 1px solid #3a4358; border-radius: 6px; padding: 6px 8px; cursor: pointer; }
  .view-controls button:hover { background: #2b3348; }
  .ramp { height: 14px; border: 1px solid #3a4358; border-radius: 6px; background: linear-gradient(to right, #394050, #3fbf6a, #e5d84c, #f08c2e, #d83b7d); }
  #hover-card { position: fixed; display: none; pointer-events: none; z-index: 10; max-width: 330px; padding: 9px 10px; border: 1px solid #3a4358; border-radius: 8px; background: rgba(15, 18, 27, 0.94); box-shadow: 0 6px 20px rgba(0,0,0,0.35); color: #e8e8e8; font-size: 12px; line-height: 1.35; }
  #hover-card b { color: #fff; }
  #hover-card .muted { color: #aab0bf; }
</style>
</head>
<body>
<div id="wrap">
  <aside id="side">
    <h2>Foundational traversability map</h2>
    <p class="hint">Classless compact map for planning diagnostics. This is not a command sink and not a reflexive safety dependency.</p>
    <div class="view-controls">
      <button id="view-center" type="button">Center</button>
      <button id="view-45" type="button">45°</button>
      <button id="view-side" type="button">Side</button>
      <button id="view-top" type="button">Top</button>
    </div>
    <div class="metric"><span>Artifact</span><code>__ARTIFACT_PATH__</code></div>
    <div class="metric"><span>Poll URL</span><code id="poll-url">__ARTIFACT_URL__</code></div>
    <div class="metric"><span>Poll status</span><code id="poll-status">offline</code></div>
    <div class="metric"><span>Map frame</span><code id="map-frame"></code></div>
    <div class="metric"><span>Cells shown</span><b id="shown-count">0</b></div>
    <div class="metric"><span>Occupied</span><b id="occupied-count">0</b></div>
    <div class="metric"><span>Free</span><b id="free-count">0</b></div>
    <div class="metric"><span>Mixed</span><b id="mixed-count">0</b></div>
    <div class="metric"><span>Stale</span><b id="stale-count">0</b></div>
    <div class="metric"><span>Low clearance</span><b id="low-clearance-count">0</b></div>
    <div class="metric"><span>Overhead risk</span><b id="overhead-risk-count">0</b></div>
    <div class="metric"><span>Minimum clearance</span><b id="min-clearance">n/a</b></div>
    <div class="metric"><span>Last update</span><code id="last-update">0</code></div>
    <div class="metric"><span>Bounds</span><code id="bounds-text">n/a</code></div>
    <h3>Layers</h3>
    <div class="controls">
      <label><input id="show-occupied" type="checkbox" checked> Occupied</label>
      <label><input id="show-free" type="checkbox" checked> Observed free</label>
      <label><input id="show-mixed" type="checkbox" checked> Mixed / conflict</label>
      <label><input id="show-stale" type="checkbox" checked> Stale</label>
      <label><input id="show-low-clearance" type="checkbox" checked> Low clearance rings</label>
      <label><input id="show-overhead" type="checkbox" checked> Overhead risk rings</label>
      <label><input id="show-cost" type="checkbox" checked> Cost coloring</label>
    </div>
    <div class="ramp"></div>
    <p class="hint">Cost colors: gray/green/yellow/orange/magenta. Rings highlight low-clearance and overhead-risk cells.</p>
  </aside>
  <canvas id="canvas"></canvas>
  <div id="hover-card"></div>
</div>
<script>
let data = __DATA_JSON__;
const artifactUrl = __ARTIFACT_URL_JSON__;
if (!data || typeof data !== "object") data = {cells: []};
if (!Array.isArray(data.cells)) data.cells = [];

const canvas = document.getElementById("canvas");
const ctx = canvas.getContext("2d");
let yaw = -0.75;
let pitch = 0.75;
let zoom = 1.0;
let viewCenter = null;
let dragging = false;
let lastX = 0;
let lastY = 0;

function el(id) { return document.getElementById(id); }
function finiteNumber(value, fallback = 0) { const n = Number(value); return Number.isFinite(n) ? n : fallback; }
function scoreText(value) { const n = Number(value); return Number.isFinite(n) ? n.toFixed(2) : "n/a"; }
function metersText(value) { const n = Number(value); return Number.isFinite(n) ? `${n.toFixed(2)} m` : "n/a"; }
function escapeHtml(value) { return String(value).replaceAll("&", "&amp;").replaceAll("<", "&lt;").replaceAll(">", "&gt;").replaceAll('"', "&quot;").replaceAll("'", "&#39;"); }
function controlsChecked(id) { const node = el(id); return !node || node.checked; }
function stateOf(cell) { return String(cell.state || "unknown"); }
function visibleCell(cell) {
  const state = stateOf(cell);
  if (state === "occupied") return controlsChecked("show-occupied");
  if (state === "observed_free") return controlsChecked("show-free");
  if (state === "mixed") return controlsChecked("show-mixed");
  if (cell.stale || state === "stale") return controlsChecked("show-stale");
  return true;
}
function clonePoint(p) { return p ? {x: p.x, y: p.y, z: p.z} : null; }
function boundsFromCells() {
  const points = data.cells.filter((c) => c && c.center).map((c) => c.center);
  if (points.length === 0) return {min_x: -10, max_x: 10, min_y: -10, max_y: 10, min_z: -5, max_z: 5};
  return {
    min_x: Math.min(...points.map((p) => p.x)), max_x: Math.max(...points.map((p) => p.x)),
    min_y: Math.min(...points.map((p) => p.y)), max_y: Math.max(...points.map((p) => p.y)),
    min_z: Math.min(...points.map((p) => p.z)), max_z: Math.max(...points.map((p) => p.z))
  };
}
function center() {
  const b = data.bounds || boundsFromCells();
  return {x: 0.5 * (b.min_x + b.max_x), y: 0.5 * (b.min_y + b.max_y), z: 0.5 * (b.min_z + b.max_z)};
}
function currentViewCenter() { return viewCenter || center(); }
function radius() {
  const b = data.bounds || boundsFromCells();
  return Math.max(1.0, b.max_x - b.min_x, b.max_y - b.min_y, b.max_z - b.min_z);
}
function recomputeBounds() { data.bounds = boundsFromCells(); }
function project(p) {
  const c = currentViewCenter();
  let x = p.x - c.x;
  let y = -(p.y - c.y);
  let z = -(p.z - c.z);
  const cy = Math.cos(yaw), sy = Math.sin(yaw);
  const cp = Math.cos(pitch), sp = Math.sin(pitch);
  const x1 = cy * x - sy * y;
  const y1 = sy * x + cy * y;
  const z1 = z;
  const y2 = cp * y1 - sp * z1;
  const z2 = sp * y1 + cp * z1;
  const scale = (0.78 * Math.min(canvas.width, canvas.height) * zoom) / radius();
  return {x: 0.5 * canvas.width + x1 * scale, y: 0.5 * canvas.height - z2 * scale, depth: y2, scale};
}
function costColor(cell, alpha = 0.72) {
  if (!controlsChecked("show-cost")) {
    if (stateOf(cell) === "occupied") return `rgba(255,110,95,${alpha})`;
    if (stateOf(cell) === "observed_free") return `rgba(95,205,125,${alpha * 0.65})`;
    if (cell.stale) return `rgba(165,165,165,${alpha * 0.7})`;
    return `rgba(165,170,185,${alpha * 0.7})`;
  }
  const t = Math.max(0, Math.min(1, finiteNumber(cell.total_cost, 0)));
  let r = 70, g = 200, b = 120;
  if (t > 0.75) { r = 216; g = 59; b = 125; }
  else if (t > 0.5) { r = 240; g = 140; b = 46; }
  else if (t > 0.25) { r = 229; g = 216; b = 76; }
  else if (stateOf(cell) === "unknown" || cell.unknown_score > 0.5) { r = 120; g = 130; b = 150; }
  return `rgba(${r},${g},${b},${alpha})`;
}
function drawPoint(p, color, r) {
  const pp = project(p);
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(pp.x, pp.y, r * window.devicePixelRatio, 0, Math.PI * 2);
  ctx.fill();
}
function drawRing(p, color, r) {
  const pp = project(p);
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.5 * window.devicePixelRatio;
  ctx.beginPath();
  ctx.arc(pp.x, pp.y, r * window.devicePixelRatio, 0, Math.PI * 2);
  ctx.stroke();
}
function drawLine(a, b, color, width = 1) {
  const pa = project(a), pb = project(b);
  ctx.strokeStyle = color;
  ctx.lineWidth = width * window.devicePixelRatio;
  ctx.beginPath();
  ctx.moveTo(pa.x, pa.y);
  ctx.lineTo(pb.x, pb.y);
  ctx.stroke();
}
function drawAxes() {
  const length = 0.25 * radius();
  const o = center();
  drawLine(o, {x: o.x + length, y: o.y, z: o.z}, "#ff6b6b", 2);
  drawLine(o, {x: o.x, y: o.y + length, z: o.z}, "#7bed9f", 2);
  drawLine(o, {x: o.x, y: o.y, z: o.z + length}, "#70a1ff", 2);
}
function updateMetrics() {
  recomputeBounds();
  const cells = data.cells || [];
  const count = (fn) => cells.filter(fn).length;
  const b = data.bounds;
  el("map-frame").textContent = String(data.map_frame_id || "");
  el("shown-count").textContent = String(cells.length);
  el("occupied-count").textContent = String(count((c) => stateOf(c) === "occupied"));
  el("free-count").textContent = String(count((c) => stateOf(c) === "observed_free"));
  el("mixed-count").textContent = String(count((c) => stateOf(c) === "mixed"));
  el("stale-count").textContent = String(count((c) => c.stale || stateOf(c) === "stale"));
  el("low-clearance-count").textContent = String(count((c) => c.low_clearance));
  el("overhead-risk-count").textContent = String(count((c) => c.overhead_risk));
  el("min-clearance").textContent = metersText(data.summary && data.summary.minimum_clearance_m);
  el("last-update").textContent = String(data.last_update_timestamp_ns || 0);
  el("bounds-text").textContent = `x[${b.min_x.toFixed(1)},${b.max_x.toFixed(1)}] y[${b.min_y.toFixed(1)},${b.max_y.toFixed(1)}] z[${b.min_z.toFixed(1)},${b.max_z.toFixed(1)}]`;
}
function draw() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  drawAxes();
  const cells = (data.cells || []).filter((cell) => cell && cell.center && visibleCell(cell)).slice().sort((a, b) => project(a.center).depth - project(b.center).depth);
  for (const cell of cells) {
    const score = Math.max(finiteNumber(cell.total_cost, 0), finiteNumber(cell.occupied_score, 0), finiteNumber(cell.free_score, 0));
    const r = Math.max(2.0, Math.min(8.0, 2.5 + score * 4.5));
    drawPoint(cell.center, costColor(cell), r);
    if (cell.low_clearance && controlsChecked("show-low-clearance")) drawRing(cell.center, "rgba(255,255,255,0.75)", r + 3.5);
    if (cell.overhead_risk && controlsChecked("show-overhead")) drawRing(cell.center, "rgba(255,80,255,0.82)", r + 6.0);
  }
}
function resize() {
  const rect = canvas.getBoundingClientRect();
  canvas.width = Math.max(1, Math.floor(rect.width * window.devicePixelRatio));
  canvas.height = Math.max(1, Math.floor(rect.height * window.devicePixelRatio));
  draw();
}
function hoverHtml(cell) {
  return [
    `<b>Foundational traversability cell</b>`,
    `<div>state: <code>${escapeHtml(stateOf(cell))}</code></div>`,
    `<div>total cost: <code>${scoreText(cell.total_cost)}</code></div>`,
    `<div>clearance: <code>${metersText(cell.nearest_obstacle_distance_m)}</code></div>`,
    `<div>clearance margin: <code>${metersText(cell.clearance_margin_m)}</code></div>`,
    `<div>vertical up: <code>${metersText(cell.vertical_clearance_up_m)}</code></div>`,
    `<div>occupied/free/unknown: <code>${scoreText(cell.occupied_score)} / ${scoreText(cell.free_score)} / ${scoreText(cell.unknown_score)}</code></div>`,
    `<div>age/stability/volatility: <code>${scoreText(cell.age_score)} / ${scoreText(cell.stability_score)} / ${scoreText(cell.volatility_score)}</code></div>`,
    `<div>hits/rays/conflicts: <code>${cell.occupied_hits_capped} / ${cell.free_rays_capped} / ${cell.conflict_count_capped}</code></div>`,
    `<div class="muted">x=${scoreText(cell.center.x)}, y=${scoreText(cell.center.y)}, z=${scoreText(cell.center.z)}</div>`
  ].join("");
}
function nearestCell(point) {
  let best = null;
  let bestDistance = Infinity;
  const threshold = 14.0 * window.devicePixelRatio;
  for (const cell of data.cells || []) {
    if (!cell || !cell.center || !visibleCell(cell)) continue;
    const pp = project(cell.center);
    const d = Math.hypot(pp.x - point.x, pp.y - point.y);
    if (d < bestDistance) { bestDistance = d; best = cell; }
  }
  return bestDistance <= threshold ? best : null;
}
function updateHover(event) {
  const card = el("hover-card");
  if (!card || dragging) { card.style.display = "none"; return; }
  const rect = canvas.getBoundingClientRect();
  const cell = nearestCell({x: (event.clientX - rect.left) * window.devicePixelRatio, y: (event.clientY - rect.top) * window.devicePixelRatio});
  if (!cell) { card.style.display = "none"; return; }
  card.innerHTML = hoverHtml(cell);
  card.style.left = `${event.clientX + 14}px`;
  card.style.top = `${event.clientY + 14}px`;
  card.style.display = "block";
}
function normalizeRemoteArtifact(artifact) {
  if (!artifact || typeof artifact !== "object" || !Array.isArray(artifact.cells)) return null;
  const cells = [];
  for (const raw of artifact.cells) {
    const center = raw.center_map || raw.center;
    if (!center) continue;
    const costs = raw.costs || {};
    cells.push({
      center,
      size: raw.size_m || raw.size || {x: 0.5, y: 0.5, z: 0.5},
      state: String(raw.state || "unknown"),
      occupied_score: finiteNumber(raw.occupied_score),
      free_score: finiteNumber(raw.free_score),
      unknown_score: finiteNumber(raw.unknown_score),
      confidence: finiteNumber(raw.confidence),
      age_score: finiteNumber(raw.age_score),
      stability_score: finiteNumber(raw.stability_score),
      volatility_score: finiteNumber(raw.volatility_score),
      nearest_obstacle_distance_m: raw.nearest_obstacle_distance_m,
      clearance_margin_m: raw.clearance_margin_m,
      vertical_clearance_up_m: raw.vertical_clearance_up_m,
      vertical_clearance_down_m: raw.vertical_clearance_down_m,
      occupied_cost: finiteNumber(costs.occupied_cost),
      proximity_cost: finiteNumber(costs.proximity_cost),
      unknown_cost: finiteNumber(costs.unknown_cost),
      stale_cost: finiteNumber(costs.stale_cost),
      overhead_cost: finiteNumber(costs.overhead_cost),
      thin_structure_cost: finiteNumber(costs.thin_structure_cost),
      total_cost: finiteNumber(costs.total_traversability_cost),
      stale: Boolean(raw.stale) || raw.state === "stale",
      low_clearance: Number.isFinite(Number(raw.clearance_margin_m)) && Number(raw.clearance_margin_m) < 0.5,
      overhead_risk: finiteNumber(costs.overhead_cost) > 0.05,
      occupied_hits_capped: finiteNumber(raw.occupied_hits_capped),
      free_rays_capped: finiteNumber(raw.free_rays_capped),
      conflict_count_capped: finiteNumber(raw.conflict_count_capped),
      refresh_count_capped: finiteNumber(raw.refresh_count_capped)
    });
  }
  return {
    schema: artifact.schema,
    site_id: artifact.site_id,
    site_frame_id: artifact.site_frame_id,
    mission_id: artifact.mission_id,
    map_frame_id: artifact.map_frame_id,
    first_update_timestamp_ns: artifact.first_update_timestamp_ns,
    last_update_timestamp_ns: artifact.last_update_timestamp_ns,
    config: artifact.config || {},
    summary: artifact.summary || {},
    export_summary: artifact.export_summary || {},
    cells,
    source_cell_count: cells.length,
    shown_cell_count: cells.length
  };
}
async function pollArtifactOnce() {
  if (!artifactUrl) return;
  try {
    const response = await fetch(artifactUrl, {cache: "no-store"});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const artifact = await response.json();
    const normalized = normalizeRemoteArtifact(artifact);
    if (!normalized) throw new Error("invalid traversability artifact");
    data = normalized;
    updateMetrics();
    draw();
    el("poll-status").textContent = "updated";
  } catch (err) {
    el("poll-status").textContent = `error: ${String(err)}`;
  }
}
function startPolling() {
  if (!artifactUrl || artifactUrl === "") { el("poll-status").textContent = "embedded"; return; }
  el("poll-status").textContent = "polling";
  pollArtifactOnce();
  window.setInterval(pollArtifactOnce, 1500);
}
function installControls() {
  for (const id of ["show-occupied", "show-free", "show-mixed", "show-stale", "show-low-clearance", "show-overhead", "show-cost"]) {
    const node = el(id);
    if (node) node.addEventListener("change", () => { updateMetrics(); draw(); });
  }
  el("view-center").addEventListener("click", () => { viewCenter = clonePoint(center()); yaw = 0; pitch = 0; zoom = 1.0; draw(); });
  el("view-45").addEventListener("click", () => { yaw = -Math.PI / 4; pitch = Math.PI / 4; zoom = 1.0; draw(); });
  el("view-side").addEventListener("click", () => { yaw = Math.PI / 2; pitch = 0; zoom = 1.0; draw(); });
  el("view-top").addEventListener("click", () => { yaw = 0; pitch = Math.PI / 2 - 0.01; zoom = 1.0; draw(); });
}
canvas.addEventListener("mousedown", (e) => { dragging = true; lastX = e.clientX; lastY = e.clientY; const card = el("hover-card"); if (card) card.style.display = "none"; });
window.addEventListener("mouseup", () => { dragging = false; });
window.addEventListener("mousemove", (e) => { if (!dragging) return; const dx = e.clientX - lastX; const dy = e.clientY - lastY; lastX = e.clientX; lastY = e.clientY; yaw += dx * 0.006; pitch += dy * 0.006; draw(); });
canvas.addEventListener("mousemove", updateHover);
canvas.addEventListener("mouseleave", () => { const card = el("hover-card"); if (card) card.style.display = "none"; });
canvas.addEventListener("wheel", (e) => { e.preventDefault(); zoom *= Math.exp(-e.deltaY * 0.001); zoom = Math.max(0.1, Math.min(20.0, zoom)); draw(); }, {passive: false});
window.addEventListener("resize", resize);
installControls();
recomputeBounds();
updateMetrics();
resize();
startPolling();
</script>
</body>
</html>
"""


def render_html(template: str, **values: object) -> str:
    out = template
    for key, value in values.items():
        out = out.replace("__" + key + "__", str(value))
    return out


def build_html(artifact_path: Path, payload: dict[str, Any], artifact_url: str) -> str:
    return render_html(
        HTML_TEMPLATE,
        ARTIFACT_PATH=html.escape(str(artifact_path)),
        ARTIFACT_URL=html.escape(artifact_url or "embedded"),
        ARTIFACT_URL_JSON=json.dumps(artifact_url or ""),
        DATA_JSON=json.dumps(payload),
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("artifact", type=Path, help="mission_traversability_map_full.json from S4A")
    parser.add_argument("--max-cells", type=int, default=4096)
    parser.add_argument("--artifact-url", default="", help="Optional URL for browser polling, usually relative to the static root")
    parser.add_argument("--output", type=Path, default=Path("out/mission_traversability_map_viewer.html"))
    args = parser.parse_args()

    artifact = load_artifact(args.artifact)
    payload = artifact_payload(artifact, args.max_cells)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(build_html(args.artifact, payload, args.artifact_url), encoding="utf-8")
    print(f"Wrote {args.output}")
    print(f"Artifact: {args.artifact}")
    print(f"Traversability cells: {payload['shown_cell_count']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
