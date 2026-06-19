#!/usr/bin/env python3
"""Generate mission_unified_viewer.html — pure SSE-driven SPA for dedalus_viewer sidecar.

All rendering state comes from the live SSE stream at /events.  No snapshot
artifacts are embedded at generation time.  Serve the generated file from
dedalus_viewer --static-root <dir>.

Five event types rendered:
  world_snapshot            — ego position, trajectory, diagnostics, sensing overlays
  mission_obstacle_map_delta — obstacle cells (live-aging colored dots)
  traversability_map_snapshot — traversability cells (exterior voxel face rendering)
  ghost_detections           — labeled colored spheres + velocity arrows
  mission_event              — scrolling mission event log panel

Usage:
  python3 tools/visualization/mission_unified_viewer.py [--output PATH]

Options:
  --output PATH   Output HTML path (default: out/mission_unified_viewer.html)
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


# Raw string: JS braces need no escaping because no Python str.format() is applied.
HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Dedalus Unified Viewer</title>
<style>
* { box-sizing: border-box; }
body { margin: 0; background: #0f1117; color: #e8e8e8; font: 13px system-ui, sans-serif; overflow: hidden; }
#wrap { display: grid; grid-template-columns: 300px 1fr; height: 100vh; }
#side { display: flex; flex-direction: column; background: #151923; border-right: 1px solid #2b2f3a; overflow: hidden; }
#side-scroll { flex: 1; overflow-y: auto; padding: 12px 14px; }
#canvas-wrap { position: relative; overflow: hidden; }
canvas { width: 100%; height: 100%; display: block; background: radial-gradient(circle at center, #1b2130, #0f1117); }
code { color: #a7d4ff; word-break: break-all; font-size: 11px; }
h2 { font-size: 15px; margin: 0 0 10px; color: #ffffff; }
h3 { font-size: 12px; margin: 10px 0 6px; color: #c0c4d0; text-transform: uppercase; letter-spacing: 0.06em; }
.metric { display: grid; grid-template-columns: 1fr auto; gap: 8px; padding: 3px 0; border-bottom: 1px solid #1e2330; font-size: 12px; }
.metric span { color: #9099b0; }
.metric b { color: #e8e8e8; text-align: right; }
.hint { color: #7a8099; font-size: 11px; line-height: 1.45; margin-bottom: 8px; }
.view-controls { display: grid; grid-template-columns: 1fr 1fr; gap: 5px; margin-bottom: 10px; }
.view-controls button { background: #1e2535; color: #d8dce8; border: 1px solid #333d54; border-radius: 5px; padding: 5px 6px; cursor: pointer; font-size: 12px; }
.view-controls button:hover { background: #28304a; }
.layer-controls { display: flex; flex-direction: column; gap: 5px; margin-bottom: 8px; }
.layer-controls label { display: flex; align-items: center; gap: 7px; font-size: 12px; color: #c0c4d0; cursor: pointer; }
.layer-controls input[type=checkbox] { accent-color: #5080e8; }
#status-bar { padding: 8px 14px; border-bottom: 1px solid #2b2f3a; display: flex; align-items: center; gap: 8px; font-size: 12px; }
#status-dot { width: 8px; height: 8px; border-radius: 50%; background: #555; flex-shrink: 0; }
#status-dot.live    { background: #30d060; box-shadow: 0 0 5px #30d060; }
#status-dot.replay  { background: #60a0ff; box-shadow: 0 0 5px #60a0ff; }
#status-dot.connecting { background: #e0a030; }
#status-dot.error   { background: #e05050; }
#status-text { color: #aab0c0; }
#event-log-section { margin-top: 4px; }
#event-log { max-height: 180px; overflow-y: auto; font-size: 11px; font-family: monospace; background: #0e1220; border: 1px solid #252a36; border-radius: 5px; padding: 6px; }
.event-log-entry { padding: 1px 0; border-bottom: 1px solid #181e2c; line-height: 1.35; }
.event-log-entry .ev-time { color: #5a6080; margin-right: 5px; }
.event-log-entry .ev-name { color: #80b0ff; font-weight: bold; margin-right: 5px; }
.event-log-entry .ev-detail { color: #9099b0; }
.height-ramp { height: 10px; border: 1px solid #3a4358; border-radius: 4px;
  background: linear-gradient(to right, #2641a8, #00a8d8, #3fbf6a, #e5d84c, #f08c2e, #d83b7d); margin: 4px 0; }
#hover-card { position: fixed; display: none; pointer-events: none; z-index: 10;
  max-width: 290px; padding: 8px 10px; border: 1px solid #3a4358; border-radius: 7px;
  background: rgba(13, 16, 26, 0.96); box-shadow: 0 6px 18px rgba(0,0,0,0.4);
  color: #e8e8e8; font-size: 11px; line-height: 1.4; }
#hover-card b { color: #ffffff; }
#hover-card code { color: #a7d4ff; }
#hover-card .muted { color: #7a8099; }
::-webkit-scrollbar { width: 5px; } ::-webkit-scrollbar-track { background: #0f1117; }
::-webkit-scrollbar-thumb { background: #2b3348; border-radius: 3px; }
</style>
</head>
<body>
<div id="wrap">
  <aside id="side">
    <div id="status-bar">
      <div id="status-dot"></div>
      <span id="status-text">offline</span>
      <code id="status-seq" style="margin-left:auto;color:#4a5070"></code>
    </div>
    <div id="side-scroll">
      <h2>Dedalus Unified Viewer</h2>
      <p class="hint">Drag to rotate · Wheel to zoom · Hover for cell detail</p>

      <div class="view-controls">
        <button id="view-center">Center</button>
        <button id="view-45">45°</button>
        <button id="view-side">Side</button>
        <button id="view-top">Top</button>
        <button id="view-zoom-in">＋ Zoom</button>
        <button id="view-zoom-out">－ Zoom</button>
      </div>

      <h3>Layers</h3>
      <div class="layer-controls">
        <label><input type="checkbox" id="toggle-obstacles" checked> Obstacle map</label>
        <label><input type="checkbox" id="toggle-trav" checked> Traversability surface</label>
        <label><input type="checkbox" id="toggle-ghosts"> Ghost detections</label>
        <label><input type="checkbox" id="toggle-sensing" checked> Sensing volumes</label>
        <label><input type="checkbox" id="toggle-trajectory" checked> Trajectory</label>
      </div>

      <h3>Metrics</h3>
      <div class="metric"><span>Stream source</span><code id="m-source">—</code></div>
      <div class="metric"><span>Seq</span><b id="m-seq">0</b></div>
      <div class="metric"><span>Obstacle cells</span><b id="m-obs-cells">0</b></div>
      <div class="metric"><span>Trav cells</span><b id="m-trav-cells">0</b></div>
      <div class="metric"><span>Trav ext. faces</span><b id="m-trav-faces">0</b></div>
      <div class="metric"><span>Ghost detections</span><b id="m-ghosts">0</b></div>
      <div class="metric"><span>World snapshots</span><b id="m-ws-count">0</b></div>
      <div class="metric"><span>Ego updates</span><b id="m-ego-count">0</b></div>
      <div class="metric"><span>Trajectory pts</span><b id="m-traj">0</b></div>
      <div class="metric"><span>Bounds</span><code id="m-bounds">—</code></div>

      <h3>Legend</h3>
      <div class="hint">Height above takeoff (obstacle + traversability colors)</div>
      <div class="height-ramp"></div>
      <div style="display:grid;grid-template-columns:repeat(5,1fr);font-size:10px;color:#6a7090;margin-bottom:6px">
        <span>0m</span><span style="text-align:center">10m</span><span style="text-align:center">20m</span><span style="text-align:center">30m</span><span style="text-align:right">40m+</span>
      </div>
      <div class="hint" style="margin-bottom:2px">
        Yellow = ego/path · White = sensing direction · Magenta = first blocked ·
        Orange/teal fill = traversability surface (exterior faces only) ·
        Cyan dot = ghost detection · Arrow = velocity
      </div>

      <div id="event-log-section">
        <h3>Mission Events</h3>
        <div id="event-log"><span style="color:#4a5070;font-size:11px">Waiting for mission events…</span></div>
      </div>
    </div>
  </aside>
  <div id="canvas-wrap">
    <canvas id="canvas"></canvas>
    <div id="hover-card"></div>
  </div>
</div>
<script>
"use strict";

// ── constants ──────────────────────────────────────────────────────────────────

const LIVE_BRIGHT_HOLD_MS    = 2000;
const LIVE_FADE_END_MS       = 10000;
const LIVE_OBSTACLE_DIM      = 0.45;
const LIVE_TRACK_DIM         = 0.55;
const LIVE_OBS_EVENT_MAX     = 6000;
const LIVE_OBS_GRID_M        = 0.35;
const PENDING_DELTA_CELL_MAX = 4096;
const MAX_TRAV_DISPLAY       = 8000;   // cap before face build to keep GPU happy
const MAX_GHOST_AGE_S        = 15;
const MAX_TRAJ_POINTS        = 6000;
const MAX_MISSION_LOG        = 200;

const HEIGHT_COLOR_MIN_M = 0.0;
const HEIGHT_COLOR_MAX_M = 40.0;
const HEIGHT_COLOR_BAND_M = 2.0;
const HEIGHT_COLOR_RAMP = [
  {v: 0.0,  rgb: [38,  65,  168]},
  {v: 5.0,  rgb: [0,   168, 216]},
  {v: 10.0, rgb: [63,  191, 106]},
  {v: 20.0, rgb: [229, 216, 76]},
  {v: 30.0, rgb: [240, 140, 46]},
  {v: 40.0, rgb: [216, 59,  125]},
];

// ── mutable scene state ────────────────────────────────────────────────────────

const state = {
  ego: null,
  egoYaw: null,
  trajectory: [],           // {x,y,z,live_seen_ms}
  sensingOverlays: [],
  firstBlocked: null,
  bounds: {min_x:-10, max_x:10, min_y:-10, max_y:10, min_z:-5, max_z:5},
  diagnostics: {},
  showObstacles:   true,
  showTrav:        true,
  showGhosts:      false,
  showSensing:     true,
  showTrajectory:  true,
};

// Obstacle cells
const obsCellsByKey = new Map();       // cellKey → normalized cell
const liveObsEventsByKey = new Map();
let   liveObsEvents = [];              // for age-decay rendering

// Traversability
const travCellsByKey = new Map();      // cellKey → trav cell
let   travOccupiedQKeys = new Set();   // quantized key → present (for neighbor lookup)
let   travFacesGeom = [];              // pre-built exterior face geometry
let   travCellSizeM = 0.5;
let   travVCellSizeM = 0.5;

// Ghost detections
let ghostDetections = [];              // {id, cls, pos, vel, size_m, seen_ms}

// Mission events
let missionEventLog = [];              // {time_s, event, detail_json}

// Connection
const live = {
  connected:    false,
  source:       "offline",   // "live" | "replay" | "disconnected" | "offline"
  seq:          0,
  worldSnapshotCount: 0,
  egoUpdateCount:     0,
  travSnapshotCount:  0,
  // pending queues
  pendingDeltaCells:  [],
  pendingDeltaSeq:    null,
  pendingWorldSnapshot:    null,
  pendingWorldSnapshotSeq: null,
  pendingTravSnapshot: null,
  pendingTravIsDelta: false,
  pendingGhostDetections: null,
  processingScheduled: false,
  coalescedDeltaFrames: 0,
};

// ── utilities ──────────────────────────────────────────────────────────────────

function el(id) { return document.getElementById(id); }

function escHtml(s) {
  return String(s).replaceAll("&","&amp;").replaceAll("<","&lt;").replaceAll(">","&gt;")
                  .replaceAll('"',"&quot;").replaceAll("'","&#39;");
}

function fin(v, fb = 0) {
  const n = Number(v);
  return Number.isFinite(n) ? n : fb;
}

function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }

function asVec3(value) {
  if (Array.isArray(value) && value.length >= 3) {
    const x = fin(value[0], NaN), y = fin(value[1], NaN), z = fin(value[2], NaN);
    if ([x,y,z].every(Number.isFinite)) return {x,y,z};
    return null;
  }
  if (!value || typeof value !== "object") return null;
  if (["x","y","z"].every(k => k in value)) {
    const x = fin(value.x, NaN), y = fin(value.y, NaN), z = fin(value.z, NaN);
    if ([x,y,z].every(Number.isFinite)) return {x,y,z};
  }
  if (["north","east","down"].every(k => k in value)) {
    return {x: fin(value.north), y: fin(value.east), z: -fin(value.down)};
  }
  return null;
}

function getPath(obj, path) {
  let cur = obj;
  for (const key of path) {
    if (!cur || typeof cur !== "object" || !(key in cur)) return null;
    cur = cur[key];
  }
  return cur;
}

function firstVec3(obj, paths) {
  for (const path of paths) { const v = asVec3(getPath(obj, path)); if (v) return v; }
  return null;
}

function firstNum(obj, paths) {
  for (const path of paths) {
    const raw = getPath(obj, path);
    if (raw === null || raw === undefined) continue;
    const n = Number(raw);
    if (Number.isFinite(n)) return n;
  }
  return null;
}

function addVec3(a, b, s = 1) { return {x: a.x+b.x*s, y: a.y+b.y*s, z: a.z+b.z*s}; }
function normVec3(v) {
  if (!v) return null;
  const len = Math.hypot(v.x, v.y, v.z);
  if (!Number.isFinite(len) || len < 1e-6) return null;
  return {x:v.x/len, y:v.y/len, z:v.z/len};
}

// ── color ──────────────────────────────────────────────────────────────────────

function topoBandH(hM) { return Math.round(hM / HEIGHT_COLOR_BAND_M) * HEIGHT_COLOR_BAND_M; }

function rgbForH(hM) {
  const h = clamp(topoBandH(hM), HEIGHT_COLOR_MIN_M, HEIGHT_COLOR_MAX_M);
  for (let i = 1; i < HEIGHT_COLOR_RAMP.length; ++i) {
    const lo = HEIGHT_COLOR_RAMP[i-1], hi = HEIGHT_COLOR_RAMP[i];
    if (h <= hi.v) {
      const t = clamp((h - lo.v) / Math.max(1e-6, hi.v - lo.v), 0, 1);
      return [
        Math.round(lo.rgb[0] + (hi.rgb[0]-lo.rgb[0])*t),
        Math.round(lo.rgb[1] + (hi.rgb[1]-lo.rgb[1])*t),
        Math.round(lo.rgb[2] + (hi.rgb[2]-lo.rgb[2])*t),
      ];
    }
  }
  return HEIGHT_COLOR_RAMP[HEIGHT_COLOR_RAMP.length-1].rgb;
}

function rgbaH(hM, a) { const c = rgbForH(hM); return `rgba(${c[0]},${c[1]},${c[2]},${a})`; }

// ── projection ─────────────────────────────────────────────────────────────────

const canvas = el("canvas");
const ctx    = canvas.getContext("2d");
let yaw   = -0.75;
let pitch =  0.75;
let zoom  =  1.0;
let viewCenter = null;

window.viewAnimationHandle = null;
window.easeInOutCubic = function(t) { return t<0.5 ? 4*t*t*t : 1-Math.pow(-2*t+2,3)/2; };
window.shortestAngleDelta = function(a, b) {
  let d = b-a;
  while (d > Math.PI) d -= 2*Math.PI;
  while (d < -Math.PI) d += 2*Math.PI;
  return d;
};

function sceneBounds() { return state.bounds; }
function sceneCenter() {
  const b = sceneBounds();
  return {x: 0.5*(b.min_x+b.max_x), y: 0.5*(b.min_y+b.max_y), z: 0.5*(b.min_z+b.max_z)};
}
function sceneRadius() {
  const b = sceneBounds();
  return Math.max(1, b.max_x-b.min_x, b.max_y-b.min_y, b.max_z-b.min_z);
}
function currentViewCenter() { return viewCenter || sceneCenter(); }
function cloneP(p) { return p ? {x:p.x, y:p.y, z:p.z} : null; }
function interpP(a, b, t) {
  if (!a || !b) return cloneP(b||a);
  return {x:a.x+(b.x-a.x)*t, y:a.y+(b.y-a.y)*t, z:a.z+(b.z-a.z)*t};
}

function project(p) {
  const c = currentViewCenter();
  let x =  (p.x - c.x);
  let y = -(p.y - c.y);   // NED Y → flip for canvas
  let z = -(p.z - c.z);   // flip Z so positive=up

  const cy = Math.cos(yaw),  sy = Math.sin(yaw);
  const cp = Math.cos(pitch), sp = Math.sin(pitch);
  const x1 = cy*x - sy*y;
  const y1 = sy*x + cy*y;
  const y2 = cp*y1 - sp*z;
  const z2 = sp*y1 + cp*z;

  const scale = (0.78 * Math.min(canvas.width, canvas.height) * zoom) / sceneRadius();
  return {
    x: 0.5*canvas.width  + x1*scale,
    y: 0.5*canvas.height - z2*scale,
    depth: y2,
    scale,
  };
}

window.animateViewPreset = function(tYaw, tPitch, tZoom, _label="view", tCenter=null) {
  if (window.viewAnimationHandle !== null) {
    cancelAnimationFrame(window.viewAnimationHandle);
    window.viewAnimationHandle = null;
  }
  const sYaw=yaw, sPitch=pitch, sZoom=zoom;
  const sCenter=cloneP(currentViewCenter());
  const fCenter=cloneP(tCenter||currentViewCenter());
  const dYaw=window.shortestAngleDelta(sYaw, tYaw);
  const dur=420, t0=performance.now();
  function step(now) {
    const rawT = clamp((now-t0)/dur, 0, 1), t = window.easeInOutCubic(rawT);
    yaw=sYaw+dYaw*t; pitch=sPitch+(tPitch-sPitch)*t; zoom=sZoom+(tZoom-sZoom)*t;
    viewCenter=interpP(sCenter, fCenter, t);
    draw();
    if (rawT < 1) { window.viewAnimationHandle=requestAnimationFrame(step); }
    else { yaw=tYaw; pitch=tPitch; zoom=tZoom; viewCenter=cloneP(fCenter); window.viewAnimationHandle=null; draw(); }
  }
  window.viewAnimationHandle=requestAnimationFrame(step);
};

// ── draw helpers ───────────────────────────────────────────────────────────────

function drawLine(a, b, color, width=1) {
  const pa=project(a), pb=project(b);
  ctx.strokeStyle=color; ctx.lineWidth=width*devicePixelRatio;
  ctx.beginPath(); ctx.moveTo(pa.x,pa.y); ctx.lineTo(pb.x,pb.y); ctx.stroke();
}

function drawPoint(p, color, r) {
  const pp=project(p);
  ctx.fillStyle=color;
  ctx.beginPath(); ctx.arc(pp.x, pp.y, r*devicePixelRatio, 0, Math.PI*2); ctx.fill();
}

function drawOrientationGizmo() {
  // Fixed bottom-right gizmo: always 70px from corner, 45px arm length.
  // Reflects yaw/pitch rotation only — does not move with pan or zoom.
  const dpr  = devicePixelRatio;
  const cx   = canvas.width  - 70 * dpr;
  const cy   = canvas.height - 70 * dpr;
  const len  = 45 * dpr;

  // Project a direction vector using current yaw/pitch (no translation, no zoom).
  // Mirrors the project() coordinate transform: NED Y-flip, Z-flip, then yaw+pitch.
  function gProj(vx, vy, vz) {
    const x = vx, y = -vy, z = -vz;          // NED handedness flip
    const cosy = Math.cos(yaw),  siny = Math.sin(yaw);
    const cosp = Math.cos(pitch), sinp = Math.sin(pitch);
    const x1 = cosy*x - siny*y;
    const y1 = siny*x + cosy*y;
    const z2 = sinp*y1 + cosp*z;
    return { sx: cx + x1*len, sy: cy - z2*len };
  }

  const axes = [
    { v:[1,0,0], label:"X", color:"#ff6b6b" },
    { v:[0,1,0], label:"Y", color:"#7bed9f" },
    { v:[0,0,1], label:"Z", color:"#70a1ff" },
  ];

  ctx.save();
  ctx.lineWidth = 2 * dpr;
  ctx.font = `bold ${10*dpr}px system-ui, sans-serif`;
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";

  // Draw in depth order (back-to-front) so front axes are not obscured
  const projected = axes.map(a => ({ ...a, tip: gProj(...a.v) }));
  // depth: a dot gizmo's depth direction (positive = into screen = further back)
  // y2 from gProj equivalent: sinp*y1 where y1 = siny*vx + cosy*(-vy)
  function gDepth(vx, vy) {
    const y1 = Math.sin(yaw)*vx + Math.cos(yaw)*(-vy);
    return Math.sin(pitch)*y1;
  }
  projected.sort((a,b) => gDepth(a.v[0],a.v[1]) - gDepth(b.v[0],b.v[1]));

  // Background disc
  ctx.beginPath();
  ctx.arc(cx, cy, 55*dpr, 0, Math.PI*2);
  ctx.fillStyle = "rgba(15,17,23,0.55)";
  ctx.fill();

  for (const {v, label, color, tip} of projected) {
    ctx.strokeStyle = color;
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(tip.sx, tip.sy); ctx.stroke();
    ctx.fillStyle = color;
    const lx = cx + (tip.sx-cx)*1.22;
    const ly = cy + (tip.sy-cy)*1.22;
    ctx.fillText(label, lx, ly);
  }

  // Center dot
  ctx.fillStyle = "rgba(255,255,255,0.7)";
  ctx.beginPath();
  ctx.arc(cx, cy, 3*dpr, 0, Math.PI*2);
  ctx.fill();

  ctx.restore();
}

// ── obstacle cell rendering ────────────────────────────────────────────────────

function cellKey(c) {
  return `${fin(c.x).toFixed(3)},${fin(c.y).toFixed(3)},${fin(c.z).toFixed(3)}`;
}

function liveObsKey(c) {
  const g=LIVE_OBS_GRID_M;
  return `${Math.round(fin(c.x)/g)},${Math.round(fin(c.y)/g)},${Math.round(fin(c.z)/g)}`;
}

function obsHeightM(cell) {
  const minZ=Number(cell.min_z_m), maxZ=Number(cell.max_z_m);
  if (Number.isFinite(minZ) && Number.isFinite(maxZ)) return -0.5*(minZ+maxZ);
  const ctr=asVec3(cell.center);
  return ctr ? -fin(ctr.z) : 0;
}

function stableObsColor(cell) {
  if (!cell._stable_color) {
    const a = cell.occupied ? 0.62 : (cell.free ? 0.30 : 0.38);
    cell._stable_color = rgbaH(obsHeightM(cell), a);
  }
  return cell._stable_color;
}

function normalizeObsCell(raw) {
  const center=asVec3(raw.center)||asVec3(raw.center_mission)||asVec3(raw.center_map);
  const size=asVec3(raw.size)||asVec3(raw.size_m)||{x:0.5,y:0.5,z:0.5};
  const oScore=fin(raw.occupied_score), fScore=fin(raw.free_score), rScore=fin(raw.risk_score);
  const occupied=raw.occupied===true||(oScore>0&&oScore>=fScore);
  const free=raw.free===true||(!occupied&&fScore>0);
  return {
    center, size, occupied, free,
    occupied_score:oScore, free_score:fScore, risk_score:rScore,
    confidence:fin(raw.confidence),
    min_z_m:fin(raw.min_z_m,NaN), max_z_m:fin(raw.max_z_m,NaN),
    last_source_provider:String(raw.last_source_provider||raw.source_provider||""),
    last_source_kind:String(raw.last_source_kind||raw.source_kind||""),
    live_seen_ms: null,
    _stable_color: null,
  };
}

function drawObstacleCells() {
  if (!state.showObstacles) return;
  const cells=[...obsCellsByKey.values()].filter(c=>c.center&&!c.live_seen_ms);
  cells.sort((a,b)=>project(a.center).depth-project(b.center).depth);
  for (const cell of cells) {
    const score=Math.max(cell.occupied_score||0, cell.free_score||0, cell.risk_score||0);
    const r=Math.max(2, Math.min(7, 2+score*0.4));
    drawPoint(cell.center, stableObsColor(cell), r);
  }
}

// live-aging overlay (obstacle deltas + trajectory)
function liveDecay(seenMs, dim) {
  if (!seenMs) return null;
  const age=Math.max(0, Date.now()-Number(seenMs));
  if (age <= LIVE_BRIGHT_HOLD_MS) return 1.0;
  if (age >= LIVE_FADE_END_MS) return dim;
  const t=(age-LIVE_BRIGHT_HOLD_MS)/(LIVE_FADE_END_MS-LIVE_BRIGHT_HOLD_MS);
  return 1-((1-dim)*t);
}

function drawLiveAgingOverlay() {
  if (!state.showObstacles) return;
  ctx.save();
  for (const ev of liveObsEvents) {
    const level=liveDecay(ev.live_seen_ms, LIVE_OBSTACLE_DIM);
    if (level===null) continue;
    const score=Math.max(ev.occupied_score||0, ev.free_score||0, ev.risk_score||0);
    const r=Math.max(2, Math.min(8, 3+score*0.5))*Math.max(0.8,level);
    if (!ev._live_color) ev._live_color=ev._stable_color||rgbaH(obsHeightM(ev), 0.88);
    const pp=project(ev.center);
    if (!pp||!Number.isFinite(pp.x)) continue;
    ctx.beginPath(); ctx.arc(pp.x, pp.y, r*devicePixelRatio, 0, Math.PI*2);
    ctx.fillStyle=ev._live_color; ctx.fill();
  }
  ctx.restore();
}

// ── traversability exterior face rendering ─────────────────────────────────────
//
// For each occupied cell, check each of the 6 face-normal directions.
// If the neighbor in that direction is absent from the occupied set,
// this face is exterior — draw it as a filled quad (painter's algorithm).
// Result: only the "boundary of union" surface is visible, not interior walls.

function travQuantKey(x, y, z) {
  const qx=Math.round(x/travCellSizeM);
  const qy=Math.round(y/travCellSizeM);
  const qz=Math.round(z/travVCellSizeM);
  return `${qx},${qy},${qz}`;
}

function buildTravFaces() {
  travFacesGeom = [];
  if (travCellsByKey.size === 0) return;

  // Collect occupied cells (with optional decimation for large maps)
  let occupiedCells = [...travCellsByKey.values()].filter(c => c.state === "occupied");
  if (occupiedCells.length > MAX_TRAV_DISPLAY) {
    // Prefer cells closest to ego for decimation
    if (state.ego) {
      const ego=state.ego;
      occupiedCells.sort((a,b)=>{
        const da=Math.hypot(a.center.x-ego.x, a.center.y-ego.y, a.center.z-ego.z);
        const db=Math.hypot(b.center.x-ego.x, b.center.y-ego.y, b.center.z-ego.z);
        return da-db;
      });
    }
    occupiedCells = occupiedCells.slice(0, MAX_TRAV_DISPLAY);
  }

  const DIRS = [
    [1,0,0], [-1,0,0],
    [0,1,0], [0,-1,0],
    [0,0,1], [0,0,-1],
  ];

  for (const cell of occupiedCells) {
    const {center, hx, hy, hz} = cell;
    const {x:cx, y:cy, z:cz} = center;
    const [qx, qy, qz] = travQuantKey(cx, cy, cz).split(",").map(Number);

    for (const [dx, dy, dz] of DIRS) {
      const nKey = `${qx+dx},${qy+dy},${qz+dz}`;
      if (travOccupiedQKeys.has(nKey)) continue; // interior face — skip

      // Build 4 corners of the exterior face
      let corners;
      if (dx !== 0) {
        const fx = cx + dx * hx;
        corners = [
          {x:fx, y:cy-hy, z:cz-hz}, {x:fx, y:cy+hy, z:cz-hz},
          {x:fx, y:cy+hy, z:cz+hz}, {x:fx, y:cy-hy, z:cz+hz},
        ];
      } else if (dy !== 0) {
        const fy = cy + dy * hy;
        corners = [
          {x:cx-hx, y:fy, z:cz-hz}, {x:cx+hx, y:fy, z:cz-hz},
          {x:cx+hx, y:fy, z:cz+hz}, {x:cx-hx, y:fy, z:cz+hz},
        ];
      } else {
        const fz = cz + dz * hz;
        corners = [
          {x:cx-hx, y:cy-hy, z:fz}, {x:cx+hx, y:cy-hy, z:fz},
          {x:cx+hx, y:cy+hy, z:fz}, {x:cx-hx, y:cy+hy, z:fz},
        ];
      }

      // Face centroid for depth sorting
      const fcx=(corners[0].x+corners[2].x)*0.5;
      const fcy=(corners[0].y+corners[2].y)*0.5;
      const fcz=(corners[0].z+corners[2].z)*0.5;

      travFacesGeom.push({
        corners,
        centroid: {x:fcx, y:fcy, z:fcz},
        heightM: -cz,
        confidence: cell.confidence,
        stale: cell.stale,
      });
    }
  }
}

function drawTravFaces() {
  if (!state.showTrav || travFacesGeom.length === 0) return;

  // Sort back-to-front for painter's algorithm.
  const sorted = travFacesGeom.slice().sort(
    (a,b) => project(a.centroid).depth - project(b.centroid).depth
  );

  ctx.save();
  ctx.lineWidth = 0.5 * devicePixelRatio;

  // Batch by fill color: group all faces sharing the same rgba key into one
  // Path2D → 2 canvas calls per unique color instead of 2 per face.
  // Color is height-banded (topoBandH), so same-height faces cluster naturally.
  // Approximation: strict depth order is maintained per color group; cross-group
  // ordering may differ slightly from per-face painter's, but is visually fine.
  const groups = new Map(); // colorKey → { fillStyle, strokeStyle, path }

  for (const face of sorted) {
    const ps = face.corners.map(c => project(c));
    if (ps.some(p => !Number.isFinite(p.x) || !Number.isFinite(p.y))) continue;

    const rgb = rgbForH(Math.max(0, face.heightM));
    const a   = face.stale ? 0.10 : clamp(0.28 + face.confidence*0.32, 0.18, 0.65);
    // Use a quantized alpha key (2 decimal places) to avoid float noise splitting groups
    const aStr  = a.toFixed(2);
    const asStr = Math.min(1, a+0.18).toFixed(2);
    const key   = `${rgb[0]},${rgb[1]},${rgb[2]},${aStr}`;

    let g = groups.get(key);
    if (!g) {
      g = {
        fillStyle:   `rgba(${rgb[0]},${rgb[1]},${rgb[2]},${aStr})`,
        strokeStyle: `rgba(${rgb[0]},${rgb[1]},${rgb[2]},${asStr})`,
        path: new Path2D(),
      };
      groups.set(key, g);
    }

    g.path.moveTo(ps[0].x, ps[0].y);
    for (let i=1; i<ps.length; i++) g.path.lineTo(ps[i].x, ps[i].y);
    g.path.closePath();
  }

  // Flush: one fill + one stroke per color group
  for (const {fillStyle, strokeStyle, path} of groups.values()) {
    ctx.fillStyle   = fillStyle;
    ctx.strokeStyle = strokeStyle;
    ctx.fill(path);
    ctx.stroke(path);
  }

  ctx.restore();

  el("m-trav-faces").textContent = String(travFacesGeom.length);
}

// ── ghost detection rendering ──────────────────────────────────────────────────

function drawGhostDetections() {
  if (!state.showGhosts) return;
  const now=Date.now(), maxAgeMs=MAX_GHOST_AGE_S*1000;
  for (const det of ghostDetections) {
    if (!det.pos) continue;
    const ageMs=Math.max(0, now-det.seen_ms);
    if (ageMs > maxAgeMs) continue;
    const alpha=clamp(1-ageMs/maxAgeMs, 0.2, 1.0);

    // Colored sphere (class-coded)
    const color=ghostClassColor(det.cls, alpha);
    const pp=project(det.pos);
    if (!pp||!Number.isFinite(pp.x)) continue;
    const r=Math.max(4, 5+fin(det.size_m?.x,1)*0.5)*devicePixelRatio;
    ctx.beginPath(); ctx.arc(pp.x, pp.y, r, 0, Math.PI*2);
    ctx.fillStyle=color; ctx.fill();
    ctx.strokeStyle=`rgba(255,255,255,${alpha*0.7})`; ctx.lineWidth=1*devicePixelRatio; ctx.stroke();

    // Velocity arrow (if non-trivial)
    if (det.vel) {
      const speed=Math.hypot(det.vel.x, det.vel.y, det.vel.z);
      if (speed > 0.05) {
        const tip=addVec3(det.pos, det.vel, 2.0/Math.max(speed,1));
        drawLine(det.pos, tip, `rgba(255,220,80,${alpha*0.9})`, 2);
      }
    }

    // Label
    const labelText=det.cls||det.id||"ghost";
    const pl=project(det.pos);
    ctx.fillStyle=`rgba(255,255,255,${alpha*0.85})`;
    ctx.font=`${Math.round(10*devicePixelRatio)}px system-ui`;
    ctx.fillText(labelText, pl.x + r + 2, pl.y - r);
  }
}

function ghostClassColor(cls, alpha) {
  const clsL=(cls||"").toLowerCase();
  if (clsL.includes("friendly")) return `rgba(80,200,255,${alpha})`;
  if (clsL.includes("hostile"))  return `rgba(255,80,80,${alpha})`;
  if (clsL.includes("neutral"))  return `rgba(200,200,80,${alpha})`;
  return `rgba(100,255,200,${alpha})`;
}

// ── sensing overlay rendering ──────────────────────────────────────────────────

function drawSensingOverlays() {
  if (!state.showSensing) return;
  for (const ov of state.sensingOverlays) {
    if (!ov.origin||!ov.forward) continue;
    const range=Math.max(1, fin(ov.range_m, 10));
    const tip=addVec3(ov.origin, ov.forward, range);
    drawLine(ov.origin, tip, "rgba(0,0,0,0.94)", 7.5);
    drawLine(ov.origin, tip, "rgba(255,255,255,1.0)", 4.2);
    drawPoint(ov.origin, "rgba(0,0,0,0.94)", 7.0);
    drawPoint(ov.origin, "rgba(255,255,255,1.0)", 4.3);
  }
}

// ── drone + trajectory rendering ───────────────────────────────────────────────

function droneVelocityDir() {
  if (!state.ego||state.trajectory.length<2) return null;
  const newest=state.trajectory[state.trajectory.length-1];
  for (let i=state.trajectory.length-2; i>=0; --i) {
    const older=state.trajectory[i];
    if (!older||!newest) continue;
    const dx=newest.x-older.x, dy=newest.y-older.y, dz=newest.z-older.z;
    if (Math.hypot(dx,dy)>0.05) {
      const len=Math.hypot(dx,dy,dz);
      if (len>1e-6) return {x:dx/len, y:dy/len, z:dz/len};
    }
  }
  return null;
}

function drawDroneMarker() {
  if (!state.ego) return;
  drawPoint(state.ego, "rgba(0,0,0,0.92)", 10.5);
  drawPoint(state.ego, "rgba(255,255,255,1.0)", 8.5);
  drawPoint(state.ego, "rgba(255,230,64,1.0)", 5.8);
  const vel=droneVelocityDir();
  if (vel) {
    const tip=addVec3(state.ego, vel, 2.2);
    drawLine(state.ego, tip, "rgba(0,0,0,0.96)", 7);
    drawLine(state.ego, tip, "rgba(255,255,255,1.0)", 3.8);
  }
}

function drawTrajectory() {
  if (!state.showTrajectory||state.trajectory.length<2) return;
  const now=Date.now();
  ctx.save();
  for (let i=1; i<state.trajectory.length; ++i) {
    const a=state.trajectory[i-1], b=state.trajectory[i];
    if (!a||!b) continue;
    if (a.live_seen_ms||b.live_seen_ms) continue;
    drawLine(a, b, "rgba(150,120,35,0.35)", 1.5);
  }

  // Live aging segments
  const liveTraj=state.trajectory.filter(p=>p&&p.live_seen_ms);
  for (let i=1; i<liveTraj.length; ++i) {
    const a=project(liveTraj[i-1]), b=project(liveTraj[i]);
    if (!a||!b||!Number.isFinite(a.x)||!Number.isFinite(b.x)) continue;
    const level=liveDecay(liveTraj[i].live_seen_ms, LIVE_TRACK_DIM)??LIVE_TRACK_DIM;
    const gb=Math.round(255*level);
    ctx.beginPath(); ctx.moveTo(a.x,a.y); ctx.lineTo(b.x,b.y);
    ctx.strokeStyle=`rgba(255,${gb},0,0.9)`;
    ctx.lineWidth=Math.max(1.5, 4*level)*devicePixelRatio;
    ctx.stroke();
  }
  if (state.ego) drawPoint(state.ego, "rgba(255,255,64,1.0)", 6.5);
  ctx.restore();
}

// ── scene bounds recompute ─────────────────────────────────────────────────────

function recomputeBounds() {
  const pts=[];
  for (const cell of obsCellsByKey.values()) { if (cell.center) pts.push(cell.center); }
  for (const p of state.trajectory) { if (p) pts.push(p); }
  if (state.ego) pts.push(state.ego);
  for (const ov of state.sensingOverlays) {
    if (ov.origin) pts.push(ov.origin);
    if (ov.origin&&ov.forward) pts.push(addVec3(ov.origin, ov.forward, fin(ov.range_m,8)));
  }
  if (state.firstBlocked) pts.push(state.firstBlocked);
  if (pts.length===0) {
    state.bounds={min_x:-10,max_x:10,min_y:-10,max_y:10,min_z:-5,max_z:5};
    return;
  }
  state.bounds={
    min_x:Math.min(...pts.map(p=>p.x)), max_x:Math.max(...pts.map(p=>p.x)),
    min_y:Math.min(...pts.map(p=>p.y)), max_y:Math.max(...pts.map(p=>p.y)),
    min_z:Math.min(...pts.map(p=>p.z)), max_z:Math.max(...pts.map(p=>p.z)),
  };
}

// ── main draw ──────────────────────────────────────────────────────────────────

let drawScheduled=false;
function scheduleDraw() {
  if (drawScheduled) return;
  drawScheduled=true;
  requestAnimationFrame(()=>{ drawScheduled=false; draw(); });
}

function draw() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  drawTravFaces();       // traversability exterior surface (back, painted first)
  drawObstacleCells();   // mission obstacle map cells
  drawLiveAgingOverlay(); // live delta events with age decay
  if (state.showTrajectory) drawTrajectory();
  drawGhostDetections();
  drawSensingOverlays();
  drawDroneMarker();
  if (state.firstBlocked) drawPoint(state.firstBlocked, "rgba(255,80,255,1.0)", 8);
  drawOrientationGizmo(); // fixed bottom-right — drawn last so it is always on top

  // Schedule next frame if live aging is active
  if (live.connected) {
    const now=Date.now();
    const hasRecent=liveObsEvents.some(e=>e.live_seen_ms&&now-Number(e.live_seen_ms)<LIVE_FADE_END_MS);
    if (hasRecent) requestAnimationFrame(draw);
  }
}

// ── metrics + UI ───────────────────────────────────────────────────────────────

function updateStatusBar() {
  const dot=el("status-dot");
  const txt=el("status-text");
  const seq=el("status-seq");
  dot.className="";
  if (!live.connected) {
    const src=live.source;
    dot.className = src==="offline"?"":"connecting";
    txt.textContent = src==="offline"?"offline (no /events URL)":"reconnecting…";
  } else if (live.source==="replay") {
    dot.className="replay"; txt.textContent="replay";
  } else if (live.source==="live") {
    dot.className="live"; txt.textContent="live";
  } else {
    dot.className="live"; txt.textContent="connected";
  }
  if (live.seq) seq.textContent=`seq ${live.seq}`;
}

function fmt(v) { const n=Number(v); return Number.isFinite(n)?Number.isInteger(n)?String(n):n.toFixed(2):"—"; }

function updateMetrics() {
  updateStatusBar();
  const b=state.bounds;
  el("m-source").textContent = live.source;
  el("m-seq").textContent    = String(live.seq);
  el("m-obs-cells").textContent  = String(obsCellsByKey.size);
  el("m-trav-cells").textContent = String(travCellsByKey.size);
  // m-trav-faces updated in drawTravFaces()
  el("m-ghosts").textContent = String(ghostDetections.filter(
    d=>Date.now()-d.seen_ms < MAX_GHOST_AGE_S*1000).length);
  el("m-ws-count").textContent  = String(live.worldSnapshotCount);
  el("m-ego-count").textContent = String(live.egoUpdateCount);
  el("m-traj").textContent = String(state.trajectory.length);
  el("m-bounds").textContent =
    `x[${b.min_x.toFixed(1)},${b.max_x.toFixed(1)}] ` +
    `y[${b.min_y.toFixed(1)},${b.max_y.toFixed(1)}] ` +
    `z[${b.min_z.toFixed(1)},${b.max_z.toFixed(1)}]`;
}

function appendMissionEvent(evJson) {
  const ev = typeof evJson==="string" ? JSON.parse(evJson) : evJson;
  const eventName = ev.event||ev.type||ev.kind||"event";
  const detail = Object.entries(ev).filter(([k])=>k!=="event"&&k!=="type"&&k!=="kind")
    .map(([k,v])=>`${k}:${typeof v==="string"?v:JSON.stringify(v)}`).join(" ");
  missionEventLog.push({event:eventName, detail, ts_ms:Date.now()});
  if (missionEventLog.length > MAX_MISSION_LOG) missionEventLog.shift();

  const logEl=el("event-log");
  const entry=document.createElement("div");
  entry.className="event-log-entry";
  const t=new Date().toISOString().substr(11,8);
  entry.innerHTML=
    `<span class="ev-time">${escHtml(t)}</span>` +
    `<span class="ev-name">${escHtml(eventName)}</span>` +
    `<span class="ev-detail">${escHtml(detail.substr(0,120))}</span>`;
  // Remove placeholder if first real entry
  if (missionEventLog.length===1) logEl.innerHTML="";
  logEl.appendChild(entry);
  logEl.scrollTop=logEl.scrollHeight;
  while (logEl.children.length > MAX_MISSION_LOG) logEl.removeChild(logEl.firstChild);
}

// ── snapshot parsers ───────────────────────────────────────────────────────────

function snapshotSensingOverlays(snap) {
  const candidates=[
    snap.obstacle_sensing_volumes, snap.sensing_volumes,
    snap.camera_sensing_volumes, snap.depth_sensing_volumes,
    snap.sensing_coverage?.volumes,
  ];
  const out=[];
  for (const arr of candidates) {
    if (!Array.isArray(arr)) continue;
    for (const raw of arr) {
      if (!raw||typeof raw!=="object") continue;
      const origin=asVec3(raw.origin_local)||asVec3(raw.origin)||
                   asVec3(raw.camera_origin_local)||asVec3(raw.position_local)||asVec3(raw.position);
      const forward=normVec3(asVec3(raw.forward_axis_local)||asVec3(raw.forward)||
                             asVec3(raw.direction_local)||asVec3(raw.direction));
      if (!origin||!forward) continue;
      out.push({
        label:String(raw.camera_id||raw.sensor_id||raw.id||"sensing"),
        origin, forward,
        range_m:Math.max(0.25, fin(raw.range_m||raw.max_range_m||raw.far_m, 8)),
      });
    }
  }
  return out;
}

function applyWorldSnapshot(snap, seq, {deferRender=false}={}) {
  if (!snap||typeof snap!=="object") return false;
  live.worldSnapshotCount++;

  const ego=firstVec3(snap,[
    ["ego_state","local_T_body","position"],["ego_state","pose","position"],
    ["ego_state","position_local"],["ego_state","position"],
    ["ego","local_T_body","position"],["ego","pose","position"],
    ["ego","position_local"],["ego","position"],["ego","local_position"],
    ["vehicle","position"],["drone","position"],["agent","position"],["ownship","position"],
  ]);
  if (ego) {
    live.egoUpdateCount++;
    state.ego=ego;
    const last=state.trajectory.length?state.trajectory[state.trajectory.length-1]:null;
    if (!last||Math.hypot(last.x-ego.x,last.y-ego.y,last.z-ego.z)>0.05) {
      state.trajectory.push({...ego, live_seen_ms:Date.now()});
      if (state.trajectory.length>MAX_TRAJ_POINTS) state.trajectory.splice(0,state.trajectory.length-MAX_TRAJ_POINTS);
    }
  }

  const yawV=firstNum(snap,[
    ["ego_state","local_T_body","rotation_rpy","z"],["ego_state","local_T_body","rotation_rpy","yaw"],
    ["ego","local_T_body","rotation_rpy","z"],["ego","local_T_body","rotation_rpy","yaw"],
    ["ego","yaw_rad"],["ego","heading_rad"],
  ]);
  if (yawV!==null) state.egoYaw=yawV;

  const ovs=snapshotSensingOverlays(snap);
  if (ovs.length>0) state.sensingOverlays=ovs;

  const safety=snap.trajectory_safety;
  if (safety&&typeof safety==="object") {
    const fb=firstVec3(safety,[["first_blocked_position_local"],["first_blocked_position"],["first_blocked_point"]]);
    if (fb) state.firstBlocked=fb;
  }

  const mission=snap.mission_local_obstacle_map;
  if (mission&&typeof mission==="object") {
    Object.assign(state.diagnostics, {
      raw_evidence_count:   fin(mission.raw_evidence_count),
      compacted_evidence_count: fin(mission.compacted_evidence_count),
    });
  }

  live.seq=seq??live.seq;
  if (!deferRender) { recomputeBounds(); updateMetrics(); scheduleDraw(); }
  return true;
}

function applyMissionObstacleMapDelta(delta, seq, {deferRender=false}={}) {
  if (!delta||typeof delta!=="object"||!Array.isArray(delta.cells)) return 0;
  let changed=0;
  const nowMs=Date.now();
  for (const raw of delta.cells) {
    const cell=normalizeObsCell(raw);
    if (!cell.center) continue;
    const k=cellKey(cell.center);
    obsCellsByKey.set(k, cell);

    const ek=liveObsKey(cell.center);
    const ev={
      key:ek, center:cell.center,
      occupied_score:cell.occupied_score, free_score:cell.free_score, risk_score:cell.risk_score,
      min_z_m:cell.min_z_m, max_z_m:cell.max_z_m,
      _stable_color:null, _live_color:null, live_seen_ms:nowMs,
    };
    ev._stable_color=stableObsColor(cell);
    const existing=liveObsEventsByKey.get(ek);
    if (existing) { Object.assign(existing, ev); }
    else { liveObsEventsByKey.set(ek, ev); liveObsEvents.push(ev); }
    changed++;
  }

  if (liveObsEvents.length > LIVE_OBS_EVENT_MAX) {
    const removed=liveObsEvents.splice(0, liveObsEvents.length-LIVE_OBS_EVENT_MAX);
    for (const e of removed) { if (e.key) liveObsEventsByKey.delete(e.key); }
  }

  live.seq=seq??live.seq;
  if (!deferRender) { recomputeBounds(); updateMetrics(); scheduleDraw(); }
  return changed;
}

function applyTravSnapshot(trav, {deferRender=false}={}) {
  if (!trav||!Array.isArray(trav.cells)) return false;
  live.travSnapshotCount++;

  travCellSizeM  = fin(trav.cell_size_m, 0.5);
  travVCellSizeM = fin(trav.vertical_cell_size_m, travCellSizeM);
  if (travCellSizeM  < 0.01) travCellSizeM  = 0.5;
  if (travVCellSizeM < 0.01) travVCellSizeM = 0.5;

  travCellsByKey.clear();
  travOccupiedQKeys.clear();

  for (const raw of trav.cells) {
    const center=asVec3(raw.center_map);
    if (!center) continue;
    const state_s=typeof raw.state==="string"?raw.state:"unknown";
    const hx=travCellSizeM*0.5, hy=travCellSizeM*0.5, hz=travVCellSizeM*0.5;
    const cell={
      center,
      hx, hy, hz,
      state: state_s,
      occupied_score: fin(raw.occupied_score,0),
      free_score:     fin(raw.free_score,0),
      confidence:     fin(raw.confidence,0),
      stale: raw.stale===true||state_s==="stale",
    };
    const k=travQuantKey(center.x, center.y, center.z);
    travCellsByKey.set(k, cell);
    if (state_s==="occupied") travOccupiedQKeys.add(k);
  }

  buildTravFaces();

  if (!deferRender) { updateMetrics(); scheduleDraw(); }
  return true;
}

function applyTravDelta(trav, {deferRender=false}={}) {
  // Incremental update: merge changed cells into the existing map without clearing.
  // Called when the server emits a traversability_map_delta event.
  if (!trav||!Array.isArray(trav.cells)||trav.cells.length===0) return false;

  const newCellSizeM  = fin(trav.cell_size_m, 0);
  const newVCellSizeM = fin(trav.vertical_cell_size_m, 0);
  if (newCellSizeM  >= 0.01) travCellSizeM  = newCellSizeM;
  if (newVCellSizeM >= 0.01) travVCellSizeM = newVCellSizeM;

  for (const raw of trav.cells) {
    const center=asVec3(raw.center_map);
    if (!center) continue;
    const state_s=typeof raw.state==="string"?raw.state:"unknown";
    const hx=travCellSizeM*0.5, hy=travCellSizeM*0.5, hz=travVCellSizeM*0.5;
    const cell={
      center, hx, hy, hz,
      state: state_s,
      occupied_score: fin(raw.occupied_score,0),
      free_score:     fin(raw.free_score,0),
      confidence:     fin(raw.confidence,0),
      stale: raw.stale===true||state_s==="stale",
    };
    const k=travQuantKey(center.x, center.y, center.z);
    travCellsByKey.set(k, cell);
    // Update occupied key set: add or remove based on new state.
    if (state_s==="occupied") { travOccupiedQKeys.add(k); }
    else { travOccupiedQKeys.delete(k); }
  }

  buildTravFaces();

  if (!deferRender) { updateMetrics(); scheduleDraw(); }
  return true;
}

function applyGhostDetections(payload, {deferRender=false}={}) {
  const frame=payload.ghost_detections;
  if (!frame||typeof frame!=="object"||!Array.isArray(frame.detections)) return;
  const nowMs=Date.now();
  for (const det of frame.detections) {
    const id=String(det.source_track_id||"");
    const pos=asVec3(det.position_local_m);
    const vel=asVec3(det.velocity_local_mps);
    const size_m=asVec3(det.size_m)||{x:1,y:1,z:1};
    const existing=ghostDetections.find(g=>g.id===id);
    if (existing) { existing.pos=pos||existing.pos; existing.vel=vel||existing.vel; existing.seen_ms=nowMs; }
    else ghostDetections.push({id, cls:String(det.class||det.class_label||""), pos, vel, size_m, seen_ms:nowMs});
  }
  // Prune stale ghosts
  ghostDetections=ghostDetections.filter(g=>nowMs-g.seen_ms < MAX_GHOST_AGE_S*2000);

  if (!deferRender) { updateMetrics(); scheduleDraw(); }
}

function applyMissionEvent(payload, {deferRender=false}={}) {
  const ev=payload.mission_event;
  if (!ev) return;
  try { appendMissionEvent(typeof ev==="string"?ev:JSON.stringify(ev)); }
  catch(e) { /* ignore parse error */ }
  if (!deferRender) updateMetrics();
}

// ── pending queue processing ───────────────────────────────────────────────────

function scheduleLiveProcessing() {
  if (live.processingScheduled) return;
  live.processingScheduled=true;
  requestAnimationFrame(processPending);
}

function processPending() {
  live.processingScheduled=false;
  const pCells=live.pendingDeltaCells, pSeq=live.pendingDeltaSeq;
  live.pendingDeltaCells=[]; live.pendingDeltaSeq=null;
  const pSnap=live.pendingWorldSnapshot, pSnapSeq=live.pendingWorldSnapshotSeq;
  live.pendingWorldSnapshot=null; live.pendingWorldSnapshotSeq=null;
  const pTrav=live.pendingTravSnapshot; const pTravIsDelta=live.pendingTravIsDelta;
  live.pendingTravSnapshot=null; live.pendingTravIsDelta=false;
  const pGhost=live.pendingGhostDetections;
  live.pendingGhostDetections=null;

  let changed=false;
  if (pCells.length>0)  changed=applyMissionObstacleMapDelta({cells:pCells}, pSeq, {deferRender:true})>0||changed;
  if (pSnap)            changed=applyWorldSnapshot(pSnap, pSnapSeq, {deferRender:true})||changed;
  if (pTrav)            changed=(pTravIsDelta
                          ? applyTravDelta(pTrav, {deferRender:true})
                          : applyTravSnapshot(pTrav, {deferRender:true}))||changed;
  if (pGhost)           { applyGhostDetections(pGhost, {deferRender:true}); changed=true; }

  if (changed) { recomputeBounds(); updateMetrics(); scheduleDraw(); }
  else { updateMetrics(); }

  if ((live.pendingDeltaCells.length||live.pendingWorldSnapshot||live.pendingTravSnapshot||live.pendingGhostDetections)
      && !live.processingScheduled) {
    scheduleLiveProcessing();
  }
}

// ── SSE connection ─────────────────────────────────────────────────────────────

function eventUrlFromLocation() {
  const params=new URLSearchParams(window.location.search);
  if (params.get("live")==="0") return null;
  const explicit=params.get("events");
  if (explicit) return explicit;
  if (window.location.protocol==="http:"||window.location.protocol==="https:") return "/events";
  return null;
}

function startLiveStream() {
  const url=eventUrlFromLocation();
  if (!url||typeof EventSource==="undefined") {
    live.source="offline"; updateStatusBar(); return;
  }

  live.source="connecting"; updateStatusBar();
  const source=new EventSource(url);

  source.onopen=()=>{ live.connected=true; live.source="live"; updateStatusBar(); };
  source.onerror=()=>{ live.connected=false; live.source="disconnected"; updateStatusBar(); };

  source.addEventListener("world_snapshot", ev => {
    try {
      const payload=JSON.parse(ev.data);
      // Infer source from viewer_status if available
      const snap=payload.world_snapshot||payload.snapshot;
      if (!snap) return;
      live.worldSnapshotCount++;
      live.pendingWorldSnapshot=snap;
      live.pendingWorldSnapshotSeq=payload.seq??null;
      live.seq=payload.seq??live.seq;
      scheduleLiveProcessing();
    } catch(e) {}
  });

  source.addEventListener("mission_obstacle_map_delta", ev => {
    try {
      const payload=JSON.parse(ev.data);
      const delta=payload.mission_obstacle_map_delta;
      if (!delta||!Array.isArray(delta.cells)) return;
      live.coalescedDeltaFrames++;
      live.pendingDeltaSeq=payload.seq??live.pendingDeltaSeq;
      for (const raw of delta.cells) live.pendingDeltaCells.push(raw);
      if (live.pendingDeltaCells.length > PENDING_DELTA_CELL_MAX) {
        live.pendingDeltaCells.splice(0, live.pendingDeltaCells.length-PENDING_DELTA_CELL_MAX);
      }
      scheduleLiveProcessing();
    } catch(e) {}
  });

  source.addEventListener("traversability_map_snapshot", ev => {
    try {
      const payload=JSON.parse(ev.data);
      const trav=payload.traversability_map_snapshot;
      if (!trav||!Array.isArray(trav.cells)) return;
      live.pendingTravSnapshot=trav;
      live.pendingTravIsDelta=false;  // full snapshot — client clears + rebuilds
      scheduleLiveProcessing();
    } catch(e) {}
  });

  source.addEventListener("traversability_map_delta", ev => {
    try {
      const payload=JSON.parse(ev.data);
      const trav=payload.traversability_map_delta;
      if (!trav||!Array.isArray(trav.cells)) return;
      // Coalesce: if a previous delta is pending, apply it immediately before
      // queuing this one so we do not lose intermediate state.
      if (live.pendingTravSnapshot&&live.pendingTravIsDelta) {
        applyTravDelta(live.pendingTravSnapshot, {deferRender:true});
      }
      live.pendingTravSnapshot=trav;
      live.pendingTravIsDelta=true;  // incremental — client merges
      scheduleLiveProcessing();
    } catch(e) {}
  });

  source.addEventListener("ghost_detections", ev => {
    try {
      const payload=JSON.parse(ev.data);
      if (!payload.ghost_detections) return;
      live.pendingGhostDetections=payload;
      scheduleLiveProcessing();
    } catch(e) {}
  });

  source.addEventListener("mission_event", ev => {
    try {
      const payload=JSON.parse(ev.data);
      // Mission events are applied immediately (low frequency)
      applyMissionEvent(payload);
      // Also pull source from viewer_status
      if (payload.source) live.source=String(payload.source);
    } catch(e) {}
  });

  // Poll /viewer_status to sync source (live/replay/disconnected)
  function pollViewerStatus() {
    fetch("/viewer_status").then(r=>r.json()).then(d=>{
      if (d&&d.source) {
        live.source=String(d.source);
        updateStatusBar();
      }
    }).catch(()=>{});
    setTimeout(pollViewerStatus, 3000);
  }
  setTimeout(pollViewerStatus, 1000);
}

// ── view controls + interaction ────────────────────────────────────────────────

function installViewControls() {
  el("view-center")?.addEventListener("click", ()=>{
    recomputeBounds();
    window.animateViewPreset(0, 0, 1.0, "center", cloneP(sceneCenter()));
  });
  el("view-45")?.addEventListener("click", ()=>window.animateViewPreset(-Math.PI/4, Math.PI/4, 1.0, "45"));
  el("view-side")?.addEventListener("click", ()=>window.animateViewPreset(Math.PI/2, 0, 1.0, "side"));
  el("view-top")?.addEventListener("click", ()=>window.animateViewPreset(0, Math.PI/2-0.01, 1.0, "top"));
  el("view-zoom-in")?.addEventListener("click", ()=>{
    zoom=clamp(zoom*1.25, 0.08, 25); scheduleDraw();
  });
  el("view-zoom-out")?.addEventListener("click", ()=>{
    zoom=clamp(zoom/1.25, 0.08, 25); scheduleDraw();
  });

  const toggles=[
    ["toggle-obstacles",  v=>{ state.showObstacles  =v; scheduleDraw(); }],
    ["toggle-trav",       v=>{ state.showTrav        =v; scheduleDraw(); }],
    ["toggle-ghosts",     v=>{ state.showGhosts      =v; scheduleDraw(); }],
    ["toggle-sensing",    v=>{ state.showSensing     =v; scheduleDraw(); }],
    ["toggle-trajectory", v=>{ state.showTrajectory  =v; scheduleDraw(); }],
  ];
  for (const [id, fn] of toggles) {
    const cb=el(id); if (cb) cb.addEventListener("change", ()=>fn(cb.checked));
  }
}

// Canvas mouse interaction
let dragging=false, lastX=0, lastY=0;
const hoverCard=el("hover-card");

canvas.addEventListener("mousedown", e=>{
  if (window.viewAnimationHandle!==null) { cancelAnimationFrame(window.viewAnimationHandle); window.viewAnimationHandle=null; }
  dragging=true; lastX=e.clientX; lastY=e.clientY;
  if (hoverCard) hoverCard.style.display="none";
});
window.addEventListener("mouseup", ()=>{ dragging=false; });
window.addEventListener("mousemove", e=>{
  if (!dragging) return;
  yaw   += (e.clientX-lastX)*0.006;
  pitch += (e.clientY-lastY)*0.006;
  lastX=e.clientX; lastY=e.clientY;
  draw();
});
canvas.addEventListener("wheel", e=>{
  e.preventDefault();
  if (hoverCard) hoverCard.style.display="none";
  zoom *= Math.exp(-e.deltaY*0.001);
  zoom=clamp(zoom, 0.08, 25);
  draw();
}, {passive:false});

// Hover card — nearest obstacle cell
function nearestObsCellToCanvas(px, py) {
  let best=null, bestD=Infinity;
  const thresh=14*devicePixelRatio;
  for (const cell of obsCellsByKey.values()) {
    if (!cell.center||cell.live_seen_ms) continue;
    const p=project(cell.center);
    if (!p||!Number.isFinite(p.x)) continue;
    const d=Math.hypot(p.x-px, p.y-py);
    if (d<bestD) { bestD=d; best=cell; }
  }
  return bestD<=thresh?best:null;
}

function hoverCellHtml(cell) {
  const hM=-fin(cell.center?.z);
  return [
    "<b>Obstacle cell</b>",
    `<div>height above takeoff: <code>${hM.toFixed(2)} m</code></div>`,
    `<div>occupied_score: <code>${fin(cell.occupied_score).toFixed(2)}</code></div>`,
    `<div>free_score:     <code>${fin(cell.free_score).toFixed(2)}</code></div>`,
    `<div>confidence:     <code>${fin(cell.confidence).toFixed(2)}</code></div>`,
    `<div class="muted">x=${fin(cell.center.x).toFixed(2)}, y=${fin(cell.center.y).toFixed(2)}, z=${fin(cell.center.z).toFixed(2)}</div>`,
  ].join("");
}

canvas.addEventListener("mousemove", e=>{
  if (dragging||!hoverCard) return;
  const rect=canvas.getBoundingClientRect();
  const px=(e.clientX-rect.left)*devicePixelRatio;
  const py=(e.clientY-rect.top)*devicePixelRatio;
  const cell=nearestObsCellToCanvas(px, py);
  if (!cell) { hoverCard.style.display="none"; return; }
  hoverCard.innerHTML=hoverCellHtml(cell);
  hoverCard.style.left=`${e.clientX+14}px`;
  hoverCard.style.top =`${e.clientY+14}px`;
  hoverCard.style.display="block";
});
canvas.addEventListener("mouseleave", ()=>{ if (hoverCard) hoverCard.style.display="none"; });

// ── canvas resize ──────────────────────────────────────────────────────────────

function resize() {
  const wrap=el("canvas-wrap");
  canvas.width  = Math.max(1, Math.floor(wrap.clientWidth  * devicePixelRatio));
  canvas.height = Math.max(1, Math.floor(wrap.clientHeight * devicePixelRatio));
  draw();
}
window.addEventListener("resize", resize);

// ── init ───────────────────────────────────────────────────────────────────────

installViewControls();
updateMetrics();
resize();
startLiveStream();
</script>
</body>
</html>
"""


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--output",
        default="build/viewer.html",
        help="Output HTML path (default: build/viewer.html)",
    )
    args = parser.parse_args()

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(HTML, encoding="utf-8")
    print(f"OK: wrote {output}")
    print(f"OK: {len(HTML):,} bytes")
    return 0


if __name__ == "__main__":
    sys.exit(main())
