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
import subprocess
import sys
from pathlib import Path


# Raw string: JS braces need no escaping because no Python str.format() is applied.
HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Dedalus</title>
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
.view-controls { display: grid; grid-template-columns: 1fr 1fr; gap: 5px; margin-bottom: 6px; }
.view-controls button { background: #1e2535; color: #d8dce8; border: 1px solid #333d54; border-radius: 5px; padding: 5px 6px; cursor: pointer; font-size: 12px; }
.view-controls button:hover { background: #28304a; }
.zoom-control { display: flex; align-items: center; gap: 4px; margin-bottom: 10px; }
.zoom-control button { background: #1e2535; color: #d8dce8; border: 1px solid #333d54; border-radius: 5px; padding: 4px 12px; cursor: pointer; font-size: 15px; font-weight: bold; line-height: 1; }
.zoom-control button:hover { background: #28304a; }
.zoom-control span { flex: 1; text-align: center; font-size: 12px; color: #c0c4d0; font-variant-numeric: tabular-nums; }
.layer-controls { display: flex; flex-direction: column; gap: 4px; margin-bottom: 8px; }
.layer-btn { display: flex; align-items: center; justify-content: space-between; width: 100%;
  background: #151b28; color: #6a738a; border: 1px solid #222a3a; border-radius: 5px;
  padding: 5px 10px; cursor: pointer; font-size: 12px; text-align: left; transition: none; }
.layer-btn:hover { background: #1a2235; color: #9099b0; }
.layer-btn.active { background: #1e2a44; color: #d8dce8; border-color: #3a5090; }
.layer-btn.active:hover { background: #253258; }
.layer-btn .ltog { width: 8px; height: 8px; border-radius: 50%; background: #2a2f3e; flex-shrink: 0; }
.layer-btn.active .ltog { background: #5080e8; box-shadow: 0 0 4px #5080e870; }
.lod-control { margin-bottom: 8px; }
.lod-row { display: flex; align-items: center; gap: 5px; margin-bottom: 2px; }
.lod-row input[type=range] { flex: 1; accent-color: #5080e8; cursor: pointer; }
.lod-end { font-size: 10px; color: #7a8099; white-space: nowrap; }
.trav-legend { display: flex; gap: 8px; align-items: center; margin: 4px 0 6px; flex-wrap: wrap; }
.trav-swatch { display: inline-block; width: 12px; height: 12px; border-radius: 2px; vertical-align: middle; margin-right: 3px; }
.trav-legend span { font-size: 11px; color: #9099b0; }
.trav-color-toggle { margin-bottom: 8px; }
.trav-mode-btn { display: flex; align-items: center; justify-content: space-between; width: 100%;
  background: #1e2535; color: #d8dce8; border: 1px solid #333d54; border-radius: 5px;
  padding: 5px 10px; cursor: pointer; font-size: 11px; margin-bottom: 5px; }
.trav-mode-btn:hover { background: #28304a; }
.trav-mode-btn .mode-tag { background: #2a3660; color: #7aa0f8; border-radius: 3px;
  padding: 1px 6px; font-size: 10px; font-weight: bold; letter-spacing: 0.03em; }
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
.metric-group { margin: 2px 0; }
.metric-group-hdr { display: flex; align-items: center; gap: 5px; padding: 3px 0;
  border-bottom: 1px solid #1e2330; font-size: 12px; cursor: pointer; user-select: none;
  position: relative; }
.metric-group-hdr:hover { background: #151e30; border-radius: 3px; }
.mg-caret { color: #5a6380; font-size: 10px; width: 10px; flex-shrink: 0; transition: none; }
.mg-label { color: #9099b0; flex: 1; }
.metric-group-hdr b { color: #e8e8e8; }
.metric-group-body { display: none; padding-left: 14px; }
.metric-group-body.open { display: block; }
.sub-metric { border-bottom-color: #181e28 !important; }
.copy-btn { display: none; background: none; border: none; color: #4070c8; cursor: pointer;
  padding: 0 3px; font-size: 13px; line-height: 1; opacity: 0.7; }
.copy-btn:hover { opacity: 1; color: #6090f0; }
.metric-group-hdr:hover .copy-btn { display: inline; }
.esdf-r-row { flex-direction: column !important; align-items: stretch !important; gap: 3px !important; }
.esdf-r-top { display: flex; justify-content: space-between; align-items: center; }
.esdf-r-slider { width: 100%; accent-color: #4a90e2; margin: 0; cursor: pointer; }
.esdf-r-labels { display: flex; justify-content: space-between; font-size: 10px; color: #4a5470; }
#debug-section { margin-top: 8px; display: none; }
#debug-log { max-height: 240px; overflow-y: auto; font-size: 10px; font-family: monospace;
  background: #090e18; border: 1px solid #1e2a3a; border-radius: 5px; padding: 5px; }
.dbg-entry { padding: 1px 0; border-bottom: 1px solid #111828; line-height: 1.35; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
.dbg-t  { color: #3a5070; margin-right: 4px; }
.dbg-type { font-weight: bold; margin-right: 4px; }
.dbg-type.t-esdf  { color: #40c080; }
.dbg-type.t-plan  { color: #6090e0; }
.dbg-type.t-trav  { color: #d0a040; }
.dbg-type.t-snap  { color: #9070d0; }
.dbg-type.t-other { color: #607080; }
.dbg-detail { color: #5a7090; }
.dbg-warn .dbg-detail { color: #d09040; }
.dbg-err  .dbg-detail { color: #d04040; }
.dbg-warn .dbg-label, .dbg-err .dbg-label { font-weight: bold; margin-right: 4px; }
.dbg-warn .dbg-label { color: #e0a030; }
.dbg-err  .dbg-label { color: #e03030; }
.height-ramp { height: 10px; border: 1px solid #3a4358; border-radius: 4px;
  background: linear-gradient(to right, #2641a8, #00a8d8, #3fbf6a, #e5d84c, #f08c2e, #d83b7d); margin: 4px 0; }
#hover-card { position: fixed; display: none; pointer-events: auto; cursor: pointer; z-index: 10;
  max-width: 290px; padding: 8px 10px; border: 1px solid #3a4358; border-radius: 7px;
  background: rgba(13, 16, 26, 0.96); box-shadow: 0 6px 18px rgba(0,0,0,0.4);
  color: #e8e8e8; font-size: 11px; line-height: 1.4; user-select: none; }
#hover-card b { color: #ffffff; }
#hover-card code { color: #a7d4ff; }
#hover-card .muted { color: #7a8099; }
#hover-card .copy-hint { color: #4a5270; font-size: 10px; margin-top: 4px; }
#hover-card.copied { border-color: #30d060; }
#hover-card .copy-flash { color: #30d060; font-size: 10px; margin-top: 4px; }
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
      <h2>Dedalus <span id="m-build-rev" style="font-size:10px;font-weight:400;color:#4a5470;margin-left:4px;">__BUILD_REV__</span></h2>
      <div id="m-region-id" style="font-size:11px;color:#7080a0;margin:-4px 0 4px;line-height:1.3;min-height:1em;"></div>
      <p class="hint">Drag to rotate · Wheel to zoom · Hover for cell detail</p>

      <div class="view-controls">
        <button id="view-center">Center</button>
        <button id="view-45">45°</button>
        <button id="view-side">Side</button>
        <button id="view-top">Top</button>
      </div>
      <div class="zoom-control">
        <button id="view-zoom-out">−</button>
        <span id="zoom-val">100%</span>
        <button id="view-zoom-in">+</button>
      </div>

      <h3>Layers</h3>
      <div class="layer-controls">
        <button class="layer-btn active" id="toggle-l0"><span>L0 ego radar</span><span class="ltog"></span></button>
        <button class="layer-btn active" id="toggle-trav"><span>L1 traversability</span><span class="ltog"></span></button>
        <div id="trav-lod-control" style="padding:2px 8px 5px;border-bottom:1px solid #1a2030;">
          <div class="lod-row">
            <span class="lod-end">32m</span>
            <input type="range" id="trav-lod" min="0" max="6" step="1" value="2">
            <span class="lod-end">0.5m</span>
          </div>
          <div style="text-align:center;font-size:11px;color:#9099b0">L1 cell size: <b id="trav-lod-val" style="color:#c0c4d0">8m</b> · L2 native: 1m</div>
        </div>
        <button class="layer-btn active" id="toggle-planning"><span>L2 planning map</span><span class="ltog"></span></button>
        <button class="layer-btn active" id="toggle-esdf"><span>L3 ESDF</span><span class="ltog"></span></button>
        <div id="esdf-r-control" style="padding:2px 8px 5px;border-bottom:1px solid #1a2030;">
          <div style="display:flex;justify-content:space-between;align-items:center;font-size:11px;color:#9099b0;margin-bottom:2px;">
            <span>smooth R</span>
            <span style="display:flex;align-items:center;gap:5px;">
              <b id="m-esdf-r" style="color:#e8e8e8;">2.0m</b>
              <label style="color:#5a6380;font-size:10px;cursor:pointer;display:flex;align-items:center;gap:2px;">
                <input type="checkbox" id="esdf-vel-mode" onchange="buildESDFArrows();scheduleDraw()" style="margin:0;">vel
              </label>
            </span>
          </div>
          <input type="range" id="esdf-r-slider" class="esdf-r-slider"
            min="1" max="20" step="0.5" value="5" oninput="onEsdfRSlider(this.value)">
          <div style="display:flex;justify-content:space-between;font-size:9px;color:#4a5470;margin-top:1px;">
            <span>0.5m</span><span>d₀</span>
          </div>
          <label style="color:#5a6380;font-size:10px;cursor:pointer;display:flex;align-items:center;gap:4px;margin-top:4px;">
            <input type="checkbox" id="esdf-show-cells" onchange="onEsdfShowCells(this.checked)" style="margin:0;">
            show cells
          </label>
        </div>
        <button class="layer-btn" id="toggle-obstacles"><span>Raw evidence</span><span class="ltog"></span></button>
        <button class="layer-btn" id="toggle-ghosts"><span>Ghost detections</span><span class="ltog"></span></button>
        <button class="layer-btn active" id="toggle-sensing"><span>Sensing volumes</span><span class="ltog"></span></button>
        <button class="layer-btn active" id="toggle-trajectory"><span>Trajectory</span><span class="ltog"></span></button>
      </div>

      <div class="trav-color-toggle">
        <button class="trav-mode-btn" id="trav-color-btn">
          L1 &amp; L2 color mode <span class="mode-tag" id="trav-color-tag">Height</span>
        </button>
        <div id="trav-legend-height">
          <div class="height-ramp" style="margin:2px 0 4px"></div>
          <div style="display:flex;justify-content:space-between;font-size:10px;color:#7a8099"><span>low</span><span>high</span></div>
        </div>
        <div id="trav-legend-type" class="trav-legend" style="visibility:hidden">
          <span><span class="trav-swatch" style="background:rgba(190,55,45,0.85)"></span>Occupied</span>
          <span><span class="trav-swatch" style="background:rgba(200,150,30,0.6)"></span>Partial</span>
          <span><span class="trav-swatch" style="background:transparent;border:1px solid #3a4358"></span>Free</span>
        </div>
      </div>

      <h3>Metrics</h3>
      <div class="metric"><span>Stream source</span><code id="m-source">—</code></div>
      <div class="metric-group" id="pipeline-group">
        <div class="metric-group-hdr" id="pipeline-hdr" onclick="togglePipelineMetrics()">
          <span class="mg-caret" id="pipeline-caret">▶</span>
          <span class="mg-label">Pipeline</span>
          <code id="m-pipeline-ego" style="font-size:10px;color:#7a8aaa;">—</code>
        </div>
        <div class="metric-group-body" id="pipeline-body">
          <div class="metric sub-metric"><span>ego</span><code id="m-pl-ego">—</code></div>
          <div class="metric sub-metric"><span>depth</span><code id="m-pl-depth">—</code></div>
          <div class="metric sub-metric"><span>detector</span><code id="m-pl-detector">—</code></div>
          <div class="metric sub-metric"><span>stabilizer</span><code id="m-pl-stabilizer">—</code></div>
          <div class="metric sub-metric"><span>tracker</span><code id="m-pl-tracker">—</code></div>
          <div class="metric sub-metric"><span>identity</span><code id="m-pl-identity">—</code></div>
          <div class="metric sub-metric"><span>projector</span><code id="m-pl-projector">—</code></div>
        </div>
      </div>
      <div class="metric"><span>Seq</span><b id="m-seq">0</b></div>
      <div class="metric"><span>L0 ego cells</span><b id="m-l0-cells">0</b></div>
      <div class="metric"><span>L2 planning cells</span><b id="m-plan-cells">0</b></div>
      <div class="metric-group" id="l3-group">
        <div class="metric-group-hdr" id="l3-hdr" onclick="toggleL3Metrics()">
          <span class="mg-caret" id="l3-caret">▶</span>
          <span class="mg-label">L3 ESDF cells</span>
          <b id="m-esdf-cells">0</b>
          <button class="copy-btn" onclick="copyL3Metrics(event)" title="Copy L3 stats">⧉</button>
        </div>
        <div class="metric-group-body" id="l3-body">
          <div class="metric sub-metric"><span>msgs rcvd</span><b id="m-esdf-msgs">0</b></div>
          <div class="metric sub-metric"><span>ESDF faces</span><b id="m-esdf-faces">0</b></div>
          <div class="metric sub-metric"><span>d range</span><b id="m-esdf-drange">—</b></div>
          <div class="metric sub-metric"><span>arrows</span><b id="m-esdf-arrows">0</b></div>
        </div>
      </div>
      <div class="metric"><span>L1 trav cells</span><b id="m-trav-cells">0</b></div>
      <div class="metric"><span>Trav ext. faces</span><b id="m-trav-faces">0</b></div>
      <div class="metric"><span>Raw evidence cells</span><b id="m-obs-cells">0</b></div>
      <div class="metric"><span>Ghost detections</span><b id="m-ghosts">0</b></div>
      <div class="metric"><span>World snapshots</span><b id="m-ws-count">0</b></div>
      <div class="metric"><span>Ego updates</span><b id="m-ego-count">0</b></div>
      <div class="metric"><span>Trajectory pts</span><b id="m-traj">0</b></div>
      <div class="metric"><span>Bounds</span><code id="m-bounds">—</code></div>

      <h3>Legend</h3>
      <div class="hint">Height above takeoff (obstacle colors)</div>
      <div class="height-ramp"></div>
      <div style="display:grid;grid-template-columns:repeat(5,1fr);font-size:10px;color:#6a7090;margin-bottom:6px">
        <span>0m</span><span style="text-align:center">10m</span><span style="text-align:center">20m</span><span style="text-align:center">30m</span><span style="text-align:right">40m+</span>
      </div>
      <div class="hint" style="margin-bottom:2px">
        Yellow = ego/path · White = sensing direction · Magenta = first blocked ·
        L0 radar (top-right): red=occupied, amber=inflated ·
        L2 planning: slate-blue persistent voxels ·
        L1 trav: red=occupied, amber=partial, shading=depth ·
        Cyan dot = ghost · Arrow = velocity
      </div>

      <div id="event-log-section">
        <h3>Mission Events</h3>
        <div id="event-log"><span style="color:#4a5070;font-size:11px">Waiting for mission events…</span></div>
      </div>

      <div id="debug-section">
        <h3>SSE Debug Log</h3>
        <div id="debug-log"><span style="color:#3a5070;font-size:10px">Waiting for events…</span></div>
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
const MAX_TRAV_DISPLAY       = 8000;   // cap on aggregated occupied voxels before face build
const TRAV_LOD_LEVELS        = [32, 16, 8, 4, 2, 1, 0.5]; // metres, index 2 = default 8m
const MAX_GHOST_AGE_S        = 15;
const MAX_TRAJ_POINTS        = 6000;
const TRAJ_MAX_GAP_M         = 1.0;   // skip segments with spatial jumps > 1 m (latency / discontinuity)
const TRAJ_MAX_GAP_MS        = 3000;  // skip if time between consecutive trajectory points > 3 s

// Sensor cone scope: camera FOV and depth-grid dimensions.
// Must match config/drone/px4_front_center.yaml + emulation detector config.
const CONE_AZ_HALF_DEG = 42;      // 84° hfov / 2
const CONE_EL_HALF_DEG = 26.86;   // 53.72° vfov / 2
const CONE_GRID_COLS   = 40;      // depth_grid_cols — az sample count
const CONE_GRID_ROWS   = 22;      // depth_grid_rows — el sample count

// Map-view nudge: shift the rendered scene center right + down so the
// top-left cone scope panel occludes less of the map content.
// Tuned to ~25 % of the panel's expected CSS dimensions (480 × 285 px).
const MAP_NUDGE_X_CSS = 110;   // rightward shift in CSS pixels
const MAP_NUDGE_Y_CSS =  75;   // downward  shift in CSS pixels
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
  takeoffPos: null,          // first recorded ego position (home / launch point)
  trajectory: [],           // {x,y,z,live_seen_ms}
  sensingOverlays: [],
  firstBlocked: null,
  bounds: {min_x:-10, max_x:10, min_y:-10, max_y:10, min_z:-5, max_z:5},
  diagnostics: {},
  showPlanning:    true,    // L2 persistent planning map (checked by default)
  showObstacles:   false,   // raw evidence (demoted — unchecked by default)
  showTrav:        true,    // L1 traversability (primary)
  showGhosts:      false,
  showSensing:     true,
  showTrajectory:  true,
  showL0:          true,    // L0 ego radar inset (top-right corner)
  showEsdf:        true,    // L3 ESDF (on by default — peer of L2)
  showEsdfCells:   false,   // L3 cell faces (off by default — arrows only)
  travColorByType: false,   // false = height-based (default), true = occupied/partial type colors
  perchCandidates: [],      // PerchCandidate list from latest snapshot
};

// L2 planning cells (persistent, slate-blue base layer)
const planningCellsByKey = new Map();  // quantKey → planning cell
// True once the server sends a live planning_map_delta (cells updated this session).
// Until then, all L2 cells are from the prior-session DB load and render muted.
// L3 arrows also render muted until this becomes true (ESDF is derived from L2).
let l2HasLiveData = false;

// Obstacle cells (raw evidence)
const obsCellsByKey = new Map();       // cellKey → normalized cell
const liveObsEventsByKey = new Map();
let   liveObsEvents = [];              // for age-decay rendering

// L0 ego-local flight map (ego-local Cartesian, up to 96 occupied/inflated cells)
let localFlightMapCells = [];     // parsed debug_cells — ego-local (X=fwd, Y=right, Z=down)
let lfmNearestM = Infinity;       // nearest obstacle range among debug_cells
let lfmCellSizeM = 0.5;
let lfmForwardRangeM = 30;
let lfmRearRangeM = 6;
let lfmLateralRangeM = 15;
// Pre-computed risk fields from C++ (compute_l0_polar_risk + collect_l0_sensor_observations)
let lfmEgoSpeedMps   = 0;         // m/s
let lfmGlobalMinTTC  = Infinity;  // s
let lfmEscapeBody    = null;      // {x,y,z} unit vec body frame, or null
let lfmPolarSectors  = [];        // [{az,vr,ttc,nr,obs}] 36 az sectors (1-D)
let lfmSphericalBins = [];        // [{az,el,ttc,vr,nr,sm}] occupied bins only (2-D az×el)
let lfmSphNumAz      = 36;
let lfmSphNumEl      = 9;
let lfmSensorObs     = [];        // [{az,el,r,vr,ttc,src}] raw sensor observations
// Authoritative sensor scope from C++ — FOV half-angles and depth grid dims.
let lfmSensorAzHalfRad = 0;
let lfmSensorElHalfRad = 0;
let lfmSensorGridCols  = 0;
let lfmSensorGridRows  = 0;

// ── Debug mode (?debug in URL) ────────────────────────────────────────────────
const DEBUG = new URLSearchParams(window.location.search).has("debug");
const MAX_DEBUG_LOG = 60;
const debugEntries = [];   // [{ts, type, detail}]
// dbgLog(type, detail, level)
// level: "info" (default) — collapses with the previous INFO entry if it is
//        the last item in the log (i.e., consecutive INFOs overwrite in place).
//        "warn" / "error" — always appended and never collapsed.
// Order is preserved: a WARN/ERROR locks the preceding INFO in place.
let _dbgLastInfoDiv = null;  // tracks the most-recently-appended INFO div
function dbgLog(type, detail, level = "info") {
  if (!DEBUG) return;
  const ts = new Date().toISOString().substr(11, 12);
  debugEntries.push({ts, type, detail, level});
  if (debugEntries.length > MAX_DEBUG_LOG) debugEntries.shift();

  const logEl = el("debug-log");
  if (logEl.firstChild?.tagName === undefined) logEl.innerHTML = "";  // clear placeholder

  const cls = type.includes("esdf")  ? "t-esdf"
             : type.includes("plan") ? "t-plan"
             : type.includes("trav") ? "t-trav"
             : type.includes("snap") ? "t-snap"
             : "t-other";

  if (level === "info") {
    // Reuse the last INFO div if it is still the last child (no WARN/ERROR since).
    if (_dbgLastInfoDiv && _dbgLastInfoDiv === logEl.lastChild) {
      _dbgLastInfoDiv.innerHTML = `<span class="dbg-t">${ts}</span><span class="dbg-type ${cls}">${escHtml(type)}</span><span class="dbg-detail">${escHtml(detail)}</span>`;
    } else {
      const div = document.createElement("div");
      div.className = "dbg-entry";
      div.innerHTML = `<span class="dbg-t">${ts}</span><span class="dbg-type ${cls}">${escHtml(type)}</span><span class="dbg-detail">${escHtml(detail)}</span>`;
      logEl.appendChild(div);
      _dbgLastInfoDiv = div;
      while (logEl.children.length > MAX_DEBUG_LOG) logEl.removeChild(logEl.firstChild);
    }
  } else {
    // WARN / ERROR: always a new entry; locks any preceding INFO in place.
    _dbgLastInfoDiv = null;
    const isErr = level === "error";
    const div = document.createElement("div");
    div.className = `dbg-entry ${isErr ? "dbg-err" : "dbg-warn"}`;
    div.innerHTML = `<span class="dbg-t">${ts}</span><span class="dbg-label">${isErr ? "ERROR" : "WARN"}</span><span class="dbg-type ${cls}">${escHtml(type)}</span><span class="dbg-detail"> ${escHtml(detail)}</span>`;
    logEl.appendChild(div);
    while (logEl.children.length > MAX_DEBUG_LOG) logEl.removeChild(logEl.firstChild);
  }
  logEl.scrollTop = logEl.scrollHeight;
}

// L3 ESDF (Stage 6): shell cells from C++ compute_esdf, emitted as esdf_delta SSE events.
const esdfCellsByKey   = new Map(); // quantKey → {x,y,z,d,gx,gy,gz} — merged DB+live view
const esdfDbCellsByKey = new Map(); // DB-loaded baseline (map_seq=0 burst); never cleared by live snapshots
let   esdfNetRepulsion = null;      // {x,y,z} world-frame APF repulsion at drone position
let   esdfD0M        = 5.0;        // truncation radius from last snapshot
let   esdfMsgCount   = 0;          // total esdf_delta messages received
let   esdfConsecErrors = 0;        // consecutive parse failures (resets on success)
let   esdfDMin       = Infinity;   // min d across all stored cells (for stats)
let   esdfDMax       = -Infinity;  // max d across all stored cells
let   esdfArrowGeom  = [];         // sparse smoothed arrows pre-built by buildESDFArrows()
let   esdfCellSizeM  = 1.0;        // XY cell size (inherited from L2)
let   esdfVCellSizeM = 2.0;        // Z  cell size (inherited from L2)
let   esdfFacesGeom  = [];         // pre-built exterior face geometry for L3
let   esdfSmoothR    = 5.0;        // Gaussian averaging radius (metres) for arrow visualization
const ESDF_A_MAX     = 3.0;        // assumed max decel (m/s²) used for velocity-based R

// Traversability (L1)
const travCellsByKey = new Map();      // cellKey → trav cell (0.5 m raw cells from server)
let   travOccupiedQKeys = new Set();   // raw quantized key → present (used by delta merge)
let   travFacesGeom = [];              // pre-built exterior face geometry (at current LOD)
let   travCellSizeM = 0.5;
let   travVCellSizeM = 0.5;
let   travDisplayLevelM = 8;           // current LOD cell size for rendering (metres)

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
  pendingPlanningSnapshot: null,
  pendingPlanningIsDelta: false,
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
  const dpr   = devicePixelRatio;
  return {
    x: 0.5*canvas.width  + MAP_NUDGE_X_CSS*dpr + x1*scale,
    y: 0.80*canvas.height + MAP_NUDGE_Y_CSS*dpr - z2*scale,
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
    updateZoomDisplay();
    draw();
    if (rawT < 1) { window.viewAnimationHandle=requestAnimationFrame(step); }
    else { yaw=tYaw; pitch=tPitch; zoom=tZoom; viewCenter=cloneP(fCenter); window.viewAnimationHandle=null; updateZoomDisplay(); draw(); }
  }
  window.viewAnimationHandle=requestAnimationFrame(step);
};

function updateZoomDisplay() {
  el("zoom-val").textContent = Math.round(zoom * 100) + "%";
}

window.animateZoom = function(targetZoom) {
  if (window.viewAnimationHandle !== null) {
    cancelAnimationFrame(window.viewAnimationHandle);
    window.viewAnimationHandle = null;
  }
  const startZoom = zoom;
  const dur = 300, t0 = performance.now();
  function step(now) {
    const rawT = clamp((now - t0) / dur, 0, 1), t = window.easeInOutCubic(rawT);
    zoom = startZoom + (targetZoom - startZoom) * t;
    updateZoomDisplay();
    draw();
    if (rawT < 1) { window.viewAnimationHandle = requestAnimationFrame(step); }
    else { zoom = targetZoom; updateZoomDisplay(); window.viewAnimationHandle = null; draw(); }
  }
  window.viewAnimationHandle = requestAnimationFrame(step);
};

// ── L0 ego radar inset ─────────────────────────────────────────────────────────
//
// Fixed top-right canvas inset — ±75° FOV sector, 12 o'clock = body heading.
//
// RISK MODEL: per-cell risk = radial closing speed (m/s).
//   v_r = dot(ego_velocity_body, cell_unit_dir_body).  TTC = dist / max(ε, v_r).
//   Cells with v_r ≤ 0 (receding) are coloured green — no collision risk.
//
// Cell colour:
//   Moving (speed > 0.3 m/s):  TTC ramp — red < 2 s, yellow < 5 s, green > 10 s
//   Hover / takeoff / landing:  distance ramp — red < 3 m, green > 22 m
//
// Escape vector: closing-speed-weighted centroid of occupied cells.
//   Points away from threats the drone is most actively converging on.
//   Hover fallback: distance-weighted centroid.
//
// Radial scale: sqrt(dist) — denser rings close-in, sparser far out.

const _L0_FOV_DEG  = 75;    // half-FOV angle
const _L0_INSET_RL = 184;   // logical radius px (80 % of original 230)

function _l0DistColor(distM) {
  // Distance → RGB for hover/stationary mode
  if (distM <  3) return [228, 42,  30];
  if (distM < 10) {
    const t = (distM - 3) / 7;
    return [228, Math.round(42 + t * 158), Math.round(30 - t * 15)];
  }
  if (distM < 22) {
    const t = (distM - 10) / 12;
    return [Math.round(228 - t * 188), Math.round(200 + t * 10), 15];
  }
  return [42, 210, 65];
}

function _l0RiskColor(ttcS) {
  // TTC (s) → RGB for moving mode
  if (!Number.isFinite(ttcS) || ttcS > 10) return [42, 210, 65];
  if (ttcS > 5) {
    const t = (10 - ttcS) / 5;
    return [Math.round(42 + t * 186), 210, Math.round(65 - t * 55)];
  }
  if (ttcS > 2) {
    const t = (5 - ttcS) / 3;
    return [228, Math.round(210 - t * 168), 10];
  }
  if (ttcS > 0.8) {
    const t = (2 - ttcS) / 1.2;
    return [228, Math.round(42 - t * 28), 10];
  }
  return [238, 14, 8];
}

function _drawRadarArrow(ctx, x1, y1, x2, y2, color, lineW, headLen) {
  const dx = x2 - x1, dy = y2 - y1;
  const len = Math.hypot(dx, dy);
  if (len < 2) return;
  const ux = dx / len, uy = dy / len;
  const hl = Math.min(headLen, len * 0.42);
  const hw = hl * 0.44;
  const perp = [-uy, ux];
  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x2 - ux * hl * 0.55, y2 - uy * hl * 0.55);
  ctx.strokeStyle = color; ctx.lineWidth = lineW; ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(x2, y2);
  ctx.lineTo(x2 - ux * hl + perp[0] * hw, y2 - uy * hl + perp[1] * hw);
  ctx.lineTo(x2 - ux * hl - perp[0] * hw, y2 - uy * hl - perp[1] * hw);
  ctx.closePath();
  ctx.fillStyle = color; ctx.fill();
}

// Lookup the C++-computed TTC for a cell by matching its body-frame azimuth
// to the nearest polar sector in lfmPolarSectors.
function _l0CellTTC(cell) {
  if (!lfmPolarSectors.length) return Infinity;
  const az = Math.atan2(cell.center.y, cell.center.x) * (180 / Math.PI); // −180…+180
  // Find the sector whose centre is nearest (wrap-safe via abs-diff ≤ 180).
  let best = null, bestDiff = Infinity;
  for (const s of lfmPolarSectors) {
    let d = Math.abs(s.az - az);
    if (d > 180) d = 360 - d;
    if (d < bestDiff) { bestDiff = d; best = s; }
  }
  return best ? best.ttc : Infinity;
}

function _l0HeadingStr(yawRad) {
  if (yawRad === null || !Number.isFinite(yawRad)) return '—';
  const deg = ((yawRad * 180 / Math.PI) % 360 + 360) % 360;
  const card = ['N','NNE','NE','ENE','E','ESE','SE','SSE',
                 'S','SSW','SW','WSW','W','WNW','NW','NNW'][Math.round(deg / 22.5) % 16];
  return `${Math.round(deg)}° ${card}`;
}

function drawL0PolarInset() {
  if (!state.showL0) return;

  const dpr     = devicePixelRatio;
  const MARGIN  = 24 * dpr;
  const INSET_R = _L0_INSET_RL * dpr;           // 230 × dpr device px
  const CX      = canvas.width - INSET_R - MARGIN;
  const CY      = INSET_R + MARGIN;

  // TTC-radial scale: centre=0s, edge=MAX_TTC_S.
  // Obstacles are positioned by reaction time, not physical distance.
  const MAX_TTC_S = 10;                          // seconds at outer edge
  const TTC_RINGS = [1, 2, 3, 5, 10];           // ring labels (s)
  const USABLE_R  = INSET_R - 8 * dpr;

  function ttcRadius(ttcS) {
    if (!Number.isFinite(ttcS) || ttcS >= MAX_TTC_S) return USABLE_R;
    return Math.max(4 * dpr, (ttcS / MAX_TTC_S) * USABLE_R);
  }

  // Body-frame azimuth → canvas XY at a given radius.
  function azRadiusToPx(azRad, rPx) {
    return [CX + Math.sin(azRad) * rPx, CY - Math.cos(azRad) * rPx];
  }

  const TS  = 14 * dpr;
  const TM  = 16 * dpr;
  const TL  = 18 * dpr;
  const TXL = 20 * dpr;

  const UP   = -Math.PI / 2;
  const FOVR = _L0_FOV_DEG * Math.PI / 180;
  const ANG0 = UP - FOVR;
  const ANG1 = UP + FOVR;

  const isMoving = lfmEgoSpeedMps > 0.3;
  const DOT_R    = 7 * dpr;

  ctx.save();

  // ── Sector background ─────────────────────────────────────────────────────────
  ctx.beginPath();
  ctx.moveTo(CX, CY);
  ctx.arc(CX, CY, INSET_R, ANG0, ANG1);
  ctx.closePath();
  ctx.fillStyle   = 'rgba(5,8,17,0.96)';
  ctx.fill();
  ctx.strokeStyle = 'rgba(55,82,168,0.90)';
  ctx.lineWidth   = 2 * dpr;
  ctx.stroke();

  // ── Clip to sector ────────────────────────────────────────────────────────────
  ctx.beginPath();
  ctx.moveTo(CX, CY);
  ctx.arc(CX, CY, INSET_R - 2 * dpr, ANG0, ANG1);
  ctx.closePath();
  ctx.clip();

  // ── TTC rings ─────────────────────────────────────────────────────────────────
  for (const ts of TTC_RINGS) {
    const rPx = ttcRadius(ts);
    ctx.beginPath();
    ctx.arc(CX, CY, rPx, ANG0, ANG1);
    ctx.strokeStyle = ts <= 2 ? 'rgba(200,60,40,0.22)' : 'rgba(55,80,160,0.18)';
    ctx.lineWidth   = (ts <= 2 ? 1.2 : 0.6) * dpr;
    ctx.stroke();
  }

  // ── Centre-line ───────────────────────────────────────────────────────────────
  ctx.strokeStyle = 'rgba(80,110,215,0.14)';
  ctx.lineWidth   = 0.5 * dpr;
  ctx.beginPath(); ctx.moveTo(CX, CY); ctx.lineTo(CX, CY - INSET_R + 3 * dpr); ctx.stroke();

  // ── Obstacle cells — placed at TTC radius, coloured by TTC ───────────────────
  for (const cell of localFlightMapCells) {
    if (!cell.occupied) continue;
    const azRad = Math.atan2(cell.center.y, cell.center.x); // body: atan2(right,fwd)
    const ttc   = _l0CellTTC(cell);
    const dist  = Math.hypot(cell.center.x, cell.center.y);
    const rPx   = isMoving ? ttcRadius(ttc) : ttcRadius(dist); // hover: dist as proxy
    const [px, py] = azRadiusToPx(azRad, rPx);
    const [r, g, b] = isMoving ? _l0RiskColor(ttc) : _l0DistColor(dist);
    const alpha = Math.min(0.97, 0.72 + cell.occupied_score * 0.25);
    ctx.fillStyle   = `rgba(${r},${g},${b},${alpha})`;
    ctx.strokeStyle = `rgba(${r},${g},${b},0.92)`;
    ctx.lineWidth   = dpr;
    ctx.beginPath(); ctx.arc(px, py, DOT_R, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
  }

  // ── Escape vector ─────────────────────────────────────────────────────────────
  if (lfmEscapeBody && (Math.abs(lfmEscapeBody.x) + Math.abs(lfmEscapeBody.y)) > 0.1) {
    const ARM = INSET_R * 0.62;
    _drawRadarArrow(ctx, CX, CY,
      CX + lfmEscapeBody.y * ARM, CY - lfmEscapeBody.x * ARM,
      'rgba(35,228,95,0.95)', 3 * dpr, 18 * dpr);
  }

  // ── L3 ESDF net repulsion (amber, planning guidance) ─────────────────────────
  if (esdfNetRepulsion) {
    const yaw=state.egoYaw||0;
    const bx= esdfNetRepulsion.x*Math.cos(yaw)+esdfNetRepulsion.y*Math.sin(yaw);
    const by=-esdfNetRepulsion.x*Math.sin(yaw)+esdfNetRepulsion.y*Math.cos(yaw);
    const mag=Math.hypot(bx,by);
    if (mag>0.001) {
      const nx=bx/mag,ny=by/mag;
      const ARM=INSET_R*0.55;
      _drawRadarArrow(ctx,CX,CY,CX+ny*ARM,CY-nx*ARM,
        'rgba(255,165,0,0.90)',3*dpr,16*dpr);
    }
  }

  // ── Flight vector (body forward = 12 o'clock) ─────────────────────────────────
  if (isMoving) {
    _drawRadarArrow(ctx, CX, CY, CX, CY - INSET_R * 0.52,
      'rgba(80,208,255,0.97)', 4 * dpr, 20 * dpr);
  }

  // ── Ego dot ───────────────────────────────────────────────────────────────────
  ctx.fillStyle   = 'rgba(255,236,68,1.0)';
  ctx.beginPath(); ctx.arc(CX, CY, 6 * dpr, 0, Math.PI * 2); ctx.fill();
  ctx.strokeStyle = 'rgba(0,0,0,0.72)'; ctx.lineWidth = 1.5 * dpr; ctx.stroke();

  ctx.restore();

  // ── External labels ───────────────────────────────────────────────────────────
  ctx.save();

  ctx.font = `${TM}px system-ui`;
  ctx.fillStyle = 'rgba(58,78,162,0.72)';
  ctx.textAlign = 'center'; ctx.textBaseline = 'bottom';
  ctx.fillText('L0 TTC radar', CX, CY - INSET_R - 24 * dpr);

  ctx.font = `bold ${TL}px system-ui`;
  ctx.fillStyle = 'rgba(110,142,238,0.92)';
  ctx.textAlign = 'center'; ctx.textBaseline = 'bottom';
  ctx.fillText(`▲ ${_l0HeadingStr(state.egoYaw)}`, CX, CY - INSET_R - 1 * dpr);

  // TTC ring labels
  ctx.font = `${TS}px system-ui`;
  ctx.fillStyle = 'rgba(55,75,158,0.72)';
  ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
  for (const ts of TTC_RINGS) {
    const rPx = ttcRadius(ts);
    if (rPx < INSET_R - 5 * dpr) ctx.fillText(`${ts}s`, CX + 4 * dpr, CY - rPx);
  }

  // Risk readout
  let readoutStr, readoutRGB;
  if (isMoving && Number.isFinite(lfmGlobalMinTTC)) {
    readoutStr = `TTC  ${lfmGlobalMinTTC.toFixed(1)} s`;
    readoutRGB = _l0RiskColor(lfmGlobalMinTTC);
  } else if (Number.isFinite(lfmNearestM) && lfmNearestM < 9999) {
    readoutStr = `${lfmNearestM.toFixed(1)} m`;
    readoutRGB = _l0DistColor(lfmNearestM);
  } else {
    readoutStr = '—';
    readoutRGB = [128, 148, 210];
  }

  let labelY = CY + 10 * dpr;
  const [rr, rg, rb] = readoutRGB;
  ctx.font = `bold ${TS}px system-ui`;
  ctx.fillStyle = `rgba(${rr},${rg},${rb},0.98)`;
  ctx.textAlign = 'center'; ctx.textBaseline = 'top';
  ctx.fillText(readoutStr, CX, labelY);
  labelY += TS + 6 * dpr;

  if (!isMoving) {
    ctx.font = `bold ${TM}px system-ui`;
    ctx.fillStyle = 'rgba(95,118,200,0.72)';
    ctx.textAlign = 'center'; ctx.textBaseline = 'top';
    ctx.fillText('● HOVER', CX, labelY);
    labelY += TM + 6 * dpr;
  }

  ctx.font = `${TS}px system-ui`;
  ctx.textBaseline = 'top';
  ctx.fillStyle = 'rgba(80,208,255,0.82)'; ctx.textAlign = 'right';
  ctx.fillText('▶ flt', CX - 6 * dpr, labelY);
  ctx.fillStyle = 'rgba(35,228,95,0.82)';  ctx.textAlign = 'left';
  ctx.fillText('▶ esc', CX + 6 * dpr, labelY);

  ctx.restore();
}

// ── L0 sensor cone scope (az × el, source-tagged, TTC-coloured) ──────────────
// Rectilinear projection of the sensor frustum. Each dot = one raw evidence
// observation from C++, placed at its body-frame (az, el) angles.
// Color outer ring = TTC risk; color inner dot = sensor source.
function drawL0ConeScope() {
  if (!state.showL0) return;
  // Only draw when we have observations or spherical bins.
  if (!lfmSensorObs.length && !lfmSphericalBins.length) return;

  const dpr = devicePixelRatio;

  // Panel: anchored top-left, below the sidebar gap.
  const INSET_R  = _L0_INSET_RL * dpr;
  const MARGIN   = 24 * dpr;
  const PAD      = { l: 32*dpr, r: 10*dpr, t: 20*dpr, b: 24*dpr };
  const W        = Math.min(480 * dpr, INSET_R * 2 + MARGIN);  // cap width
  const PW       = W - PAD.l - PAD.r;

  // Authoritative sensor scope from C++ — no estimation.
  // C++ stamps sensor_az_half_rad/el/grid_cols/rows in the SSE stream after each
  // depth-slot detect; fall back to config constants only before the first frame arrives.
  const azHalf  = lfmSensorAzHalfRad > 0
    ? lfmSensorAzHalfRad * (180 / Math.PI) : CONE_AZ_HALF_DEG;
  const elHalf  = lfmSensorElHalfRad > 0
    ? lfmSensorElHalfRad * (180 / Math.PI) : CONE_EL_HALF_DEG;
  const estCols = lfmSensorGridCols > 0 ? lfmSensorGridCols : CONE_GRID_COLS;
  const estRows = lfmSensorGridRows > 0 ? lfmSensorGridRows : CONE_GRID_ROWS;

  const CELL     = PW / estCols;        // px per az sample
  const PH       = CELL * estRows;      // square-cell plot height
  const H        = PH + PAD.t + PAD.b;
  const panX     = MARGIN;
  const panY     = MARGIN;
  const X0       = panX + PAD.l;
  const Y0       = panY + PAD.t;

  const AZ_MIN = -azHalf, AZ_MAX = azHalf;
  const EL_MIN = -elHalf, EL_MAX = elHalf;

  function azPx(az_rad) {
    const deg = az_rad * (180 / Math.PI);
    return X0 + ((deg - AZ_MIN) / (AZ_MAX - AZ_MIN)) * PW;
  }
  function elPx(el_rad) {
    const deg = el_rad * (180 / Math.PI);
    return Y0 + (1 - (deg - EL_MIN) / (EL_MAX - EL_MIN)) * PH;
  }
  function azDegPx(deg) { return X0 + ((deg - AZ_MIN) / (AZ_MAX - AZ_MIN)) * PW; }
  function elDegPx(deg) { return Y0 + (1 - (deg - EL_MIN) / (EL_MAX - EL_MIN)) * PH; }

  // Source bitmask → inner dot color
  // GT = white, Depth = mid-gray, Visual = black
  function srcColor(src) {
    if (src & 0x01) return 'rgba(255,255,255,0.95)';  // AirSimGT
    if (src & 0x02) return 'rgba(140,140,140,0.90)';  // DepthProvider
    if (src & 0x04) return 'rgba(20,20,20,0.95)';     // VisualObstacle
    return 'rgba(140,140,140,0.70)';                   // other
  }

  ctx.save();

  // Panel background (panX/panY already computed above)
  ctx.fillStyle   = 'rgba(5,8,17,0.94)';
  ctx.strokeStyle = 'rgba(55,82,168,0.80)';
  ctx.lineWidth   = 1.5 * dpr;
  ctx.beginPath();
  ctx.rect(panX, panY, W, H);
  ctx.fill(); ctx.stroke();

  // Clip to panel
  ctx.beginPath(); ctx.rect(panX + PAD.l, panY + PAD.t, PW, PH); ctx.clip();

  // Grid
  ctx.setLineDash([2*dpr, 4*dpr]);
  ctx.strokeStyle = 'rgba(55,80,160,0.16)'; ctx.lineWidth = 0.5 * dpr;
  for (const az of [-azHalf, -azHalf/2, 0, azHalf/2, azHalf]) {
    const x = azDegPx(az);
    ctx.beginPath(); ctx.moveTo(x, Y0); ctx.lineTo(x, Y0 + PH); ctx.stroke();
  }
  for (const el of [-elHalf, 0, elHalf]) {
    const y = elDegPx(el);
    ctx.beginPath(); ctx.moveTo(X0, y); ctx.lineTo(X0 + PW, y); ctx.stroke();
  }
  ctx.setLineDash([]);

  // Boresight cross-hair
  ctx.strokeStyle = 'rgba(80,110,215,0.16)'; ctx.lineWidth = 0.5 * dpr;
  ctx.beginPath(); ctx.moveTo(azDegPx(0), Y0); ctx.lineTo(azDegPx(0), Y0+PH); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(X0, elDegPx(0)); ctx.lineTo(X0+PW, elDegPx(0)); ctx.stroke();

  // ── Spherical bins — background heat (occupied bins as faint colored cells) ──
  for (const b of lfmSphericalBins) {
    const [r, g, bl] = _l0RiskColor(b.ttc);
    const x = azDegPx(b.az);
    const azW = PW / lfmSphNumAz;
    const elH = PH / lfmSphNumEl;
    const y = elDegPx(b.el) - elH * 0.5;
    ctx.fillStyle = `rgba(${r},${g},${bl},0.18)`;
    ctx.fillRect(x - azW * 0.5, y, azW, elH);
  }

  // ── Per-sensor observations — dot per raw evidence contact ────────────────────
  const isMoving = lfmEgoSpeedMps > 0.3;
  for (const o of lfmSensorObs) {
    if (!Number.isFinite(o.r) || o.r > 80) continue;
    const px = azPx(o.az);
    const py = elPx(o.el);
    const ttc = o.ttc;
    const [r, g, b] = isMoving ? _l0RiskColor(ttc) : _l0DistColor(o.r);
    // Outer TTC halo — CELL-sized so adjacent samples tile without overlap
    ctx.beginPath(); ctx.arc(px, py, CELL * 0.46, 0, Math.PI * 2);
    ctx.fillStyle = `rgba(${r},${g},${b},0.22)`; ctx.fill();
    // Inner source dot
    ctx.beginPath(); ctx.arc(px, py, CELL * 0.28, 0, Math.PI * 2);
    ctx.fillStyle = srcColor(1 << (o.src & 0x03)); ctx.fill();
    // TTC ring stroke
    ctx.beginPath(); ctx.arc(px, py, CELL * 0.46, 0, Math.PI * 2);
    ctx.strokeStyle = `rgba(${r},${g},${b},0.75)`; ctx.lineWidth = 1.5 * dpr; ctx.stroke();
  }

  ctx.restore();

  // Labels outside clip
  ctx.save();
  const TS2 = 11 * dpr;
  ctx.font = `${TS2}px system-ui`; ctx.fillStyle = 'rgba(55,75,148,0.75)';
  ctx.textAlign = 'center'; ctx.textBaseline = 'bottom';
  ctx.fillText('Sensor cone  az × el', panX + W / 2, panY + 2 * dpr);

  ctx.textBaseline = 'top';
  for (const az of [-azHalf, 0, azHalf]) {
    ctx.fillText(`${Math.round(az)}°`, azDegPx(az), panY + H - PAD.b + 2 * dpr);
  }
  ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
  for (const [el_val, el_lbl] of [
      [-elHalf, `${Math.round(-elHalf)}°`],
      [0,        '0°'],
      [elHalf,  `${Math.round(elHalf)}°`]]) {
    ctx.fillText(el_lbl, panX + PAD.l - 3 * dpr, elDegPx(el_val));
  }

  // Source legend (compact, below panel)
  const LY = panY + H + 4 * dpr;
  ctx.font = `${TS2}px system-ui`; ctx.textBaseline = 'top';
  const srcs = [
    ['rgba(255,255,255,0.95)', 'GT'],
    ['rgba(140,140,140,0.90)', 'Depth'],
    ['rgba(20,20,20,0.95)',    'Visual'],
  ];
  let lx = panX + PAD.l;
  for (const [col, lbl] of srcs) {
    ctx.fillStyle = col;
    ctx.beginPath(); ctx.arc(lx + 5 * dpr, LY + 6 * dpr, 4 * dpr, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = 'rgba(55,75,148,0.70)';
    ctx.textAlign = 'left';
    ctx.fillText(lbl, lx + 12 * dpr, LY + 1 * dpr);
    lx += (lbl.length * 6 + 20) * dpr;
  }

  ctx.restore();
}

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
  // Mirrors the project() coordinate transform for X and Y (NED Y-flip), but
  // intentionally flips Z so the gizmo shows Z pointing UP — a conventional
  // orientation indicator regardless of the NED world frame.
  function gProj(vx, vy, vz) {
    const x = vx, y = -vy, z = vz;           // NED Y-flip; Z flipped to show Z-up in gizmo
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

// ── L2 planning map rendering ──────────────────────────────────────────────────
//
// L2 cells are rendered as exterior voxel faces — the same algorithm as L1 but:
//   • Native cell size: 1 m XY / 2 m Z (no LOD aggregation needed — L2 is already coarse)
//   • Same color mode as L1 (height ramp or type colors) so the two layers read
//     consistently; L2 uses lower alpha so it reads as a persistent base layer
//   • Face alpha ~0.35 (L1 is ~0.78) to visually distinguish persistent vs current data
//
// Why no LOD slider for L2: L1's slider aggregates raw 0.5 m cells into coarser voxels.
// L2 cells are already 1 m — they are the coarse layer.  Rendering at native size is
// the correct "LOD" for L2.

function planningQuantKey(x, y, z, cellSizeM, vCellSizeM) {
  const qx = Math.round(x / cellSizeM);
  const qy = Math.round(y / cellSizeM);
  const qz = Math.round(z / vCellSizeM);
  return `${qx},${qy},${qz}`;
}

function esdfQuantKey(x, y, z) {
  return `${Math.round(x/esdfCellSizeM)},${Math.round(y/esdfCellSizeM)},${Math.round(z/esdfVCellSizeM)}`;
}

let planCellSizeM  = 1.0;
let planVCellSizeM = 2.0;
let planFacesGeom  = [];   // pre-built exterior face geometry for L2
// Bounding box of L2 cells — updated on snapshot/delta; included in sceneRadius.
let planBounds = null;  // null | {min_x,max_x,min_y,max_y,min_z,max_z}
// Max updated_at across all L2 cells (Unix seconds); drives age-decay animation scheduling.
let planNewestTs = 0;

function applyPlanningSnapshot(plan, {deferRender=false}={}) {
  if (!plan || !Array.isArray(plan.cells)) return false;

  planCellSizeM  = fin(plan.cell_size_m, 1.0)  || 1.0;
  planVCellSizeM = fin(plan.vertical_cell_size_m, 2.0) || 2.0;

  planningCellsByKey.clear();

  let mnx=Infinity,mxx=-Infinity,mny=Infinity,mxy=-Infinity,mnz=Infinity,mxz=-Infinity;
  let maxTs = 0;
  for (const raw of plan.cells) {
    const center = asVec3(raw.center_map);
    if (!center) continue;
    const updated_at = fin(raw.t, 0);
    planningCellsByKey.set(
      planningQuantKey(center.x, center.y, center.z, planCellSizeM, planVCellSizeM),
      {
        center,
        occupied_score:    fin(raw.occupied_score, 0),
        confidence:        fin(raw.confidence, 0),
        source_cell_count: fin(raw.source_cell_count, 0),
        updated_at,  // Unix seconds of last live L1 update; 0 = DB-only
      }
    );
    if (center.x < mnx) mnx=center.x; if (center.x > mxx) mxx=center.x;
    if (center.y < mny) mny=center.y; if (center.y > mxy) mxy=center.y;
    if (center.z < mnz) mnz=center.z; if (center.z > mxz) mxz=center.z;
    if (updated_at > maxTs) maxTs = updated_at;
  }
  if (planningCellsByKey.size > 0) {
    planBounds = {min_x:mnx,max_x:mxx,min_y:mny,max_y:mxy,min_z:mnz,max_z:mxz};
    planNewestTs = maxTs;
  }

  buildPlanningFaces();
  if (!deferRender) { updateMetrics(); scheduleDraw(); }
  return true;
}

function buildPlanningFaces() {
  planFacesGeom = [];
  if (!state.showPlanning || planningCellsByKey.size === 0) return;

  const csM  = planCellSizeM;
  const vcM  = planVCellSizeM;
  const halfX = csM * 0.5;
  const halfY = csM * 0.5;
  const halfZ = vcM * 0.5;

  // Build neighbour-lookup set using the same quantised keys.
  const occKeys = new Set(planningCellsByKey.keys());

  const DIRS = [
    [1,0,0, 0.60], [-1,0,0, 0.50],
    [0,1,0, 0.44], [0,-1,0, 0.38],
    [0,0,1, 0.20], [0,0,-1, 1.00],
  ];

  for (const cell of planningCellsByKey.values()) {
    const {center, occupied_score, confidence, updated_at} = cell;
    if (!center) continue;

    const cx = center.x, cy = center.y, cz = center.z;

    for (const [dx, dy, dz, shading] of DIRS) {
      // Neighbour key: offset by one cell in this direction.
      const nk = planningQuantKey(
        cx + dx * csM, cy + dy * csM, cz + dz * vcM,
        csM, vcM);
      if (occKeys.has(nk)) continue;   // interior face — skip

      let corners;
      if (dx !== 0) {
        const fx = cx + dx * halfX;
        corners = [
          {x:fx, y:cy-halfY, z:cz-halfZ}, {x:fx, y:cy+halfY, z:cz-halfZ},
          {x:fx, y:cy+halfY, z:cz+halfZ}, {x:fx, y:cy-halfY, z:cz+halfZ},
        ];
      } else if (dy !== 0) {
        const fy = cy + dy * halfY;
        corners = [
          {x:cx-halfX, y:fy, z:cz-halfZ}, {x:cx+halfX, y:fy, z:cz-halfZ},
          {x:cx+halfX, y:fy, z:cz+halfZ}, {x:cx-halfX, y:fy, z:cz+halfZ},
        ];
      } else {
        const fz = cz + dz * halfZ;
        corners = [
          {x:cx-halfX, y:cy-halfY, z:fz}, {x:cx+halfX, y:cy-halfY, z:fz},
          {x:cx+halfX, y:cy+halfY, z:fz}, {x:cx-halfX, y:cy+halfY, z:fz},
        ];
      }
      planFacesGeom.push({
        corners,
        centroid: {
          x: (corners[0].x+corners[2].x)*0.5,
          y: (corners[0].y+corners[2].y)*0.5,
          z: (corners[0].z+corners[2].z)*0.5,
        },
        shading,
        occupied_score,
        confidence,
        updated_at,  // Unix seconds of last live update; 0 = DB-only / unknown age
        cx, cy, cz,
      });
    }
  }
}

function drawPlanningFaces() {
  if (!state.showPlanning || planFacesGeom.length === 0) return;

  const sorted = planFacesGeom.slice().sort((a,b) => b.centroid.z - a.centroid.z);

  ctx.save();
  ctx.lineWidth = 0.5 * devicePixelRatio;

  const byType = state.travColorByType;
  // L2 age-decay: 0–2 s full brightness, 2–10 s linear cool to 80%, stable after.
  // updated_at=0 (unknown) renders at the dimmed final value.
  const L2_ALPHA  = 0.44;
  const L2_STROKE = 0.52;

  const groups = new Map();

  for (const face of sorted) {
    const ps = face.corners.map(c => project(c));
    if (ps.some(p => !Number.isFinite(p.x) || !Number.isFinite(p.y))) continue;

    const liveBase = byType ? [60, 100, 200] : rgbForH(Math.max(0, -face.centroid.z));
    const s = face.shading;
    const fr = Math.round(liveBase[0] * s);
    const fg = Math.round(liveBase[1] * s);
    const fb = Math.round(liveBase[2] * s);
    // ageFactor: 1.0 (fresh) → 0.8 (≥10 s old). liveDecay takes ms timestamp.
    const ageFactor = (liveDecay(face.updated_at * 1000, 0.8) ?? 0.8);
    const af10 = Math.round(ageFactor * 10);   // 8, 9, or 10 — batch key bucket
    const fa = L2_ALPHA  * ageFactor;
    const sa = L2_STROKE * ageFactor;
    const key = `${fr},${fg},${fb},${af10}`;

    let g = groups.get(key);
    if (!g) {
      g = {
        fillStyle:   `rgba(${fr},${fg},${fb},${fa})`,
        strokeStyle: `rgba(${fr},${fg},${fb},${sa})`,
        path: new Path2D(),
      };
      groups.set(key, g);
    }
    g.path.moveTo(ps[0].x, ps[0].y);
    for (let i=1; i<ps.length; i++) g.path.lineTo(ps[i].x, ps[i].y);
    g.path.closePath();
  }

  for (const {fillStyle, strokeStyle, path} of groups.values()) {
    ctx.fillStyle   = fillStyle;
    ctx.strokeStyle = strokeStyle;
    ctx.fill(path);
    ctx.stroke(path);
  }

  ctx.restore();
}

// ── L3 ESDF cell rendering ────────────────────────────────────────────────────
//
// ESDF shell cells rendered as exterior voxel faces, like L2 but colored by
// clearance distance:  red (d<1m) · amber (d<3m) · green (d≥3m).
// Negative-d cells (inside occupied) are skipped.
// Faces are pre-built by buildESDFFaces() whenever cell data changes or the
// toggle is flipped, then replayed in drawESDFFaces() each frame.

function buildESDFFaces() {
  esdfFacesGeom = [];
  if (el("m-esdf-faces")) el("m-esdf-faces").textContent = "0";
  if (!state.showEsdf || esdfCellsByKey.size === 0) return;

  const csM  = esdfCellSizeM;
  const vcM  = esdfVCellSizeM;
  const halfX = csM * 0.5;
  const halfY = csM * 0.5;
  const halfZ = vcM * 0.5;

  const occKeys = new Set();
  for (const c of esdfCellsByKey.values()) {
    occKeys.add(esdfQuantKey(c.x, c.y, c.z));
  }

  const DIRS = [
    [1,0,0, 0.60], [-1,0,0, 0.50],
    [0,1,0, 0.44], [0,-1,0, 0.38],
    [0,0,1, 0.20], [0,0,-1, 1.00],
  ];

  for (const cell of esdfCellsByKey.values()) {
    if (cell.d < 0) continue;  // inside obstacle — no face
    const cx = cell.x, cy = cell.y, cz = cell.z;
    for (const [dx, dy, dz, shading] of DIRS) {
      const nk = esdfQuantKey(cx+dx*csM, cy+dy*csM, cz+dz*vcM);
      if (occKeys.has(nk)) continue;  // interior face — skip

      let corners;
      if (dx !== 0) {
        const fx = cx + dx*halfX;
        corners = [
          {x:fx,y:cy-halfY,z:cz-halfZ},{x:fx,y:cy+halfY,z:cz-halfZ},
          {x:fx,y:cy+halfY,z:cz+halfZ},{x:fx,y:cy-halfY,z:cz+halfZ},
        ];
      } else if (dy !== 0) {
        const fy = cy + dy*halfY;
        corners = [
          {x:cx-halfX,y:fy,z:cz-halfZ},{x:cx+halfX,y:fy,z:cz-halfZ},
          {x:cx+halfX,y:fy,z:cz+halfZ},{x:cx-halfX,y:fy,z:cz+halfZ},
        ];
      } else {
        const fz = cz + dz*halfZ;
        corners = [
          {x:cx-halfX,y:cy-halfY,z:fz},{x:cx+halfX,y:cy-halfY,z:fz},
          {x:cx+halfX,y:cy+halfY,z:fz},{x:cx-halfX,y:cy+halfY,z:fz},
        ];
      }
      esdfFacesGeom.push({
        corners,
        centroid: {
          x:(corners[0].x+corners[2].x)*0.5,
          y:(corners[0].y+corners[2].y)*0.5,
          z:(corners[0].z+corners[2].z)*0.5,
        },
        shading,
        d: cell.d,
      });
    }
  }
}

function drawESDFFaces() {
  if (!state.showEsdf || esdfFacesGeom.length === 0) return;

  const sorted = esdfFacesGeom.slice().sort((a,b) => b.centroid.z - a.centroid.z);

  ctx.save();
  ctx.lineWidth = 0.5 * devicePixelRatio;

  const groups = new Map();
  for (const face of sorted) {
    const ps = face.corners.map(c => project(c));
    if (ps.some(p => !Number.isFinite(p.x) || !Number.isFinite(p.y))) continue;

    const d = face.d, s = face.shading;
    let r, g, b, fa;
    if      (d < 1) { r=200; g=40;  b=30;  fa=0.45; }
    else if (d < 3) { r=200; g=150; b=30;  fa=0.38; }
    else            { r=40;  g=180; b=60;  fa=0.30; }
    const fr=Math.round(r*s), fg=Math.round(g*s), fb=Math.round(b*s);
    const key=`${fr},${fg},${fb}`;
    let grp=groups.get(key);
    if (!grp) {
      grp={
        fillStyle:   `rgba(${fr},${fg},${fb},${fa})`,
        strokeStyle: `rgba(${fr},${fg},${fb},${Math.min(1,fa+0.10)})`,
        path: new Path2D(),
      };
      groups.set(key,grp);
    }
    grp.path.moveTo(ps[0].x,ps[0].y);
    for (let i=1;i<ps.length;i++) grp.path.lineTo(ps[i].x,ps[i].y);
    grp.path.closePath();
  }

  for (const {fillStyle, strokeStyle, path} of groups.values()) {
    ctx.fillStyle=fillStyle; ctx.strokeStyle=strokeStyle;
    ctx.fill(path); ctx.stroke(path);
  }
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
  // Sort by world Z descending: large NED-Z (near ground) drawn first,
  // small/negative NED-Z (high altitude) drawn last → correct occlusion
  // when viewed from above (higher cubes appear on top of ground-level ones).
  cells.sort((a,b)=>b.center.z - a.center.z);
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

  const levelM = travDisplayLevelM;
  const halfL  = levelM * 0.5;

  // ── Step 1: aggregate raw 0.5 m cells into coarse LOD voxels ─────────────────
  // Each coarse voxel key = snapped centre at levelM resolution.
  // State mapping per coarse voxel:
  //   occ > 0 && free > 0  → "partial"  (amber)
  //   occ > 0 && free == 0 → "occupied" (red)
  //   occ == 0             → skip (no occupied evidence in this voxel)
  //
  // "Partial" definition: the depth-based detector rarely produces explicit
  // free evidence, so we cannot use free-cell count to detect mixed voxels.
  // Instead, a coarse voxel is "partial" when it contains both solidly-occupied
  // 0.5 m cells (state="occupied"|"mixed") AND sub-threshold cells that are
  // still in the map (state="unknown"|"stale").  These sub-threshold cells
  // represent sensor hits that have not yet accumulated enough confidence —
  // they mark the fuzzy boundary of the obstacle cloud.

  const coarseGrid = new Map(); // coarseKey → {cx,cy,cz, occ, edge, conf}

  for (const cell of travCellsByKey.values()) {
    if (!cell.center) continue;
    const s = cell.state;
    const isOcc  = s === "occupied" || s === "mixed";
    const isEdge = s === "unknown"  || s === "stale";
    if (!isOcc && !isEdge) continue; // explicit free → not rendered

    const cx = Math.round(cell.center.x / levelM) * levelM;
    const cy = Math.round(cell.center.y / levelM) * levelM;
    const cz = Math.round(cell.center.z / levelM) * levelM;
    const k  = `${cx},${cy},${cz}`;
    let e = coarseGrid.get(k);
    if (!e) { e = {cx, cy, cz, occ:0, edge:0, conf:0}; coarseGrid.set(k, e); }
    if (isOcc)  { e.occ++;  e.conf += cell.confidence; }
    if (isEdge) { e.edge++; }
  }

  // ── Step 2: build neighbour-lookup set (any voxel with obstacles) ─────────────
  const coarseOccKeys = new Set();
  for (const [k, e] of coarseGrid) {
    if (e.occ > 0) coarseOccKeys.add(k);
  }

  // Decimate if too many occupied voxels (prefer closest to ego)
  let occEntries = [...coarseGrid.entries()].filter(([,e]) => e.occ > 0);
  if (occEntries.length > MAX_TRAV_DISPLAY) {
    if (state.ego) {
      const {x:ex, y:ey, z:ez} = state.ego;
      occEntries.sort(([, a], [, b]) =>
        Math.hypot(a.cx-ex, a.cy-ey, a.cz-ez) - Math.hypot(b.cx-ex, b.cy-ey, b.cz-ez));
    }
    occEntries = occEntries.slice(0, MAX_TRAV_DISPLAY);
  }

  // ── Step 3: exterior-face extraction with per-face shading ───────────────────
  // Stronger directional shading so adjacent faces of the same color look
  // clearly distinct, making 3-D stacking visible.
  //   top  (dz == -1, upward in NED)  → 1.00  (full brightness)
  //   ±X front/back faces             → 0.60 / 0.50
  //   ±Y left/right faces             → 0.44 / 0.38
  //   bottom (dz == +1)               → 0.20  (nearly black)
  // Ratio top:bottom ≈ 5×  (was 2.6× before)
  const DIRS = [
    [1,0,0, 0.60], [-1,0,0, 0.50],
    [0,1,0, 0.44], [0,-1,0, 0.38],
    [0,0,1, 0.20], [0,0,-1, 1.00],
  ];

  for (const [, e] of occEntries) {
    const {cx, cy, cz, occ, edge, conf} = e;
    const isPartial = edge > 0; // coarse voxel has sub-threshold cells mixed with occupied → boundary
    const avgConf   = occ > 0 ? clamp(conf / occ, 0, 1) : 0.5;

    for (const [dx, dy, dz, shading] of DIRS) {
      const nk = `${cx+dx*levelM},${cy+dy*levelM},${cz+dz*levelM}`;
      if (coarseOccKeys.has(nk)) continue; // interior face — skip

      let corners;
      if (dx !== 0) {
        const fx = cx + dx * halfL;
        corners = [
          {x:fx,y:cy-halfL,z:cz-halfL},{x:fx,y:cy+halfL,z:cz-halfL},
          {x:fx,y:cy+halfL,z:cz+halfL},{x:fx,y:cy-halfL,z:cz+halfL},
        ];
      } else if (dy !== 0) {
        const fy = cy + dy * halfL;
        corners = [
          {x:cx-halfL,y:fy,z:cz-halfL},{x:cx+halfL,y:fy,z:cz-halfL},
          {x:cx+halfL,y:fy,z:cz+halfL},{x:cx-halfL,y:fy,z:cz+halfL},
        ];
      } else {
        const fz = cz + dz * halfL;
        corners = [
          {x:cx-halfL,y:cy-halfL,z:fz},{x:cx+halfL,y:cy-halfL,z:fz},
          {x:cx+halfL,y:cy+halfL,z:fz},{x:cx-halfL,y:cy+halfL,z:fz},
        ];
      }

      travFacesGeom.push({
        corners,
        centroid: {x:(corners[0].x+corners[2].x)*0.5,
                   y:(corners[0].y+corners[2].y)*0.5,
                   z:(corners[0].z+corners[2].z)*0.5},
        isPartial,
        confidence: avgConf,
        shading,
        // Voxel identity — used by hover card.
        vx: cx, vy: cy, vz: cz,
        occ, edge, levelM,
      });
    }
  }
}

function drawTravFaces() {
  if (!state.showTrav || travFacesGeom.length === 0) return;

  // Sort by world Z descending: ground-level faces (large NED-Z) drawn first,
  // high-altitude faces (small/negative NED-Z) drawn last → correct stacking
  // when viewed from above. Matches the obstacle-cell sort order.
  const sorted = travFacesGeom.slice().sort(
    (a,b) => b.centroid.z - a.centroid.z
  );

  ctx.save();
  ctx.lineWidth = 0.5 * devicePixelRatio;

  // Batch faces by shaded color key: 2 canvas calls per unique color.
  // Color mode: height-based (default, matches obstacle map ramp) or type-based.
  //   Type: occupied base [190,55,45], partial base [200,150,30].
  //   Height: rgbForH(altitude) — same ramp as the obstacle cell layer.
  // Shading (directional lighting) is applied in both modes.
  const byType = state.travColorByType;
  const groups = new Map(); // colorKey → { fillStyle, strokeStyle, path }

  for (const face of sorted) {
    const ps = face.corners.map(c => project(c));
    if (ps.some(p => !Number.isFinite(p.x) || !Number.isFinite(p.y))) continue;

    const base = byType
      ? (face.isPartial ? [200, 150, 30] : [190, 55, 45])
      : rgbForH(Math.max(0, -face.centroid.z));
    const s  = face.shading;
    const fr = Math.round(base[0] * s);
    const fg = Math.round(base[1] * s);
    const fb = Math.round(base[2] * s);
    const fa = byType
      ? (face.isPartial ? 0.60 : 0.82)
      : (face.isPartial ? 0.52 : 0.78);
    const sa  = Math.min(1, fa + 0.12);
    const key = `${fr},${fg},${fb},${face.isPartial?1:0}`;

    let g = groups.get(key);
    if (!g) {
      g = {
        fillStyle:   `rgba(${fr},${fg},${fb},${fa})`,
        strokeStyle: `rgba(${fr},${fg},${fb},${sa})`,
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

function drawTakeoffMarker() {
  const p = state.takeoffPos;
  if (!p) return;
  const c = project(p);
  if (!c) return;
  const dpr = devicePixelRatio;
  const R = 9 * dpr;
  ctx.save();
  // Green diamond outline
  ctx.strokeStyle = 'rgba(32,224,80,1.0)';
  ctx.fillStyle   = 'rgba(5,12,8,0.85)';
  ctx.lineWidth   = 2 * dpr;
  ctx.beginPath();
  ctx.moveTo(c.x,      c.y - R);
  ctx.lineTo(c.x + R,  c.y);
  ctx.lineTo(c.x,      c.y + R);
  ctx.lineTo(c.x - R,  c.y);
  ctx.closePath();
  ctx.fill(); ctx.stroke();
  // "H" label
  ctx.fillStyle   = 'rgba(32,224,80,1.0)';
  ctx.font        = `bold ${Math.round(9 * dpr)}px system-ui`;
  ctx.textAlign   = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText('H', c.x, c.y);
  ctx.restore();
}

function drawPerchCandidates() {
  const candidates = state.perchCandidates;
  if (!candidates || candidates.length === 0) return;
  const dpr = devicePixelRatio;
  ctx.save();
  for (let i = 0; i < candidates.length; ++i) {
    const cand = candidates[i];
    const pos = cand.position_local || cand.position;
    if (!pos) continue;
    const c = project(pos);
    if (!c) continue;
    // Score in [0,1]: best candidates are brighter green, worse are more muted.
    const score = Math.max(0, Math.min(1, cand.score || 0));
    const R = Math.round((6 + score * 6) * dpr);
    const g = Math.round(160 + score * 95);
    ctx.strokeStyle = `rgba(0,${g},48,0.95)`;
    ctx.fillStyle   = `rgba(0,${g},48,0.22)`;
    ctx.lineWidth   = 2 * dpr;
    // Circle with crosshair
    ctx.beginPath();
    ctx.arc(c.x, c.y, R, 0, 2 * Math.PI);
    ctx.fill();
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(c.x - R, c.y); ctx.lineTo(c.x + R, c.y);
    ctx.moveTo(c.x, c.y - R); ctx.lineTo(c.x, c.y + R);
    ctx.stroke();
    // Rank label for top-3
    if (i < 3) {
      ctx.fillStyle = `rgba(0,${g},48,0.95)`;
      ctx.font = `bold ${Math.round(8 * dpr)}px system-ui`;
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(String(i + 1), c.x, c.y);
    }
  }
  ctx.restore();
}

function drawTrajectory() {
  if (!state.showTrajectory||state.trajectory.length<2) return;
  ctx.save();

  // All trajectory points carry live_seen_ms; render them with a full
  // index-based color gradient: oldest = dark red-orange, newest = bright yellow.
  // t=0 (oldest) → rgba(255,0,0,0.18)  t=1 (newest) → rgba(255,255,0,0.95)
  const liveTraj=state.trajectory.filter(p=>p&&p.live_seen_ms);
  const n=liveTraj.length;
  for (let i=1; i<n; ++i) {
    const pa=liveTraj[i-1], pb=liveTraj[i];

    // Skip discontinuous jumps (latency spikes, teleports).
    const dx=pb.x-pa.x, dy=pb.y-pa.y, dz=pb.z-pa.z;
    if (Math.hypot(dx,dy,dz) > TRAJ_MAX_GAP_M) continue;
    const dtMs=Number(pb.live_seen_ms)-Number(pa.live_seen_ms);
    if (dtMs > TRAJ_MAX_GAP_MS) continue;

    const a=project(pa), b=project(pb);
    if (!a||!b||!Number.isFinite(a.x)||!Number.isFinite(b.x)) continue;

    // t: 0=oldest segment end, 1=newest — drives color, opacity, and width.
    const t=i/n;
    const g=Math.round(255*t);              // red (old) → yellow (new)
    const alpha=(0.18+0.77*t).toFixed(2);  // faint old trail → opaque new
    const lw=(0.8+2.2*t)*devicePixelRatio; // thin old → thick new
    ctx.beginPath(); ctx.moveTo(a.x,a.y); ctx.lineTo(b.x,b.y);
    ctx.strokeStyle=`rgba(255,${g},0,${alpha})`;
    ctx.lineWidth=lw;
    ctx.stroke();
  }
  if (state.ego) drawPoint(state.ego, "rgba(255,255,64,1.0)", 6.5);
  ctx.restore();
}

// ── scene bounds recompute ─────────────────────────────────────────────────────

function recomputeBounds() {
  const pts=[];
  for (const cell of obsCellsByKey.values()) { if (cell.center) pts.push(cell.center); }
  // Include raw L1 trav cell centers (0.5 m resolution, LOD-independent) so the scene
  // auto-sizes to the traversal area.  Face corners are NOT used — they depend on LOD.
  for (const cell of travCellsByKey.values()) { if (cell.center) pts.push(cell.center); }
  // Include L2 bounding box corners so DB-only mode sizes the scene correctly.
  if (planBounds) {
    pts.push({x:planBounds.min_x,y:planBounds.min_y,z:planBounds.min_z});
    pts.push({x:planBounds.max_x,y:planBounds.max_y,z:planBounds.max_z});
  }
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
  // Trimmed-extent: ignore the outer 10% of points on each axis so a single
  // far-away outlier (one maximal cell) cannot pull the view center off-scene.
  function trimmedRange(arr) {
    arr.sort((a,b)=>a-b);
    const cut = Math.max(0, Math.floor(arr.length * 0.10));
    const sl = arr.length - 2*cut > 0 ? arr.slice(cut, arr.length - cut) : arr;
    return {lo: sl[0], hi: sl[sl.length-1]};
  }
  const rx = trimmedRange(pts.map(p=>p.x));
  const ry = trimmedRange(pts.map(p=>p.y));
  const rz = trimmedRange(pts.map(p=>p.z));
  state.bounds={
    min_x:rx.lo, max_x:rx.hi,
    min_y:ry.lo, max_y:ry.hi,
    min_z:rz.lo, max_z:rz.hi,
  };
}

// ── main draw ──────────────────────────────────────────────────────────────────

let drawScheduled=false;
function scheduleDraw() {
  if (drawScheduled) return;
  drawScheduled=true;
  requestAnimationFrame(()=>{ drawScheduled=false; draw(); });
}

// ── L3 ESDF vector field ──────────────────────────────────────────────────────
//
// buildESDFArrows(): continuous Gaussian-averaged APF vector field.
//
// Algorithm:
//   1. Bucket all shell ESDF cells into an R×R XY spatial grid.
//   2. Place output sample points on an R-spaced regular grid, aligned to the
//      cell data extent. One arrow per R×R×Z-slice grid cell.
//   3. At each sample point, gather all ESDF cells within radius R from the
//      3×3 neighboring XY buckets (same Z slice). Accumulate APF-weighted
//      Gaussian sum: w = apf(d) · exp(−dist²/2σ²), σ = R/2.
//   4. Emit a normalised arrow if any weight accumulated.
//
// No similarity threshold. Sliding R changes both sample density and averaging
// radius continuously — sparse at large R, dense at small R.
// Velocity mode sets R = braking distance (v²/2a).

function buildESDFArrows() {
  esdfArrowGeom = [];
  if (el("m-esdf-arrows")) el("m-esdf-arrows").textContent = "0";
  if (!state.showEsdf || esdfCellsByKey.size === 0) return;

  const useVel = el("esdf-vel-mode")?.checked ?? false;
  const R = useVel
    ? Math.max(0.5, Math.min(esdfD0M, (lfmEgoSpeedMps * lfmEgoSpeedMps) / (2 * ESDF_A_MAX)))
    : esdfSmoothR;
  if (el("m-esdf-r")) el("m-esdf-r").textContent = R.toFixed(1) + "m" + (useVel ? " 〜v" : "");

  const d0     = esdfD0M;
  // σ = R/2.35 satisfies the partition-of-unity condition: adjacent Gaussians
  // sum to 1.0 at every midpoint between sample points (exp(−R²/8σ²) = 0.5).
  // Gather radius = 3σ ≈ 1.28R, rounded to 1.3R for the bucket lookup.
  // We use ±2 XY bucket offsets so cells up to 2R away are considered;
  // the Gaussian weight at 1.3R is exp(−9/2) ≈ 0.011, negligible beyond.
  const sigma  = R / 2.35;
  const rho    = 1.3 * R;          // 3σ gather radius
  const R2     = rho * rho;
  const inv2s2 = 1.0 / (2.0 * sigma * sigma);
  const vcM    = esdfVCellSizeM;

  // ── Step 1: bucket shell cells by R-grid XY square and Z cell ───────────────
  // Key: "bx,by,iz"  (bx=floor(x/R), by=floor(y/R), iz=round(z/vcM))
  const buckets = new Map();
  let xMin = Infinity, xMax = -Infinity;
  let yMin = Infinity, yMax = -Infinity;
  let zMin = Infinity, zMax = -Infinity;

  for (const [, cell] of esdfCellsByKey) {
    if (cell.d <= 0 || cell.d >= d0) continue;
    const bx = Math.floor(cell.x / R);
    const by = Math.floor(cell.y / R);
    const iz = Math.round(cell.z / vcM);
    const key = `${bx},${by},${iz}`;
    let b = buckets.get(key);
    if (!b) { b = []; buckets.set(key, b); }
    b.push(cell);
    if (cell.x < xMin) xMin = cell.x; if (cell.x > xMax) xMax = cell.x;
    if (cell.y < yMin) yMin = cell.y; if (cell.y > yMax) yMax = cell.y;
    if (cell.z < zMin) zMin = cell.z; if (cell.z > zMax) zMax = cell.z;
  }

  if (zMin > zMax) return;

  // ── Step 2: R-spaced Z layers with alternating XY offset (lattice sampling) ─
  //
  // Even layer (layerIdx % 2 == 0): XY grid origin at (gxBase, gyBase)
  // Odd  layer (layerIdx % 2 == 1): XY grid origin at (gxBase+R/2, gyBase+R/2)
  //
  // For each Z layer, cells within ±ceil(R/(2*vcM)) Z-cell steps are gathered.
  // This is a non-overlapping partition: each ESDF cell belongs to exactly one Z layer
  // (the nearest one). Odd layers produce arrows only when ESDF data reaches within R/2
  // of their center — no phantom arrows floating above the actual obstacle geometry.
  const gxBase = Math.floor(xMin / R) * R;
  const gyBase = Math.floor(yMin / R) * R;
  const gzBase = Math.floor(zMin / R) * R;
  const izR    = Math.ceil(R / (2 * vcM));   // Z partition radius: half the layer spacing

  let layerIdx = 0;
  for (let gz = gzBase; gz <= zMax + R; gz += R, layerIdx++) {
    const isOdd  = (layerIdx & 1) === 1;
    const xOff   = isOdd ? R * 0.5 : 0;
    const yOff   = isOdd ? R * 0.5 : 0;
    const gxStart = gxBase + xOff;
    const gyStart = gyBase + yOff;
    const izCenter = Math.round(gz / vcM);

    for (let gx = gxStart; gx <= xMax + R; gx += R) {
      const gbx = Math.floor(gx / R);

      for (let gy = gyStart; gy <= yMax + R; gy += R) {
        const gby = Math.floor(gy / R);

        let ax = 0, ay = 0, az = 0, wSum = 0, dMin = d0, maxTs = 0;

        for (let dx = -2; dx <= 2; dx++) {
          for (let dy = -2; dy <= 2; dy++) {
            for (let diz = -izR; diz <= izR; diz++) {
              const b = buckets.get(`${gbx+dx},${gby+dy},${izCenter+diz}`);
              if (!b) continue;
              for (const cell of b) {
                const ex = cell.x - gx, ey = cell.y - gy;
                const dist2 = ex*ex + ey*ey;
                if (dist2 > R2) continue;
                // APF weight: (1/d − 1/d0) / d, then Gaussian by XY distance
                const apf = (1.0 / cell.d - 1.0 / d0) / cell.d;
                const w   = apf * Math.exp(-dist2 * inv2s2);
                const sgx = cell.sgx ?? cell.gx ?? 0;
                const sgy = cell.sgy ?? cell.gy ?? 0;
                const sgz = cell.sgz ?? cell.gz ?? 0;
                ax += w * sgx; ay += w * sgy; az += w * sgz;
                wSum += w;
                if (cell.d < dMin) dMin = cell.d;
                const ts = cell.updated_at || 0;
                if (ts > maxTs) maxTs = ts;
              }
            }
          }
        }

        if (wSum < 1e-10) continue;
        const len = Math.hypot(ax, ay, az);
        if (len < 1e-10) continue;

        esdfArrowGeom.push({
          x: gx, y: gy, z: gz, d: dMin,
          sgx: ax/len, sgy: ay/len, sgz: az/len,
          updated_at: maxTs,
        });
      }
    }
  }

  if (el("m-esdf-arrows")) el("m-esdf-arrows").textContent = String(esdfArrowGeom.length);
}

// Render prebuilt sparse smoothed arrows.
// Arrow opacity: 0–2 s full (0.80), 2–10 s linear cool to 80% (→0.64), stable after.
function drawESDFArrows() {
  if (!state.showEsdf || esdfArrowGeom.length === 0) return;
  for (const a of esdfArrowGeom) {
    const d = a.d;
    const [r,g,b] = d < 1 ? [228,42,30] : d < 3 ? [228,200,30] : [42,200,65];
    const scale = (esdfD0M - Math.abs(d)) / esdfD0M * 4.0;
    if (scale <= 0) continue;

    // Same 2/10 s age profile as L2 cells; updated_at is Unix seconds.
    const alpha = 0.80 * (liveDecay(a.updated_at * 1000, 0.8) ?? 0.8);

    const tip = {x: a.x + a.sgx*scale, y: a.y + a.sgy*scale, z: a.z + a.sgz*scale};
    const pa = project({x:a.x, y:a.y, z:a.z});
    const pb = project(tip);
    ctx.beginPath(); ctx.moveTo(pa.x, pa.y); ctx.lineTo(pb.x, pb.y);
    ctx.strokeStyle = `rgba(${r},${g},${b},${alpha.toFixed(2)})`;
    ctx.lineWidth = 1.5 * devicePixelRatio;
    ctx.stroke();
    const ddx = pb.x-pa.x, ddy = pb.y-pa.y, len = Math.hypot(ddx, ddy);
    if (len > 3) {
      const ux=ddx/len, uy=ddy/len, hw=3*devicePixelRatio, hl=6*devicePixelRatio;
      ctx.beginPath(); ctx.moveTo(pb.x, pb.y);
      ctx.lineTo(pb.x - ux*hl + uy*hw, pb.y - uy*hl - ux*hw);
      ctx.lineTo(pb.x - ux*hl - uy*hw, pb.y - uy*hl + ux*hw);
      ctx.closePath();
      ctx.fillStyle = `rgba(${r},${g},${b},${alpha.toFixed(2)})`;
      ctx.fill();
    }
  }
}

function draw() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  drawPlanningFaces();   // L2 persistent planning map (exterior faces, back-most layer)
  if (state.showEsdfCells) drawESDFFaces();  // L3 cell faces (off by default)
  drawTravFaces();       // L1 traversability exterior surface
  drawObstacleCells();   // raw evidence cells (demoted, off by default)
  drawLiveAgingOverlay(); // live delta events with age decay
  if (state.showTrajectory) drawTrajectory();
  drawGhostDetections();
  drawSensingOverlays();
  drawESDFArrows();       // L3 ESDF gradient arrows (above surfaces, below UI overlays)
  drawTakeoffMarker();
  drawPerchCandidates();
  drawDroneMarker();
  if (state.firstBlocked) drawPoint(state.firstBlocked, "rgba(255,80,255,1.0)", 8);
  drawOrientationGizmo(); // fixed bottom-right
  drawL0PolarInset();     // fixed top-right: TTC radar
  drawL0ConeScope();      // below radar: sensor az×el scope

  // Schedule next frame if any aging is still in the 0–10 s decay window.
  const now=Date.now();
  const hasRecentObs = live.connected &&
    liveObsEvents.some(e=>e.live_seen_ms&&now-Number(e.live_seen_ms)<LIVE_FADE_END_MS);
  const hasDecayingL2 = planNewestTs > 0 && now - planNewestTs*1000 < LIVE_FADE_END_MS;
  if (hasRecentObs || hasDecayingL2) requestAnimationFrame(draw);
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

// ── UI state persistence (localStorage) ───────────────────────────────────────
// Key: "dedalus_viewer_ui".  Persists layer toggles, color mode, slider values,
// and panel collapse state across page reloads.  All reads/writes are wrapped in
// try/catch so a corrupt or missing entry never breaks the viewer.

const UI_STORAGE_KEY = "dedalus_viewer_ui";

function saveUIState() {
  try {
    localStorage.setItem(UI_STORAGE_KEY, JSON.stringify({
      showL0:          state.showL0,
      showPlanning:    state.showPlanning,
      showTrav:        state.showTrav,
      showObstacles:   state.showObstacles,
      showGhosts:      state.showGhosts,
      showSensing:     state.showSensing,
      showTrajectory:  state.showTrajectory,
      showEsdf:        state.showEsdf,
      showEsdfCells:   state.showEsdfCells,
      travColorByType: state.travColorByType,
      travLodIdx:      Number(el("trav-lod")?.value ?? 2),
      esdfSmoothR:     esdfSmoothR,
      esdfVelMode:     el("esdf-vel-mode")?.checked ?? false,
      pipelineOpen:    el("pipeline-body")?.style.display !== "none",
    }));
  } catch(_) {}
}

function loadUIState() {
  let ui;
  try { ui = JSON.parse(localStorage.getItem(UI_STORAGE_KEY) || "null"); }
  catch(_) { ui = null; }
  if (!ui) return;

  // Layer toggle flags + button active class
  for (const [id, key] of [
    ["toggle-l0",         "showL0"],
    ["toggle-planning",   "showPlanning"],
    ["toggle-trav",       "showTrav"],
    ["toggle-obstacles",  "showObstacles"],
    ["toggle-ghosts",     "showGhosts"],
    ["toggle-sensing",    "showSensing"],
    ["toggle-trajectory", "showTrajectory"],
    ["toggle-esdf",       "showEsdf"],
  ]) {
    if (key in ui) { state[key] = !!ui[key]; el(id)?.classList.toggle("active", state[key]); }
  }

  // ESDF cells checkbox (state + DOM)
  if ("showEsdfCells" in ui) {
    state.showEsdfCells = !!ui.showEsdfCells;
    const cb = el("esdf-show-cells"); if (cb) cb.checked = state.showEsdfCells;
  }

  // Trav color mode
  if ("travColorByType" in ui) {
    state.travColorByType = !!ui.travColorByType;
    const tag = el("trav-color-tag");
    if (tag) tag.textContent = state.travColorByType ? "Type" : "Height";
    const lh = el("trav-legend-height"), lt = el("trav-legend-type");
    if (lh) lh.style.visibility = state.travColorByType ? "hidden" : "visible";
    if (lt) lt.style.visibility = state.travColorByType ? "visible" : "hidden";
  }

  // Trav LOD slider
  if ("travLodIdx" in ui) {
    const s = el("trav-lod");
    if (s) {
      s.value = ui.travLodIdx;
      travDisplayLevelM = TRAV_LOD_LEVELS[ui.travLodIdx] ?? travDisplayLevelM;
      const v = el("trav-lod-val"); if (v) v.textContent = travDisplayLevelM + "m";
    }
  }

  // ESDF smooth-R slider
  if ("esdfSmoothR" in ui) {
    esdfSmoothR = ui.esdfSmoothR;
    const s = el("esdf-r-slider"); if (s) s.value = esdfSmoothR;
    const m = el("m-esdf-r");      if (m) m.textContent = esdfSmoothR.toFixed(1) + "m";
  }

  // ESDF vel-mode checkbox
  if ("esdfVelMode" in ui) { const cb = el("esdf-vel-mode"); if (cb) cb.checked = !!ui.esdfVelMode; }

  // Pipeline metrics panel
  if ("pipelineOpen" in ui) {
    const body = el("pipeline-body"), caret = el("pipeline-caret");
    if (body && caret) {
      body.style.display = ui.pipelineOpen ? "" : "none";
      caret.textContent  = ui.pipelineOpen ? "▼" : "▶";
    }
  }
}

function togglePipelineMetrics() {
  const body = el("pipeline-body");
  const caret = el("pipeline-caret");
  const open = body.style.display !== "none";
  body.style.display = open ? "none" : "";
  caret.textContent = open ? "▶" : "▼";
  saveUIState();
}
el("pipeline-body").style.display = "none";  // collapsed by default

function updateMetrics() {
  updateStatusBar();
  const b=state.bounds;
  el("m-source").textContent       = live.source;
  const pl = state.diagnostics.pipeline || {};
  el("m-pipeline-ego").textContent = pl.ego || "—";
  el("m-pl-ego").textContent       = pl.ego         || "—";
  el("m-pl-depth").textContent     = pl.depth        || "—";
  el("m-pl-detector").textContent  = pl.detector     || "—";
  el("m-pl-stabilizer").textContent = pl.camera_stabilizer || "—";
  el("m-pl-tracker").textContent   = pl.tracker      || "—";
  el("m-pl-identity").textContent  = pl.identity_resolver || "—";
  el("m-pl-projector").textContent = pl.projector    || "—";
  el("m-seq").textContent          = String(live.seq);
  el("m-l0-cells").textContent   = String(localFlightMapCells.length);
  el("m-plan-cells").textContent  = String(planningCellsByKey.size);
  el("m-esdf-cells").textContent  = String(esdfCellsByKey.size);
  el("m-esdf-msgs").textContent   = String(esdfMsgCount);
  el("m-esdf-faces").textContent  = String(esdfFacesGeom.length);
  el("m-esdf-drange").textContent = esdfCellsByKey.size > 0 && Number.isFinite(esdfDMin)
    ? `${esdfDMin.toFixed(1)} → ${esdfDMax.toFixed(1)}`
    : "—";
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
    if (!state.takeoffPos) state.takeoffPos = {...ego};
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

  if (snap.depth_source_name) state.diagnostics.depth_source_name = snap.depth_source_name;
  if (snap.pipeline && typeof snap.pipeline === "object") {
    state.diagnostics.pipeline = snap.pipeline;
  }

  const mission=snap.mission_local_obstacle_map;
  if (mission&&typeof mission==="object") {
    Object.assign(state.diagnostics, {
      raw_evidence_count:   fin(mission.raw_evidence_count),
      compacted_evidence_count: fin(mission.compacted_evidence_count),
    });
  }

  // L0 ego-local flight map: debug_cells = up to 96 occupied||inflated_blocked cells,
  // already sorted occupied-first then by nearest_range_m ascending.
  // center_local [x,y,z]: X=forward, Y=right, Z=down (body frame).
  const lfm=snap.local_flight_map;
  if (lfm&&typeof lfm==="object"&&Array.isArray(lfm.debug_cells)) {
    const parsed=[];
    let nearestM=Infinity;
    for (const raw of lfm.debug_cells) {
      const center=asVec3(raw.center_local);
      if (!center) continue;
      const nr=fin(raw.nearest_range_m, Infinity);
      if (nr<nearestM) nearestM=nr;
      parsed.push({
        center,
        occupied: raw.occupied===true,
        inflated: raw.inflated_blocked===true,
        occupied_score: fin(raw.occupied_score, 0),
        recently_observed: raw.recently_observed===true,
      });
    }
    localFlightMapCells = parsed;
    lfmNearestM         = nearestM;
    lfmCellSizeM        = fin(lfm.cell_size_m, 0.5) || 0.5;
    lfmForwardRangeM    = fin(lfm.forward_range_m, 30) || 30;
    lfmRearRangeM       = fin(lfm.rear_range_m, 6) || 6;
    lfmLateralRangeM    = fin(lfm.lateral_range_m, 15) || 15;
    // Pre-computed polar risk from C++
    lfmEgoSpeedMps  = fin(lfm.ego_speed_mps, 0);
    lfmGlobalMinTTC = fin(lfm.global_min_ttc_s, Infinity);
    lfmEscapeBody   = asVec3(lfm.escape_direction_body) ?? null;
    lfmPolarSectors = Array.isArray(lfm.polar_risk_sectors)
      ? lfm.polar_risk_sectors.map(s => ({
          az:  fin(s.az,  0),
          vr:  fin(s.vr,  0),
          ttc: fin(s.ttc, Infinity),
          nr:  fin(s.nr,  Infinity),
          obs: s.obs === true,
        }))
      : [];
    lfmSphNumAz        = fin(lfm.spherical_num_az, 36) || 36;
    lfmSphNumEl        = fin(lfm.spherical_num_el, 9)  || 9;
    lfmSensorAzHalfRad = fin(lfm.sensor_az_half_rad, 0);
    lfmSensorElHalfRad = fin(lfm.sensor_el_half_rad, 0);
    lfmSensorGridCols  = (lfm.sensor_grid_cols | 0);
    lfmSensorGridRows  = (lfm.sensor_grid_rows | 0);
    lfmSphericalBins = Array.isArray(lfm.spherical_risk_bins)
      ? lfm.spherical_risk_bins.map(b => ({
          az:  fin(b.az,  0),
          el:  fin(b.el,  0),
          ttc: fin(b.ttc, Infinity),
          vr:  fin(b.vr,  0),
          nr:  fin(b.nr,  Infinity),
          sm:  b.sm | 0,
        }))
      : [];
    lfmSensorObs = Array.isArray(lfm.sensor_observations)
      ? lfm.sensor_observations.map(o => ({
          az:  fin(o.az,  0),
          el:  fin(o.el,  0),
          r:   fin(o.r,   Infinity),
          vr:  fin(o.vr,  0),
          ttc: fin(o.ttc, Infinity),
          src: o.src | 0,
        }))
      : [];
  } else {
    // No LFM block in this snapshot — clear all L0 state so stale data is not displayed.
    localFlightMapCells = [];
    lfmNearestM         = Infinity;
    lfmEgoSpeedMps      = 0;
    lfmGlobalMinTTC     = Infinity;
    lfmEscapeBody       = null;
    lfmPolarSectors     = [];
    lfmSphericalBins    = [];
    lfmSensorObs        = [];
    lfmSensorAzHalfRad  = 0;
    lfmSensorElHalfRad  = 0;
    lfmSensorGridCols   = 0;
    lfmSensorGridRows   = 0;
  }

  // Region ID — first uncertain_region from the world model.
  const regions = snap.uncertain_regions;
  if (Array.isArray(regions) && regions.length > 0) {
    const rid = String(regions[0].region_id || "");
    if (rid) {
      const friendly = rid.replace(/_/g, " ").replace(/\b\w/g, c => c.toUpperCase());
      const regionEl = el("m-region-id");
      if (regionEl) regionEl.textContent = friendly + (regions.length > 1 ? ` +${regions.length-1}` : "");
    }
  }

  // Perch candidates — green landing pads from PerchCandidateEvaluator.
  if (Array.isArray(snap.perch_candidates)) {
    state.perchCandidates = snap.perch_candidates;
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

  if (!deferRender) { recomputeBounds(); updateMetrics(); scheduleDraw(); }
  return true;
}

function applyPlanningDelta(plan, {deferRender=false}={}) {
  // Incremental update: merge changed L2 cells without clearing the full map.
  if (!plan || !Array.isArray(plan.cells) || plan.cells.length === 0) return false;
  const csM = fin(plan.cell_size_m, planCellSizeM) || planCellSizeM;
  const vcM = fin(plan.vertical_cell_size_m, planVCellSizeM) || planVCellSizeM;
  for (const raw of plan.cells) {
    const center = asVec3(raw.center_map);
    if (!center) continue;
    const updated_at = fin(raw.t, 0);
    planningCellsByKey.set(
      planningQuantKey(center.x, center.y, center.z, csM, vcM),
      { center,
        occupied_score:    fin(raw.occupied_score, 0),
        confidence:        fin(raw.confidence, 0),
        source_cell_count: fin(raw.source_cell_count, 0),
        updated_at }
    );
    if (updated_at > planNewestTs) planNewestTs = updated_at;
    // Expand planBounds for scene radius (never shrinks on delta).
    if (!planBounds) {
      planBounds={min_x:center.x,max_x:center.x,min_y:center.y,max_y:center.y,min_z:center.z,max_z:center.z};
    } else {
      if (center.x<planBounds.min_x) planBounds.min_x=center.x;
      if (center.x>planBounds.max_x) planBounds.max_x=center.x;
      if (center.y<planBounds.min_y) planBounds.min_y=center.y;
      if (center.y>planBounds.max_y) planBounds.max_y=center.y;
      if (center.z<planBounds.min_z) planBounds.min_z=center.z;
      if (center.z>planBounds.max_z) planBounds.max_z=center.z;
    }
  }
  // Any delta means live L1→L2 data arrived this session; L3 arrows go live too.
  if (!l2HasLiveData) {
    l2HasLiveData = true;
    buildESDFArrows();
  }
  buildPlanningFaces();
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

  if (!deferRender) { recomputeBounds(); updateMetrics(); scheduleDraw(); }
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
  const pPlan=live.pendingPlanningSnapshot; const pPlanIsDelta=live.pendingPlanningIsDelta;
  live.pendingPlanningSnapshot=null; live.pendingPlanningIsDelta=false;
  const pGhost=live.pendingGhostDetections;
  live.pendingGhostDetections=null;

  let changed=false;
  if (pCells.length>0)  changed=applyMissionObstacleMapDelta({cells:pCells}, pSeq, {deferRender:true})>0||changed;
  if (pSnap)            changed=applyWorldSnapshot(pSnap, pSnapSeq, {deferRender:true})||changed;
  if (pTrav)            changed=(pTravIsDelta
                          ? applyTravDelta(pTrav, {deferRender:true})
                          : applyTravSnapshot(pTrav, {deferRender:true}))||changed;
  if (pPlan)            changed=(pPlanIsDelta
                          ? applyPlanningDelta(pPlan, {deferRender:true})
                          : applyPlanningSnapshot(pPlan, {deferRender:true}))||changed;
  if (pGhost)           { applyGhostDetections(pGhost, {deferRender:true}); changed=true; }

  if (changed) { recomputeBounds(); updateMetrics(); scheduleDraw(); }
  else { updateMetrics(); }

  if ((live.pendingDeltaCells.length||live.pendingWorldSnapshot||live.pendingTravSnapshot||live.pendingPlanningSnapshot||live.pendingGhostDetections)
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
      dbgLog("trav_snapshot", `seq=${payload.seq??"-"} cells=${trav.cells.length}`);
      live.pendingTravSnapshot=trav;
      live.pendingTravIsDelta=false;  // full snapshot — client clears + rebuilds
      scheduleLiveProcessing();
    } catch(e) {}
  });

  source.addEventListener("planning_map_snapshot", ev => {
    try {
      const payload=JSON.parse(ev.data);
      const plan=payload.planning_map_snapshot;
      if (!plan||!Array.isArray(plan.cells)) return;
      dbgLog("plan_snapshot", `seq=${payload.map_seq??"-"} cells=${plan.cells.length}`);
      live.pendingPlanningSnapshot=plan;
      live.pendingPlanningIsDelta=false;  // full snapshot — clear + rebuild
      scheduleLiveProcessing();
    } catch(e) {}
  });

  source.addEventListener("planning_map_delta", ev => {
    try {
      const payload=JSON.parse(ev.data);
      const plan=payload.planning_map_delta;
      if (!plan||!Array.isArray(plan.cells)) return;
      dbgLog("plan_delta", `seq=${payload.map_seq??"-"} cells=${plan.cells.length}`);
      live.pendingPlanningSnapshot=plan;
      live.pendingPlanningIsDelta=true;   // incremental — merge only
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

  source.addEventListener("esdf_delta", ev => {
    try {
      const payload=JSON.parse(ev.data);
      const esdf=payload.esdf_delta;
      if (!esdf) return;
      esdfConsecErrors = 0;   // reset on any successful parse
      esdfMsgCount++;
      const wasFull = !esdf.is_delta;
      const mapSeq  = payload.map_seq ?? -1;
      // Separate DB baseline (map_seq=0 static-cache burst) from live snapshots.
      // Live full snapshots (is_delta=false, map_seq>0) must not erase DB-loaded cells.
      if (wasFull && mapSeq === 0) {
        // DB burst: save as permanent baseline; reset live view from scratch.
        esdfDbCellsByKey.clear();
        esdfCellsByKey.clear(); esdfDMin=Infinity; esdfDMax=-Infinity;
      } else if (wasFull) {
        // Live full snapshot: clear live cells, restore DB baseline first.
        esdfCellsByKey.clear(); esdfDMin=Infinity; esdfDMax=-Infinity;
        for (const [k,v] of esdfDbCellsByKey) {
          esdfCellsByKey.set(k, v);
          if (v.d < esdfDMin) esdfDMin = v.d;
          if (v.d > esdfDMax) esdfDMax = v.d;
        }
      }
      esdfD0M        = esdf.d0_m         || 5.0;
      esdfCellSizeM  = esdf.cell_size_m  || 1.0;
      esdfVCellSizeM = esdf.vcell_size_m || 2.0;
      // Keep slider max in sync with d0 so R can span the full truncation range.
      const rSlider = el("esdf-r-slider");
      // Slider range is fixed 1–20 m; do not override with d0M.
      if (esdf.net_rep) esdfNetRepulsion=esdf.net_rep;
      const inCells = Array.isArray(esdf.cells) ? esdf.cells : [];
      for (const c of inCells) {
        const key  = esdfQuantKey(c.x, c.y, c.z);
        const cell = {...c, updated_at: fin(c.t, 0)};
        esdfCellsByKey.set(key, cell);
        if (wasFull && mapSeq === 0) esdfDbCellsByKey.set(key, cell);
        if (c.d < esdfDMin) esdfDMin = c.d;
        if (c.d > esdfDMax) esdfDMax = c.d;
      }
      buildESDFFaces();
      buildESDFArrows();
      const repStr = esdf.net_rep
        ? `rep=(${esdf.net_rep.x?.toFixed(2)},${esdf.net_rep.y?.toFixed(2)},${esdf.net_rep.z?.toFixed(2)})`
        : "rep=null";
      dbgLog("esdf_delta",
        `seq=${payload.map_seq??"-"} ${wasFull?"FULL":"delta"} cells=${inCells.length} total=${esdfCellsByKey.size} d0=${esdfD0M} cs=${esdfCellSizeM} ${repStr}`);
      if (DEBUG) console.log("[esdf_delta]", {seq:payload.map_seq, full:wasFull, cells:inCells.length, total:esdfCellsByKey.size, faces:esdfFacesGeom.length, dMin:esdfDMin.toFixed(2), dMax:esdfDMax.toFixed(2)});
      scheduleDraw();
    } catch(e) {
      esdfConsecErrors++;
      // Log context bytes around the parse position to diagnose the root cause.
      const posMatch = e.message.match(/at position (\d+)/);
      const errPos = posMatch ? parseInt(posMatch[1]) : -1;
      const ctxSnip = errPos >= 0
        ? `| ctx: "...${ev.data.substring(Math.max(0, errPos - 25), errPos + 25)}..."`
        : "";
      dbgLog("esdf_delta", `PARSE ERROR (${esdfConsecErrors}): ${e.message} ${ctxSnip}`, "error");
      // Any SSE parse failure means the stream is misaligned — reconnect immediately.
      // The server replays the full L2/L3 snapshot to new clients so we recover fast.
      {
        esdfConsecErrors = 0;
        dbgLog("esdf_delta", "stream corrupt — reconnecting SSE", "warn");
        source.close();
        setTimeout(startLiveStream, 500);
      }
    }
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

// ── L3 metric group helpers ────────────────────────────────────────────────────

function toggleL3Metrics() {
  const body  = el("l3-body");
  const caret = el("l3-caret");
  const open  = body.classList.toggle("open");
  caret.textContent = open ? "▼" : "▶";
}

function copyL3Metrics(e) {
  e.stopPropagation();  // don't trigger toggleL3Metrics
  const lines = [
    `L3 ESDF cells: ${el("m-esdf-cells").textContent}`,
    `L3 msgs rcvd:  ${el("m-esdf-msgs").textContent}`,
    `L3 ESDF faces: ${el("m-esdf-faces").textContent}`,
    `L3 d range:    ${el("m-esdf-drange").textContent}`,
    `L3 arrows:     ${el("m-esdf-arrows").textContent}`,
    `L3 smooth R:   ${el("m-esdf-r").textContent}`,
  ];
  navigator.clipboard?.writeText(lines.join("\n")).catch(()=>{});
}

// Called when the R slider moves.  Updates the global radius and rebuilds arrows.
function onEsdfRSlider(val) {
  esdfSmoothR = parseFloat(val);
  el("m-esdf-r").textContent = esdfSmoothR.toFixed(1) + "m";
  buildESDFArrows();
  scheduleDraw();
  saveUIState();
}

function onEsdfShowCells(checked) {
  state.showEsdfCells = checked;
  scheduleDraw();
  saveUIState();
}

// ── view controls + interaction ────────────────────────────────────────────────

function installViewControls() {
  el("view-center")?.addEventListener("click", ()=>{
    // Re-center pan to the scene centroid without changing yaw/pitch.
    recomputeBounds();
    window.animateViewPreset(yaw, pitch, zoom, "center", cloneP(sceneCenter()));
  });
  // ── Unified snap-to-preset logic ──────────────────────────────────────────────
  // Canonical yaws: 45°, 90°, 135°, 270°, 225°, 315°, 360° (body-diagonals — balanced perspective).
  // First press: snap pitch to target + nearest canonical yaw. Zoom preserved.
  // Already on preset: advance +45° through the canonical cycle.
  const SNAP_YAWS_RAD = [45, 90, 135, 180, 225, 270, 315, 360].map(d => d * Math.PI / 180);
  const SNAP_TOL = 6 * Math.PI / 180;  // 6° tolerance for "are we already there?"

  function snapToPreset(targetPitchRad) {
    const normYaw = ((yaw % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI);
    let nearestIdx = 0, nearestDist = Infinity;
    for (let i = 0; i < SNAP_YAWS_RAD.length; i++) {
      const d = Math.abs(window.shortestAngleDelta(normYaw, SNAP_YAWS_RAD[i]));
      if (d < nearestDist) { nearestDist = d; nearestIdx = i; }
    }
    const atPitch     = Math.abs(pitch - targetPitchRad) < SNAP_TOL;
    const atCanonical = nearestDist < SNAP_TOL;
    // If already on this preset, step to next canonical; otherwise snap to nearest.
    const targetIdx = (atPitch && atCanonical) ? (nearestIdx + 1) % SNAP_YAWS_RAD.length : nearestIdx;
    window.animateViewPreset(SNAP_YAWS_RAD[targetIdx], targetPitchRad, zoom, "preset");
  }

  el("view-45")?.addEventListener("click",   () => snapToPreset(Math.PI / 4));
  el("view-side")?.addEventListener("click", () => snapToPreset(0));
  el("view-top")?.addEventListener("click",  () => snapToPreset(Math.PI / 2 - 0.01));
  el("view-zoom-in")?.addEventListener("click", ()=>{
    window.animateZoom(clamp(zoom * 1.25, 0.08, 25));
  });
  el("view-zoom-out")?.addEventListener("click", ()=>{
    window.animateZoom(clamp(zoom / 1.25, 0.08, 25));
  });

  const lodSlider = el("trav-lod");
  if (lodSlider) {
    lodSlider.addEventListener("input", ()=>{
      travDisplayLevelM = TRAV_LOD_LEVELS[Number(lodSlider.value)];
      el("trav-lod-val").textContent = travDisplayLevelM + "m";
      buildTravFaces();      // L1 only: aggregates 0.5 m cells at new LOD
      // L2 is not re-built here — it renders at its own native 1m cell size
      scheduleDraw();
    });
  }

  updateZoomDisplay();

  // Layer toggle buttons: click toggles .active class and matching state flag.
  const layerToggles=[
    ["toggle-l0",         "showL0"],
    ["toggle-planning",   "showPlanning"],
    ["toggle-trav",       "showTrav"],
    ["toggle-obstacles",  "showObstacles"],
    ["toggle-ghosts",     "showGhosts"],
    ["toggle-sensing",    "showSensing"],
    ["toggle-trajectory", "showTrajectory"],
    ["toggle-esdf",       "showEsdf"],
  ];
  for (const [id, stateKey] of layerToggles) {
    const btn=el(id);
    if (!btn) continue;
    btn.addEventListener("click", ()=>{
      state[stateKey] = !state[stateKey];
      btn.classList.toggle("active", state[stateKey]);
      if (stateKey==="showPlanning") buildPlanningFaces();
      if (stateKey==="showEsdf")    { buildESDFFaces(); buildESDFArrows(); }
      scheduleDraw();
      saveUIState();
    });
  }

  // Trav color mode toggle: height (default) ↔ type (occupied/partial)
  el("trav-color-btn")?.addEventListener("click", ()=>{
    state.travColorByType = !state.travColorByType;
    const tag = el("trav-color-tag");
    if (tag) tag.textContent = state.travColorByType ? "Type" : "Height";
    const lh = el("trav-legend-height");
    const lt = el("trav-legend-type");
    if (lh) lh.style.visibility = state.travColorByType ? "hidden" : "visible";
    if (lt) lt.style.visibility = state.travColorByType ? "visible" : "hidden";
    // L2 face colors depend on mode; rebuild geometry cache.
    buildPlanningFaces();
    scheduleDraw();
    saveUIState();
  });

  // esdf-vel-mode has an inline onchange handler; add saveUIState as a second listener.
  el("esdf-vel-mode")?.addEventListener("change", saveUIState);

  // Restore persisted UI state.  Must run after all handlers are wired so that
  // button active classes and state flags are in sync before the first draw.
  loadUIState();
}

// Canvas mouse interaction
let dragging=false, lastX=0, lastY=0;
const hoverCard=el("hover-card");

canvas.addEventListener("mousedown", e=>{
  if (window.viewAnimationHandle!==null) { cancelAnimationFrame(window.viewAnimationHandle); window.viewAnimationHandle=null; }
  dragging=true; lastX=e.clientX; lastY=e.clientY;
  _mouseDownX=e.clientX; _mouseDownY=e.clientY;
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
  updateZoomDisplay();
  draw();
}, {passive:false});

// ── hover card — obstacle cells + trav voxels ─────────────────────────────────

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
  return {item: bestD<=thresh?best:null, dist: bestD};
}

function nearestTravFaceToCanvas(px, py) {
  if (!state.showTrav) return {item:null, dist:Infinity};
  let best=null, bestD=Infinity;
  const thresh=20*devicePixelRatio;
  for (const face of travFacesGeom) {
    const p=project(face.centroid);
    if (!p||!Number.isFinite(p.x)) continue;
    const d=Math.hypot(p.x-px, p.y-py);
    if (d<bestD) { bestD=d; best=face; }
  }
  return {item: bestD<=thresh?best:null, dist: bestD};
}

function hoverCellHtml(cell) {
  const hM=-fin(cell.center?.z);
  return [
    "<b>Obstacle cell</b>",
    `<div>height: <code>${hM.toFixed(2)} m</code></div>`,
    `<div>occupied_score: <code>${fin(cell.occupied_score).toFixed(2)}</code></div>`,
    `<div>free_score:     <code>${fin(cell.free_score).toFixed(2)}</code></div>`,
    `<div>confidence:     <code>${fin(cell.confidence).toFixed(2)}</code></div>`,
    `<div class="muted">x=${fin(cell.center.x).toFixed(2)}, y=${fin(cell.center.y).toFixed(2)}, z=${fin(cell.center.z).toFixed(2)}</div>`,
    `<div class="copy-hint">click to copy</div>`,
  ].join("");
}
function hoverCellText(cell) {
  const hM=-fin(cell.center?.z);
  return [
    "Obstacle cell",
    `height: ${hM.toFixed(2)} m`,
    `occupied_score: ${fin(cell.occupied_score).toFixed(2)}`,
    `free_score:     ${fin(cell.free_score).toFixed(2)}`,
    `confidence:     ${fin(cell.confidence).toFixed(2)}`,
    `x=${fin(cell.center.x).toFixed(2)}, y=${fin(cell.center.y).toFixed(2)}, z=${fin(cell.center.z).toFixed(2)}`,
  ].join("\\n");
}

function hoverTravHtml(face) {
  const hM=(-face.vz).toFixed(1);
  const type=face.isPartial?"Partial":"Occupied";
  return [
    `<b>Trav voxel</b> <span class="muted">${face.levelM}m LOD</span>`,
    `<div>type: <code>${type}</code>  confidence: <code>${face.confidence.toFixed(2)}</code></div>`,
    `<div>height: <code>${hM} m</code></div>`,
    `<div>occ cells: <code>${face.occ}</code>  edge cells: <code>${face.edge}</code></div>`,
    `<div class="muted">x=${face.vx.toFixed(1)}, y=${face.vy.toFixed(1)}, z=${face.vz.toFixed(1)}</div>`,
    `<div class="copy-hint">click to copy</div>`,
  ].join("");
}
function hoverTravText(face) {
  const hM=(-face.vz).toFixed(1);
  const type=face.isPartial?"Partial":"Occupied";
  return [
    `Trav voxel (${face.levelM}m LOD)`,
    `type: ${type}  confidence: ${face.confidence.toFixed(2)}`,
    `height: ${hM} m`,
    `occ cells: ${face.occ}  edge cells: ${face.edge}`,
    `x=${face.vx.toFixed(1)}, y=${face.vy.toFixed(1)}, z=${face.vz.toFixed(1)}`,
  ].join("\\n");
}

// Active hover: {htmlFn, textFn} or null
let _hover = null;
let _copyFlashTimer = null;
let _mouseDownX = 0, _mouseDownY = 0;

function _showHoverCard(htmlFn, textFn, cx, cy) {
  _hover = {htmlFn, textFn};
  hoverCard.innerHTML = htmlFn();
  hoverCard.style.left = `${cx+14}px`;
  hoverCard.style.top  = `${cy+14}px`;
  hoverCard.style.display = "block";
}

function _copyHover() {
  if (!_hover || !hoverCard) return;
  const text = _hover.textFn();
  navigator.clipboard?.writeText(text).then(()=>{
    if (_copyFlashTimer) clearTimeout(_copyFlashTimer);
    hoverCard.innerHTML = _hover.htmlFn().replace(
      '<div class="copy-hint">click to copy</div>',
      '<div class="copy-flash">✓ Copied!</div>'
    );
    _copyFlashTimer = setTimeout(()=>{
      if (_hover) hoverCard.innerHTML = _hover.htmlFn();
    }, 1200);
  });
}

canvas.addEventListener("mousemove", e=>{
  if (dragging||!hoverCard) return;
  const rect=canvas.getBoundingClientRect();
  const px=(e.clientX-rect.left)*devicePixelRatio;
  const py=(e.clientY-rect.top)*devicePixelRatio;

  const obsRes  = nearestObsCellToCanvas(px, py);
  const travRes = nearestTravFaceToCanvas(px, py);

  const cell = obsRes.item, face = travRes.item;
  if (!cell && !face) { hoverCard.style.display="none"; _hover=null; return; }

  // Pick whichever is closer to the cursor.
  if (cell && (!face || obsRes.dist <= travRes.dist)) {
    _showHoverCard(()=>hoverCellHtml(cell), ()=>hoverCellText(cell), e.clientX, e.clientY);
  } else {
    _showHoverCard(()=>hoverTravHtml(face), ()=>hoverTravText(face), e.clientX, e.clientY);
  }
});
canvas.addEventListener("mouseleave", ()=>{ if (hoverCard) hoverCard.style.display="none"; _hover=null; });

// Canvas click: copy hover info if cursor barely moved (not a drag).
canvas.addEventListener("click", e=>{
  if (Math.hypot(e.clientX-_mouseDownX, e.clientY-_mouseDownY) > 5) return;
  _copyHover();
});
// Hover card click also copies (card is offset 14 px from cursor, still clickable).
if (hoverCard) hoverCard.addEventListener("click", _copyHover);

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
if (DEBUG) {
  el("debug-section").style.display = "block";
  console.log("Debug mode active — SSE events will be logged here and to console.");
}
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

    try:
        git_rev = subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=Path(__file__).parent,
            stderr=subprocess.DEVNULL,
            text=True,
        ).strip()
    except Exception:
        git_rev = "unknown"

    html = HTML.replace("__BUILD_REV__", git_rev)

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(html, encoding="utf-8")
    print(f"OK: wrote {output} (rev {git_rev})")
    print(f"OK: {len(html):,} bytes")
    return 0


if __name__ == "__main__":
    sys.exit(main())
