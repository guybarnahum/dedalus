#!/usr/bin/env python3
"""
l2_editor.py — Interactive browser-based editor for MissionLocalPlanningMap SQLite databases.

Usage:
    python3 tools/mission/l2_editor.py <db_path> [--port 8091] [--host 127.0.0.1]

Opens a browser editor at http://localhost:8091 with:
  • 45° isometric Three.js voxel preview  (color by age / score / confidence)
  • Filter panel: delete by age, date range, score threshold, confidence threshold
  • Preview-before-delete (highlights matching cells in red without committing)
  • Auto-backup before every destructive operation
  • Export: copy to local path, upload to S3 (requires boto3)

SQLite schema (MissionLocalPlanningMap):
    cells(xi, yi, zi, score, confidence, count, updated_ns)
    params(key, value)   — cell_size_m, vertical_cell_size_m
    esdf_cells(...)      — derived, not edited here

S3 export:  pip install boto3
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sqlite3
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from urllib.parse import urlparse

# ─── globals ──────────────────────────────────────────────────────────────────

DB_PATH: Path | None = None
DB_LOCK = threading.Lock()

# ─── SQLite helpers ───────────────────────────────────────────────────────────

def _connect() -> sqlite3.Connection:
    conn = sqlite3.connect(str(DB_PATH), timeout=10)
    conn.row_factory = sqlite3.Row
    return conn


def _get_params(conn: sqlite3.Connection) -> dict:
    try:
        rows = conn.execute("SELECT key, value FROM params").fetchall()
        return {r["key"]: r["value"] for r in rows}
    except sqlite3.OperationalError:
        return {}


def api_stats() -> dict:
    with DB_LOCK, _connect() as conn:
        params = _get_params(conn)
        total = conn.execute("SELECT COUNT(*) FROM cells").fetchone()[0]
        score_min, score_max = conn.execute(
            "SELECT MIN(score), MAX(score) FROM cells"
        ).fetchone()
        ts_min, ts_max = conn.execute(
            "SELECT MIN(updated_ns), MAX(updated_ns) FROM cells WHERE updated_ns > 0"
        ).fetchone()
        now_ns = int(time.time() * 1e9)
        return {
            "total_cells": total,
            "score_min": score_min,
            "score_max": score_max,
            "ts_min_ns": ts_min,
            "ts_max_ns": ts_max,
            "now_ns": now_ns,
            "db_path": str(DB_PATH),
            "db_size_bytes": DB_PATH.stat().st_size if DB_PATH.exists() else 0,
            "params": params,
        }


def api_cells() -> dict:
    with DB_LOCK, _connect() as conn:
        params = _get_params(conn)
        cell_size   = params.get("cell_size_m", 1.0)
        vcell_size  = params.get("vertical_cell_size_m", 2.0)
        now_ns = int(time.time() * 1e9)
        rows = conn.execute(
            "SELECT xi, yi, zi, score, confidence, count, updated_ns "
            "FROM cells ORDER BY updated_ns DESC"
        ).fetchall()
        cells = []
        for r in rows:
            xi, yi, zi = r["xi"], r["yi"], r["zi"]
            updated_ns = r["updated_ns"] or 0
            cells.append({
                "xi": xi, "yi": yi, "zi": zi,
                "cx": (xi + 0.5) * cell_size,
                "cy": (yi + 0.5) * cell_size,
                "cz": (zi + 0.5) * vcell_size,
                "score": r["score"],
                "confidence": r["confidence"],
                "count": r["count"],
                "updated_ns": updated_ns,
                # age_s == -1 means no timestamp (updated_ns==0); JS treats as max age
                "age_s": (now_ns - updated_ns) / 1e9 if updated_ns > 0 else -1,
            })
        return {
            "cells": cells,
            "params": {"cell_size_m": cell_size, "vertical_cell_size_m": vcell_size},
        }


def _build_filter_clause(body: dict) -> tuple[str, list]:
    """Return (WHERE conditions string, params list) or ('', []) if no filter."""
    conds: list[str] = []
    params: list = []
    now_ns = int(time.time() * 1e9)

    max_age_s = body.get("max_age_s")
    if max_age_s is not None:
        cutoff = now_ns - int(float(max_age_s) * 1e9)
        conds.append("updated_ns < ?")
        params.append(cutoff)

    before_ns = body.get("before_ns")
    if before_ns is not None:
        conds.append("updated_ns < ?")
        params.append(int(before_ns))

    after_ns = body.get("after_ns")
    if after_ns is not None:
        conds.append("updated_ns > ?")
        params.append(int(after_ns))

    max_score = body.get("max_score")
    if max_score is not None:
        conds.append("score < ?")
        params.append(float(max_score))

    max_confidence = body.get("max_confidence")
    if max_confidence is not None:
        conds.append("confidence < ?")
        params.append(float(max_confidence))

    # Spatial: world coords → grid indices.  Caller may pass xi_min/xi_max directly
    # or cx_min/cx_max (world metres).  Both are supported.
    with _connect() as conn:
        p = _get_params(conn)
    csz  = p.get("cell_size_m", 1.0)
    vcsz = p.get("vertical_cell_size_m", 2.0)

    def _world_to_xi(x: float) -> int:
        import math
        return int(math.floor(x / csz))

    def _world_to_yi(y: float) -> int:
        import math
        return int(math.floor(y / csz))

    def _world_to_zi(z: float) -> int:
        import math
        return int(math.floor(z / vcsz))

    for prefix, col, to_idx in [
        ("xi", "xi", int),
        ("yi", "yi", int),
        ("zi", "zi", int),
    ]:
        lo = body.get(f"{prefix}_min")
        hi = body.get(f"{prefix}_max")
        if lo is not None:
            conds.append(f"{col} >= ?")
            params.append(to_idx(lo))
        if hi is not None:
            conds.append(f"{col} <= ?")
            params.append(to_idx(hi))

    for (wk_lo, wk_hi, col, to_idx) in [
        ("cx_min", "cx_max", "xi", _world_to_xi),
        ("cy_min", "cy_max", "yi", _world_to_yi),
        ("cz_min", "cz_max", "zi", _world_to_zi),
    ]:
        lo = body.get(wk_lo)
        hi = body.get(wk_hi)
        if lo is not None:
            conds.append(f"{col} >= ?")
            params.append(to_idx(float(lo)))
        if hi is not None:
            conds.append(f"{col} <= ?")
            params.append(to_idx(float(hi)))

    return " AND ".join(conds), params


def api_preview_delete(body: dict) -> dict:
    """Dry-run: return count + sample of cells that would be deleted."""
    clause, params = _build_filter_clause(body)
    if not clause:
        return {"error": "no filter specified", "would_delete": 0}
    with DB_LOCK, _connect() as conn:
        p = _get_params(conn)
        csz  = p.get("cell_size_m", 1.0)
        vcsz = p.get("vertical_cell_size_m", 2.0)
        now_ns = int(time.time() * 1e9)
        rows = conn.execute(
            f"SELECT xi, yi, zi, score, confidence, updated_ns FROM cells WHERE {clause}",
            params,
        ).fetchall()
        count = len(rows)
        sample = []
        for r in rows[:200]:
            xi, yi, zi = r["xi"], r["yi"], r["zi"]
            updated_ns = r["updated_ns"] or 0
            sample.append({
                "xi": xi, "yi": yi, "zi": zi,
                "cx": (xi + 0.5) * csz,
                "cy": (yi + 0.5) * csz,
                "cz": (zi + 0.5) * vcsz,
                "score": r["score"],
                "confidence": r["confidence"],
                "age_s": (now_ns - updated_ns) / 1e9,
            })
    return {"would_delete": count, "sample": sample}


def api_delete(body: dict) -> dict:
    """Delete cells matching filter.  Makes a .bak backup first."""
    clause, params = _build_filter_clause(body)
    if not clause:
        return {"error": "no filter specified", "deleted": 0}

    backup_path = _backup()
    with DB_LOCK, _connect() as conn:
        cur = conn.execute(f"DELETE FROM cells WHERE {clause}", params)
        conn.commit()
        deleted = cur.rowcount
    return {"deleted": deleted, "backup": backup_path}


def _backup() -> str:
    ts = int(time.time())
    dest = str(DB_PATH) + f".bak.{ts}"
    shutil.copy2(str(DB_PATH), dest)
    return dest


def api_backup() -> dict:
    path = _backup()
    return {"backup": path}


def api_export_file(body: dict) -> dict:
    dest = body.get("dest")
    if not dest:
        return {"ok": False, "error": "dest path required"}
    try:
        dest_path = Path(dest)
        dest_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(str(DB_PATH), str(dest_path))
        return {"ok": True, "dest": str(dest_path), "size": dest_path.stat().st_size}
    except Exception as exc:
        return {"ok": False, "error": str(exc)}


def api_export_s3(body: dict) -> dict:
    bucket  = body.get("bucket", "").strip()
    key     = body.get("key", "").strip()
    region  = body.get("region", "").strip() or None
    profile = body.get("profile", "").strip() or None
    if not bucket or not key:
        return {"ok": False, "error": "bucket and key are required"}
    try:
        import boto3  # type: ignore
        session = boto3.Session(profile_name=profile, region_name=region)
        s3 = session.client("s3")
        s3.upload_file(str(DB_PATH), bucket, key)
        size = DB_PATH.stat().st_size
        return {"ok": True, "bucket": bucket, "key": key, "size": size}
    except ImportError:
        return {"ok": False, "error": "boto3 not installed — pip install boto3"}
    except Exception as exc:
        return {"ok": False, "error": str(exc)}


# ─── HTTP server ──────────────────────────────────────────────────────────────

class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):  # quiet
        pass

    def _send_json(self, data: dict, status: int = 200) -> None:
        body = json.dumps(data).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def _send_html(self, html: str) -> None:
        body = html.encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json_body(self) -> dict:
        length = int(self.headers.get("Content-Length", 0))
        if length == 0:
            return {}
        return json.loads(self.rfile.read(length))

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, DELETE, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        path = urlparse(self.path).path
        if path in ("/", "/index.html"):
            self._send_html(EDITOR_HTML)
        elif path == "/api/stats":
            self._send_json(api_stats())
        elif path == "/api/cells":
            self._send_json(api_cells())
        else:
            self.send_error(404)

    def do_POST(self):
        path = urlparse(self.path).path
        body = self._read_json_body()
        if path == "/api/preview_delete":
            self._send_json(api_preview_delete(body))
        elif path == "/api/delete":
            self._send_json(api_delete(body))
        elif path == "/api/backup":
            self._send_json(api_backup())
        elif path == "/api/export/file":
            self._send_json(api_export_file(body))
        elif path == "/api/export/s3":
            self._send_json(api_export_s3(body))
        else:
            self.send_error(404)


# ─── Embedded HTML ────────────────────────────────────────────────────────────

EDITOR_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>L2 Map Editor</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:system-ui,sans-serif;background:#111;color:#ddd;display:flex;height:100vh;overflow:hidden}

/* ── sidebar ── */
#sidebar{width:285px;min-width:285px;background:#1a1a1a;border-right:1px solid #333;display:flex;flex-direction:column;overflow:hidden}
#sidebar-top{padding:10px 12px 6px;border-bottom:1px solid #333}
#sidebar-top h1{font-size:13px;font-weight:600;color:#aaa;letter-spacing:.05em;text-transform:uppercase}
#db-path{font-size:10px;color:#555;margin-top:2px;word-break:break-all}
#stats-bar{font-size:11px;color:#888;padding:6px 12px;border-bottom:1px solid #333;line-height:1.7}
#stats-bar span{color:#ccc}

/* ── tabs ── */
#tabs{display:flex;border-bottom:1px solid #333}
.tab{flex:1;padding:7px 4px;font-size:11px;text-align:center;cursor:pointer;color:#666;border-bottom:2px solid transparent;transition:color .15s}
.tab.active{color:#5af;border-bottom-color:#5af}
.tab-panel{display:none;flex:1;overflow-y:auto;padding:10px 12px}
.tab-panel.active{display:block}

/* ── form elements ── */
label{display:block;font-size:11px;color:#888;margin-top:10px;margin-bottom:3px}
label:first-child{margin-top:0}
input[type=number],input[type=text],input[type=datetime-local],select{
  width:100%;background:#222;border:1px solid #333;border-radius:4px;
  color:#ccc;padding:5px 7px;font-size:11px}
input:focus,select:focus{outline:1px solid #5af;border-color:#5af}
.row2{display:grid;grid-template-columns:1fr 1fr;gap:6px}
.hint{font-size:10px;color:#555;margin-top:3px}
.shortcuts{display:flex;gap:5px;margin-top:5px}

/* ── buttons ── */
button{border:none;border-radius:4px;padding:7px 10px;font-size:11px;cursor:pointer;font-weight:500;width:100%;margin-top:8px;transition:opacity .15s}
button:disabled{opacity:.4;cursor:default}
button:hover:not(:disabled){opacity:.85}
.btn-sc{background:#1e2a1e;color:#6d6;padding:4px 9px;font-size:10px;width:auto;margin-top:0;border-radius:3px;flex:1}
.btn-reset{background:#222;color:#777;padding:4px 9px;font-size:10px;width:auto;margin-top:0;border-radius:3px}
.btn-delete{background:#501a1a;color:#f88}
.btn-safe{background:#1a3020;color:#7f7}
.btn-export{background:#252535;color:#99f}
#preview-count{font-size:11px;color:#fa0;margin-top:6px;min-height:16px}

/* ── colorbar ── */
#colorbar{display:flex;align-items:center;gap:6px;padding:5px 12px 4px;font-size:10px;color:#555;border-bottom:1px solid #282828}
#colorbar-gradient{height:8px;flex:1;border-radius:3px}
#color-label-lo{min-width:24px}
#color-label-hi{min-width:24px;text-align:right}

/* ── canvas ── */
#canvas-wrap{flex:1;position:relative;overflow:hidden;background:#0d0d0d;display:flex;flex-direction:column}
canvas{display:block;flex:1;width:100%}
#hud{position:absolute;top:36px;right:10px;font-size:10px;color:#444;pointer-events:none;line-height:1.8}
#hud span{color:#666}
#loading{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;font-size:13px;color:#555;pointer-events:none}
</style>
</head>
<body>

<div id="sidebar">
  <div id="sidebar-top">
    <h1>L2 Map Editor</h1>
    <div id="db-path">loading…</div>
  </div>
  <div id="stats-bar">
    Cells: <span id="s-total">–</span> &nbsp;
    Score: <span id="s-score">–</span> &nbsp;
    Age: <span id="s-age">–</span>
  </div>

  <div id="tabs">
    <div class="tab active" data-panel="filter">Filter</div>
    <div class="tab" data-panel="export">Export</div>
    <div class="tab" data-panel="view">View</div>
  </div>

  <!-- ── Filter panel ── -->
  <div class="tab-panel active" id="panel-filter">
    <label>Delete cells older than (hours)</label>
    <input type="number" id="f-age-h" placeholder="e.g. 168  =  7 days" min="0" step="1">

    <label>Date range (from – to)</label>
    <div class="row2">
      <input type="datetime-local" id="f-from" title="From (inclusive lower bound)">
      <input type="datetime-local" id="f-to"   title="To (inclusive upper bound)">
    </div>
    <div class="shortcuts">
      <button class="btn-sc" id="btn-today">Today</button>
      <button class="btn-sc" id="btn-last-hour">Last hour</button>
      <button class="btn-reset" id="btn-reset-filter">Reset</button>
    </div>

    <label>Score below (delete cells with score &lt; this)</label>
    <input type="number" id="f-score" placeholder="e.g. 2.0" min="0" step="0.1">

    <label>Confidence below (delete cells with conf &lt; this)</label>
    <input type="number" id="f-conf" placeholder="e.g. 0.3" min="0" max="1" step="0.05">

    <label>Spatial X range (metres)</label>
    <div class="row2">
      <input type="number" id="f-cx-min" placeholder="min">
      <input type="number" id="f-cx-max" placeholder="max">
    </div>

    <label>Spatial Y range (metres)</label>
    <div class="row2">
      <input type="number" id="f-cy-min" placeholder="min">
      <input type="number" id="f-cy-max" placeholder="max">
    </div>

    <label>Spatial Z range (metres)</label>
    <div class="row2">
      <input type="number" id="f-cz-min" placeholder="min">
      <input type="number" id="f-cz-max" placeholder="max">
    </div>

    <div id="preview-count"></div>
    <button class="btn-delete" id="btn-delete" disabled>Delete <span id="del-label">0</span> cells (auto-backup)</button>
    <button class="btn-safe" id="btn-backup" style="margin-top:14px">Backup .db now</button>
    <button class="btn-safe" id="btn-reload" style="background:#222;color:#888">Reload cells</button>
  </div>

  <!-- ── Export panel ── -->
  <div class="tab-panel" id="panel-export">
    <label style="margin-top:0;font-size:12px;color:#aaa">Copy to file path</label>
    <input type="text" id="exp-file" placeholder="/backup/l2_map_copy.db">
    <button class="btn-export" id="btn-exp-file">Copy to path</button>

    <label style="margin-top:16px;font-size:12px;color:#aaa">Upload to S3</label>
    <label>Bucket</label>
    <input type="text" id="exp-bucket" placeholder="my-bucket">
    <label>Key (S3 object path)</label>
    <input type="text" id="exp-key" placeholder="maps/site_id/l2_map.db">
    <label>Region</label>
    <input type="text" id="exp-region" placeholder="us-east-1">
    <label>AWS profile (optional)</label>
    <input type="text" id="exp-profile" placeholder="default">
    <button class="btn-export" id="btn-exp-s3">Upload to S3</button>
    <div id="exp-status" class="hint" style="margin-top:6px;min-height:14px"></div>
    <div class="hint" style="margin-top:10px">S3 requires: pip install boto3</div>
  </div>

  <!-- ── View panel ── -->
  <div class="tab-panel" id="panel-view">
    <label>Colour by</label>
    <select id="color-by">
      <option value="age">Age (green=new → red=old)</option>
      <option value="score">Score</option>
      <option value="confidence">Confidence</option>
    </select>
    <label>Age colour scale (days = full red)</label>
    <input type="number" id="age-scale" value="30" min="1" step="1">
    <label>Show Z layers</label>
    <div class="row2">
      <input type="number" id="z-min" placeholder="min Z (m)">
      <input type="number" id="z-max" placeholder="max Z (m)">
    </div>
    <button class="btn-safe" id="btn-apply-view" style="margin-top:10px">Apply</button>
    <div class="hint" style="margin-top:14px">
      Pan: right-drag or middle-drag<br>
      Zoom: scroll wheel<br>
      Reset view: double-click canvas
    </div>
  </div>
</div>

<!-- ── 3D view ── -->
<div id="canvas-wrap">
  <div id="colorbar">
    <span id="color-label-lo">new</span>
    <div id="colorbar-gradient" style="background:linear-gradient(to right,#3ecf6e,#f0c330,#e8732a,#d63030)"></div>
    <span id="color-label-hi">old</span>
    &nbsp;
    <span style="color:#333">|</span>
    &nbsp;
    <span style="color:#555">colour: </span>
    <span id="color-mode" style="color:#888">age</span>
    &nbsp;&nbsp;
    <span style="color:#333">highlighted: </span>
    <span id="h-hi" style="color:#fa0">0</span>
    &nbsp;/&nbsp;
    <span id="h-total" style="color:#666">–</span>
  </div>
  <canvas id="c"></canvas>
  <div id="loading">Loading cells…</div>
</div>

<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
<script>
'use strict';

// ── state ──────────────────────────────────────────────────────────────────
let allCells = [];
let highlightedKeys = new Set();   // "xi,yi,zi" strings of cells matching filter
let colorBy = 'age';
let ageScaleDays = 30;
let zFilterMin = -Infinity, zFilterMax = Infinity;
let cellSizeM = 1.0, vCellSizeM = 2.0;
let stats = {};

// ── Three.js setup ─────────────────────────────────────────────────────────
const canvas = document.getElementById('c');
const renderer = new THREE.WebGLRenderer({canvas, antialias: true});
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setClearColor(0x0d0d0d);

let frustumSize = 80;
function aspect() { return canvas.clientWidth / Math.max(canvas.clientHeight, 1); }

let camera = new THREE.OrthographicCamera(-1,1,1,-1,-2000,2000);
const scene = new THREE.Scene();
scene.add(new THREE.AmbientLight(0xffffff, 0.6));
const dirLight = new THREE.DirectionalLight(0xffffff, 0.65);
dirLight.position.set(2, 4, 3);
scene.add(dirLight);

let target = new THREE.Vector3(0, 0, 0);

// ── geometry — shared ──────────────────────────────────────────────────────
// Slightly inset box so adjacent voxels show a thin gap
const GEO = new THREE.BoxGeometry(0.96, 0.96, 0.96);
let mesh = null;
let hiMesh = null;

// ── colour ─────────────────────────────────────────────────────────────────
// NOTE: do NOT use vertexColors:true on MeshLambertMaterial with InstancedMesh.
// BoxGeometry has no vertex color attribute; the shader would multiply instance
// colors by zero, producing black.  Plain white material + instance colors is
// correct — the renderer injects USE_INSTANCING_COLOR automatically.

const _COL = new THREE.Color();

function colorForCell(c) {
  let t;
  if (colorBy === 'age') {
    // age_s=-1 means updated_ns was 0 (no timestamp) → treat as maximally old
    const a = c.age_s < 0 ? ageScaleDays * 86400
            : (isFinite(c.age_s) ? c.age_s : ageScaleDays * 86400);
    t = Math.min(Math.max(a / (ageScaleDays * 86400), 0), 1);
  } else if (colorBy === 'score') {
    const mx = (stats.score_max > 0) ? stats.score_max : 10;
    t = Math.min(Math.max(c.score / mx, 0), 1);
  } else {
    t = Math.min(Math.max(c.confidence, 0), 1);
  }
  // hue: 0.36 (green) → 0 (red), lightness 0.50 — always visible, never black
  _COL.setHSL((1 - t) * 0.36, 0.88, 0.50);
  return _COL.clone();
}

// ── build instanced meshes ─────────────────────────────────────────────────
function buildScene() {
  if (mesh)   { scene.remove(mesh);   mesh.dispose();   mesh = null; }
  if (hiMesh) { scene.remove(hiMesh); hiMesh.dispose(); hiMesh = null; }

  const visible = allCells.filter(c => c.cz >= zFilterMin && c.cz <= zFilterMax);
  const normal  = visible.filter(c => !highlightedKeys.has(`${c.xi},${c.yi},${c.zi}`));
  const hi      = visible.filter(c =>  highlightedKeys.has(`${c.xi},${c.yi},${c.zi}`));

  // Helper: place + colour an InstancedMesh
  function buildMesh(cells, scaleX, scaleY, scaleZ, colFn) {
    if (cells.length === 0) return null;
    const mat = new THREE.MeshLambertMaterial({color: 0xffffff});
    const m = new THREE.InstancedMesh(GEO, mat, cells.length);
    m.instanceMatrix.setUsage(THREE.StaticDrawUsage);
    const M = new THREE.Matrix4();
    cells.forEach((c, i) => {
      // Coordinate mapping: map X→ThreeX, map Z(alt)→ThreeY(up), map Y→Three-Z
      M.makeScale(scaleX, scaleY, scaleZ);
      M.setPosition(c.cx, c.cz, -c.cy);
      m.setMatrixAt(i, M);
      m.setColorAt(i, colFn(c));
    });
    m.instanceMatrix.needsUpdate = true;
    if (m.instanceColor) m.instanceColor.needsUpdate = true;
    return m;
  }

  const HI_COL = new THREE.Color(1.0, 0.22, 0.08);

  mesh   = buildMesh(normal, cellSizeM, vCellSizeM, cellSizeM, colorForCell);
  hiMesh = buildMesh(hi, cellSizeM * 1.06, vCellSizeM * 1.06, cellSizeM * 1.06, () => HI_COL.clone());

  if (mesh)   scene.add(mesh);
  if (hiMesh) { hiMesh.material.transparent = true; hiMesh.material.opacity = 0.88; scene.add(hiMesh); }

  document.getElementById('h-total').textContent = visible.length;
  document.getElementById('h-hi').textContent    = hi.length;
}

// ── camera helpers ─────────────────────────────────────────────────────────
function rebuildCamera() {
  const a = aspect();
  camera.left   = -frustumSize * a / 2;
  camera.right  =  frustumSize * a / 2;
  camera.top    =  frustumSize / 2;
  camera.bottom = -frustumSize / 2;
  // Isometric position: equal weight on all 3 axes from target
  camera.position.copy(target).addScaledVector(
    new THREE.Vector3(1, 1, 1).normalize(), frustumSize * 2
  );
  camera.lookAt(target);
  camera.updateProjectionMatrix();
}

function fitCamera() {
  if (allCells.length === 0) return;
  let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
  allCells.forEach(c => {
    if (c.cx < x0) x0 = c.cx; if (c.cx > x1) x1 = c.cx;
    if (c.cy < y0) y0 = c.cy; if (c.cy > y1) y1 = c.cy;
  });
  target.set((x0 + x1) / 2, 0, -(y0 + y1) / 2);
  frustumSize = Math.max(x1 - x0, y1 - y0, 20) * 1.4;
  rebuildCamera();
}

// ── render loop ────────────────────────────────────────────────────────────
function resize() {
  renderer.setSize(canvas.clientWidth, canvas.clientHeight, false);
  rebuildCamera();
}
window.addEventListener('resize', resize);
(function loop() { requestAnimationFrame(loop); renderer.render(scene, camera); })();

// ── pan / zoom controls ────────────────────────────────────────────────────
// Isometric screen axes in world space
const ISO_RIGHT = new THREE.Vector3(-1, 0,  1).normalize();
const ISO_UP    = new THREE.Vector3(-1, 2, -1).normalize();

let dragActive = false, lastMX = 0, lastMY = 0;

canvas.addEventListener('contextmenu', e => e.preventDefault());
canvas.addEventListener('mousedown', e => {
  if (e.button === 1 || e.button === 2) {
    dragActive = true; lastMX = e.clientX; lastMY = e.clientY;
    e.preventDefault();
  }
});
window.addEventListener('mouseup', () => { dragActive = false; });
window.addEventListener('mousemove', e => {
  if (!dragActive) return;
  const dx = e.clientX - lastMX, dy = e.clientY - lastMY;
  lastMX = e.clientX; lastMY = e.clientY;
  const s = frustumSize / Math.min(canvas.clientWidth, canvas.clientHeight);
  target.addScaledVector(ISO_RIGHT, -dx * s);
  target.addScaledVector(ISO_UP,     dy * s);
  rebuildCamera();
});
canvas.addEventListener('wheel', e => {
  e.preventDefault();
  frustumSize *= e.deltaY > 0 ? 1.12 : 0.89;
  frustumSize = Math.max(4, Math.min(frustumSize, 3000));
  rebuildCamera();
}, {passive: false});
canvas.addEventListener('dblclick', fitCamera);

// ── data load ──────────────────────────────────────────────────────────────
async function loadStats() {
  const r = await fetch('/api/stats');
  stats = await r.json();
  document.getElementById('db-path').textContent = stats.db_path || '';
  document.getElementById('s-total').textContent = stats.total_cells ?? '–';
  if (stats.score_min != null) {
    document.getElementById('s-score').textContent =
      `${stats.score_min.toFixed(1)}–${stats.score_max.toFixed(1)}`;
  }
  if (stats.ts_min_ns && stats.now_ns) {
    const days = ((stats.now_ns - stats.ts_min_ns) / 1e9 / 86400).toFixed(1);
    document.getElementById('s-age').textContent = `0–${days}d`;
    // Auto-set age scale to the actual data range
    const ageEl = document.getElementById('age-scale');
    if (!ageEl._touched) {
      ageEl.value = Math.max(1, Math.ceil(parseFloat(days)));
      ageScaleDays = parseFloat(ageEl.value);
    }
  }
}

async function loadCells() {
  document.getElementById('loading').style.display = 'flex';
  const r = await fetch('/api/cells');
  const data = await r.json();
  allCells = data.cells || [];
  cellSizeM  = data.params?.cell_size_m ?? 1.0;
  vCellSizeM = data.params?.vertical_cell_size_m ?? 2.0;
  document.getElementById('loading').style.display = 'none';
  runPreview();   // apply current filter immediately
  buildScene();
  fitCamera();
}

async function reload() { await loadStats(); await loadCells(); }

// ── client-side filter ────────────────────────────────────────────────────
// Preview is always live — no server round-trip needed.
// The actual delete still goes to the server (authoritative).

function hasAnyFilter() {
  return ['f-age-h','f-from','f-to','f-score','f-conf',
          'f-cx-min','f-cx-max','f-cy-min','f-cy-max','f-cz-min','f-cz-max']
    .some(id => document.getElementById(id).value.trim() !== '');
}

function cellMatchesFilter(c) {
  const ageH = parseFloat(document.getElementById('f-age-h').value);
  if (!isNaN(ageH) && ageH >= 0) {
    // delete cells OLDER than ageH hours → match if age_s > ageH*3600
    // cells with age_s=-1 (no timestamp) are treated as maximally old → always match
    const a = c.age_s < 0 ? Infinity : c.age_s;
    if (a <= ageH * 3600) return false;
  }

  const from = document.getElementById('f-from').value;
  const to   = document.getElementById('f-to').value;
  const cellMs = c.updated_ns / 1e6;
  if (from && cellMs < new Date(from).getTime()) return false;
  if (to   && cellMs > new Date(to).getTime())   return false;

  const maxScore = parseFloat(document.getElementById('f-score').value);
  if (!isNaN(maxScore) && c.score >= maxScore) return false;

  const maxConf = parseFloat(document.getElementById('f-conf').value);
  if (!isNaN(maxConf) && c.confidence >= maxConf) return false;

  for (const [pfx, val] of [['cx', c.cx], ['cy', c.cy], ['cz', c.cz]]) {
    const lo = parseFloat(document.getElementById(`f-${pfx}-min`).value);
    const hi = parseFloat(document.getElementById(`f-${pfx}-max`).value);
    if (!isNaN(lo) && val < lo) return false;
    if (!isNaN(hi) && val > hi) return false;
  }

  return true;
}

// Build the filter body to POST to /api/delete
function buildFilterBody() {
  const body = {};
  const ageH = parseFloat(document.getElementById('f-age-h').value);
  if (!isNaN(ageH) && ageH >= 0) body.max_age_s = ageH * 3600;

  const from = document.getElementById('f-from').value;
  if (from) body.after_ns = new Date(from).getTime() * 1e6;

  const to = document.getElementById('f-to').value;
  if (to) body.before_ns = new Date(to).getTime() * 1e6;

  const score = parseFloat(document.getElementById('f-score').value);
  if (!isNaN(score)) body.max_score = score;

  const conf = parseFloat(document.getElementById('f-conf').value);
  if (!isNaN(conf)) body.max_confidence = conf;

  for (const ax of ['cx','cy','cz']) {
    const lo = parseFloat(document.getElementById(`f-${ax}-min`).value);
    const hi = parseFloat(document.getElementById(`f-${ax}-max`).value);
    if (!isNaN(lo)) body[`${ax}_min`] = lo;
    if (!isNaN(hi)) body[`${ax}_max`] = hi;
  }
  return body;
}

// Always-live preview — debounced, runs client-side
let _previewTimer = null;
function schedulePreview() {
  clearTimeout(_previewTimer);
  _previewTimer = setTimeout(runPreview, 220);
}

function runPreview() {
  if (!hasAnyFilter()) {
    highlightedKeys.clear();
    document.getElementById('preview-count').textContent = '';
    document.getElementById('del-label').textContent = '0';
    document.getElementById('btn-delete').disabled = true;
    buildScene();
    return;
  }
  const matched = allCells.filter(cellMatchesFilter);
  highlightedKeys = new Set(matched.map(c => `${c.xi},${c.yi},${c.zi}`));
  const n = matched.length;
  document.getElementById('preview-count').textContent =
    n > 0 ? `${n} cells selected (red)` : 'no cells match';
  document.getElementById('del-label').textContent = n;
  document.getElementById('btn-delete').disabled = (n === 0);
  buildScene();
}

// Wire every filter input to auto-preview
['f-age-h','f-from','f-to','f-score','f-conf',
 'f-cx-min','f-cx-max','f-cy-min','f-cy-max','f-cz-min','f-cz-max']
  .forEach(id => document.getElementById(id).addEventListener('input', schedulePreview));

function toDatetimeLocal(d) {
  const pad = n => String(n).padStart(2, '0');
  return `${d.getFullYear()}-${pad(d.getMonth()+1)}-${pad(d.getDate())}` +
         `T${pad(d.getHours())}:${pad(d.getMinutes())}`;
}

// [Today] shortcut: from = midnight, to = now
document.getElementById('btn-today').addEventListener('click', () => {
  const now = new Date();
  const midnight = new Date(now); midnight.setHours(0, 0, 0, 0);
  document.getElementById('f-from').value = toDatetimeLocal(midnight);
  document.getElementById('f-to').value   = toDatetimeLocal(now);
  schedulePreview();
});

// [Last Hour] shortcut: from = now-1h, to = now
document.getElementById('btn-last-hour').addEventListener('click', () => {
  const now = new Date();
  const anHourAgo = new Date(now.getTime() - 3600 * 1000);
  document.getElementById('f-from').value = toDatetimeLocal(anHourAgo);
  document.getElementById('f-to').value   = toDatetimeLocal(now);
  schedulePreview();
});

// [Reset] clears all filter inputs and collapses preview
document.getElementById('btn-reset-filter').addEventListener('click', () => {
  ['f-age-h','f-from','f-to','f-score','f-conf',
   'f-cx-min','f-cx-max','f-cy-min','f-cy-max','f-cz-min','f-cz-max']
    .forEach(id => { document.getElementById(id).value = ''; });
  runPreview();
});

// ── delete (server-authoritative) ─────────────────────────────────────────
document.getElementById('btn-delete').addEventListener('click', async () => {
  const n = parseInt(document.getElementById('del-label').textContent) || 0;
  if (n === 0) return;
  if (!confirm(`Delete ${n} cells from the L2 map?\n\nA .bak backup is created automatically.`)) return;
  const r = await fetch('/api/delete', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(buildFilterBody()),
  });
  const data = await r.json();
  if (data.error) { alert(`Error: ${data.error}`); return; }
  alert(`Deleted ${data.deleted} cells.\nBackup: ${data.backup}`);
  await reload();
});

// ── backup ─────────────────────────────────────────────────────────────────
document.getElementById('btn-backup').addEventListener('click', async () => {
  const data = await (await fetch('/api/backup', {method:'POST'})).json();
  alert(`Backup created:\n${data.backup}`);
});

document.getElementById('btn-reload').addEventListener('click', reload);

// ── export ─────────────────────────────────────────────────────────────────
document.getElementById('btn-exp-file').addEventListener('click', async () => {
  const dest = document.getElementById('exp-file').value.trim();
  if (!dest) { alert('Enter a destination path'); return; }
  const data = await (await fetch('/api/export/file', {
    method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({dest}),
  })).json();
  const el = document.getElementById('exp-status');
  el.style.color = data.ok ? '#7f7' : '#f88';
  el.textContent = data.ok
    ? `Copied (${(data.size/1048576).toFixed(2)} MB) → ${data.dest}`
    : `Error: ${data.error}`;
});

document.getElementById('btn-exp-s3').addEventListener('click', async () => {
  const body = {
    bucket:  document.getElementById('exp-bucket').value.trim(),
    key:     document.getElementById('exp-key').value.trim(),
    region:  document.getElementById('exp-region').value.trim(),
    profile: document.getElementById('exp-profile').value.trim(),
  };
  if (!body.bucket || !body.key) { alert('Enter bucket and S3 key'); return; }
  const el = document.getElementById('exp-status');
  el.style.color = '#aaa'; el.textContent = 'Uploading…';
  const data = await (await fetch('/api/export/s3', {
    method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body),
  })).json();
  el.style.color = data.ok ? '#7f7' : '#f88';
  el.textContent = data.ok
    ? `Uploaded (${(data.size/1048576).toFixed(2)} MB) → s3://${data.bucket}/${data.key}`
    : `Error: ${data.error}`;
});

// ── tabs ───────────────────────────────────────────────────────────────────
document.querySelectorAll('.tab').forEach(tab => {
  tab.addEventListener('click', () => {
    document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
    document.querySelectorAll('.tab-panel').forEach(p => p.classList.remove('active'));
    tab.classList.add('active');
    document.getElementById(`panel-${tab.dataset.panel}`).classList.add('active');
  });
});

// ── view settings ──────────────────────────────────────────────────────────
document.getElementById('age-scale').addEventListener('input', function() { this._touched = true; });
document.getElementById('btn-apply-view').addEventListener('click', () => {
  colorBy = document.getElementById('color-by').value;
  ageScaleDays = parseFloat(document.getElementById('age-scale').value) || 30;
  const zlo = parseFloat(document.getElementById('z-min').value);
  const zhi = parseFloat(document.getElementById('z-max').value);
  zFilterMin = isNaN(zlo) ? -Infinity : zlo;
  zFilterMax = isNaN(zhi) ?  Infinity : zhi;
  document.getElementById('color-mode').textContent = colorBy;
  // Update colorbar labels
  if (colorBy === 'age') {
    document.getElementById('color-label-lo').textContent = 'new';
    document.getElementById('color-label-hi').textContent = `${ageScaleDays}d+`;
    document.getElementById('colorbar-gradient').style.background =
      'linear-gradient(to right,#3ecf6e,#f0c330,#e8732a,#d63030)';
  } else {
    document.getElementById('color-label-lo').textContent = 'low';
    document.getElementById('color-label-hi').textContent = 'high';
    document.getElementById('colorbar-gradient').style.background =
      'linear-gradient(to right,#3ecf6e,#3090d0)';
  }
  buildScene();
});

// ── init ───────────────────────────────────────────────────────────────────
resize();
reload();
</script>
</body>
</html>
"""

# ─── main ─────────────────────────────────────────────────────────────────────

def main() -> int:
    global DB_PATH

    parser = argparse.ArgumentParser(
        description="Interactive browser editor for MissionLocalPlanningMap .db files."
    )
    parser.add_argument("db", help="Path to l2_map.db (or any SQLite L2 map file)")
    parser.add_argument("--port", type=int, default=8091, help="HTTP port (default: 8091)")
    parser.add_argument("--host", default="127.0.0.1", help="Bind host (default: 127.0.0.1)")
    args = parser.parse_args()

    DB_PATH = Path(args.db)
    if not DB_PATH.exists():
        print(f"error: db not found: {DB_PATH}", file=sys.stderr)
        return 1

    # Sanity-check: file must be a valid SQLite db with a cells table.
    try:
        with sqlite3.connect(str(DB_PATH)) as conn:
            conn.execute("SELECT COUNT(*) FROM cells").fetchone()
    except sqlite3.OperationalError as exc:
        print(f"error: cannot read cells table: {exc}", file=sys.stderr)
        return 1

    server = HTTPServer((args.host, args.port), Handler)
    url = f"http://{args.host}:{args.port}"
    print(f"L2 editor  →  {url}")
    print(f"Database   →  {DB_PATH.resolve()}")
    print("Press Ctrl-C to stop.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
