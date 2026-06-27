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
                "age_s": (now_ns - updated_ns) / 1e9 if updated_ns else 0,
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
#sidebar{width:280px;min-width:280px;background:#1a1a1a;border-right:1px solid #333;display:flex;flex-direction:column;overflow:hidden}
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

/* ── buttons ── */
button{border:none;border-radius:4px;padding:7px 10px;font-size:11px;cursor:pointer;font-weight:500;width:100%;margin-top:8px;transition:opacity .15s}
button:disabled{opacity:.4;cursor:default}
button:hover:not(:disabled){opacity:.85}
.btn-preview{background:#2a3a50;color:#7bf}
.btn-delete{background:#501a1a;color:#f88}
.btn-safe{background:#1a3020;color:#7f7}
.btn-export{background:#252535;color:#99f}
#preview-count{font-size:11px;color:#fa0;margin-top:6px;min-height:16px}

/* ── colorbar ── */
#colorbar{display:flex;align-items:center;gap:6px;padding:6px 12px 4px;font-size:10px;color:#555;border-bottom:1px solid #282828}
#colorbar-gradient{height:8px;flex:1;border-radius:3px;background:linear-gradient(to right,#2ecc71,#f1c40f,#e67e22,#e74c3c)}
#color-mode{font-size:10px;padding:2px 6px;background:#1a1a1a;border:1px solid #333;border-radius:3px;color:#aaa;cursor:pointer}

/* ── canvas ── */
#canvas-wrap{flex:1;position:relative;overflow:hidden;background:#0d0d0d}
canvas{display:block;width:100%;height:100%}
#hud{position:absolute;top:8px;right:10px;font-size:10px;color:#444;pointer-events:none;line-height:1.8}
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
    <label>Max age (hours) — delete cells older than this</label>
    <input type="number" id="f-age-h" placeholder="e.g. 168 (7 days)" min="0" step="1">
    <div class="hint">Leave blank to ignore</div>

    <label>Delete before</label>
    <input type="datetime-local" id="f-before">

    <label>Delete after</label>
    <input type="datetime-local" id="f-after">

    <label>Max score (delete cells with score &lt; this)</label>
    <input type="number" id="f-score" placeholder="e.g. 2.0" min="0" step="0.1">

    <label>Max confidence (delete cells with conf &lt; this)</label>
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

    <button class="btn-preview" id="btn-preview">Preview deletion</button>
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
      <option value="age">Age (green=new, red=old)</option>
      <option value="score">Score</option>
      <option value="confidence">Confidence</option>
    </select>
    <label>Max age scale (days)</label>
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
      Reset: double-click canvas
    </div>
  </div>
</div>

<!-- ── colorbar strip ── -->
<div id="canvas-wrap">
  <div id="colorbar">
    <span>new</span>
    <div id="colorbar-gradient"></div>
    <span>old</span>
    <span id="color-mode" title="colour mode">age</span>
  </div>
  <canvas id="c"></canvas>
  <div id="hud">
    Cells: <span id="h-total">–</span><br>
    Highlighted: <span id="h-hi">0</span>
  </div>
  <div id="loading">Loading cells…</div>
</div>

<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
<script>
'use strict';

// ── state ──────────────────────────────────────────────────────────────────
let allCells = [];
let highlightedKeys = new Set();  // "xi,yi,zi" strings
let colorBy = 'age';
let ageScaleDays = 30;
let zFilterMin = -Infinity, zFilterMax = Infinity;
let cellSizeM = 1.0, vCellSizeM = 2.0;
let stats = {};

// ── Three.js setup ─────────────────────────────────────────────────────────
const canvas = document.getElementById('c');
const renderer = new THREE.WebGLRenderer({canvas, antialias: true});
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setClearColor(0x0d0d0d);

// Orthographic camera — true 45° isometric angle
const aspect = () => canvas.clientWidth / canvas.clientHeight;
let frustumSize = 80;
function makeCamera() {
  const a = aspect();
  const cam = new THREE.OrthographicCamera(
    -frustumSize * a / 2,  frustumSize * a / 2,
     frustumSize / 2,     -frustumSize / 2,
    -2000, 2000
  );
  // Isometric: equal angles on all three axes
  cam.position.set(1, 1, 1).normalize().multiplyScalar(frustumSize * 2);
  cam.lookAt(0, 0, 0);
  return cam;
}
let camera = makeCamera();

const scene = new THREE.Scene();
scene.add(new THREE.AmbientLight(0xffffff, 0.55));
const dirLight = new THREE.DirectionalLight(0xffffff, 0.7);
dirLight.position.set(3, 5, 2);
scene.add(dirLight);

// Target (pan offset) — camera orbits this point
let target = new THREE.Vector3(0, 0, 0);

// ── geometry ───────────────────────────────────────────────────────────────
const GEO = new THREE.BoxGeometry(1, 1, 1);
let mesh = null;
let highlightMesh = null;

// ── colour helpers ─────────────────────────────────────────────────────────
const COL = new THREE.Color();
function colorForCell(c) {
  if (colorBy === 'age') {
    const t = Math.min(c.age_s / (ageScaleDays * 86400), 1.0);
    // green → yellow → orange → red
    if (t < 0.33) COL.setHSL(0.38 - t * 0.38 / 0.33, 0.85, 0.42);
    else if (t < 0.66) { const u = (t - 0.33) / 0.33; COL.setHSL(0.08 - u * 0.05, 0.9, 0.43); }
    else { const u = (t - 0.66) / 0.34; COL.setHSL(0.03 - u * 0.03, 1.0, 0.40 - u * 0.05); }
  } else if (colorBy === 'score') {
    const mx = stats.score_max || 10;
    const t = Math.min(c.score / mx, 1.0);
    COL.setHSL(0.62 - t * 0.6, 0.9, 0.42);
  } else {
    const t = Math.min(Math.max(c.confidence, 0), 1);
    COL.setHSL(0.62 - t * 0.6, 0.9, 0.42);
  }
  return COL.clone();
}

// ── build scene ────────────────────────────────────────────────────────────
function buildScene() {
  if (mesh) { scene.remove(mesh); mesh.dispose(); mesh = null; }
  if (highlightMesh) { scene.remove(highlightMesh); highlightMesh.dispose(); highlightMesh = null; }

  const visible = allCells.filter(c =>
    c.cz >= zFilterMin && c.cz <= zFilterMax
  );
  const hiCells  = visible.filter(c => highlightedKeys.has(`${c.xi},${c.yi},${c.zi}`));
  const normCells = visible.filter(c => !highlightedKeys.has(`${c.xi},${c.yi},${c.zi}`));

  // Normal cells
  if (normCells.length > 0) {
    const mat = new THREE.MeshLambertMaterial({vertexColors: true});
    mesh = new THREE.InstancedMesh(GEO, mat, normCells.length);
    mesh.instanceMatrix.setUsage(THREE.StaticDrawUsage);
    const M = new THREE.Matrix4();
    const S = new THREE.Vector3(cellSizeM, cellSizeM, vCellSizeM);
    normCells.forEach((c, i) => {
      M.makeScale(S.x, S.y, S.z);
      M.setPosition(c.cx, c.cz, -c.cy);  // Y-up: map Z→Three Y, map Y→Three -Z
      mesh.setMatrixAt(i, M);
      mesh.setColorAt(i, colorForCell(c));
    });
    mesh.instanceMatrix.needsUpdate = true;
    mesh.instanceColor.needsUpdate = true;
    scene.add(mesh);
  }

  // Highlighted (preview-delete) cells — red/orange
  if (hiCells.length > 0) {
    const hmat = new THREE.MeshLambertMaterial({vertexColors: true, transparent: true, opacity: 0.85});
    highlightMesh = new THREE.InstancedMesh(GEO, hmat, hiCells.length);
    const M = new THREE.Matrix4();
    const HI = new THREE.Color(1.0, 0.25, 0.1);
    hiCells.forEach((c, i) => {
      M.makeScale(cellSizeM * 1.05, vCellSizeM * 1.05, cellSizeM * 1.05);
      M.setPosition(c.cx, c.cz, -c.cy);
      highlightMesh.setMatrixAt(i, M);
      highlightMesh.setColorAt(i, HI);
    });
    highlightMesh.instanceMatrix.needsUpdate = true;
    highlightMesh.instanceColor.needsUpdate = true;
    scene.add(highlightMesh);
  }

  document.getElementById('h-total').textContent = visible.length;
  document.getElementById('h-hi').textContent = hiCells.length;
}

// ── centre camera on loaded data ───────────────────────────────────────────
function fitCamera() {
  if (allCells.length === 0) return;
  let mx = -Infinity, mnx = Infinity, my = -Infinity, mny = Infinity;
  allCells.forEach(c => {
    if (c.cx > mx) mx = c.cx; if (c.cx < mnx) mnx = c.cx;
    if (c.cy > my) my = c.cy; if (c.cy < mny) mny = c.cy;
  });
  target.set((mx + mnx) / 2, 0, -(my + mny) / 2);
  frustumSize = Math.max(mx - mnx, my - mny, 30) * 1.3;
  rebuildCamera();
}

function rebuildCamera() {
  const a = aspect();
  camera.left   = -frustumSize * a / 2;
  camera.right  =  frustumSize * a / 2;
  camera.top    =  frustumSize / 2;
  camera.bottom = -frustumSize / 2;
  camera.position.copy(target).addScaledVector(
    new THREE.Vector3(1, 1, 1).normalize(), frustumSize * 2
  );
  camera.lookAt(target);
  camera.updateProjectionMatrix();
}

// ── render loop ────────────────────────────────────────────────────────────
function resize() {
  const w = canvas.clientWidth, h = canvas.clientHeight;
  renderer.setSize(w, h, false);
  rebuildCamera();
}
window.addEventListener('resize', resize);

(function animate() {
  requestAnimationFrame(animate);
  renderer.render(scene, camera);
})();

// ── pan / zoom ─────────────────────────────────────────────────────────────
const RIGHT = new THREE.Vector3(-1, 0, 1).normalize();  // isometric screen-X
const UP    = new THREE.Vector3(-1, 2, -1).normalize(); // isometric screen-Y

let dragBtn = -1;
let lastMX = 0, lastMY = 0;

canvas.addEventListener('contextmenu', e => e.preventDefault());
canvas.addEventListener('mousedown', e => {
  if (e.button === 1 || e.button === 2) { dragBtn = e.button; lastMX = e.clientX; lastMY = e.clientY; }
});
window.addEventListener('mouseup', () => { dragBtn = -1; });
window.addEventListener('mousemove', e => {
  if (dragBtn < 0) return;
  const dx = e.clientX - lastMX, dy = e.clientY - lastMY;
  lastMX = e.clientX; lastMY = e.clientY;
  const scale = frustumSize / Math.min(canvas.clientWidth, canvas.clientHeight);
  target.addScaledVector(RIGHT, -dx * scale);
  target.addScaledVector(UP,     dy * scale);
  rebuildCamera();
});
canvas.addEventListener('wheel', e => {
  e.preventDefault();
  frustumSize *= e.deltaY > 0 ? 1.12 : 0.89;
  frustumSize = Math.max(5, Math.min(frustumSize, 2000));
  rebuildCamera();
}, {passive: false});
canvas.addEventListener('dblclick', () => { fitCamera(); });

// ── data load ──────────────────────────────────────────────────────────────
async function loadStats() {
  const r = await fetch('/api/stats');
  stats = await r.json();
  document.getElementById('db-path').textContent = stats.db_path || '';
  document.getElementById('s-total').textContent = stats.total_cells ?? '–';
  const scoreStr = stats.score_min != null
    ? `${stats.score_min?.toFixed(1)}–${stats.score_max?.toFixed(1)}` : '–';
  document.getElementById('s-score').textContent = scoreStr;
  if (stats.ts_min_ns && stats.ts_max_ns) {
    const ageMaxDays = ((stats.now_ns - stats.ts_min_ns) / 1e9 / 86400).toFixed(1);
    document.getElementById('s-age').textContent = `0–${ageMaxDays}d`;
  }
}

async function loadCells() {
  document.getElementById('loading').style.display = 'flex';
  const r = await fetch('/api/cells');
  const data = await r.json();
  allCells = data.cells || [];
  cellSizeM  = data.params?.cell_size_m  ?? 1.0;
  vCellSizeM = data.params?.vertical_cell_size_m ?? 2.0;
  document.getElementById('loading').style.display = 'none';
  highlightedKeys.clear();
  buildScene();
  fitCamera();
}

async function reload() {
  await loadStats();
  await loadCells();
}

// ── filter helpers ─────────────────────────────────────────────────────────
function buildFilterBody() {
  const body = {};
  const ageH = parseFloat(document.getElementById('f-age-h').value);
  if (!isNaN(ageH) && ageH >= 0) body.max_age_s = ageH * 3600;

  const before = document.getElementById('f-before').value;
  if (before) body.before_ns = new Date(before).getTime() * 1e6;

  const after = document.getElementById('f-after').value;
  if (after) body.after_ns = new Date(after).getTime() * 1e6;

  const score = parseFloat(document.getElementById('f-score').value);
  if (!isNaN(score)) body.max_score = score;

  const conf = parseFloat(document.getElementById('f-conf').value);
  if (!isNaN(conf)) body.max_confidence = conf;

  for (const ax of ['cx', 'cy', 'cz']) {
    const lo = parseFloat(document.getElementById(`f-${ax}-min`).value);
    const hi = parseFloat(document.getElementById(`f-${ax}-max`).value);
    if (!isNaN(lo)) body[`${ax}_min`] = lo;
    if (!isNaN(hi)) body[`${ax}_max`] = hi;
  }
  return body;
}

// ── preview delete ─────────────────────────────────────────────────────────
document.getElementById('btn-preview').addEventListener('click', async () => {
  const body = buildFilterBody();
  const r = await fetch('/api/preview_delete', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)});
  const data = await r.json();
  if (data.error) { document.getElementById('preview-count').textContent = `⚠ ${data.error}`; return; }
  const n = data.would_delete;
  document.getElementById('preview-count').textContent = `${n} cells match`;
  document.getElementById('del-label').textContent = n;
  document.getElementById('btn-delete').disabled = (n === 0);
  highlightedKeys = new Set((data.sample || []).map(c => `${c.xi},${c.yi},${c.zi}`));
  buildScene();
});

// ── delete ─────────────────────────────────────────────────────────────────
document.getElementById('btn-delete').addEventListener('click', async () => {
  const n = parseInt(document.getElementById('del-label').textContent);
  if (!confirm(`Delete ${n} cells from the L2 map?\n\nA .bak backup will be created automatically.`)) return;
  const body = buildFilterBody();
  const r = await fetch('/api/delete', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)});
  const data = await r.json();
  if (data.error) { alert(`Error: ${data.error}`); return; }
  alert(`Deleted ${data.deleted} cells.\nBackup: ${data.backup}`);
  document.getElementById('btn-delete').disabled = true;
  document.getElementById('preview-count').textContent = '';
  await reload();
});

// ── backup ─────────────────────────────────────────────────────────────────
document.getElementById('btn-backup').addEventListener('click', async () => {
  const r = await fetch('/api/backup', {method:'POST'});
  const data = await r.json();
  alert(`Backup created:\n${data.backup}`);
});

// ── reload ─────────────────────────────────────────────────────────────────
document.getElementById('btn-reload').addEventListener('click', reload);

// ── export file ────────────────────────────────────────────────────────────
document.getElementById('btn-exp-file').addEventListener('click', async () => {
  const dest = document.getElementById('exp-file').value.trim();
  if (!dest) { alert('Enter a destination path'); return; }
  const r = await fetch('/api/export/file', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({dest})});
  const data = await r.json();
  const el = document.getElementById('exp-status');
  el.style.color = data.ok ? '#7f7' : '#f88';
  el.textContent = data.ok ? `Copied (${(data.size/1024/1024).toFixed(2)} MB) → ${data.dest}` : `Error: ${data.error}`;
});

// ── export S3 ──────────────────────────────────────────────────────────────
document.getElementById('btn-exp-s3').addEventListener('click', async () => {
  const body = {
    bucket:  document.getElementById('exp-bucket').value.trim(),
    key:     document.getElementById('exp-key').value.trim(),
    region:  document.getElementById('exp-region').value.trim(),
    profile: document.getElementById('exp-profile').value.trim(),
  };
  if (!body.bucket || !body.key) { alert('Enter bucket and S3 key'); return; }
  const el = document.getElementById('exp-status');
  el.style.color = '#aaa';
  el.textContent = 'Uploading…';
  const r = await fetch('/api/export/s3', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)});
  const data = await r.json();
  el.style.color = data.ok ? '#7f7' : '#f88';
  el.textContent = data.ok
    ? `Uploaded (${(data.size/1024/1024).toFixed(2)} MB) → s3://${data.bucket}/${data.key}`
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

// ── view apply ─────────────────────────────────────────────────────────────
document.getElementById('btn-apply-view').addEventListener('click', () => {
  colorBy = document.getElementById('color-by').value;
  ageScaleDays = parseFloat(document.getElementById('age-scale').value) || 30;
  const zlo = parseFloat(document.getElementById('z-min').value);
  const zhi = parseFloat(document.getElementById('z-max').value);
  zFilterMin = isNaN(zlo) ? -Infinity : zlo;
  zFilterMax = isNaN(zhi) ?  Infinity : zhi;
  document.getElementById('color-mode').textContent = colorBy;
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
