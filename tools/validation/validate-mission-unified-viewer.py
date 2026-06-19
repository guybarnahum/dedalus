#!/usr/bin/env python3
"""Validate generated mission_unified_viewer.html structural contracts.

Checks that tools/visualization/mission_unified_viewer.py produces an HTML
file that satisfies every required viewer contract:
  - All 5 SSE event types handled
  - Exterior voxel face rendering (traversability boundary-of-union surface)
  - Ghost detection rendering with velocity arrows
  - Mission event log panel
  - View presets wired
  - Layer toggles wired
  - /viewer_status polling for source label
  - No embedded snapshot data (pure SPA — all state from stream)
  - Correct draw call order

Usage:
  python3 tools/validation/validate-mission-unified-viewer.py <path/to/mission_unified_viewer.html>
"""

from __future__ import annotations

import argparse
from pathlib import Path


REQUIRED_SNIPPETS: dict[str, str] = {
    # ── SSE event listeners ─────────────────────────────────────────────────
    'SSE world_snapshot listener':              'addEventListener("world_snapshot"',
    'SSE mission_obstacle_map_delta listener':  'addEventListener("mission_obstacle_map_delta"',
    'SSE traversability_map_snapshot listener': 'addEventListener("traversability_map_snapshot"',
    'SSE traversability_map_delta listener':    'addEventListener("traversability_map_delta"',
    'applyTravDelta function':                  'function applyTravDelta(',
    'SSE ghost_detections listener':            'addEventListener("ghost_detections"',
    'SSE mission_event listener':               'addEventListener("mission_event"',

    # ── traversability exterior face rendering ──────────────────────────────
    'travOccupiedQKeys set':     'travOccupiedQKeys',
    'buildTravFaces function':   'function buildTravFaces()',
    'drawTravFaces function':    'function drawTravFaces()',
    'LOD coarse grid aggregation': 'coarseGrid',
    'LOD coarse occ key set':    'coarseOccKeys',
    'neighbor key lookup':       'coarseOccKeys.has(nk)',
    'face corners ±X':           'const fx = cx + dx * halfL;',
    'face corners ±Y':           'const fy = cy + dy * halfL;',
    'face corners ±Z':           'const fz = cz + dz * halfL;',
    'occupied base color':       '[190, 55, 45]',
    'partial base color':        '[200, 150, 30]',
    'face shading multiply':     'base[0] * s',
    'trav sort back-to-front':   'b.centroid.z - a.centroid.z',
    'obs sort back-to-front':    'b.center.z - a.center.z',
    'drawTravFaces called in draw': 'drawTravFaces()',
    # ── LOD slider ──────────────────────────────────────────────────────────
    'TRAV_LOD_LEVELS constant':  'const TRAV_LOD_LEVELS',
    'travDisplayLevelM state':   'travDisplayLevelM = 8',
    'LOD slider element':        'id="trav-lod"',
    'LOD slider wire-up':        'lodSlider.addEventListener',
    # ── zoom animation ──────────────────────────────────────────────────────
    'animateZoom function':      'window.animateZoom',
    'updateZoomDisplay function':'function updateZoomDisplay()',
    'zoom-val element':          'id="zoom-val"',

    # ── trav color mode toggle ───────────────────────────────────────────────
    'trav-color-btn element':    'id="trav-color-btn"',
    'trav-color-tag element':    'id="trav-color-tag"',
    'travColorByType state':     'travColorByType',
    'trav height legend':        'id="trav-legend-height"',
    'trav type legend':          'id="trav-legend-type"',
    'drawTravFaces byType branch':'byType',

    # ── hover card click-to-copy ─────────────────────────────────────────────
    'hover card click handler':  'hoverCard.addEventListener("click"',
    'hoverCellText function':    'function hoverCellText(',
    'clipboard write':           'navigator.clipboard',
    'copy flash element':        'copy-flash',

    # ── ghost detections ────────────────────────────────────────────────────
    'ghost detections state':       'ghostDetections',
    'applyGhostDetections function':'function applyGhostDetections(',
    'drawGhostDetections function': 'function drawGhostDetections()',
    'ghost velocity arrow':         'Velocity arrow',
    'ghost class color helper':     'function ghostClassColor(',
    'drawGhostDetections called':   'drawGhostDetections()',

    # ── mission event log ───────────────────────────────────────────────────
    'mission event log element': 'id="event-log"',
    'appendMissionEvent function':'function appendMissionEvent(',
    'applyMissionEvent function': 'function applyMissionEvent(',
    'mission event log scroll':   'logEl.scrollTop=logEl.scrollHeight',

    # ── layer toggles ───────────────────────────────────────────────────────
    'toggle-obstacles checkbox':  'id="toggle-obstacles"',
    'toggle-trav checkbox':       'id="toggle-trav"',
    'toggle-ghosts checkbox':     'id="toggle-ghosts"',
    'toggle-sensing checkbox':    'id="toggle-sensing"',
    'toggle-trajectory checkbox': 'id="toggle-trajectory"',

    # ── view presets ────────────────────────────────────────────────────────
    'view center preset': 'animateViewPreset(yaw, pitch, zoom, "center"',
    'view 45 preset':     'animateViewPreset(yaw + Math.PI/4, Math.PI/4',
    'view side preset':   'animateViewPreset(yaw + Math.PI/2, 0',
    'view top preset':    'animateViewPreset(yaw + Math.PI/2, Math.PI/2-0.01',

    # ── connection + status ─────────────────────────────────────────────────
    'EventSource connection':     'new EventSource(',
    'status-dot element':         'id="status-dot"',
    'viewer_status poll':         'fetch("/viewer_status")',
    'reconnect error handler':    'source.onerror',
    'connected flag':             'live.connected',

    # ── projection ──────────────────────────────────────────────────────────
    'project function':           'function project(',
    'currentViewCenter helper':   'function currentViewCenter()',
    'animateViewPreset global':   'window.animateViewPreset',
    'easeInOutCubic helper':      'window.easeInOutCubic',

    # ── draw pipeline ───────────────────────────────────────────────────────
    'drawOrientationGizmo called in draw': 'drawOrientationGizmo()',
    'drawObstacleCells called in draw': 'drawObstacleCells()',
    'drawDroneMarker called in draw':   'drawDroneMarker()',
    'drawSensingOverlays called in draw': 'drawSensingOverlays()',
    'drawTrajectory called in draw':    'drawTrajectory()',
    'drawLiveAgingOverlay called':      'drawLiveAgingOverlay()',
}

FORBIDDEN_SNIPPETS: dict[str, str] = {
    'embedded snapshot data blob':     '"cells": [',
    'Python format placeholder':       'data_json',
    'Python snapshot_path placeholder':'snapshot_path',
    'viewer debug log':                '[viewer]',
    'bare window.alert':               'window.alert(',
}


def validate_html(path: Path) -> list[str]:
    html = path.read_text(encoding="utf-8")
    failures: list[str] = []

    for label, snippet in REQUIRED_SNIPPETS.items():
        if snippet not in html:
            failures.append(f"missing {label!r}: {snippet!r}")

    for label, snippet in FORBIDDEN_SNIPPETS.items():
        if snippet in html:
            failures.append(f"forbidden {label!r}: {snippet!r}")

    # Structural checks
    script_start = html.find("<script>")
    script_end   = html.find("</script>")
    if script_start < 0 or script_end < 0:
        failures.append("missing <script>…</script> block")
        return failures

    js = html[script_start:script_end]

    # draw() must exist and call trav before obstacles (trav is background)
    draw_start = js.find("\nfunction draw()")
    if draw_start < 0:
        failures.append("missing draw() function")
    else:
        draw_body_end = js.find("\n}", draw_start)
        draw_body = js[draw_start: draw_body_end] if draw_body_end > 0 else js[draw_start:]
        trav_pos  = draw_body.find("drawTravFaces()")
        obs_pos   = draw_body.find("drawObstacleCells()")
        ghost_pos = draw_body.find("drawGhostDetections()")
        drone_pos = draw_body.find("drawDroneMarker()")
        if not (0 <= trav_pos < obs_pos):
            failures.append("draw() must call drawTravFaces() before drawObstacleCells()")
        if ghost_pos < 0:
            failures.append("draw() must call drawGhostDetections()")
        if drone_pos < 0:
            failures.append("draw() must call drawDroneMarker()")

    # buildTravFaces must check all 6 face directions
    for label, frag in [
        ("±X face check",  "dx !== 0"),
        ("±Y face check",  "dy !== 0"),
        ("±Z face (else)", "} else {"),
    ]:
        if frag not in js:
            failures.append(f"buildTravFaces missing {label}")

    # EventSource must handle all 5 event types by name
    for event_type in ["world_snapshot", "mission_obstacle_map_delta",
                        "traversability_map_snapshot", "traversability_map_delta",
                        "ghost_detections", "mission_event"]:
        if event_type not in js:
            failures.append(f"SSE event type not referenced in JS: {event_type!r}")

    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("html", type=Path,
                        help="Generated mission_unified_viewer.html to validate")
    args = parser.parse_args()

    if not args.html.exists():
        print(f"ERROR: file not found: {args.html}")
        return 1

    size_kb = args.html.stat().st_size / 1024
    failures = validate_html(args.html)

    if failures:
        for f in failures:
            print(f"ERROR: {f}")
        print(f"FAIL: {len(failures)} contract violation(s) in {args.html}")
        return 1

    print(f"OK: validated mission_unified_viewer.html ({size_kb:.0f} KB, 0 violations)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
