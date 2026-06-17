#!/usr/bin/env python3
"""Validate generated mission-local obstacle viewer HTML contracts.

This is a lightweight 4.3G guardrail for the browser viewer emitted by
tools/visualization/mission_local_obstacle_viewer.py. It validates the
operator-facing 4.3F viewer semantics without requiring a browser.
"""

from __future__ import annotations

import argparse
from pathlib import Path


REQUIRED_SNIPPETS: dict[str, str] = {
    "static center preset recenters": 'window.animateViewPreset(0, 0, 1.0, "center", recenterViewTarget());',
    "static 45 preset animates": 'window.animateViewPreset(-Math.PI / 4, Math.PI / 4, 1.0, "45");',
    "static side preset animates": 'window.animateViewPreset(Math.PI / 2, 0, 1.0, "side");',
    "static top preset animates": 'window.animateViewPreset(0, Math.PI / 2 - 0.01, 1.0, "top");',
    "animated view center state": "let viewCenter = null;",
    "current view center helper": "function currentViewCenter()",
    "manual recenter target helper": "function recenterViewTarget()",
    "projection uses animated center": "const c = currentViewCenter();",
    "view center interpolates": "viewCenter = interpolatePoint(startCenter, finalCenter, t);",
    "view center finalizes": "viewCenter = clonePoint(finalCenter);",
    "velocity vector helper": "function droneVelocityDirection()",
    "sensing vector helper": "function drawSensingOverlays()",
    "final draw uses sensing overlay": "drawSensingOverlays();",
    "final draw uses drone marker": "drawDroneMarker();",
    "velocity legend": "Short white vector: drone velocity direction",
    "sensing legend": "Long white vector: sensing/camera direction",
}

FORBIDDEN_SNIPPETS: dict[str, str] = {
    "temporary viewer debug logs": "[viewer]",
    "dynamic view preset override": "installViewPresetButtonOverrides",
    "dynamic override state": "viewPresetButtonsBound",
    "bare animateViewPreset call": " animateViewPreset(",
}


def validate_html(path: Path) -> list[str]:
    html = path.read_text(encoding="utf-8")
    failures: list[str] = []

    for label, snippet in REQUIRED_SNIPPETS.items():
        if snippet not in html:
            failures.append(f"missing {label}: {snippet}")

    for label, snippet in FORBIDDEN_SNIPPETS.items():
        if snippet in html:
            failures.append(f"forbidden {label}: {snippet}")

    # Ensure the final wrapper order remains evidence/live first, then vectors.
    base = html.find("baseDraw();")
    live = html.find("drawLiveAgingOverlay();")
    sensing = html.find("drawSensingOverlays();")
    marker = html.find("drawDroneMarker();")
    if not (0 <= base < live < sensing < marker):
        failures.append(
            "final draw order is not baseDraw -> drawLiveAgingOverlay -> "
            "drawSensingOverlays -> drawDroneMarker"
        )

    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("html", type=Path, help="Generated mission_local_obstacle_viewer.html")
    args = parser.parse_args()

    if not args.html.exists():
        print(f"ERROR: missing viewer HTML: {args.html}")
        return 1

    failures = validate_html(args.html)
    if failures:
        for failure in failures:
            print(f"ERROR: {failure}")
        return 1

    print(f"OK: validated mission-local obstacle viewer HTML: {args.html}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
