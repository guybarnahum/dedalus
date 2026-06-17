#!/usr/bin/env python3
"""Validate generated foundational traversability-map viewer HTML."""

from __future__ import annotations

import argparse
from pathlib import Path


REQUIRED_SNIPPETS: dict[str, str] = {
    "title": "Dedalus foundational traversability map",
    "offline/final layer name": "Foundational traversability map",
    "planner boundary note": "not a command sink and not a reflexive safety dependency",
    "artifact polling": "async function pollArtifactOnce()",
    "poll interval": "window.setInterval(pollArtifactOnce, 1500);",
    "remote normalizer": "function normalizeRemoteArtifact(artifact)",
    "occupied toggle": 'id="show-occupied"',
    "free toggle": 'id="show-free"',
    "stale toggle": 'id="show-stale"',
    "low-clearance toggle": 'id="show-low-clearance"',
    "overhead-risk toggle": 'id="show-overhead"',
    "cost-color helper": "function costColor(cell",
    "hover cell helper": "function hoverHtml(cell)",
    "low-clearance ring": "cell.low_clearance",
    "overhead-risk ring": "cell.overhead_risk",
}

FORBIDDEN_SNIPPETS: dict[str, str] = {
    "control coupling language": "command sink dependency",
    "planner authority language": "planner authority",
    "browser console debug marker": "[viewer]",
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

    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("html", type=Path, help="Generated mission_traversability_map_viewer.html")
    args = parser.parse_args()

    if not args.html.exists():
        print(f"ERROR: missing traversability viewer HTML: {args.html}")
        return 1

    failures = validate_html(args.html)
    if failures:
        for failure in failures:
            print(f"ERROR: {failure}")
        return 1

    print(f"OK: validated mission traversability map viewer HTML: {args.html}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
