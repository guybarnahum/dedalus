#!/usr/bin/env python3
"""Check repo text for planning-label names in durable artifacts.

This checker is intentionally small and conservative. It exists to prevent new
repo files, validators, scripts, docs, or commands from being named after
planning labels such as track numbers or milestone placeholders instead of the
stable architectural capability they implement.
"""

from __future__ import annotations

import argparse
import pathlib
import re
import sys


DEFAULT_ROOTS = (
    "HANDOFF.md",
    "LLM.md",
    "tools",
    "tests",
    "docs",
    "simulation/airsim/scripts",
)

BINARY_SUFFIXES = {
    ".png",
    ".jpg",
    ".jpeg",
    ".gif",
    ".mp4",
    ".mov",
    ".bin",
    ".onnx",
    ".pt",
    ".pth",
    ".zip",
    ".gz",
}

FORBIDDEN_EXAMPLE_LINES = {
    "track4",
    "milestone_XXX",
    "phase_YYY",
    "latest_run",
    "mission_YYYY",
    "foo.json",
    "temp.json",
    "ad-hoc simulation/artifacts/mission_* unless that is the actual architectural path produced by the repo",
}

# Planning labels that should not be used as durable repo artifact names or
# operator-facing labels. Historical milestone prose is outside this check; this
# checker guards durable artifact names and operator-facing debug labels.
PLANNING_LABEL_PATTERNS = (
    re.compile(r"\bvalidate-track\d+[-_]", re.IGNORECASE),
    re.compile(r"\btrack\d+\s+volumes=", re.IGNORECASE),
    re.compile(r"\bTRACK\d+\s+snapshot\s+dir", re.IGNORECASE),
    re.compile(r"\bmilestone[_-]x{2,}\b", re.IGNORECASE),
    re.compile(r"\bphase[_-]y{2,}\b", re.IGNORECASE),
)

ALLOWLIST_PATTERNS = (
    # Documentation can explicitly describe the convention and forbidden examples.
    re.compile(r"Avoid names based on planning labels", re.IGNORECASE),
    re.compile(r"planning labels such as", re.IGNORECASE),
    re.compile(r"do not name .* planning labels", re.IGNORECASE),
)


def iter_files(repo_root: pathlib.Path, roots: list[str]) -> list[pathlib.Path]:
    files: list[pathlib.Path] = []
    for root_text in roots:
        root = repo_root / root_text
        if not root.exists():
            continue
        if root.is_file():
            files.append(root)
            continue
        for path in root.rglob("*"):
            if path.is_file():
                files.append(path)
    return sorted(set(files))


def should_skip(path: pathlib.Path) -> bool:
    if path.suffix.lower() in BINARY_SUFFIXES:
        return True
    parts = set(path.parts)
    return any(part in parts for part in {".git", "build", "build-staging", "third_party", "out"})


def is_allowlisted(line: str) -> bool:
    stripped = line.strip()
    if stripped in FORBIDDEN_EXAMPLE_LINES:
        return True
    return any(pattern.search(line) for pattern in ALLOWLIST_PATTERNS)


def check_file(repo_root: pathlib.Path, path: pathlib.Path) -> list[str]:
    if should_skip(path):
        return []
    try:
        text = path.read_text(encoding="utf-8")
    except UnicodeDecodeError:
        return []
    failures: list[str] = []
    relative = path.relative_to(repo_root)
    for line_number, line in enumerate(text.splitlines(), start=1):
        if is_allowlisted(line):
            continue
        for pattern in PLANNING_LABEL_PATTERNS:
            if pattern.search(line):
                failures.append(f"{relative}:{line_number}: planning-label name: {line.strip()}")
    return failures


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("repo_root", nargs="?", default=".")
    parser.add_argument(
        "paths",
        nargs="*",
        help="Optional files/directories to check. Defaults to docs, tools, tests, and AirSim scripts.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    repo_root = pathlib.Path(args.repo_root).resolve()
    roots = args.paths or list(DEFAULT_ROOTS)
    failures: list[str] = []
    for path in iter_files(repo_root, roots):
        failures.extend(check_file(repo_root, path))
    if failures:
        print("architectural naming check failed:")
        for failure in failures:
            print(f"  {failure}")
        return 1
    print("architectural naming check passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
