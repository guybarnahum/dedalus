#!/usr/bin/env python3
"""Smoke-test dedalus_replay_recording artifact generation."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: test_replay_recording_smoke.py <repo-root> <build-dir>", file=sys.stderr)
        return 2

    repo_root = Path(sys.argv[1]).resolve()
    build_dir = Path(sys.argv[2]).resolve()
    app = build_dir / "apps" / "dedalus_replay_recording"
    output_dir = build_dir / "replay-recording-smoke"

    cmd = [
        str(app),
        "--config",
        "config/ci/core_stack_recorded_ci.yaml",
        "--output-dir",
        str(output_dir),
        "--max-frames",
        "1",
    ]
    subprocess.run(cmd, cwd=repo_root, check=True)

    manifest = output_dir / "snapshot_manifest.txt"
    snapshot = output_dir / "snapshot_0001.json"
    if not manifest.exists():
        raise AssertionError(f"missing manifest: {manifest}")
    if not snapshot.exists():
        raise AssertionError(f"missing snapshot: {snapshot}")

    data = json.loads(snapshot.read_text(encoding="utf-8"))
    if data.get("active_map_frame_id") != "map_recorded_ci_0001":
        raise AssertionError("snapshot did not preserve recorded fallback map frame")
    if not data.get("agents"):
        raise AssertionError("snapshot missing agents")
    if not data.get("flight_corridors"):
        raise AssertionError("snapshot missing flight corridors")

    manifest_text = manifest.read_text(encoding="utf-8")
    if "snapshot_0001.json" not in manifest_text:
        raise AssertionError("manifest missing snapshot entry")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
