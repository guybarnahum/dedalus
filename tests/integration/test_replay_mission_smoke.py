#!/usr/bin/env python3
from __future__ import annotations

import json
import shutil
import subprocess
import sys
from pathlib import Path


def main() -> int:
    repo_root = Path(sys.argv[1])
    build_dir = Path(sys.argv[2])
    app = build_dir / "apps" / "dedalus_mission_loop"
    out_dir = repo_root / "out" / "test_mission_loop_smoke"
    if out_dir.exists():
        shutil.rmtree(out_dir)
    cmd = [
        str(app),
        "--config", str(repo_root / "config" / "core_stack_mission_ci.yaml"),
        "--output-dir", str(out_dir),
        "--max-frames", "1",
        "--no-progress",
    ]
    result = subprocess.run(cmd, cwd=repo_root, text=True, capture_output=True, check=False)
    if result.returncode != 0:
        print(result.stdout)
        print(result.stderr, file=sys.stderr)
        return result.returncode
    if "Mission runtime: trajectory_mission" not in result.stdout:
        print("missing mission startup line", file=sys.stderr)
        return 1
    if "Mission summary:" not in result.stdout:
        print("missing mission summary", file=sys.stderr)
        return 1
    if "Mission events:" not in result.stdout:
        print("missing mission events artifact line", file=sys.stderr)
        return 1
    events_path = out_dir / "mission_events.jsonl"
    if not events_path.exists():
        print("missing mission_events.jsonl", file=sys.stderr)
        return 1
    events_text = events_path.read_text(encoding="utf-8")
    if '"event":"runtime_start"' not in events_text:
        print("missing runtime_start event", file=sys.stderr)
        return 1
    if '"event":"state_transition"' not in events_text:
        print("missing state_transition event", file=sys.stderr)
        return 1
    snapshots = sorted(out_dir.glob("snapshot_*.json"))
    if len(snapshots) != 1:
        print(f"expected 1 synthetic snapshot, got {len(snapshots)}", file=sys.stderr)
        return 1
    latest = json.loads(snapshots[-1].read_text(encoding="utf-8"))
    ego = latest.get("ego", {})
    if not ego.get("height_valid") or ego.get("height_m", 0) <= 0:
        print("missing mission-ready ego state", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())