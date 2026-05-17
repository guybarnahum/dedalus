#!/usr/bin/env python3
"""Smoke tests for simulation/validate-mission-artifacts.py."""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
from pathlib import Path


def write_jsonl(path: Path, records: list[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8") as output:
        for record in records:
            output.write(json.dumps(record, separators=(",", ":")) + "\n")


def valid_records() -> list[dict[str, object]]:
    return [
        {"event": "runtime_start", "tick_hz": 10.0},
        {"event": "state_transition", "tick": 1, "from": "Idle", "to": "Prepare", "status": "arming", "ego_height_m": 0.0},
        {"event": "command_dispatch", "tick": 1, "state": "Prepare", "command": "Arm"},
        {"event": "command_result", "tick": 1, "state": "Prepare", "command": "Arm", "success": True},
        {"event": "state_transition", "tick": 2, "from": "Prepare", "to": "Takeoff", "status": "armed", "ego_height_m": 0.1},
        {"event": "command_dispatch", "tick": 2, "state": "Takeoff", "command": "Takeoff"},
        {"event": "command_result", "tick": 2, "state": "Takeoff", "command": "Takeoff", "success": True},
        {"event": "state_transition", "tick": 42, "from": "Takeoff", "to": "ExecuteMission", "status": "takeoff_complete", "ego_height_m": 16.2},
        {"event": "command_dispatch", "tick": 43, "state": "ExecuteMission", "command": "Velocity"},
        {"event": "command_result", "tick": 43, "state": "ExecuteMission", "command": "Velocity", "success": True},
        {"event": "state_transition", "tick": 70, "from": "ExecuteMission", "to": "GoHome", "status": "trajectory_complete", "ego_height_m": 16.0},
        {"event": "state_transition", "tick": 90, "from": "GoHome", "to": "Land", "status": "home_reached", "ego_height_m": 15.8},
        {"event": "command_dispatch", "tick": 91, "state": "Land", "command": "Land"},
        {"event": "command_result", "tick": 91, "state": "Land", "command": "Land", "success": True},
        {"event": "state_transition", "tick": 120, "from": "Land", "to": "Complete", "status": "landed", "ego_height_m": 0.1},
        {"event": "command_dispatch", "tick": 121, "state": "Complete", "command": "Disarm"},
        {"event": "command_result", "tick": 121, "state": "Complete", "command": "Disarm", "success": True},
        {"event": "runtime_stop", "tick_count": 122, "state": "Complete"},
    ]


def run_validator(repo_root: Path, run_dir: Path, *args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(repo_root / "simulation" / "validate-mission-artifacts.py"), str(run_dir), *args],
        cwd=repo_root,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )


def main() -> int:
    repo_root = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[2]

    with tempfile.TemporaryDirectory() as tmp:
        run_dir = Path(tmp) / "valid_run"
        run_dir.mkdir()
        write_jsonl(run_dir / "mission_events.jsonl", valid_records())
        (run_dir / "snapshot_manifest.txt").write_text("snapshot_0001.json\n", encoding="utf-8")
        (run_dir / "snapshot_0001.json").write_text("{}\n", encoding="utf-8")

        ok = run_validator(repo_root, run_dir, "--expect-complete", "--safe-height-m", "16", "--landed-height-m", "0.5")
        if ok.returncode != 0:
            print(ok.stdout)
            print(ok.stderr, file=sys.stderr)
            return 1
        if "final_state: Complete" not in ok.stdout or "failures: 0" not in ok.stdout:
            print(ok.stdout)
            print("validator did not report expected success summary", file=sys.stderr)
            return 1

        low_height_dir = Path(tmp) / "low_height_run"
        low_height_dir.mkdir()
        records = valid_records()
        for record in records:
            if record.get("event") == "state_transition" and record.get("to") == "ExecuteMission":
                record["ego_height_m"] = 2.0
        write_jsonl(low_height_dir / "mission_events.jsonl", records)
        (low_height_dir / "snapshot_manifest.txt").write_text("snapshot_0001.json\n", encoding="utf-8")
        low_height = run_validator(low_height_dir=low_height_dir, repo_root=repo_root, *["--expect-complete", "--safe-height-m", "16"])
        if low_height.returncode == 0:
            print(low_height.stdout)
            print("validator accepted ExecuteMission below safe height", file=sys.stderr)
            return 1
        if "ExecuteMission reached below safe height" not in low_height.stdout:
            print(low_height.stdout)
            print("validator did not explain safe-height failure", file=sys.stderr)
            return 1

        behavior_missing = run_validator(repo_root, run_dir, "--expect-complete", "--expect-behavior")
        if behavior_missing.returncode == 0:
            print(behavior_missing.stdout)
            print("validator accepted missing object behavior events", file=sys.stderr)
            return 1
        if "target_selected" not in behavior_missing.stdout or "behavior_start" not in behavior_missing.stdout:
            print(behavior_missing.stdout)
            print("validator did not report missing behavior extension events", file=sys.stderr)
            return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
