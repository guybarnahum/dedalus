#!/usr/bin/env python3
"""Validate that an expected Abort scenario is first-class and archive-grade."""

from __future__ import annotations

import json
import shutil
import subprocess
import sys
from pathlib import Path


def main() -> int:
    repo_root = Path(sys.argv[1])
    build_dir = Path(sys.argv[2])
    out_root = repo_root / "out" / "test_mission_abort_scenarios"
    if out_root.exists():
        shutil.rmtree(out_root)

    command = [
        sys.executable,
        str(repo_root / "tools" / "mission" / "run-mission-scenario.py"),
        "--name",
        "abort_land_timeout",
        "--run-id",
        "run_0001",
        "--config",
        str(repo_root / "config" / "core_stack_synthetic_mission_abort_ci.yaml"),
        "--app",
        str(build_dir / "apps" / "dedalus_mission_loop"),
        "--output-root",
        str(out_root),
        "--max-frames",
        "220",
        "--shutdown-max-frames",
        "50",
        "--safe-height-m",
        "2",
        "--landed-height-m",
        "1",
        "--expect-final-state",
        "Abort",
        "--overwrite",
    ]
    result = subprocess.run(command, cwd=repo_root, text=True, capture_output=True, check=False)
    if result.returncode != 0:
        print(result.stdout)
        print(result.stderr, file=sys.stderr)
        return result.returncode

    run_dir = out_root / "abort_land_timeout" / "run_0001"
    metadata_path = run_dir / "metadata.json"
    validator_path = run_dir / "validator_result.txt"
    events_path = run_dir / "mission_events.jsonl"

    for path in [metadata_path, validator_path, events_path]:
        if not path.exists():
            print(f"missing abort scenario artifact: {path}", file=sys.stderr)
            return 1

    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    if metadata.get("status") != "passed" or metadata.get("expect_final_state") != "Abort":
        print(json.dumps(metadata, indent=2), file=sys.stderr)
        print("abort scenario metadata did not report expected passed Abort run", file=sys.stderr)
        return 1

    validator_text = validator_path.read_text(encoding="utf-8")
    if "final_state: Abort" not in validator_text or "failures: 0" not in validator_text:
        print(validator_text)
        print("abort validator output missing expected success summary", file=sys.stderr)
        return 1

    events_text = events_path.read_text(encoding="utf-8")
    for state in ["Prepare", "Takeoff", "ExecuteMission", "GoHome", "Land", "Abort"]:
        if f'"to":"{state}"' not in events_text and f'"state":"{state}"' not in events_text:
            print(f"abort mission events missing state: {state}", file=sys.stderr)
            return 1
    if '"state":"Complete"' in events_text or '"to":"Complete"' in events_text:
        print("abort mission unexpectedly reached Complete", file=sys.stderr)
        return 1
    if '"command":"Land"' not in events_text:
        print("abort mission should attempt landing before terminal abort", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
