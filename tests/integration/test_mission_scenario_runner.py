#!/usr/bin/env python3
"""Smoke test for tools/mission/run-mission-scenario.py."""

from __future__ import annotations

import json
import shutil
import subprocess
import sys
import time
from pathlib import Path


def run_streaming(command: list[str], cwd: Path) -> int:
    process = subprocess.Popen(
        command,
        cwd=cwd,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        bufsize=1,
    )
    assert process.stdout is not None
    last_progress_s = time.monotonic()
    for line in process.stdout:
        now = time.monotonic()
        if now - last_progress_s > 0.75:
            sys.stdout.write("\rmission_scenario_runner: still running...\033[K")
            sys.stdout.flush()
            last_progress_s = now
        sys.stdout.write(line)
        sys.stdout.flush()
    sys.stdout.write("\rmission_scenario_runner: subprocess finished\033[K\n")
    sys.stdout.flush()
    return process.wait()


def main() -> int:
    repo_root = Path(sys.argv[1])
    build_dir = Path(sys.argv[2])
    out_root = repo_root / "out" / "test_mission_scenarios"
    if out_root.exists():
        shutil.rmtree(out_root)

    command = [
        sys.executable,
        str(repo_root / "tools" / "mission" / "run-mission-scenario.py"),
        "--name",
        "ci_smoke",
        "--run-id",
        "run_0001",
        "--config",
        str(repo_root / "config" / "ci" / "core_stack_synthetic_mission_ci.yaml"),
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
        "--progress",
        "--overwrite",
    ]
    returncode = run_streaming(command, repo_root)
    if returncode != 0:
        return returncode

    run_dir = out_root / "ci_smoke" / "run_0001"
    metadata_path = run_dir / "metadata.json"
    console_path = run_dir / "console.log"
    validator_path = run_dir / "validator_result.txt"
    events_path = run_dir / "mission_events.jsonl"
    manifest_path = run_dir / "snapshot_manifest.txt"

    for path in [metadata_path, console_path, validator_path, events_path, manifest_path]:
        if not path.exists():
            print(f"missing scenario artifact: {path}", file=sys.stderr)
            return 1

    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    if metadata.get("status") != "passed":
        print(json.dumps(metadata, indent=2), file=sys.stderr)
        print("scenario metadata did not report passed", file=sys.stderr)
        return 1
    if metadata.get("scenario_name") != "ci_smoke" or metadata.get("run_id") != "run_0001":
        print(json.dumps(metadata, indent=2), file=sys.stderr)
        print("scenario metadata did not preserve identity", file=sys.stderr)
        return 1
    if metadata.get("expect_complete") is not True:
        print(json.dumps(metadata, indent=2), file=sys.stderr)
        print("scenario metadata should preserve expect-complete mode", file=sys.stderr)
        return 1
    if metadata.get("mission_returncode") != 0 or metadata.get("validator_returncode") != 0:
        print(json.dumps(metadata, indent=2), file=sys.stderr)
        print("scenario command return codes were not successful", file=sys.stderr)
        return 1

    console_text = console_path.read_text(encoding="utf-8")
    if "Mission runtime: trajectory_mission" not in console_text:
        print("console log missing mission runtime startup", file=sys.stderr)
        return 1
    if "Mission events:" not in console_text:
        print("console log missing mission events artifact line", file=sys.stderr)
        return 1

    validator_text = validator_path.read_text(encoding="utf-8")
    if "Mission artifact validation:" not in validator_text or "failures: 0" not in validator_text:
        print(validator_text)
        print("validator output missing success summary", file=sys.stderr)
        return 1

    events_text = events_path.read_text(encoding="utf-8")
    expected_states = ["Prepare", "Takeoff", "ExecuteMission", "GoHome", "Land", "Complete"]
    for state in expected_states:
        if f'"to":"{state}"' not in events_text and f'"state":"{state}"' not in events_text:
            print(f"mission events missing state: {state}", file=sys.stderr)
            return 1
    if '"event":"runtime_stop"' not in events_text or '"state":"Complete"' not in events_text:
        print("mission events missing Complete runtime_stop", file=sys.stderr)
        return 1
    if '"command":"Disarm"' not in events_text:
        print("mission events missing disarm command", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
