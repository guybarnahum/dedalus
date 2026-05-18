#!/usr/bin/env python3
"""Run one Dedalus mission scenario and preserve archive-grade artifacts.

This is the first Milestone 2.22 scenario/campaign harness slice. It wraps
`dedalus_mission_loop`, captures combined stdout/stderr to `console.log`, runs
`validate-mission-artifacts.py`, and writes `metadata.json` beside the run
artifacts.
"""

from __future__ import annotations

import argparse
import json
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")


def default_run_id() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def repo_root_from_script() -> Path:
    return Path(__file__).resolve().parents[1]


def expected_final_state_from(args: argparse.Namespace) -> str | None:
    if args.expect_final_state is not None:
        return args.expect_final_state
    if args.expect_complete:
        return "Complete"
    return None


def stream_command(command: list[str], cwd: Path, log_path: Path) -> int:
    """Stream command output in real time while preserving carriage returns.

    Python text-mode pipes use universal-newline translation, which can turn
    child `\r` progress updates into `\n`. Binary-mode reads preserve the exact
    terminal control bytes while still allowing the run artifact log to capture
    the same stream. Ctrl-C is relayed to the active mission process and then
    the wrapper keeps streaming so `dedalus_mission_loop` can perform its
    graceful finish path. A second Ctrl-C forces termination.
    """
    with log_path.open("wb") as log_file:
        process = subprocess.Popen(
            command,
            cwd=cwd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            bufsize=0,
        )
        assert process.stdout is not None
        interrupt_count = 0
        while True:
            try:
                chunk = process.stdout.read(1)
            except KeyboardInterrupt:
                interrupt_count += 1
                if interrupt_count == 1:
                    message = (
                        b"\nrun-mission-scenario: interrupt received; "
                        b"forwarding to mission process and waiting for graceful shutdown\n"
                    )
                    sys.stdout.buffer.write(message)
                    sys.stdout.buffer.flush()
                    log_file.write(message)
                    log_file.flush()
                    if process.poll() is None:
                        process.send_signal(signal.SIGINT)
                    continue
                message = b"\nrun-mission-scenario: second interrupt; terminating mission process\n"
                sys.stdout.buffer.write(message)
                sys.stdout.buffer.flush()
                log_file.write(message)
                log_file.flush()
                if process.poll() is None:
                    process.terminate()
                break
            if chunk == b"" and process.poll() is not None:
                break
            if not chunk:
                continue
            sys.stdout.buffer.write(chunk)
            sys.stdout.buffer.flush()
            log_file.write(chunk)
            log_file.flush()
        return process.wait()


def run_capture(command: list[str], cwd: Path, output_path: Path) -> int:
    result = subprocess.run(
        command,
        cwd=cwd,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    output_path.write_text(result.stdout, encoding="utf-8")
    sys.stdout.write(result.stdout)
    sys.stdout.flush()
    return result.returncode


def build_metadata(
    *,
    args: argparse.Namespace,
    repo_root: Path,
    run_dir: Path,
    mission_command: list[str],
    validator_command: list[str],
    mission_returncode: int | None,
    validator_returncode: int | None,
    started_at: str,
    finished_at: str,
    elapsed_s: float,
) -> dict[str, Any]:
    status = "passed" if mission_returncode == 0 and validator_returncode == 0 else "failed"
    expected_final_state = expected_final_state_from(args)
    return {
        "schema_version": 1,
        "scenario_name": args.name,
        "run_id": args.run_id,
        "status": status,
        "started_at": started_at,
        "finished_at": finished_at,
        "elapsed_s": round(elapsed_s, 3),
        "repo_root": str(repo_root),
        "run_dir": str(run_dir),
        "config": args.config,
        "app": args.app,
        "max_frames": args.max_frames,
        "shutdown_max_frames": args.shutdown_max_frames,
        "expect_complete": args.expect_complete,
        "expect_final_state": expected_final_state,
        "expect_behavior": args.expect_behavior,
        "safe_height_m": args.safe_height_m,
        "landed_height_m": args.landed_height_m,
        "mission_returncode": mission_returncode,
        "validator_returncode": validator_returncode,
        "mission_command": mission_command,
        "validator_command": validator_command,
        "artifacts": {
            "console_log": "console.log",
            "metadata": "metadata.json",
            "validator_result": "validator_result.txt",
            "mission_events": "mission_events.jsonl",
            "snapshot_manifest": "snapshot_manifest.txt",
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", default="trajectory_mission", help="Scenario name")
    parser.add_argument("--run-id", default=default_run_id(), help="Run directory name under <output-root>/<name>")
    parser.add_argument("--config", default="config/core_stack_trajectory_mission_placeholder.yaml")
    parser.add_argument("--output-root", default="out/mission_scenarios")
    parser.add_argument("--app", default="./build-staging/apps/dedalus_mission_loop")
    parser.add_argument("--validator", default="python3 simulation/validate-mission-artifacts.py")
    parser.add_argument("--max-frames", type=int, default=900)
    parser.add_argument("--shutdown-max-frames", type=int, default=400)
    parser.add_argument("--safe-height-m", type=float, default=16.0)
    parser.add_argument("--landed-height-m", type=float, default=1.0)
    parser.add_argument("--expect-complete", dest="expect_complete", action="store_true", default=True)
    parser.add_argument("--no-expect-complete", dest="expect_complete", action="store_false")
    parser.add_argument("--expect-final-state", choices=["Complete", "Abort"], help="Expected final mission state")
    parser.add_argument("--expect-behavior", action="store_true", help="Require M3 object-conditioned behavior events")
    parser.add_argument("--progress", action="store_true", help="Pass --progress through to dedalus_mission_loop")
    parser.add_argument("-v", "--verbose", action="count", default=0, help="Pass verbosity through to dedalus_mission_loop")
    parser.add_argument("--overwrite", action="store_true", help="Replace an existing scenario run directory")
    args = parser.parse_args()
    if args.expect_final_state is not None and args.expect_final_state != "Complete":
        args.expect_complete = False
    return args


def main() -> int:
    args = parse_args()
    repo_root = repo_root_from_script()
    output_root = Path(args.output_root)
    if not output_root.is_absolute():
        output_root = repo_root / output_root
    run_dir = output_root / args.name / args.run_id

    if run_dir.exists():
        if not args.overwrite:
            print(f"run directory already exists: {run_dir}", file=sys.stderr)
            print("Use --overwrite or choose a different --run-id.", file=sys.stderr)
            return 2
        shutil.rmtree(run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)

    mission_command = [
        args.app,
        "--config",
        args.config,
        "--output-dir",
        str(run_dir),
        "--max-frames",
        str(args.max_frames),
        "--shutdown-max-frames",
        str(args.shutdown_max_frames),
    ]
    if args.progress:
        mission_command.append("--progress")
    else:
        mission_command.append("--no-progress")
    if args.verbose > 0:
        mission_command.append("-" + "v" * min(args.verbose, 3))

    validator_command = args.validator.split() + [str(run_dir)]
    expected_final_state = expected_final_state_from(args)
    if expected_final_state is not None:
        validator_command += ["--expect-final-state", expected_final_state]
    if args.expect_behavior:
        validator_command.append("--expect-behavior")
    validator_command += [
        "--safe-height-m",
        str(args.safe_height_m),
        "--landed-height-m",
        str(args.landed_height_m),
    ]

    started_at = utc_now_iso()
    started_monotonic = time.monotonic()
    print(f"=== mission scenario: {args.name}/{args.run_id}", flush=True)
    print(f"Run directory: {run_dir}", flush=True)

    mission_returncode = stream_command(mission_command, repo_root, run_dir / "console.log")
    validator_returncode: int | None = None
    if mission_returncode == 0:
        validator_returncode = run_capture(validator_command, repo_root, run_dir / "validator_result.txt")
    else:
        (run_dir / "validator_result.txt").write_text(
            "validator skipped because mission command failed\n",
            encoding="utf-8",
        )

    finished_at = utc_now_iso()
    elapsed_s = time.monotonic() - started_monotonic
    metadata = build_metadata(
        args=args,
        repo_root=repo_root,
        run_dir=run_dir,
        mission_command=mission_command,
        validator_command=validator_command,
        mission_returncode=mission_returncode,
        validator_returncode=validator_returncode,
        started_at=started_at,
        finished_at=finished_at,
        elapsed_s=elapsed_s,
    )
    (run_dir / "metadata.json").write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    print(f"Scenario status: {metadata['status']}", flush=True)
    print(f"Metadata: {run_dir / 'metadata.json'}", flush=True)
    return 0 if metadata["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
