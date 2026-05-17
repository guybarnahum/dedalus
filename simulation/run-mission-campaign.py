#!/usr/bin/env python3
"""Run a small Dedalus mission campaign and summarize scenario artifacts.

Milestone 2.22.3: campaign-level wrapper around `run-mission-scenario.py`.
The campaign runner intentionally delegates the per-run artifact contract to the
single scenario runner, then writes a `campaign_summary.json` and
`campaign_summary.txt` at the campaign root.
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")


def default_campaign_id() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def repo_root_from_script() -> Path:
    return Path(__file__).resolve().parents[1]


def load_run_metadata(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def run_command(command: list[str], cwd: Path) -> int:
    process = subprocess.Popen(
        command,
        cwd=cwd,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        bufsize=1,
    )
    assert process.stdout is not None
    for line in process.stdout:
        sys.stdout.write(line)
        sys.stdout.flush()
    return process.wait()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--campaign", default="mission_campaign", help="Campaign name")
    parser.add_argument("--campaign-id", default=default_campaign_id(), help="Campaign run id")
    parser.add_argument("--scenario", default="trajectory_ci", help="Scenario name")
    parser.add_argument("--repeats", type=int, default=3, help="Number of scenario repeats")
    parser.add_argument("--config", default="config/core_stack_synthetic_mission_ci.yaml")
    parser.add_argument("--output-root", default="out/mission_campaigns")
    parser.add_argument("--app", default="./build-staging/apps/dedalus_mission_loop")
    parser.add_argument("--max-frames", type=int, default=220)
    parser.add_argument("--shutdown-max-frames", type=int, default=50)
    parser.add_argument("--safe-height-m", type=float, default=2.0)
    parser.add_argument("--landed-height-m", type=float, default=1.0)
    parser.add_argument("--expect-complete", dest="expect_complete", action="store_true", default=True)
    parser.add_argument("--no-expect-complete", dest="expect_complete", action="store_false")
    parser.add_argument("--expect-final-state", choices=["Complete", "Abort"], help="Expected final mission state")
    parser.add_argument("--expect-behavior", action="store_true")
    parser.add_argument("--progress", action="store_true", help="Pass --progress through to each scenario")
    parser.add_argument("-v", "--verbose", action="count", default=0, help="Pass verbosity through to each scenario")
    parser.add_argument("--overwrite", action="store_true", help="Replace an existing campaign directory")
    args = parser.parse_args()
    if args.expect_final_state is not None and args.expect_final_state != "Complete":
        args.expect_complete = False
    return args


def build_scenario_command(
    *,
    repo_root: Path,
    args: argparse.Namespace,
    campaign_dir: Path,
    run_number: int,
) -> tuple[list[str], Path]:
    run_id = f"run_{run_number:04d}"
    output_root = campaign_dir / "runs"
    command = [
        sys.executable,
        str(repo_root / "simulation" / "run-mission-scenario.py"),
        "--name",
        args.scenario,
        "--run-id",
        run_id,
        "--config",
        args.config,
        "--output-root",
        str(output_root),
        "--app",
        args.app,
        "--max-frames",
        str(args.max_frames),
        "--shutdown-max-frames",
        str(args.shutdown_max_frames),
        "--safe-height-m",
        str(args.safe_height_m),
        "--landed-height-m",
        str(args.landed_height_m),
        "--overwrite",
    ]
    if args.expect_final_state is not None:
        command += ["--expect-final-state", args.expect_final_state]
    elif not args.expect_complete:
        command.append("--no-expect-complete")
    if args.expect_behavior:
        command.append("--expect-behavior")
    if args.progress:
        command.append("--progress")
    if args.verbose > 0:
        command.append("-" + "v" * min(args.verbose, 3))
    run_dir = output_root / args.scenario / run_id
    return command, run_dir


def write_text_summary(summary: dict[str, Any], path: Path) -> None:
    lines = [
        "Mission campaign summary:",
        f"  campaign: {summary['campaign']}",
        f"  campaign_id: {summary['campaign_id']}",
        f"  scenario: {summary['scenario']}",
        f"  status: {summary['status']}",
        f"  repeats: {summary['repeats']}",
        f"  passed: {summary['passed']}",
        f"  failed: {summary['failed']}",
        f"  elapsed_s: {summary['elapsed_s']}",
        "  runs:",
    ]
    for run in summary["runs"]:
        lines.append(
            "    - "
            f"{run['run_id']}: status={run['status']} "
            f"mission_rc={run['mission_returncode']} validator_rc={run['validator_returncode']}"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    if args.repeats <= 0:
        print("--repeats must be > 0", file=sys.stderr)
        return 2

    repo_root = repo_root_from_script()
    output_root = Path(args.output_root)
    if not output_root.is_absolute():
        output_root = repo_root / output_root
    campaign_dir = output_root / args.campaign / args.campaign_id

    if campaign_dir.exists():
        if not args.overwrite:
            print(f"campaign directory already exists: {campaign_dir}", file=sys.stderr)
            print("Use --overwrite or choose a different --campaign-id.", file=sys.stderr)
            return 2
        shutil.rmtree(campaign_dir)
    campaign_dir.mkdir(parents=True, exist_ok=True)

    started_at = utc_now_iso()
    start = time.monotonic()
    print(f"=== mission campaign: {args.campaign}/{args.campaign_id} ===")
    print(f"Campaign directory: {campaign_dir}")
    print(f"Scenario: {args.scenario} repeats={args.repeats}")

    runs: list[dict[str, Any]] = []
    for run_number in range(1, args.repeats + 1):
        print(f"\n=== campaign run {run_number}/{args.repeats} ===")
        command, run_dir = build_scenario_command(
            repo_root=repo_root,
            args=args,
            campaign_dir=campaign_dir,
            run_number=run_number,
        )
        returncode = run_command(command, repo_root)
        metadata_path = run_dir / "metadata.json"
        if metadata_path.exists():
            run_metadata = load_run_metadata(metadata_path)
        else:
            run_metadata = {
                "scenario_name": args.scenario,
                "run_id": f"run_{run_number:04d}",
                "status": "failed",
                "mission_returncode": returncode,
                "validator_returncode": None,
                "run_dir": str(run_dir),
            }
        run_metadata["scenario_runner_returncode"] = returncode
        runs.append(run_metadata)

    finished_at = utc_now_iso()
    elapsed_s = round(time.monotonic() - start, 3)
    passed = sum(1 for run in runs if run.get("status") == "passed")
    failed = len(runs) - passed
    status = "passed" if failed == 0 else "failed"

    summary = {
        "schema_version": 1,
        "campaign": args.campaign,
        "campaign_id": args.campaign_id,
        "scenario": args.scenario,
        "status": status,
        "started_at": started_at,
        "finished_at": finished_at,
        "elapsed_s": elapsed_s,
        "repeats": args.repeats,
        "passed": passed,
        "failed": failed,
        "config": args.config,
        "expect_final_state": args.expect_final_state if args.expect_final_state is not None else ("Complete" if args.expect_complete else None),
        "output_root": str(output_root),
        "campaign_dir": str(campaign_dir),
        "runs": runs,
    }

    summary_json = campaign_dir / "campaign_summary.json"
    summary_txt = campaign_dir / "campaign_summary.txt"
    summary_json.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    write_text_summary(summary, summary_txt)

    print("\n" + summary_txt.read_text(encoding="utf-8"), end="")
    print(f"Campaign summary JSON: {summary_json}")
    return 0 if status == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
