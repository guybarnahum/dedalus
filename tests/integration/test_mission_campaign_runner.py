#!/usr/bin/env python3
"""Smoke test for simulation/run-mission-campaign.py."""

from __future__ import annotations

import json
import shutil
import subprocess
import sys
from pathlib import Path


def main() -> int:
    repo_root = Path(sys.argv[1])
    build_dir = Path(sys.argv[2])
    out_root = repo_root / "out" / "test_mission_campaigns"
    if out_root.exists():
        shutil.rmtree(out_root)

    command = [
        sys.executable,
        str(repo_root / "simulation" / "run-mission-campaign.py"),
        "--campaign",
        "ci_campaign",
        "--campaign-id",
        "campaign_0001",
        "--scenario",
        "synthetic_lifecycle",
        "--repeats",
        "2",
        "--config",
        str(repo_root / "config" / "core_stack_synthetic_mission_ci.yaml"),
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
        "--overwrite",
    ]
    result = subprocess.run(command, cwd=repo_root, text=True, capture_output=True, check=False)
    if result.returncode != 0:
        print(result.stdout)
        print(result.stderr, file=sys.stderr)
        return result.returncode

    campaign_dir = out_root / "ci_campaign" / "campaign_0001"
    summary_json = campaign_dir / "campaign_summary.json"
    summary_txt = campaign_dir / "campaign_summary.txt"
    if not summary_json.exists() or not summary_txt.exists():
        print(f"missing campaign summary under: {campaign_dir}", file=sys.stderr)
        return 1

    summary = json.loads(summary_json.read_text(encoding="utf-8"))
    if summary.get("status") != "passed" or summary.get("passed") != 2 or summary.get("failed") != 0:
        print(json.dumps(summary, indent=2), file=sys.stderr)
        print("campaign summary did not report 2/2 passed", file=sys.stderr)
        return 1
    if len(summary.get("runs", [])) != 2:
        print(json.dumps(summary, indent=2), file=sys.stderr)
        print("campaign summary did not preserve two run records", file=sys.stderr)
        return 1

    for index, run in enumerate(summary["runs"], start=1):
        expected_run_id = f"run_{index:04d}"
        if run.get("run_id") != expected_run_id or run.get("status") != "passed":
            print(json.dumps(run, indent=2), file=sys.stderr)
            print("run summary did not preserve expected id/status", file=sys.stderr)
            return 1
        run_dir = Path(run["run_dir"])
        for name in ["metadata.json", "mission_events.jsonl", "validator_result.txt", "console.log"]:
            if not (run_dir / name).exists():
                print(f"missing per-run artifact: {run_dir / name}", file=sys.stderr)
                return 1

    summary_text = summary_txt.read_text(encoding="utf-8")
    if "passed: 2" not in summary_text or "failed: 0" not in summary_text:
        print(summary_text)
        print("text summary missing pass/fail counts", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
