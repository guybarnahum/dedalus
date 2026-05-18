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
        "--campaign-file",
        str(repo_root / "config" / "mission_campaigns" / "synthetic_ci.json"),
        "--campaign-id",
        "campaign_0001",
        "--app",
        str(build_dir / "apps" / "dedalus_mission_loop"),
        "--output-root",
        str(out_root),
        "--overwrite",
    ]
    result = subprocess.run(command, cwd=repo_root, text=True, capture_output=True, check=False)
    if result.returncode != 0:
        print(result.stdout)
        print(result.stderr, file=sys.stderr)
        return result.returncode

    campaign_dir = out_root / "synthetic_ci" / "campaign_0001"
    summary_json = campaign_dir / "campaign_summary.json"
    summary_txt = campaign_dir / "campaign_summary.txt"
    report_md = campaign_dir / "campaign_report.md"
    if not summary_json.exists() or not summary_txt.exists() or not report_md.exists():
        print(f"missing campaign summary/report under: {campaign_dir}", file=sys.stderr)
        return 1

    summary = json.loads(summary_json.read_text(encoding="utf-8"))
    if summary.get("schema_version") != 3:
        print(json.dumps(summary, indent=2), file=sys.stderr)
        print("campaign summary did not use schema_version=3", file=sys.stderr)
        return 1
    if summary.get("status") != "passed" or summary.get("passed") != 3 or summary.get("failed") != 0:
        print(json.dumps(summary, indent=2), file=sys.stderr)
        print("campaign summary did not report 3/3 passed", file=sys.stderr)
        return 1
    if summary.get("scenario_count") != 2 or summary.get("repeats") != 3:
        print(json.dumps(summary, indent=2), file=sys.stderr)
        print("campaign summary did not preserve scenario/repeat counts", file=sys.stderr)
        return 1
    if len(summary.get("runs", [])) != 3:
        print(json.dumps(summary, indent=2), file=sys.stderr)
        print("campaign summary did not preserve three run records", file=sys.stderr)
        return 1

    expected = [
        ("synthetic_lifecycle", "run_0001", "Complete"),
        ("synthetic_lifecycle", "run_0002", "Complete"),
        ("synthetic_abort_land_timeout", "run_0001", "Abort"),
    ]
    for run, (scenario, run_id, expected_final_state) in zip(summary["runs"], expected):
        if (
            run.get("scenario_name") != scenario
            or run.get("run_id") != run_id
            or run.get("status") != "passed"
            or run.get("expect_final_state") != expected_final_state
        ):
            print(json.dumps(run, indent=2), file=sys.stderr)
            print("run summary did not preserve expected scenario/id/status/final-state", file=sys.stderr)
            return 1
        run_dir = Path(run["run_dir"])
        for name in ["metadata.json", "mission_events.jsonl", "validator_result.txt", "console.log"]:
            if not (run_dir / name).exists():
                print(f"missing per-run artifact: {run_dir / name}", file=sys.stderr)
                return 1

    summary_text = summary_txt.read_text(encoding="utf-8")
    if "passed: 3" not in summary_text or "failed: 0" not in summary_text:
        print(summary_text)
        print("text summary missing pass/fail counts", file=sys.stderr)
        return 1
    if "synthetic_abort_land_timeout/run_0001" not in summary_text:
        print(summary_text)
        print("text summary missing abort scenario row", file=sys.stderr)
        return 1

    report_text = report_md.read_text(encoding="utf-8")
    required_report_fragments = [
        "# Mission Campaign Report: synthetic_ci",
        "| Status | **passed** |",
        "`synthetic_lifecycle/run_0001`",
        "`synthetic_lifecycle/run_0002`",
        "`synthetic_abort_land_timeout/run_0001`",
        "[metadata](runs/synthetic_lifecycle/run_0001/metadata.json)",
        "[validator](runs/synthetic_abort_land_timeout/run_0001/validator_result.txt)",
        "Abort",
        "Complete",
    ]
    for fragment in required_report_fragments:
        if fragment not in report_text:
            print(report_text)
            print(f"markdown report missing fragment: {fragment}", file=sys.stderr)
            return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
