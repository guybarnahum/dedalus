#!/usr/bin/env python3
"""Smoke test for tools/mission/run-mission-campaign.py."""

from __future__ import annotations

import json
import shutil
import subprocess
import sys
from pathlib import Path


def run_campaign(command: list[str], repo_root: Path) -> int:
    result = subprocess.run(command, cwd=repo_root, text=True, capture_output=True, check=False)
    if result.returncode != 0:
        print(result.stdout)
        print(result.stderr, file=sys.stderr)
    return result.returncode


def require_file(path: Path) -> bool:
    if not path.exists():
        print(f"missing file: {path}", file=sys.stderr)
        return False
    return True


def validate_synthetic_dry_run(repo_root: Path, build_dir: Path, out_root: Path) -> int:
    command = [
        sys.executable,
        str(repo_root / "tools" / "mission" / "run-mission-campaign.py"),
        "--campaign-file",
        str(repo_root / "config" / "ci" / "synthetic_ci.json"),
        "--campaign-id",
        "synthetic_dry_run_0001",
        "--app",
        str(build_dir / "apps" / "dedalus_mission_loop"),
        "--output-root",
        str(out_root),
        "--dry-run",
        "--overwrite",
    ]
    returncode = run_campaign(command, repo_root)
    if returncode != 0:
        return returncode

    campaign_dir = out_root / "synthetic_ci" / "synthetic_dry_run_0001"
    summary_json = campaign_dir / "campaign_summary.json"
    summary_txt = campaign_dir / "campaign_summary.txt"
    report_md = campaign_dir / "campaign_report.md"
    if not all(require_file(path) for path in [summary_json, summary_txt, report_md]):
        return 1

    summary = json.loads(summary_json.read_text(encoding="utf-8"))
    if summary.get("schema_version") != 4:
        print(json.dumps(summary, indent=2), file=sys.stderr)
        print("campaign summary did not use schema_version=4", file=sys.stderr)
        return 1
    if summary.get("status") != "planned" or summary.get("dry_run") is not True:
        print(json.dumps(summary, indent=2), file=sys.stderr)
        print("synthetic campaign dry-run did not report planned status", file=sys.stderr)
        return 1
    if summary.get("planned") != 3 or summary.get("passed") != 0 or summary.get("failed") != 0:
        print(json.dumps(summary, indent=2), file=sys.stderr)
        print("synthetic dry-run did not report three planned runs", file=sys.stderr)
        return 1
    if summary.get("scenario_count") != 2 or summary.get("repeats") != 3:
        print(json.dumps(summary, indent=2), file=sys.stderr)
        print("synthetic dry-run did not preserve scenario/repeat counts", file=sys.stderr)
        return 1
    if len(summary.get("runs", [])) != 3:
        print(json.dumps(summary, indent=2), file=sys.stderr)
        print("synthetic dry-run did not preserve three run records", file=sys.stderr)
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
            or run.get("status") != "planned"
            or run.get("expect_final_state") != expected_final_state
        ):
            print(json.dumps(run, indent=2), file=sys.stderr)
            print("planned run did not preserve expected scenario/id/status/final-state", file=sys.stderr)
            return 1
        command_text = " ".join(run.get("scenario_command", []))
        if "run-mission-scenario.py" not in command_text or "--expect-final-state" not in command_text:
            print(json.dumps(run, indent=2), file=sys.stderr)
            print("planned synthetic run missing expected scenario command", file=sys.stderr)
            return 1

    summary_text = summary_txt.read_text(encoding="utf-8")
    if "planned: 3" not in summary_text and "planned" not in summary_text:
        print(summary_text)
        print("text summary missing planned state", file=sys.stderr)
        return 1
    if "synthetic_abort_land_timeout/run_0001" not in summary_text:
        print(summary_text)
        print("text summary missing abort scenario row", file=sys.stderr)
        return 1

    report_text = report_md.read_text(encoding="utf-8")
    required_report_fragments = [
        "# Mission Campaign Report: synthetic_ci",
        "| Status | **planned** |",
        "`synthetic_lifecycle/run_0001`",
        "`synthetic_lifecycle/run_0002`",
        "`synthetic_abort_land_timeout/run_0001`",
        "Abort",
        "Complete",
    ]
    for fragment in required_report_fragments:
        if fragment not in report_text:
            print(report_text)
            print(f"markdown report missing fragment: {fragment}", file=sys.stderr)
            return 1
    return 0


def validate_airsim_dry_run(repo_root: Path, build_dir: Path, out_root: Path) -> int:
    command = [
        sys.executable,
        str(repo_root / "tools" / "mission" / "run-mission-campaign.py"),
        "--campaign-file",
        str(repo_root / "config" / "ci" / "airsim_live_smoke_ci.json"),
        "--campaign-id",
        "airsim_dry_run_0001",
        "--app",
        str(build_dir / "apps" / "dedalus_mission_loop"),
        "--output-root",
        str(out_root),
        "--dry-run",
        "--overwrite",
    ]
    returncode = run_campaign(command, repo_root)
    if returncode != 0:
        return returncode

    campaign_dir = out_root / "airsim_live_smoke" / "airsim_dry_run_0001"
    summary_json = campaign_dir / "campaign_summary.json"
    report_md = campaign_dir / "campaign_report.md"
    if not all(require_file(path) for path in [summary_json, report_md]):
        return 1

    summary = json.loads(summary_json.read_text(encoding="utf-8"))
    if summary.get("status") != "planned" or summary.get("dry_run") is not True:
        print(json.dumps(summary, indent=2), file=sys.stderr)
        print("AirSim dry-run did not report planned dry-run status", file=sys.stderr)
        return 1
    if summary.get("planned") != 3 or summary.get("passed") != 0 or summary.get("failed") != 0:
        print(json.dumps(summary, indent=2), file=sys.stderr)
        print("AirSim dry-run did not report three planned runs", file=sys.stderr)
        return 1
    if summary.get("scenario_count") != 1 or summary.get("repeats") != 3:
        print(json.dumps(summary, indent=2), file=sys.stderr)
        print("AirSim dry-run did not preserve scenario/repeat counts", file=sys.stderr)
        return 1
    for run in summary.get("runs", []):
        if run.get("status") != "planned" or run.get("expect_final_state") != "Complete":
            print(json.dumps(run, indent=2), file=sys.stderr)
            print("planned AirSim run did not preserve expected status/final-state", file=sys.stderr)
            return 1
        command_text = " ".join(run.get("scenario_command", []))
        if "core_stack_trajectory_mission_placeholder.yaml" not in command_text:
            print(json.dumps(run, indent=2), file=sys.stderr)
            print("planned AirSim run missing live trajectory config", file=sys.stderr)
            return 1

    report_text = report_md.read_text(encoding="utf-8")
    if "# Mission Campaign Report: airsim_live_smoke" not in report_text:
        print(report_text)
        print("AirSim dry-run report missing title", file=sys.stderr)
        return 1
    if "trajectory_px4_bridge_live/run_0003" not in report_text:
        print(report_text)
        print("AirSim dry-run report missing third planned run", file=sys.stderr)
        return 1
    return 0


def main() -> int:
    repo_root = Path(sys.argv[1])
    build_dir = Path(sys.argv[2])
    out_root = repo_root / "out" / "test_mission_campaigns"
    if out_root.exists():
        shutil.rmtree(out_root)

    result = validate_synthetic_dry_run(repo_root, build_dir, out_root)
    if result != 0:
        return result
    return validate_airsim_dry_run(repo_root, build_dir, out_root)


if __name__ == "__main__":
    raise SystemExit(main())
