#!/usr/bin/env python3
"""Run a Dedalus mission campaign and summarize scenario artifacts.

Milestone 2.22.8: campaign-level wrapper around `run-mission-scenario.py` with
CLI-driven single-scenario campaigns, JSON campaign specification files, and a
CI-safe dry-run planning mode for validating live campaign presets without
launching AirSim/PX4.
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
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


@dataclass(frozen=True)
class ScenarioSpec:
    name: str
    config: str
    repeats: int
    max_frames: int
    shutdown_max_frames: int
    safe_height_m: float
    landed_height_m: float
    expect_final_state: str | None
    expect_behavior: bool


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--campaign-file", type=Path, help="JSON campaign specification file")
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
    parser.add_argument("--dry-run", action="store_true", help="Write a campaign plan without executing scenario runs")
    args = parser.parse_args()
    if args.expect_final_state is not None and args.expect_final_state != "Complete":
        args.expect_complete = False
    return args


def default_expected_final_state(args: argparse.Namespace) -> str | None:
    if args.expect_final_state is not None:
        return args.expect_final_state
    if args.expect_complete:
        return "Complete"
    return None


def require_positive_repeats(repeats: int, context: str) -> None:
    if repeats <= 0:
        raise ValueError(f"{context} repeats must be > 0")


def scenario_from_cli(args: argparse.Namespace) -> list[ScenarioSpec]:
    require_positive_repeats(args.repeats, args.scenario)
    return [
        ScenarioSpec(
            name=args.scenario,
            config=args.config,
            repeats=args.repeats,
            max_frames=args.max_frames,
            shutdown_max_frames=args.shutdown_max_frames,
            safe_height_m=args.safe_height_m,
            landed_height_m=args.landed_height_m,
            expect_final_state=default_expected_final_state(args),
            expect_behavior=args.expect_behavior,
        )
    ]


def load_campaign_file(path: Path, args: argparse.Namespace) -> tuple[str, list[ScenarioSpec], dict[str, Any]]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise ValueError(f"failed to read campaign file {path}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise ValueError(f"failed to parse campaign file {path}: {exc}") from exc

    if not isinstance(data, dict):
        raise ValueError("campaign file must contain a JSON object")
    scenarios_raw = data.get("scenarios")
    if not isinstance(scenarios_raw, list) or not scenarios_raw:
        raise ValueError("campaign file must contain a non-empty scenarios array")

    defaults = data.get("defaults", {})
    if defaults is None:
        defaults = {}
    if not isinstance(defaults, dict):
        raise ValueError("campaign defaults must be an object")

    campaign_name = str(data.get("campaign") or args.campaign)
    specs: list[ScenarioSpec] = []
    for index, raw in enumerate(scenarios_raw, start=1):
        if not isinstance(raw, dict):
            raise ValueError(f"scenario {index} must be an object")
        name = raw.get("name")
        config = raw.get("config")
        if not isinstance(name, str) or not name:
            raise ValueError(f"scenario {index} missing non-empty name")
        if not isinstance(config, str) or not config:
            raise ValueError(f"scenario {name} missing non-empty config")

        repeats = int(raw.get("repeats", defaults.get("repeats", args.repeats)))
        require_positive_repeats(repeats, name)
        expect_final_state = raw.get("expect_final_state", defaults.get("expect_final_state", default_expected_final_state(args)))
        if expect_final_state is not None and expect_final_state not in {"Complete", "Abort"}:
            raise ValueError(f"scenario {name} has unsupported expect_final_state: {expect_final_state}")

        specs.append(
            ScenarioSpec(
                name=name,
                config=config,
                repeats=repeats,
                max_frames=int(raw.get("max_frames", defaults.get("max_frames", args.max_frames))),
                shutdown_max_frames=int(raw.get("shutdown_max_frames", defaults.get("shutdown_max_frames", args.shutdown_max_frames))),
                safe_height_m=float(raw.get("safe_height_m", defaults.get("safe_height_m", args.safe_height_m))),
                landed_height_m=float(raw.get("landed_height_m", defaults.get("landed_height_m", args.landed_height_m))),
                expect_final_state=expect_final_state,
                expect_behavior=bool(raw.get("expect_behavior", defaults.get("expect_behavior", args.expect_behavior))),
            )
        )
    return campaign_name, specs, data


def build_scenario_command(
    *,
    repo_root: Path,
    args: argparse.Namespace,
    campaign_dir: Path,
    scenario: ScenarioSpec,
    run_number: int,
) -> tuple[list[str], Path]:
    run_id = f"run_{run_number:04d}"
    output_root = campaign_dir / "runs"
    command = [
        sys.executable,
        str(repo_root / "simulation" / "run-mission-scenario.py"),
        "--name",
        scenario.name,
        "--run-id",
        run_id,
        "--config",
        scenario.config,
        "--output-root",
        str(output_root),
        "--app",
        args.app,
        "--max-frames",
        str(scenario.max_frames),
        "--shutdown-max-frames",
        str(scenario.shutdown_max_frames),
        "--safe-height-m",
        str(scenario.safe_height_m),
        "--landed-height-m",
        str(scenario.landed_height_m),
        "--overwrite",
    ]
    if scenario.expect_final_state is not None:
        command += ["--expect-final-state", scenario.expect_final_state]
    else:
        command.append("--no-expect-complete")
    if scenario.expect_behavior:
        command.append("--expect-behavior")
    if args.progress:
        command.append("--progress")
    if args.verbose > 0:
        command.append("-" + "v" * min(args.verbose, 3))
    run_dir = output_root / scenario.name / run_id
    return command, run_dir


def write_text_summary(summary: dict[str, Any], path: Path) -> None:
    lines = [
        "Mission campaign summary:",
        f"  campaign: {summary['campaign']}",
        f"  campaign_id: {summary['campaign_id']}",
        f"  status: {summary['status']}",
        f"  scenarios: {summary['scenario_count']}",
        f"  repeats: {summary['repeats']}",
        f"  passed: {summary['passed']}",
        f"  failed: {summary['failed']}",
        f"  elapsed_s: {summary['elapsed_s']}",
        "  runs:",
    ]
    for run in summary["runs"]:
        lines.append(
            "    - "
            f"{run['scenario_name']}/{run['run_id']}: status={run['status']} "
            f"expected={run.get('expect_final_state')} "
            f"mission_rc={run['mission_returncode']} validator_rc={run['validator_returncode']}"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run_relpath(campaign_dir: Path, run: dict[str, Any], filename: str) -> str:
    try:
        return str((Path(run["run_dir"]) / filename).relative_to(campaign_dir))
    except (KeyError, ValueError):
        return str(Path(run.get("run_dir", ".")) / filename)


def write_markdown_report(summary: dict[str, Any], path: Path) -> None:
    campaign_dir = Path(summary["campaign_dir"])
    lines = [
        f"# Mission Campaign Report: {summary['campaign']}",
        "",
        "## Summary",
        "",
        "| Field | Value |",
        "|---|---:|",
        f"| Campaign ID | `{summary['campaign_id']}` |",
        f"| Status | **{summary['status']}** |",
        f"| Scenarios | {summary['scenario_count']} |",
        f"| Runs | {summary['repeats']} |",
        f"| Passed | {summary['passed']} |",
        f"| Failed | {summary['failed']} |",
        f"| Elapsed seconds | {summary['elapsed_s']} |",
        "",
        "## Scenarios",
        "",
        "| Scenario | Config | Repeats | Expected final state | Safe height | Landed height |",
        "|---|---|---:|---|---:|---:|",
    ]
    for scenario in summary["scenarios"]:
        lines.append(
            "| "
            f"`{scenario['name']}` | `{scenario['config']}` | {scenario['repeats']} | "
            f"{scenario.get('expect_final_state')} | {scenario['safe_height_m']} | {scenario['landed_height_m']} |"
        )

    lines += [
        "",
        "## Runs",
        "",
        "| Scenario / Run | Status | Expected | Mission RC | Validator RC | Artifacts |",
        "|---|---|---|---:|---:|---|",
    ]
    for run in summary["runs"]:
        run_label = f"{run['scenario_name']}/{run['run_id']}"
        artifacts = ", ".join(
            [
                f"[metadata]({run_relpath(campaign_dir, run, 'metadata.json')})",
                f"[events]({run_relpath(campaign_dir, run, 'mission_events.jsonl')})",
                f"[validator]({run_relpath(campaign_dir, run, 'validator_result.txt')})",
                f"[console]({run_relpath(campaign_dir, run, 'console.log')})",
            ]
        )
        lines.append(
            "| "
            f"`{run_label}` | {run.get('status')} | {run.get('expect_final_state')} | "
            f"{run.get('mission_returncode')} | {run.get('validator_returncode')} | {artifacts} |"
        )

    lines += [
        "",
        "## Files",
        "",
        "- `campaign_summary.json`: machine-readable campaign summary.",
        "- `campaign_summary.txt`: compact terminal-oriented summary.",
        "- `campaign_report.md`: this human-readable report.",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def build_dry_run_records(
    *,
    repo_root: Path,
    args: argparse.Namespace,
    campaign_dir: Path,
    scenarios: list[ScenarioSpec],
) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for scenario in scenarios:
        for run_number in range(1, scenario.repeats + 1):
            command, run_dir = build_scenario_command(
                repo_root=repo_root,
                args=args,
                campaign_dir=campaign_dir,
                scenario=scenario,
                run_number=run_number,
            )
            records.append(
                {
                    "scenario_name": scenario.name,
                    "run_id": f"run_{run_number:04d}",
                    "status": "planned",
                    "expect_final_state": scenario.expect_final_state,
                    "mission_returncode": None,
                    "validator_returncode": None,
                    "run_dir": str(run_dir),
                    "scenario_command": command,
                }
            )
    return records


def write_campaign_outputs(summary: dict[str, Any], campaign_dir: Path) -> tuple[Path, Path, Path]:
    summary_json = campaign_dir / "campaign_summary.json"
    summary_txt = campaign_dir / "campaign_summary.txt"
    report_md = campaign_dir / "campaign_report.md"
    summary_json.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    write_text_summary(summary, summary_txt)
    write_markdown_report(summary, report_md)
    return summary_json, summary_txt, report_md


def main() -> int:
    args = parse_args()

    repo_root = repo_root_from_script()
    campaign_spec: dict[str, Any] | None = None
    try:
        if args.campaign_file is not None:
            campaign_name, scenarios, campaign_spec = load_campaign_file(args.campaign_file, args)
        else:
            campaign_name = args.campaign
            scenarios = scenario_from_cli(args)
    except ValueError as exc:
        print(f"run-mission-campaign: {exc}", file=sys.stderr)
        return 2

    output_root = Path(args.output_root)
    if not output_root.is_absolute():
        output_root = repo_root / output_root
    campaign_dir = output_root / campaign_name / args.campaign_id

    if campaign_dir.exists():
        if not args.overwrite:
            print(f"campaign directory already exists: {campaign_dir}", file=sys.stderr)
            print("Use --overwrite or choose a different --campaign-id.", file=sys.stderr)
            return 2
        shutil.rmtree(campaign_dir)
    campaign_dir.mkdir(parents=True, exist_ok=True)

    started_at = utc_now_iso()
    start = time.monotonic()
    print(f"=== mission campaign: {campaign_name}/{args.campaign_id} ===")
    print(f"Campaign directory: {campaign_dir}")
    print(f"Scenarios: {len(scenarios)}")
    if args.dry_run:
        print("Mode: dry-run plan only")

    if args.dry_run:
        runs = build_dry_run_records(
            repo_root=repo_root,
            args=args,
            campaign_dir=campaign_dir,
            scenarios=scenarios,
        )
    else:
        runs = []
        for scenario in scenarios:
            for run_number in range(1, scenario.repeats + 1):
                print(f"\n=== campaign scenario {scenario.name} run {run_number}/{scenario.repeats} ===")
                command, run_dir = build_scenario_command(
                    repo_root=repo_root,
                    args=args,
                    campaign_dir=campaign_dir,
                    scenario=scenario,
                    run_number=run_number,
                )
                returncode = run_command(command, repo_root)
                metadata_path = run_dir / "metadata.json"
                if metadata_path.exists():
                    run_metadata = load_run_metadata(metadata_path)
                else:
                    run_metadata = {
                        "scenario_name": scenario.name,
                        "run_id": f"run_{run_number:04d}",
                        "status": "failed",
                        "expect_final_state": scenario.expect_final_state,
                        "mission_returncode": returncode,
                        "validator_returncode": None,
                        "run_dir": str(run_dir),
                    }
                run_metadata["scenario_runner_returncode"] = returncode
                runs.append(run_metadata)

    finished_at = utc_now_iso()
    elapsed_s = round(time.monotonic() - start, 3)
    passed = sum(1 for run in runs if run.get("status") == "passed")
    failed = sum(1 for run in runs if run.get("status") == "failed")
    planned = sum(1 for run in runs if run.get("status") == "planned")
    status = "planned" if args.dry_run else ("passed" if failed == 0 else "failed")

    summary = {
        "schema_version": 4,
        "campaign": campaign_name,
        "campaign_id": args.campaign_id,
        "status": status,
        "dry_run": args.dry_run,
        "started_at": started_at,
        "finished_at": finished_at,
        "elapsed_s": elapsed_s,
        "scenario_count": len(scenarios),
        "repeats": sum(s.repeats for s in scenarios),
        "passed": passed,
        "failed": failed,
        "planned": planned,
        "campaign_file": str(args.campaign_file) if args.campaign_file is not None else None,
        "campaign_spec": campaign_spec,
        "scenarios": [scenario.__dict__ for scenario in scenarios],
        "output_root": str(output_root),
        "campaign_dir": str(campaign_dir),
        "runs": runs,
    }

    summary_json, summary_txt, report_md = write_campaign_outputs(summary, campaign_dir)

    print("\n" + summary_txt.read_text(encoding="utf-8"), end="")
    print(f"Campaign summary JSON: {summary_json}")
    print(f"Campaign report Markdown: {report_md}")
    return 0 if status in {"passed", "planned"} else 1


if __name__ == "__main__":
    raise SystemExit(main())
