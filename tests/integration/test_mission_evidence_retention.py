#!/usr/bin/env python3
"""Integration tests for tools/mission/mission-evidence-retention.py.

Each test creates a synthetic mission output directory, runs the script via
subprocess, and asserts on returncode, stdout, and filesystem state.

Run: python3 tests/integration/test_mission_evidence_retention.py [repo_root]
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path


SCRIPT = Path("tools") / "mission" / "mission-evidence-retention.py"


def run_script(
    repo_root: Path,
    output_dir: Path,
    *extra_args: str,
    env_overrides: dict[str, str] | None = None,
) -> subprocess.CompletedProcess[str]:
    env = {**os.environ}
    if env_overrides:
        env.update(env_overrides)
    return subprocess.run(
        [sys.executable, str(repo_root / SCRIPT), "--output-dir", str(output_dir), "--json", *extra_args],
        cwd=repo_root,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        env=env,
    )


def seed_snapshots(run_dir: Path, count: int) -> list[str]:
    """Write count snapshot_NNNN.json files; return their names."""
    names: list[str] = []
    for i in range(1, count + 1):
        name = f"snapshot_{i:04d}.json"
        (run_dir / name).write_text(json.dumps({"tick": i}), encoding="utf-8")
        names.append(name)
    return names


def write_snapshot_manifest(run_dir: Path, names: list[str]) -> None:
    lines = "\n".join(f"{i + 1} {name}" for i, name in enumerate(names)) + "\n"
    (run_dir / "snapshot_manifest.txt").write_text(lines, encoding="utf-8")


def write_obstacle_memory_manifest(run_dir: Path, site_id: str = "", mission_id: str = "") -> None:
    (run_dir / "obstacle_memory_manifest.json").write_text(
        json.dumps({"site_id": site_id, "mission_id": mission_id}), encoding="utf-8"
    )


def write_delta_artifact(run_dir: Path) -> None:
    (run_dir / "mission_obstacle_map_deltas.jsonl").write_text(
        '{"schema":"dedalus.obstacle_map_delta.v1","cells":[]}\n', encoding="utf-8"
    )


def write_traversability_artifact(run_dir: Path) -> None:
    (run_dir / "mission_traversability_map_full.json").write_text(
        json.dumps({"schema": "dedalus.mission_local_traversability_map.v1", "cells": []}),
        encoding="utf-8",
    )


def read_manifest_json(run_dir: Path) -> dict:
    return json.loads((run_dir / "mission_evidence_retention.json").read_text(encoding="utf-8"))


def assert_ok(result: subprocess.CompletedProcess, label: str) -> None:
    if result.returncode != 0:
        print(result.stdout)
        print(result.stderr, file=sys.stderr)
        raise AssertionError(f"{label}: expected returncode 0, got {result.returncode}")


def assert_fail(result: subprocess.CompletedProcess, label: str) -> None:
    if result.returncode == 0:
        print(result.stdout)
        raise AssertionError(f"{label}: expected non-zero returncode, got 0")


# ---------------------------------------------------------------------------
# Test 1: gate not met — no artifacts at all
# ---------------------------------------------------------------------------
def test_gate_not_met_no_artifacts(tmp: Path, repo: Path) -> None:
    run_dir = tmp / "t1"
    run_dir.mkdir()
    seed_snapshots(run_dir, 5)
    (run_dir / "mission_events.jsonl").write_text("")

    result = run_script(repo, run_dir, "--keep-every-n", "2")
    assert_ok(result, "t1")

    m = read_manifest_json(run_dir)
    assert m["can_forget_raw_evidence"] is False
    assert m["mission_local_compaction_complete"] is False
    assert m["traversability_artifact_written"] is False
    assert m["reason"] == "gate_blocked_multiple"
    assert m["raw_evidence_forget_state"] == "dry_run"
    assert m["raw_snapshots_before"] == 5
    assert "snapshot_0001.json" in m["retained_snapshot_names"]
    assert "snapshot_0005.json" in m["retained_snapshot_names"]
    print("PASS test_gate_not_met_no_artifacts")


# ---------------------------------------------------------------------------
# Test 2: gate not met — only delta present (traversability missing)
# ---------------------------------------------------------------------------
def test_gate_not_met_missing_traversability(tmp: Path, repo: Path) -> None:
    run_dir = tmp / "t2"
    run_dir.mkdir()
    seed_snapshots(run_dir, 3)
    write_delta_artifact(run_dir)

    result = run_script(repo, run_dir)
    assert_ok(result, "t2")

    m = read_manifest_json(run_dir)
    assert m["can_forget_raw_evidence"] is False
    assert m["mission_local_compaction_complete"] is True
    assert m["traversability_artifact_written"] is False
    assert m["reason"] == "traversability_artifact_missing"
    print("PASS test_gate_not_met_missing_traversability")


# ---------------------------------------------------------------------------
# Test 3: gate not met — only traversability present (compaction missing)
# ---------------------------------------------------------------------------
def test_gate_not_met_missing_compaction(tmp: Path, repo: Path) -> None:
    run_dir = tmp / "t3"
    run_dir.mkdir()
    seed_snapshots(run_dir, 3)
    write_traversability_artifact(run_dir)

    result = run_script(repo, run_dir)
    assert_ok(result, "t3")

    m = read_manifest_json(run_dir)
    assert m["can_forget_raw_evidence"] is False
    assert m["mission_local_compaction_complete"] is False
    assert m["traversability_artifact_written"] is True
    assert m["reason"] == "compaction_incomplete"
    print("PASS test_gate_not_met_missing_compaction")


# ---------------------------------------------------------------------------
# Test 4: gate met, dry run — both artifacts present, no deletion
# ---------------------------------------------------------------------------
def test_gate_met_dry_run(tmp: Path, repo: Path) -> None:
    run_dir = tmp / "t4"
    run_dir.mkdir()
    names = seed_snapshots(run_dir, 10)
    write_snapshot_manifest(run_dir, names)
    write_delta_artifact(run_dir)
    write_traversability_artifact(run_dir)

    result = run_script(repo, run_dir, "--keep-every-n", "3")
    assert_ok(result, "t4")

    m = read_manifest_json(run_dir)
    assert m["can_forget_raw_evidence"] is True
    assert m["reason"] == "foundational_map_finalized"
    assert m["raw_evidence_forget_state"] == "dry_run"
    # Files must still exist (dry run)
    for name in m["would_remove_snapshot_names"]:
        assert (run_dir / name).exists(), f"dry run must not delete {name}"
    print("PASS test_gate_met_dry_run")


# ---------------------------------------------------------------------------
# Test 5: --delete without env var → no deletion (safety interlock)
# ---------------------------------------------------------------------------
def test_delete_without_env_var(tmp: Path, repo: Path) -> None:
    run_dir = tmp / "t5"
    run_dir.mkdir()
    names = seed_snapshots(run_dir, 5)
    write_delta_artifact(run_dir)
    write_traversability_artifact(run_dir)

    env = {
        "DEDALUS_MISSION_EVIDENCE_RETENTION_DELETE_SNAPSHOTS": "0",
        "DEDALUS_MISSION_EVIDENCE_RETENTION_DRY_RUN": "0",
    }
    result = run_script(repo, run_dir, "--delete", env_overrides=env)
    assert_ok(result, "t5")

    m = read_manifest_json(run_dir)
    assert m["raw_evidence_forget_state"] == "dry_run"
    for name in names:
        assert (run_dir / name).exists(), f"interlock must not delete {name}"
    assert "DEDALUS_MISSION_EVIDENCE_RETENTION_DELETE_SNAPSHOTS is not 1" in result.stdout
    print("PASS test_delete_without_env_var")


# ---------------------------------------------------------------------------
# Test 6: --delete + env var but DRY_RUN=1 → no deletion
# ---------------------------------------------------------------------------
def test_delete_dry_run_override(tmp: Path, repo: Path) -> None:
    run_dir = tmp / "t6"
    run_dir.mkdir()
    names = seed_snapshots(run_dir, 5)
    write_delta_artifact(run_dir)
    write_traversability_artifact(run_dir)

    env = {
        "DEDALUS_MISSION_EVIDENCE_RETENTION_DELETE_SNAPSHOTS": "1",
        "DEDALUS_MISSION_EVIDENCE_RETENTION_DRY_RUN": "1",
    }
    result = run_script(repo, run_dir, "--delete", env_overrides=env)
    assert_ok(result, "t6")

    m = read_manifest_json(run_dir)
    assert m["raw_evidence_forget_state"] == "dry_run"
    for name in names:
        assert (run_dir / name).exists(), f"dry_run override must not delete {name}"
    print("PASS test_delete_dry_run_override")


# ---------------------------------------------------------------------------
# Test 7: actual deletion — files removed, manifest updated, snapshot_manifest.txt pruned
# ---------------------------------------------------------------------------
def test_delete_actual(tmp: Path, repo: Path) -> None:
    run_dir = tmp / "t7"
    run_dir.mkdir()
    names = seed_snapshots(run_dir, 10)
    write_snapshot_manifest(run_dir, names)
    write_delta_artifact(run_dir)
    write_traversability_artifact(run_dir)

    env = {
        "DEDALUS_MISSION_EVIDENCE_RETENTION_DELETE_SNAPSHOTS": "1",
        "DEDALUS_MISSION_EVIDENCE_RETENTION_DRY_RUN": "0",
    }
    result = run_script(repo, run_dir, "--delete", "--keep-every-n", "5", env_overrides=env)
    assert_ok(result, "t7")

    m = read_manifest_json(run_dir)
    assert m["raw_evidence_forget_state"] == "pruned", f"got {m['raw_evidence_forget_state']}"
    assert "raw_snapshots_deleted" in m
    assert m["raw_snapshots_deleted"] > 0

    # Would-remove files must be gone
    for name in m["would_remove_snapshot_names"]:
        assert not (run_dir / name).exists(), f"expected {name} to be deleted"

    # Retained snapshots must still exist
    for name in m["retained_snapshot_names"]:
        assert (run_dir / name).exists(), f"expected retained {name} to exist"

    # snapshot_manifest.txt must not reference deleted files
    manifest_txt = (run_dir / "snapshot_manifest.txt").read_text(encoding="utf-8")
    removed_set = set(m["would_remove_snapshot_names"])
    for line in manifest_txt.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        parts = stripped.split()
        fname = Path(parts[1] if len(parts) >= 2 and parts[0].isdigit() else parts[0]).name
        assert fname not in removed_set, f"snapshot_manifest.txt still references deleted {fname}"

    print(f"PASS test_delete_actual (deleted={m['raw_snapshots_deleted']})")


# ---------------------------------------------------------------------------
# Test 8: keep-every-n logic — verify exact retained set
# ---------------------------------------------------------------------------
def test_keep_every_n(tmp: Path, repo: Path) -> None:
    run_dir = tmp / "t8"
    run_dir.mkdir()
    seed_snapshots(run_dir, 10)

    result = run_script(repo, run_dir, "--keep-every-n", "3")
    assert_ok(result, "t8")

    m = read_manifest_json(run_dir)
    kept = set(m["retained_snapshot_names"])
    # Indices 0,3,6,9 (0-based) → names 1,4,7,10 (1-based filenames)
    # Plus first (idx 0 → 0001) and last (idx 9 → 0010).
    assert "snapshot_0001.json" in kept, "first must be retained"
    assert "snapshot_0010.json" in kept, "last must be retained"
    assert "snapshot_0004.json" in kept, "every-3rd (index 3) must be retained"
    assert "snapshot_0007.json" in kept, "every-3rd (index 6) must be retained"
    # Index 1 (0002) must NOT be retained
    assert "snapshot_0002.json" not in kept
    print("PASS test_keep_every_n")


# ---------------------------------------------------------------------------
# Test 9: auto-detect site_id and mission_id
# ---------------------------------------------------------------------------
def test_auto_detect_ids(tmp: Path, repo: Path) -> None:
    run_dir = tmp / "t9"
    run_dir.mkdir()
    seed_snapshots(run_dir, 2)
    write_obstacle_memory_manifest(run_dir, site_id="site_alpha", mission_id="mission_001")

    result = run_script(repo, run_dir)
    assert_ok(result, "t9")

    m = read_manifest_json(run_dir)
    assert m["site_id"] == "site_alpha", f"expected site_alpha, got {m['site_id']}"
    assert m["mission_id"] == "mission_001", f"expected mission_001, got {m['mission_id']}"
    assert "site=site_alpha" in result.stdout
    assert "mission=mission_001" in result.stdout
    print("PASS test_auto_detect_ids")


# ---------------------------------------------------------------------------
# Test 10: manifest JSON schema fields
# ---------------------------------------------------------------------------
def test_manifest_schema(tmp: Path, repo: Path) -> None:
    run_dir = tmp / "t10"
    run_dir.mkdir()
    seed_snapshots(run_dir, 3)
    write_delta_artifact(run_dir)
    write_traversability_artifact(run_dir)

    result = run_script(repo, run_dir)
    assert_ok(result, "t10")

    m = read_manifest_json(run_dir)
    required_fields = [
        "schema", "site_id", "mission_id", "raw_evidence_forget_state", "reason",
        "active_emergency_window_clear", "mission_local_compaction_complete",
        "traversability_artifact_written", "site_map_merge_succeeded",
        "replayable_delta_stream_retained", "can_forget_raw_evidence",
        "keep_every_n", "raw_snapshots_before", "raw_snapshots_retained",
        "raw_snapshots_would_remove", "retained_outputs",
        "retained_snapshot_names", "would_remove_snapshot_names",
    ]
    for field in required_fields:
        assert field in m, f"manifest missing field: {field}"
    assert m["schema"] == "dedalus.mission_evidence_retention.v1"
    assert isinstance(m["retained_outputs"], list)
    assert isinstance(m["retained_snapshot_names"], list)
    assert isinstance(m["would_remove_snapshot_names"], list)
    print("PASS test_manifest_schema")


# ---------------------------------------------------------------------------
# Test 11: reason field correctness
# ---------------------------------------------------------------------------
def test_reason_field(tmp: Path, repo: Path) -> None:
    cases = [
        (False, False, "gate_blocked_multiple"),
        (True, False, "traversability_artifact_missing"),
        (False, True, "compaction_incomplete"),
        (True, True, "foundational_map_finalized"),
    ]
    for i, (has_delta, has_trav, expected_reason) in enumerate(cases):
        run_dir = tmp / f"t11_{i}"
        run_dir.mkdir()
        seed_snapshots(run_dir, 2)
        if has_delta:
            write_delta_artifact(run_dir)
        if has_trav:
            write_traversability_artifact(run_dir)

        result = run_script(repo, run_dir)
        assert_ok(result, f"t11_{i}")
        m = read_manifest_json(run_dir)
        assert m["reason"] == expected_reason, (
            f"t11_{i}: delta={has_delta} trav={has_trav}: "
            f"expected reason={expected_reason!r}, got {m['reason']!r}"
        )
    print("PASS test_reason_field")


# ---------------------------------------------------------------------------
# Test 12: missing output-dir → exit 1
# ---------------------------------------------------------------------------
def test_missing_output_dir(tmp: Path, repo: Path) -> None:
    result = run_script(repo, tmp / "nonexistent_dir_xyz")
    assert_fail(result, "t12")
    print("PASS test_missing_output_dir")


# ---------------------------------------------------------------------------
# Test 13: DEDALUS_MISSION_EVIDENCE_RETENTION=0 → skip (exit 0, no manifest)
# ---------------------------------------------------------------------------
def test_retention_disabled(tmp: Path, repo: Path) -> None:
    run_dir = tmp / "t13"
    run_dir.mkdir()
    seed_snapshots(run_dir, 2)

    result = run_script(repo, run_dir, env_overrides={"DEDALUS_MISSION_EVIDENCE_RETENTION": "0"})
    assert_ok(result, "t13")
    assert not (run_dir / "mission_evidence_retention.json").exists()
    print("PASS test_retention_disabled")


# ---------------------------------------------------------------------------
# Real mission output smoke test
# ---------------------------------------------------------------------------
def test_real_ci_smoke_output(repo: Path) -> None:
    """Run against the real ci_smoke run_0001 output if it exists."""
    run_dir = repo / "out" / "test_mission_scenarios" / "ci_smoke" / "run_0001"
    if not run_dir.is_dir():
        print("SKIP test_real_ci_smoke_output (directory not found)")
        return

    result = run_script(repo, run_dir, "--keep-every-n", "10")
    if result.returncode != 0:
        print(result.stdout)
        print(result.stderr, file=sys.stderr)
        raise AssertionError("real ci_smoke smoke test failed")

    m = read_manifest_json(run_dir)
    # ci_smoke has no delta or traversability artifact — gate should be blocked
    assert isinstance(m["can_forget_raw_evidence"], bool)
    assert m["raw_snapshots_before"] >= 1
    # Manifest is always written even when gate is not met
    assert m["schema"] == "dedalus.mission_evidence_retention.v1"
    print(f"PASS test_real_ci_smoke_output (snapshots={m['raw_snapshots_before']} gate={m['can_forget_raw_evidence']})")


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------
def main() -> int:
    repo = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[2]

    failures: list[str] = []

    with tempfile.TemporaryDirectory() as tmp_str:
        tmp = Path(tmp_str)

        tests = [
            lambda: test_gate_not_met_no_artifacts(tmp, repo),
            lambda: test_gate_not_met_missing_traversability(tmp, repo),
            lambda: test_gate_not_met_missing_compaction(tmp, repo),
            lambda: test_gate_met_dry_run(tmp, repo),
            lambda: test_delete_without_env_var(tmp, repo),
            lambda: test_delete_dry_run_override(tmp, repo),
            lambda: test_delete_actual(tmp, repo),
            lambda: test_keep_every_n(tmp, repo),
            lambda: test_auto_detect_ids(tmp, repo),
            lambda: test_manifest_schema(tmp, repo),
            lambda: test_reason_field(tmp, repo),
            lambda: test_missing_output_dir(tmp, repo),
            lambda: test_retention_disabled(tmp, repo),
        ]

        for test_fn in tests:
            try:
                test_fn()
            except AssertionError as exc:
                print(f"FAIL: {exc}", file=sys.stderr)
                failures.append(str(exc))

    # Real output smoke test outside tempdir
    try:
        test_real_ci_smoke_output(repo)
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        failures.append(str(exc))

    if failures:
        print(f"\n{len(failures)} test(s) failed.", file=sys.stderr)
        return 1

    print("\nAll retention tests passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
