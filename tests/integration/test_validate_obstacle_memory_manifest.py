#!/usr/bin/env python3
"""Integration coverage for obstacle-memory manifest validation formats."""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
VALIDATOR = REPO_ROOT / "tools" / "avoidance" / "validate_obstacle_memory_manifest.py"


def write_artifact(path: Path, text: str = "artifact\n") -> dict:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    return {
        "kind": path.stem,
        "path": str(path),
        "exists": True,
        "size_bytes": path.stat().st_size,
    }


def missing_artifact(path: Path, kind: str) -> dict:
    return {
        "kind": kind,
        "path": str(path),
        "exists": False,
        "size_bytes": 0,
    }


def base_manifest(tmp: Path, site_map_format: str, merge_path: str, *, full_json_enabled: bool) -> dict:
    full_json = tmp / "mission_obstacle_map_full.json"
    delta_jsonl = tmp / "mission_obstacle_map_deltas.jsonl"
    delta_sqlite = tmp / "mission_obstacle_map_deltas.sqlite"
    site_sqlite = tmp / "maps" / "site_obstacle_map.sqlite"
    site_json = tmp / "maps" / "site_obstacle_map.json"

    artifacts = {
        "full_mission_json": missing_artifact(full_json, "debug_full_mission_obstacle_map_json"),
        "delta_jsonl": missing_artifact(delta_jsonl, "compact_mission_obstacle_delta_jsonl"),
        "delta_sqlite": missing_artifact(delta_sqlite, "mission_obstacle_delta_sqlite"),
        "site_sqlite": missing_artifact(site_sqlite, "persistent_site_obstacle_map_sqlite"),
        "site_json": missing_artifact(site_json, "debug_site_obstacle_map_json"),
    }

    if site_map_format == "sqlite":
        artifacts["delta_jsonl"] = write_artifact(delta_jsonl)
        artifacts["delta_sqlite"] = write_artifact(delta_sqlite)
        artifacts["site_sqlite"] = write_artifact(site_sqlite)
    elif site_map_format == "json":
        artifacts["full_mission_json"] = write_artifact(full_json)
        artifacts["site_json"] = write_artifact(site_json)
    elif site_map_format == "both":
        artifacts["full_mission_json"] = write_artifact(full_json)
        artifacts["delta_jsonl"] = write_artifact(delta_jsonl)
        artifacts["delta_sqlite"] = write_artifact(delta_sqlite)
        artifacts["site_sqlite"] = write_artifact(site_sqlite)
        artifacts["site_json"] = write_artifact(site_json)
    elif site_map_format == "sqlite-full-json":
        artifacts["full_mission_json"] = write_artifact(full_json)
        artifacts["site_sqlite"] = write_artifact(site_sqlite)
    else:
        raise AssertionError(site_map_format)

    return {
        "schema": "dedalus.obstacle_memory_manifest.v1",
        "created_at_unix_ns": 123456789,
        "site_id": "contract_site",
        "site_frame_id": "airsim_world",
        "mission_id": "contract_mission",
        "site_map_format": site_map_format,
        "merge_path": merge_path,
        "full_json_enabled": full_json_enabled,
        "artifacts": artifacts,
        "post_mission": {
            "log": str(tmp / "post_mission.log"),
            "script": str(tmp / "post_mission.sh"),
        },
    }


def run_validator(manifest_path: Path, site_map_format: str) -> dict:
    proc = subprocess.run(
        [
            sys.executable,
            str(VALIDATOR),
            str(manifest_path),
            "--site-id",
            "contract_site",
            "--site-frame-id",
            "airsim_world",
            "--mission-id",
            "contract_mission",
            "--site-map-format",
            site_map_format,
        ],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )

    if proc.returncode != 0:
        print(proc.stdout)
        print(proc.stderr, file=sys.stderr)
        raise AssertionError(f"validator failed for {site_map_format}: rc={proc.returncode}")

    value = json.loads(proc.stdout)
    assert value["ok"] is True
    assert value["errors"] == []
    assert value["site_map_format"] == site_map_format
    return value


def write_manifest(tmp: Path, manifest: dict) -> Path:
    path = tmp / "obstacle_memory_manifest.json"
    path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return path


def test_sqlite_manifest_contract() -> None:
    with tempfile.TemporaryDirectory() as td:
        tmp = Path(td)
        manifest = base_manifest(
            tmp,
            "sqlite",
            "delta_jsonl_to_delta_sqlite_to_site_sqlite",
            full_json_enabled=False,
        )
        value = run_validator(write_manifest(tmp, manifest), "sqlite")
        assert value["merge_path"] == "delta_jsonl_to_delta_sqlite_to_site_sqlite"
        assert value["full_json_enabled"] is False


def test_json_manifest_contract() -> None:
    with tempfile.TemporaryDirectory() as td:
        tmp = Path(td)
        manifest = base_manifest(
            tmp,
            "json",
            "full_json_to_site_json",
            full_json_enabled=True,
        )
        value = run_validator(write_manifest(tmp, manifest), "json")
        assert value["merge_path"] == "full_json_to_site_json"
        assert value["full_json_enabled"] is True


def test_both_manifest_contract() -> None:
    with tempfile.TemporaryDirectory() as td:
        tmp = Path(td)
        manifest = base_manifest(
            tmp,
            "both",
            "delta_jsonl_to_delta_sqlite_to_site_sqlite_and_full_json_to_site_json",
            full_json_enabled=True,
        )
        value = run_validator(write_manifest(tmp, manifest), "both")
        assert value["merge_path"] == "delta_jsonl_to_delta_sqlite_to_site_sqlite_and_full_json_to_site_json"
        assert value["full_json_enabled"] is True


def test_sqlite_full_json_manifest_contract() -> None:
    with tempfile.TemporaryDirectory() as td:
        tmp = Path(td)
        manifest = base_manifest(
            tmp,
            "sqlite-full-json",
            "full_json_to_site_sqlite_legacy",
            full_json_enabled=True,
        )
        value = run_validator(write_manifest(tmp, manifest), "sqlite-full-json")
        assert value["merge_path"] == "full_json_to_site_sqlite_legacy"
        assert value["full_json_enabled"] is True


def main() -> int:
    test_sqlite_manifest_contract()
    test_json_manifest_contract()
    test_both_manifest_contract()
    test_sqlite_full_json_manifest_contract()
    print("validate_obstacle_memory_manifest integration coverage passed")
    return 0


if __name__ == "__main__":
    main()
