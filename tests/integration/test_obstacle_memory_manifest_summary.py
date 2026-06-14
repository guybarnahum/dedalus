#!/usr/bin/env python3
"""Integration test for obstacle-memory manifest summary CLI."""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SUMMARY_TOOL = REPO_ROOT / "tools" / "avoidance" / "obstacle_memory_manifest_summary.py"


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


def make_manifest(tmp: Path) -> Path:
    manifest = {
        "schema": "dedalus.obstacle_memory_manifest.v1",
        "created_at_unix_ns": 123456789,
        "site_id": "summary_site",
        "site_frame_id": "airsim_world",
        "mission_id": "summary_mission",
        "site_map_format": "sqlite",
        "merge_path": "delta_jsonl_to_delta_sqlite_to_site_sqlite",
        "full_json_enabled": False,
        "artifacts": {
            "full_mission_json": missing_artifact(tmp / "mission_obstacle_map_full.json", "debug_full_mission_obstacle_map_json"),
            "delta_jsonl": write_artifact(tmp / "mission_obstacle_map_deltas.jsonl"),
            "delta_sqlite": write_artifact(tmp / "mission_obstacle_map_deltas.sqlite"),
            "site_sqlite": write_artifact(tmp / "maps" / "site_obstacle_map.sqlite"),
            "site_json": missing_artifact(tmp / "maps" / "site_obstacle_map.json", "debug_site_obstacle_map_json"),
        },
        "post_mission": {
            "log": str(tmp / "post_mission.log"),
            "script": str(tmp / "post_mission.sh"),
        },
    }

    path = tmp / "obstacle_memory_manifest.json"
    path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return path


def test_json_summary() -> None:
    with tempfile.TemporaryDirectory() as td:
        manifest_path = make_manifest(Path(td))
        proc = subprocess.run(
            [sys.executable, str(SUMMARY_TOOL), str(manifest_path), "--json"],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )

        if proc.returncode != 0:
            print(proc.stdout)
            print(proc.stderr, file=sys.stderr)
            raise AssertionError(f"summary tool failed: rc={proc.returncode}")

        value = json.loads(proc.stdout)
        assert value["schema"] == "dedalus.obstacle_memory_manifest.v1"
        assert value["site_id"] == "summary_site"
        assert value["mission_id"] == "summary_mission"
        assert value["site_map_format"] == "sqlite"
        assert value["merge_path"] == "delta_jsonl_to_delta_sqlite_to_site_sqlite"
        assert value["full_json_enabled"] is False
        assert value["artifacts"]["full_mission_json"]["exists"] is False
        assert value["artifacts"]["delta_jsonl"]["exists"] is True
        assert value["artifacts"]["delta_sqlite"]["exists"] is True
        assert value["artifacts"]["site_sqlite"]["exists"] is True


def test_human_summary() -> None:
    with tempfile.TemporaryDirectory() as td:
        manifest_path = make_manifest(Path(td))
        proc = subprocess.run(
            [sys.executable, str(SUMMARY_TOOL), str(manifest_path)],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )

        if proc.returncode != 0:
            print(proc.stdout)
            print(proc.stderr, file=sys.stderr)
            raise AssertionError(f"summary tool failed: rc={proc.returncode}")

        assert "Obstacle memory manifest" in proc.stdout
        assert "summary_site" in proc.stdout
        assert "summary_mission" in proc.stdout
        assert "delta_jsonl" in proc.stdout
        assert "site_sqlite" in proc.stdout


def main() -> int:
    test_json_summary()
    test_human_summary()
    print("obstacle_memory_manifest_summary integration tests passed")
    return 0


if __name__ == "__main__":
    main()
