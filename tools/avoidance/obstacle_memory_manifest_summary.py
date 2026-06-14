#!/usr/bin/env python3
"""Print a concise obstacle-memory manifest summary."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


def load_manifest(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        value = json.load(f)
    if not isinstance(value, dict):
        raise ValueError(f"{path}: expected JSON object")
    return value


def artifact_summary(value: dict[str, Any]) -> dict[str, Any]:
    path = str(value.get("path", ""))
    exists = bool(value.get("exists", False))
    size_bytes = int(value.get("size_bytes", 0) or 0)
    return {
        "path": path,
        "exists": exists,
        "size_bytes": size_bytes,
    }


def human_size(size_bytes: int) -> str:
    value = float(size_bytes)
    for unit in ["B", "KiB", "MiB", "GiB"]:
        if value < 1024.0 or unit == "GiB":
            if unit == "B":
                return f"{int(value)} {unit}"
            return f"{value:.1f} {unit}"
        value /= 1024.0
    return f"{size_bytes} B"


def build_summary(manifest: dict[str, Any]) -> dict[str, Any]:
    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, dict):
        artifacts = {}

    names = [
        "full_mission_json",
        "delta_jsonl",
        "delta_sqlite",
        "site_sqlite",
        "site_json",
    ]

    return {
        "schema": manifest.get("schema"),
        "site_id": manifest.get("site_id"),
        "site_frame_id": manifest.get("site_frame_id"),
        "mission_id": manifest.get("mission_id"),
        "site_map_format": manifest.get("site_map_format"),
        "merge_path": manifest.get("merge_path"),
        "full_json_enabled": bool(manifest.get("full_json_enabled", False)),
        "artifacts": {
            name: artifact_summary(artifacts.get(name, {}) if isinstance(artifacts.get(name, {}), dict) else {})
            for name in names
        },
        "post_mission": manifest.get("post_mission", {}) if isinstance(manifest.get("post_mission", {}), dict) else {},
    }


def print_human(summary: dict[str, Any]) -> None:
    print("Obstacle memory manifest")
    print(f"  schema:             {summary.get('schema')}")
    print(f"  site:               {summary.get('site_id')} ({summary.get('site_frame_id')})")
    print(f"  mission:            {summary.get('mission_id')}")
    print(f"  format:             {summary.get('site_map_format')}")
    print(f"  merge path:         {summary.get('merge_path')}")
    print(f"  full JSON enabled:  {summary.get('full_json_enabled')}")
    print("  artifacts:")

    artifacts = summary.get("artifacts", {})
    if not isinstance(artifacts, dict):
        artifacts = {}

    for name in [
        "full_mission_json",
        "delta_jsonl",
        "delta_sqlite",
        "site_sqlite",
        "site_json",
    ]:
        item = artifacts.get(name, {})
        if not isinstance(item, dict):
            item = {}
        exists = bool(item.get("exists", False))
        size_bytes = int(item.get("size_bytes", 0) or 0)
        path = item.get("path", "")
        status = "present" if exists else "missing"
        print(f"    {name:<18} {status:<7} {human_size(size_bytes):>9}  {path}")

    post_mission = summary.get("post_mission", {})
    if isinstance(post_mission, dict) and post_mission:
        print("  post-mission:")
        if post_mission.get("log"):
            print(f"    log:              {post_mission.get('log')}")
        if post_mission.get("script"):
            print(f"    script:           {post_mission.get('script')}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Summarize obstacle memory manifest.")
    parser.add_argument("manifest", help="Path to obstacle_memory_manifest.json")
    parser.add_argument("--json", action="store_true", help="Emit compact JSON summary")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    manifest_path = Path(args.manifest)
    if not manifest_path.exists():
        print(f"ERROR: manifest does not exist: {manifest_path}")
        return 1

    try:
        manifest = load_manifest(manifest_path)
        summary = build_summary(manifest)
    except Exception as exc:
        print(f"ERROR: {exc}")
        return 1

    if args.json:
        print(json.dumps(summary, indent=2, sort_keys=True))
    else:
        print_human(summary)

    return 0


if __name__ == "__main__":
    main()
