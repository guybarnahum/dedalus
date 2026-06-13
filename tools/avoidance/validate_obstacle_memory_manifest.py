#!/usr/bin/env python3
"""Validate an obstacle-memory post-mission manifest.

The manifest is intentionally lightweight, so this validator checks:
- schema and expected mission/site identifiers
- selected merge path for site-map format
- artifact existence and size consistency
- full JSON artifact enabled/disabled semantics
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


EXPECTED_SCHEMA = "dedalus.obstacle_memory_manifest.v1"


EXPECTED_MERGE_PATHS = {
    "sqlite": "delta_jsonl_to_delta_sqlite_to_site_sqlite",
    "json": "full_json_to_site_json",
    "both": "delta_jsonl_to_delta_sqlite_to_site_sqlite_and_full_json_to_site_json",
    "sqlite-full-json": "full_json_to_site_sqlite_legacy",
}


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        value = json.load(f)
    if not isinstance(value, dict):
        raise ValueError(f"{path}: expected JSON object")
    return value


def require(condition: bool, message: str, errors: list[str]) -> None:
    if not condition:
        errors.append(message)


def artifact(manifest: dict[str, Any], name: str) -> dict[str, Any]:
    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, dict):
        return {}
    value = artifacts.get(name)
    if not isinstance(value, dict):
        return {}
    return value


def check_artifact(name: str, value: dict[str, Any], *, should_exist: bool, errors: list[str]) -> None:
    require(bool(value), f"missing artifact entry: {name}", errors)
    if not value:
        return

    path = value.get("path")
    exists = value.get("exists")
    size_bytes = value.get("size_bytes")

    require(isinstance(path, str) and bool(path), f"{name}.path missing/invalid", errors)
    require(isinstance(exists, bool), f"{name}.exists missing/invalid", errors)
    require(isinstance(size_bytes, int), f"{name}.size_bytes missing/invalid", errors)

    if should_exist:
        require(exists is True, f"{name} should exist", errors)
        require(isinstance(size_bytes, int) and size_bytes > 0, f"{name} should have size_bytes > 0", errors)
        if isinstance(path, str) and path:
            p = Path(path)
            require(p.exists(), f"{name}.path does not exist on disk: {path}", errors)
            if p.exists() and isinstance(size_bytes, int):
                require(p.stat().st_size == size_bytes, f"{name}.size_bytes mismatch: manifest={size_bytes}, disk={p.stat().st_size}", errors)
    else:
        require(exists is False, f"{name} should not exist", errors)


def validate(args: argparse.Namespace) -> int:
    path = Path(args.manifest)
    if not path.exists():
        print(json.dumps({"ok": False, "errors": [f"manifest does not exist: {path}"]}, indent=2))
        return 1

    try:
        manifest = load_json(path)
    except Exception as exc:
        print(json.dumps({"ok": False, "errors": [str(exc)]}, indent=2))
        return 1

    errors: list[str] = []

    require(manifest.get("schema") == EXPECTED_SCHEMA, f"schema mismatch: {manifest.get('schema')!r}", errors)

    if args.site_id:
        require(manifest.get("site_id") == args.site_id, f"site_id mismatch: {manifest.get('site_id')!r} != {args.site_id!r}", errors)
    if args.site_frame_id:
        require(manifest.get("site_frame_id") == args.site_frame_id, f"site_frame_id mismatch: {manifest.get('site_frame_id')!r} != {args.site_frame_id!r}", errors)
    if args.mission_id:
        require(manifest.get("mission_id") == args.mission_id, f"mission_id mismatch: {manifest.get('mission_id')!r} != {args.mission_id!r}", errors)

    site_map_format = manifest.get("site_map_format")
    if args.site_map_format:
        require(site_map_format == args.site_map_format, f"site_map_format mismatch: {site_map_format!r} != {args.site_map_format!r}", errors)

    expected_merge_path = EXPECTED_MERGE_PATHS.get(str(site_map_format))
    if expected_merge_path:
        require(manifest.get("merge_path") == expected_merge_path, f"merge_path mismatch: {manifest.get('merge_path')!r} != {expected_merge_path!r}", errors)

    full_json_enabled = manifest.get("full_json_enabled")
    require(isinstance(full_json_enabled, bool), "full_json_enabled missing/invalid", errors)

    # Required/default artifacts by format.
    if site_map_format == "sqlite":
        check_artifact("full_mission_json", artifact(manifest, "full_mission_json"), should_exist=False, errors=errors)
        check_artifact("delta_jsonl", artifact(manifest, "delta_jsonl"), should_exist=True, errors=errors)
        check_artifact("delta_sqlite", artifact(manifest, "delta_sqlite"), should_exist=True, errors=errors)
        check_artifact("site_sqlite", artifact(manifest, "site_sqlite"), should_exist=True, errors=errors)
    elif site_map_format == "json":
        check_artifact("full_mission_json", artifact(manifest, "full_mission_json"), should_exist=True, errors=errors)
        check_artifact("site_json", artifact(manifest, "site_json"), should_exist=True, errors=errors)
    elif site_map_format == "both":
        check_artifact("full_mission_json", artifact(manifest, "full_mission_json"), should_exist=True, errors=errors)
        check_artifact("delta_jsonl", artifact(manifest, "delta_jsonl"), should_exist=True, errors=errors)
        check_artifact("delta_sqlite", artifact(manifest, "delta_sqlite"), should_exist=True, errors=errors)
        check_artifact("site_sqlite", artifact(manifest, "site_sqlite"), should_exist=True, errors=errors)
        check_artifact("site_json", artifact(manifest, "site_json"), should_exist=True, errors=errors)
    elif site_map_format == "sqlite-full-json":
        check_artifact("full_mission_json", artifact(manifest, "full_mission_json"), should_exist=True, errors=errors)
        check_artifact("site_sqlite", artifact(manifest, "site_sqlite"), should_exist=True, errors=errors)
    else:
        errors.append(f"unsupported or missing site_map_format: {site_map_format!r}")

    result = {
        "ok": not errors,
        "schema": manifest.get("schema"),
        "site_id": manifest.get("site_id"),
        "site_frame_id": manifest.get("site_frame_id"),
        "mission_id": manifest.get("mission_id"),
        "site_map_format": site_map_format,
        "merge_path": manifest.get("merge_path"),
        "full_json_enabled": full_json_enabled,
        "errors": errors,
    }

    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if not errors else 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Validate obstacle memory manifest.")
    parser.add_argument("manifest", help="Path to obstacle_memory_manifest.json")
    parser.add_argument("--site-id", default="")
    parser.add_argument("--site-frame-id", default="")
    parser.add_argument("--mission-id", default="")
    parser.add_argument("--site-map-format", default="")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    return validate(args)


if __name__ == "__main__":
    main()
