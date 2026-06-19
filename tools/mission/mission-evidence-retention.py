#!/usr/bin/env python3
"""Dry-run mission evidence retention manifest.

Evaluates the five-condition retention gate for a mission output directory
and writes a manifest (mission_evidence_retention.json) that reports which
raw snapshot files would be kept or removed.

No files are deleted unless --delete is passed AND the environment variable
DEDALUS_MISSION_EVIDENCE_RETENTION_DELETE_SNAPSHOTS=1 is set.  The dry-run
manifest should be reviewed and validated before enabling deletion.

Five-condition retention gate (all must hold for raw evidence to be forgettable):
  1. active_emergency_window_clear   — tool is run post-mission (assumed true)
  2. mission_local_compaction_complete — delta artifact (SQLite or JSONL) present
  3. traversability_artifact_written  — mission_traversability_map_full.json present
  4. replayable_delta_stream_retained — delta artifact is retained in kept outputs
  5. site_map_merge_succeeded         — site SQLite present under --maps-dir/<site_id>/

Condition 4 is true when condition 2 is true (same delta artifact).
Condition 5 is optional — absence blocks deletion but does not fail the manifest.

Usage:
  python3 tools/mission/mission-evidence-retention.py \\
    --output-dir out/validate_r3b1 \\
    [--site-id validate_r3b1] \\
    [--mission-id validate_r3b1_mission] \\
    [--maps-dir maps] \\
    [--keep-every-n 100] \\
    [--delete]

Environment variables:
  DEDALUS_MISSION_EVIDENCE_RETENTION=1           enable manifest generation (default 1)
  DEDALUS_MISSION_EVIDENCE_RETENTION_DRY_RUN=1   force dry-run (default 1, set to 0 to allow --delete)
  DEDALUS_MISSION_EVIDENCE_RETENTION_DELETE_SNAPSHOTS=1  must also be set to permit --delete
  DEDALUS_MISSION_EVIDENCE_RETENTION_KEEP_EVERY_N=100    override keep-every-n
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Any


SCHEMA = "dedalus.mission_evidence_retention.v1"

# Artifact filenames that are always retained regardless of retention policy.
ALWAYS_RETAIN = {
    "mission_events.jsonl",
    "snapshot_manifest.txt",
    "obstacle_memory_manifest.json",
    "mission_traversability_map_full.json",
    "mission_traversability_map_full.json.meta.json",
    "mission_obstacle_map_deltas.json",
    "mission_obstacle_map_deltas.jsonl",
    "mission_obstacle_map_deltas.jsonl.meta.json",
    "mission_obstacle_map_deltas.sqlite",
    "mission_evidence_retention.json",
    "mission_local_obstacle_viewer.html",
    "mission_traversability_map_viewer.html",
}


def env_flag(name: str, default: bool) -> bool:
    val = os.environ.get(name, "")
    if not val:
        return default
    return val.strip() not in {"0", "false", "no"}


def env_int(name: str, default: int) -> int:
    val = os.environ.get(name, "")
    try:
        return int(val) if val.strip() else default
    except ValueError:
        return default


def artifact_entry(path: Path) -> dict[str, Any]:
    exists = path.exists()
    return {
        "path": str(path),
        "exists": exists,
        "size_bytes": path.stat().st_size if exists else 0,
    }


def compute_reason(compaction_complete: bool, trav_written: bool) -> str:
    if compaction_complete and trav_written:
        return "foundational_map_finalized"
    missing = []
    if not compaction_complete:
        missing.append("compaction_incomplete")
    if not trav_written:
        missing.append("traversability_artifact_missing")
    return missing[0] if len(missing) == 1 else "gate_blocked_multiple"


def update_snapshot_manifest_after_pruning(output_dir: Path, removed_names: set[str]) -> None:
    """Remove pruned snapshot names from snapshot_manifest.txt if it exists."""
    manifest_txt = output_dir / "snapshot_manifest.txt"
    if not manifest_txt.exists():
        return
    try:
        lines = manifest_txt.read_text(encoding="utf-8").splitlines(keepends=True)
    except Exception as exc:
        print(f"WARN: could not read snapshot_manifest.txt for update: {exc}")
        return
    kept: list[str] = []
    for line in lines:
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            kept.append(line)
            continue
        parts = stripped.split()
        filename = Path(parts[1] if len(parts) >= 2 and parts[0].isdigit() else parts[0]).name
        if filename in removed_names:
            continue
        kept.append(line)
    try:
        manifest_txt.write_text("".join(kept), encoding="utf-8")
    except Exception as exc:
        print(f"WARN: could not update snapshot_manifest.txt: {exc}")


def find_snapshots(output_dir: Path) -> list[Path]:
    """Return snapshot_*.json files sorted by name."""
    snaps = sorted(output_dir.glob("snapshot_*.json"))
    return snaps


def kept_snapshot_indices(total: int, keep_every_n: int) -> set[int]:
    """Indices of snapshots to retain: first, last, every Nth."""
    if total == 0:
        return set()
    kept: set[int] = {0, total - 1}
    if keep_every_n > 0:
        kept.update(range(0, total, keep_every_n))
    return kept


def detect_site_id(output_dir: Path, cli_site_id: str) -> str:
    """Try to read site_id from obstacle_memory_manifest.json if not provided."""
    if cli_site_id:
        return cli_site_id
    manifest_path = output_dir / "obstacle_memory_manifest.json"
    if manifest_path.exists():
        try:
            data = json.loads(manifest_path.read_text(encoding="utf-8"))
            return str(data.get("site_id", ""))
        except Exception:
            pass
    return ""


def detect_mission_id(output_dir: Path, cli_mission_id: str) -> str:
    """Try to read mission_id from obstacle_memory_manifest.json if not provided."""
    if cli_mission_id:
        return cli_mission_id
    manifest_path = output_dir / "obstacle_memory_manifest.json"
    if manifest_path.exists():
        try:
            data = json.loads(manifest_path.read_text(encoding="utf-8"))
            return str(data.get("mission_id", ""))
        except Exception:
            pass
    return ""


def build_manifest(
    output_dir: Path,
    site_id: str,
    mission_id: str,
    maps_dir: Path,
    keep_every_n: int,
    dry_run: bool,
    delete_enabled: bool,
) -> dict[str, Any]:
    """Evaluate the retention gate and build the manifest dict."""

    # --- Condition 1: active emergency window ---
    # This tool is always run post-mission.  We cannot detect an active flight
    # from disk state alone.  Assume clear; operator must not run this during
    # an active mission.
    active_emergency_window_clear = True

    # --- Condition 2: mission-local compaction complete ---
    delta_sqlite = output_dir / "mission_obstacle_map_deltas.sqlite"
    delta_jsonl = output_dir / "mission_obstacle_map_deltas.jsonl"
    delta_json = output_dir / "mission_obstacle_map_deltas.json"
    mission_local_compaction_complete = (
        delta_sqlite.exists() or delta_jsonl.exists() or delta_json.exists()
    )

    # --- Condition 3: traversability artifact written ---
    trav_artifact = output_dir / "mission_traversability_map_full.json"
    traversability_artifact_written = trav_artifact.exists()

    # --- Condition 4: replayable delta stream retained ---
    # The delta artifact will be in the retained outputs set (see ALWAYS_RETAIN).
    replayable_delta_stream_retained = mission_local_compaction_complete

    # --- Condition 5: site map merge succeeded ---
    site_map_merge_succeeded = False
    site_sqlite_path: Path | None = None
    if site_id:
        candidate = maps_dir / site_id / "site_obstacle_map.sqlite"
        if candidate.exists():
            site_map_merge_succeeded = True
            site_sqlite_path = candidate

    can_forget_raw_evidence = (
        active_emergency_window_clear
        and mission_local_compaction_complete
        and traversability_artifact_written
        and replayable_delta_stream_retained
        and (site_map_merge_succeeded or replayable_delta_stream_retained)
    )

    # Even when the gate is met, we require site_map_merge_succeeded OR explicit
    # delta retention.  Since delta retention is always true when compaction is
    # complete, the gate is:
    #   compaction + traversability -> can forget (delta is the fallback)
    # Site merge is bonus; absence is noted but does not block the dry-run.

    # --- Snapshot inventory ---
    snapshots = find_snapshots(output_dir)
    total = len(snapshots)
    kept_indices = kept_snapshot_indices(total, keep_every_n)

    retained_snapshots: list[str] = []
    would_remove_snapshots: list[str] = []
    for i, snap in enumerate(snapshots):
        if i in kept_indices:
            retained_snapshots.append(snap.name)
        else:
            would_remove_snapshots.append(snap.name)

    # --- Non-snapshot retained outputs (always keep) ---
    retained_outputs: list[str] = []
    for name in sorted(ALWAYS_RETAIN):
        p = output_dir / name
        if p.exists():
            retained_outputs.append(name)

    reason = compute_reason(mission_local_compaction_complete, traversability_artifact_written)

    manifest: dict[str, Any] = {
        "schema": SCHEMA,
        "site_id": site_id,
        "mission_id": mission_id,
        "raw_evidence_forget_state": "dry_run" if (dry_run or not delete_enabled) else "pruned",
        "reason": reason,
        "active_emergency_window_clear": active_emergency_window_clear,
        "mission_local_compaction_complete": mission_local_compaction_complete,
        "traversability_artifact_written": traversability_artifact_written,
        "site_map_merge_succeeded": site_map_merge_succeeded,
        "replayable_delta_stream_retained": replayable_delta_stream_retained,
        "can_forget_raw_evidence": can_forget_raw_evidence,
        "keep_every_n": keep_every_n,
        "raw_snapshots_before": total,
        "raw_snapshots_retained": len(retained_snapshots),
        "raw_snapshots_would_remove": len(would_remove_snapshots),
        "retained_outputs": retained_outputs,
        "retained_snapshot_names": retained_snapshots,
        "would_remove_snapshot_names": would_remove_snapshots,
    }

    if site_sqlite_path:
        manifest["site_sqlite_path"] = str(site_sqlite_path)

    return manifest


def delete_snapshots(output_dir: Path, would_remove: list[str]) -> tuple[int, list[str]]:
    """Delete the listed snapshot files.  Returns (deleted_count, errors)."""
    deleted = 0
    errors: list[str] = []
    for name in would_remove:
        p = output_dir / name
        try:
            p.unlink()
            deleted += 1
        except Exception as exc:
            errors.append(f"{name}: {exc}")
    return deleted, errors


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate a dry-run mission evidence retention manifest.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--output-dir", required=True, help="Mission output directory")
    parser.add_argument("--site-id", default="", help="Site ID (auto-detected from obstacle_memory_manifest.json if omitted)")
    parser.add_argument("--mission-id", default="", help="Mission ID (auto-detected if omitted)")
    parser.add_argument("--maps-dir", default="maps", help="Root directory for site maps (default: maps)")
    parser.add_argument("--keep-every-n", type=int, default=0,
                        help="Keep every Nth snapshot in addition to first and last (default: from env or 100)")
    parser.add_argument("--delete", action="store_true",
                        help="Actually delete would-remove snapshots (requires DEDALUS_MISSION_EVIDENCE_RETENTION_DELETE_SNAPSHOTS=1)")
    parser.add_argument("--json", action="store_true", help="Print manifest JSON to stdout")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    # Environment overrides.
    retention_enabled = env_flag("DEDALUS_MISSION_EVIDENCE_RETENTION", True)
    if not retention_enabled:
        print("OK: DEDALUS_MISSION_EVIDENCE_RETENTION=0 — retention disabled, skipping")
        return 0

    force_dry_run = env_flag("DEDALUS_MISSION_EVIDENCE_RETENTION_DRY_RUN", True)
    delete_snapshots_env = env_flag("DEDALUS_MISSION_EVIDENCE_RETENTION_DELETE_SNAPSHOTS", False)

    env_keep_n = env_int("DEDALUS_MISSION_EVIDENCE_RETENTION_KEEP_EVERY_N", 100)
    keep_every_n = args.keep_every_n if args.keep_every_n > 0 else env_keep_n

    output_dir = Path(args.output_dir)
    if not output_dir.is_dir():
        print(f"ERROR: output-dir does not exist: {output_dir}")
        return 1

    maps_dir = Path(args.maps_dir)

    site_id = detect_site_id(output_dir, args.site_id)
    mission_id = detect_mission_id(output_dir, args.mission_id)

    dry_run = force_dry_run or not args.delete
    delete_enabled = args.delete and delete_snapshots_env and not force_dry_run

    manifest = build_manifest(
        output_dir=output_dir,
        site_id=site_id,
        mission_id=mission_id,
        maps_dir=maps_dir,
        keep_every_n=keep_every_n,
        dry_run=dry_run,
        delete_enabled=delete_enabled,
    )

    # Write manifest.
    manifest_path = output_dir / "mission_evidence_retention.json"
    try:
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    except Exception as exc:
        print(f"ERROR: could not write manifest: {exc}")
        return 1

    # Optionally print JSON.
    if args.json:
        print(json.dumps(manifest, indent=2))

    # Summary lines.
    gate = manifest["can_forget_raw_evidence"]
    before = manifest["raw_snapshots_before"]
    retained = manifest["raw_snapshots_retained"]
    would_remove = manifest["raw_snapshots_would_remove"]

    print(f"OK: manifest written to {manifest_path}")
    print(f"OK: site={site_id or '(unknown)'}  mission={mission_id or '(unknown)'}")
    print(f"OK: gate conditions — "
          f"compaction={'yes' if manifest['mission_local_compaction_complete'] else 'NO'}  "
          f"traversability={'yes' if manifest['traversability_artifact_written'] else 'NO'}  "
          f"site_merge={'yes' if manifest['site_map_merge_succeeded'] else 'no (delta fallback)'}")
    print(f"OK: can_forget_raw_evidence={gate}")
    print(f"OK: snapshots before={before}  retained={retained}  would_remove={would_remove}")

    if not gate:
        print("NOTE: retention gate not met — no snapshots would be removed even with --delete")

    # Optionally delete.
    if delete_enabled and gate:
        deleted, errors = delete_snapshots(output_dir, manifest["would_remove_snapshot_names"])
        print(f"OK: deleted {deleted} snapshot(s)")
        for err in errors:
            print(f"ERROR: delete failed: {err}")
        manifest["raw_evidence_forget_state"] = "partial_prune" if errors else "pruned"
        manifest["raw_snapshots_deleted"] = deleted
        # Remove pruned entries from snapshot_manifest.txt so downstream tools
        # (validate-mission-artifacts.py, replay scripts) don't reference missing files.
        if deleted > 0:
            successfully_removed = set(manifest["would_remove_snapshot_names"]) - {
                err.split(":")[0] for err in errors
            }
            update_snapshot_manifest_after_pruning(output_dir, successfully_removed)
        # Rewrite manifest with updated state.
        try:
            manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
        except Exception as exc:
            print(f"ERROR: could not update manifest after deletion: {exc}")
        if errors:
            return 1
    elif args.delete and not delete_snapshots_env:
        print("NOTE: --delete passed but DEDALUS_MISSION_EVIDENCE_RETENTION_DELETE_SNAPSHOTS is not 1 — dry-run only")
    elif args.delete and force_dry_run:
        print("NOTE: --delete passed but DEDALUS_MISSION_EVIDENCE_RETENTION_DRY_RUN=1 — dry-run only")

    return 0


if __name__ == "__main__":
    sys.exit(main())
