#!/usr/bin/env python3
"""Report obstacle-detector dataflow evidence for a mission artifact directory.

This is intentionally read-only. It correlates bridge timing records, world
snapshots, sensing volumes, and obstacle evidence provenance so live AirSim
obstacle-sensing failures can be localized without guessing.
"""

from __future__ import annotations

import json
import sys
from collections import Counter
from pathlib import Path
from typing import Any, Iterable


def read_jsonl(path: Path) -> Iterable[dict[str, Any]]:
    try:
        lines = path.read_text(encoding="utf-8", errors="ignore").splitlines()
    except OSError:
        return
    for line in lines:
        line = line.strip()
        if not line or not line.startswith("{"):
            continue
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict):
            yield value


def read_json(path: Path) -> dict[str, Any] | None:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def print_bridge_depth_records(output_dir: Path) -> None:
    records: list[tuple[Path, dict[str, Any]]] = []
    for path in sorted(output_dir.rglob("*.jsonl")):
        for record in read_jsonl(path):
            if "include_depth" in record or "depth_valid_count" in record or "depth_width" in record:
                records.append((path, record))

    print("\nbridge depth records:")
    if not records:
        print("  none")
        return

    for path, record in records[-8:]:
        print(
            "  {path}: include_depth={include_depth} stride={stride} "
            "size={width}x{height} valid={valid} min={min_m} max={max_m} ego_bytes={ego_bytes}".format(
                path=path.relative_to(output_dir),
                include_depth=record.get("include_depth"),
                stride=record.get("depth_stride"),
                width=record.get("depth_width"),
                height=record.get("depth_height"),
                valid=record.get("depth_valid_count"),
                min_m=record.get("depth_min_m"),
                max_m=record.get("depth_max_m"),
                ego_bytes=record.get("ego_bytes"),
            )
        )


def print_snapshot_records(output_dir: Path) -> None:
    snapshots = sorted(output_dir.rglob("snapshot_*.json"))
    print(f"\nsnapshots: {len(snapshots)}")

    volume_provider_counts: Counter[str] = Counter()
    evidence_provider_counts: Counter[str] = Counter()
    evidence_kind_counts: Counter[str] = Counter()
    first_depth: str | None = None
    last_depth: str | None = None
    latest_rows: list[str] = []

    for path in snapshots:
        snapshot = read_json(path)
        if snapshot is None:
            continue

        for volume in snapshot.get("obstacle_sensing_volumes") or []:
            if isinstance(volume, dict):
                volume_provider_counts[str(volume.get("provider_name", ""))] += 1

        local_providers: Counter[str] = Counter()
        local_depth_count = 0
        for evidence in snapshot.get("obstacle_evidence") or []:
            if not isinstance(evidence, dict):
                continue
            provider = str(evidence.get("source_provider", ""))
            kind = str(evidence.get("source_kind", ""))
            evidence_provider_counts[provider] += 1
            evidence_kind_counts[kind] += 1
            local_providers[provider] += 1
            if provider == "airsim_depth_obstacle_detector":
                local_depth_count += 1

        if local_depth_count > 0:
            first_depth = first_depth or path.name
            last_depth = path.name

        if len(latest_rows) >= 8:
            latest_rows.pop(0)
        latest_rows.append(
            f"  {path.name}: evidence={sum(local_providers.values())} providers={dict(local_providers)}"
        )

    print("\nsensing volume providers:")
    if not volume_provider_counts:
        print("  none")
    for provider, count in volume_provider_counts.most_common():
        print(f"  {provider}: {count}")

    print("\nobstacle evidence providers:")
    if not evidence_provider_counts:
        print("  none")
    for provider, count in evidence_provider_counts.most_common():
        print(f"  {provider}: {count}")

    print("\nobstacle evidence source kinds:")
    if not evidence_kind_counts:
        print("  none")
    for kind, count in evidence_kind_counts.most_common():
        print(f"  {kind}: {count}")

    print(f"\ndepth evidence first={first_depth} last={last_depth}")

    print("\nlatest snapshot evidence rows:")
    if not latest_rows:
        print("  none")
    for row in latest_rows:
        print(row)


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: report-obstacle-detector-dataflow.py <mission_output_dir>", file=sys.stderr)
        return 2

    output_dir = Path(sys.argv[1])
    if not output_dir.exists():
        print(f"output directory not found: {output_dir}", file=sys.stderr)
        return 1

    print(f"output: {output_dir}")
    print_bridge_depth_records(output_dir)
    print_snapshot_records(output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
