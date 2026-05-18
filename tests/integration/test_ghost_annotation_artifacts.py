#!/usr/bin/env python3
"""Integration test for ghost target annotation artifacts and MP4 export dry-run."""

from __future__ import annotations

import json
import shutil
import subprocess
import sys
from pathlib import Path


def write_config(path: Path, output_dir: Path) -> None:
    path.write_text(
        "\n".join(
            [
                "frame_source: synthetic",
                "ego_provider: frame_hint",
                "detector: scripted",
                "camera_stabilizer: null",
                "tracker: simple_centroid",
                "identity_resolver: appearance_only",
                "projector: flat_ground",
                "ghost_targets_enabled: true",
                "ghost_targets_scenario: person_pair_crossing",
                "world_model: in_memory",
                "fallback_map_frame_id: map_local_0001",
                "frame_annotator: ppm_sequence",
                f"annotation_output_path: {output_dir}",
                "annotation_output_fps: 5",
                "pipeline_timing_enabled: false",
                "mission_controller: disabled",
                "flight_command_sink: disabled",
                "",
            ]
        ),
        encoding="utf-8",
    )


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: test_ghost_annotation_artifacts.py <repo-root> <build-dir>", file=sys.stderr)
        return 2

    repo_root = Path(sys.argv[1]).resolve()
    build_dir = Path(sys.argv[2]).resolve()
    output_root = build_dir / "ghost-annotation-artifacts"
    annotation_dir = output_root / "annotations"
    config_path = output_root / "core_stack_ghost_targets_ci.yaml"
    output_mp4 = output_root / "ghost_targets.mp4"

    if output_root.exists():
        shutil.rmtree(output_root)
    output_root.mkdir(parents=True)
    write_config(config_path, annotation_dir)

    app = build_dir / "apps" / "dedalus_core_stack"
    if not app.exists():
        raise AssertionError(f"missing app binary: {app}")

    result = subprocess.run(
        [str(app), "--config", str(config_path), "--max-frames", "3"],
        cwd=repo_root,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )

    snapshot = json.loads(result.stdout)
    source_track_ids = {agent.get("source_track_id", "") for agent in snapshot.get("agents", [])}
    expected_tracks = {"ghost_person_001", "ghost_person_002", "ghost_car_001"}
    if not expected_tracks <= source_track_ids:
        raise AssertionError(f"snapshot missing ghost tracks: got {sorted(source_track_ids)}")

    manifest = annotation_dir / "manifest.txt"
    if not manifest.exists():
        raise AssertionError(f"missing annotation manifest: {manifest}")
    rows = manifest.read_text(encoding="utf-8").strip().splitlines()
    if len(rows) != 4:
        raise AssertionError(f"expected header + 3 manifest rows, got {len(rows)}")

    for index in range(1, 4):
        frame = annotation_dir / f"frame_{index:06d}.ppm"
        if not frame.exists() or frame.stat().st_size == 0:
            raise AssertionError(f"missing non-empty annotated frame: {frame}")
        if frame.read_bytes()[:2] != b"P6":
            raise AssertionError(f"annotated frame is not P6 PPM: {frame}")

    subprocess.run(
        [
            sys.executable,
            "scripts/export-ppm-sequence-to-mp4.py",
            "--annotation-dir",
            str(annotation_dir),
            "--output-mp4",
            str(output_mp4),
            "--dry-run",
        ],
        cwd=repo_root,
        check=True,
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
