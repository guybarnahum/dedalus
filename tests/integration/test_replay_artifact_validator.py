#!/usr/bin/env python3
"""Integration test for scripts/validate-replay-artifacts.py."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: test_replay_artifact_validator.py <repo-root> <build-dir>", file=sys.stderr)
        return 2

    repo_root = Path(sys.argv[1]).resolve()
    build_dir = Path(sys.argv[2]).resolve()
    app = build_dir / "apps" / "dedalus_replay_recording"
    output_root = build_dir / "replay-artifact-validator"
    snapshot_dir = output_root / "snapshots"
    annotation_dir = output_root / "annotations"
    profile_jsonl = output_root / "pipeline_profile.jsonl"
    config_path = output_root / "core_stack_recorded_ppm_profile.yaml"

    output_root.mkdir(parents=True, exist_ok=True)
    config_path.write_text(
        "\n".join(
            [
                "frame_source: recorded_frames",
                "recorded_manifest_path: tests/fixtures/recorded_frames/manifest.txt",
                "ego_provider: no_telemetry",
                "detector: scripted",
                "camera_stabilizer: null",
                "tracker: simple_centroid",
                "identity_resolver: appearance_only",
                "projector: flat_ground",
                "world_model: in_memory",
                "frame_annotator: ppm_sequence",
                f"annotation_output_path: {annotation_dir}",
                "annotation_output_fps: 5",
                "pipeline_timing_enabled: true",
                f"pipeline_timing_output_path: {profile_jsonl}",
                "fallback_map_frame_id: map_recorded_ci_0001",
                "",
            ]
        ),
        encoding="utf-8",
    )

    subprocess.run(
        [
            str(app),
            "--config",
            str(config_path),
            "--output-dir",
            str(snapshot_dir),
            "--max-frames",
            "1",
        ],
        cwd=repo_root,
        check=True,
    )

    subprocess.run(
        [
            sys.executable,
            "scripts/validate-replay-artifacts.py",
            "--snapshot-dir",
            str(snapshot_dir),
            "--annotation-dir",
            str(annotation_dir),
            "--profile-jsonl",
            str(profile_jsonl),
            "--expect-frames",
            "1",
            "--expect-map-frame",
            "map_recorded_ci_0001",
            "--timestamp-soft-threshold-ms",
            "0",
            "--require-agent",
            "--require-world-keys",
        ],
        cwd=repo_root,
        check=True,
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
