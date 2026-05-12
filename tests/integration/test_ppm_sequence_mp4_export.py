#!/usr/bin/env python3
"""Integration test for scripts/export-ppm-sequence-to-mp4.py."""

from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path


def write_ppm(path: Path, rgb: bytes) -> None:
    path.write_bytes(b"P6\n2 2\n255\n" + rgb)


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: test_ppm_sequence_mp4_export.py <repo-root> <build-dir>", file=sys.stderr)
        return 2

    repo_root = Path(sys.argv[1]).resolve()
    build_dir = Path(sys.argv[2]).resolve()
    output_root = build_dir / "ppm-sequence-mp4-export"
    annotation_dir = output_root / "annotations"
    output_mp4 = output_root / "annotated.mp4"

    if output_root.exists():
        shutil.rmtree(output_root)
    annotation_dir.mkdir(parents=True)

    write_ppm(
        annotation_dir / "frame_000001.ppm",
        bytes([
            255, 0, 0,
            0, 255, 0,
            0, 0, 255,
            255, 255, 255,
        ]),
    )
    write_ppm(
        annotation_dir / "frame_000002.ppm",
        bytes([
            255, 255, 255,
            0, 0, 255,
            0, 255, 0,
            255, 0, 0,
        ]),
    )
    (annotation_dir / "manifest.txt").write_text(
        "frame_index,frame_id,timestamp_ns,path,output_fps\n"
        f"1,frame_0001,1000000000,{annotation_dir / 'frame_000001.ppm'},5\n"
        f"2,frame_0002,1200000000,{annotation_dir / 'frame_000002.ppm'},5\n",
        encoding="utf-8",
    )

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

    if shutil.which("ffmpeg") is not None:
        subprocess.run(
            [
                sys.executable,
                "scripts/export-ppm-sequence-to-mp4.py",
                "--annotation-dir",
                str(annotation_dir),
                "--output-mp4",
                str(output_mp4),
                "--overwrite",
            ],
            cwd=repo_root,
            check=True,
        )
        if not output_mp4.exists() or output_mp4.stat().st_size == 0:
            raise AssertionError("expected non-empty MP4 output")
    else:
        print("ffmpeg not found; real MP4 encode skipped after dry-run validation")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
