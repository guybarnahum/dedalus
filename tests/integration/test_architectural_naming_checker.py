#!/usr/bin/env python3
"""Smoke tests for tools/validation/check-architectural-naming.py."""

from __future__ import annotations

import subprocess
import sys
import tempfile
from pathlib import Path


def run_checker(repo_root: Path, *paths: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [
            sys.executable,
            str(repo_root / "tools" / "validation" / "check-architectural-naming.py"),
            str(repo_root),
            *[str(path.relative_to(repo_root)) for path in paths],
        ],
        cwd=repo_root,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )


def main() -> int:
    repo_root = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[2]

    with tempfile.TemporaryDirectory(dir=repo_root) as tmp:
        tmp_root = Path(tmp)

        good = tmp_root / "validate-obstacle-sensing-evidence-snapshots.py"
        good.write_text(
            "print('OBSTACLE_SENSING_EVIDENCE snapshot dir')\n",
            encoding="utf-8",
        )
        good_result = run_checker(repo_root, good)
        if good_result.returncode != 0:
            print(good_result.stdout)
            print(good_result.stderr, file=sys.stderr)
            return 1
        if "architectural naming check passed" not in good_result.stdout:
            print(good_result.stdout)
            print("checker did not report expected pass", file=sys.stderr)
            return 1

        forbidden_filename = "validate-" + "track" + "4-snapshots.py"
        forbidden_debug_label = "print('" + "track" + "4 volumes=1 evidence=2')\n"
        bad = tmp_root / forbidden_filename
        bad.write_text(forbidden_debug_label, encoding="utf-8")
        bad_result = run_checker(repo_root, bad)
        if bad_result.returncode == 0:
            print(bad_result.stdout)
            print("checker accepted planning-label naming", file=sys.stderr)
            return 1
        if "planning-label name" not in bad_result.stdout:
            print(bad_result.stdout)
            print("checker did not explain planning-label naming failure", file=sys.stderr)
            return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
