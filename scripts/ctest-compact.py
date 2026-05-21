#!/usr/bin/env python3
"""Run CTest with compact one-line progress output.

CTest's default output prints a separate "Start" line before each test. This
wrapper keeps the same test ordering and failure behavior but renders:

  1/33 Test  #1: name (label) .............  Passed    0.00 sec

The line is printed when the test starts without a trailing newline, then
completed when the test finishes.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import time
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--test-dir", type=Path, default=Path("build-staging"))
    parser.add_argument("-R", "--regex", default=None)
    parser.add_argument("--output-on-failure", action="store_true")
    return parser.parse_args()


def ctest_base_command(args: argparse.Namespace) -> list[str]:
    command = ["ctest", "--test-dir", str(args.test_dir)]
    if args.regex:
        command.extend(["-R", args.regex])
    return command


def load_tests(args: argparse.Namespace) -> list[dict]:
    command = ctest_base_command(args) + ["--show-only=json-v1"]
    result = subprocess.run(command, check=True, text=True, stdout=subprocess.PIPE)
    payload = json.loads(result.stdout)
    tests = payload.get("tests", [])
    if not isinstance(tests, list):
        raise RuntimeError("ctest JSON did not contain a tests list")
    return tests


def dotted_prefix(index: int, total: int, test: dict) -> str:
    name = str(test.get("name", "<unknown>"))
    test_number = int(test.get("number", index))
    left = f"{index:2d}/{total} Test #{test_number:2d}: {name} "
    dots = "." * max(3, 68 - len(left))
    return f"{left}{dots}"


def run_one(args: argparse.Namespace, test: dict) -> tuple[int, str, float]:
    name = str(test.get("name", ""))
    exact_name_regex = f"^{re.escape(name)}$"
    command = ctest_base_command(args) + ["-R", exact_name_regex]
    if args.output_on_failure:
        command.append("--output-on-failure")

    start = time.monotonic()
    result = subprocess.run(command, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    elapsed = time.monotonic() - start
    return result.returncode, result.stdout, elapsed


def main() -> int:
    args = parse_args()
    tests = load_tests(args)
    total = len(tests)
    failures: list[tuple[str, str]] = []

    print(f"Test project {args.test_dir}")
    for index, test in enumerate(tests, start=1):
        prefix = dotted_prefix(index, total, test)
        print(prefix, end="", flush=True)
        returncode, output, elapsed = run_one(args, test)
        if returncode == 0:
            print(f"   Passed  {elapsed:6.2f} sec")
        else:
            print(f"***Failed  {elapsed:6.2f} sec")
            failures.append((str(test.get("name", "<unknown>")), output))
            if args.output_on_failure and output.strip():
                print(output.rstrip())

    if failures:
        print("\nThe following tests FAILED:")
        for name, _ in failures:
            print(f"  - {name}")
        return 8
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
