#!/usr/bin/env bash
set -euo pipefail

BUILD_DIR="${BUILD_DIR:-build-staging}"
TEST_REGEX="${TEST_REGEX:-}"

cd "$(git rev-parse --show-toplevel)"

git pull --ff-only

cmake --build "$BUILD_DIR" -j"$(nproc)"

if [[ -n "$TEST_REGEX" ]]; then
  ctest --test-dir "$BUILD_DIR" --output-on-failure -R "$TEST_REGEX"
else
  ctest --test-dir "$BUILD_DIR" --output-on-failure
fi

python3 tools/validation/check-architectural-naming.py .

git status --short
