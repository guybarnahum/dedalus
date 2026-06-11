#!/usr/bin/env bash
set -euo pipefail

BUILD_DIR="${BUILD_DIR:-build-staging}"
TEST_MODE="${TEST_MODE:-fast}"
TEST_REGEX="${TEST_REGEX:-}"
TEST_LABEL_REGEX="${TEST_LABEL_REGEX:-}"

cd "$(git rev-parse --show-toplevel)"

git pull --ff-only

cmake --build "$BUILD_DIR" -j"$(nproc)"

ctest_args=(--test-dir "$BUILD_DIR" --output-on-failure)

if [[ -n "$TEST_REGEX" ]]; then
  ctest "${ctest_args[@]}" -R "$TEST_REGEX"
elif [[ -n "$TEST_LABEL_REGEX" ]]; then
  ctest "${ctest_args[@]}" -L "$TEST_LABEL_REGEX"
else
  case "$TEST_MODE" in
    fast)
      # Keep the default developer loop quick: contracts + native unit tests only.
      # Synthetic replay/artifact validators and scenario/campaign harnesses remain opt-in.
      ctest "${ctest_args[@]}" -LE 'synthetic|scenario'
      ;;
    full)
      ctest "${ctest_args[@]}"
      ;;
    *)
      echo "Unsupported TEST_MODE='$TEST_MODE' (expected 'fast' or 'full')" >&2
      exit 2
      ;;
  esac
fi

python3 tools/validation/check-architectural-naming.py .

git status --short
