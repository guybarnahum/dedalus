#!/usr/bin/env bash
set -euo pipefail

dedalus_build_jobs() {
  if [ -n "${JOBS:-}" ]; then
    printf '%s\n' "$JOBS"
  elif command -v nproc >/dev/null 2>&1; then
    nproc
  elif command -v sysctl >/dev/null 2>&1; then
    sysctl -n hw.ncpu
  else
    printf '%s\n' 4
  fi
}

BUILD_DIR="${BUILD_DIR:-build-staging}"
TEST_MODE="${TEST_MODE:-fast}"
TEST_REGEX="${TEST_REGEX:-}"
TEST_LABEL_REGEX="${TEST_LABEL_REGEX:-}"

cd "$(git rev-parse --show-toplevel)"

git pull --ff-only

# CUDA override: set DEDALUS_CUDA=0 to force off, DEDALUS_CUDA=1 to force on.
# Without an override, CMakeLists.txt auto-probes via find_package(CUDAToolkit QUIET).
CUDA_CMAKE_FLAGS=()
if [[ "${DEDALUS_CUDA:-}" == "0" ]]; then
    CUDA_CMAKE_FLAGS+=(-DDEDALUS_CUDA=OFF)
    echo "[build] CUDA: forced off"
elif [[ "${DEDALUS_CUDA:-}" == "1" ]]; then
    CUDA_CMAKE_FLAGS+=(-DDEDALUS_CUDA=ON)
    echo "[build] CUDA: forced on"
elif command -v nvcc >/dev/null 2>&1; then
    echo "[build] CUDA: detected (nvcc $(nvcc --version 2>/dev/null | grep -oP 'release \K[\d.]+' || echo '?')) — DEDALUS_CUDA will auto-enable"
else
    echo "[build] CUDA: not detected — CPU-only build"
fi

cmake -S . -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=RelWithDebInfo "${CUDA_CMAKE_FLAGS[@]}"

cmake --build "$BUILD_DIR" -j"$(dedalus_build_jobs)"

# Generate the viewer SPA into the build directory so it is always in sync
# with the dedalus_viewer binary.  Serve with:
#   ./build/apps/dedalus_viewer --static-root "$BUILD_DIR" --replay-dir <out-dir>
python3 tools/visualization/mission_unified_viewer.py --output "$BUILD_DIR/viewer.html"
python3 tools/validation/validate-mission-unified-viewer.py "$BUILD_DIR/viewer.html"

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
