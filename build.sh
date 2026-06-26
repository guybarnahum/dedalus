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
CUDA_CMAKE_FLAG=""
if [[ "${DEDALUS_CUDA:-}" == "0" ]]; then
    CUDA_CMAKE_FLAG="-DDEDALUS_CUDA=OFF"
    echo "[build] CUDA: forced off"
elif [[ "${DEDALUS_CUDA:-}" == "1" ]]; then
    CUDA_CMAKE_FLAG="-DDEDALUS_CUDA=ON"
    echo "[build] CUDA: forced on"
else
    # nvcc may be installed but not in PATH (common on EC2/Ubuntu).
    # Check standard CUDA toolkit locations.
    NVCC_BIN=""
    for _candidate in \
        "$(command -v nvcc 2>/dev/null)" \
        /usr/local/cuda/bin/nvcc \
        /usr/local/cuda-*/bin/nvcc \
        /usr/bin/nvcc; do
        if [[ -x "$_candidate" ]]; then
            NVCC_BIN="$_candidate"
            break
        fi
    done
    if [[ -n "$NVCC_BIN" ]]; then
        echo "[build] CUDA: detected ($NVCC_BIN) — DEDALUS_CUDA will auto-enable"
        # Ensure nvcc is reachable for CMake's CUDAToolkit probe
        export PATH="$(dirname "$NVCC_BIN"):$PATH"
    else
        echo "[build] CUDA: not detected — CPU-only build"
        echo "[build]   (install with: sudo apt-get install -y cuda-toolkit)"
    fi
fi

cmake -S . -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=RelWithDebInfo ${CUDA_CMAKE_FLAG:+"$CUDA_CMAKE_FLAG"}

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
