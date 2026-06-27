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

echo ""
echo "┌─ Build configuration ──────────────────────────────────────────┐"

# ── CUDA ─────────────────────────────────────────────────────────────
CUDA_CMAKE_FLAG=""
if [[ "${DEDALUS_CUDA:-}" == "0" ]]; then
    CUDA_CMAKE_FLAG="-DDEDALUS_CUDA=OFF"
    echo "│  CUDA:       OFF (forced)"
elif [[ "${DEDALUS_CUDA:-}" == "1" ]]; then
    CUDA_CMAKE_FLAG="-DDEDALUS_CUDA=ON"
    echo "│  CUDA:       ON  (forced)"
else
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
        GPU_NAME=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -n1 || echo "unknown")
        CUDA_VER=$(nvidia-smi 2>/dev/null | awk '/CUDA Version/{print $9}' || echo "?")
        echo "│  CUDA:       AUTO — detected nvcc at ${NVCC_BIN}"
        echo "│              GPU: ${GPU_NAME}  CUDA ${CUDA_VER}"
        export PATH="$(dirname "$NVCC_BIN"):$PATH"
    else
        echo "│  CUDA:       OFF  (nvcc not found — CPU-only)"
        echo "│              hint: sudo apt-get install -y cuda-toolkit"
    fi
fi

# ── ONNX Runtime ─────────────────────────────────────────────────────
ORT_INSTALL_DIR="${ORT_INSTALL_DIR:-/usr/local/onnxruntime}"
ONNX_CMAKE_FLAGS=""
if [[ "${DEDALUS_ONNX_DEPTH:-}" == "0" ]]; then
    echo "│  ONNX depth: OFF (forced)"
elif [[ "${DEDALUS_ONNX_DEPTH:-}" == "1" ]]; then
    ONNX_CMAKE_FLAGS="-DDEDALUS_ENABLE_ONNX_DEPTH=ON -DCMAKE_PREFIX_PATH=${ORT_INSTALL_DIR}"
    ORT_VER=$(basename "$(ls "${ORT_INSTALL_DIR}/lib/libonnxruntime.so."* 2>/dev/null | head -n1)" \
              2>/dev/null | sed 's/libonnxruntime.so.//' || echo "?")
    echo "│  ONNX depth: ON  (forced)  SDK ${ORT_VER} at ${ORT_INSTALL_DIR}"
elif [[ -f "${ORT_INSTALL_DIR}/lib/cmake/onnxruntime/onnxruntimeConfig.cmake" ]]; then
    ONNX_CMAKE_FLAGS="-DDEDALUS_ENABLE_ONNX_DEPTH=ON -DCMAKE_PREFIX_PATH=${ORT_INSTALL_DIR}"
    ORT_VER=$(basename "$(ls "${ORT_INSTALL_DIR}/lib/libonnxruntime.so."* 2>/dev/null | head -n1)" \
              2>/dev/null | sed 's/libonnxruntime.so.//' || echo "?")
    echo "│  ONNX depth: ON  (auto)    SDK ${ORT_VER} at ${ORT_INSTALL_DIR}"
else
    echo "│  ONNX depth: OFF (SDK not found at ${ORT_INSTALL_DIR})"
    echo "│              hint: run ./setup.sh  or set ORT_INSTALL_DIR"
fi

echo "│  Build dir:  ${BUILD_DIR}"
echo "│  Jobs:       $(dedalus_build_jobs)"
echo "└────────────────────────────────────────────────────────────────┘"
echo ""

cmake -S . -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    ${CUDA_CMAKE_FLAG:+"$CUDA_CMAKE_FLAG"} \
    ${ONNX_CMAKE_FLAGS:+$ONNX_CMAKE_FLAGS}

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
