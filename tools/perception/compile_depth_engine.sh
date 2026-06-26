#!/usr/bin/env bash
# compile_depth_engine.sh — Convert .onnx to TensorRT .engine (NVIDIA GPU)
#
# Run this once on the target GPU host after export_depth_anything.py has produced
# the .onnx.  The resulting .engine file is device-specific (GPU arch + TRT version)
# and is NOT committed to the repo.
#
# Supported targets: Jetson (JetPack TensorRT), L4/A10/A100 EC2 (TensorRT SDK).
#
# Requirements:
#   - NVIDIA TensorRT installed
#   - trtexec available at /usr/src/tensorrt/bin/trtexec or in PATH
#   - INT8 calibration images in CALIB_DIR (256+ frames, representative of deployment)
#     Skip --calib-dir to build a FP16 engine (recommended for L4 EC2).
#
# Usage:
#   ./tools/perception/compile_depth_engine.sh \
#       --onnx /path/to/model.onnx \
#       --output /path/to/model.engine \
#       [--calib-dir /path/to/calib/images]

set -euo pipefail

TRTEXEC="${TRTEXEC:-trtexec}"
if ! command -v "$TRTEXEC" &>/dev/null; then
    TRTEXEC=/usr/src/tensorrt/bin/trtexec
fi

ONNX_PATH=""
ENGINE_PATH=""
CALIB_DIR=""
INPUT_SIZE=518

while [[ $# -gt 0 ]]; do
    case "$1" in
        --onnx)      ONNX_PATH="$2";   shift 2 ;;
        --output)    ENGINE_PATH="$2"; shift 2 ;;
        --calib-dir) CALIB_DIR="$2";   shift 2 ;;
        --input-size) INPUT_SIZE="$2"; shift 2 ;;
        *) echo "Unknown arg: $1" >&2; exit 1 ;;
    esac
done

if [[ -z "$ONNX_PATH" || -z "$ENGINE_PATH" ]]; then
    echo "Usage: $0 --onnx <model.onnx> --output <model.engine> [--calib-dir <dir>]" >&2
    exit 1
fi

CALIB_ARGS=""
if [[ -n "$CALIB_DIR" ]]; then
    CALIB_ARGS="--int8 --calib=$CALIB_DIR"
else
    echo "WARNING: no --calib-dir supplied; building FP16 engine (lower accuracy)" >&2
    CALIB_ARGS="--fp16"
fi

echo "Compiling: $ONNX_PATH → $ENGINE_PATH"
"$TRTEXEC" \
    --onnx="$ONNX_PATH" \
    --saveEngine="$ENGINE_PATH" \
    --inputIOFormats=fp32:chw \
    --outputIOFormats=fp32:chw \
    --shapes="image:1x3x${INPUT_SIZE}x${INPUT_SIZE}" \
    --workspace=512 \
    $CALIB_ARGS

echo "OK: engine written to $ENGINE_PATH"
