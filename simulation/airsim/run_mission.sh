#!/usr/bin/env bash
#
# AirSim mission orchestrator for Dedalus object-conditioned behavior.
#
# This script assumes simulation/airsim/run.sh has already started AirSim/PX4.
# It starts the Dedalus mission loop plus companion operator/actuator tools in
# a dedicated tmux session:
#   - mission-loop: dedalus_mission_loop with existing-object/OCD config
#   - camera-pointing: AirSim camera/gimbal pitch bridge for front_center + 0
#   - overlay: optional AirSim world overlay / OSD subscriber
#
# Normal sim lifecycle:
#   simulation/airsim/run.sh AirSimNH
#   simulation/airsim/run_mission.sh
#   simulation/airsim/stop.sh

set -euo pipefail

ORIGINAL_ARGS=("$@")
cd "$(dirname "$0")"

REPO_ROOT_ABS="$(cd ../.. && pwd)"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
SESSION_NAME="dedalus-mission"
LOG_DIR_ABS="$(pwd)/logs"
mkdir -p "$LOG_DIR_ABS"

BUILD_DIR="$REPO_ROOT_ABS/build-staging"
MISSION_BIN="$BUILD_DIR/apps/dedalus_mission_loop"
CONFIG_PATH="$REPO_ROOT_ABS/config/core_stack_object_behavior_airsim_existing_object_circle.yml"
OUTPUT_DIR="$REPO_ROOT_ABS/out/object_behavior_airsim_existing_object_circle"
STREAM_HOST="127.0.0.1"
STREAM_PORT="47770"
MAX_FRAMES="5400"
SHUTDOWN_MAX_FRAMES="1800"
SAFE_HEIGHT="40"
BEHAVIOR_DURATION_S="360"
VEHICLE_NAME="PX4"
AIRSIM_HOST="127.0.0.1"
AIRSIM_RPC_PORT="41451"
CAMERAS=("front_center" "0")
CAMERA_RATE_HZ="10"
CAMERA_RESEND_S="0.25"
CAMERA_CAPTURE_EVERY_S="1.0"
WITH_CAMERA=1
WITH_OVERLAY=1
OVERLAY_RATE_HZ="5"
OVERLAY_DURATION_S="0"
ATTACH=0
KILL_EXISTING=1
PROGRESS_FLAG="--progress"

usage() {
    cat <<'EOF'
Usage:
  ./run_mission.sh [options]

Starts the validated AirSim object-conditioned behavior mission stack in tmux:
  mission-loop      dedalus_mission_loop
  camera-pointing   AirSim camera pitch bridge for front_center and 0
  overlay           optional world overlay / OSD subscriber

Prerequisite:
  ./run.sh AirSimNH

Examples:
  ./run_mission.sh
  ./run_mission.sh --attach
  ./run_mission.sh --no-overlay
  ./run_mission.sh --camera 0 --camera front_center
  ./run_mission.sh --config ../../config/core_stack_object_behavior_airsim_existing_object_circle.yml
  ./run_mission.sh --safe-height 40 --behavior-duration-s 360 --max-frames 5400

Options:
  --session NAME              tmux session name. Default: dedalus-mission
  --build-dir PATH            build dir. Default: ../../build-staging
  --config PATH               mission config. Default: ../../config/core_stack_object_behavior_airsim_existing_object_circle.yml
  --output-dir PATH           output dir. Default: ../../out/object_behavior_airsim_existing_object_circle
  --stream-host HOST          runtime stream host. Default: 127.0.0.1
  --stream-port PORT          runtime stream port. Default: 47770
  --max-frames N              mission-loop --max-frames. Default: 5400
  --shutdown-max-frames N     mission-loop --shutdown-max-frames. Default: 1800
  --safe-height M             mission-loop --safe-height. Default: 40
  --behavior-duration-s S     mission-loop --behavior-duration-s. Default: 360
  --vehicle-name NAME         AirSim vehicle name. Default: PX4
  --airsim-host HOST          AirSim RPC host. Default: 127.0.0.1
  --airsim-rpc-port PORT      AirSim RPC port. Default: 41451
  --camera CAMERA             AirSim camera to command. May repeat. Default: front_center and 0
  --no-camera                 Do not start camera-pointing bridge
  --no-overlay                Do not start overlay
  --overlay-rate-hz HZ        overlay update rate. Default: 5
  --overlay-duration-s S      overlay duration; 0 means run until session stops. Default: 0
  --no-progress               Do not pass --progress to mission-loop
  --attach                    Attach to tmux after starting
  --no-kill-existing          Do not kill an existing tmux session with the same name
  -h, --help                  Show this help
EOF
}

abs_path() {
    local path="$1"
    if [[ "$path" = /* ]]; then
        printf '%s\n' "$path"
    else
        local dir
        dir="$(dirname "$path")"
        local base
        base="$(basename "$path")"
        printf '%s/%s\n' "$(cd "$dir" && pwd)" "$base"
    fi
}

quote_cmd() {
    local out=""
    for arg in "$@"; do
        out+=" $(printf '%q' "$arg")"
    done
    printf '%s\n' "${out# }"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --session)
            SESSION_NAME="$2"
            shift 2
            ;;
        --build-dir)
            BUILD_DIR="$(abs_path "$2")"
            shift 2
            ;;
        --config)
            CONFIG_PATH="$(abs_path "$2")"
            shift 2
            ;;
        --output-dir)
            OUTPUT_DIR="$(abs_path "$2")"
            shift 2
            ;;
        --stream-host)
            STREAM_HOST="$2"
            shift 2
            ;;
        --stream-port)
            STREAM_PORT="$2"
            shift 2
            ;;
        --max-frames)
            MAX_FRAMES="$2"
            shift 2
            ;;
        --shutdown-max-frames)
            SHUTDOWN_MAX_FRAMES="$2"
            shift 2
            ;;
        --safe-height)
            SAFE_HEIGHT="$2"
            shift 2
            ;;
        --behavior-duration-s)
            BEHAVIOR_DURATION_S="$2"
            shift 2
            ;;
        --vehicle-name)
            VEHICLE_NAME="$2"
            shift 2
            ;;
        --airsim-host)
            AIRSIM_HOST="$2"
            shift 2
            ;;
        --airsim-rpc-port)
            AIRSIM_RPC_PORT="$2"
            shift 2
            ;;
        --camera)
            if [[ "${CAMERAS[*]}" == "front_center 0" ]]; then
                CAMERAS=()
            fi
            CAMERAS+=("$2")
            shift 2
            ;;
        --no-camera)
            WITH_CAMERA=0
            shift
            ;;
        --no-overlay)
            WITH_OVERLAY=0
            shift
            ;;
        --overlay-rate-hz)
            OVERLAY_RATE_HZ="$2"
            shift 2
            ;;
        --overlay-duration-s)
            OVERLAY_DURATION_S="$2"
            shift 2
            ;;
        --no-progress)
            PROGRESS_FLAG=""
            shift
            ;;
        --attach)
            ATTACH=1
            shift
            ;;
        --no-kill-existing)
            KILL_EXISTING=0
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "❌ Unknown option: $1" >&2
            usage
            exit 1
            ;;
    esac
done

MISSION_BIN="$BUILD_DIR/apps/dedalus_mission_loop"
MISSION_LOG="$LOG_DIR_ABS/mission_${TIMESTAMP}.log"
CAMERA_LOG="$LOG_DIR_ABS/camera_pointing_${TIMESTAMP}.log"
OVERLAY_LOG="$LOG_DIR_ABS/overlay_${TIMESTAMP}.log"
CAMERA_DEBUG_JSON="$OUTPUT_DIR/camera_pointing_latest.json"
OVERLAY_DEBUG_JSON="$OUTPUT_DIR/overlay_debug_latest.json"
CAMERA_FRAMES_DIR="$OUTPUT_DIR/camera_pointing_frames"

if [[ ! -x "$MISSION_BIN" ]]; then
    echo "❌ Mission binary not found or not executable: $MISSION_BIN" >&2
    echo "   Build first: cmake --build $BUILD_DIR -j\$(nproc)" >&2
    exit 1
fi

if [[ ! -f "$CONFIG_PATH" ]]; then
    echo "❌ Config not found: $CONFIG_PATH" >&2
    exit 1
fi

if [[ ${#CAMERAS[@]} -eq 0 && "$WITH_CAMERA" -eq 1 ]]; then
    echo "❌ At least one --camera is required when camera bridge is enabled." >&2
    exit 1
fi

mkdir -p "$OUTPUT_DIR" "$CAMERA_FRAMES_DIR"

MISSION_CMD=(
    "$MISSION_BIN"
    --config "$CONFIG_PATH"
    --output-dir "$OUTPUT_DIR"
    --max-frames "$MAX_FRAMES"
    --shutdown-max-frames "$SHUTDOWN_MAX_FRAMES"
    --world-snapshot-stream-port "$STREAM_PORT"
    --safe-height "$SAFE_HEIGHT"
    --behavior-duration-s "$BEHAVIOR_DURATION_S"
)
if [[ -n "$PROGRESS_FLAG" ]]; then
    MISSION_CMD+=("$PROGRESS_FLAG")
fi

CAMERA_CMD=(
    python3 "$REPO_ROOT_ABS/simulation/airsim/scripts/airsim-camera-pointing-bridge.py"
    --stream-host "$STREAM_HOST"
    --stream-port "$STREAM_PORT"
    --host "$AIRSIM_HOST"
    --rpc-port "$AIRSIM_RPC_PORT"
    --vehicle-name "$VEHICLE_NAME"
    --rate-hz "$CAMERA_RATE_HZ"
    --resend-s "$CAMERA_RESEND_S"
    --verify-pose
    --capture-dir "$CAMERA_FRAMES_DIR"
    --capture-every-s "$CAMERA_CAPTURE_EVERY_S"
    --debug
    --debug-json "$CAMERA_DEBUG_JSON"
)
for camera in "${CAMERAS[@]}"; do
    CAMERA_CMD+=(--cameras "$camera")
done

OVERLAY_CMD=(
    python3 "$REPO_ROOT_ABS/simulation/airsim/scripts/airsim-world-overlay.py"
    --stream-host "$STREAM_HOST"
    --stream-port "$STREAM_PORT"
    --follow
    --rate-hz "$OVERLAY_RATE_HZ"
    --clear
    --label
    --osd
    --debug
    --debug-json "$OVERLAY_DEBUG_JSON"
)
if [[ "$OVERLAY_DURATION_S" != "0" ]]; then
    OVERLAY_CMD+=(--duration-s "$OVERLAY_DURATION_S")
fi

if [[ "$KILL_EXISTING" -eq 1 ]]; then
    tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true
elif tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    echo "❌ tmux session already exists: $SESSION_NAME" >&2
    exit 1
fi

MISSION_SHELL="cd $(printf '%q' "$REPO_ROOT_ABS") && $(quote_cmd "${MISSION_CMD[@]}") 2>&1 | tee $(printf '%q' "$MISSION_LOG")"
tmux new-session -d -s "$SESSION_NAME" -n mission-loop "bash -lc $(printf '%q' "$MISSION_SHELL")"

if [[ "$WITH_CAMERA" -eq 1 ]]; then
    CAMERA_SHELL="cd $(printf '%q' "$REPO_ROOT_ABS") && $(quote_cmd "${CAMERA_CMD[@]}") 2>&1 | tee $(printf '%q' "$CAMERA_LOG")"
    tmux new-window -t "$SESSION_NAME" -n camera-pointing "bash -lc $(printf '%q' "$CAMERA_SHELL")"
fi

if [[ "$WITH_OVERLAY" -eq 1 ]]; then
    OVERLAY_SHELL="cd $(printf '%q' "$REPO_ROOT_ABS") && $(quote_cmd "${OVERLAY_CMD[@]}") 2>&1 | tee $(printf '%q' "$OVERLAY_LOG")"
    tmux new-window -t "$SESSION_NAME" -n overlay "bash -lc $(printf '%q' "$OVERLAY_SHELL")"
fi

tmux select-window -t "$SESSION_NAME:mission-loop"

echo "✅ AirSim mission stack started in tmux session '$SESSION_NAME'"
echo ""
echo "Mission loop:"
echo "  log:     $MISSION_LOG"
echo "  output:  $OUTPUT_DIR"
echo "  config:  $CONFIG_PATH"
echo ""
if [[ "$WITH_CAMERA" -eq 1 ]]; then
    echo "Camera pointing:"
    echo "  log:        $CAMERA_LOG"
    echo "  debug json: $CAMERA_DEBUG_JSON"
    echo "  frames:     $CAMERA_FRAMES_DIR"
    echo "  cameras:    ${CAMERAS[*]}"
    echo ""
fi
if [[ "$WITH_OVERLAY" -eq 1 ]]; then
    echo "Overlay:"
    echo "  log:        $OVERLAY_LOG"
    echo "  debug json: $OVERLAY_DEBUG_JSON"
    echo ""
fi

echo "Useful commands:"
echo "  attach: tmux attach -t $SESSION_NAME"
echo "  stop mission stack: tmux kill-session -t $SESSION_NAME"
echo "  stop simulator/PX4: ./stop.sh"
echo ""
echo "Mission command:"
echo "  $(quote_cmd "${MISSION_CMD[@]}")"

if [[ "$ATTACH" -eq 1 ]]; then
    tmux attach -t "$SESSION_NAME"
fi
