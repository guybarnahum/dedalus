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
#   - validation: canonical post-run artifact validators
#
# Normal sim lifecycle:
#   simulation/airsim/run.sh AirSimNH
#   simulation/airsim/run_mission.sh
#   simulation/airsim/stop.sh

set -euo pipefail

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
SAFE_HEIGHT=""
BEHAVIOR_MIN_HEIGHT=""
BEHAVIOR_DURATION_S="360"
VEHICLE_NAME="PX4"
AIRSIM_HOST="127.0.0.1"
AIRSIM_RPC_PORT="41451"
AIRSIM_PREFLIGHT=1
CAMERAS=("front_center" "0")
CAMERA_RATE_HZ="10"
CAMERA_RESEND_S="0.25"
CAMERA_CAPTURE_EVERY_S="1.0"
WITH_CAMERA=1
WITH_OVERLAY=1
WITH_OCCUPANCY_OVERLAY=1
WITH_OVERLAY_DEBUG=0
WITH_VALIDATION=1
OVERLAY_RATE_HZ="5"
OVERLAY_DURATION_S="0"
OVERLAY_MAX_OCCUPANCY_CELLS="32"
VALIDATION_MIN_ORBITS="2.95"
VALIDATION_RADIUS="10.0"
VALIDATION_TIMEOUT_S="300"
VALIDATION_AVG_RADIUS_ERROR_MAX="1.0"
VALIDATION_MAX_RADIUS_ERROR_AFTER_LATCH="3.0"
VALIDATION_COMPLETE_REASON="orbit_count_elapsed"
VALIDATION_EXPECT_SEQUENCE=0
VALIDATION_SEQUENCE_STEPS="approach,circle"
VALIDATION_SEQUENCE_STEP_MODES=""
ATTACH=0
EXIT_ON_COMPLETE=1
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
  validation        canonical post-run validators

Prerequisite:
  ./run.sh AirSimNH

Examples:
  ./run_mission.sh
  ./run_mission.sh --attach
  ./run_mission.sh --no-overlay
  ./run_mission.sh --overlay-debug
  ./run_mission.sh --no-occupancy-overlay
  ./run_mission.sh --max-occupancy-cells 8
  ./run_mission.sh --no-validation
  ./run_mission.sh --camera 0 --camera front_center
  ./run_mission.sh --config ../../config/core_stack_object_behavior_airsim_existing_object_circle.yml
  ./run_mission.sh --config ../../config/core_stack_object_behavior_airsim_existing_object_sequence.yml --output-dir ../../out/object_behavior_airsim_existing_object_sequence --expect-sequence --validation-complete-reason sequence_complete
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
  --safe-height M             Override takeoff/return transit height and bridge takeoff height.
                               If omitted, use config values.
  --behavior-min-height M     Override ExecuteMission minimum behavior height. If omitted, use config values.
  --behavior-duration-s S     mission-loop --behavior-duration-s. Default: 360
  --vehicle-name NAME         AirSim vehicle name. Default: PX4
  --airsim-host HOST          AirSim RPC host. Default: 127.0.0.1
  --airsim-rpc-port PORT      AirSim RPC port. Default: 41451
  --no-airsim-preflight       Skip the AirSim RPC preflight check
  --camera CAMERA             AirSim camera to command. May repeat. Default: front_center and 0
  --no-camera                 Do not start camera-pointing bridge
  --no-overlay                Do not start overlay
  --overlay-debug             Enable verbose overlay debug logs/debug JSON. Default: off
  --no-occupancy-overlay      Do not pass Track 4 occupancy render flags to overlay
  --max-occupancy-cells N     Max occupancy cells for overlay. Default: 32
  --no-validation             Do not start post-run validators
  --overlay-rate-hz HZ        overlay update rate. Default: 5
  --overlay-duration-s S      overlay duration; 0 means run until session stops. Default: 0
  --validation-min-orbits N   Circle validator --min-orbits. Default: 2.95
  --validation-radius M       Circle validator --radius. Default: 10.0
  --validation-timeout-s S    Wait timeout for runtime_stop. Default: 300; use 0 to wait forever.
  --validation-complete-reason REASON
                                Behavior complete reason for circle validator. Default: orbit_count_elapsed
  --expect-sequence            Require behavior sequence step events in artifact validation.
  --expect-sequence-steps CSV  Sequence step order. Default: approach,circle
  --expect-sequence-step-modes CSV
                                Step policy modes, e.g. approach:target:target,circle:target:target
  --no-progress               Do not pass --progress to mission-loop
  --attach                    Attach to tmux after starting
  --keep-tools-running        Do not stop camera bridge / overlay on mission runtime_stop
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

config_value() {
    local key="$1"
    (grep -E "^${key}:" "$CONFIG_PATH" | awk '{print $2}' | tail -1) || true
}

tmux_shell_with_failure_hold() {
    local command="$1"
    cat <<EOF
set +e
${command}
status=\$?
if [[ \$status -ne 0 ]]; then
  echo
  echo "❌ tmux pane command failed with status \$status"
  echo "   Inspect the log above, then press Enter to close this pane."
  read -r _
fi
exit \$status
EOF
}

airsim_rpc_reachable() {
    python3 - "$AIRSIM_HOST" "$AIRSIM_RPC_PORT" <<'PY'
import socket
import sys

host = sys.argv[1]
port = int(sys.argv[2])

try:
    with socket.create_connection((host, port), timeout=2.0):
        pass
except OSError:
    raise SystemExit(1)

raise SystemExit(0)
PY
}

require_airsim_rpc() {
    if airsim_rpc_reachable; then
        return 0
    fi

    cat >&2 <<EOF
❌ AirSim RPC is not reachable at ${AIRSIM_HOST}:${AIRSIM_RPC_PORT}.

This usually means the simulator/PX4 runtime is not running yet.

Start the simulator first:

  cd ${REPO_ROOT_ABS}/simulation/airsim
  ./run.sh AirSimNH

Wait until the Unreal/AirSim window is loaded in NICE DCV, then rerun:

  ./run_mission.sh

Useful diagnostics:

  tmux ls
  ss -ltnp | grep ${AIRSIM_RPC_PORT} || true
  tail -200 "\$(ls -t ${REPO_ROOT_ABS}/simulation/airsim/logs/sim_*.log | head -1)"

To bypass this check for advanced debugging:

  ./run_mission.sh --no-airsim-preflight ...
EOF

    return 1
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --session) SESSION_NAME="$2"; shift 2 ;;
        --build-dir) BUILD_DIR="$(abs_path "$2")"; shift 2 ;;
        --config) CONFIG_PATH="$(abs_path "$2")"; shift 2 ;;
        --output-dir) OUTPUT_DIR="$(abs_path "$2")"; shift 2 ;;
        --stream-host) STREAM_HOST="$2"; shift 2 ;;
        --stream-port) STREAM_PORT="$2"; shift 2 ;;
        --max-frames) MAX_FRAMES="$2"; shift 2 ;;
        --shutdown-max-frames) SHUTDOWN_MAX_FRAMES="$2"; shift 2 ;;
        --safe-height) SAFE_HEIGHT="$2"; shift 2 ;;
        --behavior-min-height) BEHAVIOR_MIN_HEIGHT="$2"; shift 2 ;;
        --behavior-duration-s) BEHAVIOR_DURATION_S="$2"; shift 2 ;;
        --vehicle-name) VEHICLE_NAME="$2"; shift 2 ;;
        --airsim-host) AIRSIM_HOST="$2"; shift 2 ;;
        --airsim-rpc-port) AIRSIM_RPC_PORT="$2"; shift 2 ;;
        --no-airsim-preflight) AIRSIM_PREFLIGHT=0; shift ;;
        --camera)
            if [[ "${CAMERAS[*]}" == "front_center 0" ]]; then
                CAMERAS=()
            fi
            CAMERAS+=("$2")
            shift 2
            ;;
        --no-camera) WITH_CAMERA=0; shift ;;
        --no-overlay) WITH_OVERLAY=0; shift ;;
        --overlay-debug) WITH_OVERLAY_DEBUG=1; shift ;;
        --no-occupancy-overlay) WITH_OCCUPANCY_OVERLAY=0; shift ;;
        --max-occupancy-cells) OVERLAY_MAX_OCCUPANCY_CELLS="$2"; shift 2 ;;
        --no-validation) WITH_VALIDATION=0; shift ;;
        --overlay-rate-hz) OVERLAY_RATE_HZ="$2"; shift 2 ;;
        --overlay-duration-s) OVERLAY_DURATION_S="$2"; shift 2 ;;
        --validation-min-orbits) VALIDATION_MIN_ORBITS="$2"; shift 2 ;;
        --validation-radius) VALIDATION_RADIUS="$2"; shift 2 ;;
        --validation-timeout-s) VALIDATION_TIMEOUT_S="$2"; shift 2 ;;
        --validation-complete-reason) VALIDATION_COMPLETE_REASON="$2"; shift 2 ;;
        --expect-sequence) VALIDATION_EXPECT_SEQUENCE=1; shift ;;
        --expect-sequence-steps) VALIDATION_SEQUENCE_STEPS="$2"; shift 2 ;;
        --expect-sequence-step-modes) VALIDATION_SEQUENCE_STEP_MODES="$2"; shift 2 ;;
        --no-progress) PROGRESS_FLAG=""; shift ;;
        --attach) ATTACH=1; shift ;;
        --keep-tools-running) EXIT_ON_COMPLETE=0; shift ;;
        --no-kill-existing) KILL_EXISTING=0; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "❌ Unknown option: $1" >&2; usage; exit 1 ;;
    esac
done

MISSION_BIN="$BUILD_DIR/apps/dedalus_mission_loop"
MISSION_LOG="$LOG_DIR_ABS/mission_${TIMESTAMP}.log"
CAMERA_LOG="$LOG_DIR_ABS/camera_pointing_${TIMESTAMP}.log"
OVERLAY_LOG="$LOG_DIR_ABS/overlay_${TIMESTAMP}.log"
VALIDATION_LOG="$LOG_DIR_ABS/validation_${TIMESTAMP}.log"
VALIDATION_SCRIPT="$LOG_DIR_ABS/validation_${TIMESTAMP}.sh"
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

if [[ "$AIRSIM_PREFLIGHT" -eq 1 ]]; then
    require_airsim_rpc
fi

mkdir -p "$OUTPUT_DIR" "$CAMERA_FRAMES_DIR"

MISSION_CMD=(
    "$MISSION_BIN"
    --config "$CONFIG_PATH"
    --output-dir "$OUTPUT_DIR"
    --max-frames "$MAX_FRAMES"
    --shutdown-max-frames "$SHUTDOWN_MAX_FRAMES"
    --world-snapshot-stream-port "$STREAM_PORT"
    --behavior-duration-s "$BEHAVIOR_DURATION_S"
)
if [[ -n "$SAFE_HEIGHT" ]]; then
    MISSION_CMD+=(--safe-height "$SAFE_HEIGHT")
fi
if [[ -n "$BEHAVIOR_MIN_HEIGHT" ]]; then
    MISSION_CMD+=(--behavior-min-height "$BEHAVIOR_MIN_HEIGHT")
fi
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
if [[ "$EXIT_ON_COMPLETE" -eq 1 ]]; then
    CAMERA_CMD+=(--exit-on-runtime-stop)
fi
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
)
if [[ "$WITH_OVERLAY_DEBUG" -eq 1 ]]; then
    OVERLAY_CMD+=(--debug --debug-json "$OVERLAY_DEBUG_JSON")
fi
if [[ "$WITH_OCCUPANCY_OVERLAY" -eq 1 ]]; then
    OVERLAY_CMD+=(
        --show-occupancy-summary
        --show-occupancy-cells
        --max-occupancy-cells "$OVERLAY_MAX_OCCUPANCY_CELLS"
    )
fi
if [[ "$EXIT_ON_COMPLETE" -eq 1 ]]; then
    OVERLAY_CMD+=(--exit-on-runtime-stop)
fi
if [[ "$OVERLAY_DURATION_S" != "0" ]]; then
    OVERLAY_CMD+=(--duration-s "$OVERLAY_DURATION_S")
fi

if [[ -n "$SAFE_HEIGHT" ]]; then
    VALIDATION_SAFE_HEIGHT="$SAFE_HEIGHT"
else
    VALIDATION_SAFE_HEIGHT="$(config_value 'mission_options\.flight_takeoff_height_m')"
    if [[ -z "$VALIDATION_SAFE_HEIGHT" ]]; then
        VALIDATION_SAFE_HEIGHT="$(config_value 'mission_options\.flight_safe_height_m')"
    fi
    VALIDATION_SAFE_HEIGHT="${VALIDATION_SAFE_HEIGHT:-40}"
fi

VALIDATION_SHELL=$(cat <<EOF
set -euo pipefail
cd $(printf '%q' "$REPO_ROOT_ABS")
EVENTS=$(printf '%q' "$OUTPUT_DIR/mission_events.jsonl")
TIMEOUT=$(printf '%q' "$VALIDATION_TIMEOUT_S")
python3 - "\$EVENTS" "\$TIMEOUT" <<'PY'
import json
import sys
import time
from pathlib import Path

path = Path(sys.argv[1])
timeout_s = float(sys.argv[2])
start = time.monotonic()
last_count = -1
print(f"validation: waiting for runtime_stop in {path}", flush=True)
while True:
    events = []
    if path.exists():
        for line in path.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError:
                pass
    if len(events) != last_count:
        print(f"validation: events={len(events)}", flush=True)
        last_count = len(events)
    runtime_stop = [event for event in events if isinstance(event, dict) and event.get("event") == "runtime_stop"]
    if runtime_stop:
        print("validation: runtime_stop observed", flush=True)
        break
    if timeout_s > 0 and time.monotonic() - start >= timeout_s:
        raise SystemExit(f"validation: timed out waiting for runtime_stop after {timeout_s}s")
    time.sleep(2.0)
PY
python3 tools/mission/mission-events-summary.py "\$EVENTS" --expect-complete
VALIDATE_MISSION_CMD=(python3 tools/mission/validate-mission-artifacts.py $(printf '%q' "$OUTPUT_DIR") --expect-complete --expect-behavior --safe-height-m $(printf '%q' "$VALIDATION_SAFE_HEIGHT") --landed-height-m 1.0)
EOF
)
if [[ "$VALIDATION_EXPECT_SEQUENCE" -eq 1 ]]; then
    VALIDATION_SHELL+=$'\n'
    VALIDATION_SHELL+="VALIDATE_MISSION_CMD+=(--expect-sequence --expect-sequence-steps $(printf '%q' "$VALIDATION_SEQUENCE_STEPS"))"
fi
if [[ -n "$VALIDATION_SEQUENCE_STEP_MODES" ]]; then
    VALIDATION_SHELL+=$'\n'
    VALIDATION_SHELL+="VALIDATE_MISSION_CMD+=(--expect-sequence-step-modes $(printf '%q' "$VALIDATION_SEQUENCE_STEP_MODES"))"
fi
if [[ "$WITH_CAMERA" -eq 1 ]]; then
    VALIDATION_SHELL+=$'\n'
    VALIDATION_SHELL+="VALIDATE_MISSION_CMD+=(--expect-camera-pointing --expect-camera-modes neutral,target,home,landing_area --camera-frames-dir $(printf '%q' "$CAMERA_FRAMES_DIR") --expect-camera-proof-frames)"
fi
VALIDATION_SHELL+=$'\n'
VALIDATION_SHELL+=$(cat <<EOF
"\${VALIDATE_MISSION_CMD[@]}"
python3 tools/validation/validate-circle-trajectory.py \
  --events "\$EVENTS" \
  --min-orbits $(printf '%q' "$VALIDATION_MIN_ORBITS") \
  --radius $(printf '%q' "$VALIDATION_RADIUS") \
  --avg-radius-error-max $(printf '%q' "$VALIDATION_AVG_RADIUS_ERROR_MAX") \
  --max-radius-error-after-latch $(printf '%q' "$VALIDATION_MAX_RADIUS_ERROR_AFTER_LATCH") \
  --expect-complete-reason $(printf '%q' "$VALIDATION_COMPLETE_REASON") \
  --require-terminal-settled \
  --require-lifecycle
echo "validation: PASS"
EOF
)

printf '%s\n' "$VALIDATION_SHELL" > "$VALIDATION_SCRIPT"
chmod +x "$VALIDATION_SCRIPT"

if [[ "$KILL_EXISTING" -eq 1 ]]; then
    tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true
elif tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    echo "❌ tmux session already exists: $SESSION_NAME" >&2
    exit 1
fi

if [[ "$WITH_CAMERA" -eq 1 ]]; then
    CAMERA_SHELL="cd $(printf '%q' "$REPO_ROOT_ABS") && $(quote_cmd "${CAMERA_CMD[@]}") 2>&1 | tee $(printf '%q' "$CAMERA_LOG")"
    tmux new-session -d -s "$SESSION_NAME" -n camera-pointing "bash -lc $(printf '%q' "$(tmux_shell_with_failure_hold "$CAMERA_SHELL")")"
else
    tmux new-session -d -s "$SESSION_NAME" -n launcher "bash -lc 'sleep infinity'"
fi

if [[ "$WITH_OVERLAY" -eq 1 ]]; then
    OVERLAY_SHELL="cd $(printf '%q' "$REPO_ROOT_ABS") && $(quote_cmd "${OVERLAY_CMD[@]}") 2>&1 | tee $(printf '%q' "$OVERLAY_LOG")"
    tmux new-window -t "$SESSION_NAME" -n overlay "bash -lc $(printf '%q' "$(tmux_shell_with_failure_hold "$OVERLAY_SHELL")")"
fi

if [[ "$WITH_VALIDATION" -eq 1 ]]; then
    VALIDATION_RUN_SHELL="bash $(printf '%q' "$VALIDATION_SCRIPT") 2>&1 | tee $(printf '%q' "$VALIDATION_LOG")"
    tmux new-window -t "$SESSION_NAME" -n validation "bash -lc $(printf '%q' "$(tmux_shell_with_failure_hold "$VALIDATION_RUN_SHELL")")"
fi

MISSION_SHELL="cd $(printf '%q' "$REPO_ROOT_ABS") && $(quote_cmd "${MISSION_CMD[@]}") 2>&1 | tee $(printf '%q' "$MISSION_LOG")"
tmux new-window -t "$SESSION_NAME" -n mission-loop "bash -lc $(printf '%q' "$(tmux_shell_with_failure_hold "$MISSION_SHELL")")"
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
    if [[ "$WITH_OVERLAY_DEBUG" -eq 1 ]]; then
        echo "  debug json: $OVERLAY_DEBUG_JSON"
    else
        echo "  debug json: disabled; pass --overlay-debug to enable"
    fi
    echo "  debug:      $([[ "$WITH_OVERLAY_DEBUG" -eq 1 ]] && echo enabled || echo disabled)"
    echo "  occupancy:  $([[ "$WITH_OCCUPANCY_OVERLAY" -eq 1 ]] && echo enabled || echo disabled)"
    if [[ "$WITH_OCCUPANCY_OVERLAY" -eq 1 ]]; then
        echo "  max cells:  $OVERLAY_MAX_OCCUPANCY_CELLS"
    fi
    echo ""
fi
if [[ "$WITH_VALIDATION" -eq 1 ]]; then
    echo "Validation:"
    echo "  log:        $VALIDATION_LOG"
    echo "  script:     $VALIDATION_SCRIPT"
    echo "  timeout:    $VALIDATION_TIMEOUT_S s"
    echo "  validators: mission-events-summary, validate-mission-artifacts, validate-circle-trajectory"
    echo "  complete reason: $VALIDATION_COMPLETE_REASON"
    if [[ "$VALIDATION_EXPECT_SEQUENCE" -eq 1 ]]; then
        echo "  sequence steps: $VALIDATION_SEQUENCE_STEPS"
    fi
    if [[ -n "$VALIDATION_SEQUENCE_STEP_MODES" ]]; then
        echo "  sequence step modes: $VALIDATION_SEQUENCE_STEP_MODES"
    fi
    echo ""
fi

echo "Exit on mission complete: $([[ "$EXIT_ON_COMPLETE" -eq 1 ]] && echo yes || echo no)"
echo "Useful commands:"
echo "  attach: tmux attach -t $SESSION_NAME"
echo "  stop mission stack: tmux kill-session -t $SESSION_NAME"
echo "  stop simulator/PX4: ./stop.sh"
echo ""
echo "Mission command:"
echo "  $(quote_cmd "${MISSION_CMD[@]}")"
if [[ "$WITH_OVERLAY" -eq 1 ]]; then
    echo "Overlay command:"
    echo "  $(quote_cmd "${OVERLAY_CMD[@]}")"
fi

if [[ "$ATTACH" -eq 1 ]]; then
    tmux attach -t "$SESSION_NAME"
fi
