#!/usr/bin/env bash
#
# AirSim mission orchestrator for Dedalus object-conditioned behavior.
#
# This script assumes simulation/airsim/run.sh has already started AirSim/PX4.
# It starts the Dedalus mission loop plus companion operator/actuator tools in
# a dedicated tmux session.

set -euo pipefail

CALLER_CWD="$(pwd)"
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
SOURCE_FRAME_RATE_HZ=""
WITH_FRAME_PRODUCER_TIMING=0
FRAME_PRODUCER_TIMING_PATH=""
WITH_PIPELINE_TIMING=0
PIPELINE_TIMING_PATH=""
OBSTACLE_MAP_ARTIFACT=1
OBSTACLE_MAP_ARTIFACT_PATH=""
OBSTACLE_MAP_SITE_ID="airsim_neighborhood"
OBSTACLE_MAP_SITE_FRAME_ID="airsim_world"
OBSTACLE_MAP_MISSION_ID="object_behavior_airsim_existing_object_circle"
OBSTACLE_MAP_WRITE_EVERY_UPDATES="10"
MERGE_OBSTACLE_MAP=0
SITE_OBSTACLE_MAP_PATH=""
SCENE_ID="AirSimNH"
WITH_SCENE_INVENTORY=1
REFRESH_SCENE_INVENTORY=0
SCENE_INVENTORY_PATH=""
CAMERAS=("front_center" "0")
CAMERA_RATE_HZ="10"
CAMERA_RESEND_S="0.25"
CAMERA_CAPTURE_EVERY_S="1.0"
WITH_CAMERA=1
WITH_OVERLAY=1
WITH_OCCUPANCY_OVERLAY=0
WITH_SWEPT_VOLUME_OVERLAY=0
WITH_SENSING_EVIDENCE_OVERLAY=1
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
VALIDATION_MIN_OCCUPIED_CELLS="48"
VALIDATION_COMPLETE_REASON="orbit_count_elapsed"
VALIDATION_EXPECT_SEQUENCE=0
VALIDATION_SEQUENCE_STEPS="approach,circle"
VALIDATION_SEQUENCE_STEP_MODES=""
ATTACH=0
EXIT_ON_COMPLETE=1
KILL_EXISTING=1
PROGRESS_FLAG="--progress"
SIM_CONFIG_PATH="$(dirname "$0")/run_mission_config.yaml"

usage() {
    cat <<'EOF'
Usage:
  ./run_mission.sh [options]

Starts the validated AirSim object-conditioned behavior mission stack in tmux.

Prerequisite:
  ./run.sh AirSimNH

Examples:
  ./run_mission.sh
  ./run_mission.sh --attach
  ./run_mission.sh --overlay-debug
  ./run_mission.sh --scene-id AirSimNH --refresh-scene-inventory
  ./run_mission.sh --source-frame-rate-hz 0 --pipeline-timing --frame-producer-timing

Options:
  --session NAME              tmux session name. Default: dedalus-mission
  --build-dir PATH            build dir. Default: ../../build-staging
  --config PATH               mission config. Default: ../../config/core_stack_object_behavior_airsim_existing_object_circle.yml
  --sim-config PATH           sim/validation run config YAML. Default: run_mission_config.yaml (auto-loaded if present)
  --output-dir PATH           output dir. Default: ../../out/object_behavior_airsim_existing_object_circle
  --stream-host HOST          runtime stream host. Default: 127.0.0.1
  --stream-port PORT          runtime stream port. Default: 47770
  --max-frames N              mission-loop --max-frames. Default: 5400
  --shutdown-max-frames N     mission-loop --shutdown-max-frames. Default: 1800
  --safe-height M             Override takeoff/return transit height and bridge takeoff height.
  --behavior-min-height M     Override ExecuteMission minimum behavior height.
  --behavior-duration-s S     mission-loop --behavior-duration-s. Default: 360
  --vehicle-name NAME         AirSim vehicle name. Default: PX4
  --airsim-host HOST          AirSim RPC host. Default: 127.0.0.1
  --airsim-rpc-port PORT      AirSim RPC port. Default: 41451
  --source-frame-rate-hz HZ   Override frame-source producer --rate-hz in an effective config. Use 0 for uncapped.
  --frame-producer-timing     Add --timing-jsonl to the frame producer command.
  --frame-producer-timing-path PATH
                                Timing JSONL output path. Default: <output-dir>/profile/source_frame_bridge_<timestamp>.jsonl
  --pipeline-timing           Enable C++ pipeline timing in an effective config.
  --pipeline-timing-path PATH Timing JSONL output path. Default: <output-dir>/profile/pipeline_<timestamp>.jsonl
  --no-obstacle-map-artifact
                                Do not write full mission obstacle map artifact
  --obstacle-map-artifact PATH  Full mission obstacle map artifact path. Default: <output-dir>/mission_obstacle_map_full.json
  --obstacle-map-site-id ID      Persistent obstacle-memory site id. Default: airsim_neighborhood
  --obstacle-map-site-frame-id ID
                                Persistent obstacle-memory site frame id. Default: airsim_world
  --obstacle-map-mission-id ID   Mission id recorded in obstacle map artifact
  --obstacle-map-write-every-updates N
                                Runtime artifact write cadence. Default: 10 updates
  --merge-obstacle-map          Merge full mission obstacle artifact into persistent site map after mission success
  --site-map-path PATH          Persistent site map path. Default: maps/<site-id>/site_obstacle_map.json
  --no-airsim-preflight       Skip the AirSim RPC preflight check
  --scene-id ID               Scene id for scene inventory artifact. Default: AirSimNH
  --scene-inventory PATH      Scene inventory output path. Default: ../../out/airsim_scene_inventory/<scene-id>.objects.json
  --refresh-scene-inventory   Regenerate scene inventory even if it exists
  --no-scene-inventory        Skip scene inventory generation, provider inventory override, and validation
  --camera CAMERA             AirSim camera to command. May repeat. Default: front_center and 0
  --no-camera                 Do not start camera-pointing bridge
  --no-overlay                Do not start overlay
  --overlay-debug             Enable verbose overlay debug logs/debug JSON. Default: off
  --no-occupancy-overlay      Do not pass occupancy render flags to overlay
  --occupancy-overlay         Re-enable occupancy overlay (overrides sim config default off)
  --no-swept-volume-overlay   Do not pass swept-volume render flags to overlay
  --swept-volume-overlay      Re-enable swept-volume overlay (overrides sim config default off)
  --no-sensing-evidence-overlay
                                Do not pass sensing-volume + obstacle-evidence render flags to overlay
  --sensing-evidence-overlay  Re-enable sensing/evidence overlay (overrides sim config default off)
  --max-occupancy-cells N     Max occupancy cells for overlay. Default: 32
  --no-validation             Do not start post-run validators
  --overlay-rate-hz HZ        overlay update rate. Default: 5
  --overlay-duration-s S      overlay duration; 0 means run until session stops. Default: 0
  --validation-min-orbits N   Circle validator --min-orbits. Default: 2.95
  --validation-radius M       Circle validator --radius. Default: 10.0
  --validation-timeout-s S    Wait timeout for runtime_stop. Default: 300; use 0 to wait forever.
  --validation-min-occupied-cells N
                                Artifact validator minimum occupied GT cells. Default: 48
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
    if [[ "$path" != /* ]]; then
        path="$CALLER_CWD/$path"
    fi
    local dir base
    dir="$(dirname "$path")"
    base="$(basename "$path")"
    if [[ -d "$dir" ]]; then
        printf '%s/%s\n' "$(cd "$dir" && pwd)" "$base"
    else
        printf '%s\n' "$path"
    fi
}

creatable_abs_path() {
    local path="$1"
    if [[ "$path" != /* ]]; then
        path="$CALLER_CWD/$path"
    fi
    local dir base
    dir="$(dirname "$path")"
    base="$(basename "$path")"
    mkdir -p "$dir"
    printf '%s/%s\n' "$(cd "$dir" && pwd)" "$base"
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

Start the simulator first:

  cd ${REPO_ROOT_ABS}/simulation/airsim
  ./run.sh AirSimNH

Then rerun:

  ./run_mission.sh
EOF
    return 1
}

sim_cfg() {
    local key="$1" file="$2"
    [[ -f "$file" ]] || return 0
    python3 -c "import sys, re
key = sys.argv[1]
with open(sys.argv[2]) as f:
    for line in f:
        s = line.strip()
        if not s or s.startswith('#'):
            continue
        m = re.match(r'^' + re.escape(key) + r'\s*:\s*(.+)$', s)
        if m:
            print(m.group(1).strip().strip('\"\"'))
            sys.exit(0)" "$key" "$file" 2>/dev/null || true
}

apply_sim_config() {
    local f="$SIM_CONFIG_PATH"
    [[ -f "$f" ]] || return 0
    local v
    v=$(sim_cfg airsim_host "$f");                   [[ -z "$v" ]] || AIRSIM_HOST="$v"
    v=$(sim_cfg airsim_rpc_port "$f");               [[ -z "$v" ]] || AIRSIM_RPC_PORT="$v"
    v=$(sim_cfg vehicle_name "$f");                  [[ -z "$v" ]] || VEHICLE_NAME="$v"
    v=$(sim_cfg stream_host "$f");                   [[ -z "$v" ]] || STREAM_HOST="$v"
    v=$(sim_cfg stream_port "$f");                   [[ -z "$v" ]] || STREAM_PORT="$v"
    v=$(sim_cfg max_frames "$f");                    [[ -z "$v" ]] || MAX_FRAMES="$v"
    v=$(sim_cfg shutdown_max_frames "$f");           [[ -z "$v" ]] || SHUTDOWN_MAX_FRAMES="$v"
    v=$(sim_cfg behavior_duration_s "$f");           [[ -z "$v" ]] || BEHAVIOR_DURATION_S="$v"
    v=$(sim_cfg overlay_rate_hz "$f");               [[ -z "$v" ]] || OVERLAY_RATE_HZ="$v"
    v=$(sim_cfg overlay_max_occupancy_cells "$f");   [[ -z "$v" ]] || OVERLAY_MAX_OCCUPANCY_CELLS="$v"
    v=$(sim_cfg with_occupancy_overlay "$f")
    [[ "$v" != "false" ]] || WITH_OCCUPANCY_OVERLAY=0; [[ "$v" != "true" ]] || WITH_OCCUPANCY_OVERLAY=1
    v=$(sim_cfg with_swept_volume_overlay "$f")
    [[ "$v" != "false" ]] || WITH_SWEPT_VOLUME_OVERLAY=0; [[ "$v" != "true" ]] || WITH_SWEPT_VOLUME_OVERLAY=1
    v=$(sim_cfg with_sensing_evidence_overlay "$f")
    [[ "$v" != "false" ]] || WITH_SENSING_EVIDENCE_OVERLAY=0; [[ "$v" != "true" ]] || WITH_SENSING_EVIDENCE_OVERLAY=1
    v=$(sim_cfg scene_id "$f");                     [[ -z "$v" ]] || SCENE_ID="$v"
    v=$(sim_cfg validation_timeout_s "$f");          [[ -z "$v" ]] || VALIDATION_TIMEOUT_S="$v"
    v=$(sim_cfg validation_min_orbits "$f");         [[ -z "$v" ]] || VALIDATION_MIN_ORBITS="$v"
    v=$(sim_cfg validation_radius "$f");             [[ -z "$v" ]] || VALIDATION_RADIUS="$v"
    v=$(sim_cfg validation_min_occupied_cells "$f"); [[ -z "$v" ]] || VALIDATION_MIN_OCCUPIED_CELLS="$v"
    v=$(sim_cfg validation_complete_reason "$f");    [[ -z "$v" ]] || VALIDATION_COMPLETE_REASON="$v"
    v=$(sim_cfg validation_expect_sequence "$f")
    [[ "$v" != "true" ]] || VALIDATION_EXPECT_SEQUENCE=1; [[ "$v" != "false" ]] || VALIDATION_EXPECT_SEQUENCE=0
    v=$(sim_cfg validation_sequence_steps "$f");     [[ -z "$v" ]] || VALIDATION_SEQUENCE_STEPS="$v"
    v=$(sim_cfg validation_sequence_step_modes "$f"); [[ -z "$v" ]] || VALIDATION_SEQUENCE_STEP_MODES="$v"
}

# Pre-scan for --sim-config so the config file is applied before the main arg
# loop runs. CLI args in the main loop then override config values.
for (( _i=1; _i<=$#; _i++ )); do
    if [[ "${!_i}" == "--sim-config" ]]; then
        _next=$(( _i + 1 ))
        SIM_CONFIG_PATH="$(abs_path "${!_next}")"
        break
    fi
done
apply_sim_config

while [[ $# -gt 0 ]]; do
    case "$1" in
        --session) SESSION_NAME="$2"; shift 2 ;;
        --build-dir) BUILD_DIR="$(abs_path "$2")"; shift 2 ;;
        --config) CONFIG_PATH="$(abs_path "$2")"; shift 2 ;;
        --sim-config) SIM_CONFIG_PATH="$(abs_path "$2")"; shift 2 ;;
        --output-dir) OUTPUT_DIR="$(creatable_abs_path "$2")"; shift 2 ;;
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
        --source-frame-rate-hz) SOURCE_FRAME_RATE_HZ="$2"; shift 2 ;;
        --frame-producer-timing) WITH_FRAME_PRODUCER_TIMING=1; shift ;;
        --frame-producer-timing-path) FRAME_PRODUCER_TIMING_PATH="$(creatable_abs_path "$2")"; WITH_FRAME_PRODUCER_TIMING=1; shift 2 ;;
        --pipeline-timing) WITH_PIPELINE_TIMING=1; shift ;;
        --pipeline-timing-path) PIPELINE_TIMING_PATH="$(creatable_abs_path "$2")"; WITH_PIPELINE_TIMING=1; shift 2 ;;
        --no-obstacle-map-artifact) OBSTACLE_MAP_ARTIFACT=0; shift ;;
        --obstacle-map-artifact) OBSTACLE_MAP_ARTIFACT_PATH="$(creatable_abs_path "$2")"; OBSTACLE_MAP_ARTIFACT=1; shift 2 ;;
        --obstacle-map-site-id) OBSTACLE_MAP_SITE_ID="$2"; shift 2 ;;
        --obstacle-map-site-frame-id) OBSTACLE_MAP_SITE_FRAME_ID="$2"; shift 2 ;;
        --obstacle-map-mission-id) OBSTACLE_MAP_MISSION_ID="$2"; shift 2 ;;
        --obstacle-map-write-every-updates) OBSTACLE_MAP_WRITE_EVERY_UPDATES="$2"; shift 2 ;;
        --merge-obstacle-map) MERGE_OBSTACLE_MAP=1; OBSTACLE_MAP_ARTIFACT=1; shift ;;
        --site-map-path) SITE_OBSTACLE_MAP_PATH="$(creatable_abs_path "$2")"; shift 2 ;;
        --no-airsim-preflight) AIRSIM_PREFLIGHT=0; shift ;;
        --scene-id) SCENE_ID="$2"; shift 2 ;;
        --scene-inventory) SCENE_INVENTORY_PATH="$(creatable_abs_path "$2")"; shift 2 ;;
        --refresh-scene-inventory) REFRESH_SCENE_INVENTORY=1; shift ;;
        --no-scene-inventory) WITH_SCENE_INVENTORY=0; shift ;;
        --camera)
            if [[ "${CAMERAS[*]}" == "front_center 0" ]]; then CAMERAS=(); fi
            CAMERAS+=("$2")
            shift 2
            ;;
        --no-camera) WITH_CAMERA=0; shift ;;
        --no-overlay) WITH_OVERLAY=0; shift ;;
        --overlay-debug) WITH_OVERLAY_DEBUG=1; shift ;;
        --no-occupancy-overlay) WITH_OCCUPANCY_OVERLAY=0; shift ;;
        --occupancy-overlay) WITH_OCCUPANCY_OVERLAY=1; shift ;;
        --no-swept-volume-overlay) WITH_SWEPT_VOLUME_OVERLAY=0; shift ;;
        --swept-volume-overlay) WITH_SWEPT_VOLUME_OVERLAY=1; shift ;;
        --no-sensing-evidence-overlay) WITH_SENSING_EVIDENCE_OVERLAY=0; shift ;;
        --sensing-evidence-overlay) WITH_SENSING_EVIDENCE_OVERLAY=1; shift ;;
        --max-occupancy-cells) OVERLAY_MAX_OCCUPANCY_CELLS="$2"; shift 2 ;;
        --no-validation) WITH_VALIDATION=0; shift ;;
        --overlay-rate-hz) OVERLAY_RATE_HZ="$2"; shift 2 ;;
        --overlay-duration-s) OVERLAY_DURATION_S="$2"; shift 2 ;;
        --validation-min-orbits) VALIDATION_MIN_ORBITS="$2"; shift 2 ;;
        --validation-radius) VALIDATION_RADIUS="$2"; shift 2 ;;
        --validation-timeout-s) VALIDATION_TIMEOUT_S="$2"; shift 2 ;;
        --validation-min-occupied-cells) VALIDATION_MIN_OCCUPIED_CELLS="$2"; shift 2 ;;
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
POST_MISSION_LOG="$LOG_DIR_ABS/post_mission_${TIMESTAMP}.log"
POST_MISSION_SCRIPT="$LOG_DIR_ABS/post_mission_${TIMESTAMP}.sh"
CAMERA_DEBUG_JSON="$OUTPUT_DIR/camera_pointing_latest.json"
OVERLAY_DEBUG_JSON="$OUTPUT_DIR/overlay_debug_latest.json"
CAMERA_FRAMES_DIR="$OUTPUT_DIR/camera_pointing_frames"
PROFILE_DIR="$OUTPUT_DIR/profile"
if [[ -z "$OBSTACLE_MAP_ARTIFACT_PATH" ]]; then
    OBSTACLE_MAP_ARTIFACT_PATH="$OUTPUT_DIR/mission_obstacle_map_full.json"
fi
MISSION_OBSTACLE_MAP_ARTIFACT_PATH="$OBSTACLE_MAP_ARTIFACT_PATH"
if [[ -z "$SITE_OBSTACLE_MAP_PATH" ]]; then
    SITE_OBSTACLE_MAP_PATH="$REPO_ROOT_ABS/maps/$OBSTACLE_MAP_SITE_ID/site_obstacle_map.json"
fi
if [[ -z "$SCENE_INVENTORY_PATH" ]]; then
    SCENE_INVENTORY_PATH="$REPO_ROOT_ABS/out/airsim_scene_inventory/${SCENE_ID}.objects.json"
fi
if [[ -z "$FRAME_PRODUCER_TIMING_PATH" ]]; then
    FRAME_PRODUCER_TIMING_PATH="$PROFILE_DIR/source_frame_bridge_${TIMESTAMP}.jsonl"
fi
if [[ -z "$PIPELINE_TIMING_PATH" ]]; then
    PIPELINE_TIMING_PATH="$PROFILE_DIR/pipeline_${TIMESTAMP}.jsonl"
fi

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

mkdir -p "$OUTPUT_DIR" "$CAMERA_FRAMES_DIR" "$PROFILE_DIR" "$(dirname "$MISSION_OBSTACLE_MAP_ARTIFACT_PATH")" "$(dirname "$SITE_OBSTACLE_MAP_PATH")"


if [[ -n "$SOURCE_FRAME_RATE_HZ" || "$WITH_FRAME_PRODUCER_TIMING" -eq 1 || "$WITH_PIPELINE_TIMING" -eq 1 ]]; then
    EFFECTIVE_CONFIG_PATH="$OUTPUT_DIR/effective_core_stack_${TIMESTAMP}.yml"
    python3 - "$CONFIG_PATH" "$EFFECTIVE_CONFIG_PATH" "$SOURCE_FRAME_RATE_HZ" "$WITH_FRAME_PRODUCER_TIMING" "$FRAME_PRODUCER_TIMING_PATH" "$WITH_PIPELINE_TIMING" "$PIPELINE_TIMING_PATH" <<'PY'
from __future__ import annotations
import shlex
import sys
from pathlib import Path

src = Path(sys.argv[1])
dst = Path(sys.argv[2])
frame_rate = sys.argv[3]
with_frame_timing = sys.argv[4] == "1"
frame_timing_path = sys.argv[5]
with_pipeline_timing = sys.argv[6] == "1"
pipeline_timing_path = sys.argv[7]

lines = src.read_text(encoding="utf-8").splitlines()
out: list[str] = []
bridge_seen = False
for line in lines:
    if line.startswith("bridge_command:"):
        bridge_seen = True
        command_text = line.split(":", 1)[1].strip()
        parts = shlex.split(command_text)
        def set_option(parts: list[str], flag: str, value: str) -> list[str]:
            if flag in parts:
                idx = parts.index(flag)
                if idx + 1 >= len(parts):
                    raise SystemExit(f"bridge_command has {flag} without value")
                parts[idx + 1] = value
            else:
                parts.extend([flag, value])
            return parts
        if frame_rate:
            parts = set_option(parts, "--rate-hz", frame_rate)
        if with_frame_timing:
            parts = set_option(parts, "--timing-jsonl", frame_timing_path)
        line = "bridge_command: " + shlex.join(parts)
    if not (line.startswith("pipeline_timing_enabled:") or line.startswith("pipeline_timing_output_path:")):
        out.append(line)
if not bridge_seen:
    raise SystemExit("config has no bridge_command line to override")
if with_pipeline_timing:
    out.append("pipeline_timing_enabled: true")
    out.append("pipeline_timing_output_path: " + pipeline_timing_path)
dst.write_text("\n".join(out) + "\n", encoding="utf-8")
PY
    CONFIG_PATH="$EFFECTIVE_CONFIG_PATH"
fi

if [[ "$WITH_SCENE_INVENTORY" -eq 1 ]]; then
    if [[ "$REFRESH_SCENE_INVENTORY" -eq 1 || ! -f "$SCENE_INVENTORY_PATH" ]]; then
        echo "Generating AirSim scene inventory: $SCENE_INVENTORY_PATH"
        python3 "$REPO_ROOT_ABS/simulation/airsim/scripts/airsim-list-objects.py" \
            --host "$AIRSIM_HOST" \
            --rpc-port "$AIRSIM_RPC_PORT" \
            --vehicle-name "$VEHICLE_NAME" \
            --scene-id "$SCENE_ID" \
            --name-regex '.*' \
            --inventory-schema \
            --sort class \
            --output "$SCENE_INVENTORY_PATH"
    else
        echo "Using existing AirSim scene inventory: $SCENE_INVENTORY_PATH"
    fi
fi

MISSION_CMD=(
    "$MISSION_BIN"
    --config "$CONFIG_PATH"
    --output-dir "$OUTPUT_DIR"
    --max-frames "$MAX_FRAMES"
    --shutdown-max-frames "$SHUTDOWN_MAX_FRAMES"
    --world-snapshot-stream-port "$STREAM_PORT"
    --behavior-duration-s "$BEHAVIOR_DURATION_S"
)
if [[ -n "$SAFE_HEIGHT" ]]; then MISSION_CMD+=(--safe-height "$SAFE_HEIGHT"); fi
if [[ -n "$BEHAVIOR_MIN_HEIGHT" ]]; then MISSION_CMD+=(--behavior-min-height "$BEHAVIOR_MIN_HEIGHT"); fi
if [[ -n "$PROGRESS_FLAG" ]]; then MISSION_CMD+=("$PROGRESS_FLAG"); fi

CAMERA_CMD=(
    python3 "$REPO_ROOT_ABS/simulation/airsim/scripts/airsim-camera-pointing-bridge.py"
    --stream-host "$STREAM_HOST" --stream-port "$STREAM_PORT"
    --host "$AIRSIM_HOST" --rpc-port "$AIRSIM_RPC_PORT" --vehicle-name "$VEHICLE_NAME"
    --rate-hz "$CAMERA_RATE_HZ" --resend-s "$CAMERA_RESEND_S"
    --verify-pose --capture-dir "$CAMERA_FRAMES_DIR" --capture-every-s "$CAMERA_CAPTURE_EVERY_S"
    --debug --debug-json "$CAMERA_DEBUG_JSON"
)
if [[ "$EXIT_ON_COMPLETE" -eq 1 ]]; then CAMERA_CMD+=(--exit-on-runtime-stop); fi
for camera in "${CAMERAS[@]}"; do CAMERA_CMD+=(--cameras "$camera"); done

OVERLAY_CMD=(
    python3 "$REPO_ROOT_ABS/simulation/airsim/scripts/airsim-world-overlay.py"
    --stream-host "$STREAM_HOST" --stream-port "$STREAM_PORT"
    --follow --rate-hz "$OVERLAY_RATE_HZ" --clear --label --osd
    --hide-world
    --hide-planned
    --hide-selected
)
if [[ "$WITH_OVERLAY_DEBUG" -eq 1 ]]; then OVERLAY_CMD+=(--debug --debug-json "$OVERLAY_DEBUG_JSON"); fi
if [[ "$WITH_OCCUPANCY_OVERLAY" -eq 1 ]]; then
    OVERLAY_CMD+=(--show-occupancy-summary --show-occupancy-cells --max-occupancy-cells "$OVERLAY_MAX_OCCUPANCY_CELLS")
fi
if [[ "$WITH_SWEPT_VOLUME_OVERLAY" -eq 1 ]]; then OVERLAY_CMD+=(--show-swept-volume); fi
if [[ "$WITH_SENSING_EVIDENCE_OVERLAY" -eq 1 ]]; then
    OVERLAY_CMD+=(
        --show-sensing-volumes
        --show-obstacle-evidence
        --no-sensing-volume-labels
        --obstacle-evidence-shape surfel_patch
        --obstacle-evidence-display-voxel-m 0.50
        --max-obstacle-evidence 160
    )
fi
if [[ "$EXIT_ON_COMPLETE" -eq 1 ]]; then OVERLAY_CMD+=(--exit-on-runtime-stop); fi
if [[ "$OVERLAY_DURATION_S" != "0" ]]; then OVERLAY_CMD+=(--duration-s "$OVERLAY_DURATION_S"); fi

if [[ -n "$SAFE_HEIGHT" ]]; then
    VALIDATION_SAFE_HEIGHT="$SAFE_HEIGHT"
else
    VALIDATION_SAFE_HEIGHT="$(config_value 'mission_options\.flight_takeoff_height_m')"
    if [[ -z "$VALIDATION_SAFE_HEIGHT" ]]; then VALIDATION_SAFE_HEIGHT="$(config_value 'mission_options\.flight_safe_height_m')"; fi
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
    if [event for event in events if isinstance(event, dict) and event.get("event") == "runtime_stop"]:
        print("validation: runtime_stop observed", flush=True)
        break
    if timeout_s > 0 and time.monotonic() - start >= timeout_s:
        raise SystemExit(f"validation: timed out waiting for runtime_stop after {timeout_s}s")
    time.sleep(2.0)
PY
python3 tools/mission/mission-events-summary.py "\$EVENTS" --expect-complete
VALIDATE_MISSION_CMD=(python3 tools/mission/validate-mission-artifacts.py $(printf '%q' "$OUTPUT_DIR") --expect-complete --expect-behavior --safe-height-m $(printf '%q' "$VALIDATION_SAFE_HEIGHT") --landed-height-m 1.0 --expect-occupancy --expect-occupancy-source airsim_ground_truth --expect-min-occupied-cells $(printf '%q' "$VALIDATION_MIN_OCCUPIED_CELLS") --expect-source-object-prefix gt_tree_ --expect-source-object-prefix gt_wall_ --expect-source-object-prefix gt_fence_ --expect-source-object-prefix gt_cable_ --expect-swept-volume)
EOF
)
if [[ "$WITH_SCENE_INVENTORY" -eq 1 ]]; then
    VALIDATION_SHELL+=$'\n'
    VALIDATION_SHELL+="VALIDATE_MISSION_CMD+=(--expect-scene-inventory $(printf '%q' "$SCENE_INVENTORY_PATH"))"
fi
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

POST_MISSION_SHELL=$(cat <<EOF
set -euo pipefail
cd $(printf '%q' "$REPO_ROOT_ABS")

EVENTS=$(printf '%q' "$OUTPUT_DIR/mission_events.jsonl")
TIMEOUT=$(printf '%q' "$VALIDATION_TIMEOUT_S")
MISSION_OBSTACLE_MAP_ARTIFACT_PATH=$(printf '%q' "$MISSION_OBSTACLE_MAP_ARTIFACT_PATH")
SITE_OBSTACLE_MAP_PATH=$(printf '%q' "$SITE_OBSTACLE_MAP_PATH")
OBSTACLE_MAP_SITE_ID=$(printf '%q' "$OBSTACLE_MAP_SITE_ID")
OBSTACLE_MAP_SITE_FRAME_ID=$(printf '%q' "$OBSTACLE_MAP_SITE_FRAME_ID")
MERGE_SITE_MAP_TOOL=$(printf '%q' "$REPO_ROOT_ABS/tools/avoidance/merge_site_obstacle_map.py")

python3 - "\$EVENTS" "\$TIMEOUT" <<'PYWAIT'
import json
import sys
import time
from pathlib import Path

path = Path(sys.argv[1])
timeout_s = float(sys.argv[2])
start = time.monotonic()
last_count = -1

print(f"post-mission: waiting for runtime_stop in {path}", flush=True)
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
        print(f"post-mission: events={len(events)}", flush=True)
        last_count = len(events)

    if any(isinstance(event, dict) and event.get("event") == "runtime_stop" for event in events):
        print("post-mission: runtime_stop observed", flush=True)
        break

    if timeout_s > 0 and time.monotonic() - start >= timeout_s:
        raise SystemExit(f"post-mission: timed out waiting for runtime_stop after {timeout_s}s")

    time.sleep(2.0)
PYWAIT

echo "post-mission: requested site obstacle map merge"
echo "post-mission: mission obstacle map artifact: \$MISSION_OBSTACLE_MAP_ARTIFACT_PATH"
echo "post-mission: site obstacle map: \$SITE_OBSTACLE_MAP_PATH"

if [[ ! -f "\$MERGE_SITE_MAP_TOOL" ]]; then
  echo "post-mission: ERROR missing merge tool: \$MERGE_SITE_MAP_TOOL" >&2
  exit 1
fi

if [[ ! -s "\$MISSION_OBSTACLE_MAP_ARTIFACT_PATH" ]]; then
  echo "post-mission: mission obstacle map artifact not present; skipping merge"
  exit 0
fi

mkdir -p "\$(dirname "\$SITE_OBSTACLE_MAP_PATH")"

echo "post-mission: merging mission obstacle map into site memory"
python3 "\$MERGE_SITE_MAP_TOOL" \
  "\$MISSION_OBSTACLE_MAP_ARTIFACT_PATH" \
  --site-map "\$SITE_OBSTACLE_MAP_PATH" \
  --site-id "\$OBSTACLE_MAP_SITE_ID" \
  --site-frame-id "\$OBSTACLE_MAP_SITE_FRAME_ID"

echo "post-mission: PASS"
EOF
)

printf '%s\n' "$POST_MISSION_SHELL" > "$POST_MISSION_SCRIPT"
chmod +x "$POST_MISSION_SCRIPT"

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
if [[ "$MERGE_OBSTACLE_MAP" -eq 1 ]]; then
    POST_MISSION_RUN_SHELL="bash $(printf '%q' "$POST_MISSION_SCRIPT") 2>&1 | tee $(printf '%q' "$POST_MISSION_LOG")"
    tmux new-window -t "$SESSION_NAME" -n post-mission "bash -lc $(printf '%q' "$(tmux_shell_with_failure_hold "$POST_MISSION_RUN_SHELL")")"
fi
MISSION_ENV_VARS=()
if [[ "$WITH_SCENE_INVENTORY" -eq 1 ]]; then
    MISSION_ENV_VARS+=("DEDALUS_AIRSIM_SCENE_INVENTORY=$SCENE_INVENTORY_PATH")
fi
if [[ "$OBSTACLE_MAP_ARTIFACT" -eq 1 ]]; then
    MISSION_ENV_VARS+=("DEDALUS_MISSION_OBSTACLE_MAP_ARTIFACT=1")
    MISSION_ENV_VARS+=("DEDALUS_MISSION_OBSTACLE_MAP_PATH=$MISSION_OBSTACLE_MAP_ARTIFACT_PATH")
    MISSION_ENV_VARS+=("DEDALUS_MISSION_OBSTACLE_MAP_SITE_ID=$OBSTACLE_MAP_SITE_ID")
    MISSION_ENV_VARS+=("DEDALUS_MISSION_OBSTACLE_MAP_SITE_FRAME_ID=$OBSTACLE_MAP_SITE_FRAME_ID")
    MISSION_ENV_VARS+=("DEDALUS_MISSION_OBSTACLE_MAP_MISSION_ID=$OBSTACLE_MAP_MISSION_ID")
    MISSION_ENV_VARS+=("DEDALUS_MISSION_OBSTACLE_MAP_WRITE_EVERY_UPDATES=$OBSTACLE_MAP_WRITE_EVERY_UPDATES")
fi
MISSION_ENV=""
for env_pair in "${MISSION_ENV_VARS[@]}"; do
    MISSION_ENV+="$(printf '%q' "$env_pair") "
done
MISSION_ENV="${MISSION_ENV% }"
MISSION_SHELL="cd $(printf '%q' "$REPO_ROOT_ABS") && ${MISSION_ENV:+$MISSION_ENV }$(quote_cmd "${MISSION_CMD[@]}") 2>&1 | tee $(printf '%q' "$MISSION_LOG")"
tmux new-window -t "$SESSION_NAME" -n mission-loop "bash -lc $(printf '%q' "$(tmux_shell_with_failure_hold "$MISSION_SHELL")")"
tmux select-window -t "$SESSION_NAME:mission-loop"

echo "✅ AirSim mission stack started in tmux session '$SESSION_NAME'"
if [[ -f "$SIM_CONFIG_PATH" ]]; then
    echo "  sim config: $SIM_CONFIG_PATH"
else
    echo "  sim config: none (create $(dirname "$0")/run_mission_config.yaml to set project defaults)"
fi
echo ""
echo "Mission loop:"
echo "  log:     $MISSION_LOG"
echo "  output:  $OUTPUT_DIR"
echo "  config:  $CONFIG_PATH"
if [[ "$OBSTACLE_MAP_ARTIFACT" -eq 1 ]]; then
    echo "  obstacle map artifact: $MISSION_OBSTACLE_MAP_ARTIFACT_PATH"
else
    echo "  obstacle map artifact: disabled"
fi
if [[ "$MERGE_OBSTACLE_MAP" -eq 1 ]]; then
    echo "  site obstacle map: $SITE_OBSTACLE_MAP_PATH"
else
    echo "  site obstacle map: merge disabled"
fi
if [[ "$WITH_SCENE_INVENTORY" -eq 1 ]]; then
    echo "  inventory override: $SCENE_INVENTORY_PATH"
fi
if [[ -n "$SOURCE_FRAME_RATE_HZ" || "$WITH_FRAME_PRODUCER_TIMING" -eq 1 || "$WITH_PIPELINE_TIMING" -eq 1 ]]; then
    echo "  effective config generated from launch overrides"
fi
echo ""
if [[ -n "$SOURCE_FRAME_RATE_HZ" || "$WITH_FRAME_PRODUCER_TIMING" -eq 1 || "$WITH_PIPELINE_TIMING" -eq 1 ]]; then
    echo "Frame source overrides:"
    if [[ -n "$SOURCE_FRAME_RATE_HZ" ]]; then echo "  source frame rate: $SOURCE_FRAME_RATE_HZ Hz (0 means uncapped)"; fi
    if [[ "$WITH_FRAME_PRODUCER_TIMING" -eq 1 ]]; then echo "  producer timing:   $FRAME_PRODUCER_TIMING_PATH"; fi
    if [[ "$WITH_PIPELINE_TIMING" -eq 1 ]]; then echo "  pipeline timing:   $PIPELINE_TIMING_PATH"; fi
    echo ""
fi
if [[ "$WITH_SCENE_INVENTORY" -eq 1 ]]; then
    echo "Scene inventory:"
    echo "  scene id: $SCENE_ID"
    echo "  path:     $SCENE_INVENTORY_PATH"
    echo "  refresh:  $([[ "$REFRESH_SCENE_INVENTORY" -eq 1 ]] && echo yes || echo no)"
    echo ""
fi
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
    if [[ "$WITH_OVERLAY_DEBUG" -eq 1 ]]; then echo "  debug json: $OVERLAY_DEBUG_JSON"; else echo "  debug json: disabled; pass --overlay-debug to enable"; fi
    echo "  debug:      $([[ "$WITH_OVERLAY_DEBUG" -eq 1 ]] && echo enabled || echo disabled)"
    echo "  occupancy:  $([[ "$WITH_OCCUPANCY_OVERLAY" -eq 1 ]] && echo enabled || echo disabled)"
    echo "  swept vol:  $([[ "$WITH_SWEPT_VOLUME_OVERLAY" -eq 1 ]] && echo enabled || echo disabled)"
    echo "  sense/evid: $([[ "$WITH_SENSING_EVIDENCE_OVERLAY" -eq 1 ]] && echo enabled || echo disabled)"
    if [[ "$WITH_OCCUPANCY_OVERLAY" -eq 1 ]]; then echo "  max cells:  $OVERLAY_MAX_OCCUPANCY_CELLS"; fi
    echo ""
fi
if [[ "$WITH_VALIDATION" -eq 1 ]]; then
    echo "Validation:"
    echo "  log:        $VALIDATION_LOG"
    echo "  script:     $VALIDATION_SCRIPT"
    echo "  timeout:    $VALIDATION_TIMEOUT_S s"
    echo "  validators: mission-events-summary, validate-mission-artifacts, validate-circle-trajectory"
    echo "  complete reason: $VALIDATION_COMPLETE_REASON"
    echo "  min occupied cells: $VALIDATION_MIN_OCCUPIED_CELLS"
    echo ""
fi
if [[ "$MERGE_OBSTACLE_MAP" -eq 1 ]]; then
    echo "Post-mission:"
    echo "  log:        $POST_MISSION_LOG"
    echo "  script:     $POST_MISSION_SCRIPT"
    echo "  task:       merge obstacle map"
    echo "  site map:   $SITE_OBSTACLE_MAP_PATH"
    echo ""
fi

echo "Exit on mission complete: $([[ "$EXIT_ON_COMPLETE" -eq 1 ]] && echo yes || echo no)"
echo "Useful commands:"
echo "  attach: tmux attach -t $SESSION_NAME"
echo "  stop mission stack: tmux kill-session -t $SESSION_NAME"
echo "  stop simulator/PX4: ./stop.sh"
echo ""
echo "Mission command:"
echo "  ${MISSION_ENV:+$MISSION_ENV }$(quote_cmd "${MISSION_CMD[@]}")"
if [[ "$WITH_OVERLAY" -eq 1 ]]; then
    echo "Overlay command:"
    echo "  $(quote_cmd "${OVERLAY_CMD[@]}")"
fi

if [[ "$ATTACH" -eq 1 ]]; then
    tmux attach -t "$SESSION_NAME"
fi


if [[ "$MERGE_OBSTACLE_MAP" -eq 1 ]]; then
    if [[ ! -f "$MISSION_OBSTACLE_MAP_ARTIFACT_PATH" ]]; then
        echo "ERROR: requested --merge-obstacle-map but missing artifact: $MISSION_OBSTACLE_MAP_ARTIFACT_PATH" >&2
        exit 1
    fi
    echo "Merging mission obstacle map into site memory..."
    python3 "$REPO_ROOT_ABS/tools/avoidance/merge_site_obstacle_map.py" \
        "$MISSION_OBSTACLE_MAP_ARTIFACT_PATH" \
        --site-map "$SITE_OBSTACLE_MAP_PATH" \
        --site-id "$OBSTACLE_MAP_SITE_ID" \
        --site-frame-id "$OBSTACLE_MAP_SITE_FRAME_ID"
fi
