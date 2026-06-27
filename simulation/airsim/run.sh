#!/usr/bin/env bash
#
# Master Orchestrator for the Dedalus Virtual Proving Ground
# Boots IPC, PX4 Flight Controller, Unreal Engine Physics Sim, and optionally
# synchronized flight-control plus Dedalus core-stack capture/replay.

set -e
ORIGINAL_ARGS=("$@")
cd "$(dirname "$0")"

# ------------ LOGGING CONFIGURATION ----------------
LOG_DIR="logs"
mkdir -p "$LOG_DIR"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
SIM_LOG="$(pwd)/$LOG_DIR/sim_$TIMESTAMP.log"

# ------------ CLI CONFIGURATION --------------------
TARGET_ENV="AirSimNH"
WITH_CORE_STACK=0
WITH_FLIGHT_CONTROL=0
CORE_BUILD_DIR="../../build-staging"
CORE_CONFIG="../../config/ci/core_stack_airsim_binary_rgb_ego.yaml"
CORE_OUTPUT_DIR="../../out/airsim_run_$TIMESTAMP"
CORE_MAX_FRAMES="0"
CORE_SAMPLING_FPS="5"
CONTROL_START_DELAY_S="10"
FLIGHT_CONTROL_MODE=""
FLIGHT_TRAJECTORY="../../config/behaviors/trajectories/circle_figure8.json"
FLIGHT_SAFE_HEIGHT_M="8"
AIRSIM_CAMERA_WIDTH=""
AIRSIM_CAMERA_HEIGHT=""
AIRSIM_SETTINGS_BACKUP_ABS=""
SIM_CONFIG_PATH="$(pwd)/run_config.yaml"

usage() {
    cat <<'EOF'
Usage:
  ./run.sh [TARGET_ENV] [options]

Examples:
  ./run.sh AirSimNH
  ./run.sh AirSimNH --with-core-stack
  ./run.sh AirSimNH --with-flight-control --with-core-stack --core-sampling-fps 5
  ./run.sh AirSimNH --with-flight-control --with-core-stack --core-max-frames 0
  ./run.sh AirSimNH --airsim-camera-width 640 --airsim-camera-height 360

Options:
  --with-core-stack             Start the C++ core-stack replay/capture side in a tmux window.
  --no-core-stack               Do not start the C++ core-stack side. Default.
  --with-flight-control         Start test-flight.py in a tmux window.
  --no-flight-control           Do not start test-flight.py. Default.
  --flight-control MODE         Optional test-flight.py --control value.
                                If omitted, run.sh does not pass --control and lets
                                test-flight.py use its own default behavior.
  --flight-trajectory PATH      test-flight.py --trajectory value. Default: ../../config/behaviors/trajectories/circle_figure8.json
  --flight-safe-height-m M      test-flight.py --safe-height value. Default: 8
  --control-start-delay-s N     Delay after PX4 window launch before starting velocity-control and aligned capture.
                                Default: 10
  --airsim-camera-width W       Patch launch-directory simulation/airsim/settings.json CaptureSettings Width before launch.
                                A timestamped backup is written next to settings.json.
                                Requires simulator restart because packaged AirSimNH reads settings.json at startup.
  --airsim-camera-height H      Patch launch-directory simulation/airsim/settings.json CaptureSettings Height before launch.
                                A timestamped backup is written next to settings.json.
                                Requires simulator restart because packaged AirSimNH reads settings.json at startup.
  --core-build-dir PATH         Build directory containing apps/dedalus_replay_recording.
                                Default: ../../build-staging
  --core-config PATH            Core-stack config template to use.
                                Default: ../../config/ci/core_stack_airsim_binary_rgb_ego.yaml
  --core-output-dir PATH        Snapshot output directory.
                                Default: ../out/airsim_run_<timestamp>
  --core-sampling-fps FPS       Capture sampling FPS. run.sh writes a generated runtime config
                                with bridge_command rate-hz set to this value. Default: 5
  --core-max-frames N           Frames to consume. If N=0 and --with-flight-control is set,
                                run.sh derives frames from trajectory duration * core-sampling-fps,
                                so capture ends with velocity-control. If N=0 without flight-control,
                                capture runs until the stream ends. Default: 0
  --sim-config PATH             Sim/validation run config YAML. Default: run_config.yaml (auto-loaded if present)
  -h, --help                    Show this help.
EOF
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
    v=$(sim_cfg core_sampling_fps "$f");     [[ -z "$v" ]] || CORE_SAMPLING_FPS="$v"
    v=$(sim_cfg core_max_frames "$f");       [[ -z "$v" ]] || CORE_MAX_FRAMES="$v"
    v=$(sim_cfg flight_safe_height_m "$f");  [[ -z "$v" ]] || FLIGHT_SAFE_HEIGHT_M="$v"
    v=$(sim_cfg control_start_delay_s "$f"); [[ -z "$v" ]] || CONTROL_START_DELAY_S="$v"
    v=$(sim_cfg airsim_camera_width "$f");   [[ -z "$v" ]] || AIRSIM_CAMERA_WIDTH="$v"
    v=$(sim_cfg airsim_camera_height "$f");  [[ -z "$v" ]] || AIRSIM_CAMERA_HEIGHT="$v"
}

# Pre-scan for --sim-config so the config file is applied before the main arg
# loop runs. CLI args in the main loop then override config values.
for (( _i=1; _i<=$#; _i++ )); do
    if [[ "${!_i}" == "--sim-config" ]]; then
        _next=$(( _i + 1 ))
        SIM_CONFIG_PATH="${!_next}"
        break
    fi
done
apply_sim_config

while [[ $# -gt 0 ]]; do
    case "$1" in
        --with-core-stack)
            WITH_CORE_STACK=1
            shift
            ;;
        --no-core-stack)
            WITH_CORE_STACK=0
            shift
            ;;
        --with-flight-control)
            WITH_FLIGHT_CONTROL=1
            shift
            ;;
        --no-flight-control)
            WITH_FLIGHT_CONTROL=0
            shift
            ;;
        --flight-control)
            FLIGHT_CONTROL_MODE="$2"
            shift 2
            ;;
        --flight-trajectory)
            FLIGHT_TRAJECTORY="$2"
            shift 2
            ;;
        --flight-safe-height-m)
            FLIGHT_SAFE_HEIGHT_M="$2"
            shift 2
            ;;
        --control-start-delay-s)
            CONTROL_START_DELAY_S="$2"
            shift 2
            ;;
        --airsim-camera-width)
            AIRSIM_CAMERA_WIDTH="$2"
            shift 2
            ;;
        --airsim-camera-height)
            AIRSIM_CAMERA_HEIGHT="$2"
            shift 2
            ;;
        --core-build-dir)
            CORE_BUILD_DIR="$2"
            shift 2
            ;;
        --core-config)
            CORE_CONFIG="$2"
            shift 2
            ;;
        --core-output-dir)
            CORE_OUTPUT_DIR="$2"
            shift 2
            ;;
        --core-sampling-fps)
            CORE_SAMPLING_FPS="$2"
            shift 2
            ;;
        --core-max-frames)
            CORE_MAX_FRAMES="$2"
            shift 2
            ;;
        --sim-config)
            SIM_CONFIG_PATH="$2"
            shift 2
            ;;
        --core-start-delay-s)
            echo "⚠️ --core-start-delay-s is deprecated; use --control-start-delay-s."
            CONTROL_START_DELAY_S="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        --*)
            echo "❌ Unknown option: $1"
            usage
            exit 1
            ;;
        *)
            TARGET_ENV="$1"
            shift
            ;;
    esac
done

if [[ -n "$AIRSIM_CAMERA_WIDTH" || -n "$AIRSIM_CAMERA_HEIGHT" ]]; then
    if [[ -z "$AIRSIM_CAMERA_WIDTH" || -z "$AIRSIM_CAMERA_HEIGHT" ]]; then
        echo "❌ --airsim-camera-width and --airsim-camera-height must be provided together."
        exit 1
    fi
    if ! [[ "$AIRSIM_CAMERA_WIDTH" =~ ^[0-9]+$ && "$AIRSIM_CAMERA_HEIGHT" =~ ^[0-9]+$ ]]; then
        echo "❌ AirSim camera width/height must be positive integers."
        exit 1
    fi
    if [[ "$AIRSIM_CAMERA_WIDTH" -le 0 || "$AIRSIM_CAMERA_HEIGHT" -le 0 ]]; then
        echo "❌ AirSim camera width/height must be positive integers."
        exit 1
    fi
fi

# ------------ VIDEO & DISPLAY CONFIGURATION --------
SESSION_NAME="dedalus-sim"

DCV_JSON=$(dcv describe-session "$SESSION_NAME" --json 2>/dev/null) || {
    echo "❌ Error: DCV session '$SESSION_NAME' not found. Run ../../setup.sh first."
    exit 1
}

export DISPLAY=$(echo "$DCV_JSON" | grep '"x11-display"' | awk -F'"' '{print $4}')
export XAUTHORITY=$(echo "$DCV_JSON" | grep '"x11-authority"' | awk -F'"' '{print $4}')

if [[ -z "$DISPLAY" || -z "$XAUTHORITY" ]]; then
    echo "❌ Error: Could not resolve DISPLAY or XAUTHORITY from DCV metadata."
    exit 1
fi

xhost +SI:localuser:$(whoami) >/dev/null 2>&1

# ---------------- tmux Auto-Wrapper ----------------
if [ -z "$TMUX" ]; then
    echo "🖥️  Video Configured: Display $DISPLAY | Auth $XAUTHORITY"
    echo "🚀 Spawning simulation in background tmux session ('$SESSION_NAME')..."

    tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true

    TMUX_ARGS=""
    for arg in "${ORIGINAL_ARGS[@]}"; do
        TMUX_ARGS+=" $(printf '%q' "$arg")"
    done

    # Relaunch the script inside tmux and pipe EVERYTHING to the timestamped log.
    # 'tee' allows us to see the logs in 'tmux attach' AND write to the file simultaneously.
    tmux new-session -d -s "$SESSION_NAME" "bash -c './run.sh$TMUX_ARGS 2>&1 | tee $SIM_LOG'"

    echo "✅ Simulation is booting in the background!"
    echo "📝 LOG FILE: $SIM_LOG"
    echo ""
    echo "   -> To tail logs:       tail -f $SIM_LOG"
    echo "   -> To view live GUI:   Switch to your NICE DCV Client"
    echo "   -> To attach tmux:     tmux attach -t $SESSION_NAME"
    echo "   -> To stop the sim:    ./stop.sh"
    exit 0
fi

# ---------------- Configuration ----------------
S3_BUCKET="s3://dedalus-sim-assets-colosseum"
REPO_ROOT_ABS="$(cd ../.. && pwd)"
THIRD_PARTY_DIR_ABS="$REPO_ROOT_ABS/third_party"
SIM_DIR="$THIRD_PARTY_DIR_ABS/colosseum_environments/${TARGET_ENV}_LinuxNoEditor"
PX4_DIR_ABS="$THIRD_PARTY_DIR_ABS/PX4-Autopilot"
VENV_PATH="${DEDALUS_VENV_PATH:-$REPO_ROOT_ABS/venv}"
AIRSIM_CANONICAL_SETTINGS_ABS="$(pwd)/settings.json"
CORE_BUILD_DIR_ABS="$(cd "$CORE_BUILD_DIR" 2>/dev/null && pwd || true)"
CORE_CONFIG_ABS="$(cd "$(dirname "$CORE_CONFIG")" && pwd)/$(basename "$CORE_CONFIG")"
CORE_OUTPUT_DIR_ABS="$(mkdir -p "$CORE_OUTPUT_DIR" && cd "$CORE_OUTPUT_DIR" && pwd)"
CORE_RUNTIME_CONFIG_ABS="$CORE_OUTPUT_DIR_ABS/core_stack_runtime.yaml"

if [[ "$FLIGHT_TRAJECTORY" = /* ]]; then
    FLIGHT_TRAJECTORY_ABS="$FLIGHT_TRAJECTORY"
else
    FLIGHT_TRAJECTORY_ABS="$(cd "$(dirname "$FLIGHT_TRAJECTORY")" && pwd)/$(basename "$FLIGHT_TRAJECTORY")"
fi

trajectory_duration_s() {
    python - "$1" <<'PY'
import json
import sys
from pathlib import Path

path = Path(sys.argv[1])
if not path.exists():
    print("0")
    raise SystemExit(0)

data = json.loads(path.read_text(encoding="utf-8"))
total = 0.0
for segment in data.get("segments", []):
    if segment.get("type") == "velocity_keyframes" and segment.get("keyframes"):
        total += float(segment["keyframes"][-1].get("t", 0.0))
    else:
        total += float(segment.get("duration_s", 0.0))
print(f"{total:.6f}")
PY
}

ceil_frames() {
    python - "$1" "$2" <<'PY'
import math
import sys

duration = float(sys.argv[1])
fps = float(sys.argv[2])
print(max(1, math.ceil(duration * fps)))
PY
}

write_core_runtime_config() {
    local template="$1"
    local output="$2"
    local fps="$3"

    python - "$template" "$output" "$fps" <<'PY'
import re
import sys
from pathlib import Path

template = Path(sys.argv[1])
output = Path(sys.argv[2])
fps = sys.argv[3]
text = template.read_text(encoding="utf-8")

# Keep the config file source-neutral and update only bridge commands that
# expose --rate-hz. If the command has no --rate-hz, leave it unchanged.
text = re.sub(r"(--rate-hz\s+)([^\s#]+)", rf"\g<1>{fps}", text)

output.write_text(
    "# Generated by simulation/airsim/run.sh. Do not edit.\n" + text,
    encoding="utf-8",
)
PY
}

patch_airsim_settings_in_place() {
    local settings_path="$1"
    local width="$2"
    local height="$3"

    python - "$settings_path" "$width" "$height" <<'PY'
import json
import sys
from pathlib import Path

path = Path(sys.argv[1])
width = int(sys.argv[2])
height = int(sys.argv[3])

data = json.loads(path.read_text(encoding="utf-8"))

def normalize_capture_settings(owner):
    settings = owner.setdefault("CaptureSettings", [])
    if not isinstance(settings, list):
        raise SystemExit("CaptureSettings exists but is not a list")

    by_type = {}
    for setting in settings:
        if isinstance(setting, dict) and "ImageType" in setting:
            by_type[int(setting["ImageType"])] = setting

    template = by_type.get(0)
    if template is None:
        template = {
            "ImageType": 0,
            "FOV_Degrees": 84,
            "AutoExposureSpeed": 100,
            "AutoExposureBias": 0,
            "MotionBlurAmount": 0,
        }
        settings.append(template)
        by_type[0] = template

    changed = 0
    for image_type in (-1, 0, 2):
        setting = by_type.get(image_type)
        if setting is None:
            setting = dict(template)
            setting["ImageType"] = image_type
            settings.append(setting)
            by_type[image_type] = setting

        setting["Width"] = width
        setting["Height"] = height
        setting.setdefault("FOV_Degrees", template.get("FOV_Degrees", 84))
        setting.setdefault("AutoExposureSpeed", template.get("AutoExposureSpeed", 100))
        setting.setdefault("AutoExposureBias", template.get("AutoExposureBias", 0))
        setting.setdefault("MotionBlurAmount", template.get("MotionBlurAmount", 0))
        changed += 1

    settings.sort(
        key=lambda item: int(item.get("ImageType", 999))
        if isinstance(item, dict)
        else 999
    )
    return changed

changed = 0
camera_defaults = data.setdefault("CameraDefaults", {})
changed += normalize_capture_settings(camera_defaults)

vehicles = data.get("Vehicles")
if isinstance(vehicles, dict):
    for vehicle in vehicles.values():
        if not isinstance(vehicle, dict):
            continue
        cameras = vehicle.get("Cameras")
        if not isinstance(cameras, dict):
            continue
        for camera in cameras.values():
            if isinstance(camera, dict):
                changed += normalize_capture_settings(camera)

if changed == 0:
    raise SystemExit("no CameraDefaults or vehicle camera CaptureSettings found to update")

path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")

print(f"patched_settings={path}")
print(f"requested_resolution={width}x{height}")

def print_owner(label, owner):
    print(label)
    for setting in owner.get("CaptureSettings", []):
        if isinstance(setting, dict):
            print(
                f"  ImageType={setting.get('ImageType')} "
                f"Width={setting.get('Width')} "
                f"Height={setting.get('Height')} "
                f"FOV={setting.get('FOV_Degrees')}"
            )

print_owner("CameraDefaults", data.get("CameraDefaults", {}))
for vehicle_name, vehicle in data.get("Vehicles", {}).items():
    for camera_name, camera in vehicle.get("Cameras", {}).items():
        print_owner(f"Vehicles.{vehicle_name}.Cameras.{camera_name}", camera)
PY
}

# ---------------- Traps & Cleanup ----------------
cleanup() {
    echo -e "\n🛑 Shutting down simulation stack..."
    pkill -9 -f "dedalus_replay_recording" || true
    pkill -9 -f "airsim-stream-frames" || true
    pkill -9 -f "airsim-capture-frame" || true
    pkill -9 -f "airsim-capture-ego" || true
    pkill -9 -f "test-flight.py" || true
    pkill -9 -f "Linux-Shipping" || true
    pkill -9 -f "AirSimNH" || true
    pkill -9 -f "Blocks" || true
    pkill -9 -f "px4" || true
    pkill -9 -f "iox-roudi" || true
    echo "✅ Simulation terminated."
    if [[ -n "$AIRSIM_SETTINGS_BACKUP_ABS" && -f "$AIRSIM_SETTINGS_BACKUP_ABS" ]]; then
        echo "ℹ️  AirSim settings backup remains available at: $AIRSIM_SETTINGS_BACKUP_ABS"
        echo "   Restore manually with: cp '$AIRSIM_SETTINGS_BACKUP_ABS' '$AIRSIM_CANONICAL_SETTINGS_ABS'"
    fi
    exit 0
}
trap cleanup SIGINT SIGTERM

echo "🚀 Initiating Project Dedalus Simulation Stack..."
echo "🌍 Target Environment: $TARGET_ENV"
echo "⏰ Started at: $(date)"
if [[ -n "$AIRSIM_CAMERA_WIDTH" && -n "$AIRSIM_CAMERA_HEIGHT" ]]; then
    echo "📷 AirSim camera resolution override: ${AIRSIM_CAMERA_WIDTH}x${AIRSIM_CAMERA_HEIGHT}"
    echo "   This packaged AirSimNH runtime honors launch-directory simulation/airsim/settings.json for Scene capture."
    echo "   run.sh will patch that file in place and write a timestamped backup."
fi
if [[ "$WITH_FLIGHT_CONTROL" == "1" ]]; then
    echo "🕹️  Flight control: enabled"
    if [[ -n "$FLIGHT_CONTROL_MODE" ]]; then
        echo "   Control mode: $FLIGHT_CONTROL_MODE"
    else
        echo "   Control mode: test-flight.py default"
    fi
    echo "   Trajectory:   $FLIGHT_TRAJECTORY_ABS"
    echo "   Start delay:  ${CONTROL_START_DELAY_S}s after PX4 window launch"
fi
if [[ "$WITH_CORE_STACK" == "1" ]]; then
    echo "🧠 Core stack replay: enabled"
    echo "   Config template: $CORE_CONFIG_ABS"
    echo "   Build dir:       ${CORE_BUILD_DIR_ABS:-$CORE_BUILD_DIR}"
    echo "   Output dir:      $CORE_OUTPUT_DIR_ABS"
    echo "   Sampling FPS:    $CORE_SAMPLING_FPS"
    echo "   Max frames:      $CORE_MAX_FRAMES"
fi

# ---------------- 1. IPC Daemon ----------------
echo "⚙️  Starting Eclipse iceoryx (iox-roudi)..."
if ! pgrep -f "iox-roudi" > /dev/null; then
    # Redirect internal daemon logs to a specific sub-log for debugging IPC issues
    iox-roudi -m true > "$LOG_DIR/iox_$TIMESTAMP.log" 2>&1 &
    sleep 2
else
    echo "✅ iox-roudi is already running."
fi

# ---------------- 2. Apply Colosseum Settings ----------------
echo "⚙️  Preparing AirSim launch-directory settings.json..."
if [ -f "$AIRSIM_CANONICAL_SETTINGS_ABS" ]; then
    echo "🧾 Canonical settings: $AIRSIM_CANONICAL_SETTINGS_ABS"
    if [[ -n "$AIRSIM_CAMERA_WIDTH" && -n "$AIRSIM_CAMERA_HEIGHT" ]]; then
        AIRSIM_SETTINGS_BACKUP_ABS="${AIRSIM_CANONICAL_SETTINGS_ABS}.bak_${TIMESTAMP}"
        cp "$AIRSIM_CANONICAL_SETTINGS_ABS" "$AIRSIM_SETTINGS_BACKUP_ABS"
        echo "🧾 Settings backup:   $AIRSIM_SETTINGS_BACKUP_ABS"
        patch_airsim_settings_in_place "$AIRSIM_CANONICAL_SETTINGS_ABS" "$AIRSIM_CAMERA_WIDTH" "$AIRSIM_CAMERA_HEIGHT"
        echo "✅ Patched launch-directory settings.json for ${AIRSIM_CAMERA_WIDTH}x${AIRSIM_CAMERA_HEIGHT}."
        echo "   Restore manually with: cp '$AIRSIM_SETTINGS_BACKUP_ABS' '$AIRSIM_CANONICAL_SETTINGS_ABS'"
    else
        echo "✅ Using canonical launch-directory settings.json unchanged."
    fi
else
    echo "❌ Missing simulation/airsim/settings.json"
    exit 1
fi

# ---------------- 3. PX4 Dependency Preflight ----------------
echo "🐍 Checking PX4 Python build dependencies..."
if [ ! -f "$VENV_PATH/bin/activate" ]; then
    echo "❌ Missing Python venv at $VENV_PATH. Run ./setup.sh first."
    exit 1
fi
source "$VENV_PATH/bin/activate"
python -c "import menuconfig, kconfiglib" 2>/dev/null || {
    echo "❌ Missing PX4 dependency: kconfiglib/menuconfig"
    echo "Run: ../../setup.sh --yes"
    exit 1
}
echo "✅ PX4 Python dependencies available."

# ---------------- 4. Launch Unreal Engine Physics ----------------
echo "🎮 Launching Colosseum Physics Engine ($TARGET_ENV)..."
LAUNCH_EXE=$(find "$SIM_DIR" -maxdepth 1 -type f -name "*.sh" | grep -v "CrashReportClient" | head -n 1)

if [ -z "$LAUNCH_EXE" ]; then
    echo "❌ CRITICAL: Could not find a valid .sh executable in $SIM_DIR"
    cleanup
fi

"$LAUNCH_EXE" -windowed -ResX=1280 -ResY=720 &
SIM_PID=$!

echo "⏳ Waiting for AirSim PX4 TCP server on port 4560..."
for i in {1..60}; do
    if ss -ltnp 2>/dev/null | grep -q ':4560'; then
        echo "✅ AirSim is listening on TCP 4560."
        break
    fi

    if ! kill -0 "$SIM_PID" 2>/dev/null; then
        echo "❌ Unreal/AirSim exited before opening TCP 4560."
        cleanup
    fi

    sleep 1
done

if ! ss -ltnp 2>/dev/null | grep -q ':4560'; then
    echo "❌ Timed out waiting for AirSim TCP 4560."
    cleanup
fi

# ---------------- 5. PX4 SITL Flight Controller ----------------
echo "✈️  Booting PX4 SITL (Software In The Loop)..."

PX4_LOG_ABS="$(pwd)/$LOG_DIR/px4_$TIMESTAMP.log"

if [ ! -d "$PX4_DIR_ABS" ]; then
    echo "❌ Missing PX4 checkout: $PX4_DIR_ABS"
    echo "   Run from repo root: ./setup.sh --yes"
    cleanup
fi

tmux new-window -t "$SESSION_NAME" -n px4 \
    "cd '$PX4_DIR_ABS' && source '$VENV_PATH/bin/activate' && PX4_SIM_HOSTNAME=localhost PX4_SIM_HOST_ADDR=127.0.0.1 PX4_GZ_STANDALONE=1 make px4_sitl none_iris 2>&1 | tee '$PX4_LOG_ABS'"

echo "✅ PX4 window started. Attach with: tmux attach -t $SESSION_NAME, then Ctrl-b w → px4"
echo "📝 PX4 log: $PX4_LOG_ABS"

# ---------------- 6. Optional Flight Control ----------------
if [[ "$WITH_FLIGHT_CONTROL" == "1" ]]; then
    FLIGHT_LOG_ABS="$(pwd)/$LOG_DIR/flight_control_$TIMESTAMP.log"
    if [ ! -f "$FLIGHT_TRAJECTORY_ABS" ]; then
        echo "❌ Missing flight trajectory: $FLIGHT_TRAJECTORY_ABS"
        cleanup
    fi

    FLIGHT_CONTROL_ARGS=(
        python scripts/test-flight.py
        --trajectory "$FLIGHT_TRAJECTORY_ABS"
        --safe-height "$FLIGHT_SAFE_HEIGHT_M"
    )

    if [[ -n "$FLIGHT_CONTROL_MODE" ]]; then
        FLIGHT_CONTROL_ARGS=(
            python scripts/test-flight.py
            --control "$FLIGHT_CONTROL_MODE"
            --trajectory "$FLIGHT_TRAJECTORY_ABS"
            --safe-height "$FLIGHT_SAFE_HEIGHT_M"
        )
    fi

    printf -v FLIGHT_CONTROL_CMD '%q ' "${FLIGHT_CONTROL_ARGS[@]}"

    echo "🕹️  Starting flight-control window after ${CONTROL_START_DELAY_S}s delay..."
    tmux new-window -t "$SESSION_NAME" -n flight-control \
        "sleep '$CONTROL_START_DELAY_S'; cd '$(pwd)' && source '$VENV_PATH/bin/activate' && $FLIGHT_CONTROL_CMD 2>&1 | tee '$FLIGHT_LOG_ABS'"

    echo "✅ Flight-control window started. Attach with: tmux attach -t $SESSION_NAME, then Ctrl-b w → flight-control"
    echo "📝 Flight-control log: $FLIGHT_LOG_ABS"
fi

# ---------------- 7. Optional Core Stack Replay ----------------
if [[ "$WITH_CORE_STACK" == "1" ]]; then
    CORE_APP_ABS="${CORE_BUILD_DIR_ABS:-$CORE_BUILD_DIR}/apps/dedalus_replay_recording"
    CORE_LOG_ABS="$(pwd)/$LOG_DIR/core_stack_$TIMESTAMP.log"

    if [ ! -x "$CORE_APP_ABS" ]; then
        echo "❌ Missing executable core-stack app: $CORE_APP_ABS"
        echo "   Build it first from repo root:"
        echo "   cmake -S . -B build-staging -DDEDALUS_BUILD_APPS=ON -DDEDALUS_BUILD_TESTS=ON"
        echo "   cmake --build build-staging -j\$(nproc)"
        cleanup
    fi

    if [ ! -f "$CORE_CONFIG_ABS" ]; then
        echo "❌ Missing core-stack config: $CORE_CONFIG_ABS"
        cleanup
    fi

    write_core_runtime_config "$CORE_CONFIG_ABS" "$CORE_RUNTIME_CONFIG_ABS" "$CORE_SAMPLING_FPS"

    EFFECTIVE_CORE_MAX_FRAMES="$CORE_MAX_FRAMES"
    if [[ "$CORE_MAX_FRAMES" == "0" && "$WITH_FLIGHT_CONTROL" == "1" ]]; then
        TRAJECTORY_DURATION_S="$(trajectory_duration_s "$FLIGHT_TRAJECTORY_ABS")"
        EFFECTIVE_CORE_MAX_FRAMES="$(ceil_frames "$TRAJECTORY_DURATION_S" "$CORE_SAMPLING_FPS")"
        echo "🧠 Derived core max frames from velocity-control duration: ${TRAJECTORY_DURATION_S}s × ${CORE_SAMPLING_FPS} fps = ${EFFECTIVE_CORE_MAX_FRAMES} frames"
    fi

    echo "🧠 Starting core-stack replay window aligned to control start after ${CONTROL_START_DELAY_S}s delay..."
    tmux new-window -t "$SESSION_NAME" -n core-stack \
        "sleep '$CONTROL_START_DELAY_S'; cd '$REPO_ROOT_ABS' && source '$VENV_PATH/bin/activate' && '$CORE_APP_ABS' --config '$CORE_RUNTIME_CONFIG_ABS' --output-dir '$CORE_OUTPUT_DIR_ABS' --max-frames '$EFFECTIVE_CORE_MAX_FRAMES' 2>&1 | tee '$CORE_LOG_ABS'"

    echo "✅ Core-stack window started. Attach with: tmux attach -t $SESSION_NAME, then Ctrl-b w → core-stack"
    echo "📝 Core-stack log: $CORE_LOG_ABS"
    echo "📦 Core-stack snapshots: $CORE_OUTPUT_DIR_ABS"
    echo "🧾 Generated core config: $CORE_RUNTIME_CONFIG_ABS"
fi

wait "$SIM_PID"

cleanup
