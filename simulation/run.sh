#!/usr/bin/env bash
#
# Master Orchestrator for the Dedalus Virtual Proving Ground
# Boots IPC, PX4 Flight Controller, Unreal Engine Physics Sim, and optionally
# the Dedalus core-stack replay side against live AirSim bridge providers.

set -e
cd "$(dirname "$0")"

# ------------ LOGGING CONFIGURATION ----------------
LOG_DIR="logs"
mkdir -p "$LOG_DIR"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
SIM_LOG="$LOG_DIR/sim_$TIMESTAMP.log"

# ------------ CLI CONFIGURATION --------------------
TARGET_ENV="AirSimNH"
WITH_CORE_STACK=0
CORE_BUILD_DIR="../build-staging"
CORE_CONFIG="../config/core_stack_airsim_binary_rgb_ego.yaml"
CORE_OUTPUT_DIR="../out/airsim_run_$TIMESTAMP"
CORE_MAX_FRAMES="0"
CORE_START_DELAY_S="10"

usage() {
    cat <<'EOF'
Usage:
  ./run.sh [TARGET_ENV] [options]

Examples:
  ./run.sh AirSimNH
  ./run.sh AirSimNH --with-core-stack
  ./run.sh AirSimNH --with-core-stack --core-max-frames 5

Options:
  --with-core-stack             Start the C++ core-stack replay side in a tmux window.
  --no-core-stack               Do not start the C++ core-stack side. Default.
  --core-build-dir PATH         Build directory containing apps/dedalus_replay_recording.
                                Default: ../build-staging
  --core-config PATH            Core-stack config to use from repo root/simulation context.
                                Default: ../config/core_stack_airsim_binary_rgb_ego.yaml
  --core-output-dir PATH        Snapshot output directory.
                                Default: ../out/airsim_run_<timestamp>
  --core-max-frames N           Frames to consume. 0 means run until stream ends.
                                Default: 0
  --core-start-delay-s N        Delay before starting core stack after PX4 window launch.
                                Default: 10
  -h, --help                    Show this help.
EOF
}

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
        --core-max-frames)
            CORE_MAX_FRAMES="$2"
            shift 2
            ;;
        --core-start-delay-s)
            CORE_START_DELAY_S="$2"
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

# ------------ VIDEO & DISPLAY CONFIGURATION --------
SESSION_NAME="dedalus-sim"

DCV_JSON=$(dcv describe-session "$SESSION_NAME" --json 2>/dev/null) || {
    echo "❌ Error: DCV session '$SESSION_NAME' not found. Run setup.sh first."
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
    
    # Relaunch the script inside tmux and pipe EVERYTHING to the timestamped log.
    # 'tee' allows us to see the logs in 'tmux attach' AND write to the file simultaneously.
    tmux new-session -d -s "$SESSION_NAME" "bash -c './run.sh $* 2>&1 | tee $SIM_LOG'"
    
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
SIM_DIR="colosseum_environments/${TARGET_ENV}_LinuxNoEditor"
VENV_PATH="$HOME/dedalus/venv"
REPO_ROOT_ABS="$(cd .. && pwd)"
CORE_BUILD_DIR_ABS="$(cd "$CORE_BUILD_DIR" 2>/dev/null && pwd || true)"
CORE_CONFIG_ABS="$(cd "$(dirname "$CORE_CONFIG")" && pwd)/$(basename "$CORE_CONFIG")"
CORE_OUTPUT_DIR_ABS="$(mkdir -p "$CORE_OUTPUT_DIR" && cd "$CORE_OUTPUT_DIR" && pwd)"

# ---------------- Traps & Cleanup ----------------
cleanup() {
    echo -e "\n🛑 Shutting down simulation stack..."
    pkill -9 -f "dedalus_replay_recording" || true
    pkill -9 -f "airsim-stream-frames" || true
    pkill -9 -f "airsim-capture-frame" || true
    pkill -9 -f "airsim-capture-ego" || true
    pkill -9 -f "Linux-Shipping" || true
    pkill -9 -f "AirSimNH" || true
    pkill -9 -f "Blocks" || true
    pkill -9 -f "px4" || true
    pkill -9 -f "iox-roudi" || true
    echo "✅ Simulation terminated."
    exit 0
}
trap cleanup SIGINT SIGTERM

echo "🚀 Initiating Project Dedalus Simulation Stack..."
echo "🌍 Target Environment: $TARGET_ENV"
echo "⏰ Started at: $(date)"
if [[ "$WITH_CORE_STACK" == "1" ]]; then
    echo "🧠 Core stack replay: enabled"
    echo "   Config:      $CORE_CONFIG_ABS"
    echo "   Build dir:   ${CORE_BUILD_DIR_ABS:-$CORE_BUILD_DIR}"
    echo "   Output dir:  $CORE_OUTPUT_DIR_ABS"
    echo "   Max frames:  $CORE_MAX_FRAMES"
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
echo "⚙️  Injecting Configuration (settings.json)..."
mkdir -p ~/Documents/AirSim
if [ -f "settings.json" ]; then
    cp settings.json ~/Documents/AirSim/settings.json
    echo "✅ Settings applied."
else
    echo "❌ Missing simulation/settings.json"
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
    echo "Run: ./setup.sh --yes"
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
PX4_DIR_ABS="$(pwd)/PX4-Autopilot"

tmux new-window -t "$SESSION_NAME" -n px4 \
    "cd '$PX4_DIR_ABS' && source '$VENV_PATH/bin/activate' && PX4_SIM_HOSTNAME=localhost PX4_SIM_HOST_ADDR=127.0.0.1 PX4_GZ_STANDALONE=1 make px4_sitl none_iris 2>&1 | tee '$PX4_LOG_ABS'"

echo "✅ PX4 window started. Attach with: tmux attach -t $SESSION_NAME, then Ctrl-b w → px4"
echo "📝 PX4 log: $PX4_LOG_ABS"

# ---------------- 6. Optional Core Stack Replay ----------------
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

    echo "🧠 Starting core-stack replay window after ${CORE_START_DELAY_S}s delay..."
    tmux new-window -t "$SESSION_NAME" -n core-stack \
        "sleep '$CORE_START_DELAY_S'; cd '$REPO_ROOT_ABS' && source '$VENV_PATH/bin/activate' && '$CORE_APP_ABS' --config '$CORE_CONFIG_ABS' --output-dir '$CORE_OUTPUT_DIR_ABS' --max-frames '$CORE_MAX_FRAMES' 2>&1 | tee '$CORE_LOG_ABS'"

    echo "✅ Core-stack window started. Attach with: tmux attach -t $SESSION_NAME, then Ctrl-b w → core-stack"
    echo "📝 Core-stack log: $CORE_LOG_ABS"
    echo "📦 Core-stack snapshots: $CORE_OUTPUT_DIR_ABS"
fi

wait "$SIM_PID"

cleanup