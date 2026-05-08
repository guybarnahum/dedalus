#!/usr/bin/env bash
#
# Master Orchestrator for the Dedalus Virtual Proving Ground
# Boots IPC, PX4 Flight Controller, and the Unreal Engine Physics Sim.

set -e
cd "$(dirname "$0")"

# ------------ LOGGING CONFIGURATION ----------------
LOG_DIR="logs"
mkdir -p "$LOG_DIR"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
SIM_LOG="$LOG_DIR/sim_$TIMESTAMP.log"

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
    
    # Relaunch the script inside tmux and pipe EVERYTHING to the timestamped log
    # 'tee' allows us to see the logs in 'tmux attach' AND write to the file simultaneously
    tmux new-session -d -s "$SESSION_NAME" "bash -c './run.sh $* 2>&1 | tee $SIM_LOG'"
    
    echo "✅ Simulation is booting in the background!"
    echo "📝 LOG FILE: $SIM_LOG"
    echo ""
    echo "   -> To tail logs:       tail -f $SIM_LOG"
    echo "   -> To view live GUI:   Switch to your NICE DCV Client"
    echo "   -> To kill the sim:    ./cleanup.sh"
    exit 0
fi

# ---------------- Configuration ----------------
TARGET_ENV="${1:-AirSimNH}"
S3_BUCKET="s3://dedalus-sim-assets-colosseum"
SIM_DIR="colosseum_environments/${TARGET_ENV}_LinuxNoEditor"
VENV_PATH="$HOME/dedalus/venv"

# ---------------- Traps & Cleanup ----------------
cleanup() {
    echo -e "\n🛑 Shutting down simulation stack..."
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

wait "$SIM_PID"

cleanup