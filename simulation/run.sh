#!/usr/bin/env bash
#
# Master Orchestrator for the Dedalus Virtual Proving Ground
# Boots IPC, PX4 Flight Controller, and the Unreal Engine Physics Sim.

set -e
cd "$(dirname "$0")"

# ------------ VIDEO & DISPLAY CONFIGURATION --------
# Dynamically locate the NICE DCV session and its X11 bridge
SESSION_NAME="dedalus-sim"

# 1. Probe DCV for current display and authority file
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

# 2. Grant local permission to the X-server
# This allows the background simulation process to talk to the DCV display
xhost +SI:localuser:$(whoami) >/dev/null 2>&1

echo "🖥️  Video Configured: Display $DISPLAY | Auth $XAUTHORITY"


# ---------------- tmux Auto-Wrapper ----------------
# If we are not already inside a tmux session, relaunch this script inside one!
if ! command -v tmux &>/dev/null; then
    echo "📦 Installing 'tmux' utility..."
    sudo apt-get update >/dev/null && sudo apt-get install -y tmux >/dev/null
fi

if [ -z "$TMUX" ]; then
    SESSION_NAME="dedalus-sim"
    echo "🚀 Spawning simulation in background tmux session ('$SESSION_NAME')..."
    
    # Clean up any lingering dead sessions
    tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true
    
    # Relaunch this exact script with all arguments inside a detached tmux session
    tmux new-session -d -s "$SESSION_NAME" "$0 $*"
    
    echo "✅ Simulation is booting in the background!"
    echo "   The Unreal Engine GUI will pop up on your desktop momentarily."
    echo ""
    echo "   -> To view live logs:  tmux attach -t $SESSION_NAME"
    echo "   -> To exit the logs:   Press Ctrl+B, then D"
    echo "   -> To kill the sim:    ./cleanup.sh"
    exit 0
fi

# ---------------- Configuration ----------------
TARGET_ENV="${1:-Blocks}"
S3_BUCKET="s3://dedalus-sim-assets-colosseum"
COLOSSEUM_RELEASE_TAG="v2.0.0-beta.0"

BINARY_NAME="${TARGET_ENV}.zip"
SIM_DIR="colosseum_environments/${TARGET_ENV}_LinuxNoEditor"

# Array of base URLs to cycle through if S3 fails
FALLBACK_MIRRORS=(
    "https://github.com/microsoft/AirSim/releases/download/v1.8.1-linux"
    "https://github.com/microsoft/AirSim/releases/download/v1.7.0-linux"
)

# ---------------- Traps & Cleanup ----------------
cleanup() {
    echo -e "\n🛑 Shutting down simulation stack..."
    pkill -9 -f "Linux-Shipping" || true
    pkill -9 -f "px4" || true
    pkill -9 -f "iox-roudi" || true
    echo "✅ Simulation terminated."
    exit 0
}
trap cleanup SIGINT SIGTERM

echo "🚀 Initiating Project Dedalus Simulation Stack..."
echo "🌍 Target Environment: $TARGET_ENV"

# ---------------- 0. Prerequisite Check ----------------
if ! command -v unzip &>/dev/null; then
    echo "📦 Installing 'unzip' utility..."
    sudo apt-get update >/dev/null && sudo apt-get install -y unzip >/dev/null
fi

# ---------------- 1. IPC Daemon ----------------
echo "⚙️  Starting Eclipse iceoryx (iox-roudi)..."
if ! pgrep -f "iox-roudi" > /dev/null; then
    iox-roudi -m true > /tmp/iox-roudi.log 2>&1 &
    sleep 2
else
    echo "✅ iox-roudi is already running."
fi

# ---------------- 2. Fetch Colosseum (Cascading Fallback) ----------------
if [ ! -d "$SIM_DIR" ]; then
    echo "📦 Environment '$TARGET_ENV' not found locally."
    mkdir -p colosseum_environments
    DOWNLOAD_SUCCESS=false
    
    echo "⬇️  Attempting to pull from S3 ($S3_BUCKET)..."
    if aws s3 cp "$S3_BUCKET/$BINARY_NAME" "/tmp/$BINARY_NAME" 2>/dev/null; then
        echo "✅ Downloaded from S3."
        DOWNLOAD_SUCCESS=true
    else
        echo "⚠️  S3 download failed. Checking fallback mirrors..."
        for MIRROR in "${FALLBACK_MIRRORS[@]}"; do
            URL="${MIRROR}/${BINARY_NAME}"
            echo "   -> Trying $MIRROR..."
            if wget -q --show-progress -O "/tmp/$BINARY_NAME" "$URL"; then
                echo "✅ Downloaded from fallback mirror."
                DOWNLOAD_SUCCESS=true
                break
            else
                rm -f "/tmp/$BINARY_NAME"
            fi
        done
    fi
    
    if [ "$DOWNLOAD_SUCCESS" = false ]; then
        echo "❌ CRITICAL: Failed to download '$TARGET_ENV' from all sources."
        echo ""
        echo "📂 Currently Installed Environments:"
        if [ -d "colosseum_environments" ] && [ "$(ls -A colosseum_environments)" ]; then
            ls -1 colosseum_environments | grep "_LinuxNoEditor" | sed 's/_LinuxNoEditor//' | sed 's/^/   - /' || echo "   (None)"
        else
            echo "   (None)"
        fi
        echo ""
        cleanup
    fi
    
    echo "📦 Extracting & Formatting (This may take a minute)..."
    unzip -q "/tmp/$BINARY_NAME" -d "/tmp/${TARGET_ENV}_ext"
    rm -rf "$SIM_DIR"
    mkdir -p "$SIM_DIR"
    
    EXE_PATH=$(find "/tmp/${TARGET_ENV}_ext" -type f -name "*.sh" | grep -v "CrashReportClient" | head -n 1)
    if [ -z "$EXE_PATH" ]; then
        echo "❌ CRITICAL: No .sh executable found inside downloaded zip! The archive may be corrupted or for Windows."
        rm -rf "/tmp/${TARGET_ENV}_ext" "/tmp/$BINARY_NAME"
        cleanup
    fi
    BASE_DIR=$(dirname "$EXE_PATH")
    mv "$BASE_DIR"/* "$SIM_DIR"/
    rm -rf "/tmp/${TARGET_ENV}_ext" "/tmp/$BINARY_NAME"
    chmod +x "$SIM_DIR"/*.sh
    echo "✅ Extraction complete."
fi

# ---------------- 3. PX4 SITL Flight Controller ----------------
echo "✈️  Booting PX4 SITL (Software In The Loop)..."
cd PX4-Autopilot
make px4_sitl none_iris > /tmp/px4.log 2>&1 &
cd ../
sleep 3
echo "✅ PX4 daemon active."

# ---------------- 4. Apply Colosseum Settings ----------------
echo "⚙️  Injecting Configuration (settings.json)..."
mkdir -p ~/Documents/AirSim
if [ -f "settings.json" ]; then
    cp settings.json ~/Documents/AirSim/settings.json
    echo "✅ Settings applied."
else
    echo "⚠️  No settings.json found in $(pwd). Colosseum will use defaults."
fi

# ---------------- 5. Launch Unreal Engine Physics ----------------
echo "🎮 Launching Colosseum Physics Engine ($TARGET_ENV)..."

LAUNCH_EXE=$(find "$SIM_DIR" -maxdepth 1 -type f -name "*.sh" | grep -v "CrashReportClient" | head -n 1)

if [ -z "$LAUNCH_EXE" ]; then
    echo "❌ CRITICAL: Could not find a valid .sh executable in $SIM_DIR"
    cleanup
fi

# Runs windowed so you can see the terminal and the drone simultaneously
"$LAUNCH_EXE" -windowed -ResX=1280 -ResY=720

# If the Unreal Engine GUI is closed, trigger cleanup automatically
cleanup