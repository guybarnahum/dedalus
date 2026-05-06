#!/usr/bin/env bash
#
# Master Orchestrator for the Dedalus Virtual Proving Ground
# Boots IPC, PX4 Flight Controller, and the Unreal Engine Physics Sim.

set -e
cd "$(dirname "$0")"

# ---------------- Configuration ----------------
TARGET_ENV="${1:-Blocks}"
S3_BUCKET="s3://dedalus-sim-assets-colosseum"
COLOSSEUM_RELEASE_TAG="v2.0.0-beta.0"

BINARY_NAME="${TARGET_ENV}.zip"
SIM_DIR="colosseum_environments/${TARGET_ENV}_LinuxNoEditor"
EXE_NAME="${TARGET_ENV}.sh"

# Array of base URLs to cycle through if S3 fails
FALLBACK_MIRRORS=(
    "https://github.com/CodexLabsLLC/Colosseum/releases/download/${COLOSSEUM_RELEASE_TAG}"
    "https://github.com/microsoft/AirSim/releases/download/v1.8.1-linux"
)

# ---------------- Traps & Cleanup ----------------
cleanup() {
    echo -e "\n🛑 Shutting down simulation stack..."
    pkill -9 -f "$EXE_NAME" || true
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
if [ ! -f "$SIM_DIR/$EXE_NAME" ]; then
    echo "📦 Environment '$TARGET_ENV' not found locally."
    mkdir -p colosseum_environments
    DOWNLOAD_SUCCESS=false
    
    # 1. Try S3 First
    echo "⬇️  Attempting to pull from S3 ($S3_BUCKET)..."
    if aws s3 cp "$S3_BUCKET/$BINARY_NAME" "/tmp/$BINARY_NAME" 2>/dev/null; then
        echo "✅ Downloaded from S3."
        DOWNLOAD_SUCCESS=true
    else
        echo "⚠️  S3 download failed. Checking fallback mirrors..."
        
        # 2. Cycle through fallback mirrors
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
    
    # Check if all downloads failed
    if [ "$DOWNLOAD_SUCCESS" = false ]; then
        echo "❌ CRITICAL: Failed to download '$TARGET_ENV' from all sources."
        cleanup
    fi
    
    echo "📦 Extracting & Formatting (This may take a minute)..."
    unzip -q "/tmp/$BINARY_NAME" -d "/tmp/${TARGET_ENV}_ext"
    mv "/tmp/${TARGET_ENV}_ext/"* "$SIM_DIR" 2>/dev/null || mv "/tmp/${TARGET_ENV}_ext/LinuxNoEditor" "$SIM_DIR" 2>/dev/null || mv "/tmp/${TARGET_ENV}_ext" "$SIM_DIR"
    rm -rf "/tmp/${TARGET_ENV}_ext" "/tmp/$BINARY_NAME"
    chmod +x "$SIM_DIR/$EXE_NAME"
    echo "✅ Extraction complete."
fi

# ---------------- 3. PX4 SITL Flight Controller ----------------
echo "✈️  Booting PX4 SITL (Software In The Loop)..."
cd PX4-Autopilot
# Run PX4 in the background. It listens on localhost for MAVLink connections.
make px4_sitl none_iris > /tmp/px4.log 2>&1 &
cd ../
sleep 3
echo "✅ PX4 daemon active."

# ---------------- 4. Launch Unreal Engine Physics ----------------
echo "🎮 Launching Colosseum Physics Engine ($TARGET_ENV)..."
echo "⚠️  Keep this terminal open. Press Ctrl-C to safely shut down all systems."

# Runs windowed so you can see the terminal and the drone simultaneously
./$SIM_DIR/$EXE_NAME -windowed -ResX=1280 -ResY=720

# If the user closes the Colosseum window manually, trigger cleanup
cleanup