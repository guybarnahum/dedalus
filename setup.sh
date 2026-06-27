#!/usr/bin/env bash
#
# Setup script for Project Dedalus development and AirSim/PX4 SITL environment (AWS/EC2 GPU)
# - Verifies NVIDIA L4/A10G/T4 GPU presence
# - Esculates and maintains sudo privileges
# - Installs core system dependencies (Docker, CMake, Ninja, build-essential, libacl1-dev, ffmpeg)
# - Installs Remote Visualization (XFCE & NICE DCV)
# - Fetches and builds Eclipse iceoryx (IPC)
# - Fetches and builds PX4 SITL
# - Pre-loads configured Colosseum Environments
#
set -e
# Ensure script runs from the repository root, while staging external dependencies under third_party/.
cd "$(dirname "$0")"
REPO_ROOT="$(pwd)"
AIRSIM_DIR="$REPO_ROOT/simulation/airsim"
THIRD_PARTY_DIR="$REPO_ROOT/third_party"
PX4_DIR="$THIRD_PARTY_DIR/PX4-Autopilot"
ICEORYX_BUILD_DIR="$THIRD_PARTY_DIR/iceoryx_build"
COLOSSEUM_DIR="$THIRD_PARTY_DIR/colosseum_environments"
mkdir -p "$AIRSIM_DIR" "$THIRD_PARTY_DIR" "$COLOSSEUM_DIR"

# ---------------- Configuration ----------------
PRELOAD_ENVS=("Blocks" "AirSimNH" "LandscapeMountains" "Africa_Savannah")
S3_BUCKET="s3://dedalus-sim-assets-colosseum"

# Array of base URLs to cycle through if S3 fails
FALLBACK_MIRRORS=(
    "https://github.com/microsoft/AirSim/releases/download/v1.8.1-linux"
    "https://github.com/microsoft/AirSim/releases/download/v1.7.0-linux"
    "https://sourceforge.net/projects/airsim.mirror/files/v1.7.0-linux/"
)

# ---------------- Auto-yes handling ----------------
AUTO_YES=""
for arg in "$@"; do
  case "$arg" in
    --yes|-y) AUTO_YES="--yes" ;;
    *) ;;
  esac
done

ask_yes_no() {
  local prompt="$1"
  if [[ -n "$AUTO_YES" ]]; then return 0; fi
  read -p "$prompt [y/N] " -n 1 -r
  echo
  [[ $REPLY =~ ^[Yy]$ ]]
}

# ---------------- Global State & Traps ----------------
SPINNER_PID=""

handle_abort() {
  if [[ -n "$SPINNER_PID" ]]; then
    kill "$SPINNER_PID" 2>/dev/null || true
  fi
  tput cnorm 2>/dev/null || true
  printf '\n\033[33m⚠️  Aborted by user. Exiting gracefully...\033[0m\n'
  exit 130
}

trap handle_abort INT TERM
cleanup_render() { tput cnorm 2>/dev/null || true; }
trap cleanup_render EXIT

# ---------------- UI: Run and Log ----------------
run_and_log() {
  local log_file; log_file=$(mktemp)
  local description="$1"; shift

  tput civis 2>/dev/null || true
  local cols; cols=$(tput cols 2>/dev/null || echo 120)
  local c_dim=$'\033[2m'
  local c_reset=$'\033[0m'

  (
    frames=( '⠋ ' '⠙ ' '⠹ ' '⠸ ' '⠼ ' '⠴ ' '⠦ ' '⠧ ' '⠇ ' '⠏ ' )
    i=0
    while :; do
      local last_line=""
      if [[ -s "$log_file" ]]; then
        last_line=$(tail -c 256 "$log_file" 2>/dev/null | tr '\r' '\n' | tail -n 1 | sed -E 's/\x1B\[[0-9;?]*[ -/]*[@-~]//g' | xargs)
      fi

      local prefix="${frames[i]}${description} : "
      local available_space=$((cols - ${#prefix} - 1))
      if (( available_space < 0 )); then available_space=0; fi

      if (( ${#last_line} > available_space )); then
        last_line="${last_line:0:$available_space}"
      fi

      printf '\r\033[K%s%s%s%s%s' "${c_reset}" "${prefix}" "${c_dim}" "${last_line}" "${c_reset}"
      i=$(( (i+1) % ${#frames[@]} ))
      sleep 0.15
    done
  ) &
  
  SPINNER_PID=$!

  if ! "$@" >"$log_file" 2>&1; then
    kill "$SPINNER_PID" &>/dev/null || true
    wait "$SPINNER_PID" &>/dev/null || true
    SPINNER_PID=""
    tput cnorm 2>/dev/null || true
    
    printf '\r\033[K%s❌ %s failed.\n' "${c_reset}" "$description"
    echo "--- ERROR LOG ---"
    cat "$log_file"
    echo "--- END LOG ---"
    rm -f "$log_file"
    exit 1
  fi

  kill "$SPINNER_PID" &>/dev/null || true
  wait "$SPINNER_PID" &>/dev/null || true
  SPINNER_PID=""
  tput cnorm 2>/dev/null || true

  printf '\r\033[K%s✅ %s\n' "${c_reset}" "$description"
  rm -f "$log_file"
}

echo "🚁 Project Dedalus - Simulation Environment Setup"
echo "---------------------------------------------------"

# ---------------- Step 0: Privilege Escalation ----------------
echo "🔐 Requesting administrative privileges..."
sudo -v
while true; do sudo -n true; sleep 60; kill -0 "$$" || exit; done 2>/dev/null &

# ---------------- Step 1: Hardware Validation ----------------
echo "🔎 Checking Hardware Context..."
if command -v nvidia-smi &>/dev/null; then
  GPU_NAME=$(nvidia-smi --query-gpu=name --format=csv,noheader | head -n1)
  CUDA_VER=$(nvidia-smi | grep "CUDA Version" | awk '{print $9}')
  echo "   GPU Detected: $GPU_NAME (CUDA: $CUDA_VER)"
else
  echo "❌ Fatal: nvidia-smi not found. This environment requires an NVIDIA GPU."
  exit 1
fi

# ---------------- Step 2: System Dependencies ----------------
echo "📦 Installing System Dependencies..."
if [ ! -f /var/lib/apt/periodic/update-success-stamp ] || [ $(find /var/lib/apt/periodic/update-success-stamp -mmin +1440) ]; then
  run_and_log "Update APT cache" sudo apt-get update
fi

run_and_log "Install core tools" sudo DEBIAN_FRONTEND=noninteractive apt-get install -y build-essential cmake git wget curl ninja-build python3 python-is-python3 python3-pip libacl1-dev unzip ffmpeg libsqlite3-dev nvidia-cuda-toolkit

if ! command -v docker &>/dev/null; then
  run_and_log "Install Docker Server" sudo apt-get install -y docker.io docker-compose-v2
  sudo usermod -aG docker "$USER"
else
  echo "✅ Docker is already installed."
fi

# ---------------- Step 3: Remote Visualization (NICE DCV) ----------------
echo "🖥️  Configuring Remote Visualization (NICE DCV)..."

# 1. Clean Slate: Kill zombies and contention
run_and_log "Clear GPU Contention" bash -c "
  sudo systemctl stop dcvserver || true
  systemctl --user stop dcv-session.service || true
  sudo pkill -9 AirSimNH || true
  sudo pkill -9 dcv-session || true
  sudo pkill -9 Xorg || true
  sudo rm -rf /tmp/.X11-unix /tmp/.X*-lock
"

# 2. Install missing virtual session components (XDCV)
if [ ! -f "/usr/lib/x86_64-linux-gnu/dcv/dcv-session-manager" ]; then
  run_and_log "Fetch NICE DCV Bundle" bash -c "
    wget -q https://d1uj6qtbmh3dt5.cloudfront.net/nice-dcv-ubuntu2204-x86_64.tgz
    tar -xf nice-dcv-ubuntu2204-x86_64.tgz
  "
  run_and_log "Install DCV & XDCV" bash -c "
    DCV_DIR=\$(ls -d nice-dcv-*-ubuntu2204-x86_64 | head -n 1)
    sudo dpkg -i ./\$DCV_DIR/nice-dcv-server_*.deb ./\$DCV_DIR/nice-xdcv_*.deb || true
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -f
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y xfce4 xfce4-goodies dbus-x11
  "
  rm -rf nice-dcv-ubuntu2204-x86_64.tgz nice-dcv-*-ubuntu2204-x86_64
fi

# 3. Hardware-Software Handshake (Crucial for G6/L4)
run_and_log "Configure NVIDIA Display" bash -c "
  BUS_ID=\$(nvidia-smi --query-gpu=pci.bus_id --format=csv,noheader | head -n 1)
  sudo nvidia-xconfig --preserve-busid --busid=\"\$BUS_ID\" --enable-all-gpus --connected-monitor=DFP-0
  sudo nvidia-smi -pm 1
  sudo ldconfig
"

# 4. Master Reset
run_and_log "Restart DCV Master" sudo systemctl restart dcvserver

# 5. Persistent User-Level Service
run_and_log "Configure DCV Autostart Service" bash -c "
  sudo loginctl enable-linger \$USER
  mkdir -p ~/.config/systemd/user/
  cat << EOF > ~/.config/systemd/user/dcv-session.service
[Unit]
Description=NICE DCV Virtual Session for Project Dedalus
After=network.target

[Service]
Type=simple
# Remove RemainAfterExit=yes so systemd knows when the session is actually gone
ExecStart=/usr/bin/dcv create-session --type virtual --owner %u dedalus-sim --init "/usr/bin/startxfce4"
Restart=on-failure
RestartSec=5

[Install]
WantedBy=default.target
EOF
  systemctl --user daemon-reload
  systemctl --user enable dcv-session.service
  systemctl --user restart dcv-session.service
"

echo -n "⏳ Verifying Windshield Stabilization..."
sleep 5
if dcv list-sessions | grep -q "dedalus-sim"; then
  echo " ✅ DCV SESSION RUNNING"
else
  echo " ❌ DCV SESSION FAILED"
  echo "🔎 Analyzing logs..."
  journalctl --user -u dcv-session.service -n 20 --no-pager
fi

# ---------------- Step 3b: ONNX Runtime C++ SDK ----------------
# Installs the prebuilt GPU SDK so CMake can find onnxruntime::onnxruntime.
# The Python onnxruntime-gpu wheel (Step 5) is a separate install; the C++ SDK
# provides headers + shared library for the dedalus_core build.
ORT_VERSION="1.21.1"
ORT_INSTALL_DIR="/usr/local/onnxruntime"
ORT_TARBALL="onnxruntime-linux-x64-gpu-${ORT_VERSION}.tgz"
ORT_URL="https://github.com/microsoft/onnxruntime/releases/download/v${ORT_VERSION}/${ORT_TARBALL}"

if [ ! -f "${ORT_INSTALL_DIR}/lib/cmake/onnxruntime/onnxruntimeConfig.cmake" ]; then
    run_and_log "Download ONNX Runtime C++ GPU SDK v${ORT_VERSION}" \
        wget -q -O "/tmp/${ORT_TARBALL}" "${ORT_URL}"

    run_and_log "Install ONNX Runtime C++ SDK to ${ORT_INSTALL_DIR}" bash -c "
        set -e
        cd /tmp
        tar -xzf '${ORT_TARBALL}'
        sudo rm -rf '${ORT_INSTALL_DIR}'
        sudo mv 'onnxruntime-linux-x64-gpu-${ORT_VERSION}' '${ORT_INSTALL_DIR}'
        rm -f '${ORT_TARBALL}'
    "

    # The cmake config references lib64/ but the tarball ships lib/.
    run_and_log "Symlink ORT lib → lib64" \
        sudo ln -sfn "${ORT_INSTALL_DIR}/lib" "${ORT_INSTALL_DIR}/lib64"

    # The cmake INTERFACE_INCLUDE_DIRECTORIES points at include/onnxruntime/
    # but headers live directly in include/. Create a subdirectory with symlinks.
    run_and_log "Create ORT include/onnxruntime header links" bash -c "
        set -e
        sudo mkdir -p '${ORT_INSTALL_DIR}/include/onnxruntime'
        for h in '${ORT_INSTALL_DIR}/include/'*.h; do
            [ -f \"\$h\" ] || continue
            sudo ln -sf \"\$h\" '${ORT_INSTALL_DIR}/include/onnxruntime/'\"$(basename \$h)\"
        done
    "
else
    echo "✅ ONNX Runtime C++ SDK already installed at ${ORT_INSTALL_DIR}."
fi

run_and_log "Validate ORT C++ SDK layout" bash -c "
    set -e
    test -f '${ORT_INSTALL_DIR}/lib/libonnxruntime.so'            || { echo 'MISSING: libonnxruntime.so';            exit 1; }
    test -L '${ORT_INSTALL_DIR}/lib64'                            || { echo 'MISSING: lib64 symlink';                exit 1; }
    test -f '${ORT_INSTALL_DIR}/include/onnxruntime_cxx_api.h'   || { echo 'MISSING: onnxruntime_cxx_api.h';        exit 1; }
    test -f '${ORT_INSTALL_DIR}/include/onnxruntime/onnxruntime_cxx_api.h' || { echo 'MISSING: include/onnxruntime/ symlink'; exit 1; }
    test -f '${ORT_INSTALL_DIR}/lib/cmake/onnxruntime/onnxruntimeConfig.cmake' || { echo 'MISSING: cmake config'; exit 1; }
    echo 'ORT C++ SDK layout OK'
"

# ---------------- Step 4: Eclipse iceoryx (IPC) ----------------
echo "⚙️  Building Eclipse iceoryx (IPC)..."
if [ ! -f "$ICEORYX_BUILD_DIR/.installed" ]; then
  mkdir -p "$ICEORYX_BUILD_DIR"
  cd "$ICEORYX_BUILD_DIR"
  if [ ! -d "iceoryx" ]; then
    run_and_log "Clone iceoryx" git clone --branch v2.90.0 https://github.com/eclipse-iceoryx/iceoryx.git
  fi
  cd iceoryx
  run_and_log "Configure iceoryx CMake" cmake -Bbuild -Hiceoryx_meta -DBUILD_STRICT=OFF -DROUDI_ENVIRONMENT=OFF
  run_and_log "Compile iceoryx" sudo cmake --build build --target install --parallel "$(nproc)"
  touch "$ICEORYX_BUILD_DIR/.installed"
  cd "$REPO_ROOT"
  echo "✅ iceoryx built and staged."
else
  echo "✅ iceoryx build directory found. Skipping."
fi

# ---------------- Step 5: PX4 SITL ----------------
echo "✈️  Building PX4 SITL..."
if [ ! -f "$PX4_DIR/.built" ]; then
  if [ -d "$PX4_DIR" ] && [ ! -f "$PX4_DIR/.installed" ]; then
    run_and_log "Remove broken PX4 clone" rm -rf "$PX4_DIR"
  fi
  if [ ! -d "$PX4_DIR" ]; then
    run_and_log "Clone PX4" git clone --recursive https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR"
  fi
  cd "$PX4_DIR"
  if [ ! -f ".installed" ]; then
    run_and_log "Install PX4 dependencies" bash ./Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools
    touch .installed
  fi
  run_and_log "Build PX4 SITL" make px4_sitl
  touch .built
  cd "$REPO_ROOT"
  echo "✅ PX4 SITL built and staged."
else
  echo "✅ PX4 SITL already built. Skipping."
fi

# ----------------- PYTHON VIRTUAL ENVIRONMENT -----------------
echo "🐍 Setting up Python Virtual Environment..."

# 1. Define and ensure the venv exists
VENV_PATH="${DEDALUS_VENV_PATH:-$REPO_ROOT/venv}"

if [ ! -d "$VENV_PATH" ]; then
    echo "📦 Creating fresh venv at $VENV_PATH..."
    python3 -m venv "$VENV_PATH"
fi

# 2. Activate the environment using the absolute path
source "$VENV_PATH/bin/activate"

# 3. Upgrade core packaging tools
run_and_log "Upgrade Python packaging tools" python -m pip install --upgrade pip setuptools wheel

# 4. Install build-time blockers
# AirSim 1.8.1 eagerly imports these during its own metadata generation.
run_and_log "Install AirSim build-time blockers" python -m pip install numpy msgpack-rpc-python

# 5. Install AirSim with build isolation DISABLED.
# Forces pip to use the numpy/msgpackrpc we just installed in this venv.
run_and_log "Install AirSim Python package" python -m pip install airsim --no-build-isolation

# 6. Install the remaining flight stack tools and PX4 Python build deps.
# PX4 generators still expect the Empy 3.x API, including em.RAW_OPT and
# em.BUFFERED_OPT. Keep this pinned so pip does not resolve Empy 4.x.
run_and_log "Install flight Python dependencies" python -m pip install \
  pymavlink \
  pyserial \
  kconfiglib \
  empy==3.3.4 \
  pyyaml \
  jinja2 \
  toml \
  jsonschema \
  pyros-genmsg \
  packaging

run_and_log "Verify PX4 Python build dependencies" python -c "import em, yaml, jinja2, toml, jsonschema, genmsg, packaging, menuconfig, kconfiglib; assert hasattr(em, 'RAW_OPT') and hasattr(em, 'BUFFERED_OPT')"

run_and_log "Install perception ML dependencies (depth export)" python -m pip install \
  torch \
  transformers \
  onnx \
  onnxscript \
  onnxruntime-gpu

run_and_log "Verify perception ML dependencies" python -c "import torch, transformers, onnx, onnxscript, onnxruntime; print('ORT providers:', onnxruntime.get_available_providers())"

echo "✅ Python environment ready at $VENV_PATH"
# --------------------------------------------------------------

# ---------------- Step 6: Pre-load Environments ----------------
echo "🌍 Pre-loading Colosseum Environments..."
mkdir -p "$COLOSSEUM_DIR"

for ENV in "${PRELOAD_ENVS[@]}"; do
    TARGET_DIR="$COLOSSEUM_DIR/${ENV}_LinuxNoEditor"
    EXE_NAME="${ENV}.sh"
    BINARY_NAME="${ENV}.zip"

    if [ ! -f "$TARGET_DIR/$EXE_NAME" ]; then
        echo "   ⬇️  Fetching $ENV..."
        DOWNLOAD_SUCCESS=false

        # 1. Try S3 First
        if aws s3 cp "$S3_BUCKET/$BINARY_NAME" "/tmp/$BINARY_NAME" 2>/dev/null; then
            echo "   ✅ Downloaded $ENV from S3."
            DOWNLOAD_SUCCESS=true
        else
            echo "   ⚠️  S3 failed or unavailable. Checking fallback mirrors..."
            
            # 2. Cycle through fallback mirrors
            for MIRROR in "${FALLBACK_MIRRORS[@]}"; do
                URL="${MIRROR}/${BINARY_NAME}"
                echo "      -> Trying $MIRROR..."
                if wget -q --show-progress -O "/tmp/$BINARY_NAME" "$URL"; then
                    echo "   ✅ Downloaded $ENV from fallback mirror."
                    DOWNLOAD_SUCCESS=true
                    break
                else
                    rm -f "/tmp/$BINARY_NAME"
                fi
            done
        fi
        if [ "$DOWNLOAD_SUCCESS" = true ]; then
             run_and_log "Extract & Format $ENV" bash -c "
                 set -e
                 unzip -q /tmp/$BINARY_NAME -d /tmp/${ENV}_ext
                 rm -rf $TARGET_DIR
                 mkdir -p $TARGET_DIR
                 EXE_PATH=\$(find /tmp/${ENV}_ext -type f -name '*.sh' | grep -v 'CrashReportClient' | head -n 1)
                 if [ -z \"\$EXE_PATH\" ]; then
                     echo '❌ No .sh executable found inside zip!' >&2
                     exit 1
                 fi
                 BASE_DIR=\$(dirname \"\$EXE_PATH\")
                 mv \"\$BASE_DIR\"/* $TARGET_DIR/
                 chmod +x $TARGET_DIR/*.sh
                 rm -rf /tmp/${ENV}_ext /tmp/$BINARY_NAME
             "
        else
             echo "   ❌ Failed to download $ENV from all sources. Skipping."
        fi
    else
        echo "✅ Environment '$ENV' is already pre-loaded."
    fi
done


echo ""
echo "=================================================================="
echo "✅ ENVIRONMENT SETUP COMPLETE"
echo "=================================================================="
echo "🖥️  CONNECTING TO THE 'WINDSHIELD' (NICE DCV):"
echo "   Unreal Engine cannot run over SSH. You must connect visually."
echo ""
echo "   1. Download the NICE DCV Client on your local laptop:"
echo "      https://download.nice-dcv.com/"
echo "   2. Set a password for your user (if you haven't already):"
echo "      Run: sudo passwd $USER"
echo "   3. Open the DCV Client and connect to:"
echo "      $(curl -s ifconfig.me):8443"
echo "   4. Log in using your Ubuntu credentials."
echo ""
echo "🚀 LAUNCHING THE SIMULATION:"
echo "   Once inside the graphical desktop, open a terminal and run:"
echo "   cd ~/dedalus/simulation/airsim && ./run.sh [EnvironmentName]"
echo "   (Example: ./run.sh Neighborhood)"
echo "=================================================================="