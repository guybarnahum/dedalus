#!/usr/bin/env bash
#
# Setup script for Project Dedalus Simulation Environment (AWS/EC2 GPU)
# - Verifies NVIDIA L4/A10G/T4 GPU presence
# - Esculates and maintains sudo privileges
# - Installs core system dependencies (Docker, CMake, Ninja, build-essential)
# - Installs Remote Visualization (XFCE)
# - Fetches and builds Eclipse iceoryx (IPC)
# - Fetches and builds PX4 SITL
#
set -e

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
        # Parse last 256 bytes, translate carriage returns, grab last line, strip ANSI, trim whitespace
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
# Keep-alive: update timestamp until script finishes
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

if [[ ! "$GPU_NAME" == *"NVIDIA L4"* ]] && [[ ! "$GPU_NAME" == *"A10G"* ]] && [[ ! "$GPU_NAME" == *"T4"* ]]; then
   echo "⚠️  Warning: Expected L4, A10G, or T4 GPU. Proceeding anyway..."
fi

# ---------------- Step 2: System Dependencies ----------------
echo "📦 Installing System Dependencies..."
if [ ! -f /var/lib/apt/periodic/update-success-stamp ] || [ $(find /var/lib/apt/periodic/update-success-stamp -mmin +1440) ]; then
  run_and_log "Update APT cache" sudo apt-get update
fi

run_and_log "Install core tools" sudo DEBIAN_FRONTEND=noninteractive apt-get install -y build-essential cmake git wget curl ninja-build python3-pip

if ! command -v docker &>/dev/null; then
  run_and_log "Install Docker Server" sudo apt-get install -y docker.io docker-compose-v2
  sudo usermod -aG docker "$USER"
  echo "   ⚠️ Note: Added $USER to docker group. You may need to logout/login after setup completes."
else
  echo "✅ Docker is already installed."
fi

# ---------------- Step 3: Remote Visualization ----------------
echo "🖥️  Configuring Remote Visualization..."
if ! dpkg -l | grep -q "xfce4"; then
  run_and_log "Install XFCE4 Desktop" sudo DEBIAN_FRONTEND=noninteractive apt-get install -y xfce4 xfce4-goodies dbus-x11 x11-xserver-utils
else
  echo "✅ XFCE4 Desktop already installed."
fi
echo "✅ Remote Visualization Stack verified."

# ---------------- Step 4: Eclipse iceoryx (IPC) ----------------
echo "⚙️  Building Eclipse iceoryx (IPC)..."
# PATH ADJUSTMENT: infrastructure is one level up from the simulation directory
if [ ! -d "../infrastructure/iceoryx_build" ]; then
  mkdir -p ../infrastructure/iceoryx_build
  cd ../infrastructure/iceoryx_build
  if [ ! -d "iceoryx" ]; then
    run_and_log "Clone iceoryx" git clone --branch v2.90.0 https://github.com/eclipse-iceoryx/iceoryx.git
  fi
  cd iceoryx
  run_and_log "Configure iceoryx CMake" cmake -Bbuild -Hiceoryx_meta -DBUILD_STRICT=OFF -DROUDI_ENVIRONMENT=ON
  run_and_log "Compile iceoryx" cmake --build build --target install --parallel "$(nproc)"
  cd ../../../simulation # Return to simulation directory
  echo "✅ iceoryx built and staged."
else
  echo "✅ iceoryx build directory found. Skipping."
fi

# ---------------- Step 5: PX4 SITL ----------------
echo "✈️  Building PX4 SITL..."
# PATH ADJUSTMENT: PX4 is now cloned directly into the current directory (simulation)
if [ ! -d "PX4-Autopilot" ]; then
  run_and_log "Clone PX4" git clone --recursive https://github.com/PX4/PX4-Autopilot.git
  cd PX4-Autopilot
  run_and_log "Install PX4 dependencies" bash ./Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools
  cd ../
  echo "✅ PX4 source staged."
else
  echo "✅ PX4 directory found. Skipping."
fi

echo ""
echo "✅ Setup Complete!"
echo "   Run './sitl_runner.sh' to launch the environment."