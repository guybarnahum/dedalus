#!/usr/bin/env bash
#
# Cleanup script for Project Dedalus Simulation Environment
#
set -e

# ---------------- Auto-yes handling ----------------
AUTO_YES=""
MODE=""

for arg in "$@"; do
  case "$arg" in
    --yes|-y) AUTO_YES="--yes" ;;
    --soft) MODE="soft" ;;
    --hard) MODE="hard" ;;
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

echo "🧹 Project Dedalus - Cleanup Utility"
echo "---------------------------------------------------"

# ---------------- Step 0: Privilege Escalation ----------------
sudo -v
while true; do sudo -n true; sleep 60; kill -0 "$$" || exit; done 2>/dev/null &

if [[ -z "$MODE" ]]; then
  echo "Usage: ./cleanup.sh [--soft | --hard]"
  echo "  --soft : Kills running processes and clears shared memory/build caches."
  echo "  --hard : DANGER. Deletes downloaded frameworks (PX4, iceoryx, etc)."
  exit 1
fi

# ---------------- Phase 1: Terminate Processes ----------------
echo "🛑 Terminating Active Simulation Processes..."

if pgrep -f "Colosseum" > /dev/null; then
   run_and_log "Kill Colosseum" pkill -9 -f "Colosseum" || true
else
   echo "✅ Colosseum not running."
fi

if pgrep -f "px4" > /dev/null; then
   run_and_log "Kill PX4 SITL" pkill -9 -f "px4" || true
else
   echo "✅ PX4 not running."
fi

if pgrep -f "iox-roudi" > /dev/null; then
   run_and_log "Kill iox-roudi" pkill -9 -f "iox-roudi" || true
else
   echo "✅ iox-roudi not running."
fi

if command -v docker &>/dev/null; then
  run_and_log "Stop Dedalus Docker Containers" bash -c 'docker stop $(docker ps -a -q --filter name=dedalus) 2>/dev/null || true'
fi

# ---------------- Phase 2: Soft Reset ----------------
if [[ "$MODE" == "soft" ]]; then
  echo "♻️  Performing Soft Reset..."
  run_and_log "Clear iceoryx shared memory" sudo rm -rf /dev/shm/iox*
  
  # PATH ADJUSTMENT: PX4 is local to this directory now
  if [ -d "PX4-Autopilot" ]; then
    run_and_log "Clear PX4 Build Cache" make -C PX4-Autopilot clean || true
  fi
  
  # PATH ADJUSTMENT: src is one level up
  run_and_log "Clear local CMake build artifacts" bash -c 'rm -rf ../src/build/* 2>/dev/null || true'
  
  echo "✅ Soft reset complete. Ready to rebuild."
  exit 0
fi

# ---------------- Phase 3: Hard Reset ----------------
if [[ "$MODE" == "hard" ]]; then
  echo "⚠️  WARNING: HARD RESET INITIATED."
  if ! ask_yes_no "This will delete PX4, iceoryx source, and Docker images. Continue?"; then
    echo "Aborted."
    exit 0
  fi

  echo "🗑️  Purging Heavy Frameworks..."
  # PATH ADJUSTMENT: PX4 is local, iceoryx is one level up
  run_and_log "Delete PX4 Directory" rm -rf PX4-Autopilot
  run_and_log "Delete iceoryx Build" rm -rf ../infrastructure/iceoryx_build
  run_and_log "Prune Docker Images" docker system prune -af --volumes

  echo "✅ Hard reset complete. Run ./setup.sh to rebuild from scratch."
fi