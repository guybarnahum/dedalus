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

if tput setaf 0 >/dev/null 2>&1; then
  COLOR_GRAY="$(tput setaf 8)"
  COLOR_RESET="$(tput sgr0)"
else
  COLOR_GRAY=$'\033[90m'
  COLOR_RESET=$'\033[0m'
fi

run_and_log() {
  local log_file; log_file=$(mktemp)
  local description="$1"; shift

  tput civis 2>/dev/null || true
  local prev_render=""
  local cols; cols=$(tput cols 2>/dev/null || echo 120)

  (
    frames=( '⠋ ' '⠙ ' '⠹ ' '⠸ ' '⠼ ' '⠴ ' '⠦ ' '⠧ ' '⠇ ' '⠏ ' )
    i=0
    while :; do
      local last_line=""
      if [[ -s "$log_file" ]]; then
        last_line=$(tail -n 1 "$log_file" | sed -E 's/\x1B\[[0-9;?]*[ -/]*[@-~]//g' | tr -d '\r')
      fi

      local prefix="${frames[i]}${description} : "
      local available_space=$((cols - ${#prefix} - 1))
      if (( available_space < 0 )); then available_space=0; fi

      if (( ${#last_line} > available_space )); then
        last_line="${last_line:0:$available_space}"
      fi

      local render="${COLOR_RESET}${prefix}${COLOR_GRAY}${last_line}${COLOR_RESET}"

      if [[ "$render" != "$prev_render" ]]; then
        printf '\r\033[K%s' "$render"
        prev_render="$render"
      fi

      i=$(( (i+1) % ${#frames[@]} ))
      sleep 0.2
    done
  ) &
  local spinner_pid=$!

  if ! "$@" >"$log_file" 2>&1; then
    kill "$spinner_pid" &>/dev/null || true
    wait "$spinner_pid" &>/dev/null || true
    tput cnorm 2>/dev/null || true
    
    printf '\r\033[K%s' "${COLOR_RESET}"
    printf "❌ %s failed.\n" "$description"
    echo "--- ERROR LOG ---"
    cat "$log_file"
    echo "--- END LOG ---"
    rm -f "$log_file"
    exit 1
  fi

  kill "$spinner_pid" &>/dev/null || true
  wait "$spinner_pid" &>/dev/null || true
  tput cnorm 2>/dev/null || true

  printf '\r\033[K%s' "${COLOR_RESET}"
  printf '✅ %s\n' "$description"
  rm -f "$log_file"
}

cleanup_render() { tput cnorm 2>/dev/null || true; }
trap cleanup_render EXIT INT TERM

echo "🧹 Project Dedalus - Cleanup Utility"
echo "---------------------------------------------------"

if [[ -z "$MODE" ]]; then
  echo "Usage: ./cleanup.sh [--soft | --hard]"
  echo "  --soft : Kills running processes and clears shared memory/build caches."
  echo "  --hard : DANGER. Deletes downloaded frameworks (PX4, iceoryx, etc)."
  exit 1
fi

# ---------------- Phase 1: Terminate Processes (Always Runs) ----------------
echo "🛑 Terminating Active Simulation Processes..."

# Kill Colosseum (Unreal Engine)
if pgrep -f "Colosseum" > /dev/null; then
   run_and_log "Kill Colosseum" pkill -9 -f "Colosseum" || true
else
   echo "✅ Colosseum not running."
fi

# Kill PX4
if pgrep -f "px4" > /dev/null; then
   run_and_log "Kill PX4 SITL" pkill -9 -f "px4" || true
else
   echo "✅ PX4 not running."
fi

# Kill iceoryx Router
if pgrep -f "iox-roudi" > /dev/null; then
   run_and_log "Kill iox-roudi" pkill -9 -f "iox-roudi" || true
else
   echo "✅ iox-roudi not running."
fi

# Stop dangling docker containers
if command -v docker &>/dev/null; then
  run_and_log "Stop Dedalus Docker Containers" docker stop $(docker ps -a -q --filter name=dedalus) 2>/dev/null || true
fi


# ---------------- Phase 2: Soft Reset ----------------
if [[ "$MODE" == "soft" ]]; then
  echo "♻️  Performing Soft Reset..."
  run_and_log "Clear iceoryx shared memory" sudo rm -rf /dev/shm/iox*
  
  if [ -d "simulation/PX4-Autopilot" ]; then
    run_and_log "Clear PX4 Build Cache" make -C simulation/PX4-Autopilot clean || true
  fi
  
  run_and_log "Clear local CMake build artifacts" rm -rf src/build/* 2>/dev/null || true
  
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
  run_and_log "Delete PX4 Directory" rm -rf simulation/PX4-Autopilot
  run_and_log "Delete iceoryx Build" rm -rf infrastructure/iceoryx_build
  run_and_log "Prune Docker Images" docker system prune -af --volumes

  echo "✅ Hard reset complete. Run ./setup.sh to rebuild from scratch."
fi