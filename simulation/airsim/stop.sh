#!/usr/bin/env bash
#
# Stop script for Project Dedalus Simulation Runtime
#
# This is intentionally lighter than cleanup.sh.
# It stops the active runtime stack but does NOT:
#   - clear PX4 build cache
#   - clear local CMake artifacts
#   - delete PX4 / iceoryx / environments
#   - close or reset NICE DCV
#
# Use cleanup.sh only when you intentionally want a reset/rebuild.
#

set -e
cd "$(dirname "$0")"

SESSION_NAME="dedalus-sim"

echo "🛑 Project Dedalus - Stop Simulation"
echo "---------------------------------------------------"

stop_process_group() {
  local description="$1"
  local pattern="$2"

  if pgrep -f "$pattern" >/dev/null 2>&1; then
    echo "→ Stopping $description..."
    pkill -TERM -f "$pattern" 2>/dev/null || true
  else
    echo "✅ $description not running."
  fi
}

kill_process_group() {
  local description="$1"
  local pattern="$2"

  if pgrep -f "$pattern" >/dev/null 2>&1; then
    echo "→ Force killing $description..."
    pkill -KILL -f "$pattern" 2>/dev/null || true
  fi
}

stop_tmux_session() {
  if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    echo "→ Stopping tmux session '$SESSION_NAME'..."

    # Ask the main run.sh process to handle SIGTERM through its trap.
    # The trap in run.sh terminates the simulation children.
    tmux send-keys -t "$SESSION_NAME" C-c 2>/dev/null || true
    sleep 2

    if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
      tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true
    fi
  else
    echo "✅ tmux session '$SESSION_NAME' not running."
  fi
}

echo "Stopping runtime tmux/session state..."
stop_tmux_session

echo "Stopping runtime processes..."

# Unreal / Colosseum / AirSim packaged binaries.
stop_process_group "Unreal Linux-Shipping" "Linux-Shipping"
stop_process_group "AirSimNH" "AirSimNH"
stop_process_group "Blocks" "Blocks"
stop_process_group "Colosseum" "Colosseum"

# PX4 SITL and iceoryx runtime daemon.
stop_process_group "PX4 SITL" "px4"
stop_process_group "iox-roudi" "iox-roudi"

sleep 2

echo "Checking for stubborn runtime processes..."
kill_process_group "Unreal Linux-Shipping" "Linux-Shipping"
kill_process_group "AirSimNH" "AirSimNH"
kill_process_group "Blocks" "Blocks"
kill_process_group "Colosseum" "Colosseum"
kill_process_group "PX4 SITL" "px4"
kill_process_group "iox-roudi" "iox-roudi"

# Release TCP 4560 (AirSim ↔ PX4 MAVLink bridge port).
# AirSim bind()s this port on startup; if a previous AirSim process didn't
# exit cleanly the socket lingers, causing EADDRINUSE on the next run.sh.
if command -v fuser &>/dev/null; then
  if sudo fuser 4560/tcp >/dev/null 2>&1; then
    echo "→ Releasing TCP 4560 (stale AirSim/PX4 socket)..."
    sudo fuser -k 4560/tcp 2>/dev/null || true
  fi
fi

echo ""
echo "✅ Simulation stopped."
echo ""
echo "No build artifacts were removed."
echo "Use ./cleanup.sh --soft --yes only when you want to reset/rebuild state."
