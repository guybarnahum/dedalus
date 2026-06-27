#!/usr/bin/env bash
set -euo pipefail

RUNS="${RUNS:-3}"
CONFIG="${CONFIG:-config/ci/core_stack_object_behavior_airsim_existing_object_circle.yml}"
OUTPUT_ROOT="${OUTPUT_ROOT:-out/repeat_mission_smoke}"
MAX_FRAMES="${MAX_FRAMES:-900}"
SHUTDOWN_MAX_FRAMES="${SHUTDOWN_MAX_FRAMES:-400}"
APP="${APP:-build-staging/apps/dedalus_mission_loop}"

case "${1:-}" in
  -h|--help)
    cat <<'EOF'
Usage:
  RUNS=3 tools/mission/repeat-mission-smoke.sh

Environment overrides:
  RUNS                  Default: 3
  CONFIG                Default: config/ci/core_stack_object_behavior_airsim_existing_object_circle.yml
  OUTPUT_ROOT           Default: out/repeat_mission_smoke
  MAX_FRAMES            Default: 900
  SHUTDOWN_MAX_FRAMES   Default: 400
  APP                   Default: build-staging/apps/dedalus_mission_loop

This is an operator smoke tool. It runs the mission loop repeatedly and expects
any external simulator/bridge dependencies required by CONFIG to already be
available.
EOF
    exit 0
    ;;
esac

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

SUMMARY="tools/mission/mission-events-summary.py"

if [[ ! "$RUNS" =~ ^[0-9]+$ || "$RUNS" -le 0 ]]; then
  echo "RUNS must be a positive integer: $RUNS" >&2
  exit 2
fi

if [[ ! -x "$APP" ]]; then
  echo "Missing executable: $APP" >&2
  echo "Run: cmake --build build-staging -j\$(sysctl -n hw.logicalcpu 2>/dev/null || nproc)" >&2
  echo "Or override APP=/path/to/dedalus_mission_loop" >&2
  exit 2
fi

if [[ ! -f "$CONFIG" ]]; then
  echo "Missing mission config: $CONFIG" >&2
  echo "Override with CONFIG=/path/to/config.yml" >&2
  exit 2
fi

if [[ ! -f "$SUMMARY" ]]; then
  echo "Missing summary helper: $SUMMARY" >&2
  exit 2
fi

mkdir -p "$OUTPUT_ROOT"

for run in $(seq 1 "$RUNS"); do
  run_dir="$OUTPUT_ROOT/run_${run}"
  run_log="$OUTPUT_ROOT/run_${run}.log"
  rm -rf "$run_dir"

  echo "=== mission run ${run}/${RUNS} ==="
  "$APP" \
    --config "$CONFIG" \
    --output-dir "$run_dir" \
    --max-frames "$MAX_FRAMES" \
    --shutdown-max-frames "$SHUTDOWN_MAX_FRAMES" \
    --no-progress 2>&1 | tee "$run_log"

  python3 "$SUMMARY" "$run_dir/mission_events.jsonl" --expect-complete
  echo
done

echo "repeat mission smoke passed: ${RUNS}/${RUNS} run(s) completed"
