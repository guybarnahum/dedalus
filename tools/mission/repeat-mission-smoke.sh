#!/usr/bin/env bash
set -euo pipefail

RUNS="${RUNS:-3}"
CONFIG="${CONFIG:-config/core_stack_trajectory_mission_placeholder.yaml}"
OUTPUT_ROOT="${OUTPUT_ROOT:-out/repeat_mission_smoke}"
MAX_FRAMES="${MAX_FRAMES:-900}"
SHUTDOWN_MAX_FRAMES="${SHUTDOWN_MAX_FRAMES:-400}"

case "${1:-}" in
  -h|--help)
    cat <<'EOF'
Usage:
  RUNS=3 tools/mission/repeat-mission-smoke.sh

Environment overrides:
  RUNS                  Default: 3
  CONFIG                Default: config/core_stack_trajectory_mission_placeholder.yaml
  OUTPUT_ROOT           Default: out/repeat_mission_smoke
  MAX_FRAMES            Default: 900
  SHUTDOWN_MAX_FRAMES   Default: 400
EOF
    exit 0
    ;;
esac

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

APP="./build-staging/apps/dedalus_mission_loop"
SUMMARY="./tools/mission/mission-events-summary.py"

if [[ ! -x "$APP" ]]; then
  echo "Missing executable: $APP" >&2
  echo "Run: cmake --build build-staging -j\$(nproc)" >&2
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
