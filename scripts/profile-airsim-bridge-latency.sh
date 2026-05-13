#!/usr/bin/env bash
set -euo pipefail

# Profile AirSim -> binary bridge -> pipe transport -> C++ FrameSource latency.
# AirSim must already be running. The key metric is frame_source.next_frame.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$ROOT_DIR/build-staging"
APP_PATH="$BUILD_DIR/apps/dedalus_replay_recording"
SUMMARY_SCRIPT="$ROOT_DIR/scripts/summarize-pipeline-profile.py"

FRAMES=50
FPS=5
WIDTH=""
HEIGHT=""
HOST="127.0.0.1"
RPC_PORT="41451"
VEHICLE_NAME="PX4"
CAMERA_NAME="front_center"
OUTPUT_ROOT="$ROOT_DIR/out/airsim_bridge_latency_$(date +%Y%m%d_%H%M%S)"
SKIP_BUILD=0
SKIP_CTEST=0
RUN_PPM=1
PROGRESS="auto"
PROGRESS_PID=""

usage() {
  cat <<'EOF'
Usage:
  ./scripts/profile-airsim-bridge-latency.sh [options]

Options:
  --frames N              Number of frames per profiling pass. Default: 50
  --fps FPS               AirSim bridge sampling FPS. Default: 5
  --width W               Expected AirSim frame width. Optional throughput metadata.
  --height H              Expected AirSim frame height. Optional throughput metadata.
  --host HOST             AirSim RPC host. Default: 127.0.0.1
  --rpc-port PORT         AirSim RPC port. Default: 41451
  --vehicle-name NAME     AirSim vehicle name. Default: PX4
  --camera-name NAME      AirSim camera name. Default: front_center
  --output-root PATH      Output root. Default: out/airsim_bridge_latency_<timestamp>
  --skip-build            Skip cmake build.
  --skip-ctest            Skip CTest.
  --no-ppm                Only run bridge-only profile, skip ppm annotation profile.
  --progress              Force progress monitor on.
  --no-progress           Disable progress monitor.
  -h, --help              Show this help.

The bridge latency proxy is frame_source.next_frame.
Absolute p95 latency capacity thresholds:
  GREEN  p95 <= 33.3 ms  approximately 30 FPS capable
  YELLOW p95 <= 66.7 ms  approximately 15 FPS capable
  RED    p95  > 66.7 ms  below 15 FPS bridge/capture/read capacity

Progress is implemented outside the measured C++ pipeline. It polls the timing
JSONL line count about once per second and prints one rewritten terminal line.
It is disabled automatically when stdout is not a TTY.

Stage summary formatting is delegated to scripts/summarize-pipeline-profile.py
so ad-hoc and full-profile summaries stay identical.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --frames) FRAMES="$2"; shift 2 ;;
    --fps) FPS="$2"; shift 2 ;;
    --width) WIDTH="$2"; shift 2 ;;
    --height) HEIGHT="$2"; shift 2 ;;
    --host) HOST="$2"; shift 2 ;;
    --rpc-port) RPC_PORT="$2"; shift 2 ;;
    --vehicle-name) VEHICLE_NAME="$2"; shift 2 ;;
    --camera-name) CAMERA_NAME="$2"; shift 2 ;;
    --output-root) OUTPUT_ROOT="$2"; shift 2 ;;
    --skip-build) SKIP_BUILD=1; shift ;;
    --skip-ctest) SKIP_CTEST=1; shift ;;
    --no-ppm) RUN_PPM=0; shift ;;
    --progress) PROGRESS="on"; shift ;;
    --no-progress) PROGRESS="off"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 2 ;;
  esac
done

log() { printf '\033[1;36m[airsim-bridge-profile]\033[0m %s\n' "$*"; }
fail() { printf '\033[1;31m[airsim-bridge-profile] ERROR:\033[0m %s\n' "$*" >&2; exit 1; }

positive_int() { [[ "$1" =~ ^[0-9]+$ ]] && [[ "$1" -gt 0 ]]; }
positive_number() {
  python3 - "$1" <<'PY'
import sys
try:
    v = float(sys.argv[1])
except ValueError:
    raise SystemExit(1)
raise SystemExit(0 if v > 0 else 1)
PY
}

progress_enabled() {
  if [[ "$PROGRESS" == "on" ]]; then
    return 0
  fi
  if [[ "$PROGRESS" == "off" ]]; then
    return 1
  fi
  [[ -t 1 ]]
}

count_profile_rows() {
  local profile_path="$1"
  if [[ -f "$profile_path" ]]; then
    wc -l < "$profile_path" 2>/dev/null | tr -d '[:space:]'
  else
    printf '0'
  fi
}

start_progress_monitor() {
  local pass_name="$1"
  local profile_path="$2"
  local expected_frames="$3"

  progress_enabled || return 0

  (
    local count=0
    local percent=0
    while true; do
      count="$(count_profile_rows "$profile_path")"
      if [[ "$count" =~ ^[0-9]+$ && "$expected_frames" -gt 0 ]]; then
        percent=$(( count * 100 / expected_frames ))
        if (( percent > 100 )); then percent=100; fi
      else
        percent=0
      fi
      printf '\r\033[K[airsim-bridge-profile] %s progress: %s/%s frames (%s%%)' \
        "$pass_name" "$count" "$expected_frames" "$percent"
      sleep 1
    done
  ) &
  PROGRESS_PID="$!"
}

stop_progress_monitor() {
  local pass_name="$1"
  local profile_path="$2"
  local expected_frames="$3"

  if [[ -n "$PROGRESS_PID" ]]; then
    kill "$PROGRESS_PID" 2>/dev/null || true
    wait "$PROGRESS_PID" 2>/dev/null || true
    PROGRESS_PID=""
  fi

  if progress_enabled; then
    local final_count
    final_count="$(count_profile_rows "$profile_path")"
    printf '\r\033[K[airsim-bridge-profile] %s progress: %s/%s frames (done)\n' \
      "$pass_name" "$final_count" "$expected_frames"
  fi
}

cleanup_progress() {
  if [[ -n "$PROGRESS_PID" ]]; then
    kill "$PROGRESS_PID" 2>/dev/null || true
    wait "$PROGRESS_PID" 2>/dev/null || true
    PROGRESS_PID=""
    if progress_enabled; then
      printf '\r\033[K'
    fi
  fi
}
trap cleanup_progress EXIT INT TERM

positive_int "$FRAMES" || fail "--frames must be a positive integer, got: $FRAMES"
positive_number "$FPS" || fail "--fps must be a positive number, got: $FPS"

cd "$ROOT_DIR"
OUTPUT_ROOT="$(mkdir -p "$OUTPUT_ROOT" && cd "$OUTPUT_ROOT" && pwd)"

log "repo root: $ROOT_DIR"
log "output root: $OUTPUT_ROOT"
log "frames per pass: $FRAMES"
log "sampling fps: $FPS"
[[ -n "$WIDTH" && -n "$HEIGHT" ]] && log "expected resolution: ${WIDTH}x${HEIGHT}"
if progress_enabled; then
  log "progress monitor: enabled, external 1 Hz JSONL line-count polling"
else
  log "progress monitor: disabled"
fi

[[ -f "$SUMMARY_SCRIPT" ]] || fail "missing summary script: $SUMMARY_SCRIPT"

if [[ "$SKIP_BUILD" != "1" ]]; then
  log "building core stack"
  cmake --build "$BUILD_DIR" -j"$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)"
else
  log "SKIP_BUILD=1; skipping build"
fi

if [[ "$SKIP_CTEST" != "1" ]]; then
  log "running CTest"
  ctest --test-dir "$BUILD_DIR" --output-on-failure
else
  log "SKIP_CTEST=1; skipping CTest"
fi

[[ -x "$APP_PATH" ]] || fail "missing executable: $APP_PATH"

log "checking Python AirSim package and RPC reachability"
python3 - <<PY
import airsim
client = airsim.MultirotorClient(ip="$HOST", port=int("$RPC_PORT"))
client.confirmConnection()
client.getMultirotorState(vehicle_name="$VEHICLE_NAME")
print("AirSim RPC OK")
PY

write_config() {
  local name="$1"
  local annotator="$2"
  local out_dir="$OUTPUT_ROOT/$name"
  local config_path="$out_dir/core_stack_${name}.yaml"
  mkdir -p "$out_dir"

  cat > "$config_path" <<EOF
frame_source: airsim
bridge_mode: stream_binary
bridge_transport: pipe
bridge_command: python3 simulation/airsim-stream-frames-binary.py --host $HOST --rpc-port $RPC_PORT --vehicle-name $VEHICLE_NAME --camera-name $CAMERA_NAME --count $FRAMES --rate-hz $FPS
ego_provider: airsim
ego_bridge_command: python3 simulation/airsim-capture-ego.py --host $HOST --rpc-port $RPC_PORT --vehicle-name $VEHICLE_NAME --camera-name $CAMERA_NAME
detector: scripted
camera_stabilizer: null
tracker: simple_centroid
identity_resolver: appearance_only
projector: flat_ground
world_model: in_memory
frame_annotator: $annotator
annotation_output_path: $out_dir/annotations
annotation_output_fps: $FPS
pipeline_timing_enabled: true
pipeline_timing_output_path: $out_dir/pipeline_profile.jsonl
fallback_map_frame_id: map_airsim_bridge_latency_${name}
source_host: $HOST
source_rpc_port: $RPC_PORT
vehicle_name: $VEHICLE_NAME
vehicle_camera_name: $CAMERA_NAME
EOF
  echo "$config_path"
}

run_profile() {
  local name="$1"
  local annotator="$2"
  local out_dir="$OUTPUT_ROOT/$name"
  local config_path
  local profile_path="$out_dir/pipeline_profile.jsonl"
  config_path="$(write_config "$name" "$annotator")"

  log "running profile pass: $name ($annotator)"
  rm -rf "$out_dir/snapshots" "$out_dir/annotations" "$profile_path"
  start_progress_monitor "$name" "$profile_path" "$FRAMES"
  "$APP_PATH" --config "$config_path" --output-dir "$out_dir/snapshots" --max-frames "$FRAMES"
  stop_progress_monitor "$name" "$profile_path" "$FRAMES"
}

run_profile "bridge_only" "null"
[[ "$RUN_PPM" == "1" ]] && run_profile "with_ppm" "ppm_sequence"

log "summarizing latency"
python3 - "$OUTPUT_ROOT" "$FRAMES" "$FPS" "$WIDTH" "$HEIGHT" "$RUN_PPM" "$SUMMARY_SCRIPT" <<'PY'
import json
import subprocess
import sys
from pathlib import Path

root = Path(sys.argv[1])
expected_frames = int(sys.argv[2])
fps = float(sys.argv[3])
expected_width = sys.argv[4]
expected_height = sys.argv[5]
run_ppm = sys.argv[6] == "1"
summary_script = Path(sys.argv[7])

GREEN = "\033[1;32m"
YELLOW = "\033[1;33m"
RED = "\033[1;31m"
RESET = "\033[0m"

FPS_30_MS = 1000.0 / 30.0
FPS_15_MS = 1000.0 / 15.0
FPS_10_MS = 1000.0 / 10.0
FPS_5_MS = 1000.0 / 5.0


def color_for_absolute_latency_ms(latency_ms):
    if latency_ms <= FPS_30_MS:
        return GREEN, "GREEN", ">=30 FPS capacity"
    if latency_ms <= FPS_15_MS:
        return YELLOW, "YELLOW", ">=15 FPS capacity"
    return RED, "RED", "<15 FPS capacity"


def color_for_requested_budget(latency_ms, period_ms):
    ratio = latency_ms / period_ms
    if ratio < 0.50:
        return GREEN, "GREEN", ratio
    if ratio < 0.80:
        return YELLOW, "YELLOW", ratio
    return RED, "RED", ratio


def color_text(text, color):
    return f"{color}{text}{RESET}"


def percentile(values, p):
    values = sorted(values)
    index = round((len(values) - 1) * p)
    return values[index]


def load_rows(path):
    rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    if len(rows) != expected_frames:
        raise SystemExit(f"expected {expected_frames} rows in {path}, got {len(rows)}")
    return rows


def implied_fps(latency_ms):
    if latency_ms <= 0:
        return float("inf")
    return 1000.0 / latency_ms


def summarize(name):
    path = root / name / "pipeline_profile.jsonl"
    if not path.exists():
        raise SystemExit(f"missing profile JSONL: {path}")
    rows = load_rows(path)
    stages = {}
    for row in rows:
        for stage, value in row["stages"].items():
            stages.setdefault(stage, []).append(value)

    totals = [row["total_us"] for row in rows]
    bridge = stages.get("frame_source.next_frame")
    if not bridge:
        raise SystemExit(f"{name}: missing frame_source.next_frame")

    period_ms = 1000.0 / fps
    bridge_p50_ms = percentile(bridge, 0.50) / 1000.0
    bridge_p95_ms = percentile(bridge, 0.95) / 1000.0
    bridge_p99_ms = percentile(bridge, 0.99) / 1000.0
    abs_color, abs_label, capacity_label = color_for_absolute_latency_ms(bridge_p95_ms)
    budget_color, budget_label, budget_ratio = color_for_requested_budget(bridge_p95_ms, period_ms)

    print()
    print(f"=== {name} ===")
    print(f"requested FPS: {fps:g}  frame period ms: {period_ms:.3f}")
    print("bridge/capture/read latency proxy: frame_source.next_frame")
    print(f"  mean_ms: {sum(bridge) / len(bridge) / 1000.0:.3f}")
    print(f"  p50_ms:  {bridge_p50_ms:.3f}  (~{implied_fps(bridge_p50_ms):.1f} FPS capacity)")
    print(
        f"  p95_ms:  {color_text(f'{bridge_p95_ms:.3f}', abs_color)} "
        f"({color_text(abs_label, abs_color)}, {capacity_label}, ~{implied_fps(bridge_p95_ms):.1f} FPS capacity)"
    )
    print(f"  p99_ms:  {bridge_p99_ms:.3f}  (~{implied_fps(bridge_p99_ms):.1f} FPS capacity)")
    print(f"  max_ms:  {max(bridge) / 1000.0:.3f}")
    print(
        f"  requested-FPS budget: {color_text(budget_label, budget_color)} "
        f"({budget_ratio * 100.0:.1f}% of {period_ms:.3f} ms frame period)"
    )
    print("total timed runner stages")
    print(f"  mean_ms: {sum(totals) / len(totals) / 1000.0:.3f}")
    print(f"  p95_ms:  {percentile(totals, 0.95) / 1000.0:.3f}")
    print(f"  p99_ms:  {percentile(totals, 0.99) / 1000.0:.3f}")
    print(f"  max_ms:  {max(totals) / 1000.0:.3f}")
    print(f"bridge share of timed total mean: {(sum(bridge) / max(1, sum(totals))) * 100.0:.1f}%")

    print("all stages:")
    subprocess.run(
        [sys.executable, str(summary_script), str(path)],
        check=True,
    )
    return stages


bridge_stages = summarize("bridge_only")
if run_ppm:
    summarize("with_ppm")

if expected_width and expected_height:
    w = int(expected_width)
    h = int(expected_height)
    mib = w * h * 3 / (1024 * 1024)
    print()
    print("=== expected raw RGB throughput ===")
    print(f"resolution: {w}x{h}")
    print(f"bytes/frame: {w * h * 3:,}")
    print(f"MiB/frame: {mib:.3f}")
    print(f"MiB/s at {fps:g} FPS: {mib * fps:.3f}")

period_ms = 1000.0 / fps
bridge_p95_ms = percentile(bridge_stages["frame_source.next_frame"], 0.95) / 1000.0
bridge_p99_ms = percentile(bridge_stages["frame_source.next_frame"], 0.99) / 1000.0
abs_color, abs_label, capacity_label = color_for_absolute_latency_ms(bridge_p95_ms)
budget_color, budget_label, budget_ratio = color_for_requested_budget(bridge_p95_ms, period_ms)
print()
print("=== absolute latency thresholds, based on p95 ===")
print(f"GREEN:  p95 <= {FPS_30_MS:.3f} ms  (30 FPS bridge/capture/read capacity)")
print(f"YELLOW: p95 <= {FPS_15_MS:.3f} ms  (15 FPS bridge/capture/read capacity)")
print(f"RED:    p95  > {FPS_15_MS:.3f} ms  (below 15 FPS bridge/capture/read capacity)")
print(f"Reference: 10 FPS budget = {FPS_10_MS:.3f} ms, 5 FPS budget = {FPS_5_MS:.3f} ms")

print()
print("=== requested-FPS budget thresholds ===")
print(f"GREEN:  p95 < 50% of requested frame period  (< {period_ms * 0.50:.3f} ms at {fps:g} FPS)")
print(f"YELLOW: p95 < 80% of requested frame period  (< {period_ms * 0.80:.3f} ms at {fps:g} FPS)")
print(f"RED:    p95 >= 80% of requested frame period (>= {period_ms * 0.80:.3f} ms at {fps:g} FPS)")
print(f"HARD BOTTLENECK: p95 >= 100% of requested frame period (>= {period_ms:.3f} ms at {fps:g} FPS)")

print()
print("=== decision hint ===")
print(
    f"bridge_only frame_source.next_frame p95_ms: {color_text(f'{bridge_p95_ms:.3f}', abs_color)} "
    f"({color_text(abs_label, abs_color)}, {capacity_label}, ~{implied_fps(bridge_p95_ms):.1f} FPS capacity)"
)
print(f"bridge_only frame_source.next_frame p99_ms: {bridge_p99_ms:.3f} (~{implied_fps(bridge_p99_ms):.1f} FPS capacity)")
print(
    f"requested-FPS budget: {color_text(budget_label, budget_color)} "
    f"({budget_ratio * 100.0:.1f}% of {period_ms:.3f} ms frame period)"
)
if bridge_p95_ms <= FPS_30_MS:
    print(color_text("OK: bridge p95 supports roughly 30 FPS capture/read capacity.", GREEN))
elif bridge_p95_ms <= FPS_15_MS:
    print(color_text("WATCH: bridge p95 supports roughly 15 FPS, but not 30 FPS. Good enough for 5-10 FPS validation; retest before higher-rate autonomy.", YELLOW))
else:
    print(color_text("RISK: bridge p95 is below 15 FPS capacity. Shared memory, lower resolution, lower FPS, or bridge optimization may be justified.", RED))
print()
print(f"artifacts: {root}")
PY

log "done"
