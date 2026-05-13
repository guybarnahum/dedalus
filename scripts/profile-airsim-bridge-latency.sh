#!/usr/bin/env bash
set -euo pipefail

# One-command AirSim latency profiler.
#
# AirSim must already be running. This script generates temporary core-stack
# configs and profiles one or both live AirSim ingestion paths:
#   separate_ego: stream_binary RGB + ego_provider=airsim
#   frame_ego:    stream_binary_ego RGB+ego sidecar + ego_provider=frame_hint
#
# Capacity mode is the default. It sets bridge --rate-hz 0 so
# frame_source.next_frame measures capture/read capacity instead of intentional
# frame pacing sleep. Use --paced to profile requested-FPS pacing behavior.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$ROOT_DIR/build-staging"
APP_PATH="$BUILD_DIR/apps/dedalus_replay_recording"
SUMMARY_SCRIPT="$ROOT_DIR/scripts/summarize-pipeline-profile.py"

FRAMES=300
FPS=5
WIDTH=""
HEIGHT=""
HOST="127.0.0.1"
RPC_PORT="41451"
VEHICLE_NAME="PX4"
CAMERA_NAME="front_center"
OUTPUT_ROOT="$ROOT_DIR/out/airsim_bridge_latency_$(date +%Y%m%d_%H%M%S)"
MODE="both"
CAPACITY=1
SKIP_BUILD=0
SKIP_CTEST=0
PROGRESS="auto"

usage() {
  cat <<'EOF'
Usage:
  ./scripts/profile-airsim-bridge-latency.sh [options]

Common examples:
  ./scripts/profile-airsim-bridge-latency.sh --frames 300 --width 1280 --height 720 --skip-build --skip-ctest
  ./scripts/profile-airsim-bridge-latency.sh --mode frame-ego --frames 300 --width 1280 --height 720
  ./scripts/profile-airsim-bridge-latency.sh --paced --fps 5 --frames 100

Options:
  --mode MODE             separate-ego, frame-ego, or both. Default: both
  --capacity              Disable bridge pacing with --rate-hz 0. Default
  --paced                 Use --rate-hz FPS. Useful for mission-like pacing tests
  --frames N              Number of frames per profiling pass. Default: 300
  --fps FPS               Requested FPS / paced bridge FPS. Default: 5
  --width W               Expected AirSim frame width. Optional throughput metadata
  --height H              Expected AirSim frame height. Optional throughput metadata
  --host HOST             AirSim RPC host. Default: 127.0.0.1
  --rpc-port PORT         AirSim RPC port. Default: 41451
  --vehicle-name NAME     AirSim vehicle name. Default: PX4
  --camera-name NAME      AirSim camera name. Default: front_center
  --output-root PATH      Output root. Default: out/airsim_bridge_latency_<timestamp>
  --skip-build            Skip cmake build
  --skip-ctest            Skip CTest
  --progress              Force dedalus_replay_recording progress on
  --no-progress           Disable dedalus_replay_recording progress
  -h, --help              Show this help

Metrics:
  frame_source.next_frame is the capture/bridge/read bucket.
  ego_provider.estimate shows whether ego telemetry is still a hot-path RPC.

Absolute p95 latency capacity thresholds:
  GREEN  p95 <= 33.3 ms  approximately 30 FPS capable
  YELLOW p95 <= 66.7 ms  approximately 15 FPS capable
  RED    p95  > 66.7 ms  below 15 FPS bridge/capture/read capacity

Why this is .sh and not .py:
  The shell script orchestrates builds, CTest, temporary configs, app runs, and
  progress. scripts/summarize-pipeline-profile.py is the internal Python
  formatter invoked by this script for consistent timing summaries.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode) MODE="$2"; shift 2 ;;
    --capacity) CAPACITY=1; shift ;;
    --paced) CAPACITY=0; shift ;;
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

case "$MODE" in
  separate-ego|separate_ego) MODE="separate_ego" ;;
  frame-ego|frame_ego) MODE="frame_ego" ;;
  both) MODE="both" ;;
  *) fail "--mode must be separate-ego, frame-ego, or both; got: $MODE" ;;
esac

positive_int "$FRAMES" || fail "--frames must be a positive integer, got: $FRAMES"
positive_number "$FPS" || fail "--fps must be a positive number, got: $FPS"

cd "$ROOT_DIR"
OUTPUT_ROOT="$(mkdir -p "$OUTPUT_ROOT" && cd "$OUTPUT_ROOT" && pwd)"
[[ -f "$SUMMARY_SCRIPT" ]] || fail "missing summary script: $SUMMARY_SCRIPT"

RATE_HZ="$FPS"
PROFILE_KIND="paced"
if [[ "$CAPACITY" == "1" ]]; then
  RATE_HZ="0"
  PROFILE_KIND="capacity"
fi

progress_arg=()
case "$PROGRESS" in
  on) progress_arg=(--progress) ;;
  off) progress_arg=(--no-progress) ;;
  auto) progress_arg=() ;;
  *) fail "invalid progress mode: $PROGRESS" ;;
esac

log "repo root: $ROOT_DIR"
log "output root: $OUTPUT_ROOT"
log "mode: $MODE"
log "profile kind: $PROFILE_KIND (bridge --rate-hz $RATE_HZ)"
log "frames per pass: $FRAMES"
log "requested FPS reference: $FPS"
[[ -n "$WIDTH" && -n "$HEIGHT" ]] && log "expected resolution: ${WIDTH}x${HEIGHT}"

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
  local pass_name="$1"
  local out_dir="$OUTPUT_ROOT/$pass_name"
  local config_path="$out_dir/core_stack_${pass_name}.yaml"
  mkdir -p "$out_dir"

  local bridge_mode="stream_binary"
  local ego_provider="airsim"
  local bridge_extra=""
  local ego_bridge_line="ego_bridge_command: python3 simulation/airsim-capture-ego.py --host $HOST --rpc-port $RPC_PORT --vehicle-name $VEHICLE_NAME --camera-name $CAMERA_NAME"

  if [[ "$pass_name" == "frame_ego" ]]; then
    bridge_mode="stream_binary_ego"
    ego_provider="frame_hint"
    bridge_extra=" --include-ego"
    ego_bridge_line="# ego_bridge_command intentionally omitted: frame_hint uses FramePacket::ego_hint"
  fi

  cat > "$config_path" <<EOF
frame_source: airsim
bridge_mode: $bridge_mode
bridge_transport: pipe
bridge_command: python3 simulation/airsim-stream-frames-binary.py --host $HOST --rpc-port $RPC_PORT --vehicle-name $VEHICLE_NAME --camera-name $CAMERA_NAME --count $FRAMES --rate-hz $RATE_HZ$bridge_extra
ego_provider: $ego_provider
$ego_bridge_line
detector: scripted
camera_stabilizer: null
tracker: simple_centroid
identity_resolver: appearance_only
projector: flat_ground
world_model: in_memory
frame_annotator: null
pipeline_timing_enabled: true
pipeline_timing_output_path: $out_dir/pipeline_profile.jsonl
fallback_map_frame_id: map_airsim_bridge_latency_${pass_name}
source_host: $HOST
source_rpc_port: $RPC_PORT
vehicle_name: $VEHICLE_NAME
vehicle_camera_name: $CAMERA_NAME
EOF
  echo "$config_path"
}

run_profile() {
  local pass_name="$1"
  local out_dir="$OUTPUT_ROOT/$pass_name"
  local profile_path="$out_dir/pipeline_profile.jsonl"
  local config_path
  config_path="$(write_config "$pass_name")"

  log "running profile pass: $pass_name"
  rm -rf "$out_dir/snapshots" "$profile_path"
  "$APP_PATH" \
    --config "$config_path" \
    --output-dir "$out_dir/snapshots" \
    --max-frames "$FRAMES" \
    "${progress_arg[@]}"
}

passes=()
if [[ "$MODE" == "both" || "$MODE" == "separate_ego" ]]; then
  passes+=("separate_ego")
fi
if [[ "$MODE" == "both" || "$MODE" == "frame_ego" ]]; then
  passes+=("frame_ego")
fi

for pass_name in "${passes[@]}"; do
  run_profile "$pass_name"
done

log "summarizing latency"
python3 - "$OUTPUT_ROOT" "$FRAMES" "$FPS" "$WIDTH" "$HEIGHT" "$PROFILE_KIND" "$SUMMARY_SCRIPT" "${passes[@]}" <<'PY'
import json
import subprocess
import sys
from pathlib import Path

root = Path(sys.argv[1])
expected_frames = int(sys.argv[2])
fps = float(sys.argv[3])
expected_width = sys.argv[4]
expected_height = sys.argv[5]
profile_kind = sys.argv[6]
summary_script = Path(sys.argv[7])
passes = sys.argv[8:]

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


def color_text(text, color):
    return f"{color}{text}{RESET}"


def percentile(values, p):
    ordered = sorted(values)
    return ordered[round((len(ordered) - 1) * p)]


def implied_fps(latency_ms):
    if latency_ms <= 0:
        return float("inf")
    return 1000.0 / latency_ms


def load_rows(path):
    rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    if len(rows) != expected_frames:
        raise SystemExit(f"expected {expected_frames} rows in {path}, got {len(rows)}")
    return rows


def stage_values(rows, stage):
    values = []
    for row in rows:
        stages = row.get("stages", {})
        if stage in stages:
            values.append(stages[stage])
    if not values:
        raise SystemExit(f"missing stage {stage}")
    return values


def summarize_pass(pass_name):
    path = root / pass_name / "pipeline_profile.jsonl"
    if not path.exists():
        raise SystemExit(f"missing profile JSONL: {path}")
    rows = load_rows(path)
    bridge = stage_values(rows, "frame_source.next_frame")
    ego = stage_values(rows, "ego_provider.estimate")
    total = [row["total_us"] for row in rows]

    bridge_p95_ms = percentile(bridge, 0.95) / 1000.0
    bridge_p99_ms = percentile(bridge, 0.99) / 1000.0
    ego_p95_ms = percentile(ego, 0.95) / 1000.0
    total_p95_ms = percentile(total, 0.95) / 1000.0
    color, label, capacity = color_for_absolute_latency_ms(bridge_p95_ms)

    print()
    print(f"=== {pass_name} ===")
    print(f"profile kind: {profile_kind}")
    print(f"bridge p95: {color_text(f'{bridge_p95_ms:.3f} ms', color)} ({color_text(label, color)}, {capacity}, ~{implied_fps(bridge_p95_ms):.1f} FPS capacity)")
    print(f"bridge p99: {bridge_p99_ms:.3f} ms (~{implied_fps(bridge_p99_ms):.1f} FPS capacity)")
    print(f"ego_provider p95: {ego_p95_ms:.3f} ms")
    print(f"total runner p95: {total_p95_ms:.3f} ms (~{implied_fps(total_p95_ms):.1f} FPS capacity)")
    print("stage summary:")
    subprocess.run([sys.executable, str(summary_script), str(path)], check=True)
    return {
        "pass": pass_name,
        "bridge_p95_ms": bridge_p95_ms,
        "bridge_p99_ms": bridge_p99_ms,
        "ego_p95_ms": ego_p95_ms,
        "total_p95_ms": total_p95_ms,
    }

results = [summarize_pass(pass_name) for pass_name in passes]

if expected_width and expected_height:
    w = int(expected_width)
    h = int(expected_height)
    mib = w * h * 3 / (1024 * 1024)
    print()
    print("=== expected raw RGB throughput ===")
    print(f"resolution: {w}x{h}")
    print(f"bytes/frame: {w * h * 3:,}")
    print(f"MiB/frame: {mib:.3f}")
    print(f"MiB/s at {fps:g} FPS: {mib * fps:.3f} for paced mode reference")

print()
print("=== absolute latency thresholds, based on frame_source.next_frame p95 ===")
print(f"GREEN:  p95 <= {FPS_30_MS:.3f} ms  (30 FPS bridge/capture/read capacity)")
print(f"YELLOW: p95 <= {FPS_15_MS:.3f} ms  (15 FPS bridge/capture/read capacity)")
print(f"RED:    p95  > {FPS_15_MS:.3f} ms  (below 15 FPS bridge/capture/read capacity)")
print(f"Reference: 10 FPS budget = {FPS_10_MS:.3f} ms, 5 FPS budget = {FPS_5_MS:.3f} ms")

if len(results) == 2:
    by_name = {result["pass"]: result for result in results}
    if "separate_ego" in by_name and "frame_ego" in by_name:
        sep = by_name["separate_ego"]
        frm = by_name["frame_ego"]
        ego_saved = sep["ego_p95_ms"] - frm["ego_p95_ms"]
        total_saved = sep["total_p95_ms"] - frm["total_p95_ms"]
        print()
        print("=== comparison ===")
        print(f"ego_provider p95 reduction: {ego_saved:.3f} ms")
        print(f"total runner p95 reduction: {total_saved:.3f} ms")
        print(f"separate_ego total p95 capacity: ~{implied_fps(sep['total_p95_ms']):.1f} FPS")
        print(f"frame_ego total p95 capacity:    ~{implied_fps(frm['total_p95_ms']):.1f} FPS")

print()
print(f"artifacts: {root}")
PY

log "done"
