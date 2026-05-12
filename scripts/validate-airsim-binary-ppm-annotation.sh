#!/usr/bin/env bash
set -euo pipefail

# validate-airsim-binary-ppm-annotation.sh
#
# Validates Milestone 2.8 ppm_sequence annotation on live AirSim binary RGB capture.
#
# It:
#   1. Configures/builds Dedalus.
#   2. Runs CTest.
#   3. Checks that the Python AirSim package is importable.
#   4. Checks that AirSim RPC is reachable.
#   5. Writes a finite AirSim binary RGB + ppm_sequence config.
#   6. Runs dedalus_replay_recording for N frames.
#   7. Verifies snapshot + annotation artifacts.
#   8. Validates annotation rows align with snapshot rows.
#   9. Reports annotation frame timestamps vs snapshot/ego timestamps.
#  10. Enforces a configurable soft timestamp-delta threshold.
#
# Assumptions:
#   - Colosseum/AirSim is already running.
#   - PX4 vehicle is present if vehicle_name=PX4.
#   - AirSim RPC is reachable, default 127.0.0.1:41451.
#
# Useful env overrides:
#   BUILD_DIR=...
#   OUTPUT_DIR=...
#   CONFIG_PATH=...
#   AIRSIM_HOST=127.0.0.1
#   AIRSIM_RPC_PORT=41451
#   AIRSIM_VEHICLE_NAME=PX4
#   AIRSIM_CAMERA_NAME=front_center
#   AIRSIM_FRAME_COUNT=10
#   AIRSIM_RATE_HZ=5
#   TIMESTAMP_SOFT_THRESHOLD_MS=500
#   SKIP_BUILD=1
#   SKIP_CTEST=1

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

BUILD_DIR="${BUILD_DIR:-$ROOT_DIR/build-staging}"
OUTPUT_DIR="${OUTPUT_DIR:-$ROOT_DIR/out/airsim_binary_ppm_validation}"
CONFIG_PATH="${CONFIG_PATH:-$ROOT_DIR/config/core_stack_airsim_binary_ppm_validation.yaml}"

AIRSIM_HOST="${AIRSIM_HOST:-127.0.0.1}"
AIRSIM_RPC_PORT="${AIRSIM_RPC_PORT:-41451}"
AIRSIM_VEHICLE_NAME="${AIRSIM_VEHICLE_NAME:-PX4}"
AIRSIM_CAMERA_NAME="${AIRSIM_CAMERA_NAME:-front_center}"
AIRSIM_FRAME_COUNT="${AIRSIM_FRAME_COUNT:-10}"
AIRSIM_RATE_HZ="${AIRSIM_RATE_HZ:-5}"
TIMESTAMP_SOFT_THRESHOLD_MS="${TIMESTAMP_SOFT_THRESHOLD_MS:-500}"

APP_PATH="$BUILD_DIR/apps/dedalus_replay_recording"

log() {
  printf '\033[1;36m[validate-airsim-ppm]\033[0m %s\n' "$*"
}

warn() {
  printf '\033[1;33m[validate-airsim-ppm] WARN:\033[0m %s\n' "$*" >&2
}

fail() {
  printf '\033[1;31m[validate-airsim-ppm] ERROR:\033[0m %s\n' "$*" >&2
  exit 1
}

require_file() {
  local path="$1"
  [[ -f "$path" ]] || fail "missing expected file: $path"
}

require_grep() {
  local needle="$1"
  local path="$2"
  grep -q "$needle" "$path" || fail "expected '$needle' in $path"
}

positive_int() {
  local value="$1"
  [[ "$value" =~ ^[0-9]+$ ]] && [[ "$value" -gt 0 ]]
}

positive_number() {
  local value="$1"
  python3 - "$value" <<'PY'
import sys
try:
    value = float(sys.argv[1])
except ValueError:
    raise SystemExit(1)
raise SystemExit(0 if value > 0 else 1)
PY
}

log "repo root: $ROOT_DIR"
cd "$ROOT_DIR"

if ! positive_int "$AIRSIM_FRAME_COUNT"; then
  fail "AIRSIM_FRAME_COUNT must be a positive integer; got '$AIRSIM_FRAME_COUNT'"
fi

if ! positive_number "$TIMESTAMP_SOFT_THRESHOLD_MS"; then
  fail "TIMESTAMP_SOFT_THRESHOLD_MS must be a positive number; got '$TIMESTAMP_SOFT_THRESHOLD_MS'"
fi

if command -v git >/dev/null 2>&1; then
  log "git HEAD: $(git rev-parse HEAD 2>/dev/null || echo unknown)"
fi

if [[ "${SKIP_BUILD:-0}" != "1" ]]; then
  log "configuring CMake: $BUILD_DIR"
  cmake -S "$ROOT_DIR" -B "$BUILD_DIR" \
    -DDEDALUS_BUILD_APPS=ON \
    -DDEDALUS_BUILD_TESTS=ON

  log "building"
  cmake --build "$BUILD_DIR" -j"$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)"
else
  log "SKIP_BUILD=1; skipping CMake configure/build"
fi

if [[ "${SKIP_CTEST:-0}" != "1" ]]; then
  log "running CTest"
  ctest --test-dir "$BUILD_DIR" --output-on-failure
else
  log "SKIP_CTEST=1; skipping CTest"
fi

require_file "$APP_PATH"

log "checking Python AirSim package"
python3 - <<'PY'
try:
    import airsim  # noqa: F401
except Exception as exc:
    raise SystemExit(f"failed to import airsim Python package: {exc}")
print("AirSim Python package import OK")
PY

log "checking AirSim RPC reachability: $AIRSIM_HOST:$AIRSIM_RPC_PORT vehicle=$AIRSIM_VEHICLE_NAME camera=$AIRSIM_CAMERA_NAME"
python3 - <<PY
import sys
import airsim

host = "$AIRSIM_HOST"
port = int("$AIRSIM_RPC_PORT")
vehicle = "$AIRSIM_VEHICLE_NAME"

try:
    client = airsim.MultirotorClient(ip=host, port=port)
    client.confirmConnection()
    # This also verifies vehicle lookup enough for the ego bridge path.
    client.getMultirotorState(vehicle_name=vehicle)
except Exception as exc:
    raise SystemExit(f"AirSim RPC check failed: {exc}")

print("AirSim RPC check OK")
PY

log "writing validation config: $CONFIG_PATH"
cat > "$CONFIG_PATH" <<EOF
frame_source: airsim
bridge_mode: stream_binary
bridge_transport: pipe
bridge_command: python3 simulation/airsim-stream-frames-binary.py --host $AIRSIM_HOST --rpc-port $AIRSIM_RPC_PORT --vehicle-name $AIRSIM_VEHICLE_NAME --camera-name $AIRSIM_CAMERA_NAME --count $AIRSIM_FRAME_COUNT --rate-hz $AIRSIM_RATE_HZ
ego_provider: airsim
ego_bridge_command: python3 simulation/airsim-capture-ego.py --host $AIRSIM_HOST --rpc-port $AIRSIM_RPC_PORT --vehicle-name $AIRSIM_VEHICLE_NAME --camera-name $AIRSIM_CAMERA_NAME
detector: scripted
camera_stabilizer: null
tracker: simple_centroid
identity_resolver: appearance_only
projector: flat_ground
world_model: in_memory
frame_annotator: ppm_sequence
annotation_output_path: out/airsim_binary_ppm_validation/annotations
annotation_output_fps: $AIRSIM_RATE_HZ
fallback_map_frame_id: map_airsim_binary_ppm_validation_0001
source_host: $AIRSIM_HOST
source_rpc_port: $AIRSIM_RPC_PORT
vehicle_name: $AIRSIM_VEHICLE_NAME
vehicle_camera_name: $AIRSIM_CAMERA_NAME
EOF

log "cleaning output: $OUTPUT_DIR"
rm -rf "$OUTPUT_DIR"
mkdir -p "$OUTPUT_DIR"

log "running replay with AirSim binary RGB + ppm_sequence"
"$APP_PATH" \
  --config "$CONFIG_PATH" \
  --output-dir "$OUTPUT_DIR/snapshots" \
  --max-frames "$AIRSIM_FRAME_COUNT"

ANNOTATION_DIR="$OUTPUT_DIR/annotations"
ANNOTATION_MANIFEST="$ANNOTATION_DIR/manifest.txt"
SNAPSHOT_DIR="$OUTPUT_DIR/snapshots"
SNAPSHOT_MANIFEST="$SNAPSHOT_DIR/snapshot_manifest.txt"

log "checking artifact manifests"
require_file "$ANNOTATION_MANIFEST"
require_file "$SNAPSHOT_MANIFEST"

ANNOTATION_COUNT="$(awk -F',' 'NR > 1 && NF >= 5 {count++} END {print count+0}' "$ANNOTATION_MANIFEST")"
SNAPSHOT_COUNT="$(awk 'NR > 1 && NF >= 4 {count++} END {print count+0}' "$SNAPSHOT_MANIFEST")"

log "annotation frame count: $ANNOTATION_COUNT"
log "snapshot count:         $SNAPSHOT_COUNT"

[[ "$ANNOTATION_COUNT" -eq "$AIRSIM_FRAME_COUNT" ]] || \
  fail "expected $AIRSIM_FRAME_COUNT annotation rows, got $ANNOTATION_COUNT"

[[ "$SNAPSHOT_COUNT" -eq "$AIRSIM_FRAME_COUNT" ]] || \
  fail "expected $AIRSIM_FRAME_COUNT snapshot rows, got $SNAPSHOT_COUNT"

log "checking first/last expected artifacts"
require_file "$ANNOTATION_DIR/frame_000001.ppm"
require_file "$SNAPSHOT_DIR/snapshot_0001.json"

LAST_FRAME_NAME="$(printf 'frame_%06d.ppm' "$AIRSIM_FRAME_COUNT")"
LAST_SNAPSHOT_NAME="$(printf 'snapshot_%04d.json' "$AIRSIM_FRAME_COUNT")"

require_file "$ANNOTATION_DIR/$LAST_FRAME_NAME"
require_file "$SNAPSHOT_DIR/$LAST_SNAPSHOT_NAME"

log "checking manifest content"
require_grep "frame_000001.ppm" "$ANNOTATION_MANIFEST"
require_grep "$LAST_FRAME_NAME" "$ANNOTATION_MANIFEST"
require_grep "map_airsim_binary_ppm_validation_0001" "$SNAPSHOT_MANIFEST"

log "checking PPM files"
python3 - <<PY
from pathlib import Path

annotation_dir = Path("$ANNOTATION_DIR")
count = int("$AIRSIM_FRAME_COUNT")

for idx in (1, count):
    p = annotation_dir / f"frame_{idx:06d}.ppm"
    data = p.read_bytes()
    parts = data.split(b"\\n", 3)
    if len(parts) < 4:
        raise SystemExit(f"{p}: invalid PPM header")
    magic = parts[0].decode()
    dims = parts[1].decode()
    maxval = parts[2].decode()
    payload = parts[3]
    width, height = map(int, dims.split())
    expected_payload = width * height * 3

    print(f"  {p.name}: {magic} {width}x{height} max={maxval} payload={len(payload)} expected={expected_payload}")

    if magic != "P6":
        raise SystemExit(f"{p}: invalid PPM magic {magic}")
    if maxval != "255":
        raise SystemExit(f"{p}: invalid maxval {maxval}")
    if len(payload) != expected_payload:
        raise SystemExit(f"{p}: payload size mismatch")
PY

log "checking timestamp relationship"
python3 - <<PY
from pathlib import Path

annotation_manifest = Path("$ANNOTATION_MANIFEST")
snapshot_manifest = Path("$SNAPSHOT_MANIFEST")
soft_threshold_ms = float("$TIMESTAMP_SOFT_THRESHOLD_MS")

annotation_rows = []
for line in annotation_manifest.read_text().splitlines():
    if not line.strip() or line.startswith("frame_index,"):
        continue
    parts = line.split(",")
    if len(parts) < 5:
        raise SystemExit(f"bad annotation manifest line: {line}")
    annotation_rows.append({
        "index": int(parts[0]),
        "frame_id": parts[1],
        "timestamp_ns": int(parts[2]),
        "path": parts[3],
        "fps": parts[4],
    })

snapshot_rows = []
for line in snapshot_manifest.read_text().splitlines():
    if not line.strip() or line.startswith("#"):
        continue
    parts = line.split()
    if len(parts) < 4:
        raise SystemExit(f"bad snapshot manifest line: {line}")
    snapshot_rows.append({
        "index": int(parts[0]),
        "path": parts[1],
        "timestamp_ns": int(parts[2]),
        "map_frame": parts[3],
    })

if len(annotation_rows) != len(snapshot_rows):
    raise SystemExit(f"row count mismatch: annotations={len(annotation_rows)} snapshots={len(snapshot_rows)}")

print("  index annotation_ts snapshot_ts delta_ms frame_id snapshot")
print("  ----- ------------- ----------- -------- -------- --------")
max_abs_delta_ns = 0
for ann, snap in zip(annotation_rows, snapshot_rows):
    if ann["index"] != snap["index"]:
        raise SystemExit(f"index mismatch: annotation={ann['index']} snapshot={snap['index']}")
    delta_ns = snap["timestamp_ns"] - ann["timestamp_ns"]
    max_abs_delta_ns = max(max_abs_delta_ns, abs(delta_ns))
    delta_ms = delta_ns / 1_000_000.0
    print(
        f"  {ann['index']:>5} "
        f"{ann['timestamp_ns']} "
        f"{snap['timestamp_ns']} "
        f"{delta_ms:>8.3f} "
        f"{ann['frame_id']} "
        f"{snap['path']}"
    )

max_abs_delta_ms = max_abs_delta_ns / 1_000_000.0
print(f"  rows: {len(annotation_rows)}")
print(f"  max_abs_delta_ms: {max_abs_delta_ms:.3f}")
print(f"  soft_threshold_ms: {soft_threshold_ms:.3f}")
print()
print("NOTE:")
print("  For AirSim live RGB + ego_provider: airsim, exact timestamp equality is not expected today.")
print("  Annotation timestamps come from the AirSim image response.")
print("  Snapshot timestamps come from the per-sample ego bridge, which uses time.time_ns().")
print("  This check validates row/index alignment and enforces a soft delta threshold.")

if max_abs_delta_ms > soft_threshold_ms:
    raise SystemExit(
        "timestamp delta too large for current validation threshold: "
        f"{max_abs_delta_ms:.3f} ms > {soft_threshold_ms:.3f} ms"
    )

print("OK: AirSim annotation rows align with snapshot rows")
PY

log "validation passed"
log "artifacts:"
find "$OUTPUT_DIR" -maxdepth 3 -type f | sort
