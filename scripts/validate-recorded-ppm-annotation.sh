#!/usr/bin/env bash
set -euo pipefail

# validate-recorded-ppm-annotation.sh
#
# Validates Milestone 2.8 ppm_sequence annotation on the recorded_frames provider.
#
# It:
#   1. Configures/builds Dedalus.
#   2. Runs CTest.
#   3. Creates a recorded_frames + ppm_sequence validation config.
#   4. Runs dedalus_replay_recording.
#   5. Verifies snapshot + annotation artifacts.
#   6. Verifies annotation and snapshot timestamps align.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$ROOT_DIR/build-staging}"
OUTPUT_DIR="${OUTPUT_DIR:-$ROOT_DIR/out/recorded_ppm_validation}"
CONFIG_PATH="${CONFIG_PATH:-$ROOT_DIR/config/core_stack_recorded_ppm_annotation_ci.yaml}"

APP_PATH="$BUILD_DIR/apps/dedalus_replay_recording"

log() {
  printf '\033[1;36m[validate-recorded-ppm]\033[0m %s\n' "$*"
}

fail() {
  printf '\033[1;31m[validate-recorded-ppm] ERROR:\033[0m %s\n' "$*" >&2
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

log "repo root: $ROOT_DIR"
cd "$ROOT_DIR"

if command -v git >/dev/null 2>&1; then
  log "git HEAD: $(git rev-parse HEAD 2>/dev/null || echo unknown)"
fi

log "configuring CMake: $BUILD_DIR"
cmake -S "$ROOT_DIR" -B "$BUILD_DIR" \
  -DDEDALUS_BUILD_APPS=ON \
  -DDEDALUS_BUILD_TESTS=ON

log "building"
cmake --build "$BUILD_DIR" -j"$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)"

log "running CTest"
ctest --test-dir "$BUILD_DIR" --output-on-failure

require_file "$APP_PATH"

log "writing validation config: $CONFIG_PATH"
cat > "$CONFIG_PATH" <<'EOF'
frame_source: recorded_frames
recorded_manifest_path: tests/fixtures/recorded_frames/manifest.txt
ego_provider: no_telemetry
detector: scripted
camera_stabilizer: null
tracker: simple_centroid
identity_resolver: appearance_only
projector: flat_ground
world_model: in_memory
frame_annotator: ppm_sequence
annotation_output_path: out/recorded_ppm_validation/annotations
annotation_output_fps: 5
fallback_map_frame_id: map_recorded_ci_0001
EOF

log "cleaning output: $OUTPUT_DIR"
rm -rf "$OUTPUT_DIR"
mkdir -p "$OUTPUT_DIR"

log "running replay with recorded_frames + ppm_sequence"
"$APP_PATH" \
  --config "$CONFIG_PATH" \
  --output-dir "$OUTPUT_DIR/snapshots" \
  --max-frames 0

ANNOTATION_FRAME="$OUTPUT_DIR/annotations/frame_000001.ppm"
ANNOTATION_MANIFEST="$OUTPUT_DIR/annotations/manifest.txt"
SNAPSHOT_JSON="$OUTPUT_DIR/snapshots/snapshot_0001.json"
SNAPSHOT_MANIFEST="$OUTPUT_DIR/snapshots/snapshot_manifest.txt"

log "checking artifacts"
require_file "$ANNOTATION_FRAME"
require_file "$ANNOTATION_MANIFEST"
require_file "$SNAPSHOT_JSON"
require_file "$SNAPSHOT_MANIFEST"

log "checking annotation manifest"
require_grep "recorded_frame_0001" "$ANNOTATION_MANIFEST"
require_grep "123456789" "$ANNOTATION_MANIFEST"
require_grep "frame_000001.ppm" "$ANNOTATION_MANIFEST"

log "checking snapshot manifest"
require_grep "123456789" "$SNAPSHOT_MANIFEST"
require_grep "map_recorded_ci_0001" "$SNAPSHOT_MANIFEST"

ANNOTATION_TS="$(awk -F',' 'NR==2 {print $3}' "$ANNOTATION_MANIFEST")"
SNAPSHOT_TS="$(awk 'NR==2 {print $3}' "$SNAPSHOT_MANIFEST")"

log "annotation timestamp: $ANNOTATION_TS"
log "snapshot timestamp:   $SNAPSHOT_TS"

[[ "$ANNOTATION_TS" == "$SNAPSHOT_TS" ]] || \
  fail "timestamp mismatch: annotation=$ANNOTATION_TS snapshot=$SNAPSHOT_TS"

log "checking PPM header"
PPM_MAGIC="$(head -c 2 "$ANNOTATION_FRAME")"
[[ "$PPM_MAGIC" == "P6" ]] || fail "expected P6 PPM magic, got: $PPM_MAGIC"

log "PPM metadata"
python3 - <<PY
from pathlib import Path

p = Path("$ANNOTATION_FRAME")
data = p.read_bytes()
parts = data.split(b"\\n", 3)

if len(parts) < 4:
    raise SystemExit("invalid PPM: missing header fields")

magic = parts[0].decode()
dims = parts[1].decode()
maxval = parts[2].decode()
payload = parts[3]

width, height = map(int, dims.split())
expected_payload = width * height * 3

print(f"  magic: {magic}")
print(f"  dims: {width}x{height}")
print(f"  maxval: {maxval}")
print(f"  payload bytes: {len(payload)}")
print(f"  expected payload bytes: {expected_payload}")

if magic != "P6":
    raise SystemExit("invalid PPM magic")
if maxval != "255":
    raise SystemExit("invalid PPM maxval")
if len(payload) != expected_payload:
    raise SystemExit("payload size mismatch")
PY

log "validation passed"
log "artifacts:"
find "$OUTPUT_DIR" -maxdepth 3 -type f | sort

