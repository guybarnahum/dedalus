# Binary Frame Bridge Protocol

`bridge_mode: stream_binary` is the efficient frame-stream bridge mode for simulation/camera providers.

It avoids the overhead of:

```text
JSON image payloads
base64 encoding
PPM parsing
```

The bridge process writes one binary frame after another to stdout. The C++ `AirSimFrameSource` reads a fixed-size header followed by raw RGB bytes through `BridgeTransport::read_stream_bytes()`.

Milestone 2.13 adds `bridge_mode: stream_binary_ego`, which uses the same fixed header and RGB payload but appends a small UTF-8 JSON sidecar containing ego telemetry. The frame source parses the sidecar into `FramePacket::ego_hint`, allowing live AirSim configs to use `ego_provider: frame_hint` and avoid one separate AirSim ego RPC from C++ per frame.

## Runtime Config

RGB-only stream with separate ego bridge:

```yaml
frame_source: airsim
bridge_transport: pipe
bridge_mode: stream_binary
bridge_command: python3 simulation/airsim/scripts/airsim-stream-frames-binary.py --count 0 --rate-hz 5
ego_provider: airsim
ego_bridge_command: python3 simulation/airsim/scripts/airsim-capture-ego.py
```

RGB + ego sidecar stream:

```yaml
frame_source: airsim
bridge_transport: pipe
bridge_mode: stream_binary_ego
bridge_command: python3 simulation/airsim/scripts/airsim-stream-frames-binary.py --count 0 --rate-hz 5 --include-ego
ego_provider: frame_hint
```

Use:

```bash
./build-validation/apps/dedalus_replay_recording \
  --config config/ci/core_stack_airsim_binary_rgb_ego.yaml \
  --output-dir out/airsim_binary_snapshots \
  --max-frames 5
```

For the frame-attached ego path:

```bash
./build-validation/apps/dedalus_replay_recording \
  --config config/ci/core_stack_airsim_binary_rgb_ego_hint.yaml \
  --output-dir out/airsim_binary_ego_hint_snapshots \
  --max-frames 5
```

## Frame Header

All integer fields are little-endian.

```text
magic[8]        = DEDFRM1\0
header_size     uint32, currently 56
version         uint32, 1 for RGB-only, 2 for RGB + ego sidecar
sequence        uint64
timestamp_ns    int64
width           uint32
height          uint32
channels        uint32, currently 3
pixel_format    uint32, currently 1 for RGB8
payload_size    uint32
sidecar_size    uint32, 0 for version 1, ego JSON byte size for version 2
payload         raw RGB bytes
sidecar         optional UTF-8 JSON bytes when version == 2
```

Version 2 sidecar JSON currently uses the same flat schema as `simulation/airsim/scripts/airsim-capture-ego.py`:

```json
{"timestamp_ns":123456789,"position":[1,2,3],"rotation_rpy":[0,0,0],"velocity":[0,0,0],"angular_velocity":[0,0,0]}
```

## Validation Rules

The C++ parser validates:

```text
magic == DEDFRM1\0
header_size == 56
version == 1 or version == 2
width > 0
height > 0
channels == 3
pixel_format == RGB8
payload_size == width * height * channels
version 1 sidecar_size == 0
```

When `sidecar_size > 0`, `AirSimFrameSource` reads the sidecar bytes after the RGB payload and populates `FramePacket::ego_hint`. `FrameHintEgoProvider` then returns that telemetry-backed ego state.

## CI Coverage

CI uses:

```text
config/ci/core_stack_airsim_binary_ci.yaml
tests/fixtures/airsim_binary_bridge_ci_fake.py
tests/unit/test_airsim_provider_boundary.cpp
```

The frame-attached ego path uses:

```text
config/ci/core_stack_airsim_binary_state_ci.yaml
tests/fixtures/airsim_binary_state_bridge_ci_fake.py
tests/unit/test_airsim_provider_boundary.cpp
```

The fake bridges emit two binary frames and exit. The boundary test verifies that both frames are read from one persistent process, metadata is preserved, frame-attached ego telemetry is parsed when present, and end-of-stream is detected cleanly.

## Relationship to Other Modes

```text
one_shot_ppm
  Simple debug path, one process per frame.

stream_jsonl
  Persistent and easy to inspect, but image bytes are base64 encoded.

stream_binary
  Persistent and efficient RGB-only stream. Uses a separate ego provider when telemetry is needed.

stream_binary_ego
  Persistent RGB stream with frame-attached ego telemetry. Preferred for live AirSim profiling when ego_provider.estimate is the bottleneck.
```
