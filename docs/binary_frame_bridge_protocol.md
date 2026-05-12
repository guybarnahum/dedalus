# Binary Frame Bridge Protocol

`bridge_mode: stream_binary` is the efficient frame-stream bridge mode for simulation/camera providers.

It avoids the overhead of:

```text
JSON image payloads
base64 encoding
PPM parsing
```

The bridge process writes one binary frame after another to stdout. The C++ `AirSimFrameSource` reads a fixed-size header followed by raw RGB bytes through `BridgeTransport::read_stream_bytes()`.

## Runtime Config

```yaml
frame_source: airsim
bridge_transport: pipe
bridge_mode: stream_binary
bridge_command: python3 simulation/airsim-stream-frames-binary.py --count 0 --rate-hz 5
ego_provider: airsim
ego_bridge_command: python3 simulation/airsim-capture-ego.py
```

Use:

```bash
./build-validation/apps/dedalus_replay_recording \
  --config config/core_stack_airsim_binary_rgb_ego.yaml \
  --output-dir out/airsim_binary_snapshots \
  --max-frames 5
```

## Frame Header

All integer fields are little-endian.

```text
magic[8]        = DEDFRM1\0
header_size     uint32, currently 56
version         uint32, currently 1
sequence        uint64
timestamp_ns    int64
width           uint32
height          uint32
channels        uint32, currently 3
pixel_format    uint32, currently 1 for RGB8
payload_size    uint32
reserved        uint32
payload         raw RGB bytes
```

## Validation Rules

The C++ parser validates:

```text
magic == DEDFRM1\0
header_size == 56
version == 1
width > 0
height > 0
channels == 3
pixel_format == RGB8
payload_size == width * height * channels
```

## CI Coverage

CI uses:

```text
config/core_stack_airsim_binary_ci.yaml
tests/fixtures/airsim_binary_bridge_ci_fake.py
tests/unit/test_airsim_provider_boundary.cpp
```

The fake bridge emits two binary frames and exits. The boundary test verifies that both frames are read from one persistent process, metadata is preserved, and end-of-stream is detected cleanly.

## Relationship to Other Modes

```text
one_shot_ppm
  Simple debug path, one process per frame.

stream_jsonl
  Persistent and easy to inspect, but image bytes are base64 encoded.

stream_binary
  Persistent and efficient. Preferred for higher-rate simulation/camera ingestion.
```
