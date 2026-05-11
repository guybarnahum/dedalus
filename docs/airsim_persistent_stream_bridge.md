# AirSim Persistent Stream Bridge

This document describes the current persistent AirSim RGB stream path.

## Status

`AirSimFrameSource` supports two bridge modes:

```text
one_shot_ppm   -> call one bridge command per frame
stream_jsonl   -> keep one bridge command open and read one JSON line per frame
```

The persistent path avoids spawning Python for every RGB frame. It still keeps the C++ core stack dependency-free: AirSim remains in the Python bridge process, not in the C++ build.

## Runtime Config

Use:

```text
config/core_stack_airsim_stream_rgb_ego.yaml
```

Key fields:

```yaml
frame_source: airsim
airsim_bridge_mode: stream_jsonl
airsim_bridge_command: python3 simulation/airsim-stream-frames.py --count 0 --rate-hz 5
ego_provider: airsim
airsim_ego_bridge_command: python3 simulation/airsim-capture-ego.py
```

`--count 0` means the Python bridge streams until stopped. `dedalus_replay_recording --max-frames N` controls how many frames the C++ side consumes.

## Usage

Start AirSim/Colosseum first:

```bash
cd ~/dedalus/simulation
./run.sh
```

Then run the core stack from the repo root:

```bash
cd ~/dedalus
./build-validation/apps/dedalus_core_stack \
  --config config/core_stack_airsim_stream_rgb_ego.yaml
```

For a bounded snapshot sequence:

```bash
./build-validation/apps/dedalus_replay_recording \
  --config config/core_stack_airsim_stream_rgb_ego.yaml \
  --output-dir out/airsim_stream_snapshots \
  --max-frames 5
```

## Bridge Protocol

The persistent frame bridge writes one compact JSON object per stdout line:

```json
{"frame_id":"airsim_stream_frame_0001","timestamp_ns":123,"camera_id":"front_center","ppm_b64":"..."}
```

`ppm_b64` is a base64-encoded P6 PPM image. The C++ `AirSimFrameSource` decodes the payload, parses it into an `ImageView`, and returns a `FramePacket`.

## CI Coverage

CI does not require AirSim. It uses:

```text
config/core_stack_airsim_stream_ci.yaml
tests/fixtures/airsim_stream_bridge_ci_fake.py
tests/unit/test_airsim_provider_boundary.cpp
```

The fake stream bridge emits two JSONL frames and exits cleanly. The test verifies that `AirSimFrameSource` reads both frames from one persistent process and then returns end-of-stream.

## Remaining Gaps

```text
AirSim ego bridge is still per-sample, not persistent.
AirSim depth projection is still not implemented.
AirSim ground-truth detection is still not implemented.
```
