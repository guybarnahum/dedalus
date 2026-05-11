# AirSim Live RGB Provider

This document describes the current live AirSim RGB ingestion path.

## Status

The core stack now supports a live RGB AirSim frame source through an external bridge command:

```text
frame_source: airsim
airsim_bridge_command: python3 simulation/airsim-capture-frame.py
ego_provider: no_telemetry
detector: scripted
projector: flat_ground
```

This is not a direct C++ AirSim RPC client. The C++ `AirSimFrameSource` remains dependency-free by invoking the configured bridge command, reading P6 PPM bytes from stdout, and converting them into a `FramePacket`.

## Runtime Config

Use:

```text
config/core_stack_airsim_live_rgb.yaml
```

Example:

```yaml
frame_source: airsim
airsim_bridge_command: python3 simulation/airsim-capture-frame.py
ego_provider: no_telemetry
detector: scripted
tracker: simple_centroid
identity_resolver: appearance_only
projector: flat_ground
world_model: in_memory
fallback_map_frame_id: map_airsim_live_rgb_0001

airsim_host: 127.0.0.1
airsim_rpc_port: 41451
airsim_vehicle_name: PX4
airsim_camera_name: front_center
```

## Usage

Start Colosseum/AirSim first:

```bash
cd ~/dedalus/simulation
./run.sh
```

Then run the core stack from the repo root:

```bash
cd ~/dedalus
./build-validation/apps/dedalus_core_stack \
  --config config/core_stack_airsim_live_rgb.yaml
```

For multiple snapshots:

```bash
./build-validation/apps/dedalus_replay_recording \
  --config config/core_stack_airsim_live_rgb.yaml \
  --output-dir out/airsim_live_snapshots \
  --max-frames 5
```

## What Works

```text
AirSim scene image
  -> simulation/airsim-capture-frame.py
  -> P6 PPM stdout
  -> AirSimFrameSource
  -> FramePacket
  -> NoTelemetryEgoProvider
  -> ScriptedDetector
  -> SimpleCentroidTracker
  -> FlatGroundProjector
  -> InMemoryWorldModel
  -> WorldSnapshot JSON
```

## What Still Does Not Work

The following providers are still explicit unavailable integration stubs:

```text
ego_provider: airsim
detector: airsim_ground_truth
projector: airsim_depth
```

Those providers preserve the config and module boundary for later work but do not yet call AirSim RPC APIs.

## CI Coverage

CI does not require AirSim. It validates the live bridge path with:

```text
config/core_stack_airsim_bridge_ci.yaml
tests/fixtures/airsim_bridge_fake.py
tests/unit/test_airsim_provider_boundary.cpp
```

The fake bridge emits a tiny P6 PPM frame to stdout. This verifies that `AirSimFrameSource` can execute an external bridge command, parse the frame, and run through `CoreStackRunner`, without installing AirSim.

## Next Step

The next AirSim slice should add one of:

```text
AirSimEgoStateProvider bridge/backend
AirSimDepthProjector bridge/backend
AirSimGroundTruthDetector bridge/backend
```

Keep those as integration providers and keep default unit tests dependency-free.
