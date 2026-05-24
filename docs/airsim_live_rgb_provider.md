# AirSim Live RGB + Ego Providers

This document describes the current live AirSim RGB and ego-state ingestion path.

## Status

The core stack now supports live AirSim scene images and ego-state telemetry through external bridge commands:

```text
frame_source: airsim
airsim_bridge_command: python3 simulation/airsim/scripts/airsim-capture-frame.py
ego_provider: airsim
airsim_ego_bridge_command: python3 simulation/airsim/scripts/airsim-capture-ego.py
detector: scripted
projector: flat_ground
```

This is not a direct C++ AirSim RPC client. The C++ `AirSimFrameSource` and `AirSimEgoStateProvider` remain dependency-free by invoking configured bridge commands. The frame bridge returns P6 PPM bytes on stdout. The ego bridge returns flat JSON pose/velocity telemetry on stdout.

## Runtime Config

For RGB only with fallback/no-telemetry ego, use:

```text
config/core_stack_airsim_live_rgb.yaml
```

For RGB plus AirSim ego telemetry, use:

```text
config/core_stack_airsim_live_rgb_ego.yaml
```

Example RGB + ego config:

```yaml
frame_source: airsim
airsim_bridge_command: python3 simulation/airsim/scripts/airsim-capture-frame.py
ego_provider: airsim
airsim_ego_bridge_command: python3 simulation/airsim/scripts/airsim-capture-ego.py
detector: scripted
tracker: simple_centroid
identity_resolver: appearance_only
projector: flat_ground
world_model: in_memory
fallback_map_frame_id: map_airsim_live_ego_0001

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
  --config config/core_stack_airsim_live_rgb_ego.yaml
```

For multiple snapshots:

```bash
./build-validation/apps/dedalus_replay_recording \
  --config config/core_stack_airsim_live_rgb_ego.yaml \
  --output-dir out/airsim_live_snapshots \
  --max-frames 5
```

## What Works

```text
AirSim scene image
  -> simulation/airsim/scripts/airsim-capture-frame.py
  -> P6 PPM stdout
  -> AirSimFrameSource
  -> FramePacket

AirSim vehicle state
  -> simulation/airsim/scripts/airsim-capture-ego.py
  -> flat JSON stdout
  -> AirSimEgoStateProvider
  -> EgoState

FramePacket + EgoState
  -> ScriptedDetector
  -> SimpleCentroidTracker
  -> FlatGroundProjector
  -> InMemoryWorldModel
  -> WorldSnapshot JSON
```

## What Still Does Not Work

The following providers are still explicit unavailable integration stubs:

```text
detector: airsim_ground_truth
projector: airsim_depth
```

Those providers preserve the config and module boundary for later work but do not yet call AirSim RPC APIs.

## CI Coverage

CI does not require AirSim. It validates the live bridge path with:

```text
config/core_stack_airsim_bridge_ci.yaml
tests/fixtures/airsim_bridge_ci_fake.py
tests/fixtures/airsim_ego_bridge_ci_fake.py
tests/unit/test_airsim_provider_boundary.cpp
```

The fake frame bridge emits a tiny P6 PPM frame to stdout. The fake ego bridge emits deterministic pose and velocity JSON. This verifies that `AirSimFrameSource` and `AirSimEgoStateProvider` can execute external bridge commands, parse their outputs, and run through `CoreStackRunner`, without installing AirSim.

## Next Step

The next AirSim slice should add one of:

```text
AirSimDepthProjector bridge/backend
AirSimGroundTruthDetector bridge/backend
```

Keep those as integration providers and keep default unit tests dependency-free.
