# Core Stack Current State

This document captures the implemented state of the Dedalus core-stack and its relationship to the current mission roadmap. Keep milestone names out of code/entity names; use architectural names for files, tests, and modules.

## Current implemented architecture spine

The repo has a buildable C++20 core-stack skeleton with dependency-free provider composition:

```text
config/ci/core_stack_ci.yaml OR config/ci/core_stack_recorded_ci.yaml OR config/ci/core_stack_airsim_example.yaml
  -> load_core_stack_config()
      -> CoreStackProviderConfig
          -> ProviderRegistry
              -> FrameSource provider
                  -> SyntheticFrameSource OR VideoOnlyFrameSource OR RecordedFrameSource OR AirSimFrameSource stub
              -> EgoStateProvider provider
                  -> FrameHintEgoProvider OR NoTelemetryEgoProvider OR AirSimEgoStateProvider stub
              -> Detector provider
                  -> ScriptedDetector OR AirSimGroundTruthDetector stub
              -> Tracker provider
                  -> SimpleCentroidTracker
              -> IdentityResolver provider
                  -> AppearanceOnlyIdentityResolver
              -> Projector3D provider
                  -> FlatGroundProjector OR AirSimDepthProjector stub
              -> WorldModel provider
                  -> InMemoryWorldModel
          -> CoreStackRunner
              -> PerceptionPipeline
              -> InMemoryWorldModel
              -> WorldSnapshot JSON
              -> EffectiveWorldView
```

The live mission path now extends this spine through behavior and flight-control integration:

```text
AirSim live frame + ego sidecar
  -> AirSimFrameSource
  -> FrameHintEgoProvider
  -> CoreStackRunner
  -> InMemoryWorldModel
  -> LatestWorldSnapshot
  -> MissionRuntime async loop
  -> TrajectoryMissionController
  -> Px4BridgeCommandSink
  -> tools/px4/px4-command-bridge.py
  -> PX4 / AirSim
```

The direct synthetic wiring that originally lived in apps has been moved behind config-driven provider composition. This is a plugin-style composition boundary, not dynamic shared-library loading yet.

## Mission state after Milestone 2.20

Milestone 2.20 closed the first live mission robustness phase:

```text
- dedalus_mission_loop flies through AirSim/PX4 using flight_command_sink=px4_bridge
- repeated mission-loop runs work without restarting AirSim
- safe-height climb, trajectory execution, GoHome, Land, and Disarm work
- Ctrl-C / SIGTERM requests graceful mission finish
- mission_events.jsonl is the compact source artifact for mission debugging
- tools/mission/mission-events-summary.py summarizes and validates mission event artifacts
- tools/mission/repeat-mission-smoke.sh validates repeated live mission runs
```

Current mission-loop artifacts:

```text
snapshot_XXXX.json
snapshot_manifest.txt
mission_events.jsonl
```

## Replay artifact workflow

Recorded or simulation-exported frames can be replayed into deterministic snapshot artifacts:

```bash
./build-validation/apps/dedalus_replay_recording \
  --config config/ci/core_stack_recorded_ci.yaml \
  --output-dir out/replay_snapshots \
  --max-frames 0
```

`--max-frames 0` means run until the configured frame source is exhausted. The replay app writes:

```text
out/replay_snapshots/snapshot_0001.json
out/replay_snapshots/snapshot_0002.json
...
out/replay_snapshots/snapshot_manifest.txt
```

This remains the preferred regression/debug artifact path for recorded fixtures and AirSim-exported frames.

## Config format decision

Current config uses a flat YAML subset because the repo already has YAML config files and provider selection benefits from human-readable key/value pairs.

The parser is intentionally dependency-free and only supports flat `key: value` entries for now. Do not add `yaml-cpp`, TOML, OpenCV, GStreamer, or AirSim as a requirement for core unit tests.

Provider selection and mission behavior should remain config-driven rather than hardcoded directly into apps or tests.

## Current tree shape

```text
config/
include/dedalus/
src/
apps/
tests/
simulation/
docs/
```

Important application entry points:

```text
apps/dedalus_core_stack.cpp
apps/dedalus_dump_world.cpp
apps/dedalus_replay_recording.cpp
apps/dedalus_mission_loop.cpp
```

Important mission tooling:

```text
simulation/airsim/scripts/test-flight.py
tools/px4/px4-command-bridge.py
simulation/airsim/scripts/airsim-prepare-session.py
simulation/airsim/scripts/airsim-stream-frames-binary.py
tools/mission/mission-events-summary.py
tools/mission/repeat-mission-smoke.sh
```

## Public contracts added

Current public headers include core, sensors, perception, runtime, ipc, world-model, and behavior contracts.

Important value contracts now represented:

```text
FramePacket
ImageView
CameraIntrinsics
EgoStateEstimate
RecordedFrameManifestEntry
AirSimProviderConfig
CoreStackProviderConfig
CoreStackProviders
Detection2D
Track2D
IdentityHypothesis
Observation3D
EgoState
AgentState
ContainerState
ExclusionZone
MapFrame
StaticStructure
FlightCorridor
Landmark
UncertainRegion
WorldSnapshot
EffectiveWorldView
MissionController
MissionRuntime
VelocityTrajectory
FlightCommandSink
LatestWorldSnapshot
```

## Implemented placeholder and integration modules

```text
SyntheticFrameSource
ReplayFrameSource
VideoOnlyFrameSource
RecordedFrameSource
FrameHintEgoProvider
NoTelemetryEgoProvider
ScriptedDetector
SimpleCentroidTracker
AppearanceOnlyIdentityResolver
FlatGroundProjector
ConeExclusionMapper
RoughFlightMapBuilder
InMemoryWorldModel
InProcessBus
ProviderRegistry
CoreStackRunner
load_core_stack_config
AirSimFrameSource / AirSim bridge integration path
TrajectoryMissionController
MissionRuntime
Px4BridgeCommandSink
AirSimVelocityCommandSink
mission event artifact writer
mission event summary helper
repeat mission smoke helper
```

These are deliberately deterministic placeholders or explicit integration adapters. They exist to lock down interfaces, tests, JSON shape, CI gates, and module boundaries before adding real perception, mapping, memory, or advanced behavior.

## Tests

Current tests cover world snapshot JSON, perception/world-model flow, video-only flow, provider composition, config loading, recorded-frame flow, AirSim provider boundary, mission runtime, trajectory mission controller, latest world snapshot behavior, command sinks, replay smoke, annotation/export smoke, and mission-loop smoke.

Run:

```bash
cmake --build build-staging -j$(nproc)
ctest --test-dir build-staging --output-on-failure
```

Expected result after the Milestone 2.20 closeout:

```text
100% tests passed, 0 tests failed out of 18
```

## CI smoke contract

The CI, staging, and production workflows build with:

```text
-DDEDALUS_BUILD_APPS=ON
-DDEDALUS_BUILD_TESTS=ON
```

Then they run:

```text
ctest --test-dir <build-dir> --output-on-failure
```

Core and recorded-frame paths remain dependency-free. Live AirSim/PX4 mission validation is an operator/integration path and should not become mandatory for unit tests.

## Current status

```text
Core-stack bootstrap: implemented
Provider/plugin-style composition boundary: implemented
Config-driven provider composition: implemented
CI-safe provider config: implemented
Recorded-frame CI config: implemented
Synthetic perception pipeline: implemented
Video-only/no-telemetry ingestion boundary: implemented
Replay frame-source boundary: implemented
Recorded frame-source boundary: implemented
AirSim bridge/live mission path: implemented for simulation integration
Replay snapshot artifact generation: implemented
In-memory world model: implemented
Tactical exclusion layer placeholder: implemented
EffectiveWorldView placeholder: implemented
Rough global flight-map placeholder: implemented
Mission runtime and trajectory mission controller: implemented
PX4 bridge mission path: implemented and validated for repeated runs
Mission event artifacts and summary tooling: implemented
```

## Still intentionally not implemented

Do not assume these exist yet:

```text
Dynamic plugin loading from shared libraries
Nested/full YAML parser
TOML parser
RecordedVideoFrameSource backed by real media decode
MpegCameraSource backed by MPEG/RTSP/GStreamer/OpenCV
Production AirSim C++ RPC backend
YoloOnnxDetector
YoloTensorRtDetector
real ReID
container-aware identity resolver
object behavior mission controller
behavior spec parser
target selector
follow/circle/approach behaviors
persistent memory layer
relative map store
real flight corridor extraction
voxel/SDF obstacle mapping
tactical avoidance planner
route cache
map + drone POV visualization
iceoryx runtime transport
```

## Next recommended step

The next architectural step is:

```text
Milestone 2.21 — Mission artifact validation and replay-grade diagnostics
```

Immediate goal:

```text
live mission output directory
  -> mission_events.jsonl + snapshot_XXXX.json
  -> formal validator
  -> state ordering, command failures, height gates, landing/completion checks
```

This keeps the now-working live mission path protected before adding object-conditioned behavior.

## Roadmap to Milestone 3

Milestone 3 is now defined as:

```text
Milestone 3.0 — Object-conditioned flight behavior demo
```

Path:

```text
2.21 Mission artifact validator
2.22 Scenario/campaign harness
2.23 Behavior spec parser foundation
2.24 Target selector from WorldSnapshot agents
2.25 ObjectBehaviorMissionController
2.26 Follow behavior
2.27 Circle behavior
2.28 Approach + behavior sequence
2.29 M3 demo hardening
3.0  Object-conditioned flight behavior demo
```

Milestone 3 should demonstrate:

```text
detected/tracked class instance such as person or car
  -> TargetSelector
  -> follow / circle / approach / sequence behavior
  -> bounded velocity vectors through existing PX4 bridge
  -> GoHome / Land / Complete
  -> mission_events + snapshots prove the behavior sequence
```

## Post-M3 spatial autonomy roadmap

Post-M3 work adds tactical spatial reasoning while preserving the behavior/sink boundary:

```text
BehaviorController
  -> desired velocity vector
  -> TacticalAvoidancePlanner
  -> safe velocity vector
  -> Px4BridgeCommandSink
```

Roadmap:

```text
4.0 Local tactical occupancy map
5.0 Reactive obstacle avoidance planner
6.0 Persistent traverse map / flight memory
7.0 Cached route solutions
8.0 Tactical map + drone POV visualization
9.0 Spatial autonomy demo with avoidance
10.0 Multi-flight site memory
```

Design rules:

```text
- M3 is object-conditioned behavior, not full obstacle avoidance.
- Avoidance belongs between behavior and the flight sink.
- The flight sink should only receive bounded velocity/yaw intent.
- Fresh tactical sensing overrides persistent traverse memory and route cache.
```
