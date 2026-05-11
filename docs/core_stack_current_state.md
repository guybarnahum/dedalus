# Core Stack Current State

This document captures the implemented state of the Dedalus core-stack after the current sensors → perception → world-model work. Keep milestone names out of code/entity names; use architectural names for files, tests, and modules.

## Implemented architecture spine

The repo now has a buildable C++20 core-stack skeleton with dependency-free provider composition:

```text
config/core_stack_ci.yaml OR config/core_stack_recorded_ci.yaml OR config/core_stack_airsim_example.yaml
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

The direct synthetic wiring that originally lived in apps has been moved behind config-driven provider composition. This is a plugin-style composition boundary, not dynamic shared-library loading yet.

## Replay artifact workflow

Recorded or simulation-exported frames can now be replayed into deterministic snapshot artifacts:

```bash
./build-validation/apps/dedalus_replay_recording \
  --config config/core_stack_recorded_ci.yaml \
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

This is the preferred regression/debug artifact path for recorded fixtures and AirSim-exported frames.

## Config format decision

Current config uses a flat YAML subset because the repo already has YAML config files and provider selection benefits from human-readable key/value pairs.

```text
config/core_stack_ci.yaml
config/core_stack_recorded_ci.yaml
config/core_stack_airsim_example.yaml
```

The parser is intentionally dependency-free and only supports flat `key: value` entries for now. Do not add `yaml-cpp`, TOML, OpenCV, GStreamer, or AirSim as a requirement for core unit tests.

Supported keys:

```text
frame_source
recorded_manifest_path
ego_provider
detector
tracker
identity_resolver
projector
world_model
fallback_map_frame_id
airsim_host
airsim_rpc_port
airsim_vehicle_name
airsim_camera_name
```

## Current tree shape

```text
config/
├── behaviors.yaml
├── camera_intrinsics.yaml
├── core_stack_airsim_example.yaml
├── core_stack_ci.yaml
└── core_stack_recorded_ci.yaml

include/dedalus/
├── core/
├── ipc/
├── perception/
├── runtime/
├── sensors/
├── simulation/
└── world_model/

src/
├── perception/
├── runtime/
├── sensors/
├── simulation/
└── world_model/

apps/
├── dedalus_core_stack.cpp
├── dedalus_dump_world.cpp
└── dedalus_replay_recording.cpp

tests/
├── fixtures/
│   └── recorded_frames/
│       ├── frame_0001.ppm
│       └── manifest.txt
├── integration/
│   └── test_replay_recording_smoke.py
└── unit/
    ├── test_airsim_provider_boundary.cpp
    ├── test_core_stack_config_loader.cpp
    ├── test_perception_world_model_flow.cpp
    ├── test_provider_composition.cpp
    ├── test_recorded_frame_world_model_flow.cpp
    ├── test_video_only_world_model_flow.cpp
    └── test_world_snapshot_json.cpp
```

## Public contracts added

Current public headers include:

```text
include/dedalus/core/types.hpp
include/dedalus/sensors/frame_source.hpp
include/dedalus/sensors/ego_state_provider.hpp
include/dedalus/sensors/recorded_frame_source.hpp
include/dedalus/sensors/replay_frame_source.hpp
include/dedalus/simulation/airsim_providers.hpp
include/dedalus/perception/types.hpp
include/dedalus/perception/perception_pipeline.hpp
include/dedalus/runtime/config_loader.hpp
include/dedalus/runtime/provider_registry.hpp
include/dedalus/runtime/core_stack_runner.hpp
include/dedalus/ipc/in_process_bus.hpp
include/dedalus/world_model/world_snapshot.hpp
include/dedalus/world_model/effective_world_view.hpp
include/dedalus/world_model/in_memory_world_model.hpp
include/dedalus/world_model/tactical_obstacle_mapper.hpp
include/dedalus/world_model/rough_flight_map_builder.hpp
```

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
```

## Implemented placeholder modules

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
AirSimFrameSource stub
AirSimEgoStateProvider stub
AirSimDepthProjector stub
AirSimGroundTruthDetector stub
dedalus_replay_recording
```

These are deliberately deterministic placeholders or explicit integration stubs. They exist to lock down interfaces, tests, JSON shape, CI gates, and module boundaries before adding real perception, mapping, memory, or simulation adapters.

`RecordedFrameSource` is a real file-backed adapter for CI-safe recorded frames. It currently reads a simple manifest plus P3/P6 PPM image files. This is intentionally not a full media decoder.

The AirSim providers are registered integration stubs. They intentionally throw a clear runtime error in dependency-free builds. They define the provider boundary and config names for future AirSim/PX4 integration but do not yet call AirSim RPC APIs.

## Provider names currently registered

```text
frame_source: synthetic, video_only, recorded_frames, airsim
ego_provider: frame_hint, no_telemetry, airsim
detector: scripted, airsim_ground_truth
tracker: simple_centroid
identity_resolver: appearance_only
projector: flat_ground, airsim_depth
world_model: in_memory
```

Future real providers should be added behind this registry and selected through config rather than hardcoded directly into apps or tests.

## Runtime/debug apps

```text
apps/dedalus_core_stack.cpp
apps/dedalus_dump_world.cpp
apps/dedalus_replay_recording.cpp
```

`dedalus_core_stack` and `dedalus_dump_world` load provider config with `--config`, use `ProviderRegistry` + `CoreStackRunner`, and emit one `WorldSnapshot` JSON. If `--config` is omitted, they default to `config/core_stack_ci.yaml`.

`dedalus_replay_recording` loads provider config with `--config`, runs until the configured frame source is exhausted, and writes one snapshot JSON per processed frame plus `snapshot_manifest.txt`.

Examples:

```bash
./build-validation/apps/dedalus_core_stack --config config/core_stack_ci.yaml
./build-validation/apps/dedalus_core_stack --config config/core_stack_recorded_ci.yaml
./build-validation/apps/dedalus_core_stack --config config/core_stack_airsim_example.yaml
./build-validation/apps/dedalus_replay_recording --config config/core_stack_recorded_ci.yaml --output-dir out/replay_snapshots
```

The AirSim example command will fail in the current dependency-free build with an explicit integration-provider unavailable error.

## Tests

Current tests are architectural, not milestone-named:

```text
tests/unit/test_world_snapshot_json.cpp
tests/unit/test_perception_world_model_flow.cpp
tests/unit/test_video_only_world_model_flow.cpp
tests/unit/test_provider_composition.cpp
tests/unit/test_core_stack_config_loader.cpp
tests/unit/test_recorded_frame_world_model_flow.cpp
tests/unit/test_airsim_provider_boundary.cpp
tests/integration/test_replay_recording_smoke.py
```

`test_world_snapshot_json` guards the previous `map_frames` serialization bug.

`test_perception_world_model_flow` validates the full synthetic perception/world-model flow and asserts:

```text
one detection
one track
one identity hypothesis
one 3D observation
one agent
one tactical exclusion zone
one uncertain region
one static structure
one flight corridor
one landmark
EffectiveWorldView contains actual state and uncertainty
```

`test_video_only_world_model_flow` validates the degraded/no-telemetry ingestion path and asserts:

```text
VideoOnlyFrameSource emits no ego_hint
NoTelemetryEgoProvider creates a low-confidence fallback ego state
map_video_only_0001 is preserved as the relative map frame
perception/world-model artifacts are still emitted
appearance confidence remains low
```

`test_provider_composition` validates the provider abstraction itself:

```text
ProviderRegistry lists multiple frame sources
synthetic + frame_hint composition runs through CoreStackRunner
video_only + no_telemetry composition runs through CoreStackRunner
unknown provider names are rejected
```

`test_core_stack_config_loader` validates:

```text
config/core_stack_ci.yaml parses into expected provider names
fallback_map_frame_id is loaded
config-composed CoreStackRunner emits expected state
```

`test_recorded_frame_world_model_flow` validates:

```text
RecordedFrameSource loads tests/fixtures/recorded_frames/manifest.txt
P3 PPM frame metadata and dimensions are preserved
config/core_stack_recorded_ci.yaml parses recorded_manifest_path
recorded_frames + no_telemetry composition runs through CoreStackRunner
recorded flow emits world-model artifacts
```

`test_airsim_provider_boundary` validates:

```text
config/core_stack_airsim_example.yaml parses AirSim provider names and connection fields
ProviderRegistry lists the AirSim frame source
AirSimFrameSource throws a clear integration-provider unavailable error in dependency-free builds
```

`test_replay_recording_smoke.py` validates:

```text
dedalus_replay_recording runs on config/core_stack_recorded_ci.yaml
snapshot_0001.json is written
snapshot_manifest.txt is written
snapshot JSON preserves map_recorded_ci_0001 and contains world-model artifacts
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

Then they run `dedalus_core_stack` through the CI provider config:

```text
./build-*/apps/dedalus_core_stack --config config/core_stack_ci.yaml
```

and validate emitted JSON contains:

```text
active_map_frame_id = map_local_0001
person agent
car container
map_frame_id = map_local_0001
tactical_exclusion_zones
reason = dynamic_observation_cone
uncertain_regions
flight_corridors
corridor_id = corridor_forward_0001
static_structures
structure_id = structure_building_0001
landmarks
landmark_id = landmark_building_corner_0001
```

Recorded-frame ingestion, AirSim provider-boundary checks, and replay artifact generation are covered by CTest, not by the primary smoke artifact.

## Current status

```text
Core-stack bootstrap: implemented
Provider/plugin-style composition boundary: implemented
Config-driven provider composition: implemented
CI-safe provider config: implemented
Recorded-frame CI config: implemented
AirSim provider example config: implemented
Synthetic perception pipeline: implemented
Video-only/no-telemetry ingestion boundary: implemented
Replay frame-source boundary: implemented
Recorded frame-source boundary: implemented
AirSim provider boundary: implemented as explicit unavailable stubs
Replay snapshot artifact generation: implemented
In-memory world model: implemented
Tactical exclusion layer placeholder: implemented
EffectiveWorldView placeholder: implemented
Rough global flight-map placeholder: implemented
Unit/flow/provider/config/recorded-ingestion/AirSim-boundary/replay tests: implemented
CI/staging/production smoke assertions: implemented
```

## Still intentionally not implemented

Do not assume these exist yet:

```text
Dynamic plugin loading from shared libraries
Nested/full YAML parser
TOML parser
RecordedVideoFrameSource backed by real media decode
MpegCameraSource backed by MPEG/RTSP/GStreamer/OpenCV
Real AirSim RPC backend
Real AirSimFrameSource frame capture
Real AirSimEgoStateProvider telemetry capture
Real AirSimDepthProjector depth-image projection
Real AirSimGroundTruthDetector object/segmentation ground truth
NullDetector
IouTracker
KalmanTracker2D
YoloOnnxDetector
YoloTensorRtDetector
real ReID
container-aware identity resolver
persistent memory layer
relative map store
real flight corridor extraction
voxel/SDF obstacle mapping
behavior tree/control output
iceoryx runtime transport
```

## Next recommended step

The next architectural step should be replay artifact comparison and regression baselines, not behavior/control:

```text
recorded/simulation replay input
  -> dedalus_replay_recording
  -> snapshot JSON sequence
  -> baseline manifest / diff tool
  -> CI-safe regression checks for stable fields
```

Keep real AirSim/PX4/OpenCV/GStreamer dependencies out of unit tests. Simulation/media adapters should be integration-path modules, not required for core library tests.
