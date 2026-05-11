# Core Stack Current State

This document captures the implemented state of the Dedalus core-stack after the current sensors → perception → world-model work. Keep milestone names out of code/entity names; use architectural names for files, tests, and modules.

## Implemented architecture spine

The repo now has a buildable C++20 core-stack skeleton with two dependency-free frame-ingestion modes:

```text
SyntheticFrameSource OR VideoOnlyFrameSource
  -> optional EgoStateProvider
      -> synthetic ego hint OR NoTelemetryEgoProvider fallback
  -> PerceptionPipeline
      -> ScriptedDetector
      -> SimpleCentroidTracker
      -> AppearanceOnlyIdentityResolver
      -> FlatGroundProjector
  -> InMemoryWorldModel
      -> Dynamic agent output
      -> Container placeholder output
      -> Tactical exclusion-zone output
      -> Uncertain-region output
      -> Rough flight-map output
          -> Static structures
          -> Flight corridors
          -> Landmarks
  -> WorldSnapshot JSON
  -> EffectiveWorldView
```

## Public contracts added

Current public headers include:

```text
include/dedalus/core/types.hpp
include/dedalus/sensors/frame_source.hpp
include/dedalus/sensors/ego_state_provider.hpp
include/dedalus/sensors/replay_frame_source.hpp
include/dedalus/perception/types.hpp
include/dedalus/perception/perception_pipeline.hpp
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
NoTelemetryEgoProvider
ScriptedDetector
SimpleCentroidTracker
AppearanceOnlyIdentityResolver
FlatGroundProjector
ConeExclusionMapper
RoughFlightMapBuilder
InMemoryWorldModel
InProcessBus
```

These are deliberately deterministic placeholders. They exist to lock down interfaces, tests, JSON shape, CI gates, and module boundaries before adding real perception, mapping, memory, or simulation adapters.

## Runtime/debug apps

```text
apps/dedalus_core_stack.cpp
apps/dedalus_dump_world.cpp
```

Both run the synthetic pipeline and emit `WorldSnapshot` JSON. `dedalus_core_stack` is used by CI smoke validation.

## Tests

Current tests are architectural, not milestone-named:

```text
tests/unit/test_world_snapshot_json.cpp
tests/unit/test_perception_world_model_flow.cpp
tests/unit/test_video_only_world_model_flow.cpp
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

Then they run `dedalus_core_stack` and validate emitted JSON contains:

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

## Current status

```text
Core-stack bootstrap: implemented
Synthetic perception pipeline: implemented
Video-only/no-telemetry ingestion boundary: implemented
Replay frame-source boundary: implemented
In-memory world model: implemented
Tactical exclusion layer placeholder: implemented
EffectiveWorldView placeholder: implemented
Rough global flight-map placeholder: implemented
Unit/flow tests: implemented
CI/staging/production smoke assertions: implemented
```

## Still intentionally not implemented

Do not assume these exist yet:

```text
RecordedVideoFrameSource backed by real media decode
MpegCameraSource backed by MPEG/RTSP/GStreamer/OpenCV
AirSimFrameSource
AirSimEgoStateProvider
AirSimDepthProjector
AirSimGroundTruthDetector
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

The next architectural step should be simulation ingestion, not behavior/control:

```text
AirSimFrameSource or RecordedVideoFrameSource backed by actual media frames
  -> same FrameSource contract
  -> same PerceptionPipeline contract
  -> same InMemoryWorldModel contract
  -> WorldSnapshot JSON parity with synthetic/video-only sources
```

Keep real AirSim/PX4/OpenCV/GStreamer dependencies out of unit tests. Simulation/media adapters should be integration-path modules, not required for core library tests.
