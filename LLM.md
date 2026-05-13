# Project Dedalus - LLM Context & Operational State

> Purpose: This file is the project handoff document for LLMs. It should let an LLM understand the architecture, repo conventions, current simulation state, known bugs, and safe modification patterns without rediscovering the debugging history.

> Current implementation note: for the latest concrete core-stack implementation state, see `docs/core_stack_current_state.md`. That file tracks the buildable contracts, placeholder modules, tests, CI smoke contract, and next recommended step.

> Current handoff checkpoint: this file was updated after the dependency-free PPM frame annotation provider slice. The active development phase remains **Milestone 2: Video and Simulation Input**, with the core focus now on validating visual annotation artifacts against recorded/simulated frame streams and preparing optional MP4/video export without adding mandatory media dependencies.

---

## 0. Current Milestone 2 State

The repo is currently in Milestone 2. Milestone 1 is complete and the active work is connecting real or simulated frame sources into the C++ core-stack while keeping providers modular and CI dependency-free.

The current architectural rule is:

```text
Core contracts should not know whether a source is AirSim, MPEG, camera SDK,
OpenCV, GStreamer, shared memory, pipe, or future hardware.

Provider implementations may know source-specific details.
Config keys should remain source-neutral wherever possible.
```

The current buildable core-stack path is:

```text
FrameSource
  -> EgoStateProvider
  -> Detector
  -> CameraStabilizer
  -> Tracker
  -> IdentityResolver
  -> Projector3D
  -> InMemoryWorldModel
  -> FrameAnnotationSink
  -> WorldSnapshot JSON
```

Current CI-safe provider composition defaults:

```yaml
frame_source: synthetic
ego_provider: frame_hint
detector: scripted
camera_stabilizer: null
tracker: simple_centroid
identity_resolver: appearance_only
projector: flat_ground
world_model: in_memory
frame_annotator: null
fallback_map_frame_id: map_local_0001
```

Dependency-light visual validation provider:

```yaml
frame_annotator: ppm_sequence
annotation_output_path: out/annotations
annotation_output_fps: 5
```

`frame_annotator: null` remains the default and CI-safe no-op provider. `frame_annotator: ppm_sequence` is the first real visual annotation provider. It writes annotated PPM frames and a manifest using raw RGB pixel operations only, with no OpenCV, FFmpeg, GStreamer, or media-codec dependency. `frame_annotator: mp4` remains an explicit placeholder for a later optional export provider.

### Current source/provider work completed in Milestone 2

Milestone 2 currently includes the following implemented slices:

```text
2A — Recorded-frame ingestion provider
  Implemented via RecordedFrameSource and recorded-frame fixture configs.

2B — AirSim provider boundary
  Implemented as registered provider names and provider configs.

2C — AirSim export bridge
  Implemented as Python AirSim frame export to PPM/manifest, consumable by RecordedFrameSource.

2D — Replay snapshot artifacts
  Implemented via apps/dedalus_replay_recording.

2E.1 — Live AirSim RGB ingestion
  Implemented through a bridge-backed frame source.

2E.2 — Live AirSim ego-state ingestion
  Implemented through a bridge-backed ego-state provider.

2E.3 — Persistent AirSim frame stream
  Implemented through stream_jsonl mode.

2E.4 — Bridge transport abstraction
  Implemented through BridgeTransport with pipe implemented and shared_memory placeholder.

2E.5 — Binary framed bridge protocol
  Implemented through stream_binary mode.

2E.6 — Simulation/run orchestration
  Implemented in simulation/run.sh with optional core-stack and flight-control windows,
  sampling FPS control, and capture alignment to velocity-control start.

2E.7 — Perception stabilization and annotation hooks
  Implemented with NullCameraStabilizer and NullFrameAnnotationSink.

2E.8 — Dependency-free visual annotation output
  Implemented with PpmFrameAnnotationSink behind frame_annotator: ppm_sequence.
  The provider writes annotated PPM image sequences plus manifest.txt for visual validation.

2E.9 — stream_binary_ego frame-attached ego sidecar path
  Implemented through FramePacket::ego_hint + FrameHintEgoProvider.
  Removes the separate airsim_ego_bridge_command runtime cost when using --include-ego.

Current optimization slice:
  Milestone 2.14 — optimize AirSim co-stream sampling.
  First step: remove the extra simGetVehiclePose() RPC from
  simulation/airsim-stream-frames-binary.py and derive pose/orientation from
  getMultirotorState().kinematics_estimated.
  Goal: reduce stream_binary_ego frame_source.next_frame latency while preserving
  the frame-attached ego contract.
```

### Current source-neutral bridge config contract

The project intentionally removed old AirSim-prefixed config keys from the config loader.

Use only generic keys:

```yaml
source_host: 127.0.0.1
source_rpc_port: 41451
vehicle_name: PX4
vehicle_camera_name: front_center

bridge_transport: pipe
bridge_mode: stream_binary
bridge_command: python3 simulation/airsim-stream-frames-binary.py --count 0 --rate-hz 5

ego_bridge_command: python3 simulation/airsim-capture-ego.py
```

Do **not** use these removed legacy keys:

```yaml
airsim_host:
airsim_rpc_port:
airsim_vehicle_name:
airsim_camera_name:
airsim_transport:
airsim_bridge_mode:
airsim_bridge_command:
airsim_ego_bridge_command:
```

If those keys are used, `load_core_stack_config()` should fail with `unknown core-stack config key`.

### Current bridge modes

Frame bridge modes currently supported by `AirSimFrameSource`:

```text
one_shot_ppm
  Debug/simple path. Runs a bridge command per frame and expects P6 PPM bytes.

stream_jsonl
  Persistent line-oriented path. Reads one JSON record per frame with base64 PPM payload.

stream_binary
  Preferred current path. Reads a fixed-size binary header plus raw RGB payload.
```

Preferred runtime config:

```text
config/core_stack_airsim_binary_rgb_ego.yaml
```

Preferred command:

```bash
./build-staging/apps/dedalus_replay_recording \
  --config config/core_stack_airsim_binary_rgb_ego.yaml \
  --output-dir out/airsim_binary_snapshots \
  --max-frames 5
```

### Current bridge transports

The bridge transport layer is separate from provider semantics.

Current transport interface:

```cpp
class BridgeTransport {
public:
    virtual std::string request_once(const std::string& command) = 0;
    virtual std::optional<std::string> read_stream_line(const std::string& command) = 0;
    virtual std::optional<std::string> read_stream_bytes(const std::string& command, std::size_t byte_count) = 0;
    virtual void close_stream() = 0;
};
```

Implemented:

```text
PipeBridgeTransport
```

Explicit placeholder:

```text
SharedMemoryBridgeTransport
```

`shared_memory` is config-selectable but intentionally throws:

```text
shared_memory bridge transport is not implemented yet
```

Do not implement real shared memory until the binary frame protocol and timing semantics are stable.

### Binary frame protocol

`bridge_mode: stream_binary` uses the protocol documented in:

```text
docs/binary_frame_bridge_protocol.md
```

Header:

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

C++ validates:

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

### Current simulation/run.sh orchestration

`simulation/run.sh` now supports launching both the simulator/control side and the core-stack capture side.

Example:

```bash
cd ~/dedalus/simulation

./run.sh AirSimNH \
  --with-flight-control \
  --with-core-stack \
  --core-sampling-fps 5 \
  --core-max-frames 0
```

Timing model:

```text
PX4 window starts
sleep control-start-delay-s
flight-control starts
core-stack capture starts at the same scheduled time
```

`--core-max-frames 0` means:

```text
With --with-flight-control:
  derive max frames from trajectory duration × core-sampling-fps
  so capture ends with the velocity-control trajectory.

Without --with-flight-control:
  keep old behavior and run until the stream ends.
```

`run.sh` generates:

```text
out/airsim_run_<timestamp>/core_stack_runtime.yaml
```

and rewrites `--rate-hz` inside the selected bridge command to match `--core-sampling-fps`.

---

## 1. Project Identity

Dedalus is a virtual proving ground and edge-autonomy stack for drone behavior, perception, world modeling, tactical mapping, memory, and control experiments.

The current repo combines:

- C++20 autonomy/runtime code under `src/`
- configuration under `config/`
- simulation orchestration under `simulation/`
- infrastructure helpers under `infrastructure/`
- project strategy/architecture notes in `WHITEPAPER.md` and this file

The simulation environment is working well enough to become the core-stack integration harness:

```text
Colosseum / AirSim fork  <->  PX4 SITL  <->  Python flight test client
```

The immediate engineering focus is no longer only simulation bring-up. The next phase is the **core-stack**:

```text
sensors -> perception -> world_model
```

The goal is to build a modular C++ perception-to-world-model runtime that can first run with placeholder blocks, then continuously improve as better detectors, trackers, depth estimators, VIO, mapping, memory, identity, and planning modules become available.

---

## 2. Current Repository Structure

The core-stack spine is implemented. Public contracts live under `include/dedalus/`, implementations under `src/`, apps under `apps/`, and tests under `tests/`. The `tools/` directory is not yet created.

Current layout:

```text
include/dedalus/
├── core/types.hpp
├── sensors/frame_source.hpp
├── perception/types.hpp
├── perception/perception_pipeline.hpp
├── ipc/in_process_bus.hpp
└── world_model/
    ├── world_snapshot.hpp
    ├── effective_world_view.hpp
    ├── in_memory_world_model.hpp
    ├── tactical_obstacle_mapper.hpp
    └── rough_flight_map_builder.hpp

src/
├── sensors/synthetic_frame_source.cpp
├── perception/perception_pipeline.cpp
└── world_model/
    ├── world_snapshot.cpp
    ├── in_memory_world_model.cpp
    ├── tactical_obstacle_mapper.cpp
    └── rough_flight_map_builder.cpp

apps/
├── dedalus_core_stack.cpp
└── dedalus_dump_world.cpp

tests/unit/
├── test_world_snapshot_json.cpp
└── test_perception_world_model_flow.cpp

tools/
└── (not yet created)
```

---

## 3. Core Runtime Domains

- `sensors/`: camera, video stream, IMU, MAVLink/FCU state, ego-state ingestion
- `perception/`: detection, tracking, identity, features, depth, segmentation, lighting/appearance estimation, projection, TensorRT inference
- `world_model/`: dynamic agents, containers, tactical exclusion zones, relative/global flight maps, landmarks, localization, appearance conditions, memory
- `behavior/`: policy, intent, behavior trees, mission logic
- `safety/`: command mux, kill switch, manual override, bounded outputs
- `ipc/`: low-latency data exchange; intended production IPC is Eclipse iceoryx

Immediate focus:

```text
Frame input
    -> detection
    -> tracking + identity hypotheses
    -> depth / geometry / projection
    -> dynamic agents
    -> container relationships
    -> tactical obstacle/exclusion zones
    -> relative map
    -> rough global flight map
    -> landmarks
    -> appearance-condition model
    -> actual + memory world model
    -> WorldSnapshot / EffectiveWorldView
```

Do **not** jump into behavior trees, intercept behavior, or command output until world-model snapshots are stable.

The autonomy stack should eventually produce bounded kinematic intents:

```text
velocity vector + yaw/yaw-rate intent
```

The flight controller remains responsible for stabilization, estimator fusion, arming state, motor control, failsafes, and low-level flight safety.

Command priority should remain:

```text
Hardware Kill Switch
    >
Human RC Override
    >
Safety Constraint Layer
    >
AI Planner Intent
```

---

## 4. Core-Stack Design Principle

Every stage must be a replaceable module behind a stable interface.

First implementations may be simplistic:

```text
ScriptedDetector
SimpleCentroidTracker
NullCameraStabilizer
NullFrameAnnotationSink
AppearanceOnlyIdentityResolver
FlatGroundProjector
ConeExclusionMapper
InMemoryWorldModel
DisabledMemoryLayer
```

Future implementations should fit behind the same contracts:

```text
YoloTensorRtDetector
ReIdKalmanTracker
FeatureBasedCameraStabilizer
Mp4FrameAnnotationSink
ContainerAwareIdentityResolver
ClothingChangeRobustIdentityResolver
VioRayProjector
VoxelSdfMapper
MultiResolutionFlightMap
RelativeMapStore
PersistentMemoryLayer
LightingRobustEmbeddings
```

Avoid hardcoding AirSim, TensorRT, PX4, YOLO, iceoryx, OpenCV, GStreamer, FFmpeg, shared memory, or a specific camera path into the core contracts.

---

## 5. New Core Concepts To Preserve

### Container-aware identity

People can enter cars, boats, houses, garages, and buildings. These containers can hide or transport people. A person can later exit with changed appearance or clothing. Preserve identity as a probabilistic hypothesis using appearance, motion continuity, entry/exit events, container trajectory, location, timing, gait/body cues, and confidence.

### Relative map frames

Dedalus must support mapping without global location anchoring. A drone may remember a neighborhood, canyon, industrial yard, dock, building cluster, or mountain pass without knowing latitude/longitude. A relative map can later receive a geographic anchor without rewriting the map.

### Appearance conditions

Day/night, season, sun angle, shadows, weather, dust, wet roads, snow, visible/IR/thermal mode, and artificial lighting can change the appearance of people, objects, roads, terrain, and buildings. Do not treat every appearance change as a persistent map change.

---

## 6. Core Contracts

Contracts should live under `include/dedalus/...`.

Important contracts:

```text
FramePacket
Detection2D
Track2D
IdentityHypothesis
EgoState
Observation3D
AgentState
ContainerState
ContainmentEvent
ContainmentHypothesis
ExclusionZone
MapFrame
RelativeMapReference
FlightMapCell
StaticStructure
FlightCorridor
AppearanceCondition
AppearanceConditionSignature
WorldSnapshot
EffectiveWorldView
```

### 5.1 FramePacket

```cpp
struct FramePacket {
    FrameId frame_id;
    TimePoint timestamp;
    CameraId camera_id;
    ImageView image;
    CameraIntrinsics intrinsics;

    std::optional<Pose3> camera_T_world;
    std::optional<Pose3> camera_T_body;
    std::optional<EgoState> ego_hint;
    std::optional<AppearanceCondition> appearance_condition;
};
```

Supported frame-source modes:

```text
SyntheticFrameSource
RecordedVideoFrameSource
MpegCameraSource
AirSimFrameSource
```

Future frame-source modes:

```text
MipiCsiCameraSource
GmslCameraSource
NvArgusCameraSource
```

Important: support video-only sources with no telemetry. Missing ego-state must be explicit and downstream confidence should degrade accordingly.

### 5.2 Detection2D

```cpp
struct Detection2D {
    DetectionId detection_id;
    FrameId frame_id;
    TimePoint timestamp;
    Rect2 bbox_px;
    float confidence;
    ClassLabel class_label;
    FactionLabel faction;
    FeatureVector appearance;
    AppearanceCondition appearance_condition;
};
```

Initial detectors:

```text
NullDetector
ScriptedDetector
AirSimGroundTruthDetector
CpuMockDetector
```

Future detectors:

```text
YoloOnnxDetector
YoloTensorRtDetector
SegmentationDetector
PoseDetector
LightingRobustDetector
```

### 5.3 Tracking and Identity

```cpp
struct Track2D {
    TrackId track_id;
    TimePoint timestamp;
    Rect2 bbox_px;
    ClassLabel class_label;
    FactionLabel faction;
    float confidence;
    TrackState state;      // tentative, confirmed, lost
    int age_frames;
    int missed_frames;
};

struct IdentityHypothesis {
    IdentityId identity_id;
    TrackId track_id;
    TimePoint timestamp;
    float confidence;
    std::vector<std::string> evidence;  // appearance, motion, container_exit, location, timing
};
```

Initial trackers:

```text
SimpleCentroidTracker
IouTracker
AppearanceOnlyIdentityResolver
```

Future trackers:

```text
KalmanTracker2D
DeepSortLikeTracker
OpticalFlowAssistedTracker
ReIdTracker
ContainerAwareIdentityResolver
ClothingChangeRobustIdentityResolver
```

### 5.3.1 CameraStabilizer

Camera stabilization is now an explicit pipeline provider after detection and before tracking.

Current stage order:

```text
FramePacket
    -> Detector
    -> CameraStabilizer
    -> Tracker
    -> IdentityResolver
    -> Projector3D
```

The stabilizer receives the current frame plus detections. This placement allows the stabilizer to use detected objects, keypoints, or future feature tracks to estimate camera motion and stabilize detections before tracker association.

Current contract:

```cpp
struct StabilizedFrame {
    FramePacket frame;
    std::vector<Detection2D> detections;
    bool transform_available;
    double dx_px;
    double dy_px;
    double rotation_rad;
    double confidence;
};

class CameraStabilizer {
public:
    virtual ~CameraStabilizer() = default;
    virtual StabilizedFrame stabilize(
        const FramePacket& frame,
        const std::vector<Detection2D>& detections) = 0;
};
```

Current provider:

```text
NullCameraStabilizer
```

`NullCameraStabilizer` is a pass-through implementation. It copies the input frame and detections unchanged and reports `transform_available = false`.

Future providers may use optical flow, ORB/SIFT/SuperPoint-like features, IMU hints, homography estimation, static landmarks, or world-model landmarks.

### 5.4 EgoState

```cpp
struct EgoState {
    TimePoint timestamp;
    Pose3 local_T_body;
    Vec3 velocity_local;
    Vec3 angular_velocity_body;
    Covariance6 covariance;
    MapFrameId map_frame_id;
};
```

Initial ego-state providers:

```text
StaticEgoStateProvider
NoTelemetryEgoProvider
AirSimGroundTruthEgoProvider
Px4LocalPositionProvider
```

Future providers:

```text
VioProvider
VisualInertialSlamProvider
Px4EstimatorBridge
RelativeMapLocalizationProvider
```

### 5.5 Observation3D

```cpp
struct Observation3D {
    TrackId track_id;
    TimePoint timestamp;
    Vec3 position_body;
    Vec3 position_local;
    MapFrameId map_frame_id;
    Covariance3 covariance;
    ClassLabel class_label;
    FactionLabel faction;
    float confidence;
};
```

Initial projectors:

```text
FlatGroundProjector
KnownSizeProjector
AirSimDepthProjector
FakeDepthProjector
```

Future projectors:

```text
VioRayProjector
MonocularDepthProjector
StereoProjector
StructureFromMotionProjector
```

---

## 7. World Model Layers

The world model is not one map. It is a layered state system.

```text
WorldModel
├── DynamicAgentLayer
├── ContainerRelationshipLayer
├── TacticalObstacleLayer
├── RelativeMapLayer
├── GlobalFlightMapLayer
├── LandmarkLayer
├── EgoLocalizationLayer
├── AppearanceConditionLayer
└── MemoryLayer
```

Perception modules produce observations. The world model owns fused state.

### 6.1 DynamicAgentLayer

Tracks moving entities such as drones, people, vehicles, animals, and other mission-relevant actors.

```cpp
struct AgentState {
    AgentId agent_id;
    IdentityId identity_id;
    TrackId source_track_id;
    TimePoint last_seen;
    Vec3 position_local;
    Vec3 velocity_local;
    MapFrameId map_frame_id;
    Covariance6 covariance;
    ClassLabel class_label;
    FactionLabel faction;
    AgentLifecycle lifecycle;  // new, active, occluded, contained, stale, retired
    float confidence;
};
```

Start with in-memory motion extrapolation. Later add EKF, ReID, clothing-change robustness, and container-aware identity reasoning.

### 6.2 ContainerRelationshipLayer

Dedalus must understand that cars, boats, houses, garages, and buildings can hide or transport people.

Examples:

```text
person enters car
person exits car wearing different clothes
person enters house and becomes unobservable
person exits another side of the house
person enters boat and moves with the boat
vehicle stops and multiple people emerge
```

```cpp
struct ContainerState {
    AgentId container_id;
    ContainerType type;       // car, boat, house, building, garage, room, unknown
    Pose3 local_T_container;
    Vec3 velocity_local;
    MapFrameId map_frame_id;
    float capacity_estimate;
    float confidence;
};

struct ContainmentEvent {
    EventId event_id;
    TimePoint timestamp;
    ContainmentEventType type; // enter, exit, possible_enter, possible_exit, transfer
    AgentId subject_agent_id;
    AgentId container_agent_id;
    float confidence;
    std::vector<std::string> evidence;
};

struct ContainmentHypothesis {
    IdentityId identity_id;
    AgentId possible_container_id;
    TimePoint entered_at;
    TimePoint last_supported;
    float confidence;
};
```

This layer feeds the identity resolver and preserves plausible identity hypotheses through occlusion and transport.

### 6.3 TacticalObstacleLayer

Short-range, high-urgency obstacle representation for local collision avoidance.

Its question is:

```text
Where should the drone not fly in the next 0.5-5 seconds?
```

Start with cone/frustum-based exclusion zones. Upgrade later to voxel occupancy or SDF.

```cpp
struct ExclusionZone {
    ZoneId zone_id;
    TimePoint timestamp;
    ZoneType type;          // cone, cylinder, box, voxel_cluster
    Pose3 local_T_zone;
    Vec3 dimensions;
    MapFrameId map_frame_id;
    float confidence;
    float inflation_radius_m;
    TimePoint expires_at;
    std::string reason;
};
```

Initial implementations:

```text
ConeExclusionMapper
SimpleInflationPolicy
TacticalObstacleLayer
```

Future implementations:

```text
VoxelExclusionMapper
LocalSdfMapper
VelocityObstacleMapper
```

### 6.4 RelativeMapLayer

Dedalus must support mapping without global location anchoring. A drone may learn a neighborhood, canyon, industrial yard, dock, building cluster, or mountain pass without knowing latitude/longitude.

```cpp
struct MapFrame {
    MapFrameId map_frame_id;
    std::optional<GeoAnchor> geo_anchor;
    Pose3 local_T_anchor;
    float scale_confidence;
    float orientation_confidence;
    TimePoint created_at;
    TimePoint last_used;
};

struct RelativeMapReference {
    MapFrameId map_frame_id;
    Vec3 position_in_map;
    Covariance3 covariance;
};
```

Rules:

```text
1. A relative map is valid even without GPS.
2. All observations should carry a MapFrameId when possible.
3. Maps may be merged when landmarks and geometry align.
4. A relative map may later receive a GeoAnchor without rewriting its internal structure.
5. Memory can store relative maps and retrieve them by visual/structural signature.
```

### 6.5 GlobalFlightMapLayer

Low-resolution, longer-horizon navigational map used for rough route planning.

The global flight map may be global geographically or only global inside a relative map frame.

Its questions are:

```text
Where can the drone probably fly over the next 50-1000 meters?
What corridors exist between buildings, mountains, trees, or terrain?
What static structures are useful for visual localization?
```

Represent as a pyramid of resolutions:

```text
Level 0: coarse 2.5D height / traversability map
Level 1: cylinders and primitive static obstacle volumes
Level 2: landmark structures with visual feature signatures
Level 3: optional dense local patches near important regions
```

Contracts:

```cpp
struct FlightMapCell {
    CellId cell_id;
    Bounds3 bounds;
    MapFrameId map_frame_id;
    float min_safe_altitude_m;
    float max_safe_altitude_m;
    float occupancy_probability;
    float traversability_score;
    float confidence;
};

struct StaticStructure {
    StructureId structure_id;
    StructureType type;     // building, tree_cluster, road, river, ridge, tower
    Primitive3D primitive;  // cylinder, box, mesh_proxy, polyline, height_patch
    MapFrameId map_frame_id;
    FeatureSignature signature;
    AppearanceConditionSignature condition_signature;
    float confidence;
    TimePoint first_seen;
    TimePoint last_confirmed;
};

struct FlightCorridor {
    CorridorId corridor_id;
    std::vector<Vec3> centerline;
    MapFrameId map_frame_id;
    float radius_m;
    float min_altitude_m;
    float max_altitude_m;
    float confidence;
};
```

Initial implementations:

```text
MultiResolutionHeightMap
PrimitiveStaticMapper
FlightCorridorExtractor
```

### 6.6 LandmarkLayer and EgoLocalizationLayer

The landmark layer stores visual/spatial anchors useful for map-relative localization:

```text
building corners
road curves
river edges
tree clusters
mountain ridges
towers
unique roof outlines
```

Landmarks should include geometry plus feature signatures. The localization layer can later match current visual features against landmarks to reduce drift in GPS-denied flight, retrieve relative maps, merge maps, and relocalize after drift.

### 6.7 AppearanceConditionLayer

Dedalus should explicitly model appearance changes caused by illumination, season, time of day, weather, and sensor mode.

Examples:

```text
day vs night
sun angle and shadows
overcast vs bright sunlight
seasonal foliage changes
wet roads vs dry roads
snow or dust
visible light vs infrared
headlights or artificial lighting
```

```cpp
struct AppearanceCondition {
    TimePoint timestamp;
    LightingMode lighting_mode;   // day, night, dawn, dusk, artificial, unknown
    WeatherMode weather_mode;     // clear, rain, fog, dust, snow, unknown
    SeasonMode season_mode;       // spring, summer, fall, winter, unknown
    SensorMode sensor_mode;       // rgb, mono, ir, thermal, unknown
    float confidence;
};

struct AppearanceConditionSignature {
    FeatureVector invariant_features;
    FeatureVector condition_specific_features;
    AppearanceCondition observed_condition;
    float invariance_confidence;
};
```

Memory should store condition-specific signatures when useful, but prefer condition-invariant features for identity, landmark matching, and map retrieval.

### 6.8 MemoryLayer

The memory layer allows Dedalus to become familiar with an area.

Separate:

```text
Actual World Model
    live, short-lived, sensor-driven

Working Memory
    current mission/session, minutes to hours

Persistent Memory
    saved across missions, area familiarity

Prior Map
    imported map, satellite map, survey map, or human-provided map
```

Planning should read an effective view:

```cpp
struct EffectiveWorldView {
    WorldSnapshot actual;
    WorldMemorySnapshot memory;
    std::vector<MapConflict> conflicts;
    std::vector<UncertainRegion> uncertain_regions;
};
```

Rules:

```text
1. Fresh confident actual observations override memory.
2. If actual is missing, memory may be used with uncertainty inflation.
3. If actual contradicts memory once, mark a conflict.
4. If actual repeatedly contradicts memory under comparable appearance conditions, update or retire memory.
5. If memory has not been confirmed recently, decay confidence.
6. Unknown regions are treated conservatively, not as free space.
7. Relative maps are valid memory artifacts even without global geo anchoring.
8. Container and identity hypotheses decay over time unless supported by new observations.
```

Memory statistics:

```cpp
struct MemoryStats {
    int observations_confirmed;
    int observations_missing;
    int observations_contradicted;
    TimePoint first_seen;
    TimePoint last_seen;
    TimePoint last_confirmed;
    float persistence_score;
    float confidence;
};
```

Memory states:

```text
Candidate      seen once or twice
Confirmed      observed repeatedly in one mission
Persistent     observed across missions or times
Conflicted     current actual observations disagree with memory
Retired        repeatedly contradicted or no longer useful
```

---

## 8. WorldSnapshot Contract

The world model should publish a debug-friendly snapshot early.

```cpp
struct WorldSnapshot {
    TimePoint timestamp;
    EgoState ego;
    MapFrameId active_map_frame_id;

    std::vector<AgentState> agents;
    std::vector<ContainerState> containers;
    std::vector<ContainmentEvent> containment_events;
    std::vector<ExclusionZone> tactical_exclusion_zones;
    std::vector<FlightCorridor> flight_corridors;
    std::vector<StaticStructure> static_structures;
    std::vector<Landmark> landmarks;
    std::vector<UncertainRegion> uncertain_regions;
    std::vector<MapFrame> map_frames;
    AppearanceCondition appearance_condition;
};
```

Early apps should be able to emit JSON snapshots for tests and visualization:

```json
{
  "timestamp_ns": 123456789,
  "active_map_frame_id": "map_local_0001",
  "appearance_condition": {
    "lighting_mode": "day",
    "weather_mode": "clear"
  },
  "ego": {
    "position_local": [0.0, 0.0, -12.0],
    "velocity_local": [1.2, 0.0, 0.0]
  },
  "agents": [],
  "tactical_exclusion_zones": [],
  "flight_corridors": [],
  "static_structures": [],
  "landmarks": [],
  "uncertain_regions": [],
  "containers": [],
  "containment_events": [],
  "map_frames": []
}
```

---

## 8.1 Visual Validation / Frame Annotation Contract

The runtime now has a post-world-model annotation hook.

Runtime order:

```text
FramePacket
  -> perception pipeline
  -> world model update
  -> WorldSnapshot
  -> FrameAnnotationSink
```

The annotation sink receives:

```cpp
struct AnnotationContext {
    FramePacket frame;
    PerceptionPipelineOutput perception;
    WorldSnapshot world_snapshot;
};
```

Current CI-safe provider:

```text
NullFrameAnnotationSink
```

Dependency-light visual validation provider:

```text
PpmFrameAnnotationSink
```

Future placeholder provider:

```text
Mp4FrameAnnotationSink
```

Config shape:

```yaml
frame_annotator: null

# dependency-light image sequence:
frame_annotator: ppm_sequence
annotation_output_path: out/annotations
annotation_output_fps: 5

# future optional video export:
frame_annotator: mp4
annotation_output_path: out/annotated.mp4
annotation_output_fps: 5
```

The PPM provider is implemented and writes annotated RGB frames plus manifest.txt using raw pixel operations. It is intended as the first real visual validation artifact path because it keeps normal unit tests dependency-free.

The MP4 provider currently throws a clear not-implemented error and must remain optional. Do not add OpenCV, FFmpeg, or GStreamer as mandatory dependencies for unit tests. A future MP4 implementation should render world-model state onto processed frames and preserve raw input timing semantics so raw and annotated video can be watched side-by-side with the same mission duration.

---

## 8.2 Pipeline Timing / Profiling Contract

The core runtime now has a dependency-free coarse timing harness for Milestone 2 validation.

Config:

```yaml
pipeline_timing_enabled: false
pipeline_timing_output_path: out/profile/pipeline_profile.jsonl
```

The default is off. When enabled, `dedalus_replay_recording` creates a JSONL timing writer and passes it into `CoreStackRunner`.

The profiler writes one JSON object per processed frame:

```json
{"frame_id":"frame_0001","timestamp_ns":123456789,"total_us":962,"stages":{"frame_source.next_frame":437,"ego_provider.estimate":0,"perception_pipeline.process":497,"world_model.update_ego":0,"world_model.update_appearance":0,"world_model.ingest":23,"world_model.snapshot":5,"frame_annotator.annotate":0}}
```

Current stages:

```text
frame_source.next_frame
ego_provider.estimate
perception_pipeline.process
world_model.update_ego
world_model.update_appearance   # emitted when appearance metadata exists
world_model.ingest
world_model.snapshot
frame_annotator.annotate
```

This should remain dependency-free. Do not add Tracy, perfetto, OpenCV, FFmpeg, GStreamer, or platform profilers to the core timing path. Rich profiling can be added later as optional tooling.

---

## 8.3 Replay Artifact Validation Contract

Milestone 2.10 adds a CI-safe replay/capture artifact validator:

```text
scripts/validate-replay-artifacts.py
```

It validates the consistency of:

```text
snapshot_manifest.txt
snapshot_*.json
annotation manifest.txt, when annotation output is present
annotated PPM frames, when annotation output is present
pipeline_profile.jsonl, when timing output is present
```

Typical recorded-frame validation:

```bash
python3 scripts/validate-replay-artifacts.py \
  --snapshot-dir out/recorded_ppm_validation/snapshots \
  --annotation-dir out/recorded_ppm_validation/annotations \
  --profile-jsonl out/recorded_profile/pipeline_profile.jsonl \
  --expect-frames 1 \
  --expect-map-frame map_recorded_ci_0001 \
  --timestamp-soft-threshold-ms 0 \
  --require-agent \
  --require-world-keys
```

Typical AirSim validation:

```bash
python3 scripts/validate-replay-artifacts.py \
  --snapshot-dir out/airsim_binary_ppm_profile_validation/snapshots \
  --annotation-dir out/airsim_binary_ppm_profile_validation/annotations \
  --profile-jsonl out/airsim_binary_ppm_profile_validation/pipeline_profile.jsonl \
  --expect-frames 10 \
  --timestamp-soft-threshold-ms 500 \
  --require-agent \
  --require-world-keys
```

Exact timestamp equality is required by default. Use `--timestamp-soft-threshold-ms` for AirSim runs because frame timestamps currently come from the image bridge while snapshot/ego timestamps come from a separate per-sample ego bridge.

This closes the next validation gap before optional MP4 export, shared-memory transport, or Milestone 3 tactical obstacle work.

---

## 9. IPC Strategy

Production IPC target:

```text
Eclipse iceoryx
```

Early implementation should also support:

```text
InProcessBus
```

Do not make iceoryx mandatory for all tests. Unit tests and simple integration tests should run without RouDi or shared memory setup.

Canonical message families:

```text
FramePacket
Detection2DArray
Track2DArray
IdentityHypothesisArray
Observation3DArray
WorldSnapshot
EffectiveWorldView
ControlIntent later
```

---

## 10. Runtime Composition

Modules should be selected by config.

Example placeholder config:

```yaml
runtime:
  rate_hz: 30
  mode: in_process

sensors:
  frame_source:
    type: airsim
    camera: front_center
  ego_state:
    type: airsim_ground_truth

perception:
  detector:
    type: scripted
  camera_stabilizer:
    type: null
  tracker:
    type: iou
  identity_resolver:
    type: appearance_only
  appearance_condition:
    type: simple_metadata
  projector:
    type: airsim_depth

world_model:
  dynamic_agents:
    type: in_memory
  containers:
    type: simple_events
  tactical_obstacles:
    type: cone_exclusion
  relative_map:
    type: local_frame
  global_flight_map:
    type: multires_height_map
  memory:
    type: disabled

visualization:
  frame_annotator:
    type: null
    output_path: out/annotated.mp4
    output_fps: 5
```

Future production-style config:

```yaml
runtime:
  rate_hz: 30
  mode: iceoryx

sensors:
  frame_source:
    type: mipi_csi
  ego_state:
    type: vio

perception:
  detector:
    type: yolo_tensorrt
    engine_path: models/detectors/yolo11_int8.engine
  camera_stabilizer:
    type: feature_homography
  tracker:
    type: reid_kalman
  identity_resolver:
    type: container_aware
  appearance_condition:
    type: learned_condition_estimator
  projector:
    type: vio_ray_projector

world_model:
  dynamic_agents:
    type: ekf
  containers:
    type: probabilistic_containment
  tactical_obstacles:
    type: voxel_sdf
  relative_map:
    type: visual_structural_map
  global_flight_map:
    type: multires_primitives
  memory:
    type: persistent
    path: data/world_memory

visualization:
  frame_annotator:
    type: mp4
    output_path: out/annotated.mp4
    output_fps: 10
```

---

## 11. Implementation Roadmap

### Milestone 1: Core Contracts and In-Process Pipeline ✅ Complete

All contracts, placeholder modules, tactical/map placeholders, unit tests, and CI smoke gates are implemented. See §23 for current implementation state.

Built contracts:

```text
FramePacket / ImageView / CameraIntrinsics
Detection2D / Track2D / IdentityHypothesis / Observation3D
EgoState / AgentState / ContainerState
ExclusionZone / MapFrame
StaticStructure / FlightCorridor / Landmark / UncertainRegion
AppearanceCondition
WorldSnapshot / EffectiveWorldView
```

Built modules:

```text
SyntheticFrameSource
ScriptedDetector
SimpleCentroidTracker
AppearanceOnlyIdentityResolver
FlatGroundProjector
ConeExclusionMapper
RoughFlightMapBuilder
InMemoryWorldModel
InProcessBus (header-only)
dedalus_core_stack
dedalus_dump_world
```

Goal achieved:

```text
Synthetic frame → PerceptionPipeline → InMemoryWorldModel → WorldSnapshot JSON
All world-model layers represented in snapshot (agents, containers, exclusion zones,
uncertain regions, static structures, flight corridors, landmarks)
Unit tests pass under ctest
CI, staging, and production smoke-validate the JSON contract
```

### Milestone 2: Video and Simulation Input ✅ In Progress

Milestone 2 is the current active milestone. Its purpose is to connect real/simulated frame sources into the core-stack while preserving provider modularity and keeping CI dependency-free.

Completed in Milestone 2 so far:

```text
RecordedFrameSource
Replay snapshot artifact app
AirSim provider boundary
AirSim export bridge
Live AirSim RGB bridge
Live AirSim ego bridge
Persistent JSONL stream bridge
BridgeTransport abstraction
PipeBridgeTransport
SharedMemoryBridgeTransport placeholder
Binary framed RGB bridge
simulation/run.sh core-stack orchestration
simulation/run.sh flight-control/capture alignment
CameraStabilizer provider slot
NullCameraStabilizer
FrameAnnotationSink provider slot
NullFrameAnnotationSink
Mp4FrameAnnotationSink placeholder
```

Current key files:

```text
apps/dedalus_replay_recording.cpp
include/dedalus/simulation/bridge_transport.hpp
src/simulation/bridge_transport.cpp
include/dedalus/simulation/airsim_providers.hpp
src/simulation/airsim_providers.cpp
simulation/airsim-capture-frame.py
simulation/airsim-capture-ego.py
simulation/airsim-stream-frames.py
simulation/airsim-stream-frames-binary.py
tests/fixtures/airsim_bridge_ci_fake.py
tests/fixtures/airsim_ego_bridge_ci_fake.py
tests/fixtures/airsim_stream_bridge_ci_fake.py
tests/fixtures/airsim_binary_bridge_ci_fake.py
config/core_stack_airsim_binary_rgb_ego.yaml
config/core_stack_airsim_binary_ci.yaml
docs/binary_frame_bridge_protocol.md
docs/bridge_transport_plugins.md
docs/perception_stabilization_annotation.md
```

Current provider names:

```text
frame_source:
  synthetic
  video_only
  recorded_frames
  airsim

ego_provider:
  frame_hint
  no_telemetry
  airsim

detector:
  scripted
  airsim_ground_truth

camera_stabilizer:
  null

tracker:
  simple_centroid

identity_resolver:
  appearance_only

projector:
  flat_ground
  airsim_depth

world_model:
  in_memory

frame_annotator:
  null
  ppm_sequence
  mp4
```

Current important limitation:

```text
AirSimDepthProjector is still an explicit unavailable integration provider.
AirSimGroundTruthDetector is still an explicit unavailable integration provider.
Mp4FrameAnnotationSink is still an explicit unavailable/placeholder provider.
SharedMemoryBridgeTransport is still an explicit unavailable/placeholder transport.
```

Current visual annotation output:

```text
PpmFrameAnnotationSink is implemented and registered as frame_annotator: ppm_sequence.
It writes:

out/annotations/frame_000001.ppm
out/annotations/frame_000002.ppm
...
out/annotations/manifest.txt

The manifest records frame index, frame ID, timestamp, path, and configured output FPS.
The first overlay slice draws detection boxes, track boxes, track IDs, class labels,
frame ID, timestamp, active map frame, ego map frame, and simple debug markers for
tactical exclusion zones, flight corridors, and landmarks.
```

Current preferred AirSim frame processing paths:

```text
1) RGB-only binary stream + separate ego provider:
   AirSim / Colosseum
     -> simulation/airsim-stream-frames-binary.py
     -> PipeBridgeTransport
     -> AirSimFrameSource
     -> FramePacket
     -> AirSimEgoStateProvider
     -> PerceptionPipeline
     -> InMemoryWorldModel
     -> FrameAnnotationSink
     -> WorldSnapshot JSON

2) Preferred profiling/optimization path (avoids separate ego RPC from C++):
   AirSim / Colosseum
     -> simulation/airsim-stream-frames-binary.py --include-ego
     -> PipeBridgeTransport
     -> AirSimFrameSource
     -> FramePacket::ego_hint
     -> FrameHintEgoProvider
     -> PerceptionPipeline
     -> InMemoryWorldModel
     -> FrameAnnotationSink
     -> WorldSnapshot JSON
```

Preferred command:

```bash
cd ~/dedalus/simulation

./run.sh AirSimNH \
  --with-flight-control \
  --with-core-stack \
  --core-sampling-fps 5 \
  --core-max-frames 0
```

This launches:

```text
tmux session: dedalus-sim
  main window: Colosseum/AirSim
  px4 window: PX4 SITL
  flight-control window: test-flight.py
  core-stack window: dedalus_replay_recording
```

Milestone 2 remaining recommended work:

```text
1. Run full ctest and fix any compile/test fallout from the ppm_sequence provider.
2. Validate ppm_sequence against recorded_frames replay artifacts.
3. Validate ppm_sequence against AirSim binary RGB capture artifacts.
4. Add optional MP4 export or image-sequence-to-video conversion without making OpenCV/FFmpeg/GStreamer mandatory in unit tests.
5. Add timestamp-driven duplication/drop policy for encoded video output.
6. Make ego telemetry persistent or co-streamed with frame data.
7. Implement AirSimDepthProjector bridge/backend.
8. Implement AirSimGroundTruthDetector bridge/backend.
9. Add optional shared-memory ring-buffer transport after binary pipe protocol stabilizes.
10. Add a real camera/media provider, using the same bridge/source-neutral contracts.
```

### Milestone 3: Tactical Obstacle Layer

Add:

```text
ConeExclusionMapper
SimpleInflationPolicy
TacticalObstacleLayer
```

Goal:

```text
Generate short-range tactical exclusion zones from uncertain obstacles and dynamic agents.
```

### Milestone 4: Relative and Rough Global Flight Maps

Add:

```text
RelativeMapLayer
MapFrame management
MultiResolutionHeightMap
PrimitiveStaticMapper
FlightCorridorExtractor
LandmarkLayer
```

Goal:

```text
Build low-resolution traversability and candidate flight corridors in a relative map frame, even without global geo anchoring.
```

### Milestone 5: Container-Aware Identity

Add:

```text
ContainerRelationshipLayer
ContainmentEvent detector
ContainerAwareIdentityResolver
IdentityHypothesis history
```

Goal:

```text
Preserve identity hypotheses when people enter/exit cars, boats, houses, or buildings, including after clothing changes.
```

### Milestone 6: Memory Layer

Add:

```text
PersistentMapStore
RelativeMapStore
ActualVsMemoryFusion
MapConflictDetector
MemoryConfidenceDecay
AppearanceCondition-aware memory comparison
```

Goal:

```text
Load remembered relative or anchored map state, use it when actual observations are incomplete, and update memory only after repeated confirmation under comparable conditions.
```

### Milestone 7: Production Perception Upgrades

Add:

```text
YoloOnnxDetector
YoloTensorRtDetector
KalmanTracker2D
ReIdTracker
ClothingChangeRobustIdentityResolver
VioRayProjector
MonocularDepthProjector
LightingRobustEmbeddings
```

Goal:

```text
Replace placeholders without changing downstream world-model contracts.
```

---

## 12. Simulation Stack

The current simulation environment uses:

```text
Colosseum / AirSim fork
PX4 SITL
AirSim Python API
NICE DCV for rendering
tmux for process orchestration
```

The important files are:

```text
simulation/setup.sh       installs/builds dependencies
simulation/cleanup.sh     resets runtime/build state
simulation/run.sh         launches DCV-bound simulation stack
simulation/stop.sh        stops simulation process/session state
simulation/settings.json  AirSim/Colosseum vehicle configuration
simulation/test-flight.py flight test and trajectory runner
```

The intended workflow is:

```bash
cd ~/dedalus/simulation
./setup.sh --yes
./cleanup.sh --soft --yes
./run.sh AirSimNH
```

For Milestone 2 core-stack integration, the preferred workflow is:

```bash
cd ~/dedalus

cmake -S . -B build-staging \
  -DDEDALUS_BUILD_APPS=ON \
  -DDEDALUS_BUILD_TESTS=ON

cmake --build build-staging -j$(nproc)

cd simulation

./run.sh AirSimNH \
  --with-flight-control \
  --with-core-stack \
  --core-sampling-fps 5 \
  --core-max-frames 0
```

This starts simulator, PX4, velocity-control, and core-stack capture in one tmux session.

Manual flight test remains available:

```bash
cd ~/dedalus/simulation
source ~/dedalus/venv/bin/activate
python test-flight.py
```

The simulation should now be used as an integration harness for the core-stack. `run.sh` may orchestrate `test-flight.py` and `dedalus_replay_recording` together, but the core-stack itself should remain decoupled from `test-flight.py`. The coupling should remain at orchestration/config level, not inside core perception/world-model contracts.

---

## 13. Setup / Cleanup / Run Responsibilities

### `setup.sh`

`setup.sh` should be the only script responsible for installing or verifying dependencies.

It should:

- verify GPU / NVIDIA runtime
- configure NICE DCV
- build/install Eclipse iceoryx
- clone/stage PX4
- create and update `~/dedalus/venv`
- install Python dependencies:

```text
airsim
numpy
msgpack-rpc-python
pymavlink
pyserial
kconfiglib
```

Important PX4 dependency note:

```text
kconfiglib is required because PX4 imports menuconfig/kconfiglib during build.
```

Do **not** run:

```bash
make px4_sitl none_iris
```

inside `setup.sh` as a "build verification" step. That target starts PX4 and will block waiting for the simulator TCP server on port `4560`.

Use build-only targets in setup, for example:

```bash
make px4_sitl
```

### `cleanup.sh`

`cleanup.sh` should reset runtime state before restarting simulation.

Use:

```bash
./cleanup.sh --soft --yes
```

for normal restarts.

Use:

```bash
./cleanup.sh --px4 --yes
```

when PX4 build/source state must be forced to rebuild.

Use:

```bash
./cleanup.sh --hard --yes
```

only for deeper reset.

Cleanup should kill stale:

```text
AirSimNH / Blocks / Linux-Shipping
px4
iox-roudi
dedalus-sim tmux session
stale iceoryx shared memory
```

### `run.sh`

`run.sh` should own process orchestration.

Current working order:

```text
1. Resolve DCV DISPLAY / XAUTHORITY
2. Spawn tmux session if not already inside tmux
3. Start iox-roudi
4. Copy simulation/settings.json to ~/Documents/AirSim/settings.json
5. Verify Python/PX4 dependencies
6. Launch Colosseum / Unreal / AirSim first
7. Wait for AirSim TCP server on 127.0.0.1:4560
8. Launch PX4 SITL in tmux window named px4
9. Optionally launch flight-control tmux window
10. Optionally launch core-stack capture tmux window
11. Keep Unreal process in foreground of main tmux window
```

The critical point is:

```text
PX4 waits for simulator TCP 4560, so AirSim/Colosseum must be started first.
```

Current core-stack orchestration options:

```bash
--with-core-stack
--no-core-stack
--with-flight-control
--no-flight-control
--flight-control px4
--flight-trajectory trajectories/circle_figure8.json
--flight-safe-height-m 8
--control-start-delay-s 10
--core-build-dir ../build-staging
--core-config ../config/core_stack_airsim_binary_rgb_ego.yaml
--core-output-dir ../out/airsim_run_<timestamp>
--core-sampling-fps 5
--core-max-frames 0
```

Timing rule:

```text
PX4 window starts
sleep control-start-delay-s
flight-control starts
core-stack capture starts at the same scheduled time
```

`--core-max-frames 0` semantics:

```text
With --with-flight-control:
  derive max frames from trajectory duration × core-sampling-fps.

Without --with-flight-control:
  run until stream ends.
```

`run.sh` generates:

```text
out/airsim_run_<timestamp>/core_stack_runtime.yaml
```

and rewrites bridge command `--rate-hz` to match `--core-sampling-fps`.

Expected healthy AirSim log:

```text
Waiting for TCP connection on port 4560, local IP 127.0.0.1
Connected to SITL over TCP.
Connecting to PX4 Control UDP port 14600, local IP 127.0.0.1, remote IP 127.0.0.1 ...
received first heartbeat
Ground control connected over UDP.
```

Expected healthy PX4 log:

```text
INFO  [simulator_mavlink] using TCP on remote host localhost port 4560
INFO  [simulator_mavlink] Simulator connected on TCP port 4560.
INFO  [tone_alarm] home set
INFO  [commander] Ready for takeoff!
```

---

## 14. AirSim / PX4 Settings

The working AirSim PX4 mode is TCP simulator link, not UDP.

`simulation/settings.json` should use:

```json
{
    "SettingsVersion": 1.2,
    "SimMode": "Multirotor",
    "LogMessagesVisible": true,
    "ShowDashboard": true,
    "ViewMode": "FlyWithMe",
    "OriginGeopoint": {
        "Latitude": 47.641468,
        "Longitude": -122.140165,
        "Altitude": 121
    },
    "ClockSpeed": 1.0,
    "Vehicles": {
        "PX4": {
            "VehicleType": "PX4Multirotor",
            "UseSerial": false,
            "UseTcp": true,
            "LockStep": true,
            "TcpPort": 4560,
            "ControlIp": "127.0.0.1",
            "ControlPort": 14600,
            "Parameters": {
                "COM_ARM_WO_GPS": 1,
                "NAV_RCL_ACT": 0,
                "NAV_DLL_ACT": 0,
                "COM_OBL_ACT": 1
            }
        }
    }
}
```

Historical note:

```text
UDP mode created partial/invalid states for this environment.
PX4 was waiting for simulator TCP 4560 while AirSim was configured for UDP.
That caused HIL missing warnings and arming/control failures.
```

---

## 15. Known AirSim / Colosseum PX4 Limitation

The AirSim/Colosseum RPC call:

```python
client.armDisarm(True, vehicle_name="PX4")
```

currently fails with:

```text
rpclib: function 'armDisarm' (called with 2 arg(s)) threw an exception.
The exception is not derived from std::exception.
No further information available.
```

This is isolated to the AirSim/Colosseum PX4 arm/disarm RPC bridge.

Evidence:

```text
client.enableApiControl(True) works
AirSim GPS sensor is valid
PX4 TCP simulator link works
PX4 receives heartbeat
PX4 reports Ready for takeoff
pxh> commander arm spins props
pymavlink MAV_CMD_COMPONENT_ARM_DISARM returns MAV_RESULT_ACCEPTED
```

Therefore:

```text
Do not depend on AirSim armDisarm() for PX4 in this environment.
```

Confirmed working path:

```text
PX4 shell commander arm
PX4 shell commander takeoff
AirSim API velocity-vector trajectory playback
PX4 shell commander land
PX4 shell commander disarm
```

The Python flight test currently exposes this via:

```bash
python test-flight.py --control px4
```

and the default:

```bash
python test-flight.py
```

should use:

```text
--control auto
```

where `auto` currently prefers the confirmed PX4 shell path.

---

## 16. PX4 Shell Access

PX4 runs in a tmux window named:

```text
dedalus-sim:px4
```

Attach:

```bash
tmux attach -t dedalus-sim
```

Select window:

```text
Ctrl-b w
```

The PX4 shell prompt is:

```text
pxh>
```

Important terminal note:

```text
PX4 shell input may not visibly echo in tmux/DCV, but commands still execute after pressing Enter.
```

Alternative from another SSH terminal:

```bash
tmux send-keys -t dedalus-sim:px4 'commander status' C-m
tmux send-keys -t dedalus-sim:px4 'mavlink status' C-m
tmux send-keys -t dedalus-sim:px4 'commander arm' C-m
tmux send-keys -t dedalus-sim:px4 'commander takeoff' C-m
tmux send-keys -t dedalus-sim:px4 'commander land' C-m
```

Useful PX4 shell commands:

```bash
commander status
mavlink status
listener sensor_gps
listener vehicle_gps_position
listener vehicle_local_position
commander arm
commander takeoff
commander land
commander disarm
```

---

## 17. `test-flight.py` Current Behavior

`simulation/test-flight.py` is a flight-test harness.

It should:

- connect to AirSim
- detect vehicle from `listVehicles()`
- verify AirSim vehicle state
- verify AirSim GPS sensor
- enable API control
- optionally measure timestamp-derived API FPS
- run a selected control strategy
- play velocity-vector trajectories from JSON when using the PX4 shell path

Control modes:

```bash
python test-flight.py --control auto
python test-flight.py --control px4
python test-flight.py --control mavlink
python test-flight.py --control airsim
```

Current status:

```text
--control auto     confirmed working; prefers PX4 shell path
--control px4      confirmed working; arm/takeoff/land via pxh>, trajectory via AirSim velocity commands
--control mavlink  experimental; arms successfully, NAV_TAKEOFF ACKs, but may not produce physical climb
--control airsim   expected to fail due to broken AirSim armDisarm() RPC
```

MAVLink status:

```text
pymavlink can receive heartbeat
pymavlink arm command gets MAV_RESULT_ACCEPTED
NAV_TAKEOFF can get MAV_RESULT_ACCEPTED
but no real climb may occur
```

So `--control mavlink` should verify real local-z movement and report failure if takeoff is ACKed but no climb is observed.

---

## 18. Trajectory System

The preferred mission-body interface is a JSON trajectory file that defines velocity-vector commands.

Default intended trajectory file:

```text
simulation/trajectories/circle_figure8.json
```

Run:

```bash
python test-flight.py --control px4 --trajectory trajectories/circle_figure8.json
```

The pattern is:

```text
PX4 shell: commander arm
PX4 shell: commander takeoff
AirSim: stream velocity vectors from JSON trajectory
PX4 shell: commander land
PX4 shell: commander disarm
```

Supported trajectory segment types:

```text
hold
circle_velocity
figure8_velocity
velocity_keyframes
```

The trajectory player should print live status on one terminal line where appropriate, for example:

```text
t=  9.9/ 10.0s v=(+0.00,+0.00,+0.00) m/s
```

and avoid spamming repeated lines for MAVLink climb verification.

---

## 19. Known Debugging Milestones

These are important facts future LLMs should not rediscover from scratch:

1. SimpleFlight works with AirSim RPC:

     ```text
     enableApiControl works
     armDisarm works
     props spin
     ```

2. PX4 with wrong settings can produce a partial AirSim vehicle shell:

     ```text
     listVehicles() returns ['PX4']
     getMultirotorState() returns timestamp
     but HIL/GPS/control path is broken
     ```

3. The real PX4 connection fix was:

     ```text
     UseTcp=true
     TcpPort=4560
     start AirSim first
     wait for TCP 4560
     then start PX4
     ```

4. The AirSim `armDisarm()` crash persisted even after PX4 was healthy.

5. Manual PX4 shell control is currently the only confirmed complete arm/takeoff/land path.

6. MAVLink raw command path can arm but may not cause takeoff motion.

7. AirSim velocity-vector commands are used for trajectory playback after PX4 shell takeoff.

---

## 20. Common Commands

Start clean:

```bash
cd ~/dedalus/simulation
./cleanup.sh --soft --yes
./run.sh
```

Force PX4 rebuild:

```bash
cd ~/dedalus/simulation
./cleanup.sh --px4 --yes
./setup.sh --yes
./run.sh
```

Run default flight:

```bash
cd ~/dedalus/simulation
source ~/dedalus/venv/bin/activate
python test-flight.py
```

Run explicit PX4 shell control:

```bash
python test-flight.py --control px4
```

Run circle + figure-eight:

```bash
python test-flight.py --control px4 --trajectory trajectories/circle_figure8.json
```

Check sockets:

```bash
ss -ltnp | grep 4560
ss -tanp | grep 4560
ss -lunp | grep -E '145|146|4560'
```

Attach PX4 shell:

```bash
tmux attach -t dedalus-sim
```

Send PX4 shell commands from SSH:

```bash
tmux send-keys -t dedalus-sim:px4 'commander status' C-m
```

---

## 21. Style / Contribution Guidance for LLMs

When modifying this repo:

- Prefer targeted patches over large rewrites.
- Keep all setup state inside `setup.sh`.
- Keep all reset behavior inside `cleanup.sh`.
- Keep orchestration inside `run.sh`.
- Avoid one-off shell instructions unless also captured in scripts.
- Do not assume AirSim PX4 `armDisarm()` works.
- Do not silently report flight success unless motion is actually observed.
- Preserve `--control auto` as a safe default.
- Keep trajectory behavior editable through JSON rather than hardcoding mission paths in Python.
- For the core-stack, add interfaces first and placeholder implementations second.
- Do not make TensorRT, AirSim, PX4, or iceoryx mandatory for unit tests.
- Keep behavior/control downstream of `WorldSnapshot` / `EffectiveWorldView`.
- Do not let perception modules directly own world-model state.
- Do not let world-model modules directly command PX4.

When debugging simulation:

- First check whether PX4 and AirSim are connected:

    ```text
    AirSim log: Connected to SITL over TCP.
    PX4 log: Simulator connected on TCP port 4560.
    PX4 log: Ready for takeoff!
    ```

- If AirSim says:

    ```text
    not receiving any messages from HIL
    ```

    then PX4 is not connected to the simulator.

- If PX4 says:

    ```text
    Waiting for simulator to accept connection on TCP port 4560
    ```

    then AirSim did not open TCP `4560`, or PX4 started too early.

- If `client.armDisarm()` throws:

    ```text
    rpclib ... exception is not derived from std::exception
    ```

    do not keep debugging PX4 health. Use `--control px4` and treat it as the known Colosseum/AirSim RPC bridge bug.

When implementing the core-stack:

- First success condition is a JSON `WorldSnapshot`, not flight autonomy.
- Build `InProcessBus` before `IceoryxBus`.
- Build `ScriptedDetector` before TensorRT.
- Build `ConeExclusionMapper` before voxel/SDF.
- Build `RelativeMapLayer` before geo-anchored global maps.
- Build `MultiResolutionHeightMap` before dense SLAM.
- Build `AppearanceConditionLayer` before treating map changes as persistent changes.
- Build `DisabledMemoryLayer` / in-memory memory before persistent storage.
- Keep C++ contracts stable and small.

---

## 22. CI/CD Structure and Decisions

Three GitHub Actions workflows live in `.github/workflows/`. They are the sole automated validation gate at this stage. All three run on `ubuntu-22.04`.

### Workflow Files and Triggers

| File | Trigger | Build type |
|---|---|---|
| `ci.yml` | PR targeting `staging` or `main`; `workflow_dispatch` | Debug |
| `staging.yml` | Push to `staging` branch; push to `main` branch; `workflow_dispatch` | Debug |
| `production.yml` | Push of `v*.*.*` tag; `workflow_dispatch` | Release (`-DCMAKE_BUILD_TYPE=Release`) |

**Rationale for this trigger model:**

- CI is PR-only. Validation fires on contributions before they land, not after.
- Staging fires on `staging` branch pushes (normal integration path) **and** on `main` pushes (shortcut-to-production path without publishing a version). This ensures `main` is never dark after a direct merge.
- Production is tag-triggered, not merge-triggered. A push to `main` alone does not promote a build to production. A human must cut a `v*.*.*` tag explicitly. This prevents accidental production promotions during routine iteration.

### Branch Strategy

```text
feat/* or fix/*  →  PR targeting staging or main  →  CI workflow
staging push     →  Staging workflow
main push        →  Staging workflow (pre-release gate)
v*.*.* tag push  →  Production workflow
```

### Smoke Validation Contract

Every workflow builds with `-DDEDALUS_BUILD_APPS=ON -DDEDALUS_BUILD_TESTS=ON`, then runs:

```bash
ctest --test-dir <build-dir> --output-on-failure
```

Then runs `dedalus_core_stack` and asserts the emitted JSON contains:

1. `python3 -m json.tool` exits 0 — output is valid JSON.
2. `"active_map_frame_id": "map_local_0001"` — `WorldSnapshot` carries an active map frame.
3. `"class": "person"` — at least one person agent is present and serialized.
4. `"type": "car"` — at least one car container is present and serialized.
5. `"map_frame_id": "map_local_0001"` — `map_frames` array is populated (guards against silent empty-array serializer bug).
6. `"tactical_exclusion_zones"` — tactical layer is present in snapshot.
7. `"reason": "dynamic_observation_cone"` — exclusion zone carries a reason field.
8. `"uncertain_regions"` — uncertain region layer is present.
9. `"flight_corridors"` — flight corridor layer is present.
10. `"corridor_id": "corridor_forward_0001"` — at least one corridor is serialized.
11. `"static_structures"` — static structure layer is present.
12. `"structure_id": "structure_building_0001"` — at least one structure is serialized.
13. `"landmarks"` — landmark layer is present.
14. `"landmark_id": "landmark_building_corner_0001"` — at least one landmark is serialized.

When new `WorldSnapshot` fields are made mandatory, add `grep -q` assertions to the "Check required fields" step and a corresponding row to the "Write job summary" step in **all three** workflow files.

Results are written to `$GITHUB_STEP_SUMMARY` as a markdown table visible in the GitHub Actions job UI.

### Action Versions

Use `actions/checkout@v6` and `actions/upload-artifact@v6`. These target Node.js 24 natively. Do not downgrade to `@v4` (Node.js 20, deprecated May 2025).

### Cutting a Release

```bash
git tag -a v0.1.0 -m "core-stack spine: synthetic pipeline to WorldSnapshot"
git push origin v0.1.0
# triggers Production workflow
```

---

## 23. Core Stack Implementation State

This section captures what is actually built and in the repo. Use it to avoid assuming modules exist before they do.

### Architecture Spine

```text
SyntheticFrameSource
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

### Public Headers

```text
include/dedalus/core/types.hpp
include/dedalus/sensors/frame_source.hpp
include/dedalus/perception/types.hpp
include/dedalus/perception/perception_pipeline.hpp
include/dedalus/ipc/in_process_bus.hpp
include/dedalus/world_model/world_snapshot.hpp
include/dedalus/world_model/effective_world_view.hpp
include/dedalus/world_model/in_memory_world_model.hpp
include/dedalus/world_model/tactical_obstacle_mapper.hpp
include/dedalus/world_model/rough_flight_map_builder.hpp
```

### Value Contracts

```text
FramePacket / ImageView / CameraIntrinsics
Detection2D / Track2D / IdentityHypothesis / Observation3D
EgoState / AgentState / ContainerState
ExclusionZone / MapFrame
StaticStructure / FlightCorridor / Landmark / UncertainRegion
WorldSnapshot / EffectiveWorldView
```

### Placeholder Modules

All deliberately deterministic. They lock down interfaces, JSON shape, CI gates, and module boundaries before real perception/mapping/memory adapters are added.

```text
SyntheticFrameSource
ScriptedDetector
SimpleCentroidTracker
AppearanceOnlyIdentityResolver
FlatGroundProjector
ConeExclusionMapper
RoughFlightMapBuilder
InMemoryWorldModel
InProcessBus
```

### Tests

Tests are named architecturally, not after milestones.

```text
tests/unit/test_world_snapshot_json.cpp
  — guards the map_frames serialization bug (empty-array regression)

tests/unit/test_perception_world_model_flow.cpp
  — validates the full synthetic perception/world-model flow
  — asserts: one detection, one track, one identity hypothesis,
    one 3D observation, one agent, one tactical exclusion zone,
    one uncertain region, one static structure, one flight corridor,
    one landmark, EffectiveWorldView contains actual state and uncertainty
```

### Not Yet Implemented

Do not assume these exist:

```text
RecordedVideoFrameSource / MpegCameraSource
AirSimFrameSource / AirSimEgoStateProvider
AirSimDepthProjector / AirSimGroundTruthDetector
NullDetector
IouTracker / KalmanTracker2D
YoloOnnxDetector / YoloTensorRtDetector
real ReID
container-aware identity resolver
persistent memory layer / relative map store
real flight corridor extraction
voxel/SDF obstacle mapping
behavior tree / control output
iceoryx runtime transport
```

### Next Architectural Step

Simulation/video ingestion — not behavior/control.

```text
RecordedVideoFrameSource or AirSimFrameSource
  -> same PerceptionPipeline contract
  -> same InMemoryWorldModel contract
  -> WorldSnapshot JSON parity with synthetic source
```

Keep real AirSim/PX4 dependencies out of unit tests. Simulation adapters are integration-path modules, not required for core library tests.
