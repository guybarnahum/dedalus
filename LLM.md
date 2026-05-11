# Project Dedalus - LLM Context & Operational State

> Purpose: This file is the project handoff document for LLMs. It should let an LLM understand the architecture, repo conventions, current simulation state, known bugs, and safe modification patterns without rediscovering the debugging history.

> Current implementation note: for the latest concrete core-stack implementation state, see `docs/core_stack_current_state.md`. That file tracks the buildable contracts, placeholder modules, tests, CI smoke contract, and next recommended step.

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
ContainerAwareIdentityResolver
ClothingChangeRobustIdentityResolver
VioRayProjector
VoxelSdfMapper
MultiResolutionFlightMap
RelativeMapStore
PersistentMemoryLayer
LightingRobustEmbeddings
```

Avoid hardcoding AirSim, TensorRT, PX4, YOLO, iceoryx, or a specific camera path into the core contracts.

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

### Milestone 2: Video and Simulation Input

Add:

```text
RecordedVideoFrameSource
MpegCameraSource
AirSimFrameSource
AirSimEgoStateProvider
AirSimDepthProjector or AirSimGroundTruthDetector
SimpleAppearanceConditionEstimator
```

Goal:

```text
Run the perception/world-model stack during an existing Colosseum/PX4 trajectory.
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
./run.sh
```

Then in another terminal:

```bash
cd ~/dedalus/simulation
source ~/dedalus/venv/bin/activate
python test-flight.py
```

The simulation should now be used as an integration harness for the core-stack. Avoid coupling the core-stack directly to `test-flight.py`; instead, add adapters or tools that can observe/consume simulation frames while `test-flight.py` handles the known-good flight motion path.

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
9. Keep Unreal process in foreground of main tmux window
```

The critical point is:

```text
PX4 waits for simulator TCP 4560, so AirSim/Colosseum must be started first.
```

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
