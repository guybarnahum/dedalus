# Architecture White Paper: Project Dedalus

**Document Type:** Technical Strategy & Architecture Review

**Target Audience:** Engineering Leadership, Principal Architects, Autonomy Runtime Contributors

**Subject:** Modular Vision-Centric Perception, Mapping, World-Model, and Control Stack for Edge-Compute UAVs

---

## 1. Executive Summary

Project Dedalus is a vision-centric autonomous drone stack and virtual proving ground for high-speed perception, world modeling, tactical obstacle avoidance, global flight mapping, and bounded flight-control intent generation.

The system is designed around a simple architectural principle:

```text
Every capability is a replaceable module behind a stable contract.
```

The immediate development focus is the **core-stack** from camera input through perception and world-model construction. Flight control already has a working simulation harness through Colosseum/AirSim and PX4 SITL; the next system milestone is to make the drone perceive, track, localize, map, and maintain a useful world state while flying.

Dedalus is not a monolithic autonomy program. It is a modular runtime where simple placeholder implementations can be installed first, then continuously upgraded with better detectors, trackers, depth estimators, VIO systems, obstacle mappers, landmark systems, memory layers, and planners as they become available.

The target production runtime is C++20 on NVIDIA Jetson Orin-class edge compute. Python remains useful for simulation harnesses, dataset tooling, model training, visualization, and offline conversion, but Python should not be placed in the production flight path.

---

## 2. Design Philosophy

Dedalus follows six core design principles.

### 2.1 Modularity at Every Stage

Each stage of the stack should be independently replaceable:

```text
Frame source
Detector
Tracker
Identity resolver
Depth / geometry estimator
3D projector
Tactical obstacle mapper
Global flight-map builder
Landmark extractor
World-memory layer
Planner
Command output
```

The first working implementation may use mock detections, AirSim ground truth, simple geometry, or low-resolution maps. The contracts should not change when those implementations are replaced by TensorRT, learned depth, VIO, SLAM, ReID, EKF, container-aware identity reasoning, lighting-robust embeddings, or more advanced mapping.

### 2.2 Simulation First, Hardware Later

Dedalus follows:

```text
Train Heavy, Simulate Accurately, Fly Light
```

The Colosseum/AirSim + PX4 SITL environment is the proving ground. Before a behavior or perception change reaches physical hardware, it should be executable in simulation. Simulation should not be a separate architecture; the same C++ core-stack should run against simulation adapters and later against hardware adapters.

### 2.3 Edge Runtime Discipline

The production runtime should be C++20-first. It should be deterministic, bounded, testable, and compatible with NVIDIA JetPack, CUDA, TensorRT, and low-latency IPC. Python belongs in the ML factory, scenario tooling, test harnesses, and visualization tools.

### 2.4 World Model as the Source of Truth

Perception modules produce observations. The world model owns fused state.

The world model is not a single map. It is a layered spatial state system that includes:

```text
Dynamic agents
People and object-containment relationships
Tactical obstacle/exclusion zones
Relative and global rough flight maps
Landmarks and feature signatures
Ego-localization support
Appearance-condition models
Actual current observations
Persistent memory of familiar areas
Uncertainty and conflict state
```

Behavior and planning should read world-model products, not raw detector outputs.

### 2.5 Safety Through Conservative Uncertainty

Unknown space is not automatically free space. If the drone cannot see a region and only has memory or weak inference, the system should use lower confidence, larger margins, reduced speed, increased altitude, or require more observation before committing to a route.

### 2.6 Identity Is Not Only Appearance

Dedalus should not assume that visual appearance is stable. People can enter cars, boats, and houses; those containers can hide or transport people; people can leave later with different clothing; and lighting, time of day, weather, and seasons can change the appearance of people, objects, roads, terrain, and buildings.

The world model must therefore reason over identity using a combination of:

```text
appearance
motion continuity
entry/exit events
container relationships
location and timing
behavioral patterns
landmark-relative position
confidence and ambiguity
```

---

## 3. Hardware and Sensor Realities

High-speed autonomous flight imposes strict constraints that software cannot fully compensate for.

* **Production camera path:** The preferred production sensor path is a global-shutter camera over MIPI CSI-2 or GMSL2. Rolling-shutter and high-latency network cameras are unsuitable for high-speed closed-loop flight.
* **Video-only input path:** Dedalus must also support MPEG/MP4/RTSP/GStreamer-style camera feeds without telemetry. This is necessary for replay, dataset testing, external cameras, low-fidelity experiments, and degraded operation. Outputs from video-only mode should carry lower confidence when ego-state is unavailable.
* **Inference rate:** A 5 FPS perception loop is a failure state for high-speed flight. The target runtime should preserve a 30+ FPS perception path where possible.
* **Raw imagery:** Real-time fisheye dewarping is expensive. The preferred production strategy is to train perception models directly on raw distorted imagery and push geometric correction into model/data strategy or downstream projection.
* **Compute tier:** Jetson Orin NX 16GB or AGX Orin are the primary edge-compute targets. Desktop GPUs may be used for development and training but are not the production SWaP target.

---

## 4. System Domains

The runtime is organized into six domains.

```text
sensors/       camera, video stream, IMU, ego-state, PX4/FCU state ingestion
perception/    detection, tracking, identity, features, depth, segmentation, projection
world_model/   dynamic agents, containers, tactical map, relative/global maps, landmarks, appearance, memory
behavior/      mission policy, behavior trees, goal selection
safety/        command constraints, kill switch, manual override, bounded outputs
ipc/           in-process and zero-copy inter-process transport
```

The immediate core-stack focus is:

```text
sensors -> perception -> world_model
```

Behavior and control should remain downstream consumers until the world model is stable.

---

## 5. Perception-to-World-Model Pipeline

The initial vertical slice should prove the dataflow below:

```text
Camera / Video / Simulation Frame
        ↓
FramePacket
        ↓
Detector
        ↓
Detection2D
        ↓
Tracker + Identity Resolver
        ↓
Track2D / IdentityHypothesis
        ↓
Depth / Geometry / Projection
        ↓
Observation3D
        ↓
World Model Builders
        ↓
WorldSnapshot / EffectiveWorldView
```

The perception pipeline should not directly decide how to fly. It should generate typed observations with timestamps, confidence, geometry, identity hypotheses, containment events, lighting condition estimates, and optional feature signatures.

### 5.1 Frame Source Contract

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

Initial implementations:

```text
SyntheticFrameSource
RecordedVideoFrameSource
MpegCameraSource
AirSimFrameSource
```

Future implementations:

```text
MipiCsiCameraSource
GmslCameraSource
NvArgusCameraSource
```

### 5.2 Detection Contract

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

Initial implementations:

```text
NullDetector
ScriptedDetector
AirSimGroundTruthDetector
CpuMockDetector
```

Future implementations:

```text
YoloOnnxDetector
YoloTensorRtDetector
SegmentationDetector
PoseDetector
LightingRobustDetector
```

### 5.3 Tracking and Identity Contract

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

Initial implementations:

```text
SimpleCentroidTracker
IouTracker
AppearanceOnlyIdentityResolver
```

Future implementations:

```text
KalmanTracker2D
DeepSortLikeTracker
OpticalFlowAssistedTracker
ReIdTracker
ContainerAwareIdentityResolver
ClothingChangeRobustIdentityResolver
```

### 5.4 Ego-State Contract

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

Initial implementations:

```text
StaticEgoStateProvider
NoTelemetryEgoProvider
AirSimGroundTruthEgoProvider
Px4LocalPositionProvider
```

Future implementations:

```text
VioProvider
VisualInertialSlamProvider
Px4EstimatorBridge
RelativeMapLocalizationProvider
```

### 5.5 3D Observation Contract

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

Initial implementations:

```text
FlatGroundProjector
KnownSizeProjector
AirSimDepthProjector
FakeDepthProjector
```

Future implementations:

```text
VioRayProjector
MonocularDepthProjector
StereoProjector
StructureFromMotionProjector
```

---

## 6. World Model Architecture

The world model fuses observations into a layered state representation. It should expose both a raw current snapshot and a planning-oriented effective view.

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

### 6.1 Dynamic Agent Layer

The dynamic agent layer tracks moving entities such as drones, people, cars, boats, animals, and other mission-relevant actors.

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

Initial implementation:

```text
InMemoryAgentLayer with simple motion extrapolation
```

Future implementation:

```text
EKF-backed agent fusion with occlusion handling, ReID-assisted persistence, clothing-change robustness, and container-aware identity reasoning
```

### 6.2 Container Relationship Layer

Some objects hide, carry, or transform the observability of other objects. Dedalus should model these as **people containers** or more generally **agent containers**.

Examples:

```text
person enters car
person exits car wearing different clothes
person enters house and becomes unobservable
person exits another side of the house
person enters boat and moves with the boat
vehicle stops and multiple people emerge
```

The goal is not to claim certainty. The goal is to preserve plausible identity hypotheses through occlusion and transport.

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

This layer feeds the identity resolver. If a person enters a vehicle, the system can preserve a hypothesis that the person is now associated with the vehicle. If a person later exits nearby, the system can reconnect identity using timing, location, container trajectory, body shape, gait, and appearance if available.

### 6.3 Tactical Obstacle Layer

The tactical obstacle layer is short-range, high-urgency, and conservative. Its job is not to build a beautiful map. Its job is to answer:

```text
Where should the drone not fly in the next 0.5-5 seconds?
```

Representations may include:

```text
Cone/frustum exclusion zones
Inflated bounding volumes
Voxel occupancy
Signed distance fields
Velocity-obstacle cones
```

Start with cone/frustum-based exclusion zones because they are easy to generate from uncertain monocular data and are conservative enough for early obstacle avoidance.

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

### 6.4 Relative Map Layer

Dedalus must support mapping without global location anchoring. A drone may learn a neighborhood, canyon, industrial yard, dock, building cluster, or mountain pass without knowing its absolute latitude/longitude.

The relative map layer provides a local coordinate frame that can later be anchored globally if GPS, prior maps, or human labeling become available.

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

This lets Dedalus remember a neighborhood even when the system does not know where that neighborhood is globally.

### 6.5 Global Flight Map Layer

The global flight map is a low-resolution, longer-horizon navigational representation. It identifies rough traversability, flight corridors, static structures, and localization anchors.

It should be built as a pyramid of resolutions:

```text
Level 0: coarse 2.5D height / traversability map
Level 1: cylinders and primitive static obstacle volumes
Level 2: landmark structures with visual feature signatures
Level 3: optional dense local patches near important regions
```

The global flight map may be global in the geographic sense or global only within a relative map frame.

The global flight map should answer:

```text
Where can the drone probably fly over the next 50-1000 meters?
What corridors exist between buildings, mountains, trees, or terrain?
What static structures are useful for visual localization?
```

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

### 6.6 Landmark and Localization Layer

The landmark layer stores static visual/spatial anchors that can support ego-localization in GPS-denied conditions.

Examples:

```text
building corners
road curves
river edges
tree clusters
mountain ridges
towers
unique roof outlines
```

Landmarks should include both geometry and feature signatures. The same layer can later support map-relative localization, relocalization after drift, relative-map retrieval, map merging, and shared-map updates.

### 6.7 Appearance Condition Layer

Dedalus should explicitly model the fact that appearance changes with illumination, season, time of day, weather, and sensor mode.

Examples:

```text
day vs night
sun angle and hard shadows
overcast vs bright sunlight
seasonal foliage changes
wet roads vs dry roads
snow or dust
visible light vs infrared
headlights or artificial lighting
```

The goal is to prevent the system from treating every appearance change as an object change or map conflict.

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

The memory layer should store condition-specific signatures when useful, but prefer condition-invariant features for identity, landmark matching, and map retrieval.

### 6.8 Memory Layer

The memory layer allows Dedalus to become familiar with an area over repeated passes or missions.

The world model should distinguish:

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

Planning should consume an effective view:

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

Persistent structures should carry memory statistics:

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

## 7. World Snapshot Contract

The world model should publish a debug-friendly snapshot early, even if production later uses binary zero-copy transport.

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

A JSON debug form should exist for tests, replay, and visualization:

```json
{
  "timestamp_ns": 123456789,
  "active_map_frame_id": "map_local_0001",
  "appearance_condition": {
    "lighting_mode": "day",
    "weather_mode": "clear",
    "season_mode": "unknown"
  },
  "ego": {
    "position_local": [0.0, 0.0, -12.0],
    "velocity_local": [1.2, 0.0, 0.0]
  },
  "agents": [
    {
      "agent_id": "agent_0001",
      "identity_id": "identity_unknown_0001",
      "class": "person",
      "faction": "unknown",
      "position_local": [18.2, -3.1, -10.5],
      "velocity_local": [2.0, 0.4, 0.0],
      "confidence": 0.82,
      "lifecycle": "active"
    }
  ],
  "containers": [],
  "containment_events": [],
  "tactical_exclusion_zones": [],
  "flight_corridors": [],
  "static_structures": [],
  "landmarks": [],
  "map_frames": []
}
```

---

## 8. IPC Strategy

The production IPC target is Eclipse iceoryx for zero-copy shared-memory transport. However, early implementation should also include an in-process bus.

```text
InProcessBus  -> fast unit/integration tests
IceoryxBus    -> production-style multi-node runtime
```

Do not make iceoryx mandatory for every test. Core contracts and unit tests should run without RouDi or shared memory setup.

Canonical message families should remain small:

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

## 9. Recommended Repository Structure

The current repo already has the correct top-level shape:

```text
dedalus/
├── CMakeLists.txt
├── config/
├── infrastructure/
├── models/
├── scripts/
├── simulation/
├── src/
├── README.md
├── INSTALL.md
├── LLM.md
└── WHITEPAPER.md
```

As the core-stack grows, add public headers under `include/dedalus/` and keep implementations under `src/`.

Recommended expansion:

```text
include/dedalus/
├── core/
│   ├── time.hpp
│   ├── ids.hpp
│   ├── geometry.hpp
│   ├── image.hpp
│   ├── status.hpp
│   └── module.hpp
├── sensors/
│   ├── frame_source.hpp
│   ├── video_stream_source.hpp
│   ├── mpeg_camera_source.hpp
│   ├── camera_model.hpp
│   └── ego_state_provider.hpp
├── perception/
│   ├── detector.hpp
│   ├── tracker.hpp
│   ├── identity_resolver.hpp
│   ├── depth_estimator.hpp
│   ├── optical_flow.hpp
│   ├── feature_extractor.hpp
│   ├── semantic_segmenter.hpp
│   ├── appearance_condition_estimator.hpp
│   ├── projector.hpp
│   ├── types.hpp
│   └── perception_pipeline.hpp
├── world_model/
│   ├── world_model.hpp
│   ├── world_snapshot.hpp
│   ├── dynamic_agent_layer.hpp
│   ├── container_relationship_layer.hpp
│   ├── tactical_obstacle_layer.hpp
│   ├── exclusion_zone.hpp
│   ├── relative_map_layer.hpp
│   ├── global_flight_map_layer.hpp
│   ├── flight_corridor.hpp
│   ├── landmark_layer.hpp
│   ├── localization_layer.hpp
│   ├── appearance_condition_layer.hpp
│   ├── memory_layer.hpp
│   ├── map_conflict.hpp
│   └── effective_world_view.hpp
├── ipc/
│   ├── bus.hpp
│   ├── topics.hpp
│   └── serializers.hpp
└── runtime/
    ├── node.hpp
    ├── pipeline_runner.hpp
    └── config_loader.hpp
```

Implementation layout:

```text
src/
├── core/
├── sensors/
├── perception/
├── world_model/
├── ipc/
├── behavior/
├── safety/
└── runtime/
```

Applications:

```text
apps/
├── dedalus_core_stack.cpp
├── dedalus_perception_node.cpp
├── dedalus_world_model_node.cpp
└── dedalus_dump_world.cpp
```

Tests and tools:

```text
tests/
├── unit/
├── integration/
└── fixtures/

tools/
├── replay_sequence.py
├── export_airsim_frames.py
├── visualize_world_model.py
├── visualize_tactical_map.py
├── visualize_flight_map.py
├── inspect_memory_map.py
├── inspect_identity_hypotheses.py
├── visualize_relative_map.py
└── convert_yolo_to_engine.py
```

Configuration:

```text
config/
├── behaviors.yaml
├── camera_intrinsics.yaml
├── core_stack.yaml
├── perception/
│   ├── detector_mock.yaml
│   ├── detector_yolo_trt.yaml
│   ├── identity_resolver.yaml
│   └── appearance_condition.yaml
└── world_model/
    ├── tactical_obstacle_layer.yaml
    ├── relative_map.yaml
    ├── global_flight_map.yaml
    ├── container_relationship_layer.yaml
    ├── memory_layer.yaml
    └── localization_layer.yaml
```

---

## 10. Runtime Composition

Modules should be selected by config, not hardcoded.

Example placeholder configuration:

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

Future production-style configuration:

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

### Milestone 1: Core Contracts and In-Process Pipeline

Build:

```text
FramePacket
Detection2D
Track2D
IdentityHypothesis
Observation3D
EgoState
AgentState
ContainerState
ContainmentEvent
ExclusionZone
MapFrame
FlightMapCell
StaticStructure
FlightCorridor
AppearanceCondition
WorldSnapshot
EffectiveWorldView
```

Add:

```text
NullDetector
ScriptedDetector
SimpleCentroidTracker
AppearanceOnlyIdentityResolver
FlatGroundProjector
InMemoryWorldModel
InProcessBus
dedalus_core_stack
dedalus_dump_world
```

Goal:

```text
Run a fake/synthetic sequence and emit WorldSnapshot JSON.
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

## 12. Behavior and Control Boundary

The autonomy stack should eventually output bounded kinematic intent:

```text
velocity vector + yaw/yaw-rate intent
```

PX4 remains responsible for attitude stabilization, estimator fusion, motor control, arming state, failsafes, and low-level flight safety.

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

Behavior and control should consume `EffectiveWorldView`, not raw detections.

---

## 13. Non-Goals for the Immediate Core-Stack Phase

Do not start by implementing:

```text
Full behavior trees
Autonomous intercept behavior
Production command mux
Dense SLAM mesh reconstruction
Distributed multi-drone map sharing
Hardware camera drivers
Mandatory iceoryx runtime for every test
```

The first success condition is simpler:

```text
A modular C++ core-stack can consume frames, create observations, maintain a layered world model, and emit debuggable snapshots.
```

Once that is stable, flight behavior and control can be layered on top.
