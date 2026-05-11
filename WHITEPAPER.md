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

Dedalus follows five core design principles.

### 2.1 Modularity at Every Stage

Each stage of the stack should be independently replaceable:

```text
Frame source
Detector
Tracker
Depth / geometry estimator
3D projector
Tactical obstacle mapper
Global flight-map builder
Landmark extractor
World-memory layer
Planner
Command output
```

The first working implementation may use mock detections, AirSim ground truth, simple geometry, or low-resolution maps. The contracts should not change when those implementations are replaced by TensorRT, learned depth, VIO, SLAM, ReID, EKF, or more advanced mapping.

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
Tactical obstacle/exclusion zones
Global rough flight map
Landmarks and feature signatures
Ego-localization support
Actual current observations
Persistent memory of familiar areas
Uncertainty and conflict state
```

Behavior and planning should read world-model products, not raw detector outputs.

### 2.5 Safety Through Conservative Uncertainty

Unknown space is not automatically free space. If the drone cannot see a region and only has memory or weak inference, the system should use lower confidence, larger margins, reduced speed, increased altitude, or require more observation before committing to a route.

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
perception/    detection, tracking, features, depth, segmentation, projection
world_model/   dynamic agents, tactical map, global map, landmarks, memory
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
Tracker
        ↓
Track2D
        ↓
Depth / Geometry / Projection
        ↓
Observation3D
        ↓
World Model Builders
        ↓
WorldSnapshot / EffectiveWorldView
```

The perception pipeline should not directly decide how to fly. It should generate typed observations with timestamps, confidence, geometry, and optional feature signatures.

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
```

### 5.3 Tracking Contract

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
```

Initial implementations:

```text
SimpleCentroidTracker
IouTracker
```

Future implementations:

```text
KalmanTracker2D
DeepSortLikeTracker
OpticalFlowAssistedTracker
ReIdTracker
```

### 5.4 Ego-State Contract

```cpp
struct EgoState {
    TimePoint timestamp;
    Pose3 local_T_body;
    Vec3 velocity_local;
    Vec3 angular_velocity_body;
    Covariance6 covariance;
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
```

### 5.5 3D Observation Contract

```cpp
struct Observation3D {
    TrackId track_id;
    TimePoint timestamp;
    Vec3 position_body;
    Vec3 position_local;
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
├── TacticalObstacleLayer
├── GlobalFlightMapLayer
├── LandmarkLayer
├── EgoLocalizationLayer
└── MemoryLayer
```

### 6.1 Dynamic Agent Layer

The dynamic agent layer tracks moving entities such as drones, people, cars, and other mission-relevant actors.

```cpp
struct AgentState {
    AgentId agent_id;
    TrackId source_track_id;
    TimePoint last_seen;
    Vec3 position_local;
    Vec3 velocity_local;
    Covariance6 covariance;
    ClassLabel class_label;
    FactionLabel faction;
    AgentLifecycle lifecycle;  // new, active, occluded, stale, retired
    float confidence;
};
```

Initial implementation:

```text
InMemoryAgentLayer with simple motion extrapolation
```

Future implementation:

```text
EKF-backed agent fusion with occlusion handling and ReID-assisted persistence
```

### 6.2 Tactical Obstacle Layer

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
    float confidence;
    float inflation_radius_m;
    TimePoint expires_at;
    std::string reason;
};
```

### 6.3 Global Flight Map Layer

The global flight map is a low-resolution, longer-horizon navigational representation. It identifies rough traversability, flight corridors, static structures, and localization anchors.

It should be built as a pyramid of resolutions:

```text
Level 0: coarse 2.5D height / traversability map
Level 1: cylinders and primitive static obstacle volumes
Level 2: landmark structures with visual feature signatures
Level 3: optional dense local patches near important regions
```

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
    FeatureSignature signature;
    float confidence;
    TimePoint first_seen;
    TimePoint last_confirmed;
};

struct FlightCorridor {
    CorridorId corridor_id;
    std::vector<Vec3> centerline;
    float radius_m;
    float min_altitude_m;
    float max_altitude_m;
    float confidence;
};
```

### 6.4 Landmark and Localization Layer

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

Landmarks should include both geometry and feature signatures. The same layer can later support map-relative localization, relocalization after drift, and shared-map updates.

### 6.5 Memory Layer

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
4. If actual repeatedly contradicts memory, update or retire memory.
5. If memory has not been confirmed recently, decay confidence.
6. Unknown regions are treated conservatively, not as free space.
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

    std::vector<AgentState> agents;
    std::vector<ExclusionZone> tactical_exclusion_zones;
    std::vector<FlightCorridor> flight_corridors;
    std::vector<StaticStructure> static_structures;
    std::vector<Landmark> landmarks;
    std::vector<UncertainRegion> uncertain_regions;
};
```

A JSON debug form should exist for tests, replay, and visualization:

```json
{
  "timestamp_ns": 123456789,
  "ego": {
    "position_local": [0.0, 0.0, -12.0],
    "velocity_local": [1.2, 0.0, 0.0]
  },
  "agents": [
    {
      "agent_id": "agent_0001",
      "class": "drone",
      "faction": "unknown",
      "position_local": [18.2, -3.1, -10.5],
      "velocity_local": [2.0, 0.4, 0.0],
      "confidence": 0.82,
      "lifecycle": "active"
    }
  ],
  "tactical_exclusion_zones": [],
  "flight_corridors": [],
  "static_structures": [],
  "landmarks": []
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
│   ├── depth_estimator.hpp
│   ├── optical_flow.hpp
│   ├── feature_extractor.hpp
│   ├── semantic_segmenter.hpp
│   ├── projector.hpp
│   ├── types.hpp
│   └── perception_pipeline.hpp
├── world_model/
│   ├── world_model.hpp
│   ├── world_snapshot.hpp
│   ├── dynamic_agent_layer.hpp
│   ├── tactical_obstacle_layer.hpp
│   ├── exclusion_zone.hpp
│   ├── global_flight_map_layer.hpp
│   ├── flight_corridor.hpp
│   ├── landmark_layer.hpp
│   ├── localization_layer.hpp
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
│   └── detector_yolo_trt.yaml
└── world_model/
    ├── tactical_obstacle_layer.yaml
    ├── global_flight_map.yaml
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
  projector:
    type: airsim_depth

world_model:
  dynamic_agents:
    type: in_memory
  tactical_obstacles:
    type: cone_exclusion
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
  projector:
    type: vio_ray_projector

world_model:
  dynamic_agents:
    type: ekf
  tactical_obstacles:
    type: voxel_sdf
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
Observation3D
EgoState
AgentState
ExclusionZone
FlightMapCell
StaticStructure
FlightCorridor
WorldSnapshot
EffectiveWorldView
```

Add:

```text
NullDetector
ScriptedDetector
SimpleCentroidTracker
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

### Milestone 4: Rough Global Flight Map

Add:

```text
MultiResolutionHeightMap
PrimitiveStaticMapper
FlightCorridorExtractor
LandmarkLayer
```

Goal:

```text
Build low-resolution traversability and candidate flight corridors from repeated frames.
```

### Milestone 5: Memory Layer

Add:

```text
PersistentMapStore
ActualVsMemoryFusion
MapConflictDetector
MemoryConfidenceDecay
```

Goal:

```text
Load remembered map state, use it when actual observations are incomplete, and update memory only after repeated confirmation.
```

### Milestone 6: Production Perception Upgrades

Add:

```text
YoloOnnxDetector
YoloTensorRtDetector
KalmanTracker2D
ReIdTracker
VioRayProjector
MonocularDepthProjector
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
