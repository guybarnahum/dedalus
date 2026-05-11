# Project Dedalus - LLM Context & Operational State

> **Purpose:** This file is the project handoff document for LLMs. It should let an LLM understand the architecture, repo conventions, current simulation state, known bugs, and safe modification patterns without rediscovering the debugging history.

---

## 1. Project Identity

**Project Name:** Dedalus

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

The goal is to build a modular C++ perception-to-world-model runtime that can first run with placeholder blocks, then continuously improve as better detectors, trackers, depth estimators, VIO, mapping, memory, and planning modules become available.

---

## 2. Current Repository Structure

Current top-level structure:

```text
dedalus/
├── CMakeLists.txt
├── config/
│   ├── behaviors.yaml
│   └── camera_intrinsics.yaml
├── infrastructure/
│   ├── aws/
│   │   └── main.tf
│   └── docker/
│       └── Dockerfile.l4t_cross
├── INSTALL.md
├── LLM.md
├── models/
├── README.md
├── scripts/
├── simulation/
│   ├── cleanup.sh
│   ├── INSTALL.md
│   ├── README.md
│   ├── run.sh
│   ├── scenarios/
│   ├── settings.json
│   ├── setup.sh
│   ├── stop.sh
│   ├── test-flight.py
│   └── trajectories/
│       └── circle_figure8.json
├── src/
│   ├── behavior/
│   ├── CMakeLists.txt
│   ├── ipc/
│   ├── perception/
│   ├── safety/
│   ├── sensors/
│   └── world_model/
└── WHITEPAPER.md
```

The current `src/` directories are domain placeholders. The next implementation should add public contracts under `include/dedalus/...`, concrete implementations under `src/...`, small runtime apps under `apps/`, and tests under `tests/`.

Recommended expansion:

```text
include/dedalus/
├── core/
├── sensors/
├── perception/
├── world_model/
├── ipc/
└── runtime/

apps/
├── dedalus_core_stack.cpp
├── dedalus_perception_node.cpp
├── dedalus_world_model_node.cpp
└── dedalus_dump_world.cpp

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
└── inspect_memory_map.py
```

---

## 3. Architectural Direction

Dedalus is intended to support a modular drone autonomy stack.

### 3.1 Runtime Domains

Major runtime domains:

- `sensors/`: camera, video stream, IMU, MAVLink/FCU state, ego-state ingestion
- `perception/`: detection, tracking, features, depth, segmentation, projection, TensorRT inference
- `world_model/`: dynamic agents, tactical exclusion zones, global flight map, landmarks, localization, memory
- `behavior/`: policy, intent, behavior trees, mission logic
- `safety/`: command mux, kill switch, manual override, bounded outputs
- `ipc/`: low-latency data exchange; intended production IPC is Eclipse iceoryx

### 3.2 Immediate Focus

The immediate focus is:

```text
Frame input
    -> detection
    -> tracking
    -> depth / geometry / projection
    -> dynamic agents
    -> tactical obstacle/exclusion zones
    -> rough global flight map
    -> landmarks
    -> actual + memory world model
    -> debuggable WorldSnapshot / EffectiveWorldView
```

Do **not** jump directly into behavior trees, intercept behavior, or command output until world-model snapshots are stable.

### 3.3 Control Philosophy

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

The most important rule for the core-stack:

```text
Every stage must be a replaceable module behind a stable interface.
```

First implementations may be simplistic:

```text
ScriptedDetector
SimpleCentroidTracker
FlatGroundProjector
ConeExclusionMapper
InMemoryWorldModel
DisabledMemoryLayer
```

Future implementations should fit behind the same contracts:

```text
YoloTensorRtDetector
ReIdKalmanTracker
VioRayProjector
VoxelSdfMapper
MultiResolutionFlightMap
PersistentMemoryLayer
```

Avoid hardcoding AirSim, TensorRT, PX4, YOLO, iceoryx, or a specific camera path into the core contracts.

---

## 5. Core Contracts

The following contracts define the initial perception-to-world-model runtime. They should live under `include/dedalus/...`.

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
```

### 5.3 Track2D

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

Initial trackers:

```text
SimpleCentroidTracker
IouTracker
```

Future trackers:

```text
KalmanTracker2D
DeepSortLikeTracker
OpticalFlowAssistedTracker
ReIdTracker
```

### 5.4 EgoState

```cpp
struct EgoState {
    TimePoint timestamp;
    Pose3 local_T_body;
    Vec3 velocity_local;
    Vec3 angular_velocity_body;
    Covariance6 covariance;
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
```

### 5.5 Observation3D

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

## 6. World Model Layers

The world model is not one map. It is a layered state system.

```text
WorldModel
├── DynamicAgentLayer
├── TacticalObstacleLayer
├── GlobalFlightMapLayer
├── LandmarkLayer
├── EgoLocalizationLayer
└── MemoryLayer
```

Perception modules produce observations. The world model owns fused state.

### 6.1 DynamicAgentLayer

Tracks moving entities such as drones, people, vehicles, animals, and other mission-relevant actors.

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

Start with simple in-memory motion extrapolation. Later add EKF and ReID-assisted persistence.

### 6.2 TacticalObstacleLayer

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

### 6.3 GlobalFlightMapLayer

Low-resolution, longer-horizon navigational map used for rough route planning.

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

Initial implementations:

```text
MultiResolutionHeightMap
PrimitiveStaticMapper
FlightCorridorExtractor
```

### 6.4 LandmarkLayer and EgoLocalizationLayer

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

Landmarks should include geometry plus feature signatures. The localization layer can later match current visual features against landmarks to reduce drift in GPS-denied flight.

### 6.5 MemoryLayer

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
4. If actual repeatedly contradicts memory, update or retire memory.
5. If memory has not been confirmed recently, decay confidence.
6. Unknown regions are treated conservatively, not as free space.
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

## 7. WorldSnapshot Contract

The world model should publish a debug-friendly snapshot early.

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

Early apps should be able to emit JSON snapshots for tests and visualization:

```json
{
  "timestamp_ns": 123456789,
  "ego": {
    "position_local": [0.0, 0.0, -12.0],
    "velocity_local": [1.2, 0.0, 0.0]
  },
  "agents": [],
  "tactical_exclusion_zones": [],
  "flight_corridors": [],
  "static_structures": [],
  "landmarks": [],
  "uncertain_regions": []
}
```

---

## 8. IPC Strategy

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
Observation3DArray
WorldSnapshot
EffectiveWorldView
ControlIntent later
```

---

## 9. Runtime Composition

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

## 10. Implementation Roadmap

### Milestone 1: Core Contracts and In-Process Pipeline

Add public headers and placeholder implementations.

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

## 11. Simulation Stack

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

## 12. Setup / Cleanup / Run Responsibilities

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

## 13. AirSim / PX4 Settings

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

## 14. Known AirSim / Colosseum PX4 Limitation

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

## 15. PX4 Shell Access

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

## 16. `test-flight.py` Current Behavior

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

## 17. Trajectory System

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

## 18. Known Debugging Milestones

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

## 19. Common Commands

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

## 20. Style / Contribution Guidance for LLMs

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
- Build `MultiResolutionHeightMap` before dense SLAM.
- Build `DisabledMemoryLayer` / in-memory memory before persistent storage.
- Keep C++ contracts stable and small.
