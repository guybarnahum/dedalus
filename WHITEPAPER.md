# Architecture White Paper: Project Dedalus

**Document Type:** Technical Strategy & Architecture Review

**Target Audience:** Engineering Leadership, Principal Architects, Autonomy Runtime Contributors

**Subject:** Modular Vision-Centric Perception, World Modeling, Behavior, Mapping, and Bounded Flight-Control Intent for Edge-Compute UAVs

---

## 1. Executive Summary

Project Dedalus is a vision-centric autonomous drone stack and virtual proving ground for perception-driven flight behavior, world modeling, tactical obstacle avoidance, persistent traverse memory, and bounded flight-control intent generation.

The system is designed around a simple architectural principle:

```text
Every capability is a replaceable module behind a stable contract.
```

Milestone 2.20 closed the first live-mission robustness phase. Dedalus can now run repeatable AirSim/PX4 missions through a stable mission runtime and a PX4 bridge that emits bounded velocity setpoints through the proven `pymavlink` path. The next major objective is not simply to fly a preloaded trajectory, but to fly based on detected/tracked objects.

The next system-level demonstration, Milestone 3.0, is **object-conditioned flight behavior**:

```text
vision / world-model target
  -> target selection
  -> follow, circle, approach, or sequence behavior
  -> bounded velocity vector into PX4 SITL / AirSim
  -> return / land / disarm
  -> validated mission artifacts
```

Post-Milestone 3, Dedalus should become spatially adaptive. The same behaviors should execute through a tactical occupancy and avoidance layer so the drone can reroute around static and dynamic obstacles, remember known-safe corridors in a persistent traverse map, cache successful flight solutions when appropriate, and visualize both drone-relative maps and drone POV explanations.

The target production runtime remains C++20 on NVIDIA Jetson Orin-class edge compute. Python remains useful for simulation harnesses, dataset tooling, model training, visualization, and the currently validated PX4/MAVLink bridge. Production migration should be staged; working mission behavior should not be destabilized by prematurely rewriting the PX4 control path.

---

## 2. Design Philosophy

Dedalus follows these core design principles.

### 2.1 Modularity at Every Stage

Each stage of the stack should be independently replaceable:

```text
Frame source
Detector
Tracker
Identity resolver
Depth / geometry estimator
3D projector
World model
Target selector
Behavior controller
Tactical obstacle mapper
Avoidance planner
Persistent traverse map
Route cache
Visualization / artifact layer
Command output
```

The first implementation may use mock detections, AirSim ground truth, simple geometry, or scripted targets. The contracts should not change when those implementations are replaced by TensorRT detectors, learned depth, VIO, SLAM, ReID, EKF tracking, container-aware identity reasoning, lighting-robust embeddings, tactical voxel maps, or more advanced planners.

### 2.2 Simulation First, Hardware Later

Dedalus follows:

```text
Train Heavy, Simulate Accurately, Fly Light
```

The Colosseum/AirSim + PX4 SITL environment is the proving ground. Before a behavior or perception change reaches physical hardware, it should be executable in simulation. Simulation should not be a separate architecture; the same C++ core-stack should run against simulation adapters and later against hardware adapters.

### 2.3 Edge Runtime Discipline

The production runtime should be C++20-first. It should be deterministic, bounded, testable, and compatible with NVIDIA JetPack, CUDA, TensorRT, and low-latency IPC. Python belongs in the ML factory, scenario tooling, test harnesses, visualization tools, and the currently validated PX4/MAVLink simulation bridge until a deliberate native replacement is built.

### 2.4 World Model as the Source of Truth

Perception modules produce observations. The world model owns fused state.

The world model is not a single map. It is a layered spatial state system that includes:

```text
Dynamic agents
People and object-containment relationships
Target-selection facts
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

### 2.5 Behavior Emits Intent, Not Low-Level Flight Control

Dedalus behavior controllers should output bounded kinematic intent:

```text
velocity vector + yaw/yaw-rate intent
```

PX4 remains responsible for attitude stabilization, estimator fusion, motor control, arming state, failsafes, and low-level flight safety. The flight sink should not understand mission semantics, obstacles, or target identity. It should send bounded commands.

### 2.6 Safety Through Conservative Uncertainty

Unknown space is not automatically free space. If the drone cannot see a region and only has memory or weak inference, the system should use lower confidence, larger margins, reduced speed, increased altitude, or require more observation before committing to a route.

### 2.7 Identity Is Not Only Appearance

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

## 3. Current Proven Mission Control Baseline

Milestone 2.20 established a stable live AirSim/PX4 mission path:

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
  -> simulation/px4-command-bridge.py
  -> PX4 / AirSim
```

The working flight-control split deliberately mirrors `simulation/test-flight.py`:

```text
PX4 shell:
  commander arm
  commander takeoff
  commander land
  commander disarm

pymavlink:
  wait heartbeat
  prime OFFBOARD velocity stream
  set PX4 OFFBOARD mode
  climb to safe height using LOCAL_POSITION_NED feedback
  send SET_POSITION_TARGET_LOCAL_NED velocity setpoints
```

This split is important. Synchronous command dispatch OK is not vehicle-state truth. Mission transitions into flight execution must be driven by world-model telemetry such as ego height and state.

The current default live mission path should remain:

```yaml
flight_command_sink: px4_bridge
```

Do not replace it with the native experimental C++ MAVLink sink while building Milestone 3 behavior.

---

## 4. Perception-to-Behavior Pipeline

The Milestone 3 vertical slice should prove this dataflow:

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
Projection / 3D estimate / simulation hint
        ↓
WorldSnapshot agents
        ↓
TargetSelector
        ↓
BehaviorRuntime / ObjectBehaviorMissionController
        ↓
Desired velocity / yaw intent
        ↓
FlightCommandSink
        ↓
PX4 / AirSim
```

The perception pipeline should not directly decide how to fly. It should generate typed observations with timestamps, confidence, geometry, identity hypotheses, containment events, lighting condition estimates, and optional feature signatures. Behavior consumes world-model state.

A minimum Milestone 3 target state is:

```json
{
  "id": "agent_1",
  "class_label": "person",
  "confidence": 0.82,
  "position_local_m": [12.0, 4.0, 0.0],
  "position_valid": true,
  "velocity_local_mps": [0.5, 0.0, 0.0],
  "velocity_valid": true
}
```

For Milestone 3, it is acceptable to start with scripted, simulated, or ground-truth-projected target state. The proof is object-conditioned flight behavior, not perfect monocular 3D.

---

## 5. Behavior Language v1

The behavior language should be small and declarative.

Core concepts:

```text
Target selector
Behavior type
Reference frame
Desired relative geometry
Constraints
Completion condition
Fallback behavior
```

Initial behavior types:

```text
hold
search
follow
approach
circle
go_home
land
sequence
```

Reference frames to support over time:

```text
target_heading_frame
  offset relative to target direction of travel

drone_heading_frame
  offset relative to current drone heading

world_local_frame
  offset fixed in takeoff-local / map coordinates

camera_frame
  offset based on drone POV/image bearing
```

For Milestone 3, prefer `target_heading_frame` and `world_local_frame` first.

### 5.1 Follow

Follow means maintaining a relative 3D offset from a selected target.

```yaml
mission:
  name: follow_person_demo

target:
  selector:
    class: person
    confidence_min: 0.55
    policy: highest_confidence

behavior:
  type: follow
  target_frame: target_heading_frame
  relative_offset_m:
    x: -8.0
    y: 0.0
    z: 4.0
  max_speed_mps: 2.0
  position_tolerance_m: 1.5
  lost_target_timeout_s: 5.0

completion:
  after_s: 30
  then: go_home_land
```

The controller computes a desired drone position relative to the target and emits a bounded velocity vector toward it.

### 5.2 Circle

Circle means orbiting a selected target at a chosen radius and altitude offset. It is best for static or slow targets such as parked cars, standing people, suspicious objects, or inspection points.

```yaml
mission:
  name: circle_car_demo

target:
  selector:
    class: car
    confidence_min: 0.6
    policy: nearest

behavior:
  type: circle
  radius_m: 10.0
  altitude_offset_m: 5.0
  angular_speed_deg_s: 12.0
  direction: clockwise
  center:
    target: selected_target
  max_speed_mps: 3.0
```

For a slow target, the orbit center can track the target with smoothing.

### 5.3 Approach

Approach means moving toward a target until a relative condition is satisfied, then optionally running another behavior.

```yaml
behavior:
  type: sequence
  steps:
    - type: approach
      stop_distance_m: 8.0
      altitude_offset_m: 4.0
    - type: circle
      radius_m: 10.0
      duration_s: 20.0
    - type: go_home_land
```

Approach completion conditions can include:

```text
distance_to_target <= stop_distance
target centered in image
target confidence high enough
time budget expired
mission condition triggered
```

---

## 6. Mission 3.0 Success Criteria

Milestone 3.0 is:

```text
Object-conditioned flight behavior demo
```

Success criteria:

```text
1. Drone takes off and reaches safe height.
2. WorldSnapshot contains a valid detected/tracked target class instance.
3. TargetSelector selects a class or instance, e.g. person or car.
4. BehaviorRuntime starts follow, circle, approach, or sequence behavior.
5. Controller emits bounded velocity vectors through the existing PX4 bridge path.
6. Drone returns home, lands, and disarms.
7. mission_events + snapshots prove target selection, behavior execution, and completion.
```

Expected future mission events:

```json
{"event":"target_selected","class":"car","track_id":"agent_3","confidence":0.86}
{"event":"behavior_start","behavior":"approach","target":"agent_3"}
{"event":"behavior_complete","behavior":"approach","reason":"standoff_reached"}
{"event":"behavior_start","behavior":"circle","target":"agent_3"}
{"event":"behavior_complete","behavior":"circle","reason":"duration_elapsed"}
```

Milestone 3 should not own full obstacle avoidance. It should prove that object-conditioned behavior can drive the existing velocity-command path.

---

## 7. Post-Milestone 3 Spatial Autonomy

After M3, the same behaviors should become obstacle-aware. The architecture should insert tactical avoidance between behavior and the flight sink:

```text
BehaviorController
  -> desired velocity vector
  -> TacticalAvoidancePlanner
  -> safe velocity vector
  -> Px4BridgeCommandSink
```

The sink still receives bounded velocity/yaw intent only.

### 7.1 Milestone 4 — Local Tactical Occupancy Map

Build a drone-relative real-time occupancy map from vision and ego motion.

```text
LocalOccupancyMap
  voxel grid or cone/ray occupancy structure
  origin: takeoff pose
  update source: vision + ego pose
  decay: dynamic obstacles fade faster
  confidence: per cell / cone / ray
```

It answers:

```text
Where is unsafe right now?
Where is probably free?
Where is unknown?
What obstacles are static?
What obstacles are moving?
```

### 7.2 Milestone 5 — Reactive Obstacle Avoidance Planner

Modify desired velocity or trajectory intent around occupied/risky space.

```text
Mission / Behavior intent
  -> desired velocity / waypoint / trajectory segment
  -> TacticalAvoidancePlanner
  -> safe bounded velocity
  -> Px4BridgeCommandSink
```

An early algorithm can sample candidate velocity vectors, score them for progress, clearance, smoothness, height safety, and alignment with desired motion, then emit the best safe vector.

### 7.3 Milestone 6 — Persistent Traverse Map

Convert local tactical maps into persistent flight memory.

```text
TraverseMap
  known-safe corridors
  known-blocked regions
  historical obstacle locations
  risk/cost surfaces
  preferred flight lanes
  previously successful paths
  confidence and freshness
```

Persistent memory is advisory, not truth. Current tactical sensing overrides stale memory.

### 7.4 Milestone 7 — Cached Flight Solutions

Cache and reuse successful route solutions when the environment is similar.

```text
start_region
goal_region
constraints
trajectory_solution
clearance_margin
success_count
failure_count
last_success
map_version
risk_score
```

If live tactical sensing contradicts a cached route, the planner must modify or abandon it.

### 7.5 Milestone 8 — Tactical Map and Drone POV Visualization

Visualize both:

```text
1. drone-relative / takeoff-origin map
2. drone POV view
```

Map layers should include:

```text
drone path
current drone pose
target trajectory
safe planned vector
occupied voxels / cones
unknown regions
static obstacles
dynamic obstacles
cached route / remembered safe corridor
```

Drone POV overlays should include detections/tracks, obstacle cones, free-space corridor, desired direction, avoidance direction, nearest obstacle, map confidence, and planner status.

### 7.6 Milestone 9 — Integrated Spatial Autonomy Demo

Demonstrate object-conditioned behavior with live obstacle avoidance and persistent map updates:

```text
detect object
follow/circle/approach target
obstacle appears
local map marks obstacle
planner reroutes
mission completes
traverse map stores blocked/safe regions
next run uses remembered priors while respecting live sensing
```

### 7.7 Milestone 10 — Multi-Flight Site Memory

Maintain site-local memory across missions:

```text
SiteMemory
  traverse map
  semantic landmarks
  route cache
  risk history
  obstacle history
  mission outcomes
```

This supports repeated patrol, inspection corridors, hazard history, preferred altitudes, and dock-to-observation-point routes.

---

## 8. Implementation Roadmap

```text
2.20  Reliable live mission loop                            DONE

2.21  Mission artifact validator
2.22  Scenario/campaign harness
2.23  Behavior spec parser foundation
2.24  Target selector from WorldSnapshot agents
2.25  ObjectBehaviorMissionController
2.26  Follow behavior
2.27  Circle behavior
2.28  Approach + behavior sequence
2.29  M3 demo hardening

3.0   Object-conditioned flight behavior demo

4.0   Local tactical occupancy map
5.0   Reactive obstacle avoidance planner
6.0   Persistent traverse map / flight memory
7.0   Cached route solutions
8.0   Tactical map + drone POV visualization
9.0   Spatial autonomy demo with avoidance
10.0  Multi-flight site memory
```

---

## 9. Runtime Composition

Modules should be selected by config, not hardcoded.

Milestone 3 configuration should look conceptually like:

```yaml
runtime:
  rate_hz: 30
  mode: in_process

sensors:
  frame_source:
    type: airsim
    camera: front_center
  ego_state:
    type: frame_hint

perception:
  detector:
    type: scripted_or_yolo
  tracker:
    type: simple_centroid
  projector:
    type: flat_ground_or_airsim_hint

world_model:
  dynamic_agents:
    type: in_memory

behavior:
  controller:
    type: object_behavior_mission
  spec_path: simulation/behaviors/follow_person.yaml

flight:
  sink: px4_bridge
```

Post-M3 configuration should add:

```yaml
avoidance:
  planner:
    type: tactical_velocity_sampler
  occupancy_map:
    type: local_voxel_or_cone

memory:
  traverse_map:
    type: persistent
    path: data/traverse_memory
```

---

## 10. Repository Structure Direction

As the core-stack grows, add public headers under `include/dedalus/` and keep implementations under `src/`.

Recommended behavior additions:

```text
include/dedalus/behavior/
  behavior_spec.hpp
  target_selector.hpp
  object_behavior_mission_controller.hpp
  behavior_runtime.hpp

src/behavior/
  behavior_spec.cpp
  target_selector.cpp
  object_behavior_mission_controller.cpp
  behavior_runtime.cpp

simulation/behaviors/
  follow_person.yaml
  circle_car.yaml
  approach_then_circle.yaml
```

Recommended post-M3 spatial additions:

```text
include/dedalus/world_model/
  local_occupancy_map.hpp
  traverse_map.hpp
  route_cache.hpp

include/dedalus/behavior/
  tactical_avoidance_planner.hpp

src/world_model/
  local_occupancy_map.cpp
  traverse_map.cpp
  route_cache.cpp

src/behavior/
  tactical_avoidance_planner.cpp

tools/
  visualize_tactical_map.py
  visualize_drone_pov.py
  inspect_traverse_map.py
```

---

## 11. Behavior and Control Boundary

The autonomy stack should output bounded kinematic intent:

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
Avoidance Planner
    >
Behavior Intent
```

Behavior and control should consume `WorldSnapshot` / future `EffectiveWorldView`, not raw detections.

Do not place obstacle avoidance inside the flight sink. Avoidance is a planner layer that changes desired motion into safe motion before the sink.

---

## 12. Non-Goals for Milestone 3

Do not make Milestone 3 own:

```text
Full tactical obstacle avoidance
Persistent traverse memory
Cached route solutions
Dense SLAM mesh reconstruction
Distributed multi-drone map sharing
Hardware camera drivers
Mandatory iceoryx runtime for every test
```

Milestone 3 success is narrower and sharper:

```text
A detected/tracked object class instance can drive follow, circle, approach, or sequence behavior through the existing velocity-command path, and the mission artifacts prove it.
```

---

## 13. Summary

Dedalus has crossed from frame/world-model infrastructure into a working live mission loop. The next step is object-conditioned autonomy: the drone should fly because it saw and selected a target, not because it was handed a static trajectory.

The immediate roadmap should stay disciplined:

```text
M3 proves object-conditioned behavior.
M4-M5 add tactical occupancy and avoidance.
M6-M7 add traverse memory and route caching.
M8 adds map + POV visualization.
M9-M10 demonstrate spatial autonomy and site memory.
```

This keeps the architecture modular while moving toward the larger goal: drones that perceive, remember, choose behavior, avoid obstacles, and improve over repeated flights.
