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

Dedalus can now run repeatable AirSim/PX4 missions through a stable mission runtime and a PX4 bridge that emits bounded velocity setpoints through the proven `pymavlink` path. The next major objective is not simply to fly a preloaded trajectory, but to fly based on detected/tracked objects.

The next system-level demonstration, Milestone 3.0, is **object-conditioned flight behavior**:

```text
vision / world-model target
  -> target selection
  -> follow, circle, approach, or sequence behavior
  -> bounded velocity vector into PX4 SITL / AirSim
  -> return / land / disarm
  -> validated mission artifacts
```

A central architectural invariant for this phase is that Dedalus must distinguish tracker continuity, world-model objects, and recognized identity. A behavior should be able to select a specific tracked person or car from a group and keep that target stable across frames. It must not switch targets merely because a neighboring object has higher confidence.

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

The first implementation may use mock detections, AirSim ground truth, simple geometry, scripted targets, or ghost detections. The contracts should not change when those implementations are replaced by TensorRT detectors, learned depth, VIO, SLAM, ReID, EKF tracking, container-aware identity reasoning, lighting-robust embeddings, tactical voxel maps, or more advanced planners.

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

### 2.5 Object Identity Is Layered

Dedalus must not collapse detections, tracks, agents, and identities into one field. They represent different ownership and certainty levels:

```text
detection_id  ->  track_id  ->  agent_id  ->  identity_id
single frame      tracker       world model    recognized identity
```

The layers mean:

```text
detection_id:
  A single detector observation in one frame.

track_id:
  Tracker-owned frame-to-frame continuity for one moving blob/object.
  It is usually local to one tracker session and may reset after restart or long target loss.

source_track_id:
  The tracker ID preserved inside AgentState as provenance.
  It answers: which tracker track produced this world-model agent?

agent_id:
  World-model-owned object handle.
  Behavior and planning should select agents, not raw detections.
  Today it may be derived from source_track_id; later it can represent a fused object from multiple sensors/tracks.

identity_id:
  Identity-resolver-owned recognized real-world identity.
  Later this may be a known person, vehicle plate, drone serial, or cross-mission identity.
```

This distinction enables a behavior to select one object from a group. For example, if two people are visible:

```text
track_001: person, confidence 0.82
track_002: person, confidence 0.91
```

then the world model should represent them as distinct agents:

```text
agent_id:        agent_track_001
source_track_id: track_001
identity_id:     identity_track_001
class:           person
confidence:      0.82

agent_id:        agent_track_002
source_track_id: track_002
identity_id:     identity_track_002
class:           person
confidence:      0.91
```

A behavior may choose `track_001` and should keep that target even when `track_002` has higher confidence. Confidence is a ranking signal, not a stable identity.

### 2.6 Behavior Emits Intent, Not Low-Level Flight Control

Dedalus behavior controllers should output bounded kinematic intent:

```text
velocity vector + yaw/yaw-rate intent
```

PX4 remains responsible for attitude stabilization, estimator fusion, motor control, arming state, failsafes, and low-level flight safety. The flight sink should not understand mission semantics, obstacles, or target identity. It should send bounded commands.

### 2.7 Safety Through Conservative Uncertainty

Unknown space is not automatically free space. If the drone cannot see a region and only has memory or weak inference, the system should use lower confidence, larger margins, reduced speed, increased altitude, or require more observation before committing to a route.

### 2.8 Identity Is Not Only Appearance

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

### 2.9 Runtime Event Boundaries

The runtime is organized around typed publishers and subscribers. The key distinction is between control-critical in-process handoff and external/debug subscription.

```text
Control-critical path:
  Publisher<T>
    -> in-process subscriber
    -> behavior / mission runtime / flight sink

External observation path:
  Publisher<T>
    -> RuntimeEventStreamServer
    -> TCP JSONL event stream
    -> visualization, diagnostics, customer tools

Evidence path:
  Publisher<T>
    -> artifact writer
    -> files for validation/debug after the run
```

This means the world model does not know who consumes it, and the AirSim overlay does not compute runtime state. Producers publish typed events. Subscribers decide what to consume.

Current runtime event flow:

```text
AirSim / PX4
  -> AirSimFrameSource + FrameHintEgoProvider
  -> CoreStackRunner
       -> PerceptionPipeline
       -> optional GhostTargetProvider::frame_at(...)
            -> GhostDetectionsPublisher
            -> PerceptionPipelineOutput.observations
       -> InMemoryWorldModel
       -> WorldSnapshotPublisher
            -> LatestWorldSnapshotSubscriber -> MissionRuntime -> FlightCommandSink -> PX4 / AirSim
            -> ArtifactSnapshotWriter        -> snapshot artifacts
            -> RuntimeEventStreamServer      -> TCP JSONL stream

RuntimeEventStreamServer
  -> ghost_detections events
  -> world_snapshot events
  -> simulation/airsim/scripts/airsim-world-overlay.py and external/debug subscribers
```

The design rule is:

```text
Artifacts are evidence, not IPC.
Behavior consumes WorldSnapshot, not GhostDetectionsFrame.
GhostDetectionsFrame exists to make simulation/debug visibility consume the same ghost evaluation that was injected into perception.
```

The detailed source-to-sink diagrams live in [docs/runtime_dataflow.md](docs/runtime_dataflow.md).

---

## 3. Current Proven Mission Control Baseline

The stable live AirSim/PX4 mission path is:

```text
AirSim live frame + ego sidecar
  -> AirSimFrameSource
  -> FrameHintEgoProvider
  -> CoreStackRunner
       -> optional GhostTargetProvider::frame_at(...)
       -> InMemoryWorldModel
       -> WorldSnapshotPublisher
            -> LatestWorldSnapshotSubscriber
            -> RuntimeEventStreamServer, if enabled
  -> LatestWorldSnapshot
  -> MissionRuntime async loop
  -> TrajectoryMissionController / ObjectBehaviorMissionController
  -> Px4BridgeCommandSink
  -> tools/px4/px4-command-bridge.py
  -> PX4 / AirSim
```

The working flight-control split deliberately mirrors `simulation/airsim/scripts/test-flight.py`:

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
Detector or GhostDetectionProvider
        ↓
Detection2D / scripted Observation3D
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
  "agent_id": "agent_track_001",
  "source_track_id": "track_001",
  "identity_id": "identity_track_001",
  "class": "person",
  "confidence": 0.82,
  "position_local": [12.0, 4.0, 0.0],
  "velocity_local": [0.5, 0.0, 0.0],
  "lifecycle": "active"
}
```

For Milestone 3, it is acceptable to start with scripted, simulated, ghost, or ground-truth-projected target state. The proof is object-conditioned flight behavior, not perfect monocular 3D.

A good pre-camera validation path is:

```text
Synthetic / AirSim frame
  -> GhostTargetProvider::frame_at(...)
       -> GhostDetectionsFrame for runtime-event stream/debug subscribers
       -> Observation3D objects appended to PerceptionPipelineOutput.observations
  -> InMemoryWorldModel
  -> WorldSnapshot.agents with stable source_track_id
  -> TargetSelector
  -> ObjectBehaviorMissionController later
```

Avoid shortcuts that set `selected_target` directly from config. That bypasses the same chain real camera detections must use.

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
go_home_land
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

### 5.1 Target Selection

Target selection should support selecting a class group or a specific track/agent:

```yaml
target:
  selector:
    class: person
    track_id: ghost_person_001
    confidence_min: 0.55
    policy: persistent_track
    reacquire_timeout_s: 5.0
```

At least one of `class`, `track_id`, or `agent_id` should be present. If a previous target remains valid, `persistent_track` should keep it even if another same-class object has higher confidence.

### 5.2 Follow

Follow means maintaining a relative 3D offset from a selected target.

```yaml
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
```

The controller computes a desired drone position relative to the target and emits a bounded velocity vector toward it.

### 5.3 Circle

Circle means orbiting a selected target at a chosen radius and altitude offset. It is best for static or slow targets such as parked cars, standing people, suspicious objects, or inspection points.

```yaml
behavior:
  type: circle
  radius_m: 10.0
  altitude_offset_m: 5.0
  angular_speed_deg_s: 12.0
  direction: clockwise
  max_speed_mps: 3.0
```

For a slow target, the orbit center can track the target with smoothing.

### 5.4 Approach

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

Approach completion conditions can include distance to target, image centering, confidence threshold, or time budget. For M3, prefer distance-to-target standoff first.

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
3. TargetSelector selects a class or specific source_track_id/agent_id.
4. Selected target identity remains stable unless reacquire/lost is expected.
5. BehaviorRuntime starts follow, circle, approach, or sequence behavior.
6. Controller emits bounded velocity vectors through the existing PX4 bridge path.
7. Drone returns home, lands, and disarms.
8. mission_events + snapshots prove target selection, behavior execution, and completion.
```

Expected future mission events should include world-model and tracker handles:

```json
{"event":"target_selected","class":"person","agent_id":"agent_ghost_person_001","source_track_id":"ghost_person_001","identity_id":"identity_ghost_person_001","confidence":0.82}
{"event":"behavior_start","behavior":"follow","agent_id":"agent_ghost_person_001","source_track_id":"ghost_person_001"}
{"event":"behavior_complete","behavior":"follow","reason":"duration_elapsed"}
```

Milestone 3 should not own full obstacle avoidance. It should prove that object-conditioned behavior can drive the existing velocity-command path.

---

## 7. Post-Milestone 3 Spatial Autonomy

After M3, the same behaviors should become obstacle-aware. The architecture inserts tactical avoidance between behavior and the flight sink:

```text
BehaviorController
  -> desired velocity vector
  -> TacticalAvoidancePlanner
  -> safe velocity vector
  -> Px4BridgeCommandSink
```

The sink still receives bounded velocity/yaw intent only.

### Three-Level Obstacle Map Architecture

The spatial layer is structured as three complementary obstacle maps:

```text
L0  LocalFlightMap          ego-local, per-tick, reflexive emergency avoidance
                             Cartesian voxels (→ polar cones: planned direction)
                             Viewer: ego sub-window (planned)

L1  MissionLocalTraversabilityMap   mission-local accumulator, per-flight
                             0.5 m uniform voxels
                             Time-based score decay (0.05/s), score-floor pruning
                             Streamed to viewer as traversability overlay

L2  MissionLocalPlanningMap  cross-mission persistent planning map
                             1 m × 1 m × 2 m voxels (16× fewer than L1)
                             No time decay; evidence-keyed (free space evicts)
                             Disk-backed: saved at landing, loaded at startup
                             Planned: OctoMap-style octree for adaptive resolution
```

Evidence flows: depth sensor → raw obstacle map → L1 accumulator → L2 planning map (incremental).

L2 gives the planner a memory of obstacles that persists across power cycles. A structure mapped on a prior mission is still present until the drone explicitly observes free space at that location. Current tactical sensing (L0/L1) always overrides stale L2 memory.

See `docs/two-level-obstacle-map.md` for the full architecture, lifecycle tables, and open design questions.

Post-M3 roadmap:

```text
4.0 Local tactical occupancy map (L0 + L1)  ✅ Implemented
  L0: ego-local Cartesian voxel crop driving TrajectorySafetyEvaluator.
  L1: mission-local accumulator with time decay and score-floor pruning.

5.0 Reactive obstacle avoidance planner
  Modify desired behavior velocity into safe velocity using L0 / L1.

6.0 Persistent traverse map (L2)  ✅ Implemented
  L2 planning map: evidence-keyed, disk-backed, cross-mission persistence.
  Open: OctoMap octree, polar cone L0, path planner integration.

7.0 Cached flight solutions
  Cache and reuse successful route solutions when the environment is similar.

8.0 Tactical map and drone POV visualization
  L0 ego sub-window in viewer (planned).
  L1 as primary obstacle view; raw evidence as optional debug overlay (planned).

9.0 Integrated spatial autonomy demo
  Object-conditioned behavior with live obstacle avoidance and persistent map updates.

10.0 Multi-flight site memory
  Maintain site-local memory across missions using L2 as the foundation.
```

Persistent memory is advisory, not truth. Current tactical sensing overrides stale memory.

---

## 8. Implementation Roadmap

```text
2.20  Reliable live mission loop                            DONE
2.21  Mission artifact validator                            DONE
2.22  Scenario/campaign harness                             DONE
2.23  Behavior spec parser foundation                       DONE
2.24  Target selector from WorldSnapshot agents             DONE
2.25  ObjectBehaviorMissionController baseline              DONE
2.26  Follow behavior + runtime-event stream                ACTIVE
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
  event_stream:
    enabled: true
    bind_host: 127.0.0.1
    port: 47770

sensors:
  frame_source:
    type: airsim
    camera: front_center
  ego_state:
    type: frame_hint

perception:
  detector:
    type: scripted_or_yolo_or_ghost
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
  spec_path: config/behaviors/follow_specific_track.yaml

flight:
  sink: px4_bridge
```

Post-M3 configuration should add avoidance and memory modules, but those are not part of M3.

---

## 10. Repository Structure Direction

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

config/behaviors/
  follow_person.yaml
  circle_car.yaml
  approach_target.yaml
  sequence_approach_circle.yaml

config/behaviors/ghost_detections/
  person_pair_crossing.json

config/behaviors/trajectories/
  ghost_person_001_crossing.json
  ghost_person_002_crossing.json
```

Recommended post-M3 spatial additions:

```text
include/dedalus/world_model/
  local_occupancy_map.hpp
  traverse_map.hpp
  route_cache.hpp

include/dedalus/behavior/
  tactical_avoidance_planner.hpp
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
Perfect monocular 3D
Long-lived cross-mission identity recognition
```

Milestone 3 success is narrower and sharper:

```text
A detected/tracked object class instance can drive follow, circle, approach, or sequence behavior through the existing velocity-command path, and the mission artifacts prove it.
```

---

## 13. L3 ESDF and Velocity-Aware Trajectory Planning

### 13.1 The Attractor–Repulsor Model

Dedalus trajectory planning uses an Artificial Potential Field (APF) decomposition:

```
F_total(p, v) = F_attract(p, traj) + F_repulse(p, v)
```

The **attractor** is the desired trajectory: a sequence of waypoints or a spline that the drone tries to track by minimising position error. Without obstacle awareness, velocity commands come entirely from this term.

The **repulsor** is the L3 ESDF field. It pushes the drone away from detected obstacles in proportion to proximity, bending trajectories smoothly around surfaces rather than requiring hard replanning.

This separation is important: the attractor provides the mission intent and global path; the repulsor provides local, reactive deformation. The two are additive and independently tunable.

### 13.2 The L3 ESDF

L3 (`LocalESDFMap`) is a sparse Euclidean Signed Distance Field derived from L2 (`MissionLocalPlanningMap`, the cross-mission persistent obstacle map). Only "shell" cells — those within the truncation radius `d0` of an obstacle surface — are stored. Interior occupied cells are clamped; exterior free space beyond `d0` is omitted entirely. This keeps the working set small (typically a few thousand cells for a 80×80×20 m window) and the map fast to query.

The EDT (Euclidean Distance Transform) runs using the Felzenszwalb–Huttenlocher 3-phase separable algorithm, O(N) in the number of voxels. Each shell cell stores:

- `d` — exact signed distance from the obstacle surface (negative inside, positive outside)
- `grad` — unit gradient from central finite differences on the squared-distance grid; points away from the nearest obstacle surface
- `sgrad` — 1-hop smoothed gradient: average of `grad` with the cell's 6 face-adjacent shell neighbours, renormalized. More surface-normal-like than `grad`; reduces APF discontinuities at edges and corners

The stored geometry is static between L2 updates. Velocity-dependence is handled entirely at query time.

### 13.3 The APF Repulsion Formula

For a cell at signed distance `d` with gradient direction `dir`:

```
F = k · (1/d − 1/d₀) / d² · dir
```

This is the standard Khatib APF repulsion. Key properties:
- Zero at `d = d₀` (no force outside truncation radius)
- Grows without bound as `d → 0` (strong repulsion near surfaces)
- Direction is `dir`, which should point away from the nearest obstacle surface

`repulsion()` uses `sgrad` for `dir` and queries the single cell at the drone position. This is suitable for quasi-static safety checks and hover mode.

### 13.4 Velocity-Aware Repulsion: Why and How

At drone speed `v`, the stopping distance is:

```
d_stop = v² / (2 · a_max)
```

This is the minimum spatial scale the drone can react to. Field features — gradient discontinuities at obstacle corners, thin walls, concave regions — smaller than `d_stop` cannot be safely tracked. The drone will have flown past them before the velocity command can take effect.

The solution is to average the repulsion field over a spatial kernel of radius `R(v)`:

```
R(v) = clamp(v² / (2 · a_max),  R_min = cell_size,  R_max = d₀)
```

At low speed `R → R_min` (one cell), which is identical to `repulsion()`. At high speed `R → d₀`, averaging over the full truncation radius — the drone sees a wide, smooth repulsion field without sharp local features.

The Gaussian kernel weights each neighbouring cell by:

```
w_i = exp(−‖Δx_i‖² / (2σ²)),  σ = R/2
```

The weighted force contributions are summed and divided by the total weight, giving the spatially-averaged APF force at the query position. The distance `d` remains exact (from EDT); only the force direction and magnitude are spatially blended.

`repulsion_smoothed(pos, vel, d0, k, a_max)` implements this. It replaces `repulsion()` in the trajectory planner. `repulsion()` is retained for safety queries (is_clear, collision checking) where exact local geometry is needed.

**Complexity:** For `R = 3 m` at `cell_size = 1 m`, the kernel is at most `7³ = 343` hash-map probes per call. Most probes miss in a sparse field (shell cells are only near obstacle surfaces). Real cost is typically O(tens of probes) per query, well under 1 µs.

### 13.5 Why Not Store the Smoothed Field?

The smoothed repulsion direction depends on `v`. Baking a single `R` into the stored map would make it correct for one speed and wrong for all others. Keeping `R` at query time means:

- The stored L3 geometry is speed-agnostic, persistent, and shareable across sessions.
- The planner applies its own `R` based on its current velocity estimate.
- The viewer can explore any `R` interactively via slider without recomputing the ESDF.

### 13.6 Viewer: Live Exploration of R

The mission viewer exposes a **smooth R** slider in the L3 metrics panel (range: `cell_size` to `d₀`). Changing R instantly rebuilds the arrow visualization using the same Gaussian kernel as the C++ planner:

- **R = 0.5 m** (minimum): arrows show the raw C++ `sgrad`, maximum local detail, noisy near corners.
- **R = 2–3 m**: arrows show surface normals clearly; edges show directional transitions. Typical mid-speed tuning point.
- **R = d₀**: highly blended field; arrows are nearly uniform near large flat surfaces, strongest curvature variation at corners.

When the **vel** checkbox is active, R is computed from the last reported drone speed (`lfmEgoSpeedMps`) with `a_max = 3 m/s²`, so the viewer shows exactly what the planner experiences at the current flight speed.

Arrow colours encode signed distance: **red** `d < 1 m` (near-surface, high-force zone), **yellow** `1 ≤ d < 3 m` (approach zone), **green** `d ≥ 3 m` (outer shell). Arrow length encodes force magnitude: `scale = (d₀ − |d|) / d₀ × 4 m`. Only transition cells and isolated cells are drawn (DOT_THRESH = cos 23°), keeping the field readable at any R.

### 13.7 Ground Plane Detection

The ground/floor does not appear in L3 by default because L2 is seeded only by detected obstacles, and ground hits are typically absent from the forward-facing depth camera FOV. The floor is **not** injected artificially — Dedalus is a real-flight simulator, and the avoidance system must rely on what sensors detect.

Ground repulsion is currently handled as a hard altitude floor constraint in the flight controller, decoupled from the APF. If a downward-facing sensor is added, floor cells will appear in L2 and propagate into L3 naturally, requiring no architecture change.

---

## 15. Visual Depth Perception Pipeline

### 15.1 Context and Motivation

The obstacle map stack (Stages 1–6, L0–L3, all complete) was developed and validated
against AirSim's ground-truth `DepthPlanar` API — a perfect depth sensor available only
in simulation. The next major system capability is replacing that oracle with a real,
vision-only depth pipeline that runs identically in AirSim (using the RGB camera) and on
physical hardware (Jetson Orin + fisheye camera). The AirSim environment continues to
serve as the proving ground; the GT depth API becomes a **validation oracle only**, not
the operational source.

The fundamental engineering constraint is low-SWaP (Size, Weight, and Power). Active
emission sensors — LiDAR, structured light, radar — are excluded. All depth must be
extracted passively from optical streams. This section documents the architectural
decisions, component interfaces, memory model, and implementation roadmap for the
production visual depth pipeline.

### 15.2 Gimbaled Camera Architecture

The front camera is mounted on a **mission-controlled gimbal**. The mission runtime
issues `CameraPointingCommand` each tick based on the active behavior mode:

| Mode | Gimbal direction | Primary use |
|---|---|---|
| angle-from-velocity | Tilted forward, angle ∝ speed | Forward obstacle detection |
| stare-at-target | Aimed at tracked actor | Actor depth + observation |
| landing-approach | Tilted down toward landing zone | Floor / perch detection |
| fixed-forward | Body-axis forward | Maximum forward coverage |

The gimbal's **encoder reading at frame capture time** (not the commanded setpoint) is the
authoritative source for the camera-to-ego extrinsic rotation `R_cam_ego` and translation
`T_cam_ego`. These are published as `ObstacleSensingVolume` every tick. A 10–40 ms
actuator lag between commanded and actual gimbal angle, at 10 m/s drone speed, introduces
up to 40 cm projection error against a 50 cm voxel grid — significant. The camera bridge
is responsible for timestamp-aligning encoder readings to frame capture times; the
detector is stateless about this alignment.

The `ObstacleEvidenceShape::FrustumBin` and spherical polar binning already support
varying frustum orientations. Evidence is always projected to world frame before ingestion
into L1; L0's spherical bins indexed by `(az, el)` naturally capture the gimbaled
frustum's actual coverage without any architectural change.

Coverage gaps during stare-at-target mode (where the forward obstacle arc is offset from
the direction of travel) are handled by L1 map persistence: previously observed geometry
remains in L1 at its measured confidence, decaying at 0.05/s. The behavior tree must
respect this by capping forward speed based on L1 cell age when in tracking mode.

### 15.3 Depth Estimation Paradigm Selection

Three paradigms were evaluated for the low-SWaP AUV profile:

**Monocular Depth Estimation (MDE):** Single-frame inference using a Vision Transformer
or CNN encoder-decoder. Outputs relative depth (scale-ambiguous); requires coupling with
a `MetricScaleEstimate` to recover metric units. Robust against textureless surfaces via
learned semantic priors. Selected as the primary paradigm.

**Traditional and Deep Stereo Matching:** Requires rigidly synchronized stereo pair.
SGM fails on textureless walls and specular surfaces. Deep stereo (RAFT-Stereo) exceeds
the 33 ms budget on Orin Nano. Excluded for V0; available as a secondary engine behind
`DepthEngineInterface` for V1 evaluation.

**Structure from Motion (SfM) / Optical Flow:** Multi-frame buffering introduces
unacceptable latency at high speed. Dynamic obstacles violate the constant-velocity
assumption. Excluded.

**Selected architecture:** DepthAnythingV2-Small, INT8 TensorRT engine on Jetson Orin,
ONNX Runtime fallback on Mac/CPU for development.

**Scale source:** For V0 (AirSim development), the AirSim camera config provides a
fixed, known metric scale applied as a global multiplier. A single global scale factor
is sufficient because L1 map persistence corrects peripheral geometry errors over
subsequent frames as the drone moves. VIO-coupled dynamic scale (`MetricScaleEstimator`)
is deferred to VD7.

**Fisheye handling:** No rectify-then-infer. Dewarping kernels consume memory bandwidth,
introduce per-frame latency jitter, and crop edge pixels. The network is trained and
inferred directly on raw distorted frames; the geometric warp is absorbed into the
spatial layers of the network weights. `LensDistortion` (Brown-Conrady or
Kannala-Brandt) is stored in `VisualDepthFrame` for the undistort step in the
unproject kernel only.

### 15.4 Component Interfaces

#### Key types (all in `include/dedalus/sensing/`)

```cpp
// Camera calibration carried per-frame.
struct CameraIntrinsics { float fx, fy, cx, cy; int width, height; };
enum class LensModel { Pinhole, BrownConrady, KannalaBrandt };
struct LensDistortion { LensModel model; float k[6]; };

// Input to the visual depth pipeline.
struct VisualDepthFrame {
    TimePoint timestamp; FrameId source_frame_id;
    CameraIntrinsics intrinsics; LensDistortion distortion;
    int width, height;
    std::vector<uint8_t> rgb;           // H×W×3, row-major
    std::vector<uint8_t> rgb_right;     // empty = monocular (V0)
    float stereo_baseline_m{0.0F};
};

// Single global scale (V0: fixed; V1: VIO-coupled).
struct MetricScaleEstimate {
    float scale{1.0F};       // multiply relative → metric depth
    float confidence{0.0F};  // 0 = invalid; use altitude fallback
    double age_s{0.0};
    bool is_valid() const { return confidence > 0.05F && age_s < 0.5; }
};

// Platform-transparent inference interface.
struct DepthInferenceResult {
    const float* depth_device;    // device ptr valid until next infer_device()
    int width, height;
    TimePoint frame_timestamp;
    TimePoint gimbal_timestamp;   // encoder reading used to build ObstacleSensingVolume
};
class DepthEngineInterface {
public:
    virtual DepthInferenceResult infer_device(const VisualDepthFrame&) = 0;
    virtual std::string name() const = 0;
};

// GPU-side POD result — 48 bytes, no heap pointers, never stored permanently.
struct alignas(16) DeviceObstacleEvidence {
    float cx, cy, cz;       // center_local
    float sx, sy, sz;       // size_m
    float nx, ny, nz;       // surface_normal_local
    float range_m, bearing_rad, elevation_rad, confidence;
    uint8_t state, shape, flags, _pad;  // cast from ObstacleEvidenceState/Shape
};
static_assert(sizeof(DeviceObstacleEvidence) == 48);

// All kernel inputs — pure POD, passed by value, no host ptrs.
struct alignas(16) ProjectionParams {
    float fx, fy, cx, cy;
    float R[9];    // cam→ego rotation (row-major)
    float T[3];    // cam→ego translation
    float metric_scale, min_depth_m, max_depth_m, voxel_size_m;
    int   width, height, stride, max_evidence;
};
```

#### Engine implementations

| Class | Platform | Notes |
|---|---|---|
| `ONNXDepthEngine` | Mac / dev | ONNX Runtime CPU/MPS; identical output shape |
| `TensorRTDepthEngine` | Jetson Orin | Loads `.engine`; INT8 calibrated; DLA optional |
| `AirSimEmulationDepthEngine` | Sim validation | Wraps AirSim GT with optional noise; oracle mode |

Engine files (`.onnx`, `.engine`) are never committed to the repository. They are
generated on target hardware via `tools/perception/export_depth_anything.py` and
`tools/perception/compile_depth_engine.sh`.

### 15.5 Zero-Copy Memory Architecture

The production path guarantees that the full depth buffer (518×518×4 ≈ 1.07 MB) never
crosses the host-device bus. Only the compact `DeviceObstacleEvidence` array
(≤ 1024 × 48 = 49 152 bytes) is read by the CPU.

```
[MIPI CSI / AirSim RGB] ─unified ptr─► [TRT INT8 / ONNX engine]
                                               │ depth_device ptr (stays on device)
                         ┌─────────────────────┘
                         │  project_depth_to_device_evidence()   ─┐
                         │  detect_thin_structures_device()        │ same CUDA stream
                         │  fit_surface_patches_device()          ─┘
                         ▼
                  DeviceEvidenceBuffer[] + LineSegmentBuffer[]
                  (cudaMallocManaged — zero-copy on Jetson unified memory)
                         │
                  cudaStreamSynchronize()   ← single sync point
                         │
                  CPU inflate() ─► ObstacleEvidence[]
                                          │
                  mission_local_obstacle_map_.update()
```

On Jetson Orin: `cudaMallocManaged` allocates from the shared physical memory pool. The
CPU reads the same DRAM the GPU wrote — no DMA, no coherency stall beyond the stream
sync. On a discrete dev GPU (Mac via MPS or remote CUDA): only the 49 KB POD array
is transferred; the depth buffer remains on device.

The `VisualDepthObstacleDetector` owns the device buffers as members, allocated once at
construction and reused across frames.

### 15.6 Thin Structure Detection

Wires, poles, and guy-wires present a catastrophic failure mode for both MDE and SGM.
A 10 mm wire at 20 m projects to 0.2 px on a 400 px focal-length lens — below the
14×14 patch tokenization threshold of any ViT backbone.

A separate parallel kernel path handles thin structures without re-running inference:

1. **Sobel gradient** (CUDA, full-resolution luma channel): computes per-pixel gradient
   magnitude and orientation.
2. **Non-maximum suppression** along gradient direction: thins edges to 1-pixel width.
3. **Connected-component labeling**: groups spatially coherent high-gradient pixels.
4. **Aspect-ratio filter**: components with length/width > threshold are candidate segments.
5. **Depth assignment**: segment midpoint depth from the nearest valid MDE depth cell.
6. **Endpoint unprojection**: produces `endpoint_a_local`, `endpoint_b_local` in world frame.

Output: `ObstacleEvidence{shape=LineSegment, state=ThinStructureRisk,
is_thin_structure_hint=true, radius_m=0.05}`.

No OpenCV anywhere in the production path. The kernel is ~300 lines of CUDA C++;
the CPU fallback uses the same algorithm in scalar form.

### 15.7 Surface and Perch Detection

#### Detector output (SurfacePatch evidence)

When the gimbal tilts toward a floor, rooftop, ledge, or other flat surface, the
projected point cloud contains the geometry. The `fit_surface_patches_device()` kernel
runs alongside the obstacle projection kernel (same stream, same point cloud, no
re-inference):

1. **Normal estimation per projected point**: cross-product of depth-gradient neighbors.
2. **Orientation histogram**: bin normals into coarse (az × el) buckets.
3. **RANSAC per dominant bin** (32 iterations, ~10 candidates): fit plane, count inliers.
4. **Emit per fitted plane**: `ObstacleEvidence{shape=SurfacePatch, state=Occupied,
   has_surface_normal=true, is_surface_hint=true, size_m=inlier_bbox}`.

`state=Occupied` is deliberate: the floor IS an obstacle during normal flight. The
repulsion field correctly pushes the drone away from it. The `is_surface_hint` flag is
neutral — it says "this is a classifiable flat region," not "land here."

**Landability semantics belong in the behavior tree**, not in the detector. The behavior
tree, in landing mode, queries for SurfacePatch evidence where `is_surface_hint=true`
and `surface_normal_local` is within angular tolerance of world-up. APF gain suppression
for the below-horizon sector (allowing descent) is a behavior tree concern triggered by
`AgentLifecycle == Landing`.

#### PerchCandidateEvaluator (non-realtime)

Runs outside the 30 Hz tick loop. Queries L1 for recent SurfacePatch evidence, applies
a multi-factor scoring pipeline, and outputs a ranked `PerchCandidate` list into
`WorldSnapshot`:

| Factor | Source | Threshold |
|---|---|---|
| Normal deviation from world-up | `surface_normal_local` vs IMU attitude | < 15° |
| Minimum area | RANSAC inlier bounding box | > drone footprint + margin |
| Surface roughness | depth variance within inlier set | < σ_max |
| Clearance above | L2 `ray_cast` upward from surface | > rotor gap + margin |
| Depth stability | multi-frame normal consistency (future) | — |

Clearance querying via L2 `ray_cast` is the reason the evaluator is non-realtime: it
requires L2 read access that is too expensive to take every tick. The evaluator runs
opportunistically, triggered when the mission runtime is in a perch-seeking state.

"Perch" rather than "land" is the intentional framing — the drone may settle on rooftops,
ledges, fence posts, or any stable flat projection, not only flat ground.

### 15.8 CoreStackRunner Integration

Minimal additions to the existing runner:

```cpp
// In CoreStackRunnerConfig:
std::shared_ptr<DepthEngineInterface>    visual_depth_engine;   // null = disabled
VisualDepthObstacleDetectorConfig        visual_depth_detector;

// In CoreStackRunner private:
std::optional<VisualDepthObstacleDetector> visual_depth_detector_;
MetricScaleEstimate                        current_scale_estimate_;

// New public method (called by VIO subscriber or fixed at construction):
void update_metric_scale(MetricScaleEstimate);
```

In `run_once()`, both detectors can coexist during validation (VD2–VD3):

```cpp
// GT oracle path (validation only, disabled at VD4):
if (airsim_gt_enabled_ && airsim_frame_available) {
    auto ev = airsim_detector_.detect(gt_frame, sensing_vol);
    mission_local_obstacle_map_.update(ev);  // tagged AirSimGroundTruth
}
// Visual path:
if (visual_depth_detector_ && visual_frame_available) {
    auto ev = visual_depth_detector_->detect(vframe, sensing_vol, current_scale_estimate_);
    mission_local_obstacle_map_.update(ev);  // tagged VisualObstacleDetector
}
```

Evidence from both detectors flows into the same `mission_local_obstacle_map_.update()`
call and is distinguished by `source_kind`. The viewer can display both simultaneously
for delta analysis.

### 15.9 Stage Definitions and Validation Milestones

```
VD1  Types and headers
       New: visual_depth_frame.hpp, metric_scale_estimate.hpp,
            depth_engine.hpp, depth_projection_kernel.hpp
       Changed: occupancy_types.hpp (+is_surface_hint to ObstacleEvidence)
       Milestone: all headers compile; 44+ ctests unchanged.

VD2  ONNXDepthEngine + CPU projection
       New: onnx_depth_engine.cpp, depth_projection_kernel.cpp,
            visual_depth_obstacle_detector.cpp
       New: tools/perception/export_depth_anything.py
       Scale: fixed AirSim camera config value.
       Validation: run both GT and visual detectors; log delta per frame.
       Milestone: ±1 voxel match on major obstacles (walls, buildings)
                  for 90%+ of frames on a standard AirSim scene.

VD3  Surface patch + thin structure (CPU)
       New: fit_surface_patches_device() and detect_thin_structures_device() in
            depth_projection_kernel.cpp. No OpenCV.
       Milestone: AirSim scene with wall + pole → SurfacePatch (wall),
                  ThinStructureRisk LineSegment (pole). Verified in viewer.

VD4  CoreStackRunner integration, GT removal
       Wire visual detector into CoreStackRunner. Disable GT path.
       update_metric_scale() method. Gimbal-aware ProjectionParams.
       Milestone: full AirSim mission completes, visual depth only.
                  L1/L2 maps build; L3 ESDF renders; viewer shows all layers.

VD5  PerchCandidateEvaluator
       Non-realtime evaluator. L1 SurfacePatch + L2 clearance.
       PerchCandidate list in WorldSnapshot. Viewer: green landing pads.
       Milestone: AirSim rooftop identified, ranked, clearance passes.

VD6  CUDA kernel path (Jetson, when hardware available)
       depth_projection_kernel.cu + thin structure .cu + surface patch .cu.
       TensorRTDepthEngine. cudaMallocManaged (zero-copy on Jetson).
       tools/perception/compile_depth_engine.sh (trtexec INT8).
       Milestone: ≥30 Hz on Orin Nano; total latency ≤20 ms.

VD7  VIO scale coupling (deferred — after VD1-VD5 stable)
       MetricScaleEstimator: velocity + feature tracking → MetricScaleEstimate.
       Altitude fallback for invalid VIO state.
```

---

## 14. Summary

Dedalus has crossed from frame/world-model infrastructure into a working live mission loop. The next step is object-conditioned autonomy: the drone should fly because it saw and selected a target, not because it was handed a static trajectory.

The immediate roadmap should stay disciplined:

```text
M3 proves object-conditioned behavior and stable target selection.
M4-M5 add tactical occupancy and avoidance.
M6-M7 add traverse memory and route caching.
M8 adds map + POV visualization.
M9-M10 demonstrate spatial autonomy and site memory.
```

This keeps the architecture modular while moving toward the larger goal: drones that perceive, preserve target identity, choose behavior, avoid obstacles, and improve over repeated flights.
