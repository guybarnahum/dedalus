# Object-Conditioned Behavior Plan

This document is the focused implementation plan for the Milestone 3 object-conditioned behavior arc.

Milestone 2.20 proved that Dedalus can run a repeatable live AirSim/PX4 mission loop. Milestone 2.23 added the behavior-spec parser foundation. Milestone 2.24 added target-selection foundations, ghost/scripted pre-camera validation, world-model reprojection validation, and camera-scoped view evidence on `WorldSnapshot` agents.

Related detailed plan:

```text
docs/world_model_reprojection_validation_plan.md
```

That reprojection plan is broader than ghost detections. It covers the general perception-validation loop:

```text
2D camera detection / track
  -> projected to 3D ego/local/world state
  -> captured, fused, processed, and enriched by the world model
  -> exposed through WorldSnapshot.agents
  -> reprojected back into the current camera viewport
  -> compared against original or current 2D evidence
```

---

## 1. Milestone 3 Definition

```text
Milestone 3.0 — Object-conditioned flight behavior demo
```

Goal:

```text
A detected/tracked class instance, such as person or car,
  -> is represented as a WorldSnapshot AgentState with stable track provenance
  -> is selected by a TargetSelector
  -> drives follow, circle, approach, or sequence behavior
  -> emits bounded velocity vectors through the existing PX4 bridge path
  -> returns home, lands, and disarms
  -> is validated through mission_events + snapshots
```

Milestone 3 is not an obstacle-avoidance milestone. Avoidance starts after M3 by inserting a planner between behavior and the flight sink.

---

## 2. Current Control Boundary

The current live flight path is:

```text
MissionRuntime
  -> MissionController
  -> VelocityCommand
  -> Px4BridgeCommandSink
  -> simulation/px4-command-bridge.py
  -> PX4 / AirSim
```

The bridge owns the proven PX4/MAVLink transport behavior:

```text
PX4 shell:
  arm / takeoff / land / disarm

pymavlink:
  heartbeat
  OFFBOARD priming
  OFFBOARD mode set
  LOCAL_POSITION_NED feedback climb
  SET_POSITION_TARGET_LOCAL_NED velocity setpoints
```

Object-conditioned behavior must use this existing velocity-command path. Do not put behavior logic into `px4-command-bridge.py`, `Px4BridgeCommandSink`, or PX4 shell helpers. Those layers are transport/sink layers. They should receive bounded kinematic intent only.

---

## 3. Proposed M3 Runtime Architecture

```text
WorldSnapshot
  -> TargetSelector
  -> BehaviorRuntime
  -> ObjectBehaviorMissionController
  -> desired velocity vector
  -> Px4BridgeCommandSink
  -> PX4 / AirSim
```

Future post-M3 architecture:

```text
WorldSnapshot
  -> TargetSelector
  -> BehaviorRuntime
  -> ObjectBehaviorMissionController
  -> desired velocity vector
  -> TacticalAvoidancePlanner
  -> safe velocity vector
  -> Px4BridgeCommandSink
  -> PX4 / AirSim
```

The M3 implementation should be structured so the post-M3 avoidance planner can be inserted without rewriting behavior controllers.

---

## 4. Object Identity and Evidence Layers

Do not collapse detections, tracks, agents, identities, or view evidence. They answer different questions.

```text
detection_id  ->  track_id  ->  agent_id  ->  identity_id
single frame      tracker       world model    recognized identity
```

Definitions:

```text
detection_id:
  A single detector observation in one frame.

track_id:
  Tracker-owned frame-to-frame continuity for one moving blob/object.
  It is usually local to one tracker session and may reset after tracker restart or long target loss.

source_track_id:
  The tracker ID preserved inside AgentState as provenance.
  It answers: which tracker track produced this world-model agent?

agent_id:
  World-model-owned object handle.
  Behavior and planning should select agents, not raw detections.
  Today it may be derived from source_track_id; later it can represent a fused object from multiple sensors/tracks.

identity_id:
  Identity-resolver-owned recognized real-world identity.
  It may eventually represent a known person, vehicle plate, drone serial, or long-lived cross-mission identity.

latest_view_evidence:
  World-model-owned camera-scoped evidence on AgentState.
  Camera-derived agents carry source_frame_id, source_detection_id, source_bbox_px, and source_center_px.
  Ghost/scripted agents normally omit it because they have no source camera detection.
```

Architecture rule:

```text
PerceptionPipelineOutput is evidence.
WorldSnapshot is autonomy state.
Behavior consumes WorldSnapshot, not perception internals.
```

Why all are needed:

```text
track_id:
  what the tracker followed frame-to-frame

source_track_id:
  provenance from tracker into world-model artifacts

agent_id:
  the object handle autonomy should reason about

identity_id:
  who/what the object is believed to be, when recognition is strong enough

latest_view_evidence:
  what camera evidence most recently contributed to the world-model agent
```

Concrete example:

```text
Tracker sees:
  track_001: person, confidence 0.82
  track_002: person, confidence 0.91

WorldSnapshot agents:
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

A behavior spec can select `track_001` and keep that object even if `track_002` has higher confidence. This is the core reason Milestone 2.24 had to be track-aware.

---

## 5. Behavior Spec v1

The behavior language is small, declarative, and config-driven.

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

Reference frames:

```text
target_heading_frame
world_local_frame
drone_heading_frame
camera_frame
```

For M3, implement only what is needed for a stable demo. Prefer `target_heading_frame` and `world_local_frame` first.

---

## 6. Behavior Config Location

Behavior specs are autonomy/runtime configuration, not simulation assets.

Canonical behavior specs live under:

```text
config/behaviors/
```

Current sample specs:

```text
config/behaviors/follow_person.yaml
config/behaviors/follow_specific_track.yaml
config/behaviors/circle_car.yaml
config/behaviors/approach_target.yaml
config/behaviors/sequence_approach_circle.yaml
```

Simulation-specific target fixtures remain under:

```text
simulation/ghost_targets/person_pair_crossing.yaml
```

Design rule:

```text
Behavior spec:
  config/behaviors/*.yaml

Ghost/scripted target scenario:
  simulation/ghost_targets/*.yaml

Core-stack config that references behavior:
  config/core_stack_object_behavior_mission.yaml
```

This matters because the same behavior spec should be usable against live AirSim, recorded frames, synthetic CI, and future real hardware. Simulation may provide test targets/scenarios, but it should not own the behavior language.

---

## 7. Example Behavior Specs

### 7.1 Select by class

```yaml
mission:
  name: follow_person_demo

target:
  selector:
    class: person
    confidence_min: 0.55
    policy: highest_confidence
    reacquire_timeout_s: 5.0

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

### 7.2 Select a specific track from a group

```yaml
mission:
  name: follow_specific_person_demo

target:
  selector:
    class: person
    track_id: ghost_person_001
    confidence_min: 0.55
    policy: persistent_track
    reacquire_timeout_s: 5.0

behavior:
  type: follow
  target_frame: target_heading_frame
  relative_offset_m:
    x: -8.0
    y: 0.0
    z: 4.0
  max_speed_mps: 2.0
```

Meaning:

```text
Follow the specific tracked person `ghost_person_001`, not merely the highest-confidence person.
```

### 7.3 Circle nearest car

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
  max_speed_mps: 3.0

completion:
  after_s: 30
  then: go_home_land
```

### 7.4 Approach then circle

```yaml
mission:
  name: approach_then_circle_demo

target:
  selector:
    class: vehicle
    confidence_min: 0.6
    policy: nearest

behavior:
  type: sequence
  steps:
    - type: approach
      stop_distance_m: 8.0
      altitude_offset_m: 4.0
      max_speed_mps: 2.0
    - type: circle
      radius_m: 10.0
      altitude_offset_m: 5.0
      angular_speed_deg_s: 10.0
      duration_s: 20.0
    - type: go_home_land
```

---

## 8. TargetSelector Contract

The TargetSelector converts `WorldSnapshot.agents` into a selected target.

Input:

```text
WorldSnapshot
TargetSelectorSpec
previous TargetSelection, if any
current time
```

Output:

```text
TargetSelection
  selected: true/false
  status: selected / reacquiring / lost / no_candidates / invalid_spec
  agent_id
  source_track_id
  identity_id
  class_label
  confidence
  position_local_m
  velocity_local_mps
  target_age_s
  reason
```

Selector fields:

```text
class:
  Required class group, such as person/car/vehicle. Behavior semantics are class-dependent.

track_id:
  Prefer/select a specific source_track_id from the tracker inside the class group.

agent_id:
  Prefer/select a specific world-model agent inside the class group.

identity_id:
  Future selector for recognized identity. Do not require it for M3.

confidence_min:
  Minimum confidence threshold.

policy:
  Ranking/persistence policy when multiple candidates match.
```

Validation rule:

```text
class is required. track_id and agent_id are optional specificity handles within that class.
```

Initial selection policies:

```text
highest_confidence:
  Select the valid matching agent with the highest confidence.

nearest:
  Select the valid matching agent closest to ego.local_T_body.position.

persistent_track:
  If the previous selected agent/source_track_id remains valid, keep it even if another object has higher confidence.
  Otherwise reacquire according to a fallback ranking policy.
```

Filtering rules:

```text
- lifecycle must be New or Active initially
- confidence >= confidence_min
- class matches
- source_track_id matches when track_id is specified
- agent_id matches when agent_id is specified
- map/local position must be usable
```

Target loss semantics:

```text
selected:
  A valid target is selected.

reacquiring:
  Previous target is missing but still within reacquire_timeout_s.

lost:
  Previous target has been missing beyond reacquire_timeout_s.

no_candidates:
  No matching target exists and there was no active previous target.
```

Do not silently continue flying toward stale target positions without a clear timeout and event.

---

## 9. Required WorldSnapshot Agent Fields

M3 behavior needs enough target state to produce local velocity commands and stable target selection.

Minimum useful agent state:

```json
{
  "agent_id": "agent_track_001",
  "source_track_id": "track_001",
  "identity_id": "identity_track_001",
  "class": "person",
  "confidence": 0.82,
  "position_local": [12.0, 4.0, 0.0],
  "velocity_local": [0.5, 0.0, 0.0],
  "lifecycle": "active",
  "last_seen_timestamp_ns": 1234567890,
  "latest_view_evidence": {
    "source_frame_id": "synthetic_mission_3",
    "source_detection_id": "det_0001",
    "source_center_px": [300.0, 250.0]
  }
}
```

Current rule:

```text
InMemoryWorldModel derives agent_id and identity_id from Observation3D.track_id, stores the original tracker ID in AgentState.source_track_id, and stores camera-scoped source evidence in AgentState.latest_view_evidence when the agent came from camera detections. Snapshot JSON includes source_track_id and latest_view_evidence so artifacts can prove which tracker target became which world-model agent and which camera evidence contributed to it.
```

If true 3D from vision is not ready, M3 may use scripted target positions, AirSim ground-truth projected positions, flat-ground projection, manual/sim hints, or ghost detections. That is acceptable. M3 is about behavior plumbing and velocity control from selected object state, not perfect monocular 3D.

---

## 10. Ghost / Scripted Target Validation Scheme

Before real camera detections are reliable, validation should use ghost detections that enter at the same semantic boundary as real detections.

Good validation path:

```text
Synthetic / AirSim frame
  -> GhostDetectionProvider or ScriptedTargetProvider
  -> PerceptionPipelineOutput.observations
  -> InMemoryWorldModel
  -> WorldSnapshot.agents with stable source_track_id
  -> TargetSelector
  -> ObjectBehaviorMissionController later
```

Bad shortcut:

```text
config says selected_target = ghost_person_001
```

That bypasses perception/world-model/selector plumbing and should not be used as the main validation path.

Example ghost target scenario:

```yaml
scenario:
  name: person_pair_crossing

targets:
  - track_id: ghost_person_001
    class: person
    confidence: 0.82
    trajectory:
      type: linear
      start_local_m: [12.0, -4.0, 0.0]
      velocity_local_mps: [0.3, 0.0, 0.0]

  - track_id: ghost_person_002
    class: person
    confidence: 0.91
    trajectory:
      type: linear
      start_local_m: [8.0, 4.0, 0.0]
      velocity_local_mps: [-0.2, 0.0, 0.0]
```

A behavior spec can then target `ghost_person_001`. A meaningful validation proves that Dedalus keeps the selected lower-confidence track instead of switching to the higher-confidence person.

Validation levels:

```text
Level 1 — Pure unit tests:
  Construct WorldSnapshot by hand and validate TargetSelector policy/persistence.

Level 2 — Synthetic pipeline tests:
  Feed ghost observations into InMemoryWorldModel and validate agent IDs, source_track_id artifacts, and selector output.

Level 3 — Mission scenario tests:
  Run dedalus_mission_loop with ghost targets enabled and validate mission_events + snapshots contain target_selected and stable target identity.

Level 4 — Visual artifact tests:
  Render WorldSnapshot agents back onto captured camera frames and export deterministic annotated frame artifacts, sidecar JSON, and MP4 clips.

Level 5 — AirSim viewport debug overlay:
  Optionally mirror WorldSnapshot agents back into the AirSim/Unreal viewport using debug markers or spawned debug actors. This is for operator visibility only; the authoritative validation artifacts remain mission_events, snapshots, and Dedalus-generated annotated video.
```

---

## 11. World-Model Visualization and Recording Plan

World-model state should be visible at three distinct levels, with clear separation between validation artifacts and simulator/operator display.

```text
1. Snapshot artifacts:
   snapshot_XXXX.json records WorldSnapshot.agents, source_track_id, agent_id, class, confidence, local position, velocity, and latest_view_evidence when camera-derived.

2. Dedalus annotated camera artifacts:
   FrameAnnotationSink draws world-model agents, selected target state, behavior state, and diagnostics onto captured camera frames.
   PPM sequence output is the deterministic CI-friendly path.
   frame_XXXXXX.world_overlay.json records projected coordinates, visibility/reason, depth/range/bearing, source 2D evidence, and residuals when available.
   MP4 recording/export should be supported for human review when the environment has the required encoder tooling.

3. AirSim viewport overlay:
   A future AirSimWorldAnnotationSink mirrors selected WorldSnapshot state back into the simulator view using AirSim debug APIs or lightweight spawned debug actors.
   This must be an optional adapter and must not change autonomy semantics or contaminate binary bridge stdout.
```

Design rules:

```text
- WorldSnapshot remains the source of truth.
- Annotators consume WorldSnapshot; they do not modify mission state.
- Reprojection is a general perception-validation layer, not a ghost-only visualization feature.
- AirSim viewport annotation is operator/debug display only, not validation truth.
- MP4 files are review artifacts; JSON snapshots and mission_events remain validation truth.
- Keep Dedalus artifact annotation working without live AirSim.
```

Planned visual overlays:

```text
Detector evidence:
  D: 2D detector boxes and detection IDs

Tracker evidence:
  T: 2D tracks and track IDs

World agents:
  AG: reprojected WorldSnapshot agent with source_track_id, agent_id, class, confidence, local position/velocity, depth, and reprojection residual when available

Selection:
  SEL: selected target, selection reason, target age, reacquiring/lost state

Behavior:
  BH: behavior_start/current behavior, desired velocity vector, completion/fallback state

Mission:
  mission phase, arm/takeoff/land/disarm status, safe-height gate, terminal_settled state
```

See `docs/world_model_reprojection_validation_plan.md` for the detailed projection/reprojection design, including the stationary-object / moving-drone stress case.

---

## 12. Follow Behavior Semantics

Follow means maintaining a relative 3D offset from a selected target.

Given target position/velocity, drone position, relative offset, and reference frame, compute a desired drone position and emit bounded velocity toward it.

For `target_heading_frame`, use target velocity to compute forward/right axes. If target velocity is invalid or near zero, fallback options include last valid target heading, drone-to-target bearing, world-local frame, hold, or reacquire.

---

## 13. Circle Behavior Semantics

Circle means orbiting a selected static or slow target at radius and altitude/height offset. For a slow target, the circle center may be smoothed. M3 can start with static or slow scripted targets.

---

## 14. Approach Behavior Semantics

Approach means moving toward a target until a relative condition is satisfied. For M3, prefer the geometric condition:

```text
distance_to_target <= stop_distance_m
```

Approach should obey standoff and speed constraints. It should not fly directly into the target.

---

## 15. Sequence Behavior Semantics

Sequence composes behaviors, for example:

```text
approach -> circle -> go_home_land
```

Each step should emit `behavior_start` and `behavior_complete`. The sequence advances only when a child behavior returns complete.

---

## 16. Mission Events for M3

Recommended event types:

```text
target_selected
target_reacquired
target_lost
behavior_start
behavior_tick_sample
behavior_complete
behavior_failed
fallback_start
fallback_complete
```

Events should include both world-model and tracker handles when available:

```json
{"event":"target_selected","class":"person","agent_id":"agent_ghost_person_001","source_track_id":"ghost_person_001","identity_id":"identity_ghost_person_001","confidence":0.82}
```

Do not rely on console prints for validation. The event log is the durable artifact.

---

## 17. M3 Validation Expectations

Required checks:

```text
final_state == Complete
failures == 0
safe height reached before behavior execution
at least one target_selected event
selected target identity remains stable unless reacquire/lost is expected
at least one behavior_start event
expected behavior_complete event
velocity commands emitted during behavior
GoHome / Land / Complete reached
```

Useful optional checks:

```text
target remained selected long enough
follow offset error converged below tolerance
circle radius stayed within tolerance
approach stopped near standoff distance
fallback was not triggered unless expected
annotated frame artifacts include selected target overlays
MP4 review artifact generated when encoder tooling is available
AirSim viewport debug overlay shows selected target when enabled
world-model agent reprojection residuals stay within expected tolerance for camera-derived detections
stationary-object / moving-drone reprojection stress test passes
```

---

## 18. Proposed Files

Initial implementation files:

```text
include/dedalus/behavior/behavior_spec.hpp
include/dedalus/behavior/target_selector.hpp
include/dedalus/behavior/behavior_runtime.hpp
include/dedalus/behavior/object_behavior_mission_controller.hpp

src/behavior/behavior_spec.cpp
src/behavior/target_selector.cpp
src/behavior/behavior_runtime.cpp
src/behavior/object_behavior_mission_controller.cpp

config/behaviors/follow_person.yaml
config/behaviors/follow_specific_track.yaml
config/behaviors/circle_car.yaml
config/behaviors/approach_target.yaml
config/behaviors/sequence_approach_circle.yaml
simulation/ghost_targets/person_pair_crossing.yaml

config/core_stack_object_behavior_mission.yaml
```

Visibility/recording files:

```text
include/dedalus/visualization/world_to_image_projector.hpp
src/visualization/world_to_image_projector.cpp
include/dedalus/visualization/world_overlay_sidecar.hpp
src/visualization/world_overlay_sidecar.cpp
simulation/airsim-world-annotation.py
```

Tests:

```text
tests/unit/test_behavior_spec.cpp
tests/unit/test_target_selector.cpp
tests/unit/test_behavior_runtime.cpp
tests/unit/test_object_behavior_mission_controller.cpp
tests/unit/test_world_to_image_projector.cpp
tests/integration/test_object_behavior_mission_smoke.py
tests/integration/test_world_reprojection_artifacts.py
```

---

## 19. Non-Goals for M3

Do not make M3 own:

```text
full obstacle avoidance
tactical voxel/cone occupancy mapping
persistent traverse maps
route solution caching
site memory
native C++ PX4/MAVLink rewrite
real monocular 3D perfection
multi-target assignment
multi-drone coordination
```

M3 should stay focused:

```text
Select one detected/tracked class instance and fly a behavior relative to it.
```

---

## 20. Post-M3 Avoidance Boundary

Post-M3, add avoidance here:

```text
ObjectBehaviorMissionController
  -> desired velocity vector
  -> TacticalAvoidancePlanner
  -> safe velocity vector
  -> Px4BridgeCommandSink
```

Do not push avoidance into the flight sink. Fresh tactical sensing overrides persistent memory and cached routes.

---

## 21. Build and Validation Commands

After code changes:

```bash
cmake --build build-staging -j$(nproc)
ctest --test-dir build-staging --output-on-failure
```

Focused TargetSelector checks:

```bash
ctest --test-dir build-staging --output-on-failure -R 'world_snapshot_json|perception_world_model_flow|target_selector|core_stack_config_loader'
```

After visual artifact / reprojection changes:

```bash
ctest --test-dir build-staging --output-on-failure -R 'world_to_image_projector|world_reprojection_artifacts|ppm_frame_annotation_sink'
```

After live mission behavior changes:

```bash
RUNS=3 simulation/repeat-mission-smoke.sh
```

After object-conditioned behavior exists:

```bash
./build-staging/apps/dedalus_mission_loop \
  --config config/core_stack_object_behavior_mission.yaml \
  --output-dir out/object_behavior_mission \
  --max-frames 900 \
  --shutdown-max-frames 400 \
  --progress

python3 simulation/validate-mission-artifacts.py out/object_behavior_mission --expect-final-state Complete --expect-behavior
```

---

## 22. Implementation Order

```text
1. BehaviorSpec parser with tests. DONE for 2.23.
2. WorldSnapshot agent track hygiene. DONE for 2.24A.
3. TargetSelectorSpec fields for track_id / agent_id. DONE for 2.24B.
4. TargetSelector with tests over synthetic WorldSnapshot agents. DONE for 2.24C.
5. Ghost/scripted target provider for expressive pre-camera validation. DONE for 2.24D.
6. Runtime ghost target injection + artifact visibility. DONE for 2.24E.
7. WorldSnapshot-to-annotation overlays for agents/selection/behavior and MP4 review export. DONE for 2.24F baseline.
8. World-model reprojection validation: project WorldSnapshot agents back into camera viewport and measure residuals. DONE for 2.24G baseline.
9. Behavior config location cleanup: canonical behavior specs under config/behaviors, simulation fixtures stay under simulation. DONE as 2.25 prep.
10. ObjectBehaviorMissionController skeleton.
11. BehaviorRuntime for hold/fallback/sequence mechanics.
12. Follow behavior math and unit tests.
13. Circle behavior math and unit tests.
14. Approach behavior math and unit tests.
15. Mission event extensions.
16. Artifact validator extensions.
17. Live AirSim/PX4 object-conditioned demo.
```

Keep every step artifact-driven and testable.
