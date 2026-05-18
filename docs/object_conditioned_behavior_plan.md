# Object-Conditioned Behavior Plan

This document is the focused implementation plan for the Milestone 3 object-conditioned behavior arc.

Milestone 2.20 proved that Dedalus can run a repeatable live AirSim/PX4 mission loop. Milestone 2.23 added the behavior-spec parser foundation. Milestone 2.24 starts the target-selection foundation: the system must be able to select one tracked object from a group and keep that target stable across frames.

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

## 4. Object Identity Layers

Do not collapse detections, tracks, agents, and identities. They answer different questions.

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

A behavior spec can select `track_001` and keep that object even if `track_002` has higher confidence. This is the core reason Milestone 2.24 must be track-aware.

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

## 6. Example Behavior Specs

### 6.1 Select by class

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

### 6.2 Select a specific track from a group

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

### 6.3 Circle nearest car

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

### 6.4 Approach then circle

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

## 7. TargetSelector Contract

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
  Match a class group, such as person/car/vehicle.

track_id:
  Prefer/select a specific source_track_id from the tracker.

agent_id:
  Prefer/select a specific world-model agent.

identity_id:
  Future selector for recognized identity. Do not require it for M3.

confidence_min:
  Minimum confidence threshold.

policy:
  Ranking/persistence policy when multiple candidates match.
```

Validation rule:

```text
At least one of class, track_id, or agent_id must be present.
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
- class matches when class is specified
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

## 8. Required WorldSnapshot Agent Fields

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
  "last_seen_timestamp_ns": 1234567890
}
```

Current 2.24A rule:

```text
InMemoryWorldModel derives agent_id and identity_id from Observation3D.track_id, and stores the original tracker ID in AgentState.source_track_id. Snapshot JSON must include source_track_id so artifacts can prove which tracker target became which world-model agent.
```

If true 3D from vision is not ready, M3 may use scripted target positions, AirSim ground-truth projected positions, flat-ground projection, manual/sim hints, or ghost detections. That is acceptable. M3 is about behavior plumbing and velocity control from selected object state, not perfect monocular 3D.

---

## 9. Ghost / Scripted Target Validation Scheme

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
```

---

## 10. Follow Behavior Semantics

Follow means maintaining a relative 3D offset from a selected target.

Given target position/velocity, drone position, relative offset, and reference frame, compute a desired drone position and emit bounded velocity toward it.

For `target_heading_frame`, use target velocity to compute forward/right axes. If target velocity is invalid or near zero, fallback options include last valid target heading, drone-to-target bearing, world-local frame, hold, or reacquire.

---

## 11. Circle Behavior Semantics

Circle means orbiting a selected static or slow target at radius and altitude/height offset. For a slow target, the circle center may be smoothed. M3 can start with static or slow scripted targets.

---

## 12. Approach Behavior Semantics

Approach means moving toward a target until a relative condition is satisfied. For M3, prefer the geometric condition:

```text
distance_to_target <= stop_distance_m
```

Approach should obey standoff and speed constraints. It should not fly directly into the target.

---

## 13. Sequence Behavior Semantics

Sequence composes behaviors, for example:

```text
approach -> circle -> go_home_land
```

Each step should emit `behavior_start` and `behavior_complete`. The sequence advances only when a child behavior returns complete.

---

## 14. Mission Events for M3

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

## 15. M3 Validation Expectations

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
```

---

## 16. Proposed Files

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

simulation/behaviors/follow_person.yaml
simulation/behaviors/circle_car.yaml
simulation/behaviors/approach_target.yaml
simulation/behaviors/sequence_approach_circle.yaml
simulation/ghost_targets/person_pair_crossing.yaml

config/core_stack_object_behavior_mission.yaml
```

Tests:

```text
tests/unit/test_behavior_spec.cpp
tests/unit/test_target_selector.cpp
tests/unit/test_behavior_runtime.cpp
tests/unit/test_object_behavior_mission_controller.cpp
tests/integration/test_object_behavior_mission_smoke.py
```

---

## 17. Non-Goals for M3

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

## 18. Post-M3 Avoidance Boundary

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

## 19. Build and Validation Commands

After code changes:

```bash
cmake --build build-staging -j$(nproc)
ctest --test-dir build-staging --output-on-failure
```

Focused 2.24A checks:

```bash
ctest --test-dir build-staging --output-on-failure -R 'world_snapshot_json|perception_world_model_flow'
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

## 20. Implementation Order

```text
1. BehaviorSpec parser with tests. DONE for 2.23.
2. WorldSnapshot agent track hygiene. STARTED in 2.24A.
3. TargetSelectorSpec fields for track_id / agent_id.
4. TargetSelector with tests over synthetic WorldSnapshot agents.
5. Ghost/scripted target provider for expressive pre-camera validation.
6. BehaviorRuntime for hold/fallback/sequence mechanics.
7. Follow behavior math and unit tests.
8. Circle behavior math and unit tests.
9. Approach behavior math and unit tests.
10. ObjectBehaviorMissionController integration.
11. Mission event extensions.
12. Artifact validator extensions.
13. Live AirSim/PX4 object-conditioned demo.
```

Keep every step artifact-driven and testable.
