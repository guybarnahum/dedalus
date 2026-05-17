# Object-Conditioned Behavior Plan

This document is the focused implementation plan for the Milestone 3 object-conditioned behavior arc.

It complements:

```text
LLM.md
WHITEPAPER.md
HANDOFF.md
docs/core_stack_current_state.md
docs/mission_pipeline_current_state.md
```

Milestone 2.20 proved that Dedalus can run a repeatable live AirSim/PX4 mission loop. Milestone 3 should prove that the drone can fly based on detected/tracked object classes or instances.

---

## 1. Milestone 3 Definition

```text
Milestone 3.0 — Object-conditioned flight behavior demo
```

Goal:

```text
A detected/tracked class instance, such as person or car,
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

Object-conditioned behavior must use this existing velocity-command path.

Do not put behavior logic into:

```text
px4-command-bridge.py
Px4BridgeCommandSink
PX4 shell helpers
```

Those layers are transport/sink layers. They should receive bounded kinematic intent only.

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

## 4. Behavior Spec v1

The behavior language should be small, declarative, and config-driven.

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

Reference frames:

```text
target_heading_frame
  Offset relative to target direction of travel.
  Useful for following behind or beside a moving car/person.

world_local_frame
  Offset fixed in takeoff-local/map coordinates.
  Useful for early demos and scripted/simulated targets.

drone_heading_frame
  Offset relative to current drone heading.
  Useful later for camera/egocentric behavior.

camera_frame
  Offset based on image bearing / drone POV.
  Useful later for visual servoing.
```

For M3, implement only what is needed for a stable demo. Prefer:

```text
target_heading_frame
world_local_frame
```

---

## 5. Example Behavior Specs

### 5.1 Follow person

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

Meaning:

```text
Select a person, then keep the drone roughly 8m behind and 4m above that target.
```

### 5.2 Circle car

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

completion:
  after_s: 30
  then: go_home_land
```

Meaning:

```text
Select a car, then orbit it at 10m radius and 5m altitude offset.
```

### 5.3 Approach then circle

```yaml
mission:
  name: approach_then_circle_car_demo

target:
  selector:
    class: car
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

Meaning:

```text
Select a car, approach to a standoff distance, circle it, then return and land.
```

---

## 6. TargetSelector Contract

The TargetSelector converts `WorldSnapshot` agents into a selected target.

Input:

```text
WorldSnapshot
TargetSelectorSpec
current selected target, if any
current time
```

Output:

```text
TargetSelection
  selected: true/false
  target_id
  class_label
  confidence
  position_local_m
  velocity_local_mps
  target_age_s
  status
```

Initial selection policies:

```text
highest_confidence
nearest
persistent_track
```

### 6.1 `highest_confidence`

Select the valid agent of the requested class with the highest confidence.

Useful for early demos where detections are clean and position is approximate.

### 6.2 `nearest`

Select the valid agent of the requested class closest to the drone.

Useful for approach/circle demos when multiple objects exist.

### 6.3 `persistent_track`

Prefer the previously selected track while it remains valid. If lost beyond timeout, reacquire according to a fallback policy.

Useful for stable follow/circle behavior.

### 6.4 Target loss semantics

Target loss should be explicit and evented.

Possible states:

```text
selected
reacquiring
lost
fallback_started
```

Target loss policy examples:

```yaml
fallback:
  on_target_lost: hold_then_go_home
  hold_s: 5.0
```

or:

```yaml
fallback:
  on_target_lost: search_then_go_home
  search_s: 10.0
```

Do not silently continue flying toward stale target positions without a clear timeout and event.

---

## 7. Required WorldSnapshot Agent Fields

M3 behavior needs enough target state to produce local velocity commands.

Minimum useful agent state:

```json
{
  "id": "agent_1",
  "class_label": "person",
  "confidence": 0.82,
  "position_local_m": [12.0, 4.0, 0.0],
  "position_valid": true,
  "velocity_local_mps": [0.5, 0.0, 0.0],
  "velocity_valid": true,
  "last_seen_timestamp_ns": 1234567890
}
```

If true 3D from vision is not ready, M3 may use:

```text
scripted target positions
AirSim ground-truth projected positions
flat-ground projection
manual/sim hints
```

That is acceptable. M3 is about behavior plumbing and velocity control from selected object state, not perfect monocular 3D.

---

## 8. Follow Behavior Semantics

Follow means:

```text
Maintain a relative 3D offset from a selected target.
```

Given:

```text
target_position_local
target_velocity_local, if valid
drone_position_local
relative_offset_m
reference frame
```

Compute:

```text
desired_drone_position_local
position_error = desired_drone_position_local - drone_position_local
desired_velocity_local = bounded_gain(position_error)
```

For `target_heading_frame`:

```text
forward = normalize(target_velocity_local.xy)
right = rotate_90deg(forward)
up/down follows local z convention
```

If target velocity is invalid or near zero, fallback options:

```text
use last valid target heading
use drone-to-target bearing
use world_local_frame
hold/reacquire
```

Velocity output should be bounded by behavior constraints:

```text
max_speed_mps
max_vertical_speed_mps
position_tolerance_m
minimum_standoff_m
```

---

## 9. Circle Behavior Semantics

Circle means:

```text
Orbit a selected static or slow target at radius R and altitude/height offset.
```

Given:

```text
target_position_local
drone_position_local
radius_m
altitude_offset_m
angular_speed_deg_s
direction
phase
```

Compute:

```text
phase += angular_speed * dt
desired_position.x = center.x + radius * cos(phase)
desired_position.y = center.y + radius * sin(phase)
desired_position.z = target_z - altitude_offset_or_height_policy
velocity = bounded_gain(desired_position - drone_position)
```

For a slow-moving target, the circle center may be smoothed:

```yaml
center_tracking:
  enabled: true
  smoothing_s: 2.0
  max_center_speed_mps: 1.0
```

M3 can start with static or slow scripted targets. Full fast-moving orbit behavior can come later.

---

## 10. Approach Behavior Semantics

Approach means:

```text
Move toward a target until a relative condition is satisfied.
```

Typical completion conditions:

```text
distance_to_target <= stop_distance_m
relative offset reached
target centered in image
target confidence above threshold
time budget expired
```

For M3, prefer the geometric condition:

```text
distance_to_target <= stop_distance_m
```

Then transition into another behavior, such as:

```text
circle
follow
hold
go_home_land
```

Approach should obey standoff and speed constraints. It should not fly directly into the target.

---

## 11. Sequence Behavior Semantics

Sequence composes behaviors.

Example:

```text
approach -> circle -> go_home_land
```

Each step should emit:

```text
behavior_start
behavior_complete
```

The sequence should advance only when a child behavior returns complete. If a child behavior fails, the sequence should enter fallback or abort according to spec.

---

## 12. Mission Events for M3

The existing `mission_events.jsonl` should be extended with object-behavior events.

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

Examples:

```json
{"event":"target_selected","class":"car","track_id":"agent_3","confidence":0.86}
{"event":"behavior_start","behavior":"approach","target":"agent_3"}
{"event":"behavior_complete","behavior":"approach","reason":"standoff_reached"}
{"event":"behavior_start","behavior":"circle","target":"agent_3"}
{"event":"behavior_complete","behavior":"circle","reason":"duration_elapsed"}
{"event":"target_lost","target":"agent_3","age_s":5.2}
{"event":"fallback_start","policy":"hold_then_go_home","reason":"target_lost"}
```

Do not rely on console prints for validation. The event log is the durable artifact.

---

## 13. M3 Validation Expectations

The mission artifact validator should be able to check object-conditioned behavior runs.

Required checks:

```text
final_state == Complete
failures == 0
safe height reached before behavior execution
at least one target_selected event
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

## 14. Proposed Files

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
simulation/behaviors/approach_then_circle.yaml

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

Validator extension:

```text
simulation/validate-mission-artifacts.py
```

If `validate-mission-artifacts.py` does not exist yet, implement it in Milestone 2.21 before M3 behavior work starts.

---

## 15. Non-Goals for M3

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

## 16. Post-M3 Avoidance Boundary

Post-M3, add avoidance here:

```text
ObjectBehaviorMissionController
  -> desired velocity vector
  -> TacticalAvoidancePlanner
  -> safe velocity vector
  -> Px4BridgeCommandSink
```

Do not push avoidance into the flight sink. The sink remains a transport abstraction.

Post-M3 spatial autonomy sequence:

```text
4.0 Local tactical occupancy map
5.0 Reactive obstacle avoidance planner
6.0 Persistent traverse map / flight memory
7.0 Cached route solutions
8.0 Tactical map + drone POV visualization
9.0 Integrated spatial autonomy demo with avoidance
10.0 Multi-flight site memory
```

Important rule:

```text
Fresh tactical sensing overrides persistent memory and cached routes.
```

---

## 17. Build and Validation Commands

After code changes:

```bash
cmake --build build-staging -j$(nproc)
ctest --test-dir build-staging --output-on-failure
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

python3 simulation/validate-mission-artifacts.py out/object_behavior_mission --expect-complete --expect-behavior
```

---

## 18. Implementation Order

Recommended order after 2.21/2.22 infrastructure:

```text
1. BehaviorSpec parser with tests.
2. TargetSelector with tests over synthetic WorldSnapshot agents.
3. BehaviorRuntime for hold/fallback/sequence mechanics.
4. Follow behavior math and unit tests.
5. Circle behavior math and unit tests.
6. Approach behavior math and unit tests.
7. ObjectBehaviorMissionController integration.
8. Scripted/simulated target scenario.
9. Mission event extensions.
10. Artifact validator extensions.
11. Live AirSim/PX4 object-conditioned demo.
```

Keep every step artifact-driven and testable.
