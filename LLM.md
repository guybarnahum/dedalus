# Dedalus LLM Operating Brief

This file is the active orientation document for a new LLM session. Keep it short, current, and action-oriented. Historical notes and superseded debugging context live in `LLM.back.md`.

Repository:

```text
guybarnahum/dedalus
```

Active milestone state:

```text
Milestones 2.20, 2.21, 2.22, and 2.23 are implemented / validated.

Milestone 2.24 — TargetSelector, ghost validation, reprojection validation, and world-model evidence plumbing.
Status: implemented through 2.24G.9 baseline.

2.24 highlights:
- track-addressable WorldSnapshot agents
- TargetSelectorSpec track_id / agent_id support
- TargetSelector policies and tests
- ghost/scripted targets for pre-camera validation
- WorldSnapshot-to-camera reprojection projector
- projected AG markers in annotation artifacts
- frame_XXXXXX.world_overlay.json sidecars
- camera-derived 2D provenance carried into AgentState.latest_view_evidence
- reprojection residual metadata for camera-derived agents
- CTest layers: contracts -> unit -> synthetic -> scenario

Current prep for Milestone 2.25:
Behavior specs are autonomy configuration and live under `config/behaviors/`, not `simulation/behaviors/`.
Simulation-only fixtures, such as ghost targets, remain under `simulation/ghost_targets/`.

Next milestone:
Milestone 2.25 — ObjectBehaviorMissionController skeleton.
Goal: consume WorldSnapshot + behavior spec + TargetSelector, emit target/behavior mission events, hold/no-op safely during ExecuteMission, and complete through GoHome/Land/Disarm.
```

Patch policy:

```text
Default: apply changes directly to main.
Do not create branches or PRs unless explicitly requested.
Do not leave completed work sitting on a feature branch.
Prefer GitHub connector file updates directly on main when available.
If connector patching fails or is ambiguous, provide an exact manual patch.
```

Validation after code patches:

```bash
cmake --build build-staging -j$(nproc)
ctest --test-dir build-staging --output-on-failure
```

Focused reprojection / world evidence validation:

```bash
ctest --test-dir build-staging --output-on-failure -R 'world_to_image_projector|world_reprojection_artifacts'
```

Scenario/campaign validation:

```bash
ctest --test-dir build-staging --output-on-failure -R 'mission_(scenario|campaign|abort)_'
```

Live mission validation, when AirSim/PX4 is already running:

```bash
RUNS=3 simulation/repeat-mission-smoke.sh

python3 simulation/run-mission-campaign.py \
  --campaign-file config/mission_campaigns/airsim_live_smoke.json \
  --campaign-id live_<N> \
  --output-root out/mission_campaigns \
  --progress \
  --overwrite
```

---

## 1. Current Architecture

Current live mission pipeline:

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
  -> persistent simulation/px4-command-bridge.py
       - PX4 shell: arm, takeoff, land, disarm
       - pymavlink: OFFBOARD mode + SET_POSITION_TARGET_LOCAL_NED velocity
       - LOCAL_POSITION_NED feedback climb to safe height
  -> PX4 / AirSim
```

Milestone 3 target architecture:

```text
AirSim live frame + ego sidecar
  -> AirSimFrameSource
  -> detector / tracker / projector, or ghost/scripted target provider for pre-camera validation
  -> WorldSnapshot agents with agent_id, source_track_id, identity_id, class, confidence, local position, velocity, latest_view_evidence when camera-derived
  -> TargetSelector
  -> BehaviorRuntime / ObjectBehaviorMissionController
  -> desired velocity vector
  -> Px4BridgeCommandSink
  -> PX4 / AirSim
```

Main current app:

```text
apps/dedalus_mission_loop.cpp
```

The old name `dedalus_replay_mission` is obsolete. Do not use it.

---

## 2. Proven Control Path

The working control split deliberately mirrors `simulation/test-flight.py`.

```text
Prepare:
  deterministic AirSim/PX4 session prep
  confirm AirSim connection, GPS validity, API control, and MAVLink reachability

Arm:
  PX4 shell: commander arm

Takeoff:
  PX4 shell: commander takeoff
  bridge waits for takeoff settle

First velocity / climb:
  lazy pymavlink connection after shell takeoff
  prime OFFBOARD velocity stream
  set PX4 OFFBOARD mode
  climb to safe height using LOCAL_POSITION_NED feedback

Trajectory / behavior velocity:
  pymavlink SET_POSITION_TARGET_LOCAL_NED velocity setpoints

Landing:
  PX4 shell: commander land
  wait for ego telemetry / landed height

Complete:
  PX4 shell: commander disarm
```

Core rule:

```text
Synchronous command dispatch success is not vehicle-state truth.
Mission transitions into flight execution must be driven by world-model telemetry.
```

Do not reintroduce the hand-written native C++ MAVLink encoder as the default live path. `src/behavior/px4_mavlink_command_sink.cpp` may remain as experimental/deprecated code, but the live mission should use `flight_command_sink: px4_bridge`.

---

## 3. Object Identity and Evidence Model

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
  Usually local to one tracker session.

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

latest_view_evidence:
  World-model-owned camera-scoped evidence on AgentState.
  Camera-derived agents carry source_frame_id, source_detection_id, source_bbox_px, and source_center_px.
  Ghost/scripted agents normally omit latest_view_evidence because they have no source camera detection.
```

Current implementation rule:

```text
PerceptionPipelineOutput is evidence.
WorldSnapshot is autonomy state.
Behavior consumes WorldSnapshot, not perception internals.
```

Example:

```text
track_id:        ghost_person_001
agent_id:        agent_ghost_person_001
identity_id:     identity_ghost_person_001
source_track_id: ghost_person_001
```

Why this matters:

```text
Target selection must be able to focus on one specific object in a group. It must not switch to another person/car merely because another detection has higher confidence.
```

---

## 4. Behavior Config Location

Behavior specs are global autonomy/runtime configuration, not simulation assets.

Canonical behavior spec location:

```text
config/behaviors/follow_person.yaml
config/behaviors/follow_specific_track.yaml
config/behaviors/circle_car.yaml
config/behaviors/approach_target.yaml
config/behaviors/sequence_approach_circle.yaml
```

Simulation-only target fixtures remain here:

```text
simulation/ghost_targets/person_pair_crossing.yaml
```

Rule:

```text
Behavior spec:
  config/behaviors/*.yaml

Ghost/scripted target scenario:
  simulation/ghost_targets/*.yaml

Core-stack config that references behavior:
  config/core_stack_object_behavior_mission.yaml
```

---

## 5. Ghost / Scripted Target Validation

Before real camera detections are reliable, use ghost targets that enter at the same semantic boundary as real detections.

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

That bypasses the perception/world-model/selector chain and should not be the main validation path.

Meaningful ghost scenario:

```text
Two person targets exist:
  ghost_person_001 confidence 0.82
  ghost_person_002 confidence 0.91

Behavior spec requests ghost_person_001 with policy=persistent_track.
Expected result:
  TargetSelector keeps ghost_person_001 and does not switch to ghost_person_002 merely because confidence is higher.
```

---

## 6. Behavior Language v1

The behavior language is small and declarative:

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

Sample specs live in:

```text
config/behaviors/follow_person.yaml
config/behaviors/follow_specific_track.yaml
config/behaviors/circle_car.yaml
config/behaviors/approach_target.yaml
config/behaviors/sequence_approach_circle.yaml
```

2.23 parser files:

```text
include/dedalus/behavior/behavior_spec.hpp
src/behavior/behavior_spec.cpp
tests/unit/test_behavior_spec.cpp
```

---

## 7. Mission Events and Validation Expectations for M3

Expected later M3 events should include both world-model and tracker handles when available:

```json
{"event":"target_selected","class":"person","agent_id":"agent_ghost_person_001","source_track_id":"ghost_person_001","identity_id":"identity_ghost_person_001","confidence":0.82}
{"event":"behavior_start","behavior":"follow","agent_id":"agent_ghost_person_001","source_track_id":"ghost_person_001"}
{"event":"behavior_complete","behavior":"follow","reason":"duration_elapsed"}
```

M3 validation should prove:

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

---

## 8. Scenario / Campaign Harness

Key files:

```text
simulation/run-mission-scenario.py
simulation/run-mission-campaign.py
simulation/validate-mission-artifacts.py
config/core_stack_synthetic_mission_ci.yaml
config/core_stack_synthetic_mission_abort_ci.yaml
config/mission_campaigns/synthetic_ci.json
config/mission_campaigns/airsim_live_smoke.json
docs/mission_scenario_runner.md
```

Campaign Ctrl-C semantics:

```text
First Ctrl-C:
  active mission receives graceful interrupt
  active mission should GoHome -> Land -> Disarm
  campaign stops after active scenario finishes
  no next repeat starts
  campaign_summary status=interrupted
  process exits 130

Second Ctrl-C:
  force-terminates active child process
```

---

## 9. Known Traps

```text
- Do not use dedalus_replay_mission. Use dedalus_mission_loop.
- Do not treat command helper OK as vehicle-state truth.
- Do not hide arming inside velocity commands.
- Do not collapse flight_control.arm_state and ego.armed.
- Do not move to ExecuteMission until Takeoff is confirmed by ego height.
- Do not make the native C++ MAVLink sink the default live path; use px4_bridge.
- Do not rewrite the working pymavlink control path in C++ while stabilizing behavior.
- Do not let telemetry sidecar and command bridge bind the same MAVLink endpoint.
- Do not let human diagnostics contaminate binary bridge stdout; binary frame bridge stdout is protocol bytes only.
- Do not make mission_campaign_runner CTest execute repeated real missions.
- Do not put obstacle avoidance inside the flight sink.
- Do not let route memory override fresh tactical sensing.
- Do not let Milestone 3 balloon into full obstacle avoidance; M3 is object-conditioned behavior. Avoidance starts post-M3.
- Do not collapse track_id/source_track_id/agent_id/identity_id into one field.
- Do not select targets only by confidence when a stable track/agent target is specified.
- Do not keep global behavior specs under simulation/behaviors; use config/behaviors.
- Do not create branches or PRs unless explicitly requested.
```

---

## 10. Pointers

Read next when relevant:

```text
docs/object_conditioned_behavior_plan.md     detailed M3 behavior + identity plan
docs/world_model_reprojection_validation_plan.md reprojection and world-model evidence plan
docs/mission_scenario_runner.md             scenario/campaign harness
docs/mission_pipeline_current_state.md       mission loop architecture
docs/core_stack_current_state.md             broader core-stack status
WHITEPAPER.md                                architectural rationale
HANDOFF.md                                   handoff prompt template
LLM.back.md                                  historical context only
```