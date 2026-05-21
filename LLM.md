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

Milestone 2.25 — ObjectBehaviorMissionController skeleton and first object-conditioned behavior runtime.
Status: implemented through object behavior lifecycle and bounded follow behavior baseline.

Milestone 2.26 — AirSim ghost behavior validation and live runtime-event plumbing.
Status: implemented through 2.26D.7 baseline.
```

Current 2.26D state:

```text
Ghost simulation:
  simulation/ghost_detections/person_pair_crossing.json
    -> references simulation/trajectories/*.json
    -> loaded by GhostScenario
    -> evaluated by VelocityTrajectory-backed motion

Ghost people now move cross -> wait -> cross back -> wait.

Mission-loop ghost injection and publication:
  CoreStackRunner
    -> GhostTargetProvider::frame_at(...)
       -> one GhostScenario evaluation at frame time
       -> GhostDetectionsFrame event for live stream/debug subscribers
       -> Observation3D list injected into PerceptionPipelineOutput.observations
    -> InMemoryWorldModel
    -> WorldSnapshot.agents

Runtime publish path:
  CoreStackRunner
    -> GhostDetectionsPublisher
       -> RuntimeEventStreamServer          optional TCP JSONL stream, event type ghost_detections
    -> WorldSnapshotPublisher
       -> LatestWorldSnapshotSubscriber     behavior / MissionRuntime
       -> ArtifactSnapshotWriter            durable evidence/debug snapshots
       -> RuntimeEventStreamServer          optional TCP JSONL stream, event type world_snapshot
  MissionRuntime
    -> MissionEventPublisher
       -> mission_events.jsonl              durable evidence/debug event log
       -> RuntimeEventStreamServer          optional TCP JSONL stream, event type mission_event

Reusable event service:
  include/dedalus/runtime/pubsub.hpp
    EventPublisher<T>
    EventSubscriber<T>

WorldSnapshot domain adapter:
  include/dedalus/world_model/world_snapshot_publisher.hpp
    WorldSnapshotPublisher = EventPublisher<WorldSnapshot>
    WorldSnapshotSubscriber keeps on_snapshot(...) while adapting to generic on_event(...)

Ghost detections domain adapter:
  include/dedalus/perception/ghost_targets.hpp
    GhostDetectionsPublisher = EventPublisher<GhostDetectionsFrame>
    GhostDetectionsSubscriber keeps on_ghost_detections(...) while adapting to generic on_event(...)

Mission events domain adapter:
  include/dedalus/behavior/mission_runtime.hpp
    MissionEventPublisher = EventPublisher<MissionEvent>
    MissionEventSubscriber keeps on_mission_event(...) while adapting to generic on_event(...)

Live runtime event stream:
  dedalus_mission_loop --world-snapshot-stream-port 47770
  emits JSONL records on one TCP stream:
    {"type":"ghost_detections","seq":N,"timestamp_ns":...,"map_frame_id":"...","ghost_detections":{...}}
    {"type":"world_snapshot","seq":N,"timestamp_ns":...,"active_map_frame_id":"...","snapshot":{...}}
    {"type":"mission_event","seq":N,"timestamp_ns":...,"mission_event":{...}}

AirSim overlay:
  simulation/airsim-world-overlay.py is now a stream-only subscriber/renderer.
  It no longer evaluates ghost scenarios locally and no longer reads snapshot_manifest.txt.
  It renders PLAN / PLAN* from ghost_detections, AG / EGO from world_snapshot, and SEL from mission_event target_selected matched against the latest world_snapshot agent.

Artifact files remain evidence/debug outputs, not IPC.
```

Next milestone slice:

```text
2.26D.8:
  prune naming drift around world_snapshot_stream_server file/test names if desired, or keep compatibility if churn outweighs value.

Post-2.26D:
  continue behavior-facing dynamic ghost validation and then expand behavior math beyond follow.
```

---

## 0. Engineering Hygiene Policy

Use this policy for every implementation pass, especially after refactors.

```text
Review the implementation and look for legacy pre-refactor leftovers to clean up.
Always strive for state-of-the-art clean code, carefully balancing code structure, runtime efficiency, and complexity.
Do not carry legacy code through shims or other inelegant solutions due to momentum.
When appropriate, suggest and perform refactors that avoid drift, duplication, and bloat.
Keep code streamlined: one clear ownership boundary, one canonical path, and explicit compatibility only when it has a current user and a removal plan.
```

Design / implementation planning rule:

```text
When a design choice is non-trivial and/or has architectural or runtime importance, prepare a concise plan for approval before implementation.
If the design choice is already agreed upon or trivial from architecture, complexity, and runtime perspectives, proceed directly with implementation.
```

Refactor expectations:

```text
Prefer deleting stale call paths over adding compatibility shims.
Prefer typed adapters at domain boundaries over leaking generic names everywhere.
Prefer evidence/debug artifacts as outputs, not as runtime IPC.
Prefer small reusable services when the same mechanism will be used by WorldSnapshot, mission events, telemetry, detections, or health streams.
Prefer synchronous in-process subscribers for control-critical handoff unless a stage explicitly requires async transport.
For async/external subscribers, expose sequence numbers and dropped-client/frame stats.
```

Recent cleanup examples that should guide future work:

```text
WorldSnapshotPublisher was generalized into EventPublisher<T> / EventSubscriber<T>.
WorldSnapshotSubscriber preserves domain-specific on_snapshot(...) while adapting to generic on_event(...).
GhostDetectionsSubscriber preserves domain-specific on_ghost_detections(...) while adapting to generic on_event(...).
MissionEventSubscriber preserves domain-specific on_mission_event(...) while adapting to generic on_event(...).
Inline snapshot file writing was removed from dedalus_mission_loop and moved to ArtifactSnapshotWriter.
LatestWorldSnapshot is now fed through LatestWorldSnapshotSubscriber, not a special CoreStackRunner constructor path.
The obsolete world_snapshot_publisher.cpp implementation was deleted after moving to header-only typed pub/sub.
AirSim overlay no longer evaluates GhostScenario or polls artifacts in normal mode; it subscribes to runtime events.
Tests were updated to use the actual WorldSnapshot AgentState type instead of stale AgentTrack assumptions.
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

Focused 2.26D pub/sub and runtime stream validation:

```bash
ctest --test-dir build-staging --output-on-failure -R 'pubsub|world_snapshot_publisher|world_snapshot_stream_server|mission_runtime'
```

Focused object behavior validation:

```bash
ctest --test-dir build-staging --output-on-failure -R 'object_behavior_mission_controller|object_behavior_mission_smoke|core_stack_config_loader|behavior_spec|target_selector'
```

Focused ghost / reprojection / world evidence validation:

```bash
ctest --test-dir build-staging --output-on-failure -R 'ghost_scenario|ghost_scenario_eval_cli|perception_world_model_flow|world_to_image_projector|world_reprojection_artifacts'
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

Live runtime event stream smoke:

```bash
./build-staging/apps/dedalus_mission_loop \
  --config config/core_stack_object_behavior_airsim_ghost.yaml \
  --output-dir out/object_behavior_airsim_ghost \
  --max-frames 900 \
  --shutdown-max-frames 400 \
  --world-snapshot-stream-port 47770 \
  --progress

nc 127.0.0.1 47770 | head -5
```

Live AirSim overlay smoke:

```bash
python3 simulation/airsim-world-overlay.py \
  --stream-port 47770 \
  --follow \
  --rate-hz 5 \
  --duration-s 180 \
  --clear \
  --label \
  --debug
```

---

## 1. Current Architecture

Current live mission pipeline:

```text
AirSim live frame + ego sidecar
  -> AirSimFrameSource
  -> FrameHintEgoProvider
  -> CoreStackRunner
       -> optional GhostTargetProvider::frame_at(...)
            -> GhostDetectionsPublisher
            -> PerceptionPipelineOutput.observations
  -> InMemoryWorldModel
  -> WorldSnapshotPublisher
       -> LatestWorldSnapshotSubscriber
       -> ArtifactSnapshotWriter
       -> optional RuntimeEventStreamServer
  -> LatestWorldSnapshot
  -> MissionRuntime async loop
       -> MissionEventPublisher
  -> TrajectoryMissionController today / ObjectBehaviorMissionController for object behavior
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
  -> WorldSnapshotPublisher
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

## 3. Runtime Event Publish / Subscribe Boundary

WorldSnapshot, ghost detections, and mission events are now published as typed runtime events.

```text
CoreStackRunner::run_once()
  -> GhostTargetProvider::frame_at(...), when enabled
       -> GhostDetectionsPublisher.publish(frame)
       -> append frame.observations to PerceptionPipelineOutput
  -> world_model.snapshot()
       -> WorldSnapshotPublisher.publish(snapshot)

MissionRuntime::write_event(...)
  -> MissionEventPublisher.publish(event)
  -> mission_events.jsonl
```

Current subscribers:

```text
LatestWorldSnapshotSubscriber:
  in-process behavior handoff.

ArtifactSnapshotWriter:
  writes snapshot_XXXX.json and snapshot_manifest.txt.
  artifacts are durable evidence/debug outputs, not runtime IPC.

RuntimeEventStreamServer:
  optional TCP JSONL live stream for external tools/customers.
  subscribes to GhostDetectionsPublisher, WorldSnapshotPublisher, and MissionEventPublisher.
  enabled by --world-snapshot-stream-port.
```

Design rule:

```text
The world model, ghost provider, and mission runtime should not know who consumes their events.
Consumers should subscribe to typed state/events and own their side effects.
```

Do not add direct file-writing, AirSim overlay behavior, or stream-specific behavior back into `CoreStackRunner` or `InMemoryWorldModel`.

---

## 4. Object Identity and Evidence Model

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
GhostDetectionsFrame is a simulation/debug runtime event emitted from the same evaluation that injects Observation3D into perception.
MissionEvent is the live event form of the same event JSON also written to mission_events.jsonl.
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

## 5. Behavior Config Location

Behavior specs are global autonomy/runtime configuration, not simulation assets.

Canonical behavior spec location:

```text
config/behaviors/follow_person.yaml
config/behaviors/follow_specific_track.yaml
config/behaviors/circle_car.yaml
config/behaviors/approach_target.yaml
config/behaviors/sequence_approach_circle.yaml
```

Simulation-only target scenarios live under simulation, currently:

```text
simulation/ghost_detections/person_pair_crossing.json
simulation/trajectories/ghost_person_001_crossing.json
simulation/trajectories/ghost_person_002_crossing.json
```

Rule:

```text
Behavior spec:
  config/behaviors/*.yaml

Ghost/scripted target scenario:
  simulation/ghost_detections/*.json

Trajectory referenced by ghost scenario:
  simulation/trajectories/*.json

Core-stack config that references behavior:
  config/core_stack_object_behavior_airsim_ghost.yaml
```

---

## 6. Ghost / Scripted Target Validation

Before real camera detections are reliable, use ghost targets that enter at the same semantic boundary as real detections.

Good validation path:

```text
Synthetic / AirSim frame
  -> GhostTargetProvider::frame_at(...)
      -> GhostDetectionsFrame live event for visualization/debug
      -> Observation3D objects appended to PerceptionPipelineOutput.observations
  -> InMemoryWorldModel
  -> WorldSnapshot.agents with stable source_track_id
  -> WorldSnapshotPublisher
  -> TargetSelector
  -> ObjectBehaviorMissionController
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

## 7. Behavior Language v1

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

Parser files:

```text
include/dedalus/behavior/behavior_spec.hpp
src/behavior/behavior_spec.cpp
tests/unit/test_behavior_spec.cpp
```

---

## 8. Mission Events and Validation Expectations

Expected object behavior events include both world-model and tracker handles when available:

```json
{"event":"target_selected","class":"person","agent_id":"agent_ghost_person_001","source_track_id":"ghost_person_001","identity_id":"identity_ghost_person_001","confidence":0.82}
{"event":"behavior_start","behavior":"follow","agent_id":"agent_ghost_person_001","source_track_id":"ghost_person_001"}
{"event":"behavior_tick_sample","behavior":"follow","source_track_id":"ghost_person_001"}
{"event":"behavior_complete","behavior":"follow","reason":"duration_elapsed"}
```

Current validation proves:

```text
final_state == Complete
terminal_settled == true
Arm / Takeoff / Land / Disarm commands dispatched and succeed
safe height reached before behavior execution
target_selected selects ghost_person_001
behavior_start / behavior_tick_sample / behavior_complete are emitted
follow emits bounded non-zero velocity during ExecuteMission
GoHome / Land / Complete reached
WorldSnapshot publisher/subscriber and runtime-event stream unit tests pass
```

Future validation should later prove:

```text
selected target identity remains stable across crossing/waiting/crossing-back motion
PLAN/AG/EGO/SEL markers update from live runtime event stream in AirSim overlay
circle / approach / sequence behavior math emits correct bounded velocity
obstacle avoidance remains outside M3 unless explicitly started post-M3
```

---

## 9. Scenario / Campaign Harness

Key files:

```text
simulation/run-mission-scenario.py
simulation/run-mission-campaign.py
simulation/validate-mission-artifacts.py
simulation/validate-object-behavior-airsim-ghost.py
config/core_stack_synthetic_mission_ci.yaml
config/core_stack_synthetic_mission_abort_ci.yaml
config/core_stack_object_behavior_airsim_ghost.yaml
config/mission_campaigns/synthetic_ci.json
config/mission_campaigns/airsim_live_smoke.json
docs/mission_scenario_runner.md
docs/object_behavior_airsim_ghost_runbook.md
docs/runtime_dataflow.md
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

## 10. Known Traps

```text
- Do not use dedalus_replay_mission. Use dedalus_mission_loop.
- Do not treat command helper OK as vehicle-state truth.
- Do not hide arming inside velocity commands.
- Do not collapse flight_control.arm_state and ego.armed.
- Do not move to ExecuteMission until Takeoff is confirmed by ego height.
- Do not stop at raw Complete state; wait for terminal_settled / Complete status=complete.
- Do not treat Abort as stop immediately when recovery is possible.
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
- Do not use artifact files as runtime IPC when a live stream or in-process subscriber is the right boundary.
- Do not make simulation/airsim-world-overlay.py evaluate GhostScenario or poll snapshot artifacts in normal mode; it should subscribe and render.
- Do not add shims to preserve stale pre-refactor APIs unless there is a current user and an explicit removal plan.
- Do not create branches or PRs unless explicitly requested.
```

---

## 11. Pointers

Read next when relevant:

```text
docs/runtime_dataflow.md                     canonical source->publisher->server->subscriber->sink diagrams
docs/object_behavior_airsim_ghost_runbook.md current AirSim ghost behavior + live stream runbook
docs/object_conditioned_behavior_plan.md     detailed M3 behavior + identity plan
docs/world_model_reprojection_validation_plan.md reprojection and world-model evidence plan
docs/mission_scenario_runner.md             scenario/campaign harness
docs/mission_pipeline_current_state.md       mission loop architecture
docs/core_stack_current_state.md             broader core-stack status
WHITEPAPER.md                                architectural rationale
HANDOFF.md                                   handoff prompt template
LLM.back.md                                  historical context only
```