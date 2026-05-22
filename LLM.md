# Dedalus LLM Operating Brief

This file is the active orientation document for a new LLM session. Keep it short, current, and action-oriented. Historical notes and superseded debugging context live in `LLM.back.md`.

Repository:

```text
guybarnahum/dedalus
```

Current validated reference point for this handoff:

```text
Commit inspected: c1ba05ea554daf759e8f2a328bcd836e5449c119
Commit title: chore(mission): richer display_state/detail for execute-phase events
GitHub status checks: none attached; local build/test validation is still required.
```

Active milestone state:

```text
Milestones 2.20, 2.21, 2.22, and 2.23 are implemented / validated.

Milestone 2.24 — TargetSelector, ghost validation, reprojection validation, and world-model evidence plumbing.
Status: implemented through 2.24G.9 baseline.

Milestone 2.25 — ObjectBehaviorMissionController skeleton and first object-conditioned behavior runtime.
Status: implemented through object behavior lifecycle and bounded follow behavior baseline.

Milestone 2.26 — AirSim ghost behavior validation, live runtime-event plumbing, AirSim existing-object binding, follow arrival control, and live overlay/OSD.
Status: implemented through 2.26E.13 design/code shape at commit c1ba05e. Needs local build/test and AirSim visual validation.
```

## Current 2.26D/E State

```text
Runtime-event pipeline is the canonical IPC boundary.
Artifact files remain durable evidence/debug outputs, not runtime IPC.

Core publish path:
  CoreStackRunner
    -> GhostTargetProvider::frame_at(...), when enabled
       -> GhostDetectionsPublisher
       -> PerceptionPipelineOutput.observations
    -> InMemoryWorldModel
    -> WorldSnapshotPublisher
       -> LatestWorldSnapshotSubscriber
       -> ArtifactSnapshotWriter
       -> RuntimeEventStreamServer, optional TCP JSONL

  MissionRuntime
    -> MissionEventPublisher
       -> mission_events.jsonl
       -> RuntimeEventStreamServer, optional TCP JSONL

Live runtime event stream:
  dedalus_mission_loop --world-snapshot-stream-port 47770
  emits JSONL records on one TCP stream:
    {"type":"ghost_detections","seq":N,...}
    {"type":"world_snapshot","seq":N,...}
    {"type":"mission_event","seq":N,...}

AirSim overlay:
  simulation/airsim-world-overlay.py is a stream-only subscriber/renderer.
  It renders PLAN / PLAN* from ghost_detections.
  It renders AG / EGO from world_snapshot.
  It renders SEL from mission_event target_selected matched against the latest world_snapshot agent.
  It does not evaluate GhostScenario, discover AirSim objects, or poll snapshot artifacts in normal mode.
```

AirSim existing-object ghost binding:

```text
Canonical live config:
  config/core_stack_object_behavior_airsim_existing_object.yaml
    -> binds ghost_person_001 to BRPlayer_01_96 by default

Example/template config:
  config/core_stack_object_behavior_airsim_existing_object_example.yaml

Focused runbook:
  docs/airsim_existing_object_ghost_runbook.md

Discovery / pose tools:
  python3 simulation/airsim-list-objects.py --match-class person --sort distance --format table
  python3 simulation/airsim-object-poses.py --object BRPlayer_01_96

Runtime source path:
  GhostTargetProvider(AirSimGhostObjectSourceConfig)
    -> calls simulation/airsim-object-poses.py through BridgeTransport
    -> converts selected AirSim object poses to GhostDetectionState
    -> emits GhostDetectionsFrame + Observation3D list
```

Object behavior / follow control:

```text
ObjectBehaviorMissionController consumes WorldSnapshot agents through TargetSelector.
Follow behavior now uses stable observation geometry, not ego-relative bearing drift.
Follow arrival velocity is target-relative:
  command_velocity = target_velocity + closing_velocity
Static targets naturally converge to zero relative velocity.
Moving targets converge toward matched target velocity.

Debug events include:
  arrival_mode
  desired_error_xy_m
  closing_speed_mps
  target_speed_xy_mps
  relative_speed_xy_mps
  follow geometry fields
```

Live overlay / OSD:

```text
The overlay provides:
  DEDALUS        fixed-width numeric flight line: height, vz, vxy, heading
  DEDALUS-STATE mission/operator state line
  EGO marker
  optional EGO XY velocity arrow

Dynamic AirSim markers/arrows should use short durations because AirSim Python plot APIs do not provide per-marker update/delete handles. Some blinking is acceptable to avoid marker accumulation.
```

Display-state ownership boundary at c1ba05e:

```text
MissionRuntime owns lifecycle / command display:
  Arm, Takeoff, Mission, GoHome, Land, Disarm, Settled, Failed

ObjectBehaviorMissionController owns behavior detail:
  arriving, following, positioned, circling, done

Overlay should render display_state + display_detail and should not infer behavior semantics.
Fallback tables in overlay are compatibility only for old event streams.
```

Expected OSD examples:

```text
DEDALUS-STATE  Arm      arming
DEDALUS-STATE  Takeoff  climbing
DEDALUS-STATE  Mission  arriving
DEDALUS-STATE  Mission  following
DEDALUS-STATE  Mission  positioned
DEDALUS-STATE  GoHome   returning
DEDALUS-STATE  Land     landing
DEDALUS-STATE  Disarm   ok
DEDALUS-STATE  Settled  done
DEDALUS-STATE  Failed   Velocity
```

## Current Architecture

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
  -> ObjectBehaviorMissionController for object behavior
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

## Engineering Hygiene Policy

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

Patch policy:

```text
Default: apply changes directly to main.
Do not create branches or PRs unless explicitly requested.
Do not leave completed work sitting on a feature branch.
Prefer GitHub connector file updates directly on main when available.
If connector patching fails, is ambiguous, is blocked, or would require a risky broad rewrite, stop using the connector for that code change.
Generate an exact manual patch and ask the user to apply it locally.
Do not keep retrying increasingly complex connector paths after a connector failure.
```

## Validation Commands

General validation after code patches:

```bash
cmake --build build-staging -j$(nproc)
ctest --test-dir build-staging --output-on-failure
```

Focused 2.26D/E pub/sub and runtime stream validation:

```bash
ctest --test-dir build-staging --output-on-failure -R 'pubsub|world_snapshot_publisher|world_snapshot_stream_server|mission_runtime'
```

Focused object behavior validation:

```bash
ctest --test-dir build-staging --output-on-failure -R 'object_behavior_mission_controller|object_behavior_mission_smoke|core_stack_config_loader|behavior_spec|target_selector'
```

Focused overlay syntax validation:

```bash
python3 -m py_compile simulation/airsim-world-overlay.py
```

Existing AirSim object pose bridge smoke:

```bash
python3 simulation/airsim-object-poses.py \
  --object BRPlayer_01_96
```

Live existing-object validation:

```bash
rm -rf out/object_behavior_airsim_existing_object out/object_behavior_airsim_existing_object_annotation

./build-staging/apps/dedalus_mission_loop \
  --config config/core_stack_object_behavior_airsim_existing_object.yaml \
  --output-dir out/object_behavior_airsim_existing_object \
  --max-frames 2400 \
  --shutdown-max-frames 1800 \
  --world-snapshot-stream-port 47770 \
  --safe-height 40 \
  --behavior-duration-s 90 \
  --progress
```

Overlay validation:

```bash
python3 simulation/airsim-world-overlay.py \
  --stream-port 47770 \
  --follow \
  --rate-hz 5 \
  --duration-s 180 \
  --clear \
  --label \
  --osd \
  --debug \
  --debug-json out/object_behavior_airsim_existing_object/overlay_debug_latest.json
```

Inspect behavior display events:

```bash
grep '"display_state"' out/object_behavior_airsim_existing_object/mission_events.jsonl | tail -40
```

Expected display-state event flow:

```text
Arm / arming or ok
Takeoff / climbing
Mission / arriving
Mission / following or positioned
GoHome / returning
Land / landing
Disarm / ok
Settled / done
```

## Immediate Next Tasks

```text
1. On AirSim machine, build/test c1ba05e or later:
     python3 -m py_compile simulation/airsim-world-overlay.py
     cmake --build build-staging -j$(nproc)
     ctest --test-dir build-staging --output-on-failure -R 'mission_runtime|object_behavior_mission_controller|object_behavior_mission_smoke|core_stack_config_loader|behavior_spec|target_selector'

2. Run live existing-object validation with overlay OSD:
     - confirm PLAN / AG / SEL align over BRPlayer_01_96
     - confirm no disruptive marker/arrow accumulation
     - confirm DEDALUS numeric line remains stable
     - confirm DEDALUS-STATE uses display_state/display_detail from mission events

3. Confirm follow behavior:
     - no spin when near/above the selected object
     - arrival_mode transitions cruise/slow/hold as expected
     - relative_speed_xy_mps trends toward zero near arrival

4. If validation passes, record 2.26E as complete and start 2.27A.
```

## 2.27A Proposed Next Slice

```text
Implement circle behavior with velocity-matched orbit insertion:
  - approach a 3 o'clock orbit entry point
  - blend into tangential velocity before capture
  - maintain orbit using target_velocity + tangent_velocity + radial correction
  - emit behavior display details: arriving -> circling
  - keep overlay renderer-only
```

## Known Traps

```text
- Do not use dedalus_replay_mission. Use dedalus_mission_loop.
- Do not treat command helper OK as vehicle-state truth.
- Do not hide arming inside velocity commands.
- Do not collapse flight_control.arm_state and ego.armed.
- Do not move to ExecuteMission until Takeoff is confirmed by ego height.
- Do not stop at raw Complete state; wait for terminal_settled / Complete status=complete.
- Do not make the native C++ MAVLink sink the default live path; use px4_bridge.
- Do not rewrite the working pymavlink control path in C++ while stabilizing behavior.
- Do not let telemetry sidecar and command bridge bind the same MAVLink endpoint.
- Do not let human diagnostics contaminate binary bridge stdout; binary frame bridge stdout is protocol bytes only.
- Do not put obstacle avoidance inside the flight sink.
- Do not let Milestone 3 balloon into full obstacle avoidance; M3 is object-conditioned behavior. Avoidance starts post-M3.
- Do not collapse track_id/source_track_id/agent_id/identity_id into one field.
- Do not select targets only by confidence when a stable track/agent target is specified.
- Do not bypass WorldSnapshot/TargetSelector by hardcoding selected_target in config for main validation.
- Do not keep global behavior specs under simulation/behaviors; use config/behaviors.
- Do not use artifact files as runtime IPC when a live stream or in-process subscriber is the right boundary.
- Do not make simulation/airsim-world-overlay.py evaluate GhostScenario, discover AirSim objects, or poll snapshot artifacts in normal mode; it should subscribe and render.
- Do not put behavior semantics in overlay logic. Mission/behavior events should publish display_state/display_detail; overlay should render them.
- Do not add shims to preserve stale pre-refactor APIs unless there is a current user and an explicit removal plan.
- Do not create branches or PRs unless explicitly requested.
```

## Pointers

Read next when relevant:

```text
docs/airsim_existing_object_ghost_runbook.md focused existing-object validation
docs/runtime_dataflow.md                     source->publisher->server->subscriber->sink diagrams
docs/object_behavior_airsim_ghost_runbook.md AirSim ghost behavior + live stream runbook
docs/object_conditioned_behavior_plan.md     detailed M3 behavior + identity plan
docs/world_model_reprojection_validation_plan.md reprojection and world-model evidence plan
docs/mission_scenario_runner.md             scenario/campaign harness
docs/mission_pipeline_current_state.md       mission loop architecture
docs/core_stack_current_state.md             broader core-stack status
docs/llm_connector_patch_policy.md          connector/manual patch safety policy
WHITEPAPER.md                                architectural rationale
HANDOFF.md                                   handoff prompt template
LLM.back.md                                  historical context only
```
