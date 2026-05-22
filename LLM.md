# Dedalus LLM Operating Brief

This file is the active orientation document for a new LLM session. Keep it short, current, and action-oriented. Historical notes and superseded debugging context live in `LLM.back.md`.

Repository:

```text
guybarnahum/dedalus
```

Current handoff state:

```text
2.26E is complete and live-validated by the operator.
The next active work is 2.27A: circle behavior with velocity-matched orbit insertion.

GitHub status checks may be absent; continue to run local build/tests after code changes.
```

---

## 1. Active Milestones

```text
Milestones 2.20-2.23: implemented / validated.
Milestone 2.24: TargetSelector, ghost validation, reprojection validation, world-model evidence plumbing. Implemented through 2.24G.9 baseline.
Milestone 2.25: ObjectBehaviorMissionController skeleton and bounded follow baseline. Implemented.
Milestone 2.26: AirSim ghost behavior validation, live runtime-event stream, AirSim existing-object binding, follow arrival control, and live overlay / OSD. Complete after 2.26E validation.
Milestone 2.27: Expand object-conditioned behaviors beyond follow. Active next slice: 2.27A circle behavior.
```

---

## 2. Current Runtime Architecture

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
  -> ObjectBehaviorMissionController
  -> Px4BridgeCommandSink
  -> persistent simulation/px4-command-bridge.py
       - PX4 shell: arm, takeoff, land, disarm
       - pymavlink: OFFBOARD velocity setpoints
       - LOCAL_POSITION_NED feedback climb to safe height
  -> PX4 / AirSim
```

Runtime-event stream:

```text
dedalus_mission_loop --world-snapshot-stream-port 47770
  -> one TCP JSONL stream with:
       ghost_detections
       world_snapshot
       mission_event
```

Core boundary:

```text
WorldSnapshot is autonomy state.
PerceptionPipelineOutput is evidence.
Ghost detections enter through the same Observation3D path as real detections.
Artifacts are evidence/debug outputs, not IPC.
Overlay is a subscriber/renderer only.
```

---

## 3. 2.26E Completed Capabilities

AirSim existing-object binding:

```text
config/core_stack_object_behavior_airsim_existing_object.yaml
  -> binds ghost_person_001 to BRPlayer_01_96 by default

simulation/airsim-object-poses.py
  -> calls AirSim simGetObjectPose(object_name)
  -> returns compact JSON object poses

GhostTargetProvider(AirSimGhostObjectSourceConfig)
  -> converts selected AirSim object poses to GhostDetectionState
  -> emits GhostDetectionsFrame + Observation3D list
```

Follow behavior:

```text
ObjectBehaviorMissionController consumes WorldSnapshot agents through TargetSelector.
Follow uses target-relative observation geometry.
Follow arrival command is:
  command_velocity = target_velocity + closing_velocity
Static targets converge to zero relative velocity.
Moving targets converge toward matched target velocity.
Follow no longer relies on latching yaw to hide target-location drift.
```

Overlay / OSD:

```text
simulation/airsim-world-overlay.py is stream-only.
It renders PLAN / PLAN* from ghost_detections.
It renders AG / EGO from world_snapshot.
It renders SEL from mission_event target_selected.
It renders DEDALUS numeric OSD from ego motion.
It renders DEDALUS-STATE from mission_event display_state/display_detail.
It should not infer behavior semantics.
```

Display-state ownership:

```text
MissionRuntime owns lifecycle / command display:
  Arm, Takeoff, Mission, GoHome, Land, Disarm, Settled, Failed

ObjectBehaviorMissionController owns behavior detail:
  arriving, following, positioned, circling, done

Overlay renders display_state + display_detail.
Fallback tables in overlay are compatibility only for old event streams.
```

Expected validated OSD flow:

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

---

## 4. Active Next Work: 2.27A Circle Behavior

Goal:

```text
Add a real circle behavior that approaches a stable orbit entry point, velocity-matches into orbit, then maintains a smooth target-relative circular path.
```

Design intent:

```text
Circle is not follow-with-sideways-offset.
Circle should have an explicit orbit geometry and entry strategy.
The drone should approach the 3 o'clock point of the orbit, match the desired tangential velocity there, and then transition smoothly into circling.
The controller should be target-relative and work for static and moving targets.
```

Proposed circle controller phases:

```text
arriving:
  compute orbit entry point, initially the 3 o'clock point in target-relative XY space
  desired position = target_position + orbit_radius * entry_axis
  desired velocity = target_velocity + desired_tangent_velocity_at_entry
  use arrival controller to reduce position error while matching desired velocity

circling:
  desired radial distance = configured orbit radius
  desired velocity = target_velocity + tangent_velocity + radial_correction
  tangent_velocity magnitude = configured orbit speed
  radial_correction pulls inward/outward to maintain radius
  yaw/camera heading uses trajectory camera offset semantics, not ad hoc yaw hacks

complete:
  duration / mission finish condition moves to GoHome -> Land -> Disarm -> Settled
```

Suggested implementation order:

```text
1. Inspect existing BehaviorSpec circle fields and config/behaviors/circle_car.yaml.
2. Review current FollowGeometry and arrival controller for reusable target-relative helpers.
3. Add CircleGeometry / CircleCommand computation with clear math helpers.
4. Add unit tests for static target orbit entry, moving target target_velocity addition, radial correction sign, speed clamping, and display_detail arriving -> circling.
5. Wire behavior_tick_sample debug fields for circle: phase, radius error, radial correction, tangent velocity, desired velocity, actual radius.
6. Validate in synthetic/unit tests first.
7. Validate in AirSim using the existing object binding and overlay OSD.
```

Expected display details for 2.27A:

```text
Mission / arriving
Mission / circling
```

Non-goals for 2.27A:

```text
No obstacle avoidance.
No mesh/person asset insertion work.
No overlay-side behavior inference.
No direct GhostDetectionsFrame consumption by behavior.
No file-IPC reintroduction.
```

---

## 5. Validation Commands

General validation after code patches:

```bash
cmake --build build-staging -j$(nproc)
ctest --test-dir build-staging --output-on-failure
```

Focused current behavior/runtime validation:

```bash
python3 -m py_compile simulation/airsim-world-overlay.py

ctest --test-dir build-staging --output-on-failure -R \
  'mission_runtime|object_behavior_mission_controller|object_behavior_mission_smoke|core_stack_config_loader|behavior_spec|target_selector|world_snapshot_stream_server'
```

AirSim existing-object validation:

```bash
python3 simulation/airsim-object-poses.py --object BRPlayer_01_96

./build-staging/apps/dedalus_mission_loop \
  --config config/core_stack_object_behavior_airsim_existing_object.yaml \
  --output-dir out/object_behavior_airsim_existing_object \
  --max-frames 2400 \
  --shutdown-max-frames 1800 \
  --world-snapshot-stream-port 47770 \
  --safe-height 40 \
  --behavior-duration-s 90 \
  --progress

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

Inspect display-state events:

```bash
grep '"display_state"' out/object_behavior_airsim_existing_object/mission_events.jsonl | tail -40
```

---

## 6. Engineering Hygiene Policy

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

---

## 7. Known Traps

```text
- Do not use dedalus_replay_mission. Use dedalus_mission_loop.
- Do not treat command helper OK as vehicle-state truth.
- Do not hide arming inside velocity commands.
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

---

## 8. Pointers

```text
docs/runtime_dataflow.md                     source->publisher->server->subscriber->sink diagrams
docs/airsim_existing_object_ghost_runbook.md AirSim existing-object validation
docs/object_behavior_airsim_ghost_runbook.md AirSim ghost behavior + live stream runbook
docs/object_conditioned_behavior_plan.md     detailed M3 behavior + identity plan
docs/mission_scenario_runner.md             scenario/campaign harness
docs/world_model_reprojection_validation_plan.md reprojection and world-model evidence plan
docs/mission_pipeline_current_state.md       mission loop architecture
docs/core_stack_current_state.md             broader core-stack status
docs/llm_connector_patch_policy.md          connector/manual patch safety policy
WHITEPAPER.md                                architectural rationale
HANDOFF.md                                   handoff prompt template
LLM.back.md                                  historical context only
```
