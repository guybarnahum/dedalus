# Dedalus LLM Operating Brief

This file is the active orientation document for a new LLM session. Keep it short, current, and action-oriented. Historical notes and superseded debugging context live in `LLM.back.md`.

Repository:

```text
guybarnahum/dedalus
```

Current handoff state:

```text
2.26E is complete and live-validated by the operator.
2.27A is complete and live-validated by the operator.
2.27B is complete and locally validated by build/tests plus the circle trajectory validator.

Active next work: 2.28 — approach behavior and optional approach -> circle sequence validation.

GitHub status checks may be absent; continue to run local build/tests after code changes.
```

---

## 1. Active Milestones

```text
Milestones 2.20-2.23: implemented / validated.
Milestone 2.24: TargetSelector, ghost validation, reprojection validation, world-model evidence plumbing. Implemented through 2.24G.9 baseline.
Milestone 2.25: ObjectBehaviorMissionController skeleton and bounded follow baseline. Implemented.
Milestone 2.26: AirSim ghost behavior validation, live runtime-event stream, AirSim existing-object binding, follow arrival control, and live overlay / OSD. Complete after 2.26E validation.
Milestone 2.27A: robust circle behavior with continuous orbit-capture control law and orbit-count completion. Complete after 3-orbit AirSim validation.
Milestone 2.27B: validation hardening for circle/orbit behavior. Complete after validator CLI, validator test, runbook update, 34/34 CTest, and live trajectory validator pass.
Milestone 2.28: approach behavior and approach -> circle sequence validation. Active next slice.
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
  -> persistent tools/px4/px4-command-bridge.py
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

## 3. Completed Capabilities Through 2.27B

AirSim existing-object binding:

```text
config/core_stack_object_behavior_airsim_existing_object.yaml
  -> binds ghost_person_001 to BRPlayer_01_96 by default

config/core_stack_object_behavior_airsim_existing_object_circle.yml
  -> circle validation config using explicit existing-object binding

config/behaviors/circle_existing_object_person.yaml
  -> circle behavior spec for the validated existing-object path

simulation/airsim/scripts/airsim-object-poses.py
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

Circle behavior:

```text
Circle is implemented as a continuous orbit-capture control law, not follow-with-sideways-offset and not a brittle waypoint script.

Command law:
  desired_velocity =
      target_velocity
    + tangent_velocity_at_current_radial_angle
    + radial_correction_velocity
    + altitude_correction_velocity

Behavior is robust to imperfect insertion geometry:
  - starts outside radius -> radial correction inward while tangential motion is active
  - starts inside radius -> radial correction outward while tangential motion is active
  - starts near radius but at arbitrary bearing -> enters orbit from current radial angle
  - overshoots -> radial correction recovers on subsequent ticks
  - once orbit mode is reached -> orbit_mode_latched remains true until completion/reset

For known static AirSim existing-object bindings, object_behavior_zero_target_velocity may be enabled so the controller does not velocity-match synthetic/static-object velocity noise. This zeroes target_velocity only; tangent velocity remains active.
```

2.27A live validation result:

```text
3-orbit AirSim existing-object circle run succeeded.
behavior_complete reason=orbit_count_elapsed.
Mission transitioned to GoHome after orbit-count completion.
```

2.27B validation result:

```text
Focused tests passed:
  behavior_spec
  core_stack_config_loader
  object_behavior_mission_controller
  circle_trajectory_validator

Full CTest passed:
  34/34 tests passed

Circle trajectory validator passed on the 3-orbit AirSim run:
  events: 3636
  circle samples: 144
  circling samples: 131
  post-latch samples: 131
  orbit radius target: 10.0
  orbit mode latched observed: true
  actual radius min/avg/max: 9.703 / 10.246 / 12.434
  radius error min/avg/max: -0.297 / 0.246 / 2.434
  avg abs radius error: 0.256
  completed orbits first/last: 0.000 -> 2.982
  behavior_complete reason: orbit_count_elapsed
  runtime_stop terminal_settled: true
  state path: Idle -> Prepare -> Takeoff -> ExecuteMission -> GoHome -> Land -> Complete
```

Circle validator:

```text
tools/validation/validate-circle-trajectory.py
  -> parses mission_events.jsonl
  -> checks target_selected, circling samples, orbit_mode_latched, orbit count, radius stats,
     behavior_complete reason, terminal_settled, and GoHome -> Land -> Complete lifecycle evidence

tests/integration/test_circle_trajectory_validator.py
  -> deterministic synthetic fixture test for pass/fail validator behavior
```

Overlay / OSD:

```text
simulation/airsim/scripts/airsim-world-overlay.py is stream-only.
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
Mission / following, positioned, or circling
GoHome / returning
Land / landing
Disarm / ok
Settled / done
```

---

## 4. Active Next Work: 2.28 Approach Behavior / Approach -> Circle Sequence

Goal:

```text
Add approach behavior as the next object-conditioned control law and validate a simple approach -> circle -> go_home_land sequence.
```

Motivation:

```text
Follow maintains a relative offset.
Circle maintains a target-relative orbit.
Approach should move toward a selected target until a standoff condition is satisfied, then hold/complete without colliding with the target.

Approach is the natural bridge to sequence behavior because useful object-conditioned missions often look like:
  select target -> approach to standoff -> circle/inspect -> go home and land
```

Proposed approach semantics:

```text
Approach consumes the same TargetSelector output as follow/circle.
It should produce bounded kinematic intent only.
It must not implement obstacle avoidance.

Command law, first implementation:
  radial vector = ego position relative to selected target
  actual_distance = norm_xy(target_to_ego)
  desired_standoff = stop_distance_m or behavior.radius_m-style configured standoff
  distance_error = actual_distance - desired_standoff

If outside standoff:
  move inward toward the target along the radial direction, with target_velocity added when appropriate.

If at/inside standoff:
  stop closing and hold/track target velocity tangentially/locally as needed.

Vertical behavior:
  use altitude_offset_m and existing safe-height policy.
  keep horizontal approach law separate from altitude correction.
```

Required robust-control principle:

```text
Like circle, approach should tolerate imperfect initial position, velocity, attitude, estimator noise, and overshoot.
It should not require hitting a single waypoint.
It should continuously recompute distance/standoff error and recover if the drone overshoots too close or drifts too far.
```

Suggested 2.28 implementation phases:

```text
1. Inspect current BehaviorSpec approach fields and config/behaviors/approach_target.yaml.
2. Define exact approach fields to support now:
   - stop_distance_m or equivalent standoff distance
   - altitude_offset_m
   - max_speed_mps
   - max_vertical_speed_mps
   - position_tolerance_m
   - lost_target_timeout_s if already parsed
3. Add ApproachGeometry / approach_velocity helpers near the follow/circle helpers.
4. Add behavior_tick_sample / behavior_debug fields for approach:
   - approach_phase
   - standoff_distance_m
   - actual_distance_m
   - distance_error_m
   - closing_velocity_mps
   - target_velocity_mps
   - desired_velocity_mps
5. Add unit tests:
   - far target command points toward target
   - moving target command includes target_velocity when appropriate
   - inside standoff does not continue closing into target
   - overshoot recovers outward or holds safely
   - speed is clamped
   - display_detail transitions arriving -> positioned / approached
6. Validate synthetic/unit tests first.
7. Add an approach existing-object config if needed.
8. Validate live AirSim existing-object approach path.
9. Add or extend a sequence config:
   approach -> circle -> go_home_land
   Keep sequence minimal; do not balloon into a general mission planner.
```

Expected display details for 2.28:

```text
Mission / arriving       while closing to standoff
Mission / positioned     when standoff is captured / held
Mission / circling       if sequence advances into circle
```

Suggested 2.28 success criteria:

```text
Build passes.
Full CTest passes.
Approach unit tests pass.
Approach debug events expose standoff geometry.
Live existing-object approach reaches standoff without collision-like closing behavior.
Optional approach -> circle sequence reaches circle, completes orbit_count_elapsed, then GoHome -> Land -> Disarm -> Settled.
```

Non-goals for 2.28:

```text
No obstacle avoidance.
No moving-actor full validation unless trivial from current ghost path.
No native C++ MAVLink rewrite.
No overlay-side behavior inference.
No general mission planner.
No multi-target assignment.
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
python3 -m py_compile simulation/airsim/scripts/airsim-world-overlay.py

ctest --test-dir build-staging --output-on-failure -R \
  'mission_runtime|object_behavior_mission_controller|object_behavior_mission_smoke|core_stack_config_loader|behavior_spec|target_selector|world_snapshot_stream_server|circle_trajectory_validator'
```

AirSim existing-object follow validation:

```bash
python3 simulation/airsim/scripts/airsim-object-poses.py --object BRPlayer_01_96

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

AirSim existing-object circle validation:

```bash
./build-staging/apps/dedalus_mission_loop \
  --config config/core_stack_object_behavior_airsim_existing_object_circle.yml \
  --output-dir out/object_behavior_airsim_existing_object_circle \
  --max-frames 5400 \
  --shutdown-max-frames 1800 \
  --world-snapshot-stream-port 47770 \
  --safe-height 40 \
  --behavior-duration-s 360 \
  --progress
```

Circle trajectory validator:

```bash
python3 tools/validation/validate-circle-trajectory.py \
  --events out/object_behavior_airsim_existing_object_circle/mission_events.jsonl \
  --min-orbits 3.0 \
  --radius 10.0 \
  --avg-radius-error-max 1.0 \
  --max-radius-error-after-latch 3.0 \
  --expect-complete-reason orbit_count_elapsed \
  --require-terminal-settled \
  --require-lifecycle
```

Overlay subscriber for live operator review:

```bash
python3 simulation/airsim/scripts/airsim-world-overlay.py \
  --stream-port 47770 \
  --follow \
  --rate-hz 5 \
  --duration-s 240 \
  --clear \
  --label \
  --osd \
  --debug \
  --debug-json out/object_behavior_airsim_existing_object_circle/overlay_debug_latest.json
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
- Do not make simulation/airsim/scripts/airsim-world-overlay.py evaluate GhostScenario, discover AirSim objects, or poll snapshot artifacts in normal mode; it should subscribe and render.
- Do not put behavior semantics in overlay logic. Mission/behavior events should publish display_state/display_detail; overlay should render them.
- Do not implement circle as a yaw-only, latch-only, or fixed-waypoint visual hack.
- Do not require exact 3 o'clock orbit insertion; circle must tolerate imperfect initial position, velocity, attitude, and overshoot.
- Do not let approach fly directly into the target; standoff capture and overshoot recovery are required.
- Do not add shims to preserve stale pre-refactor APIs unless there is a current user and an explicit removal plan.
- Do not create branches or PRs unless explicitly requested.
```

---

## 8. Pointers

```text
docs/runtime_dataflow.md                     source->publisher->server->subscriber->sink diagrams
docs/flight_behavior_control_laws.md        robust behavior control-law principle, especially circle/orbit
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
