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
2.27B is complete and locally/live validated by build/tests plus the circle trajectory validator.
2.28B is complete: no-shim repo layout migration is validated.
2.28B.1 is complete: generated third-party dependencies are staged under third_party/ and validated from empty-state setup.

Active next work: 2.28C — CameraPointingIntent and AirSim/MAVLink-gimbal sink abstraction for target-stare pitch.

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
Milestone 2.28B: no-shim target/tool/dependency layout migration. Complete after empty-state setup and live AirSim validation.
Milestone 2.28B.1: generated dependency staging under third_party/. Complete after PX4 SITL, iceoryx, and Colosseum assets staged under third_party and no stale generated dirs under root/simulation.
Milestone 2.28C: CameraPointingIntent and camera/gimbal sink abstraction. Active next slice.
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

Repo layout boundary:

```text
simulation/airsim/
  Dedalus-owned AirSim target runtime scripts, settings, logs, and AirSim-specific validation.

simulation/airsim/run.sh
  Starts AirSim / PX4 SITL runtime.

simulation/airsim/stop.sh
  Normal way to stop AirSim / PX4 SITL runtime.

cleanup.sh
  Root cleanup/reset helper for rebuild/reset state. Do not present cleanup.sh as the normal simulation stop command.

third_party/
  Generated external dependency checkouts/downloads:
    PX4-Autopilot/
    iceoryx_build/
    colosseum_environments/

tools/px4/
  Dedalus PX4/MAVLink protocol tools.

tools/validation/
  Artifact validators.

config/behaviors/
  Behavior specs, trajectories, and ghost fixture assets.
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

## 3. Completed Capabilities Through 2.28B.1

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

2.28B / 2.28B.1 validation result:

```text
Empty-state setup passed on EC2 L4.
setup.sh completed end-to-end.
PX4 SITL built successfully from third_party/PX4-Autopilot.
iceoryx build state staged under third_party/iceoryx_build.
Colosseum/AirSim environments downloaded/extracted under third_party/colosseum_environments.
No stale generated dependency dirs were present under repo root, simulation/, simulation/airsim/, or infrastructure/.

Script validation passed:
  bash -n setup.sh
  bash -n cleanup.sh
  bash -n simulation/airsim/run.sh
  bash -n simulation/airsim/stop.sh

Python tool validation passed:
  py_compile for simulation/airsim/scripts, simulation/airsim/validation, tools/px4, tools/mission, tools/validation, tools/trajectory

Build/test validation passed:
  cmake configure passed
  cmake build passed
  full ctest passed: 34/34
  focused 2.28 tests passed: 7/7

Live/runtime validation passed:
  simulation/airsim/run.sh launches AirSim / PX4 SITL.
  simulation/airsim/stop.sh stops the runtime and reports no build artifacts removed.
  AirSim camera/gimbal probe passed with overall_ok=True.
  Existing-object circle mission reached Complete / terminal_settled.
  validate-circle-trajectory passed for configured orbit_count=1.0.
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

## 4. Active Next Work: 2.28C CameraPointingIntent / Target-Stare Pitch

Goal:

```text
Add a hardware-neutral camera/gimbal pointing intent for vertical target stare, with AirSim camera pitch as the first concrete sink and MAVLink gimbal support as the real-hardware path.
```

Motivation:

```text
Target stare is a 2-axis pointing policy:
  - vehicle yaw keeps the target centered horizontally in the camera image
  - camera/gimbal pitch keeps the target centered vertically in the camera image

PX4 velocity/yaw remains the vehicle control path.
Camera/gimbal pitch should not be hidden inside PX4 velocity commands.
It should be a separate CameraPointingIntent emitted by the behavior/runtime layer and executed by a camera/gimbal sink.
```

Proposed architecture:

```text
ObjectBehaviorMissionController / MissionRuntime
  -> VelocityCommand {vx, vy, vz, yaw}
       -> Px4BridgeCommandSink
       -> tools/px4/px4-command-bridge.py
       -> PX4 OFFBOARD / AirSim or real vehicle

  -> CameraPointingIntent {camera_name, pitch_rad, yaw_rad?, source_track_id, mode}
       -> AirSimCameraPointingSink for simulation
       -> future MavlinkGimbalPointingSink for real hardware
       -> NullCameraPointingSink with one-time warning when unsupported
```

Required 2.28C implementation phases:

```text
1. Define CameraPointingIntent and a small sink boundary.
2. Compute target pitch from selected target geometry:
     pitch_to_target = atan2(delta_z, hypot(delta_x, delta_y))
   Verify sign convention against AirSim probe images before declaring done.
3. Add config fields for camera pointing:
     object_behavior_camera_pointing_sink: none | airsim | mavlink_gimbal
     object_behavior_camera_pitch_mode: none | target | fixed | hold
     object_behavior_camera_name: front_center
     object_behavior_camera_pitch_min_deg
     object_behavior_camera_pitch_max_deg
4. Add AirSim camera pointing implementation using the proven simSetCameraPose path.
5. Add mission events / debug fields for camera_pointing:
     pitch_mode
     camera
     pitch_rad / pitch_deg
     target_elevation_rad
     source_track_id
6. Add Null sink with one-time warning when pitch targeting is requested but unavailable.
7. Add MAVLink gimbal design/stub path using MAVLink Gimbal Protocol v2 / gimbal manager, without requiring real gimbal hardware for CI.
8. Validate in AirSim with circle/follow target-stare yaw plus camera pitch enabled.
```

Expected 2.28C success criteria:

```text
Build passes.
Full CTest passes.
AirSim camera/gimbal probe still passes overall_ok=True.
CameraPointingIntent events are emitted when configured.
AirSim sink changes camera pitch at runtime.
No camera/gimbal behavior semantics are placed in overlay logic.
Vehicle yaw and camera pitch remain separate intent/sink paths.
```

Non-goals for 2.28C:

```text
No obstacle avoidance.
No real hardware gimbal validation requirement.
No native C++ MAVLink flight sink rewrite.
No overlay-side behavior inference.
No general mission planner.
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

Setup/layout validation:

```bash
bash -n setup.sh
bash -n cleanup.sh
bash -n simulation/airsim/run.sh
bash -n simulation/airsim/stop.sh

python3 -m py_compile \
  simulation/airsim/scripts/*.py \
  simulation/airsim/validation/*.py \
  tools/px4/*.py \
  tools/mission/*.py \
  tools/validation/*.py \
  tools/trajectory/*.py

du -sh \
  third_party/PX4-Autopilot \
  third_party/colosseum_environments \
  third_party/iceoryx_build 2>/dev/null

du -sh \
  PX4-Autopilot \
  simulation/PX4-Autopilot \
  simulation/airsim/PX4-Autopilot \
  simulation/colosseum_environments \
  simulation/airsim/colosseum_environments \
  infrastructure/iceoryx_build 2>/dev/null
```

AirSim runtime control:

```bash
# Start AirSim / PX4 SITL:
simulation/airsim/run.sh AirSimNH

# Stop AirSim / PX4 SITL:
simulation/airsim/stop.sh

# Use cleanup.sh only for reset/rebuild cleanup, not normal runtime stop.
```

AirSim camera/gimbal probe:

```bash
python3 simulation/airsim/scripts/airsim-camera-gimbal-probe.py \
  --host 127.0.0.1 \
  --rpc-port 41451 \
  --vehicle-name PX4 \
  --output-dir out/airsim_camera_gimbal_probe
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

Circle trajectory validator for current checked-in config (`orbit_count: 1.0`):

```bash
python3 tools/validation/validate-circle-trajectory.py \
  --events out/object_behavior_airsim_existing_object_circle/mission_events.jsonl \
  --min-orbits 1.0 \
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
- Do not use cleanup.sh as the normal way to stop AirSim/PX4 SITL. Use simulation/airsim/stop.sh for runtime shutdown; cleanup.sh is for reset/rebuild cleanup.
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