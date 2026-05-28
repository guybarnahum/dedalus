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
2.28C is complete: camera/gimbal target-stare policy, runtime dispatch, hardware/simulation sinks, and one-command AirSim mission workflow are validated.
2.29A-D are complete: multi-stage behavior sequence parsing/runtime, AirSim sequence configs, far-person SEL run, active step observability, and canonical per-step yaw/camera mode validation are validated through commit 3cc96a1.

Active next work: 2.29E — mixed-mode sequence validation, proving yaw_mode and camera_pointing_mode are independent across stages, e.g. approach yaw=target/camera=target and circle yaw=trajectory/camera=target.

Do not jump to moving SEL / animal yet unless explicitly requested. Moving targets should be 2.30 after mixed-mode static validation.

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
Milestone 2.28C: camera/gimbal pointing foundation. Complete after typed C++ CameraPointingCommand, lifecycle camera policy, runtime CameraPointingSink dispatch, native C++ MAVLink gimbal sink, AirSim camera bridge, one-command run_mission.sh, and canonical post-run validation.
Milestone 2.29A: behavior spec sequence schema foundation. Complete: BehaviorSpec carries optional yaw_mode and camera_pointing_mode per behavior node / sequence step.
Milestone 2.29B: sequence runtime execution. Complete: ObjectBehaviorMissionController executes approach -> circle sequence steps and applies per-step yaw/camera overrides during ExecuteMission.
Milestone 2.29C: AirSim sequence config and canonical sequence validation. Complete: sequence config/spec plus validate-mission-artifacts --expect-sequence and run_mission.sh sequence validation flags.
Milestone 2.29D: far static SEL and observability validation. Complete: BRPlayer_36 far-person sequence run validates active step fields and per-step yaw/camera mode assertions.
Milestone 2.29E: mixed-mode sequence validation. Active next slice.
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
       -> BehaviorSpec sequence runtime
            -> active_behavior() step selection
            -> approach standoff step
            -> robust circle/orbit step
            -> behavior_sequence_step_start / behavior_sequence_step_complete
            -> behavior_tick_sample active step observability
       -> VelocityCommand policy for vehicle translation/yaw
       -> CameraPointingCommand policy for target/home/landing/neutral camera pitch
  -> Px4BridgeCommandSink
       -> persistent tools/px4/px4-command-bridge.py
       -> PX4 shell: arm, takeoff, land, disarm
       -> pymavlink: OFFBOARD velocity setpoints
       -> LOCAL_POSITION_NED feedback climb to safe height
  -> CameraPointingSink
       -> NullCameraPointingSink by default
       -> MavlinkGimbalPointingSink for real PX4/MAVLink gimbals
       -> runtime event projection for AirSim camera bridge compatibility
  -> PX4 / AirSim
```

Runtime stream camera side-channel for AirSim:

```text
dedalus_mission_loop --world-snapshot-stream-port 47770
  -> mission_event camera_pointing_intent
  -> simulation/airsim/scripts/airsim-camera-pointing-bridge.py
  -> AirSim simSetCameraPose(front_center, pitch)
  -> AirSim simSetCameraPose(0, pitch)
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

simulation/airsim/run_mission.sh
  Starts mission-loop + AirSim camera bridge + overlay + post-run validation in tmux.
  It now has a friendly AirSim RPC preflight so running it before run.sh exits with useful instructions.
  It supports sequence validation flags:
    --expect-sequence
    --expect-sequence-steps approach,circle
    --expect-sequence-step-modes approach:target:target,circle:target:target
    --validation-complete-reason sequence_complete

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

tools/mission/
  Canonical mission artifact validators and summaries.

tools/validation/
  Behavior/trajectory-specific validators.

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

## 3. Completed Capabilities Through 2.29D

AirSim existing-object binding:

```text
config/core_stack_object_behavior_airsim_existing_object.yaml
  -> binds ghost_person_001 to BRPlayer_01_96 by default

config/core_stack_object_behavior_airsim_existing_object_circle.yml
  -> circle validation config using explicit existing-object binding

config/core_stack_object_behavior_airsim_existing_object_sequence.yml
  -> approach -> circle sequence config for BRPlayer_01_96 / ghost_person_001

config/core_stack_object_behavior_airsim_existing_object_sequence_far_person.yml
  -> approach -> circle sequence config for BRPlayer_36 / ghost_far_person_001

config/behaviors/circle_existing_object_person.yaml
  -> circle behavior spec for the validated existing-object path

config/behaviors/sequence_approach_circle_existing_object.yaml
  -> sequence behavior spec for ghost_person_001

config/behaviors/sequence_approach_circle_far_person.yaml
  -> sequence behavior spec for ghost_far_person_001 / BRPlayer_36

simulation/airsim/scripts/airsim-list-objects.py
  -> lists selectable scene objects by class/distance; used to pick BRPlayer_36 as a farther static person target

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

Sequence behavior:

```text
behavior.type: sequence is implemented in ObjectBehaviorMissionController.
The controller tracks sequence_step_index_ and active_behavior().
Approach steps complete when standoff is reached:
  range <= stop_distance_m + position_tolerance_m
Circle steps complete by orbit_count or duration_s.
Intermediate step completion emits:
  behavior_sequence_step_complete
  behavior_sequence_step_start for next step
Final step completion emits:
  behavior_complete reason=sequence_complete

Sequence step observability is present in behavior_tick_sample:
  active_behavior
  active_yaw_mode
  active_camera_pointing_mode
  sequence_step_index
  sequence_step_count
  sequence_step_behavior
  sequence_step_yaw_mode
  sequence_step_camera_pointing_mode
```

Target-stare yaw and camera/gimbal pitch:

```text
Horizontal target stare:
  C++ ObjectBehaviorMissionController owns yaw_mode=target and emits vehicle yaw through VelocityCommand.

Vertical camera stare:
  C++ ObjectBehaviorMissionController owns pitch policy and emits typed CameraPointingCommand.
  MissionRuntime dispatches camera_pointing through CameraPointingSink.
  MissionRuntime also writes camera_pointing_dispatch / camera_pointing_result events.
  ObjectBehaviorMissionController still emits camera_pointing_intent JSON for runtime-stream compatibility with the AirSim bridge.

Default lifecycle policy:
  Prepare        -> neutral, pitch 0
  Takeoff        -> neutral, pitch 0
  ExecuteMission -> target, unless active sequence step overrides camera_pointing_mode
  GoHome         -> home / recovery location
  Land           -> landing_area
  Complete       -> neutral, pitch 0

Validated AirSim camera mapping:
  front_center = Dedalus capture/perception camera.
  0 = AirSim operator FPV/F view camera candidate.
```

Real hardware gimbal path:

```text
MavlinkGimbalPointingSink is a native C++ CameraPointingSink for real PX4/MAVLink hardware.
It sends MAVLink Gimbal Manager pitch/yaw commands directly from dedalus_mission_loop.
Normal real-hardware path does not require tools/px4/mavlink-gimbal-pointing-bridge.py.
The Python MAVLink bridge remains a diagnostic/prototype tool.
```

AirSim camera path:

```text
AirSim still uses simulation/airsim/scripts/airsim-camera-pointing-bridge.py because AirSim camera tilt is simSetCameraPose, not PX4/MAVLink gimbal control.
This is intentional. Do not fake AirSim as MAVLink unless the simulator exposes an actual MAVLink gimbal component.
```

One-command AirSim mission workflow:

```text
simulation/airsim/run_mission.sh
  -> tmux session dedalus-mission
       camera-pointing
       overlay
       validation
       mission-loop

The validation window waits for runtime_stop, then composes canonical tools:
  tools/mission/mission-events-summary.py --expect-complete
  tools/mission/validate-mission-artifacts.py --expect-complete --expect-behavior --expect-camera-pointing ...
  tools/validation/validate-circle-trajectory.py ...
```

2.27B validation result:

```text
Full CTest passed: 34/34
Circle trajectory validator passed on a 3-orbit AirSim run.
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
Full CTest passed: 34/34.
AirSim camera/gimbal probe passed overall_ok=True.
Existing-object circle mission reached Complete / terminal_settled.
validate-circle-trajectory passed for configured orbit_count=1.0.
```

2.28C validation result:

```text
AirSim can change vehicle camera pitch at runtime through simSetCameraPose.
C++ emits typed CameraPointingCommand from selected-target/home/landing geometry.
MissionRuntime dispatches typed camera_pointing through CameraPointingSink.
The native C++ MAVLink gimbal sink builds successfully after chrono include fix.
The AirSim bridge consumes camera_pointing_intent and applies the same pitch to front_center and 0.
The AirSim bridge and overlay can exit on runtime_stop.
run_mission.sh starts mission-loop, AirSim camera bridge, overlay, and canonical validation in tmux.
Validation log now captures full validator output, not just final PASS.
```

2.29D far-person validation result at commit 3cc96a1:

```text
Target:
  BRPlayer_36 / ghost_far_person_001 / class person
  Selected because AirSim object list showed it as a farther static person target than BRPlayer_01_96.

Run command:
  cd ~/dedalus/simulation/airsim
  ./run_mission.sh \
    --config ../../config/core_stack_object_behavior_airsim_existing_object_sequence_far_person.yml \
    --output-dir ../../out/object_behavior_airsim_existing_object_sequence_far_person \
    --expect-sequence \
    --expect-sequence-steps approach,circle \
    --expect-sequence-step-modes approach:target:target,circle:target:target \
    --validation-complete-reason sequence_complete \
    --attach

Validation log highlights:
  final_state: Complete
  state_path: Idle -> Prepare -> Takeoff -> ExecuteMission -> GoHome -> Land -> Complete
  commands: Arm ok=1, Takeoff ok=1, Velocity ok=1133, Land ok=1, Disarm ok=2
  behavior_sequence_step_start: 2
  behavior_sequence_step_complete: 1
  sequence_started_steps: approach,circle
  sequence_completed_steps: approach
  sequence_step_modes:
    approach: yaw=target camera=target
    circle: yaw=target camera=target
  camera_pointing_events:
    camera_pointing_dispatch: 1447
    camera_pointing_intent: 1447
    camera_pointing_result: 1447
  camera_pointing_modes: home=516, landing_area=257, neutral=55, target=619
  camera_proof_frames: 0=113, front_center=113
  circle avg_abs_radius_error: 0.375m
  max_radius_error_after_latch: 1.805m
  completed_orbits: 0.986, required >= 0.950
  behavior_complete reason: sequence_complete
  runtime_stop terminal_settled: True
  validation: PASS
```

---

## 4. Active Next Work: 2.29E Mixed-Mode Sequence Validation

Goal:

```text
Prove stage-level yaw and camera pointing are independent by validating a sequence where yaw changes mode between stages while camera target-stare remains active.
```

Recommended 2.29E scope:

```text
1. Add a mixed-mode sequence behavior spec or config variant:
     approach:
       yaw_mode: target
       camera_pointing_mode: target
     circle:
       yaw_mode: trajectory
       camera_pointing_mode: target

2. Use the same far static person target first:
     BRPlayer_36 / ghost_far_person_001

3. Validate with canonical step-mode checks:
     --expect-sequence-step-modes approach:target:target,circle:trajectory:target

4. Confirm behavior_tick_sample shows:
     active_behavior: approach -> circle
     active_yaw_mode: target -> trajectory
     active_camera_pointing_mode: target -> target

5. Keep object_behavior_zero_target_velocity=true for static existing-object targets.
```

Non-goals for 2.29E:

```text
No obstacle avoidance.
No new planner.
No AirSim C++ RPC rewrite.
No mission DSL explosion.
No direct overlay-side behavior inference.
No moving SEL / animal yet.
```

Moving SEL / animal target guidance:

```text
Defer to 2.30 unless explicitly requested.
Before flying relative to an animal, prove pose is live and stable:

  python3 simulation/airsim/scripts/airsim-object-poses.py \
    --host 127.0.0.1 \
    --rpc-port 41451 \
    --objects <AnimalAIController_C_...>

Run repeatedly and verify x/y changes smoothly.
For moving targets, set:
  mission_options.object_behavior_zero_target_velocity: false
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
python3 -m py_compile \
  simulation/airsim/scripts/airsim-world-overlay.py \
  simulation/airsim/scripts/airsim-camera-pointing-bridge.py \
  tools/px4/mavlink-gimbal-pointing-bridge.py \
  tools/mission/validate-mission-artifacts.py \
  tools/mission/mission-events-summary.py \
  tools/validation/validate-circle-trajectory.py

bash -n simulation/airsim/run_mission.sh

ctest --test-dir build-staging --output-on-failure -R \
  'mission_artifact_validator|circle_trajectory_validator|mission_runtime|object_behavior_mission_controller|object_behavior_mission_smoke|core_stack_config_loader|behavior_spec|target_selector|world_snapshot_stream_server'
```

Setup/layout validation:

```bash
bash -n setup.sh
bash -n cleanup.sh
bash -n simulation/airsim/run.sh
bash -n simulation/airsim/run_mission.sh
bash -n simulation/airsim/stop.sh

python3 -m py_compile \
  simulation/airsim/scripts/*.py \
  simulation/airsim/validation/*.py \
  tools/px4/*.py \
  tools/mission/*.py \
  tools/validation/*.py \
  tools/trajectory/*.py
```

AirSim runtime control:

```bash
# Start AirSim / PX4 SITL:
simulation/airsim/run.sh AirSimNH

# Start object-conditioned behavior mission, camera bridge, overlay, and validation:
simulation/airsim/run_mission.sh --attach

# Stop AirSim / PX4 SITL:
simulation/airsim/stop.sh

# Use cleanup.sh only for reset/rebuild cleanup, not normal runtime stop.
```

Canonical circle AirSim mission validation:

```bash
tools/mission/mission-events-summary.py \
  out/object_behavior_airsim_existing_object_circle/mission_events.jsonl \
  --expect-complete

python3 tools/mission/validate-mission-artifacts.py \
  out/object_behavior_airsim_existing_object_circle \
  --expect-complete \
  --expect-behavior \
  --expect-camera-pointing \
  --expect-camera-modes neutral,target,home,landing_area \
  --camera-frames-dir out/object_behavior_airsim_existing_object_circle/camera_pointing_frames \
  --expect-camera-proof-frames \
  --safe-height-m 40 \
  --landed-height-m 1.0

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

Canonical far-person sequence AirSim validation:

```bash
cd ~/dedalus/simulation/airsim

./run_mission.sh \
  --config ../../config/core_stack_object_behavior_airsim_existing_object_sequence_far_person.yml \
  --output-dir ../../out/object_behavior_airsim_existing_object_sequence_far_person \
  --expect-sequence \
  --expect-sequence-steps approach,circle \
  --expect-sequence-step-modes approach:target:target,circle:target:target \
  --validation-complete-reason sequence_complete \
  --attach
```

Expected far-person validation evidence:

```text
sequence_started_steps: approach,circle
sequence_completed_steps: approach
sequence_step_modes:
  approach: yaw=target camera=target
  circle: yaw=target camera=target
camera_pointing_modes:
  neutral
  target
  home
  landing_area
Circle trajectory checks PASS
validation: PASS
```

Manual camera bridge validation:

```bash
rm -rf out/object_behavior_airsim_existing_object_circle/camera_pointing_frames
mkdir -p out/object_behavior_airsim_existing_object_circle/camera_pointing_frames

python3 simulation/airsim/scripts/airsim-camera-pointing-bridge.py \
  --stream-port 47770 \
  --host 127.0.0.1 \
  --rpc-port 41451 \
  --vehicle-name PX4 \
  --cameras front_center \
  --cameras 0 \
  --rate-hz 10 \
  --resend-s 0.25 \
  --verify-pose \
  --capture-dir out/object_behavior_airsim_existing_object_circle/camera_pointing_frames \
  --capture-every-s 1.0 \
  --debug \
  --debug-json out/object_behavior_airsim_existing_object_circle/camera_pointing_latest.json \
  --exit-on-runtime-stop
```

Expected camera pointing artifacts:

```text
camera_pointing_latest.json includes:
  cameras: ["front_center", "0"]
  per_camera_verify.front_center.accepted_pitch_deg
  per_camera_verify.0.accepted_pitch_deg

camera_pointing_frames includes paired frames such as:
  camera_pointing_00042_front_center_-074.95.png
  camera_pointing_00042_0_-074.95.png
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
  --debug-json out/object_behavior_airsim_existing_object_circle/overlay_debug_latest.json \
  --exit-on-runtime-stop
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
When generating manual patches for the user, always provide them in unified git diff format, suitable for `git apply`.
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
- Do not make the native C++ MAVLink sink the default live flight-control path; use px4_bridge for vehicle velocity/yaw flight control.
- Do not rewrite the working pymavlink vehicle-control path in C++ while stabilizing behavior.
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
- Do not move camera/gimbal pitch into VelocityCommand. Vehicle yaw and camera pitch are separate actuator paths.
- Do not pretend AirSim camera pitch is a PX4/MAVLink gimbal unless the simulator exposes an actual MAVLink gimbal component.
- Do not create duplicate mission validators. Extend tools/mission/validate-mission-artifacts.py and compose existing tools.
- Do not add shims to preserve stale pre-refactor APIs unless there is a current user and an explicit removal plan.
- Do not jump to moving SEL / animal before mixed-mode static sequence validation unless explicitly requested.
- Do not create branches or PRs unless explicitly requested.
```

---

## 8. Pointers

```text
docs/camera_pointing_intent.md              camera/gimbal policy, sinks, lifecycle modes, AirSim/MAVLink split
docs/runtime_dataflow.md                    source->publisher->server->subscriber->sink diagrams
docs/flight_behavior_control_laws.md        robust behavior control-law principle, especially circle/orbit
docs/airsim_existing_object_ghost_runbook.md AirSim existing-object validation
docs/object_behavior_airsim_ghost_runbook.md AirSim ghost behavior + live stream runbook
docs/object_conditioned_behavior_plan.md    detailed M3 behavior + identity plan
docs/mission_scenario_runner.md            scenario/campaign harness
docs/world_model_reprojection_validation_plan.md reprojection and world-model evidence plan
docs/mission_pipeline_current_state.md      mission loop architecture
docs/core_stack_current_state.md            broader core-stack status
docs/llm_connector_patch_policy.md         connector/manual patch safety policy
WHITEPAPER.md                               architectural rationale
HANDOFF.md                                  handoff prompt template
LLM.back.md                                 historical context only
```