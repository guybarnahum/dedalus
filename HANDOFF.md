# Dedalus Handoff Template

This file is a template for generating LLM handoff prompts.

To generate a current handoff, read `LLM.md` and the current repo state, then fill in the template below.

**Usage trigger:** `use HANDOFF.md to generate a handoff prompt from our current state`

---

## How to Generate a Handoff

1. Read `LLM.md` first. Treat it as the active operating brief.
2. Read `docs/runtime_dataflow.md` for the current source -> publisher -> server -> subscriber -> sink architecture.
3. Read `docs/object_conditioned_behavior_plan.md` before 2.27 / M3 behavior work.
4. Read `docs/airsim_existing_object_ghost_runbook.md` before AirSim existing-object validation.
5. Read `docs/mission_scenario_runner.md` for the scenario/campaign harness workflow.
6. Read `docs/world_model_reprojection_validation_plan.md` before reprojection, annotation, or world-model evidence work.
7. Read `WHITEPAPER.md` when architectural rationale is needed.
8. Read `docs/milestone_2_30a_slow_moving_sel_animal_validation.md` and `docs/milestone_2_30b_results.md` when continuing moving-target/object-conditioned behavior work.
9. Read `LLM.back.md` only for historical context when needed; `LLM.md` is authoritative.
10. Run `git log --oneline -1` to get the current commit SHA.
11. Substitute all `<PLACEHOLDER>` values below with current state.
12. Emit the filled-in prompt as plain text — no surrounding explanation.

---

## Template

```text
You are continuing work on the Dedalus repo.

Repository:
  guybarnahum/dedalus

Current commit:
  <CURRENT_COMMIT_SHA>

First read:
  LLM.md

Then read:
  docs/runtime_dataflow.md
  docs/object_conditioned_behavior_plan.md
  docs/airsim_existing_object_ghost_runbook.md
  docs/mission_scenario_runner.md
  docs/world_model_reprojection_validation_plan.md

Historical context, only if needed:
  LLM.back.md

Active milestone:
  <ACTIVE_MILESTONE_AND_STAGE>

Current architecture:
  <DATA_FLOW_DIAGRAM — copy from LLM.md Current Runtime Architecture or update if pipeline changed>

Current observed behavior:
  <MOST_RECENT_NOTABLE_LOGS_OR_RUNTIME_OUTPUT>

Current diagnosis:
  <WHAT_IS_VERIFIED, WHAT_IS_STILL_UNVALIDATED, AND WHAT_IS LIKELY MISSING OR BROKEN>

Immediate tasks:
  <NUMBERED_TASK_LIST — copy from LLM.md Active Next Work or update to reflect current state>

Do not:
  <DO_NOT_LIST — copy from LLM.md Known Traps and add any session-specific traps>

Patch policy:
  Apply changes directly to main.
  Do not create branches or PRs unless explicitly requested.
  If GitHub connector patching fails, is ambiguous, is blocked, or would require a risky broad rewrite, stop using the connector for that code change.
  When generating manual patches for the user, always provide one unified git diff suitable for git apply.
  Generate an exact manual patch and ask the user to apply it locally.
  Do not keep retrying increasingly complex connector paths after a connector failure.

Validation:
  Always give build/test commands after code patches:

    cmake --build build-staging -j$(nproc)
    ctest --test-dir build-staging --output-on-failure

  For current behavior/runtime changes, also include:

    python3 -m py_compile simulation/airsim/scripts/airsim-world-overlay.py

    ctest --test-dir build-staging --output-on-failure -R \
      'mission_runtime|object_behavior_mission_controller|object_behavior_mission_smoke|core_stack_config_loader|behavior_spec|target_selector|world_snapshot_stream_server'

  For live AirSim validation, include:

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

    python3 simulation/airsim/scripts/airsim-world-overlay.py \
      --stream-port 47770 \
      --follow \
      --rate-hz 5 \
      --duration-s 180 \
      --clear \
      --label \
      --osd \
      --debug \
      --debug-json out/object_behavior_airsim_existing_object/overlay_debug_latest.json

Expected success:
  <SPECIFIC_LOG_STATE_OR_TEST_RESULT_THAT SIGNALS TASK COMPLETE>
```

---

## Reference — Current Strategic Handoff Shape

This reference reflects the state after 2.26E validation. When generating a new handoff, update the commit SHA and any current runtime observations.

```text
You are continuing work on the Dedalus repo.

Repository:
  guybarnahum/dedalus

Current commit:
  <CURRENT_COMMIT_SHA>

First read:
  LLM.md

Then read:
  docs/runtime_dataflow.md
  docs/object_conditioned_behavior_plan.md
  docs/airsim_existing_object_ghost_runbook.md
  docs/mission_scenario_runner.md
  docs/world_model_reprojection_validation_plan.md

Historical context, only if needed:
  LLM.back.md

Active milestone:
  Milestone 2.27A — Circle behavior with velocity-matched orbit insertion.

Current architecture:
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
    -> PX4 / AirSim

  RuntimeEventStreamServer emits one TCP JSONL stream:
    ghost_detections
    world_snapshot
    mission_event

  simulation/airsim/scripts/airsim-world-overlay.py is a stream-only subscriber/renderer.
  It renders PLAN / PLAN* from ghost_detections, AG / EGO from world_snapshot, SEL from mission_event target_selected, and OSD from mission_event display_state/display_detail.

Current observed behavior:
  2.26E is complete and live-validated by the operator.
  AirSim existing-object binding works for explicit object names.
  config/core_stack_object_behavior_airsim_existing_object.yaml binds ghost_person_001 to BRPlayer_01_96 by default.
  Follow behavior uses target-relative observation geometry and target_velocity + closing_velocity arrival control.
  Follow validation reached GoHome -> Land -> Disarm -> Settled without failures.
  Overlay OSD renders DEDALUS and DEDALUS-STATE from runtime events.
  Behavior display semantics now come from mission/behavior events, not overlay inference.

Current diagnosis:
  Follow behavior and live overlay are good enough to use as the baseline for additional object-conditioned behaviors. The next gap is circle behavior. Circle should not be implemented as follow-with-sideways-offset; it needs explicit orbit geometry, orbit entry, tangential velocity matching, radial correction, and behavior display details.

Immediate tasks:
  1. Inspect current behavior spec and implementation:
       - config/behaviors/circle_car.yaml
       - include/dedalus/behavior/behavior_spec.hpp
       - src/behavior/behavior_spec.cpp
       - src/behavior/object_behavior_mission_controller.cpp
       - tests/unit/test_object_behavior_mission_controller.cpp

  2. Prepare a short implementation plan for 2.27A before coding because circle behavior is architecture/runtime-sensitive.

  3. Implement circle behavior in clean phases:
       - arriving: approach the 3 o'clock orbit entry point relative to the target
       - velocity-match: arrive with target_velocity + desired tangent velocity
       - circling: maintain target_velocity + tangent_velocity + radial_correction
       - complete: reuse existing duration / finish path into GoHome -> Land -> Disarm -> Settled

  4. Add/extend debug events:
       - circle_phase
       - orbit_radius_m
       - actual_radius_m
       - radius_error_m
       - radial_correction_mps
       - tangent_velocity_mps
       - target_velocity_mps
       - desired_velocity_mps
       - display_state=Mission
       - display_detail=arriving or circling

  5. Add tests before AirSim validation:
       - static target orbit-entry command points toward 3 o'clock entry
       - moving target command includes target velocity
       - circling command includes tangent velocity with correct sign/direction
       - radial correction pushes outward/inward with correct sign
       - command speed is clamped
       - behavior display detail transitions arriving -> circling

  6. Validate in AirSim using the existing explicit object binding and overlay OSD.

Do not:
  - Do not use dedalus_replay_mission. Use dedalus_mission_loop.
  - Do not treat command helper OK as vehicle-state truth.
  - Do not move to ExecuteMission until Takeoff is confirmed by ego height.
  - Do not stop at raw Complete state; wait for terminal_settled / Complete status=complete.
  - Do not make the native C++ MAVLink sink the default live path; use px4_bridge.
  - Do not rewrite the working pymavlink control path in C++ while stabilizing behavior.
  - Do not put obstacle avoidance inside the flight sink.
  - Do not let Milestone 3 balloon into full obstacle avoidance; M3 is object-conditioned behavior. Avoidance starts post-M3.
  - Do not collapse track_id/source_track_id/agent_id/identity_id into one field.
  - Do not select targets only by confidence when a stable track/agent target is specified.
  - Do not bypass WorldSnapshot/TargetSelector by hardcoding selected_target in config for main validation.
  - Do not keep global behavior specs under simulation/behaviors; use config/behaviors.
  - Do not use artifact files as runtime IPC when a live stream or in-process subscriber is the right boundary.
  - Do not make simulation/airsim/scripts/airsim-world-overlay.py evaluate GhostScenario, discover AirSim objects, or poll snapshot artifacts in normal mode; it should subscribe and render.
  - Do not put behavior semantics in overlay logic. Mission/behavior events should publish display_state/display_detail; overlay should render them.
  - Do not implement circle as a yaw-only or latch-based visual hack.
  - Do not add shims to preserve stale pre-refactor APIs unless there is a current user and an explicit removal plan.
  - Do not create branches or PRs unless explicitly requested.

Patch policy:
  Apply changes directly to main.
  Do not create branches or PRs unless explicitly requested.
  If GitHub connector patching fails, is ambiguous, is blocked, or would require a risky broad rewrite, stop using the connector for that code change.
  Generate an exact manual patch and ask the user to apply it locally.
  Do not keep retrying increasingly complex connector paths after a connector failure.

Validation:
  Always give build/test commands after code patches:

    cmake --build build-staging -j$(nproc)
    ctest --test-dir build-staging --output-on-failure

  Focused 2.27A:

    python3 -m py_compile simulation/airsim/scripts/airsim-world-overlay.py

    ctest --test-dir build-staging --output-on-failure -R \
      'mission_runtime|object_behavior_mission_controller|object_behavior_mission_smoke|core_stack_config_loader|behavior_spec|target_selector|world_snapshot_stream_server'

Expected success:
  Build and focused tests pass.
  Circle behavior emits bounded velocity commands.
  Circle debug events expose orbit-entry and orbit-maintenance geometry.
  mission_events.jsonl shows Mission / arriving, then Mission / circling.
  AirSim overlay shows PLAN / AG / SEL over the selected object and DEDALUS-STATE changes to Mission / circling.
  Circle mission reaches GoHome -> Land -> Disarm -> Settled without failures.
```
