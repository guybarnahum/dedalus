# Dedalus Handoff Template

This file is a template for generating LLM handoff prompts.

To generate a current handoff, read `LLM.md` and the current repo state, then fill in the template below.

**Usage trigger:** `use HANDOFF.md to generate a handoff prompt from our current state`

---

## How to Generate a Handoff

1. Read `LLM.md` first. Treat it as the active operating brief.
2. Read `docs/runtime_dataflow.md` for the current source -> publisher -> server -> subscriber -> sink architecture.
3. Read `docs/airsim_existing_object_ghost_runbook.md` before AirSim existing-object validation.
4. Read `docs/mission_scenario_runner.md` for the scenario/campaign harness workflow.
5. Read `docs/object_conditioned_behavior_plan.md` before M3 / 2.27 behavior work.
6. Read `docs/world_model_reprojection_validation_plan.md` before reprojection, annotation, or world-model evidence work.
7. Read `WHITEPAPER.md` when architectural rationale is needed.
8. Read `LLM.back.md` only for historical context when needed; `LLM.md` is authoritative.
9. Run `git log --oneline -1` to get the current commit SHA.
10. Substitute all `<PLACEHOLDER>` values below with current state.
11. Emit the filled-in prompt as plain text — no surrounding explanation.

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
  docs/airsim_existing_object_ghost_runbook.md
  docs/mission_scenario_runner.md
  docs/object_conditioned_behavior_plan.md
  docs/world_model_reprojection_validation_plan.md

Historical context, only if needed:
  LLM.back.md

Active milestone:
  <ACTIVE_MILESTONE_AND_STAGE>

Current architecture:
  <DATA_FLOW_DIAGRAM — copy from LLM.md Current Architecture or update if pipeline changed>

Current observed behavior:
  <MOST_RECENT_NOTABLE_LOGS_OR_RUNTIME_OUTPUT>

Current diagnosis:
  <WHAT_IS_VERIFIED, WHAT_IS_STILL_UNVALIDATED, AND WHAT_IS_LIKELY MISSING OR BROKEN>

Immediate tasks:
  <NUMBERED_TASK_LIST — copy from LLM.md Immediate Next Tasks or update to reflect current state>

Do not:
  <DO_NOT_LIST — copy from LLM.md Known Traps and add any session-specific traps>

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

  For current 2.26D/E runtime stream and object behavior changes, also include:

    python3 -m py_compile simulation/airsim-world-overlay.py

    ctest --test-dir build-staging --output-on-failure -R \
      'mission_runtime|object_behavior_mission_controller|object_behavior_mission_smoke|core_stack_config_loader|behavior_spec|target_selector|world_snapshot_stream_server'

  For live AirSim existing-object validation, include:

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

Expected success:
  <SPECIFIC_LOG_STATE_OR_TEST_RESULT_THAT SIGNALS TASK COMPLETE>
```

---

## Reference — Current Strategic Handoff Shape

This reference reflects the 2.26E state after commit `c1ba05e` and the LLM docs refresh. When generating a new handoff, update the commit SHA and any current runtime observations.

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
  docs/airsim_existing_object_ghost_runbook.md
  docs/mission_scenario_runner.md
  docs/object_conditioned_behavior_plan.md
  docs/world_model_reprojection_validation_plan.md

Historical context, only if needed:
  LLM.back.md

Active milestone:
  Milestone 2.26E — AirSim existing-object object behavior, follow arrival control, and live overlay / OSD validation.

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
    -> persistent simulation/px4-command-bridge.py
    -> PX4 / AirSim

  RuntimeEventStreamServer emits one TCP JSONL stream:
    ghost_detections
    world_snapshot
    mission_event

  simulation/airsim-world-overlay.py is a stream-only subscriber/renderer.
  It renders PLAN / PLAN* from ghost_detections, AG / EGO from world_snapshot, SEL from mission_event target_selected, and OSD from mission_event display_state/display_detail.

Current observed behavior:
  2.26D runtime-event plumbing is implemented.
  2.26E AirSim existing-object binding is implemented for explicit object names.
  config/core_stack_object_behavior_airsim_existing_object.yaml binds ghost_person_001 to BRPlayer_01_96 by default.
  Follow behavior uses target-relative observation geometry and target_velocity + closing_velocity arrival control.
  MissionRuntime emits lifecycle/command display_state/display_detail.
  ObjectBehaviorMissionController emits behavior display_detail such as arriving, following, positioned, circling, and done.
  Overlay OSD renders DEDALUS and DEDALUS-STATE lines and includes an optional EGO XY velocity arrow.
  GitHub status checks may be absent; local build/test and AirSim validation are required.

Current diagnosis:
  Source structure matches the current design. Remaining risk is live validation quality: marker accumulation/blinking, OSD display stability, and AirSim mission behavior around the visible bound object. Behavior semantics should remain in mission/behavior events, not in the overlay.

Immediate tasks:
  1. Pull current head and run focused validation:

       python3 -m py_compile simulation/airsim-world-overlay.py
       cmake --build build-staging -j$(nproc)
       ctest --test-dir build-staging --output-on-failure -R 'mission_runtime|object_behavior_mission_controller|object_behavior_mission_smoke|core_stack_config_loader|behavior_spec|target_selector|world_snapshot_stream_server'

  2. Run live AirSim existing-object validation:
       - confirm BRPlayer_01_96 exists with simulation/airsim-object-poses.py
       - run dedalus_mission_loop with --world-snapshot-stream-port 47770
       - run simulation/airsim-world-overlay.py with --osd and --debug-json
       - verify PLAN / AG / SEL align over the visible object
       - verify DEDALUS-STATE shows Mission / arriving -> following or positioned, then GoHome / returning, Land / landing, Disarm / ok, Settled / done

  3. Confirm follow behavior:
       - no spin near/above SEL
       - arrival_mode transitions cruise/slow/hold as expected
       - relative_speed_xy_mps trends toward zero near arrival

  4. If validation passes, record 2.26E complete and start 2.27A.

  5. 2.27A proposed next slice:
       - implement circle behavior with velocity-matched orbit insertion
       - approach a 3 o'clock orbit entry point
       - blend into tangential velocity before capture
       - maintain orbit using target_velocity + tangent_velocity + radial correction
       - emit behavior display details: arriving -> circling
       - keep overlay renderer-only

Do not:
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

  Focused 2.26E:

    python3 -m py_compile simulation/airsim-world-overlay.py

    ctest --test-dir build-staging --output-on-failure -R \
      'mission_runtime|object_behavior_mission_controller|object_behavior_mission_smoke|core_stack_config_loader|behavior_spec|target_selector|world_snapshot_stream_server'

Expected success:
  Build and focused tests pass.
  mission_events.jsonl contains display_state/display_detail.
  Overlay shows stable DEDALUS and DEDALUS-STATE lines.
  PLAN / AG / SEL align over the explicitly bound AirSim object.
  Follow behavior reaches GoHome -> Land -> Disarm -> Settled without failures.
```
