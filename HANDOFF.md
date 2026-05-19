# Dedalus Handoff Template

This file is a template for generating LLM handoff prompts.

To generate a current handoff, read `LLM.md` and the current repo state, then fill in the template below.

**Usage trigger:** `use HANDOFF.md to generate a handoff prompt from our current state`

---

## How to Generate a Handoff

1. Read `LLM.md` first. Treat it as the active operating brief.
2. Read `docs/mission_scenario_runner.md` for the current scenario/campaign harness workflow.
3. Read `docs/object_conditioned_behavior_plan.md` before M3 behavior work.
4. Read `docs/world_model_reprojection_validation_plan.md` before reprojection, annotation, or world-model evidence work.
5. Read `WHITEPAPER.md` when architectural rationale is needed.
6. Read `LLM.back.md` only for historical context when needed.
7. Run `git log --oneline -1` to get the current commit SHA.
8. Substitute all `<PLACEHOLDER>` values below with current state.
9. Emit the filled-in prompt as plain text — no surrounding explanation.

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
  docs/mission_scenario_runner.md
  docs/object_conditioned_behavior_plan.md
  docs/world_model_reprojection_validation_plan.md

Historical context, only if needed:
  LLM.back.md

Active milestone:
  <ACTIVE_MILESTONE_AND_STAGE>

Current architecture:
  <DATA_FLOW_DIAGRAM — copy from LLM.md §1 or update if pipeline changed>

Current observed behavior:
  <MOST_RECENT_NOTABLE_LOGS_OR_RUNTIME_OUTPUT>

Current diagnosis:
  <WHAT_IS LIKELY MISSING OR BROKEN>

Immediate tasks:
  <NUMBERED_TASK_LIST — copy from LLM.md recommended next stage or update to reflect current state>

Do not:
  <DO_NOT_LIST — copy from LLM.md Known Traps and add any session-specific traps>

Patch policy:
  Apply changes directly to main.
  Do not create branches or PRs unless explicitly requested.
  If GitHub connector patching fails or is ambiguous, provide an exact manual patch.

Validation:
  Always give build/test commands after code patches:

    cmake --build build-staging -j$(nproc)
    ctest --test-dir build-staging --output-on-failure

  For reprojection/world-evidence changes, also include:

    ctest --test-dir build-staging --output-on-failure -R 'world_to_image_projector|world_reprojection_artifacts'

  For scenario/campaign harness changes, also include:

    ctest --test-dir build-staging --output-on-failure -R 'mission_(scenario|campaign|abort)_'

  For live mission changes, also include one of:

    RUNS=3 simulation/repeat-mission-smoke.sh

    python3 simulation/run-mission-campaign.py \
      --campaign-file config/mission_campaigns/airsim_live_smoke.json \
      --campaign-id live_<N> \
      --output-root out/mission_campaigns \
      --progress \
      --overwrite

Expected success:
  <SPECIFIC_LOG_STATE_OR_TEST_RESULT_THAT SIGNALS TASK COMPLETE>
```

---

## Reference — Current Strategic Handoff Shape

This reference is valid after the 2.25 prep cleanup. When generating a new handoff, update the commit SHA and any current runtime observations.

```text
You are continuing work on the Dedalus repo.

Repository:
  guybarnahum/dedalus

Current commit:
  <CURRENT_COMMIT_SHA>

First read:
  LLM.md

Then read:
  docs/mission_scenario_runner.md
  docs/object_conditioned_behavior_plan.md
  docs/world_model_reprojection_validation_plan.md

Historical context, only if needed:
  LLM.back.md

Active milestone:
  Milestone 2.25 — ObjectBehaviorMissionController skeleton

Current architecture:
  AirSim live frame + ego sidecar
    -> AirSimFrameSource
    -> FrameHintEgoProvider
    -> CoreStackRunner
    -> InMemoryWorldModel
    -> LatestWorldSnapshot
    -> MissionRuntime async loop
    -> TrajectoryMissionController today / ObjectBehaviorMissionController next
    -> Px4BridgeCommandSink
    -> simulation/px4-command-bridge.py
    -> PX4 / AirSim

  Milestone 3 target architecture:
    AirSim live frame + ego sidecar
      -> AirSimFrameSource
      -> detector / tracker / projector, or ghost/scripted target provider for pre-camera validation
      -> WorldSnapshot agents with agent_id, source_track_id, identity_id, class, confidence, local position, velocity, latest_view_evidence when camera-derived
      -> TargetSelector
      -> BehaviorRuntime / ObjectBehaviorMissionController
      -> desired velocity vector
      -> Px4BridgeCommandSink
      -> PX4 / AirSim

Current observed behavior:
  Milestones 2.20, 2.21, 2.22, and 2.23 are implemented / validated.

  Milestone 2.24 is implemented through 2.24G.9 baseline:
    - track-addressable WorldSnapshot agents
    - TargetSelectorSpec track_id / agent_id support
    - TargetSelector policies and tests
    - ghost/scripted targets for pre-camera validation
    - WorldSnapshot-to-camera reprojection projector
    - projected AG markers in annotation artifacts
    - frame_XXXXXX.world_overlay.json sidecars
    - camera-derived 2D provenance carried into AgentState.latest_view_evidence
    - reprojection residual metadata for camera-derived agents
    - world_reprojection_artifacts synthetic test validates PPM marker, sidecar metadata, residuals, and snapshot latest_view_evidence

  Behavior specs are now canonical runtime/autonomy config under:
    config/behaviors/
      follow_person.yaml
      follow_specific_track.yaml
      circle_car.yaml
      approach_target.yaml
      sequence_approach_circle.yaml

  Simulation-only target fixtures remain under:
    simulation/ghost_targets/person_pair_crossing.yaml

Current diagnosis:
  The drone can fly and validate reliable preconfigured missions and scenario/campaign artifacts. Behavior specs parse, TargetSelector works, ghost targets and world-model reprojection validation work. The next gap to M3 is wiring ObjectBehaviorMissionController into the mission path so behavior can consume WorldSnapshot + behavior specs and emit target/behavior events safely.

Important architectural rules:
  PerceptionPipelineOutput is evidence.
  WorldSnapshot is autonomy state.
  Behavior consumes WorldSnapshot, not perception internals.

  Behavior specs are global autonomy/runtime configuration, not simulation assets.
  Use config/behaviors/*.yaml for behavior specs.
  Use simulation/ghost_targets/*.yaml only for simulation/synthetic target fixtures.

Immediate tasks:
  1. Build/test current head:

       cmake --build build-staging -j$(nproc)
       ctest --test-dir build-staging --output-on-failure

  2. Start Milestone 2.25A/B:
       - add ObjectBehaviorMissionController skeleton
       - add config path for mission_controller: object_behavior
       - load behavior spec from config/behaviors/follow_specific_track.yaml
       - consume LatestWorldSnapshot / WorldSnapshot agents
       - run TargetSelector during ExecuteMission
       - emit target_selected, behavior_start, behavior_complete events
       - emit safe zero/hold velocity only; no follow/circle motion math yet
       - complete normally through GoHome -> Land -> Disarm

  3. Add tests:
       - unit test ObjectBehaviorMissionController target selection and no-op/hold command behavior
       - synthetic scenario using ghost_person_001 validates stable selected lower-confidence target
       - mission_events prove target_selected, behavior_start, behavior_complete

Do not:
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
  - Do not collapse detection_id/track_id/source_track_id/agent_id/identity_id/latest_view_evidence.
  - Do not select targets only by confidence when a stable track/agent target is specified.
  - Do not bypass WorldSnapshot/TargetSelector by hardcoding selected_target in config for main validation.
  - Do not keep global behavior specs under simulation/behaviors; use config/behaviors.
  - Do not create branches or PRs unless explicitly requested.

Patch policy:
  Apply changes directly to main.
  Do not create branches or PRs unless explicitly requested.
  If GitHub connector patching fails or is ambiguous, provide an exact manual patch.

Validation:
  Always give build/test commands after code patches:

    cmake --build build-staging -j$(nproc)
    ctest --test-dir build-staging --output-on-failure

  For current behavior/reprojection work:

    ctest --test-dir build-staging --output-on-failure -R 'behavior_spec|target_selector|world_to_image_projector|world_reprojection_artifacts'

  For scenario/campaign harness changes:

    ctest --test-dir build-staging --output-on-failure -R 'mission_(scenario|campaign|abort)_'

  For live mission changes:

    RUNS=3 simulation/repeat-mission-smoke.sh

    python3 simulation/run-mission-campaign.py \
      --campaign-file config/mission_campaigns/airsim_live_smoke.json \
      --campaign-id live_<N> \
      --output-root out/mission_campaigns \
      --progress \
      --overwrite

Expected success:
  For 2.25 skeleton:
    ObjectBehaviorMissionController consumes WorldSnapshot agents through TargetSelector.
    target_selected includes agent_id and source_track_id.
    behavior_start and behavior_complete are emitted.
    selected ghost_person_001 remains stable despite ghost_person_002 having higher confidence.
    Mission still reaches GoHome -> Land -> Complete/status=complete.
```
