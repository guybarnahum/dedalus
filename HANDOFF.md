# Dedalus Handoff Template

This file is a template for generating LLM handoff prompts.

To generate a current handoff, read `LLM.md` and the current repo state, then fill in the template below.

**Usage trigger:** `use HANDOFF.md to generate a handoff prompt from our current state`

---

## How to Generate a Handoff

1. Read `LLM.md` first. Treat it as the active operating brief.
2. Read `docs/mission_scenario_runner.md` for the current scenario/campaign harness workflow.
3. Read `docs/object_conditioned_behavior_plan.md` before Milestone 2.23 / 2.24 / M3 behavior work.
4. Read `WHITEPAPER.md` when architectural rationale is needed.
5. Read `LLM.back.md` only for historical context when needed.
6. Run `git log --oneline -1` to get the current commit SHA.
7. Substitute all `<PLACEHOLDER>` values below with current state.
8. Emit the filled-in prompt as plain text — no surrounding explanation.

---

## Template

```
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

Historical context, only if needed:
  LLM.back.md

Active milestone:
  <ACTIVE_MILESTONE_AND_STAGE>

Current architecture:
  <DATA_FLOW_DIAGRAM — copy from LLM.md §1 or update if pipeline changed>

Current observed behavior:
  <MOST_RECENT_NOTABLE_LOGS_OR_RUNTIME_OUTPUT>

Current diagnosis:
  <WHAT_IS_LIKELY_MISSING_OR_BROKEN>

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

This reference is valid after Milestone 2.23 and during Milestone 2.24. When generating a new handoff, update the commit SHA and any current runtime observations.

```
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

Historical context, only if needed:
  LLM.back.md

Active milestone:
  Milestone 2.24 — TargetSelector from WorldSnapshot agents

Current architecture:
  AirSim live frame + ego sidecar
    -> AirSimFrameSource
    -> FrameHintEgoProvider
    -> CoreStackRunner
    -> InMemoryWorldModel
    -> LatestWorldSnapshot
    -> MissionRuntime async loop
    -> TrajectoryMissionController
    -> Px4BridgeCommandSink
    -> simulation/px4-command-bridge.py
    -> PX4 / AirSim

  Milestone 3 target architecture:
    AirSim live frame + ego sidecar
      -> AirSimFrameSource
      -> detector / tracker / projector, or ghost/scripted target provider for pre-camera validation
      -> WorldSnapshot agents with agent_id, source_track_id, identity_id, class, confidence, local position, velocity
      -> TargetSelector
      -> BehaviorRuntime / ObjectBehaviorMissionController
      -> desired velocity vector
      -> Px4BridgeCommandSink
      -> PX4 / AirSim

Current observed behavior:
  Milestones 2.20, 2.21, 2.22, and 2.23 are implemented.

  The live AirSim/PX4 mission loop works through flight_command_sink=px4_bridge:
    - takeoff reaches safe height
    - trajectory executes
    - GoHome / Land / Disarm complete
    - repeated mission runs work without restarting AirSim
    - mission_events.jsonl validates successful runs

  Scenario/campaign harness is available:
    - run-mission-scenario.py runs one archive-grade scenario
    - run-mission-campaign.py runs/dry-runs campaigns
    - validate-mission-artifacts.py validates Complete and Abort final states
    - campaign reports write JSON, text, and Markdown summaries
    - Ctrl-C during a campaign gracefully finishes the active mission and then stops the campaign

  Behavior spec parser foundation is available:
    - include/dedalus/behavior/behavior_spec.hpp
    - src/behavior/behavior_spec.cpp
    - tests/unit/test_behavior_spec.cpp
    - sample specs under simulation/behaviors/

  Milestone 2.24A track-addressable WorldSnapshot agents is implemented:
    - AgentState.agent_id is derived from Observation3D.track_id
    - AgentState.identity_id is derived from Observation3D.track_id
    - AgentState.source_track_id preserves the tracker ID
    - WorldSnapshot JSON emits source_track_id
    - tests cover multi-agent track preservation and JSON artifacts

Current diagnosis:
  The drone can fly and validate reliable preconfigured missions and scenario/campaign artifacts. Behavior specs parse. The gap to Milestone 3 is target selection and object-conditioned behavior. Next step: make TargetSelectorSpec and TargetSelector explicitly support class, source_track_id/track_id, agent_id, and persistent target selection from WorldSnapshot agents.

Important identity model:
  Do not collapse detection_id, track_id, source_track_id, agent_id, and identity_id.

  detection_id:
    single detector observation in one frame

  track_id:
    tracker-owned frame-to-frame continuity

  source_track_id:
    tracker ID preserved inside WorldSnapshot AgentState as provenance

  agent_id:
    world-model-owned object handle that behavior/planning should select

  identity_id:
    recognized real-world identity, future-facing for people/vehicles/drones across missions

Immediate tasks:
  1. Build/test current head:

       cmake --build build-staging -j$(nproc)
       ctest --test-dir build-staging --output-on-failure

  2. Continue Milestone 2.24:
       - extend TargetSelectorSpec with optional track_id and agent_id fields
       - validate at least one of class, track_id, or agent_id is present
       - add TargetSelection output with agent_id, source_track_id, identity_id, class, confidence, position, velocity, status, reason
       - implement highest_confidence, nearest, and persistent_track policies
       - add tests proving explicit track/agent selection from groups and persistence over higher-confidence neighbors

  3. Plan expressive pre-camera validation:
       - add ghost/scripted targets that enter as PerceptionPipelineOutput.observations
       - feed them through InMemoryWorldModel into WorldSnapshot.agents
       - validate TargetSelector over ghost_person_001 vs ghost_person_002 instead of directly setting selected_target from config

Do not:
  - Do not use dedalus_replay_mission. Use dedalus_mission_loop.
  - Do not treat command helper OK as vehicle-state truth.
  - Do not hide arming inside velocity commands.
  - Do not collapse flight_control.arm_state and ego.armed.
  - Do not move to ExecuteMission until Takeoff is confirmed by ego height.
  - Do not make the native C++ MAVLink sink the default live path; use px4_bridge.
  - Do not rewrite the working pymavlink control path in C++ while stabilizing behavior.
  - Do not let human diagnostics contaminate binary bridge stdout; binary frame bridge stdout is protocol bytes only.
  - Do not make mission_campaign_runner CTest execute repeated real missions.
  - Do not put obstacle avoidance inside the flight sink.
  - Do not let route memory override fresh tactical sensing.
  - Do not let Milestone 3 balloon into full obstacle avoidance; M3 is object-conditioned behavior. Avoidance starts post-M3.
  - Do not collapse track_id/source_track_id/agent_id/identity_id into one field.
  - Do not select targets only by confidence when a stable track/agent target is specified.
  - Do not bypass WorldSnapshot/TargetSelector by hardcoding selected_target in config for main validation.
  - Do not create branches or PRs unless explicitly requested.

Patch policy:
  Apply changes directly to main.
  Do not create branches or PRs unless explicitly requested.
  If GitHub connector patching fails or is ambiguous, provide an exact manual patch.

Validation:
  Always give build/test commands after code patches:

    cmake --build build-staging -j$(nproc)
    ctest --test-dir build-staging --output-on-failure

  For current 2.24A / selector work:

    ctest --test-dir build-staging --output-on-failure -R 'world_snapshot_json|perception_world_model_flow|behavior_spec|target_selector'

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
  For 2.24:
    TargetSelector can deterministically select one WorldSnapshot agent by class, source_track_id/track_id, or agent_id.
    persistent_track keeps the prior selected object even when a neighboring same-class object has higher confidence.
    Tests prove no direct selected_target shortcut is needed.

  For later M3:
    mission_events + snapshots should prove:
      target_selected with agent_id and source_track_id
      behavior_start
      velocity commands during behavior
      behavior_complete
      GoHome / Land / Complete
```
