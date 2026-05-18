# Dedalus Handoff Template

This file is a template for generating LLM handoff prompts.

To generate a current handoff, read `LLM.md` and the current repo state, then fill in the template below.

**Usage trigger:** `use HANDOFF.md to generate a handoff prompt from our current state`

---

## How to Generate a Handoff

1. Read `LLM.md` first. Treat it as the active operating brief.
2. Read `docs/mission_scenario_runner.md` for the current scenario/campaign harness workflow.
3. Read `docs/object_conditioned_behavior_plan.md` before Milestone 2.23 / M3 behavior work.
4. Read `LLM.back.md` only for historical context when needed.
5. Run `git log --oneline -1` to get the current commit SHA.
6. Substitute all `<PLACEHOLDER>` values below with current state.
7. Emit the filled-in prompt as plain text — no surrounding explanation.

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

  For normal patches:

    cat > /tmp/change.patch <<'PATCH'
    diff --git ...
    PATCH
    git apply /tmp/change.patch

  For full-file replacements:

    cat > /tmp/update_file.sh <<'SH'
    #!/usr/bin/env bash
    set -euo pipefail
    cat > path/to/file <<'EOF'
    ... complete file content ...
    EOF
    SH
    bash /tmp/update_file.sh

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

This reference is valid after the Milestone 2.22 closeout and before Milestone 2.23 starts.
When generating a new handoff, update the commit SHA and any current runtime observations.

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
  Milestone 2.23 — Behavior spec parser foundation for M3 object-conditioned behavior

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
      -> detector / tracker / projector
      -> WorldSnapshot agents with class, confidence, local position, velocity
      -> TargetSelector
      -> BehaviorRuntime / ObjectBehaviorMissionController
      -> desired velocity vector
      -> Px4BridgeCommandSink
      -> PX4 / AirSim

Current observed behavior:
  Milestone 2.20, 2.21, and 2.22 are implemented / validated.

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
    - synthetic_ci campaign mixes Complete and expected-Abort scenarios
    - airsim_live_smoke campaign is the live EC2/AirSim/PX4 preset
    - campaign reports write JSON, text, and Markdown summaries
    - Ctrl-C during a campaign gracefully finishes the active mission and then stops the campaign

Current diagnosis:
  The drone can fly and validate reliable preconfigured missions and scenario/campaign artifacts. The gap to Milestone 3 is object-conditioned behavior: parse behavior specs, select detected/tracked objects from WorldSnapshot, and run follow/circle/approach/sequence behaviors that emit bounded velocity vectors into the existing PX4 bridge path.

Immediate tasks:
  1. Build/test current head:

       cmake --build build-staging -j$(nproc)
       ctest --test-dir build-staging --output-on-failure

  2. Start Milestone 2.23 behavior spec parser foundation:
       - add a small behavior spec data model for target selector + behavior + completion/fallback
       - add parser support for YAML/JSON behavior specs without binding it to live flight yet
       - add sample specs for follow, circle, approach, and sequence
       - add unit tests for valid specs, defaults, and invalid combinations
       - preserve event extension points already expected by the validator:
           target_selected
           target_lost
           behavior_start
           behavior_complete
           behavior_failed
           fallback_start

  3. Preserve the Milestone 3 roadmap:
       - behavior spec parser
       - target selector from WorldSnapshot agents
       - ObjectBehaviorMissionController
       - follow behavior
       - circle behavior
       - approach + sequence behavior
       - M3 object-conditioned demo hardening

Do not:
  - Do not use dedalus_replay_mission. Use dedalus_mission_loop.
  - Do not treat command helper OK as vehicle-state truth.
  - Do not hide arming inside velocity commands.
  - Do not collapse flight_control.arm_state and ego.armed.
  - Do not move to ExecuteMission until Takeoff is confirmed by ego height.
  - Do not make the native C++ MAVLink sink the default live path; use px4_bridge.
  - Do not rewrite the working pymavlink control path in C++ while stabilizing behavior.
  - Do not let human diagnostics contaminate binary bridge stdout; binary frame bridge stdout is protocol bytes only.
  - Do not make mission_campaign_runner CTest execute repeated real missions; keep campaign CTest dry-run and leave real lifecycle coverage to scenario/abort tests.
  - Do not put obstacle avoidance inside the flight sink.
  - Do not let route memory override fresh tactical sensing.
  - Do not let Milestone 3 balloon into full obstacle avoidance; M3 is object-conditioned behavior. Avoidance starts post-M3.
  - Do not create branches or PRs unless explicitly requested.

Patch policy:
  Apply changes directly to main.
  Do not create branches or PRs unless explicitly requested.
  If GitHub connector patching fails or is ambiguous, provide an exact manual patch.

Validation:
  Always give build/test commands after code patches:

    cmake --build build-staging -j$(nproc)
    ctest --test-dir build-staging --output-on-failure

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
  For 2.23:
    Behavior specs parse from samples and unit tests prove defaults/errors.
    No live flight behavior changes are required yet.

  For later M3:
    mission_events + snapshots should prove:
      target_selected
      behavior_start
      velocity commands during behavior
      behavior_complete
      GoHome / Land / Complete
```
