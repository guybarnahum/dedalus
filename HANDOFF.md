# Dedalus Handoff Template

This file is a template for generating LLM handoff prompts.

To generate a current handoff, read `LLM.md` and the current repo state, then fill in the template below.

**Usage trigger:** `use HANDOFF.md to generate a handoff prompt from our current state`

---

## How to Generate a Handoff

1. Read `LLM.md` first. Treat it as the active operating brief.
2. Read `LLM.back.md` only for historical context when needed.
3. Run `git log --oneline -1` to get the current commit SHA.
4. Substitute all `<PLACEHOLDER>` values below with current state.
5. Emit the filled-in prompt as plain text — no surrounding explanation.

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
  Prefer GitHub connector patches. If connector patching fails or is ambiguous, provide an exact manual patch.

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

  For live mission changes, also include:

    RUNS=3 simulation/repeat-mission-smoke.sh

Expected success:
  <SPECIFIC_LOG_STATE_OR_TEST_RESULT_THAT_SIGNALS_TASK_COMPLETE>
```

---

## Reference — Current Strategic Handoff Shape

This reference is valid after the Milestone 2.20 closeout and the Milestone 3 roadmap redefinition.
When generating a new handoff, update the commit SHA and any current runtime observations.

```
You are continuing work on the Dedalus repo.

Repository:
  guybarnahum/dedalus

Current commit:
  <CURRENT_COMMIT_SHA>

First read:
  LLM.md

Historical context, only if needed:
  LLM.back.md

Active milestone:
  Milestone 2.21 — Mission artifact validation and replay-grade diagnostics

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
  Milestone 2.20 is closed / validated.

  The live AirSim/PX4 mission loop works through flight_command_sink=px4_bridge:
    - takeoff reaches safe height
    - trajectory executes
    - GoHome / Land / Disarm complete
    - repeated mission runs work without restarting AirSim
    - mission_events.jsonl and summary helper validate successful runs

Current diagnosis:
  The drone can now fly a reliable preconfigured mission. The gap to Milestone 3 is object-conditioned behavior: selecting a detected/tracked class instance from WorldSnapshot and using follow, circle, approach, or sequence behaviors to emit bounded velocity vectors into the existing PX4 bridge path.

Immediate tasks:
  1. Build/test current head:

       cmake --build build-staging -j$(nproc)
       ctest --test-dir build-staging --output-on-failure

  2. Implement Milestone 2.21 artifact validation:
       - validate mission_events.jsonl + snapshots as a formal run artifact directory
       - validate state ordering: Prepare -> Takeoff -> ExecuteMission -> GoHome -> Land -> Complete
       - validate height gates: safe height before ExecuteMission, landed height before Complete
       - validate command exceptions/failures are absent for successful runs
       - leave extension points for future object-behavior events

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
  - Do not put obstacle avoidance inside the flight sink.
  - Do not let route memory override fresh tactical sensing.
  - Do not let Milestone 3 balloon into full obstacle avoidance; M3 is object-conditioned behavior. Avoidance starts post-M3.

Patch policy:
  Prefer GitHub connector patches. If connector patching fails or is ambiguous, provide an exact manual patch.

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

  For mission-loop changes:

    RUNS=3 simulation/repeat-mission-smoke.sh

Expected success:
  For 2.21:
    A new validator can inspect a live run output directory and confirm:
      final_state=Complete
      failures=0
      safe height was reached before ExecuteMission
      landed height was reached before Complete
      command/state ordering is valid

  For later M3:
    mission_events + snapshots should prove:
      target_selected
      behavior_start
      velocity commands during behavior
      behavior_complete
      GoHome / Land / Complete
```
