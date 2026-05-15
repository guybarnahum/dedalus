# Dedalus Handoff Template

This file is a template for generating LLM handoff prompts.

To generate a current handoff, read `LLM.md` and the current repo state, then fill in the template below.

**Usage trigger:** `use HANDOFF.md to generate a handoff prompt from our current state`

---

## How to Generate a Handoff

1. Read `LLM.md` (active operating brief).
2. Run `git log --oneline -1` to get the current commit SHA.
3. Substitute all `<PLACEHOLDER>` values below with current state.
4. Emit the filled-in prompt as plain text — no surrounding explanation.

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
  <NUMBERED_TASK_LIST — copy from LLM.md §3.3 or update to reflect current state>

Do not:
  <DO_NOT_LIST — copy from LLM.md §10 and add any session-specific traps>

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

Expected success:
  <SPECIFIC_LOG_STATE_OR_TEST_RESULT_THAT_SIGNALS_TASK_COMPLETE>
```

---

## Reference — Last Known Good Handoff

The handoff below was valid as of commit `76ff9cc13f8d7a6227b972346e8c06151bbfbb13`.
Update it when generating a new one.

```
You are continuing work on the Dedalus repo.

Repository:
  guybarnahum/dedalus

Current commit:
  76ff9cc13f8d7a6227b972346e8c06151bbfbb13

First read:
  LLM.md

Historical context, only if needed:
  LLM.back.md

Active milestone:
  Milestone 2.19 — Mission / behavior / flight-control pipeline

Current architecture:
  AirSim live frame + ego sidecar
    -> AirSimFrameSource
    -> FrameHintEgoProvider
    -> CoreStackRunner
    -> InMemoryWorldModel
    -> LatestWorldSnapshot
    -> MissionRuntime async loop
    -> TrajectoryMissionController
    -> AirSimVelocityCommandSink
    -> simulation/airsim-send-velocity.py
    -> PX4 shell for Arm/Disarm
    -> AirSim moveByVelocityAsync for Velocity

Current observed behavior:
  The mission loop dispatches Arm successfully:

    OK command=arm dispatch=px4_shell target=dedalus-sim:px4

  It should record command intent:

    flight_control.arm_state = arm_requested

  But it remains in Prepare:

    status=waiting_for_armed_telemetry

Current diagnosis:
  The mission loop is structurally correct. It should not transition to Takeoff from
  synchronous dispatch OK. The missing piece is reliable async armed telemetry in
  WorldSnapshot.ego. AirSim getMultirotorState() may not expose armed state for this
  PX4-backed Colosseum setup. The next likely fix is MAVLink heartbeat armed-state parsing:

    armed = bool(heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
    armed_valid = true

Immediate tasks:
  1. Build/test current head:

       cmake --build build-staging -j$(nproc)
       ctest --test-dir build-staging --output-on-failure

  2. Fix any compile/test failures from the new WorldSnapshot.flight_control overlay.

  3. Patch apps/dedalus_mission_loop.cpp so debug snapshot artifacts are written from:

       latest_snapshot->latest().value_or(runner.snapshot())

     rather than runner.snapshot(), so flight_control.arm_state is visible.

  4. Add tests for LatestWorldSnapshot:
       - mark_command_dispatched(Arm) -> arm_requested
       - publishing fresh ego snapshot with armed_valid=false preserves arm_requested
       - publishing fresh ego snapshot with armed_valid=true, armed=true -> armed_confirmed
       - mark_command_dispatched(Disarm) -> disarm_requested
       - publishing fresh ego snapshot with armed_valid=true, armed=false -> disarmed_confirmed

  5. Probe MAVLink heartbeat endpoints:
       udpin:127.0.0.1:14550
       udpin:127.0.0.1:14540
       udpin:127.0.0.1:14600

  6. If AirSim armed telemetry is invalid, add MAVLink heartbeat armed-state into
     simulation/airsim-stream-frames-binary.py and emit armed/armed_valid in the ego sidecar.

Do not:
  - Do not use sync command OK as mission transition truth.
  - Do not hide arm/disarm inside velocity commands.
  - Do not collapse flight_control.arm_state and ego.armed.
  - Do not use the old dedalus_replay_mission name.
  - Do not move to ExecuteMission until Takeoff is confirmed by ego height.
  - Do not let old Milestone 2 frame-ingestion history distract from the current 2.19 mission task.

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

Expected success:
  Mission log should eventually show:

    state=Prepare status=arming command=yes
    flight_control.arm_state=arm_requested in snapshot artifacts
    ego.armed_valid=true ego.armed=true from telemetry
    state=Takeoff status=armed_confirmed_by_ego
    state=Takeoff status=takeoff_climb command=yes
```
