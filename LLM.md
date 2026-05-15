# Dedalus LLM Operating Brief

This file is the active orientation document for a new LLM session. Keep it short, current, and action-oriented.

Historical notes, superseded debugging context, and older milestone details live in `LLM.back.md`.

Repository:

```text
guybarnahum/dedalus
```

Baseline commit for this handoff:

```text
c777732edfbce53dee599722df3e575d6e5a8715
```

Previous implementation baseline before the doc split:

```text
06125a7e6cdb29efee67a0ff5a93529f486119ed
```

Current active milestone:

```text
Milestone 2.19 — Mission / behavior / flight-control pipeline
```

Current active problem:

```text
The mission loop dispatches Arm successfully and records command intent in the world-model handoff, but it still needs reliable asynchronous armed telemetry before transitioning from Prepare to Takeoff.
```

Core rule:

```text
Synchronous command dispatch success is not vehicle-state truth.
Mission transitions must be driven by world-model telemetry.
```

---

## 1. System Architecture

Dedalus is a live/simulated drone autonomy stack:

```text
sensors / simulation
  -> perception
  -> world model
  -> behavior / mission controller
  -> flight-command sink
  -> AirSim / PX4
```

Current live mission pipeline:

```text
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
```

Main current app:

```text
apps/dedalus_mission_loop.cpp
```

The old name `dedalus_replay_mission` is obsolete. Do not use it.

Snapshot artifacts are debug outputs showing what the world model / mission handoff saw. They are not necessarily replay inputs.

---

## 2. Separation of Concerns

Keep these three layers separate.

### 2.1 Command intent

Command intent records what the mission runtime requested or dispatched.

Current world-model overlay:

```cpp
enum class FlightControlArmState {
    Unknown,
    Disarmed,
    ArmRequested,
    ArmedConfirmed,
    DisarmRequested,
    DisarmedConfirmed,
    ArmFailed,
    DisarmFailed,
};

struct FlightControlState {
    FlightControlArmState arm_state;
    TimePoint updated_at;
    TimePoint last_arm_request_at;
    TimePoint last_disarm_request_at;
    std::string status;
};

struct WorldSnapshot {
    TimePoint timestamp;
    EgoState ego;
    FlightControlState flight_control;
    ...
};
```

Examples:

```text
flight_control.arm_state = arm_requested
  The Arm command dispatch path returned OK.

flight_control.arm_state = arm_failed
  The Arm command dispatch path failed.

flight_control.arm_state = armed_confirmed
  Ego telemetry later confirmed the drone is armed.
```

### 2.2 Telemetry truth

Telemetry truth comes from the vehicle/sim state represented in `WorldSnapshot.ego`.

Examples:

```text
ego.armed_valid && ego.armed
ego.armed_valid && !ego.armed
ego.height_valid && ego.height_m >= safe_height
ego.flight_status
```

### 2.3 Mission lifecycle

The mission state machine owns mission phase:

```text
Prepare
Takeoff
ExecuteMission
GoHome
Land
Complete
Abort
```

Mission lifecycle may look at command intent and telemetry truth, but it must transition to flight states only from telemetry truth.

Do not collapse these layers.

---

## 3. Active Handoff

### 3.1 Current observed live behavior

The latest mission run showed:

```text
dedalus_mission: tick=1 state=Prepare status=arming ... command=yes
dedalus_mission: send_command kind=Arm ...
dedalus_flight_sink: helper_output=OK command=arm dispatch=px4_shell target=dedalus-sim:px4
dedalus_mission: command_result kind=Arm success=true ...
dedalus_mission: tick=2 state=Prepare status=waiting_for_armed_telemetry ...
```

Interpretation:

```text
The mission loop is alive.
The Arm intent dispatch path works.
The mission correctly does not transition to Takeoff based only on sync OK.
The missing piece is reliable armed telemetry in WorldSnapshot.ego.
```

### 3.2 Current likely gap

`simulation/airsim-stream-frames-binary.py` was patched to include:

```json
{
  "armed": true,
  "armed_valid": true
}
```

and `src/simulation/airsim_providers.cpp` was patched to parse that into:

```cpp
WorldSnapshot.ego.armed
WorldSnapshot.ego.armed_valid
```

However, AirSim `getMultirotorState()` may not expose a valid armed flag for this PX4-backed Colosseum setup.

If the mission stays in:

```text
status=waiting_for_armed_telemetry
```

then the next patch should read PX4 armed state from MAVLink heartbeat:

```text
armed = bool(heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
armed_valid = true
```

and emit that into the AirSim binary ego sidecar.

### 3.3 Immediate next steps

Do these in order:

```text
1. Build/test current head.
2. Fix any compile/test failures from the new WorldSnapshot.flight_control overlay.
3. Change apps/dedalus_mission_loop.cpp to write snapshot artifacts from latest_snapshot->latest() instead of runner.snapshot().
4. Add tests for LatestWorldSnapshot flight_control overlay behavior.
5. Probe PX4 MAVLink heartbeat armed-state locally.
6. If AirSim state does not expose armed_valid, add MAVLink heartbeat armed telemetry to simulation/airsim-stream-frames-binary.py.
7. Verify Prepare -> Takeoff transition is driven by ego.armed_valid && ego.armed.
8. After arm telemetry works, evaluate replacing AirSim moveByVelocityAsync with PX4-native takeoff/offboard velocity control.
```

### 3.4 Important next patch detail

At the current baseline, `MissionRuntime` updates the `LatestWorldSnapshot` handoff with `flight_control.arm_state`.

But `apps/dedalus_mission_loop.cpp` may still write debug artifacts from:

```cpp
const auto snapshot = runner.snapshot();
```

That direct world-model snapshot may not include the mission/runtime `flight_control` overlay.

Prefer this in `dedalus_mission_loop`:

```cpp
const auto latest = latest_snapshot->latest();
const auto snapshot = latest.has_value() ? *latest : runner.snapshot();
```

This keeps debug artifacts aligned with what the mission runtime is actually reading.

---

## 4. Current Milestone Journey — Milestone 2

Milestone 2 is the active milestone. It started as simulation/video/world-model ingestion and is now in the behavior/mission/flight-control slice.

| Stage | Name | Status | Notes |
|---|---|---:|---|
| 2.1–2.13 | Frame/source/provider foundation | Done | Synthetic, recorded, AirSim provider boundary, replay snapshots, binary bridge |
| 2.14 | AirSim co-stream ego optimization | Done | Frame-attached ego sidecar via `stream_binary_ego` |
| 2.15–2.16 | Annotation / artifact validation | Done | Dependency-light PPM annotation and replay artifacts |
| 2.17 | AirSim camera resolution control | Done | Canonical `simulation/settings.json` patching validated; `--settings` behavior was unreliable |
| 2.18A–E | Bridge latency and async frame read | Done | Async prefetch and explicit timing accounting |
| 2.19A | WorldModel ego-state foundation | Done | Ego pose, height, armed fields added/used |
| 2.19B | Mission config contract | Done | `mission_controller`, `mission_tick_hz`, `flight_command_sink`, `mission_options` |
| 2.19C | `TrajectoryMissionController` | Done, evolving | Placeholder lifecycle state machine |
| 2.19D | Flight command sink | Done, placeholder | AirSim/PX4 helper path exists |
| 2.19E | Async runtime wiring | Done | `LatestWorldSnapshot` + `MissionRuntime` |
| 2.19F | Integration harness | Done | `dedalus_mission_loop` |
| 2.19G | Debug and naming cleanup | Done | Mission logs and renamed app |
| 2.19H | Explicit command kinds | Done | `Arm`, `Velocity`, `Disarm` |
| 2.19I | Async command outcome semantics | Done conceptually | Telemetry truth drives transitions |
| 2.19J | Flight-control intent overlay | In progress | `WorldSnapshot.flight_control` added; needs build/test and artifact validation |
| 2.19K | Reliable armed telemetry | Next | Likely MAVLink heartbeat, not AirSim state |

---

## 5. Overall Roadmap

| Milestone | Name | Status | Notes |
|---|---|---:|---|
| 1 | Core contracts and dependency-free stack | Done | Provider contracts, world model, CI tests |
| 2 | Simulation/video/world-model/mission pipeline | Active | Currently in 2.19 behavior/flight-control |
| 3 | PX4-native command/control path | Planned | Arm, takeoff, offboard velocity setpoints, land/disarm |
| 4 | Mission lifecycle robustness | Planned | Retry policy, abort reasons, command throttling, safety envelope |
| 5 | World-model-driven behavior | Planned | React to agents, obstacles, risk, priority, mission goals |
| 6 | Validation and reproducibility | Planned | Integration profiles, logs, artifacts, replayable mission traces |
| 7 | Productization / field readiness | Future | Deployment, observability, config hardening |

---

## 6. Commands

### 6.1 Build/test

```bash
cd ~/dedalus
source venv/bin/activate

cmake --build build-staging -j$(nproc)
ctest --test-dir build-staging --output-on-failure
```

### 6.2 Start AirSim

```bash
cd ~/dedalus/simulation
./stop.sh
./run.sh AirSimNH --airsim-camera-width 640 --airsim-camera-height 360
```

### 6.3 Run live mission loop

```bash
cd ~/dedalus
source venv/bin/activate

./build-staging/apps/dedalus_mission_loop \
  --config config/core_stack_trajectory_mission_placeholder.yaml \
  --output-dir out/airsim_mission_snapshots \
  --max-frames 300 \
  --progress 2>&1 | tee out/airsim_mission_debug.log
```

### 6.4 Probe MAVLink heartbeat armed state

```bash
cd ~/dedalus
source venv/bin/activate

python3 - <<'PY'
from pymavlink import mavutil

for endpoint in ["udpin:127.0.0.1:14550", "udpin:127.0.0.1:14540", "udpin:127.0.0.1:14600"]:
    print("trying", endpoint)
    try:
        mav = mavutil.mavlink_connection(endpoint, autoreconnect=True, source_system=255)
        hb = mav.wait_heartbeat(timeout=3)
        if hb:
            armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            print("OK", endpoint, "base_mode=", hb.base_mode, "armed=", armed)
            break
    except Exception as e:
        print("failed", endpoint, e)
PY
```

Then arm manually:

```bash
tmux send-keys -t dedalus-sim:px4 "commander arm" C-m
```

Run the heartbeat probe again and confirm whether `armed=True`.

---

## 7. Patch and Commit Rules for LLM Workers

### 7.1 Preferred patch path

Preferred order:

```text
1. Use the GitHub connector to patch files directly when possible.
2. If connector patching fails or is likely to be ambiguous, provide a manual patch.
3. Manual patches must be exact and directly applicable.
```

### 7.2 Connector patch rules

When using the GitHub connector:

```text
- Fetch the current file first.
- Patch only the intended files.
- Use small commits with descriptive messages.
- Do not combine unrelated code and docs changes unless explicitly requested.
- If a connector write fails, explain the failure and provide a manual patch instead of pretending the patch landed.
```

Commit message format:

```text
feat|fix|test|docs|chore(scope): short title

Optional body:
- What changed
- Why
- Validation / expected test
```

Example:

```text
fix(behavior): gate takeoff on ego armed telemetry

Use WorldSnapshot.ego.armed_valid/armed as the transition authority
for Prepare -> Takeoff. Keep command helper OK as dispatch feedback only.
```

### 7.3 Manual patch format

When direct connector patching is not possible, provide one of these exact formats.

Preferred for normal code patches:

```bash
cat > /tmp/change.patch <<'PATCH'
diff --git a/path/file.cpp b/path/file.cpp
index ...
--- a/path/file.cpp
+++ b/path/file.cpp
@@ -10,7 +10,7 @@
 old line
 new line
PATCH

git apply /tmp/change.patch
```

Preferred for full-file replacements or doc rewrites:

```bash
cat > /tmp/update_docs.sh <<'SH'
#!/usr/bin/env bash
set -euo pipefail

cat > LLM.md <<'EOF'
... complete file content ...
EOF
SH

bash /tmp/update_docs.sh
```

For full-file replacements, use heredoc scripts instead of fake partial diffs. This avoids corrupted patches when the old file is too large or unavailable.

Always include validation commands after a manual patch:

```bash
cmake --build build-staging -j$(nproc)
ctest --test-dir build-staging --output-on-failure
```

If the change is docs-only, say so explicitly.

---

## 8. Handoff Prompt Format

Every new worker handoff should include:

```text
1. Repo and current commit.
2. Active milestone and exact stage.
3. Current architecture summary.
4. Current observed behavior / logs.
5. Current diagnosis.
6. Immediate next tasks in order.
7. Explicit non-goals / traps.
8. Build/test/run commands.
9. Expected success signal.
```

Template:

```text
You are continuing work on the Dedalus repo.

Current commit:
  <sha>

Active milestone:
  <milestone and stage>

Current architecture:
  <short data-flow diagram>

Current observed behavior:
  <important logs / summary>

Current diagnosis:
  <what is likely missing or broken>

Immediate next tasks:
  1. ...
  2. ...
  3. ...

Do not:
  - ...
  - ...

Validation:
  <build/test commands>
  <runtime command>

Expected success:
  <specific log/state/test result>
```

---

## 9. LLM.md / LLM.back.md Maintenance Rules

`LLM.md` must stay tight.

Rules:

```text
- Keep LLM.md as the active operating brief.
- Keep current state near the top.
- Do not paste long logs into LLM.md.
- Do not keep superseded debugging narratives in LLM.md.
- Move old context to LLM.back.md.
- Prefer tables for milestone status.
- Update the current commit and active handoff after significant milestone changes.
- Keep exact commands current.
- Keep "Do not" traps explicit.
```

When updating `LLM.md`, update only:

```text
- current commit
- active handoff
- current milestone status
- immediate next steps
- known traps
- commands, if changed
```

When a topic becomes historical, move or summarize it in `LLM.back.md`.

---

## 10. Known Traps

```text
- Do not use dedalus_replay_mission. It was renamed to dedalus_mission_loop.
- Do not treat snapshot artifacts as proof of replay input.
- Do not treat command helper OK as vehicle-state truth.
- Do not hide arming inside velocity commands.
- Do not collapse flight_control.arm_state and ego.armed.
- Do not assume AirSim getMultirotorState().armed works for PX4-backed Colosseum.
- Do not move to ExecuteMission until Takeoff is confirmed by ego height.
- Do not prioritize shared_memory/VRAM frame transport unless timing shows transport dominates.
```

---

## 11. Pointers

Detailed / historical context:

```text
LLM.back.md
docs/core_stack_current_state.md
docs/shared_memory_frame_transport.md
docs/binary_frame_bridge_protocol.md
```

Future recommended doc:

```text
docs/mission_pipeline_current_state.md
```
