# Dedalus LLM Operating Brief

This file is the active orientation document for a new LLM session. Keep it short, current, and action-oriented.

Historical notes, superseded debugging context, and older milestone details live in `LLM.back.md`.

Repository:

```text
guybarnahum/dedalus
```

Current code baseline for this handoff:

```text
main after Milestone 2.20E closeout
```

Active milestone state:

```text
Milestone 2.20 — Mission robustness, observability, and cleanup
Status: closed / validated.
```

Current working result:

```text
- `simulation/test-flight.py --trajectory trajectories/circle_figure8.json` works well.
- `dedalus_mission_loop` flies through the mission path using `flight_command_sink: px4_bridge`.
- Back-to-back mission-loop runs now work without restarting AirSim.
- Build succeeds.
- CTest expected result: 18/18 passing.
- Live mission reaches safe height through pymavlink OFFBOARD control, executes the trajectory, goes home, lands, and disarms through PX4 shell lifecycle commands.
- `mission_events.jsonl` is the source artifact for mission debugging and final summaries.
```

Core rule:

```text
Synchronous command dispatch success is not vehicle-state truth.
Mission transitions into flight execution must be driven by world-model telemetry.
```

---

## 1. Current Architecture

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
  -> Px4BridgeCommandSink
  -> persistent simulation/px4-command-bridge.py
       - PX4 shell: arm, takeoff, land, disarm
       - pymavlink: OFFBOARD mode + SET_POSITION_TARGET_LOCAL_NED velocity
       - LOCAL_POSITION_NED feedback climb to safe height
  -> PX4 / AirSim
```

Main current app:

```text
apps/dedalus_mission_loop.cpp
```

The old name `dedalus_replay_mission` is obsolete. Do not use it.

Snapshot artifacts are debug outputs showing what the world model / mission handoff saw. They are not necessarily replay inputs.

---

## 2. Control Path That Works

The working control split deliberately mirrors `simulation/test-flight.py`.

```text
Prepare:
  deterministic AirSim/PX4 session prep
  confirm AirSim connection, GPS validity, API control, and MAVLink reachability

Arm:
  PX4 shell: commander arm

Takeoff:
  PX4 shell: commander takeoff
  bridge waits for takeoff settle

First velocity / climb:
  lazy pymavlink connection after shell takeoff
  prime OFFBOARD velocity stream
  set PX4 OFFBOARD mode
  climb to safe height using LOCAL_POSITION_NED feedback

Trajectory:
  pymavlink SET_POSITION_TARGET_LOCAL_NED velocity setpoints
  trajectory loaded from simulation/trajectories/circle_figure8.json

Landing:
  PX4 shell: commander land
  wait for ego telemetry / landed height

Complete:
  PX4 shell: commander disarm
```

Key files:

```text
simulation/test-flight.py                    known-good standalone reference
simulation/px4-command-bridge.py             persistent bridge used by mission runtime
src/behavior/px4_bridge_command_sink.cpp      C++ JSONL process-backed command sink
src/behavior/trajectory_mission_controller.cpp
src/behavior/mission_runtime.cpp
apps/dedalus_mission_loop.cpp
config/core_stack_trajectory_mission_placeholder.yaml
```

Do not reintroduce the hand-written native C++ MAVLink encoder as the default live path. `src/behavior/px4_mavlink_command_sink.cpp` may remain as experimental/deprecated code, but the live mission should use `flight_command_sink: px4_bridge`.

---

## 3. Python Helpers vs Native C++ Decision

AirSim itself does have a native C++ client API. Python helpers are not used because C++ cannot talk to AirSim.

The current boundary decision is:

```text
- Keep the mission state machine, world model, runtime, and provider interfaces in C++.
- Keep PX4/MAVLink/OFFBOARD mission control in the Python `px4-command-bridge.py` for now because it uses the same `pymavlink` behavior proven by `simulation/test-flight.py`.
- Do not rewrite the working MAVLink control path in C++ while the mission is still being stabilized.
```

Important distinction:

```text
AirSim RPC control != PX4 control
```

AirSim C++ can eventually replace Python helpers for simulator-side concerns:

```text
- frame streaming
- ego/state reads
- session prep / API control
```

But PX4 trajectory control currently depends on the validated `pymavlink` path:

```text
- MAVLink heartbeat and target routing
- PX4 mode mapping and COMMAND_ACK handling
- OFFBOARD priming timing
- LOCAL_POSITION_NED feedback climb
- SET_POSITION_TARGET_LOCAL_NED velocity setpoints
```

Recommended migration order:

```text
1. Stabilize current `px4_bridge` mission path.
2. Migrate AirSim frame/ego/session helpers to native C++ if needed.
3. Only later consider a native C++ PX4/MAVLink backend using a real tested MAVLink library, not ad-hoc packet encoding.
```

Bottom line:

```text
Python is not required for AirSim access. It is currently required for the stable PX4/MAVLink mission-control path because `pymavlink` + `test-flight.py` is the proven implementation.
```

---

## 4. Separation of Concerns

Keep these three layers separate.

### 4.1 Command intent

Command intent records what the mission runtime requested or dispatched.

Examples:

```text
flight_control.arm_state = arm_requested
  The Arm command dispatch path returned OK.

flight_control.arm_state = arm_failed
  The Arm command dispatch path failed.

flight_control.arm_state = armed_confirmed
  Ego telemetry later confirmed the drone is armed.
```

### 4.2 Telemetry truth

Telemetry truth comes from the vehicle/sim state represented in `WorldSnapshot.ego`.

Examples:

```text
ego.armed_valid && ego.armed
ego.armed_valid && !ego.armed
ego.height_valid && ego.height_m >= safe_height
ego.flight_status
```

### 4.3 Mission lifecycle

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

Mission lifecycle may look at command intent and telemetry truth, but it must transition to flight execution only from telemetry truth.

Repeat runs exposed a stale armed-telemetry case even though shell arm/takeoff remained healthy. The live config therefore enables:

```yaml
mission_options.flight_arm_dispatch_fallback_s: 2.0
```

This fallback may advance from `Prepare` to `Takeoff` after successful Arm dispatch and a short settle interval when armed telemetry is stale. It does **not** move to `ExecuteMission`; `ExecuteMission` remains gated by ego height reaching safe height.

Do not collapse these layers.

---

## 5. Milestone Journey

| Stage | Name | Status | Notes |
|---|---|---:|---|
| 2.1–2.18 | Frame/source/provider/bridge foundation | Done | Synthetic, recorded, AirSim, binary bridge, timing, annotation |
| 2.19A–K | Mission/ego/control foundation | Done | Ego state, mission config, runtime, command intent, armed telemetry |
| 2.19L | PX4 bridge mission parity | Done | Mission path follows `test-flight.py` control sequence |
| 2.19M | Quiet/verbose logs | Done enough | CLI verbosity exists; default is high-level output plus final summary |
| 2.20A | Mission docs/current state | Done | `docs/mission_pipeline_current_state.md` |
| 2.20B | Mission event artifacts | Done | `mission_events.jsonl` from `MissionRuntime` |
| 2.20C | Final summary from events | Done | `dedalus_mission_loop` summarizes `mission_events.jsonl` |
| 2.20D | Repeatable-run hardening | Done / validated | Back-to-back mission runs work without restarting AirSim |
| 2.20E | Closeout tooling/checkpoint | Done | Event summary helper + repeat smoke wrapper + updated docs |

---

## 6. Mission Artifacts and Tools

Mission loop output directory contains:

```text
snapshot_XXXX.json
snapshot_manifest.txt
mission_events.jsonl
```

`mission_events.jsonl` is the compact structured timeline and should be the first artifact inspected for mission behavior.

Summary helper:

```bash
python3 simulation/mission-events-summary.py out/airsim_mission_snapshots/mission_events.jsonl
python3 simulation/mission-events-summary.py out/airsim_mission_snapshots/mission_events.jsonl --expect-complete
```

Repeat-run smoke helper:

```bash
RUNS=3 simulation/repeat-mission-smoke.sh
```

`repeat-mission-smoke.sh` assumes AirSim/PX4 is already running. It runs `dedalus_mission_loop` repeatedly and validates each produced `mission_events.jsonl` with `--expect-complete`.

If the script is not executable after GitHub checkout, run:

```bash
chmod +x simulation/repeat-mission-smoke.sh simulation/mission-events-summary.py
```

---

## 7. Commands

### 7.1 Build/test

```bash
cd ~/dedalus
source venv/bin/activate

cmake --build build-staging -j$(nproc)
ctest --test-dir build-staging --output-on-failure
```

Expected current result:

```text
100% tests passed, 0 tests failed out of 18
```

### 7.2 Standalone known-good test flight

```bash
cd ~/dedalus
source venv/bin/activate

python ./simulation/test-flight.py --trajectory trajectories/circle_figure8.json
```

### 7.3 Start AirSim

```bash
cd ~/dedalus/simulation
./stop.sh
./run.sh AirSimNH --airsim-camera-width 640 --airsim-camera-height 360
```

### 7.4 Run live mission loop

Quiet/default:

```bash
cd ~/dedalus
source venv/bin/activate

./build-staging/apps/dedalus_mission_loop \
  --config config/core_stack_trajectory_mission_placeholder.yaml \
  --output-dir out/airsim_mission_snapshots \
  --max-frames 900 \
  --shutdown-max-frames 400 \
  --progress 2>&1 | tee out/airsim_mission_debug.log
```

Full debug:

```bash
./build-staging/apps/dedalus_mission_loop \
  --config config/core_stack_trajectory_mission_placeholder.yaml \
  --output-dir out/airsim_mission_snapshots \
  --max-frames 900 \
  --shutdown-max-frames 400 \
  --progress \
  --verbose 2>&1 | tee out/airsim_mission_debug.log
```

Repeat validation:

```bash
RUNS=3 simulation/repeat-mission-smoke.sh
```

Verbosity contract:

```text
default: high-level mission state transitions + final summary
-v: lifecycle/prep details
-vv: command summaries + sampled world snapshots
-vvv / --verbose: full detailed tick/sink/bridge tracing
```

---

## 8. Ctrl-C / Shutdown Behavior

`dedalus_mission_loop` handles interrupts:

```text
First Ctrl-C / SIGTERM:
  requests graceful mission finish through MissionRuntime.

Second Ctrl-C:
  stops the local main loop after local cleanup paths run.
```

The Python PX4 bridge handles a JSONL `shutdown` command from the C++ sink. On shutdown, if MAVLink was active, it sends a short zero-velocity settle stream, closes the MAVLink socket, and resets internal OFFBOARD/safe-height state before process exit.

Abort is terminal/diagnostic and does not emit velocity commands.

---

## 9. Known Traps

```text
- Do not use dedalus_replay_mission. It was renamed to dedalus_mission_loop.
- Do not treat snapshot artifacts as proof of replay input.
- Do not treat command helper OK as vehicle-state truth.
- Do not hide arming inside velocity commands.
- Do not collapse flight_control.arm_state and ego.armed.
- Do not move to ExecuteMission until Takeoff is confirmed by ego height.
- Do not make the native C++ MAVLink sink the default live path; use px4_bridge.
- Do not debug the working mission by rewriting MAVLink packet encoding in C++.
- Do not let the telemetry sidecar and command bridge fight over the same MAVLink endpoint.
- Do not replace `px4-command-bridge.py` with native C++ until the mission is stable and a real tested MAVLink C++ backend is planned.
- Do not refactor `test-flight.py` / `px4-command-bridge.py` unless repeat-run smoke remains stable.
```

---

## 10. Recommended Next Stage

Recommended next active stage:

```text
Milestone 2.21 — Mission artifact validation and replay-grade diagnostics
```

Suggested first tasks:

```text
1. Turn mission_events + snapshots into a formal validator for live-run artifact directories.
2. Validate state ordering: Prepare -> Takeoff -> ExecuteMission -> GoHome -> Land -> Complete.
3. Validate height gates: safe height reached before ExecuteMission; landed height before Complete.
4. Validate final disarm requested/confirmed semantics.
5. Keep mission event validation separate from frame replay semantics.
```

---

## 11. Handoff Prompt Format

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

---

## 12. Pointers

Detailed / historical context:

```text
LLM.back.md
docs/core_stack_current_state.md
docs/bridge_transport_plugins.md
docs/binary_frame_bridge_protocol.md
docs/perception_stabilization_annotation.md
docs/mission_pipeline_current_state.md
```