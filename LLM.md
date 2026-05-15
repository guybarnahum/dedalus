# Dedalus LLM Operating Brief

This file is the active orientation document for a new LLM session. Keep it short, current, and action-oriented.

Historical notes, superseded debugging context, and older milestone details live in `LLM.back.md`.

Repository:

```text
guybarnahum/dedalus
```

Current code baseline for this handoff:

```text
31331bae938e23b3adfa041e9d52a9c47cd3db82
```

Active milestone state:

```text
Milestone 2.19 — Mission / behavior / flight-control pipeline
Status: working live AirSim/PX4 trajectory mission using the persistent px4_bridge path.
```

Current working result:

```text
- `simulation/test-flight.py --trajectory trajectories/circle_figure8.json` works well.
- `dedalus_mission_loop` now also flies through the mission path using `flight_command_sink: px4_bridge`.
- Build succeeds.
- CTest passes: 18/18.
- Live mission reaches safe height through pymavlink OFFBOARD control, executes the trajectory, then lands/disarms through PX4 shell lifecycle commands.
```

Current remaining cleanup:

```text
Verbosity level 0 is still too noisy because subprocesses print directly:
- `airsim-prepare-session.py` prints AirSim connection text.
- `px4-command-bridge.py` prints safe-height progress samples.
- telemetry sidecar may still print `14600` bind noise if configured.

A manual patch was prepared to quiet subprocesses by default, pass verbosity into the Python PX4 bridge, and remove the unused `14600` telemetry endpoint. It has not been confirmed as landed on main.
```

Core rule:

```text
Synchronous command dispatch success is not vehicle-state truth.
Mission transitions must be driven by world-model telemetry.
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

Mission lifecycle may look at command intent and telemetry truth, but it must transition to flight states only from telemetry truth.

Do not collapse these layers.

---

## 5. Current Milestone Journey — Milestone 2

| Stage | Name | Status | Notes |
|---|---|---:|---|
| 2.1–2.18 | Frame/source/provider/bridge foundation | Done | Synthetic, recorded, AirSim, binary bridge, timing, annotation |
| 2.19A | WorldModel ego-state foundation | Done | Ego pose, height, armed fields |
| 2.19B | Mission config contract | Done | `mission_controller`, `mission_tick_hz`, `flight_command_sink`, `mission_options` |
| 2.19C | `TrajectoryMissionController` | Working | Prepare/Takeoff/ExecuteMission/GoHome/Land/Complete |
| 2.19D | Flight command sink | Working | `px4_bridge` is the working live path |
| 2.19E | Async runtime wiring | Done | `LatestWorldSnapshot` + `MissionRuntime` |
| 2.19F | Integration harness | Done | `dedalus_mission_loop` |
| 2.19G | Debug and naming cleanup | Done | Renamed app; verbosity now partly implemented |
| 2.19H | Explicit command kinds | Done | `Arm`, `Takeoff`, `Velocity`, `Land`, `Disarm` |
| 2.19I | Async command outcome semantics | Done | Telemetry truth drives transitions |
| 2.19J | Flight-control intent overlay | Done | Command intent stored in `WorldSnapshot.flight_control` |
| 2.19K | Reliable armed telemetry | Done enough for live mission | MAVLink heartbeat/ego sidecar path confirms arm/disarm |
| 2.19L | PX4 bridge mission parity | Working | Mission path now follows `test-flight.py` control sequence |
| 2.19M | Quiet/verbose logs | In progress | CLI flags exist; subprocess output still needs final cleanup |

---

## 6. Next Stage

Recommended next active stage:

```text
Milestone 2.20 — Mission robustness, observability, and cleanup
```

Immediate next tasks, in order:

```text
1. Land the quiet-verbosity cleanup if not already applied:
   - silence prepare-session output at verbosity 0
   - pass verbosity into `px4-command-bridge.py`
   - make safe-height progress level-3 only
   - remove `14600` from default telemetry sidecar config if it keeps binding noisily

2. Re-run:
   cmake --build build-staging -j$(nproc)
   ctest --test-dir build-staging --output-on-failure
   live mission with verbosity 0 and --verbose

3. Create a small `docs/mission_pipeline_current_state.md`:
   - current architecture
   - control split
   - run commands
   - known traps
   - log interpretation

4. Make mission artifacts more useful:
   - include mission state / command status in manifest or a sidecar JSONL
   - preserve final mission summary
   - keep snapshots as debug artifacts, not replay input

5. Improve graceful completion:
   - avoid repeated land commands while landing is already in progress
   - record explicit landing/disarm timeout reasons
   - make abort behavior safe and visible

6. Refactor only after stability:
   - factor shared control helpers between `test-flight.py` and `px4-command-bridge.py`
   - do not rewrite the working control path prematurely
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

Verbosity contract:

```text
default: high-level mission state transitions + final summary
-v: lifecycle/prep details
-vv: command summaries + sampled world snapshots
-vvv / --verbose: full detailed tick/sink/bridge tracing
```

---

## 8. Known Traps

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
- Do not refactor `test-flight.py` / `px4-command-bridge.py` until the mission path remains stable across repeated runs.
```

---

## 9. Handoff Prompt Format

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

## 10. Pointers

Detailed / historical context:

```text
LLM.back.md
docs/core_stack_current_state.md
docs/bridge_transport_plugins.md
docs/binary_frame_bridge_protocol.md
docs/perception_stabilization_annotation.md
```

Future recommended doc:

```text
docs/mission_pipeline_current_state.md
```