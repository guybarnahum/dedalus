# Mission Pipeline Current State

This document describes the current Dedalus live AirSim/PX4 mission pipeline as of the Milestone 2.20 closeout.

It is meant for developers and operators who need to run, debug, or safely extend the mission loop.

## Status

The live mission path is working and repeatable through the persistent PX4 bridge. Back-to-back mission-loop runs now complete without restarting AirSim.

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
  -> simulation/px4-command-bridge.py
  -> PX4 / AirSim
```

Known-good standalone reference:

```bash
python ./simulation/test-flight.py --trajectory trajectories/circle_figure8.json
```

The mission-loop path deliberately mirrors that known-good control sequence.

## Control split

The current live config uses:

```yaml
flight_command_sink: px4_bridge
```

The control path is intentionally hybrid:

```text
PX4 shell:
  commander arm
  commander takeoff
  commander land
  commander disarm

pymavlink:
  wait heartbeat
  prime OFFBOARD velocity stream
  set PX4 OFFBOARD mode
  climb to safe height using LOCAL_POSITION_NED feedback
  send SET_POSITION_TARGET_LOCAL_NED velocity setpoints
```

The C++ mission runtime owns mission lifecycle state. The Python bridge owns the PX4/MAVLink transport behavior that has already been validated by `simulation/test-flight.py`.

Do not replace this with the native C++ MAVLink sink as the default live path. The native sink may remain in-tree as experimental/deprecated code.

## Command intent versus telemetry truth

The synchronous command result is dispatch feedback only. It is not vehicle-state truth.

Examples:

```text
{"ok":true,"command":"arm"}
```

means the bridge dispatched an arm request. It does not prove PX4 is armed.

Mission transitions must be driven by telemetry in `WorldSnapshot.ego`, for example:

```text
ego.armed_valid && ego.armed
ego.armed_valid && !ego.armed
ego.height_valid && ego.height_m >= safe_height
```

The separate `WorldSnapshot.flight_control` overlay records command intent and command dispatch/failure state.

### Arm dispatch fallback

Repeat runs exposed a case where armed telemetry could be stale even though PX4 shell arm/takeoff remained healthy. The live config therefore enables:

```yaml
mission_options.flight_arm_dispatch_fallback_s: 2.0
```

This fallback may advance from `Prepare` to `Takeoff` after a successful Arm dispatch and a short settle interval when armed telemetry is stale. It does **not** advance into `ExecuteMission` from command OK. `ExecuteMission` still requires ego height telemetry to confirm takeoff/safe-height completion.

## Runtime states

Current mission lifecycle:

```text
Prepare
Takeoff
ExecuteMission
GoHome
Land
Complete
Abort
```

High-level behavior:

```text
Prepare:
  request Arm until telemetry confirms armed, or until Arm dispatch fallback unblocks shell takeoff

Takeoff:
  request Takeoff
  request velocity climb after initial takeoff lift
  transition only when ego height reaches safe height

ExecuteMission:
  play configured velocity trajectory

GoHome:
  move laterally toward initial ego pose

Land:
  send one Land command
  wait for landed-height telemetry
  abort on land timeout

Complete:
  request Disarm until telemetry confirms disarmed or timeout

Abort:
  terminal diagnostic state; does not emit velocity commands
```

## Shutdown, Ctrl-C, and repeatable runs

The Python PX4 bridge handles a JSONL `shutdown` command from the C++ sink. On shutdown, if MAVLink was active, the bridge sends a short zero-velocity settle stream, closes the MAVLink socket, and resets its internal OFFBOARD/safe-height state before process exit.

`dedalus_mission_loop` also handles interrupts:

```text
First Ctrl-C / SIGTERM:
  request graceful mission finish through MissionRuntime

Second Ctrl-C:
  stop the local main loop after local cleanup paths run
```

This is intended to leave the drone in a good state when the operator interrupts a live run.

Repeatability validation procedure:

```bash
# Start AirSim once.
cd ~/dedalus/simulation
./stop.sh
./run.sh AirSimNH --airsim-camera-width 640 --airsim-camera-height 360

# Terminal 2: run several mission loops without restarting AirSim.
cd ~/dedalus
source venv/bin/activate
RUNS=3 simulation/repeat-mission-smoke.sh
```

Expected result for every run:

```text
Mission summary:
  final_state: Complete
  failures: 0
```

Manual two-run validation remains useful while debugging:

```bash
./build-staging/apps/dedalus_mission_loop \
  --config config/core_stack_trajectory_mission_placeholder.yaml \
  --output-dir out/airsim_mission_snapshots_run1 \
  --max-frames 900 \
  --shutdown-max-frames 400 \
  --progress 2>&1 | tee out/airsim_mission_debug_run1.log

./build-staging/apps/dedalus_mission_loop \
  --config config/core_stack_trajectory_mission_placeholder.yaml \
  --output-dir out/airsim_mission_snapshots_run2 \
  --max-frames 900 \
  --shutdown-max-frames 400 \
  --progress 2>&1 | tee out/airsim_mission_debug_run2.log
```

If a run fails, compare:

```bash
tail -n 80 out/airsim_mission_snapshots_run1/mission_events.jsonl
tail -n 80 out/airsim_mission_snapshots_run2/mission_events.jsonl
```

## Main files

```text
apps/dedalus_mission_loop.cpp
include/dedalus/behavior/mission_controller.hpp
include/dedalus/behavior/trajectory_mission_controller.hpp
include/dedalus/behavior/flight_command_sinks.hpp
src/behavior/mission_runtime.cpp
src/behavior/trajectory_mission_controller.cpp
src/behavior/px4_bridge_command_sink.cpp
simulation/px4-command-bridge.py
simulation/airsim-prepare-session.py
simulation/airsim-stream-frames-binary.py
simulation/mission-events-summary.py
simulation/repeat-mission-smoke.sh
config/core_stack_trajectory_mission_placeholder.yaml
```

## Build and test

```bash
cd ~/dedalus
source venv/bin/activate

cmake --build build-staging -j$(nproc)
ctest --test-dir build-staging --output-on-failure
```

Expected result:

```text
100% tests passed
```

## Start AirSim

```bash
cd ~/dedalus/simulation
./stop.sh
./run.sh AirSimNH --airsim-camera-width 640 --airsim-camera-height 360
```

## Run live mission

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

## Verbosity contract

```text
default:
  high-level mission transitions and final summary

-v:
  lifecycle/prep details

-vv:
  command summaries and sampled world snapshots

-vvv / --verbose:
  full tick, sink, bridge, and safe-height tracing
```

Subprocess output should be quiet at default verbosity. If noisy output leaks from a helper, route it through the verbosity contract rather than adding more unconditional prints.

## Expected success signal

A healthy live run should show this high-level shape:

```text
state=Prepare status=arming
state=Takeoff status=armed_confirmed_by_ego
state=Takeoff status=takeoff_request
state=ExecuteMission status=takeoff_complete
state=ExecuteMission status=trajectory_execute
state=GoHome status=trajectory_complete
state=Land status=home_reached
state=Land status=landing_command_sent
state=Complete status=landed
state=Complete status=complete
```

On repeat runs where armed telemetry is stale, this may include:

```text
state=Takeoff status=arm_dispatch_ok_waiting_for_takeoff_height
```

That is acceptable as long as `ExecuteMission` is still reached only after ego height confirms safe height.

The final console summary is generated from `mission_events.jsonl` and should look like:

```text
Mission summary:
  final_state: Complete
  ticks: 902
  events: 1234
  state_path: Idle -> Prepare -> Takeoff -> ExecuteMission -> GoHome -> Land -> Complete
  commands:
    Arm: ok=1 failed=0
    Takeoff: ok=1 failed=0
    Velocity: ok=900 failed=0
    Land: ok=1 failed=0
    Disarm: ok=1 failed=0
  failures: 0
```

With `--verbose`, bridge-level details should include:

```text
Trying MAVLink endpoint: udpin:127.0.0.1:14550
MAVLink heartbeat received
Priming PX4 Offboard velocity stream
Setting PX4 mode OFFBOARD
Climbing to safe height
Safe height reached
```

## Artifacts

The mission loop writes debug and event artifacts to the configured output directory.

Current artifacts:

```text
snapshot_XXXX.json
snapshot_manifest.txt
mission_events.jsonl
```

`snapshot_XXXX.json` files describe what the world model and mission handoff saw. They are debug artifacts and are not necessarily replay inputs.

`mission_events.jsonl` is the compact structured mission timeline. It is independent of console verbosity and is the source artifact used to generate the final console summary.

Representative events:

```json
{"event":"runtime_start","tick_hz":10.000000}
{"event":"state_transition","tick":1,"from":"Idle","to":"Prepare","status":"arming"}
{"event":"command_dispatch","tick":1,"state":"Prepare","command":"Arm"}
{"event":"command_result","tick":1,"state":"Prepare","command":"Arm","success":true}
{"event":"state_transition","tick":42,"from":"Takeoff","to":"ExecuteMission","status":"takeoff_complete"}
{"event":"state_transition","tick":901,"from":"Land","to":"Complete","status":"landed"}
{"event":"runtime_stop","tick_count":902,"state":"Complete"}
```

Quick inspection:

```bash
tail -n 40 out/airsim_mission_snapshots/mission_events.jsonl
python3 simulation/mission-events-summary.py out/airsim_mission_snapshots/mission_events.jsonl
python3 simulation/mission-events-summary.py out/airsim_mission_snapshots/mission_events.jsonl --expect-complete
```

## Known traps

```text
- Do not use dedalus_replay_mission; use dedalus_mission_loop.
- Do not treat command dispatch OK as vehicle-state truth.
- Do not hide arm/disarm inside velocity commands.
- Do not collapse flight_control.arm_state and ego.armed.
- Do not move to ExecuteMission until ego height confirms safe height.
- Do not make px4_mavlink the default live sink.
- Do not rewrite the working pymavlink control path in C++ while stabilizing the mission.
- Do not let telemetry sidecar and command bridge bind the same MAVLink endpoint.
- Do not refactor test-flight.py and px4-command-bridge.py until repeated mission runs are stable.
```

## Recommended next stage

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