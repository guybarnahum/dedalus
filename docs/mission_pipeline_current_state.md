# Mission Pipeline Current State

This document describes the current Dedalus live AirSim/PX4 mission pipeline as of the Milestone 2.21 artifact-validation stage.

It is meant for developers and operators who need to run, debug, validate, or safely extend the mission loop.

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
  -> tools/px4/px4-command-bridge.py
  -> PX4 / AirSim
```

Known-good standalone reference:

```bash
python ./simulation/airsim/scripts/test-flight.py --trajectory trajectories/circle_figure8.json
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

The C++ mission runtime owns mission lifecycle state. The Python bridge owns the PX4/MAVLink transport behavior that has already been validated by `simulation/airsim/scripts/test-flight.py`.

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
RUNS=3 tools/mission/repeat-mission-smoke.sh
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
  --config config/core_stack_object_behavior_airsim_existing_object_circle.yml \
  --output-dir out/airsim_mission_snapshots_run1 \
  --max-frames 900 \
  --shutdown-max-frames 400 \
  --progress 2>&1 | tee out/airsim_mission_debug_run1.log

./build-staging/apps/dedalus_mission_loop \
  --config config/core_stack_object_behavior_airsim_existing_object_circle.yml \
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
tools/px4/px4-command-bridge.py
simulation/airsim/scripts/airsim-prepare-session.py
simulation/airsim/scripts/airsim-stream-frames-binary.py
tools/mission/mission-events-summary.py
tools/mission/validate-mission-artifacts.py
tools/mission/repeat-mission-smoke.sh
config/core_stack_object_behavior_airsim_existing_object_circle.yml
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
