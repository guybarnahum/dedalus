# Dedalus LLM Historical Notes

This file is the archive for older LLM handoff material, superseded debugging context, and milestone history.

`LLM.md` is the active operating brief. Read it first.

This archive exists so historical context is preserved without distracting new LLM sessions from the current active task.

---

## H1. Historical Milestone 2 Context

Milestone 2 began as video/simulation input work and gradually became the integration path for live mission behavior.

Earlier focus areas included:

```text
- Recorded-frame ingestion
- AirSim provider boundary
- AirSim export bridge
- Replay snapshot artifacts
- Live AirSim RGB ingestion
- Live AirSim ego-state ingestion
- Persistent AirSim frame streams
- Bridge transport abstraction
- Binary framed bridge protocol
- Simulation/run orchestration
- Dependency-free visual annotation output
- Frame-attached ego sidecar
```

Most of this work is now stable enough to serve as infrastructure for the mission pipeline.

---

## H2. AirSim Resolution / settings.json History

A prior investigation showed that passing `--settings` to the packaged Colosseum/AirSim environment was unreliable in practice. The effective workaround was to patch the canonical:

```text
simulation/settings.json
```

before starting AirSim.

Validated result:

```text
actual_resolution_counts: 640x360
```

This mattered because AirSim capture latency and payload size directly affected bridge profiling.

Stale trap:

```text
Do not assume ~/Documents/AirSim/settings.json is the active source for the mission pipeline.
Do not assume --settings is honored by the packaged environment.
```

---

## H3. Bridge Latency / 2.18 History

Profiling showed the dominant cost was AirSim image extraction/RPC via `simGetImages`, not stdout transfer.

Important observations:

```text
- Lowering resolution helped, but AirSim capture remained a major cost.
- stdout_write_ms was not the main bottleneck.
- Shared memory / VRAM handles are future work, not the immediate blocker.
- Async prefetching overlapped AirSim frame wait with processing and recovered useful throughput.
```

The final useful profiler distinction was:

```text
frame_source.next_frame_wait
  main-thread wait for the next prefetched frame

background_fetch_detail.*
  attribution-only timings for background frame read
```

This avoided double-counting overlapped background work.

---

## H4. Frame Annotation / Replay Artifact History

A dependency-light PPM annotation path was added so CI and local validation could write visual artifacts without OpenCV, FFmpeg, or GStreamer.

Current note:

```text
frame_annotator: ppm_sequence
```

is useful for debugging, but the active milestone is now mission / behavior / flight-control. Do not let old annotation details distract from the current mission loop.

---

## H5. Old App Naming

The mission-capable app was initially named:

```text
dedalus_replay_mission
```

That name was misleading because the app can consume live AirSim frames. It was renamed to:

```text
dedalus_mission_loop
```

Use the new name only.

---

## H6. 2.19 Mission Pipeline History

2.19 introduced the behavior / mission / flight-control spine.

Progression:

```text
2.19A — WorldModel ego-state foundation
2.19B — Mission config contract
2.19C — TrajectoryMissionController
2.19D — FlightCommandSink
2.19E — Async runtime wiring
2.19F — Integration harness
2.19G — Debug + naming cleanup
2.19H — Explicit command kinds
2.19I — Async command-outcome semantics
2.19J — Flight-control intent overlay
2.19K — Reliable armed telemetry
2.19L — PX4 bridge mission parity
2.19M — Quiet/verbose logs
```

Critical design correction:

```text
The synchronous command result is dispatch feedback only.
Vehicle state must be confirmed asynchronously from telemetry in WorldSnapshot.ego.
```

This correction happened after the system briefly used command-helper OK as if it confirmed Arm/Disarm. That was wrong and should not be reintroduced.

---

## H7. Superseded Flight Control Paths

The mission pipeline went through several control paths before the current working `px4_bridge` path.

### H7.1 AirSim velocity helper path

Earlier placeholder split:

```text
Arm / Disarm:
  dispatched through PX4 shell using tmux send-keys

Velocity:
  dispatched through AirSim moveByVelocityAsync via simulation/airsim-send-velocity.py
```

Observed behavior:

```text
- Arm could work.
- Takeoff/climb behavior was inconsistent or incomplete.
- Trajectory segments were not reliably executed by PX4 once PX4 owned control mode.
```

Conclusion:

```text
AirSim moveByVelocityAsync is not the correct live mission control path for the PX4-backed Colosseum setup.
```

### H7.2 Native C++ MAVLink sink path

A native C++ sink was introduced:

```text
src/behavior/px4_mavlink_command_sink.cpp
```

It attempted to encode MAVLink packets directly and send:

```text
COMMAND_LONG for lifecycle / mode commands
SET_POSITION_TARGET_LOCAL_NED for velocity
```

Meaningful failure:

```text
simulation/test-flight.py worked, but the native sink did not reliably climb/execute trajectory.
```

Root cause category:

```text
The working `pymavlink` path owns endpoint binding, heartbeat peer learning, MAVLink1/MAVLink2 parsing, target routing, mode mapping, ACK handling, and message encoding. Reimplementing that in C++ created avoidable failure modes.
```

Conclusion:

```text
Do not make the native C++ MAVLink sink the default live path. Keep it experimental/deprecated unless there is a deliberate future effort to replace pymavlink with tested C++ MAVLink support.
```

### H7.3 Persistent Python PX4 bridge path

The current working path uses:

```text
src/behavior/px4_bridge_command_sink.cpp
simulation/px4-command-bridge.py
```

This preserves the C++ mission state machine while delegating flight-critical PX4/OFFBOARD control to the same `pymavlink` behavior proven in:

```text
simulation/test-flight.py
```

Important bridge fixes that made it match `test-flight.py`:

```text
- open MAVLink lazily after shell arm/takeoff, not during bridge construction
- use PX4 shell for arm/takeoff/land/disarm
- use pymavlink OFFBOARD priming and PX4 mode set
- climb to safe height inside the bridge using LOCAL_POSITION_NED feedback
- use a dedicated command MAVLink endpoint, typically udpin:127.0.0.1:14550
- keep telemetry sidecar on separate endpoint(s)
```

---

## H8. Python Helpers vs Native C++ Rationale

AirSim has a native C++ client API. Python helpers are not used because AirSim is Python-only.

The important split is:

```text
AirSim RPC / simulator interaction != PX4 flight-control interaction
```

AirSim-side tasks are plausible native C++ migration targets:

```text
- frame streaming
- ego/state reads
- session prep / API control
- GPS/state checks
```

PX4/MAVLink mission control is different. The current working path depends on `pymavlink`, because it already handles the details that broke the native C++ experiment:

```text
- heartbeat and target system/component routing
- MAVLink1/MAVLink2 framing
- PX4 mode mapping
- COMMAND_ACK behavior
- OFFBOARD priming timing
- LOCAL_POSITION_NED feedback climb
- SET_POSITION_TARGET_LOCAL_NED velocity setpoints
- UDP endpoint ownership semantics
```

Decision:

```text
Keep the C++ autonomy core, world model, mission runtime, and mission state machine.
Keep the PX4/MAVLink/OFFBOARD bridge in Python until mission behavior is repeatedly stable.
If reducing helpers is desired, migrate AirSim frame/ego/session helpers to native C++ first.
Only later consider a native C++ PX4/MAVLink backend, and only with a real tested MAVLink library rather than ad-hoc packet encoding.
```

Bottom line:

```text
Python is not required for AirSim access. It is currently the stable choice for PX4/MAVLink mission control because `pymavlink` plus `simulation/test-flight.py` is the proven implementation.
```

---

## H9. Historical Debugging Rule

When a run shows:

```text
helper_output=OK
```

or:

```text
{"ok":true,"command":"..."}
```

that only means the helper/bridge dispatched something successfully.

It does not prove:

```text
vehicle armed
vehicle took off
vehicle moved
vehicle landed
vehicle disarmed
```

Always check world-model telemetry truth.

---

## H10. Verbosity Cleanup History

`dedalus_mission_loop` gained verbosity flags:

```text
-v
-vv
-vvv
--verbose
```

The intended contract is:

```text
default: high-level mission transitions and final summary
-v: lifecycle / prep details
-vv: command summaries and sampled snapshots
-vvv / --verbose: full tick/sink/bridge tracing
```

The main C++ runtime follows this contract. Some subprocess output may still leak at verbosity 0 if helpers print directly.

Known noisy sources historically included:

```text
- AirSim `confirmConnection()` output during prepare-session
- `px4-command-bridge.py` safe-height progress samples
- telemetry sidecar endpoint bind warnings, especially old 14600 fallback
```

---

## H11. Archive Policy

When `LLM.md` grows with stale history, move details here.

Keep `LLM.md` focused on:

```text
- architecture
- active handoff
- milestone status
- commands
- next tasks
- design invariants
- known traps
```

Keep `LLM.back.md` focused on:

```text
- why old decisions were made
- historical debugging context
- superseded approaches
- old logs summarized, not pasted in full
```

---

## H12. Milestone 2.20 Closeout History

Milestone 2.20 closed the mission robustness / observability loop.

Key outcomes:

```text
- `mission_events.jsonl` became the compact source artifact for mission behavior.
- `dedalus_mission_loop` prints a final summary derived from `mission_events.jsonl`.
- `simulation/mission-events-summary.py` can summarize and validate event artifacts after the run.
- `simulation/repeat-mission-smoke.sh` can run repeated live missions and validate each event artifact.
- Back-to-back mission-loop runs were validated without restarting AirSim.
```

Important reliability fixes:

```text
- The PX4 bridge shutdown path sends a short zero-velocity settle stream, closes MAVLink, and resets bridge state.
- `Px4BridgeCommandSink` waits for bridge shutdown cleanup before closing pipes/reaping the child.
- `dedalus_mission_loop` handles Ctrl-C/SIGTERM by requesting graceful mission finish on first interrupt.
- Abort is terminal/diagnostic and no longer emits velocity commands.
```

Important design compromise:

```text
mission_options.flight_arm_dispatch_fallback_s: 2.0
```

This exists because repeat runs showed stale armed telemetry even though PX4 shell arm/takeoff was healthy and `simulation/test-flight.py` succeeded. The fallback may move from `Prepare` to `Takeoff` after successful Arm dispatch and a short settle interval when armed telemetry is stale.

This is acceptable because:

```text
- it does not enter ExecuteMission from command OK
- ExecuteMission remains gated by ego height reaching safe height
- command intent and telemetry truth remain separate
```

Known small cleanup:

```text
If `apps/dedalus_mission_loop.cpp` still warns about ++ on volatile sig_atomic_t, replace the increment in handle_interrupt_signal with read/current plus assignment. This is warning-only; build and runtime behavior were otherwise validated.
```
