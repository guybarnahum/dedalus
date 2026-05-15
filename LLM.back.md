
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
```

Critical design correction:

```text
The synchronous command result is dispatch feedback only.
Vehicle state must be confirmed asynchronously from telemetry in WorldSnapshot.ego.
```

This correction happened after the system briefly used command-helper OK as if it confirmed Arm/Disarm. That was wrong and should not be reintroduced.

---

## H7. Flight Control / PX4 Notes

Current placeholder command split:

```text
Arm / Disarm:
  dispatched through PX4 shell using tmux send-keys

Velocity:
  dispatched through AirSim moveByVelocityAsync
```

This is not yet the final robust PX4 control path.

Likely future direction:

```text
PX4-native arm
PX4-native takeoff or offboard entry
MAVLink local-NED velocity setpoints
PX4-native land/disarm
```

AirSim `moveByVelocityAsync` may not be sufficient once PX4 owns control mode and arming state.

---

## H8. Historical Debugging Rule

When a run shows:

```text
helper_output=OK
```

that only means the helper dispatched something successfully.

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

## H9. Archive Policy

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
