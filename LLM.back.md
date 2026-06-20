# Dedalus LLM Historical Notes

This file is the archive for older LLM handoff material, superseded debugging context, and milestone history.

`LLM.md` is the active operating brief. Read it first.

This archive exists so historical context is preserved without distracting new LLM sessions from the current active task.

(AWS ec2 instance is at `35.82.43.156`)

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
simulation/airsim/settings.json
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
  dispatched through AirSim moveByVelocityAsync via simulation/airsim/scripts/airsim-send-velocity.py
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
simulation/airsim/scripts/test-flight.py worked, but the native sink did not reliably climb/execute trajectory.
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
tools/px4/px4-command-bridge.py
```

This preserves the C++ mission state machine while delegating flight-critical PX4/OFFBOARD control to the same `pymavlink` behavior proven in:

```text
simulation/airsim/scripts/test-flight.py
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
Python is not required for AirSim access. It is currently the stable choice for PX4/MAVLink mission control because `pymavlink` plus `simulation/airsim/scripts/test-flight.py` is the proven implementation.
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
- `tools/mission/mission-events-summary.py` can summarize and validate event artifacts after the run.
- `tools/mission/repeat-mission-smoke.sh` can run repeated live missions and validate each event artifact.
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

This exists because repeat runs showed stale armed telemetry even though PX4 shell arm/takeoff was healthy and `simulation/airsim/scripts/test-flight.py` succeeded. The fallback may move from `Prepare` to `Takeoff` after successful Arm dispatch and a short settle interval when armed telemetry is stale.

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

---

## H13. Milestone 3 Redefinition: Object-Conditioned Flight Behavior

After Milestone 2.20 closed the reliable live mission loop, the roadmap was sharpened.

Milestone 3 is no longer defined as tactical obstacle mapping. It is now:

```text
Milestone 3.0 — Object-conditioned flight behavior demo
```

The intended demo:

```text
AirSim/PX4 live mission starts
  -> drone takes off and reaches safe height
  -> perception/world model provides a detected/tracked class instance, such as person or car
  -> TargetSelector selects a class or instance
  -> BehaviorRuntime executes follow, circle, approach, or sequence behavior
  -> behavior emits bounded velocity vectors into the existing PX4 SITL/AirSim path
  -> drone returns home, lands, and disarms
  -> mission_events + snapshots prove the behavior sequence
```

Initial behavior language concepts:

```text
Target selector
Behavior type
Reference frame
Desired relative geometry
Constraints
Completion condition
Fallback behavior
```

Initial behavior types:

```text
hold
search
follow
approach
circle
go_home
land
sequence
```

Important behavior semantics:

```text
follow:
  maintain a relative 3D offset from a selected target

circle:
  orbit a static or slow selected target at radius/altitude/angular speed

approach:
  move toward a target until a standoff or relative-position condition, then optionally run another action
```

M3 should not own full obstacle avoidance. M3 proves object-conditioned behavior on top of the existing velocity-command path.

---

## H14. Post-Milestone 3 Spatial Autonomy Plan

After M3, the same behaviors should become obstacle-aware by inserting an avoidance layer between behavior and the flight sink:

```text
BehaviorController
  -> desired velocity vector
  -> TacticalAvoidancePlanner
  -> safe velocity vector
  -> Px4BridgeCommandSink
```

The sink must remain simple. It should receive bounded velocity/yaw intent. It should not own obstacle logic, target logic, or behavior policy.

Post-M3 roadmap:

```text
4.0 Local tactical occupancy map
  Drone-relative real-time map from vision/ego motion.

5.0 Reactive obstacle avoidance planner
  Modify desired behavior velocity into safe velocity using tactical occupancy.

6.0 Persistent traverse map / flight memory
  Remember known-safe corridors, blocked regions, risk/cost surfaces, and preferred lanes across flights.

7.0 Cached route solutions
  Reuse successful routes when live sensing and map priors agree; invalidate when live sensing contradicts memory.

8.0 Tactical map + drone POV visualization
  Visualize both takeoff-origin/drone-relative map and camera POV overlays.

9.0 Spatial autonomy demo with avoidance
  Follow/circle/approach an object-conditioned target while avoiding static/dynamic obstacles and updating traverse memory.

10.0 Multi-flight site memory
  Maintain route priors, hazard history, repeated obstacle history, and mission outcomes across missions.
```

Core post-M3 design split:

```text
Tactical occupancy map:
  real-time, short-horizon, safety-critical

Persistent traverse map:
  slower, historical, advisory

Route cache:
  proposed solution memory, invalidated by live sensing

Behavior:
  decides intent

Avoidance planner:
  modifies intent into safe motion

Flight sink:
  sends bounded velocity/yaw intent to PX4/SITL
```

Do not let route memory override fresh tactical sensing.

---

## H15. Milestone 2.24 Target Identity Rationale

Milestone 2.24 clarified that object-conditioned behavior must select a target object, not just the highest-confidence detection every frame.

The identity layers are deliberately separate:

```text
detection_id  ->  track_id  ->  agent_id  ->  identity_id
single frame      tracker       world model    recognized identity
```

Definitions:

```text
detection_id:
  A single detector observation in one frame.

track_id:
  Tracker-owned frame-to-frame continuity for one moving blob/object.
  Usually local to one tracker session and may reset after restart or long target loss.

source_track_id:
  The tracker ID preserved inside AgentState as provenance.
  It answers: which tracker track produced this world-model agent?

agent_id:
  World-model-owned object handle.
  Behavior and planning should select agents, not raw detections.
  Today it may be derived from source_track_id; later it can represent a fused object from multiple sensors/tracks.

identity_id:
  Identity-resolver-owned recognized real-world identity.
  Later this may be a known person, vehicle plate, drone serial, or cross-mission identity.
```

The practical M3 rule is:

```text
Select by agent_id/source_track_id when a stable target is specified, preserve that object across frames, and do not switch merely because another same-class object has higher confidence.
```

2.24A implementation decision:

```text
InMemoryWorldModel derives agent_id and identity_id from Observation3D.track_id, while preserving Observation3D.track_id as AgentState.source_track_id. WorldSnapshot JSON emits source_track_id so artifacts can prove which tracker object became which world-model agent.
```

Example:

```text
track_id:        ghost_person_001
agent_id:        agent_ghost_person_001
identity_id:     identity_ghost_person_001
source_track_id: ghost_person_001
```

Pre-camera validation direction:

```text
Synthetic / AirSim frame
  -> GhostDetectionProvider or ScriptedTargetProvider
  -> PerceptionPipelineOutput.observations
  -> InMemoryWorldModel
  -> WorldSnapshot.agents with stable source_track_id
  -> TargetSelector
  -> ObjectBehaviorMissionController later
```

Do not validate target selection by hardcoding `selected_target` directly from config as the main path. That bypasses the world-model and TargetSelector plumbing that real camera detections must use.

---

## H16. Milestone 2.26 Runtime Event Stream History

2.26D moved runtime visualization and external subscribers away from artifact-file polling.

The important design correction was:

```text
simulation/airsim/scripts/airsim-world-overlay.py should be a subscriber/renderer only.
It should not evaluate GhostScenario locally.
It should not poll snapshot_manifest.txt in normal mode.
It should not own source modes such as combined/world_snapshot/artifact_snapshot.
```

The runtime path became:

```text
CoreStackRunner
  -> GhostTargetProvider::frame_at(...)
       evaluates GhostScenario once at frame time
       publishes GhostDetectionsFrame for runtime-event subscribers
       injects the same Observation3D objects into PerceptionPipelineOutput
  -> InMemoryWorldModel
  -> WorldSnapshotPublisher
```

The publisher topology became:

```text
GhostDetectionsPublisher
  -> RuntimeEventStreamServer

WorldSnapshotPublisher
  -> LatestWorldSnapshotSubscriber
  -> ArtifactSnapshotWriter
  -> RuntimeEventStreamServer
```

The runtime event stream emits both:

```text
ghost_detections
world_snapshot
```

on the same TCP JSONL stream. This lets the AirSim overlay render PLAN / PLAN* from `ghost_detections` and AG / EGO from `world_snapshot` without any file dependency.

Important process note:

```text
Generalizing the snapshot-only stream server into a multi-event RuntimeEventStreamServer was an architectural/runtime design choice. Future similarly meaningful design choices should pause for a concise plan and approval before implementation, unless the design was already explicitly agreed or is trivial.
```

Canonical current diagrams are in:

```text
docs/runtime_dataflow.md
```

## Persistent obstacle memory checkpoint

Persistent obstacle memory plan recorded after mission-local obstacle map validation.

Current rule:
- Mission-local maps are one-flight/takeoff-relative.
- Persistent maps are site-local and merged across missions using explicit `site_T_mission`.
- Persistent artifacts must store `time_unit: unix_ns` and primitive evidence/timestamp/count fields.
- Calendar time alone should not erase maps for sites that have not been revisited.
- Use site-relative age normalization:
  `relative_gap_seconds = max(0, cell_age_seconds - site_staleness_seconds)`.
- Derived `freshness_score`, `active_score`, and `status` should be recomputable by tooling.
- Next slices: 5H export mission maps, 5I merge site maps, 5J score/age maps, 5K run_mission.sh post-process, 5L diagnostics-only preload.

---

## H9. Track 4.3 obstacle diagnostics consolidation

Track 4.3 established the canonical diagnostics-only classless obstacle stack:

```text
ObstacleEvidence
  -> MissionLocalObstacleMap
       provider-neutral same-update compaction / fusion
       per-cell positive/negative/duplicate counters
       no AirSim detector-side coalescing policy
  -> LocalFlightMapAccumulator::update_from_mission_local_map(...)
       ego-local crop
       inflated exclusion diagnostics
  -> WorldSnapshot JSON
       mission-local and local-flight diagnostic counters
  -> mission_local_obstacle_viewer
       existing viewer, extended metrics only
```

Important correction:

```text
AirSim depth should emit raw provider-neutral ObstacleEvidence.
Detector-side coalescing flags or detector-local map policy should not be reintroduced.
Compaction/fusion belongs in MissionLocalObstacleMap.
```

4.3A-D were diagnostics/observability slices only. They did not add planner blocking, replanning, command gating, or command-sink coupling. Future avoidance/control work should start as a separately scoped 6.x effort.

---

## H17. Two-Level Obstacle Map — Milestone History

Prior to this milestone, MissionLocalTraversabilityMap had:
  occupied_score_decay_per_second = 0.0  (no decay — cells accumulated forever)
  No pruning — cells were never removed from cells_ / cell_index_
  No cross-mission persistence — RAM-only, lost at process exit

The two-level map introduced:

L1 enhancements (MissionLocalTraversabilityMap):
  occupied_score_decay_per_second = 0.05  (decays from 1.5 → 0.1 in ~28 s)
  prune_min_occupied_score = 0.1         (eviction floor)
  prune_interval_ticks = 10              (O(N) compaction every 10 assimilator ticks)
  prune_weak_cells(): stable_partition + index rebuild

L2 (MissionLocalPlanningMap, new class):
  1 m × 1 m × 2 m voxels
  Evidence-keyed: occupied max-merges in; free-space multiplicatively evicts
    (free_evidence_weight = 0.5; 1 observation clears barely-occupied cell,
     ~4 observations clear max-evidence cell)
  No time decay — absence of observation ≠ free space
  Disk persistence: planning_map_v1 text format, one cell per line
  Atomic save via temp-then-rename
  CoreStackRunnerConfig::planning_map_persistence_path wires the file path

Open architectural questions documented in docs/two-level-obstacle-map.md:
  L0: should LocalFlightMap use polar cone representation?
  L1: octree for multi-resolution at site scale?
  L2: OctoMap-style octree as long-term direction?

Viewer state at this milestone:
  Raw evidence (MissionLocalObstacleMap deltas) shown as "obstacle cells"
  L1 traversability shown as "trav cells" with exterior face rendering
  L0 ego sub-window planned but not yet implemented
  Raw evidence is transient sensor input; L1 is the accumulated filtered view

