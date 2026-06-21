# Dedalus LLM Historical Notes

Archive for superseded context, milestone history, and architectural rationale.
`LLM.md` is the active operating brief. Read it first.

---

## H7. Superseded Flight Control Paths

The mission pipeline tried two dead-end paths before the current working `px4_bridge`:

**H7.1 AirSim velocity helper** — `airsim-send-velocity.py`. Arm could work; takeoff/trajectory were inconsistent. `moveByVelocityAsync` is not the correct live mission control path with PX4-backed Colosseum.

**H7.2 Native C++ MAVLink sink** — `src/behavior/px4_mavlink_command_sink.cpp`. Failed to reliably climb/execute trajectory. Root cause: `pymavlink` already handles heartbeat peer learning, MAVLink1/2 framing, PX4 mode mapping, ACK handling, OFFBOARD priming, and LOCAL_POSITION_NED feedback. Reimplementing all that in C++ created avoidable failure modes. Do not make this the default path.

**H7.3 Current path** — `Px4BridgeCommandSink` + `tools/px4/px4-command-bridge.py`. Keeps C++ autonomy core; delegates PX4/OFFBOARD control to pymavlink (same as proven `test-flight.py`). Key fixes: open MAVLink lazily after shell arm/takeoff; use PX4 shell for lifecycle; use OFFBOARD priming; climb via LOCAL_POSITION_NED feedback.

---

## H8. Python vs Native C++ Rationale

AirSim-side tasks (frame streaming, ego/state reads, session prep) are plausible C++ migration targets — AirSim has a native C++ client API.

PX4/MAVLink mission control is different. `pymavlink` owns heartbeat routing, MAVLink1/2 parsing, PX4 mode mapping, COMMAND_ACK, OFFBOARD priming, and UDP endpoint semantics. The native C++ experiment broke on all of these.

**Decision:** Keep C++ autonomy core, world model, mission runtime, and state machine. Keep PX4/MAVLink bridge in Python until mission behavior is repeatedly stable. If reducing helpers is desired, migrate AirSim frame/ego helpers to C++ first. Only then consider a C++ PX4/MAVLink backend — and only with a tested MAVLink library, not ad-hoc packet encoding.

---

## H9. Command Dispatch Is Not Vehicle Truth

`helper_output=OK` or `{"ok":true,"command":"..."}` means the helper/bridge dispatched something. It does not prove the vehicle armed, took off, moved, landed, or disarmed. Always confirm from world-model telemetry (`WorldSnapshot.ego`). `mission_options.flight_arm_dispatch_fallback_s=2.0` exists because stale armed telemetry was observed on repeat runs even with healthy PX4 shell arm/takeoff.

---

## H13. Milestone 3: Object-Conditioned Flight Behavior

After M2, roadmap was sharpened. M3 = object-conditioned flight behavior demo. Not full obstacle avoidance — M3 proves follow/circle/approach/sequence on top of the velocity-command path.

Behavior types: `hold`, `search`, `follow`, `approach`, `circle`, `go_home`, `land`, `sequence`.

Key behavioral semantics (all complete as of 2.30B):
- **follow:** maintain 3D offset from selected target. Moving targets converge to matched target velocity.
- **circle:** continuous orbit-capture control law. Robust to imperfect insertion geometry. `object_behavior_zero_target_velocity` for known-static AirSim bindings.
- **approach:** move toward target until standoff, then optionally chain next behavior.
- **sequence:** `approach → circle` steps tracked via `sequence_step_index_`; emits `behavior_sequence_step_complete` / `behavior_sequence_step_start` / `behavior_complete reason=sequence_complete`.

Milestone 2.30B: moving-target stress matrix validated (medium, side-motion, diagonal far-animal trajectories). Orbit law is direction-agnostic.

---

## H14. Post-M3 Spatial Autonomy Roadmap

The L0/L1/L2/L3 architecture described in `LLM.md` Section 4 is the implementation of this roadmap.

```
4.0 Local tactical occupancy map                  → L0 (done)
5.0 Reactive obstacle avoidance planner            → L0 TTC + escape vector (done)
6.0 Persistent traverse map / flight memory        → L2 SQLite (Stage 1, next)
7.0 Cached route solutions                         → L3 ESDF + trajectory optimizer (Stages 3–7)
8.0 Tactical map + drone POV visualization         → viewer (L0 radar + cone scope, done; L3 arrows, Stage 6)
9.0 Spatial autonomy with avoidance                → L0/L3 calibration (Stage 8)
10.0 Multi-flight site memory                      → L2 site persistence (Stage 1–2)
```

Core design split:
- Tactical occupancy map (L0): real-time, short-horizon, safety-critical.
- Persistent traverse map (L2): slower, historical, advisory.
- Planning primitive (L3 ESDF): distance field + gradient, derived from L2.
- Behavior: decides intent.
- Avoidance planner: modifies intent into safe motion.
- Flight sink: bounded velocity/yaw to PX4. Does not own obstacle logic.

Do not let route memory override fresh tactical sensing.

---

## H15. Target Identity Rationale

Behavior selects agents, not raw detections. Identity layers:

```
detection_id → track_id → agent_id → identity_id
single frame    tracker    world model  recognized identity
```

`agent_id` / `source_track_id` are the stable handles for behavior. Do not switch targets just because another same-class object has higher confidence. `InMemoryWorldModel` derives `agent_id` from `Observation3D.track_id`, preserving it as `AgentState.source_track_id`.

Do not validate target selection by hardcoding `selected_target` from config. That bypasses world-model and TargetSelector plumbing that real camera detections must use.

---

## H16. Runtime Event Stream Architecture

`airsim-world-overlay.py` must be a subscriber/renderer only. It must not evaluate `GhostScenario`, poll `snapshot_manifest.txt`, or own source modes. This was corrected in 2.26D.

Current publisher topology:
```
GhostDetectionsPublisher → RuntimeEventStreamServer
WorldSnapshotPublisher   → LatestWorldSnapshotSubscriber
                         → ArtifactSnapshotWriter
                         → RuntimeEventStreamServer
```

Both `ghost_detections` and `world_snapshot` go on the same TCP JSONL stream. Canonical diagrams: `docs/runtime_dataflow.md`.

Future similarly meaningful design choices should pause for a concise plan and approval before implementation, unless the design was already explicitly agreed or is trivial.

---

## H17. Two-Level Obstacle Map — Design Decisions

Prior to this milestone, `MissionLocalTraversabilityMap` had no decay, no pruning, and no persistence. Changes:

**L1 enhancements:**
- `occupied_score_decay_per_second = 0.05` (decays 1.5 → 0.1 in ~28 s)
- `prune_min_occupied_score = 0.1`
- `prune_interval_ticks = 10`
- `prune_weak_cells()`: `stable_partition` + index rebuild

**L2 (new class `MissionLocalPlanningMap`):**
- 1 m × 1 m × 2 m voxels (16× fewer than L1)
- Occupied max-merges in; free-space multiplicatively evicts (`free_evidence_weight = 0.5`)
- No time decay — absence of observation ≠ free space
- Disk persistence: `planning_map_v1` text format, atomic temp-then-rename
- `CoreStackRunnerConfig::planning_map_persistence_path` wires the file path

Architectural questions resolved since this milestone:
- L0 polar representation: **done** — `compute_l0_polar_risk()` produces 36 az sectors + 36×9 spherical bins.
- L2 SQLite: **next** — Stage 1 of the L3 plan in `LLM.md`.
- L2 octree: deferred until SQLite is validated.

---

## Persistent Obstacle Memory — Decay Policy

- Store absolute timestamps with explicit `time_unit: unix_ns`.
- Store raw evidence primitives: first/last seen, last confirmed occupied, last observed free, positive/negative counts, source stats.
- Do not blindly decay/delete obstacles because a site has not been visited.
- Normalize cell age against whole-site staleness: `relative_gap_seconds = max(0, cell_age_seconds - site_staleness_seconds)`.
- Strong decay should come from contradiction or revisits without reconfirmation, not calendar time alone.
- Persisted maps are site-local, not necessarily geodetic/global, until a real site anchor is available.
- Derived `freshness_score`, `active_score`, and `status` should be recomputable by tooling, not stored as primary fields.

---

## H18. Completed Behavioral Milestones (2.24–2.30B)

For full validation records see:
- `docs/moving_target_behavior_validation_results.md`
- `docs/selected_entity_slow_moving_animal_validation.md`
- `docs/airsim_depth_obstacle_detector_validation.md`

All sequence behavior milestones (2.24–2.30B) are complete. The behavioral runtime, target selector, ghost provider, follow/circle/approach/sequence controllers, camera/gimbal pointing, and AirSim existing-object binding are validated and stable. Do not modify flight/runtime behavior semantics unless explicitly scoped.
