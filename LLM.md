# Dedalus LLM Operating Brief

Active orientation document for a new LLM session. Keep short, current, and action-oriented.
Historical notes and superseded context live in `LLM.back.md`.

Repository: `guybarnahum/dedalus`

---

## System Goals

Dedalus is an autonomous drone that behaves like a curious, social bird:

1. **Map** — Fly the region autonomously, build a persistent obstacle and route map (L2), fly faster and with more confidence over time, pre-empt what is around the corner (wires, poles, safe corridors).

2. **Perceive actors** — Detect and identify dynamic actors (vehicles, individuals) in real time. Classify by type and track with velocity/heading estimates.

3. **Interact** — Approach, chase, circle, inspect, "go over" a selected actor. Mimic how a curious crow engages with something interesting.

4. **Coordinate** — Communicate with peer drones to partition the map and allocate dynamic actors so a swarm explores and tracks without redundancy.

Goals 1–2 are the active focus. Goals 3–4 are next.

---

## 0. Ground-Truth Patch Policy

```
Before offering code changes, inspect the current repo files that define the call
path, data flow, flags, schemas, tests, and scripts being changed.

Do not guess file structure, option names, parser blocks, test layout, function
signatures, enum values, generated artifact paths, or runtime wiring.

Do not assume a wrapper script forwards a flag just because the binary supports it.
Verify the wrapper parser and pass-through path.

When enhancing runtime or data-flow code, first trace:
  source of data → owning publisher/accumulator → serialization boundary
  → transport or artifact writer → consuming test/tool/viewer → validation command

Prefer small, anchored patches against inspected code. If the local file structure
differs from the expected anchor, stop and inspect instead of emitting broad patches.

Balance architectural purity, implementation efficiency, and development risk.
Use C++ when the feature belongs in the runtime ownership boundary; use Python/tools
only for diagnostics, offline conversion, or intentionally external workflows.
```

---

## 1. Active Milestones

### Obstacle map stack — COMPLETE (Stages 1–6)

```
L0  LocalFlightMapSnapshot      ego-local, 0.5 m, rebuilt every tick
                                → TTC, escape vector, polar + spherical risk grid
                                → Viewer: TTC radar inset + sensor cone scope inset
L1  MissionLocalTraversabilityMap  per-flight accumulator, 0.5 m, decay 0.05/s
                                → feeds L2 each tick
                                → Viewer: exterior voxel surface (LOD, throttled 2 s)
L2  MissionLocalPlanningMap     persistent site map, 1 m × 1 m × 2 m voxels
                                → SQLite + WAL + dirty-cell flush thread (10 s)
                                → slide_window() bounded in-memory (±150 m)
                                → ray_cast + query_occupied_in_box planning API
                                → delta SSE streaming (write_seq watermark)
                                → Viewer: planning cells + age dimming
L3  LocalESDFMap                ESDF derived from L2 window (always recomputed, never saved)
                                → Meijster 3D EDT; sparse shell cells only; signed field
                                → update_incremental() in tick loop (Path 2)
                                → SSE: esdf_delta events (full on startup, delta on change)
                                → Viewer: gradient arrows (colored by clearance) +
                                          net repulsion in L0 radar inset (amber)
```

All ctests pass (44+ tests). Viewer contract validator: 0 violations.

### Next active slice: Perception (see Section 3)

---

## 2. Current Runtime Architecture

```
AirSim live frame + ego sidecar
  -> AirSimFrameSource
  -> FrameHintEgoProvider
  -> CoreStackRunner
       -> GhostTargetProvider -> GhostDetectionsPublisher
       -> SensingCoverageProvider -> obstacle_sensing_volumes
       -> AirSimDepthObstacleDetector -> PerceptionPipelineOutput.obstacle_evidence
  -> MissionLocalObstacleMap (raw evidence, per-tick)
  -> MissionMapAssimilator
       -> L1: MissionLocalTraversabilityMap (0.5 m, decay 0.05/s)
              -> traversability_map_publisher -> SSE (throttled 2 s)
       -> L2: MissionLocalPlanningMap (1 m×2 m, SQLite-backed, slide_window)
              -> MissionLocalPlanningMapPublisher -> SSE (delta, watermark)
  -> L3: LocalESDFMap (derived from L2 each tick, never saved to disk)
              -> LocalESDFMapPublisher -> SSE (esdf_delta)
  -> InMemoryWorldModel -> WorldSnapshotPublisher
       -> LatestWorldSnapshotSubscriber
       -> ArtifactSnapshotWriter
       -> RuntimeEventStreamServer (TCP JSONL + HTTP/SSE)
  -> LatestWorldSnapshot
       -> compute_l0_polar_risk(snap.local_flight_map, vel_body)
       -> collect_l0_sensor_observations(snap.local_flight_map, snap.obstacle_evidence, vel_body)
  -> MissionRuntime -> ObjectBehaviorMissionController
       -> BehaviorSpec sequence runtime (approach → circle steps)
       -> VelocityCommand / CameraPointingCommand
  -> Px4BridgeCommandSink -> PX4 / AirSim
```

Sidecar viewer path:
```
RuntimeEventStreamServer TCP JSONL (port 7788 / --world-snapshot-stream-port)
  -> dedalus_viewer (apps/dedalus_viewer.cpp)
       --host/--port: upstream TCP
       --http-port: browser-facing SSE (default 8090)
       --replay-dir: replay from artifact dir
       --static-root / --static-default-file: static HTML serving
       L2 DB loaded on connect; static cache burst replayed to new SSE clients.
  -> tools/visualization/mission_unified_viewer.py -> build/viewer.html
       Pure SPA; all state from /events SSE stream.
       Renders: L0 TTC radar + cone scope, L1 trav surface, L2 planning cells,
                L3 ESDF arrows + net repulsion, obstacle evidence, ghost detections,
                trajectory, sensing volumes, mission event log, orientation gizmo,
                takeoff marker.
  -> tools/validation/validate-mission-unified-viewer.py (0 violations gate)
```

---

## 3. Active Next Slice: Perception

Active focus is perception — identifying and tracking dynamic actors in the scene.
Planning work (Stages 7–8) is deferred; see Section 5.

### P1 — Actor detection baseline

Goal: get per-frame bounding box + class label for dynamic actors (people, vehicles)
from the existing AirSim depth + RGB pipeline, without coupling to avoidance.

Key questions to resolve before implementation:
- Which detector backend? Options: (a) AirSim GT labels as oracle for simulation
  development (already partially wired via ghost/GT path); (b) real depth-based
  clustering from the existing `AirSimDepthObstacleDetector`; (c) vision model (DETR
  or lightweight YOLOv8n running locally).
- Detection output goes into `PerceptionPipelineOutput` as a new `ActorObservation`
  type (separate from `ObstacleEvidence` — obstacles are static geometry; actors are
  dynamic agents with identity).
- **Do not reuse `ObstacleEvidence`** for dynamic actors — different semantics, different
  accumulation policy (actors have velocity, decay fast, are not written to L2).

### P2 — Actor tracking

Goal: maintain a short-memory track of each actor: position, velocity, heading, class.
Track list is part of `WorldSnapshot` (not `LocalFlightMapSnapshot` — not ego-local).

Primitives needed:
- `ActorTrack`: id, class, position, velocity, heading, last_seen_ns, confidence.
- Simple Kalman or constant-velocity predictor per track.
- Track lifecycle: init on first detection, update on match, age out after 2 s no-match.
- Publish as `actor_tracks` SSE event; viewer draws labeled dots with velocity vectors.

### P3 — Exploration planner (curiosity flight)

Goal: given the current L2 map, identify the frontier (boundary of mapped vs. unknown
space) and generate waypoints to extend coverage. Feeds the mission controller.

Primitives needed:
- `FrontierMap`: derived from L2, marks cells adjacent to unmapped space.
- `ExplorationPlanner`: ranks frontier cells by information gain + distance cost;
  returns next waypoint.
- Plugs into mission controller as a new behavior type: `ExploreStep`.

### P4 — Tag / chase / inspect behavior

Goal: select an actor track and fly approach → orbit → inspection sequence.
Builds on the existing `BehaviorSpec` sequence runtime.

New behavior steps: `ApproachActorStep`, `OrbitActorStep`, `InspectActorStep`.
Each step takes a `track_id` and closes the loop on the `ActorTrack` position.

### P5 — Multi-drone coordination (future)

Not started. Requires: peer discovery, map-partition protocol, actor-ownership protocol.
Likely IPC: shared L2 DB merge (offline `tools/mission/merge_l2_maps.py`) + lightweight
peer heartbeat stream. Out of scope until P1–P4 are stable.

---

## 4. Architecture Boundaries

```
WorldSnapshot           autonomy state
PerceptionPipelineOutput  evidence + actor observations
ObstacleEvidence        static geometry evidence → feeds L1/L2
ActorObservation        dynamic agent evidence → feeds actor tracker, NOT L2
Ghost detections        enter through same Observation3D path as real detections
Artifacts               evidence/debug outputs, not IPC
Overlay                 subscriber/renderer only
L0                      reflexive avoidance — no planner coupling
L3                      planning primitive — no flight command coupling until Stage 7 scoped
```

Do not:
- Feed dynamic actor evidence into L2 (actors are not static obstacles).
- Add planner blocking, replanning, or command-sink avoidance unless explicitly scoped.
- Use YOLO/DETR outputs as prerequisite for *obstacle avoidance* (fine for actor ID).
- Couple obstacle persistence, map-building, or sensing coverage to a flight command sink.
- Merge L0/L1/L2/L3 representations.
- Name files or symbols after planning labels or temporary session shorthand.

---

## 5. Deferred Planning Work (Stages 7–8)

Deferred pending perception milestone. Do not start until explicitly resumed.

### Stage 7 — Navigation function + trajectory optimizer

`compute_navigation_function(l2, goal) → NavigationMap`: Dijkstra wavefront on L2 grid
using `ray_cast` and `query_occupied_in_box` (Stage 2.5, already implemented).
`optimize_trajectory(nav_map, l3, start, goal, config) → Trajectory`: minimum-snap
polynomial + ESDF soft penalty. Uses `update_tube()` (Stage 4, already implemented).
`TrajectoryConfig`: `energy_weight`, `clearance_weight`, `time_weight`.

Validation: topology correctness in maze; all waypoints `is_clear(r_vehicle)`; snap
bounded; weight monotonicity; obstacle-avoidance routes around wall.

### Stage 8 — L0/L3 calibration (no new code)

Simulation run: L0 trigger rate vs L3 clearance margin on a planned trajectory.
Target: 0 L0 triggers when trajectory `d_min > 2 m`.

### L3 persistence policy

L3 is always recomputed from L2 — it is never saved to disk. The `esdf_persistence_path`
field in `CoreStackRunnerConfig` is intentionally unwired in `dedalus_mission_loop.cpp`.
L3 recompute from L2 at startup costs ~6 ms and is always correct.

---

## 6. Geo-Region L2 Map — Storage, Naming, and Anchor Rules

### Directory layout

```
maps/
  airsim_32.457N_117.123W/
    l2_map.db       # MissionLocalPlanningMap SQLite — cross-mission persistent L2
    site.yaml       # anchor definition (see below)
  rw_32.457N_117.123W/
    l2_map.db
    site.yaml
```

Source prefixes: `airsim_` (AirSim SITL), `rw_` (real-world GPS), `px4sitl_` (PX4 SITL with custom origin).
Coordinates: lat then lon, 3 decimal places (~111 m resolution), cardinal suffix (N/S, E/W). No minus signs.
`maps/` is gitignored — it is a runtime artifact, not a repo asset.

### site.yaml — anchor definition

```yaml
# Created once on first mission for this site. Never overwritten.
src: airsim                    # airsim | rw | px4sitl
anchor_lat:  32.457
anchor_lon: -117.123
anchor_alt_m: 0.0
created_at: 2026-06-24T00:00:00Z
frame: NED                     # all L2 cell coordinates are NED relative to this anchor
```

### ⚠ WARNING — Anchor conflict / overlapping maps

Two missions with different anchors but overlapping flight areas will produce conflicting DBs.
Rules: anchor is a site config, not a mission property. Check existing `maps/` before creating
a new site. Offline merge tool: `tools/mission/merge_l2_maps.py` (to be written).

---

## 7. Patch Output and Safety Policy

```
- Patch scripts: concise OK:/ERROR: lines only. No grep/diff dumps after patches.
- Do not call sys.exit(), raise SystemExit, shell exit from generated patch snippets.
- Do not rely on set -e for patch control flow.
- Do not assume out/, generated artifacts, runtime logs, or validation dirs exist
  inside patch logic unless explicitly provided for that patch.
- If anchors do not match, print ERROR and do not write partial changes.
```
