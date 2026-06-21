# Dedalus LLM Operating Brief

Active orientation document for a new LLM session. Keep short, current, and action-oriented.
Historical notes and superseded context live in `LLM.back.md`.

Repository: `guybarnahum/dedalus`

---

## Current Handoff State

```
Viewer (mission_unified_viewer.py / build/viewer.html):
  - L0 TTC radar (top-right inset): TTC-radial positioning, escape/flight vectors,
    HOVER mode, 80% size, heading label.
  - L0 sensor cone scope (top-left inset): az×el rectilinear panel, source-tagged dots
    (white=GT, gray=Depth, black=Visual), TTC halo coloring.
  - L0 C++ compute: compute_l0_polar_risk() + collect_l0_sensor_observations() in
    local_flight_map.cpp, serialized via world_snapshot SSE stream.
  - Viewer cosmetics: unified snap-to-preset (45°/Top/Side), zoom preserved across
    preset transitions, zoom display updates during animation.
  - Takeoff/home marker: green diamond "H" at first recorded ego position.
  - JSON serialization bug (spherical_risk_bins) fixed: bool first_bin pattern.
  - All 44/44 ctests pass. Viewer contract validator: 0 violations.

L0/L1/L2 obstacle map: implemented and operator-validated.
  See docs/two-level-obstacle-map.md for full architecture.

Next: L2 SQLite persistence (Stage 1 of L2/L3 plan below).
```

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

```
Milestones 2.20–2.30B: complete (behavior/follow/circle/sequence). See LLM.back.md.
Milestone 4.1C: AirSim depth obstacle detector, live-validated.
Milestone 4.3A-D: obstacle diagnostics hardening, complete.
Milestone 4.3F: viewer polish (view presets, coloring, vectors), operator-validated.
Milestone 4.3G: viewer HTML contract validator, complete.

Two-level obstacle map (L0+L1+L2): implemented and syntax-validated.
  L0  LocalFlightMapSnapshot: ego-local, 0.5 m cells, polar risk + sensor cone scope.
        compute_l0_polar_risk() + collect_l0_sensor_observations() run each tick.
        Serialized in WorldSnapshot SSE stream. Viewer: TTC radar + cone scope insets.
  L1  MissionLocalTraversabilityMap: 0.5 m voxels, decay 0.05/s, pruned at 0.1.
        Streamed to viewer as traversability surface (exterior voxel faces, LOD).
  L2  MissionLocalPlanningMap: 1 m × 1 m × 2 m voxels, evidence-keyed, no time decay.
        Flat vector + hashmap. Save/load: versioned plain text (planning_map_v1).
        Saved at finalize_mission_map_after_landing(); loaded at process start.

Active next slice: L2 SQLite persistence + L3 ESDF (see Section 4).
```

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
       -> L2: MissionLocalPlanningMap (1 m×2 m, evidence-keyed, disk-backed)
              -> MissionLocalPlanningMapPublisher -> SSE
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
  -> tools/visualization/mission_unified_viewer.py -> build/viewer.html
       Pure SPA; all state from /events SSE stream.
       Renders: L0 TTC radar + cone scope, L1 trav surface, L2 planning cells,
                obstacle evidence, ghost detections, trajectory, sensing volumes,
                mission event log, orientation gizmo, takeoff marker.
  -> tools/validation/validate-mission-unified-viewer.py (0 violations gate)
```

---

## 3. Persistent Obstacle Memory (5Q–5U, complete)

Default path: compact-delta-first, SQLite-backed.

```
Runtime:
  AirSim depth evidence -> mission_local_obstacle_map_deltas.jsonl (compact delta)
  --write-full-obstacle-map-artifact adds mission_obstacle_map_full.json (debug only)

Post-mission:
  mission_obstacle_map_deltas.jsonl
    -> mission_obstacle_map_deltas.sqlite
    -> maps/<site_id>/site_obstacle_map.sqlite
    -> out/<run>/obstacle_memory_manifest.json
```

Key commands:
```bash
simulation/airsim/run_mission.sh \
  --output-dir out/<run> \
  --merge-obstacle-map \
  --obstacle-map-site-id <site_id> \
  --obstacle-map-site-frame-id airsim_world \
  --obstacle-map-mission-id <mission_id>

python3 tools/avoidance/validate_obstacle_memory_manifest.py \
  out/<run>/obstacle_memory_manifest.json \
  --site-id <site_id> --site-frame-id airsim_world \
  --mission-id <mission_id> --site-map-format sqlite
```

---

## 4. L2 SQLite + L3 ESDF Implementation Plan

### Layer stack

```
L0  LocalFlightMapSnapshot      ego-local, 0.5 m, rebuilt every tick
                                → reflexive avoidance, TTC, escape vector
L1  MissionLocalTraversabilityMap  per-flight accumulator, decay 0.05/s, 0.5 m
                                → current sensor picture, feeds L2
L2  MissionLocalPlanningMap     persistent site map, 1 m voxels
                                → durable obstacle memory across flights
L3  LocalESDFMap                ESDF derived from local L2 window
                                → distance field + gradient = planning primitive
```

### Stage 1 — L2 SQLite persistence

Replace plain-text save/load with SQLite + WAL.
Schema: `cells(xi,yi,zi INTEGER PK, score REAL, confidence REAL, count INTEGER, updated_ns INTEGER)` + R-tree index.
Add dirty-cell set to `MissionLocalPlanningMap`; flush background thread every 10 s (dirty cells only).
Startup: spatial query for window around last known drone position.

Validation: roundtrip 10k cells; partial load by bbox; flush latency < 50 ms; crash recovery (WAL); ctests green.

### Stage 2 — Bounded in-memory L2 with spatial eviction

Cap in-memory L2 to `±horizon_m` (default 150 m) of drone position.
`slide_window(drone_pos)`: evict cells beyond `2×horizon`, stream in newly entered cells from SQLite.
Called from `CoreStackRunner::run_once()` when drone moves > `horizon/4`.

Validation: memory stays bounded over 2 km flight; continuity on re-entry; slide latency < 5 ms.

### Stage 2.5 — L2 planning API (`ray_cast` + `query_occupied_in_box`)

Add two query primitives directly on `MissionLocalPlanningMap` — these are L2-level, independent of L3:
- `ray_cast(origin, dir, max_range_m) → optional<Vec3>`: march through L2 voxels along a ray; return first occupied hit. Used by the navigation function (Stage 7) and feasibility checks.
- `query_occupied_in_box(bbox) → vector<Vec3>`: return centres of all occupied cells within an axis-aligned box. Used by the trajectory optimizer for local collision checking.

These are pure query functions — no side effects on L2 state.

Validation: ray through known wall returns hit at correct distance; ray through free space returns nullopt; bbox query returns exactly the expected cells; both complete in < 1 ms for a 40×40×20 m query volume.

**L1 OctoMap gate:** If L2's in-memory structure ever changes (e.g., to octree or multi-resolution), L1 OctoMap must come first — L2 is built from L1 leaf nodes. The current flat hashmap is fine for Stages 1–7. Explicitly defer L1 OctoMap until L2 structure needs to change; re-evaluate before any L2 structural refactor.

### Stage 3 — L3 EDT compute (free function)

`compute_esdf(l2, centre_world, window_half_m, d0_m) → LocalESDFMap`.
3D separable EDT (Meijster, O(N)); sparse hash map of shell cells only (`|d| < d0`).
Signed: negative inside occupied, clamped at −0.5 m. Gradient `∇d` precomputed per shell cell.
Interface: `query(pos) → (d, grad)`, `repulsion(pos, d0, k) → Vec3`, `is_clear(pos, r) → bool`.

Validation: flat-wall correctness; corner geometry; signed field; perf < 5 ms on 80×80×20 m window; all ctests green.

### Stage 4 — L3 incremental updates

`update_incremental(l2, dirty_voxels, d0)`: recompute ESDF within d0 of each dirty voxel only.
`update_tube(trajectory_points, tube_radius)`: JIT refinement along a proposed path.
Wired: `update_incremental()` after each L1→L2 step; `update_tube()` from trajectory planner (Stage 7).

Validation: incremental ≡ full recompute (< 0.01 m error); incremental < 1 ms for 50 dirty cells; tube < 2 ms.

### Stage 5 — Incremental L2 SSE streaming

Replace full-snapshot emission with distance-sorted chunked streaming (500 cells/message, 50 ms yield).
After initial load: delta mode — only cells touched since client's sequence watermark.

Validation: no single SSE message > 20 ms; complete after full stream; delta sends only changed cells.

### Stage 6 — L3 SSE streaming + viewer

Two viewer additions:

**a) ESDF gradient arrows (new layer):** New SSE event `esdf_delta`: changed shell cells as `{x,y,z,d,gx,gy,gz}`.
`drawESDFArrows()` — 3D line segments from cell centre in gradient direction,
colored by clearance (red d<1 m, yellow d<3 m, green d<d0). Toggle in layer controls.

**b) L3 net vector in L0 radar inset:** Compute the net L3 repulsion vector at the drone's position from the ESDF field. Draw it in the existing L0 radar inset alongside the reactive escape vector — same inset, distinct color (amber). Operators see both: immediate threat response (L0 escape, green) and pre-computed planned guidance (L3 net, amber) co-located in one view.

Validation: ESDF arrows point away from known obstacle; render < 4 ms for 2k shell cells; L3 net vector visible in radar inset and directionally consistent with ESDF arrows; contract validator green.

### Stage 7 — Navigation function + trajectory optimizer

`compute_navigation_function(l2, goal) → NavigationMap`: Dijkstra wavefront on L2 grid using `ray_cast` and `query_occupied_in_box` (Stage 2.5). No local minima by construction.
`optimize_trajectory(nav_map, l3, start, goal, config) → Trajectory`: minimum-snap polynomial + ESDF soft penalty. Uses `update_tube()` (Stage 4) for JIT L3 refinement along each candidate path.
`TrajectoryConfig`: `energy_weight`, `clearance_weight`, `time_weight`.

Validation: topology correctness in maze; all waypoints `is_clear(r_vehicle)`; snap bounded; weight monotonicity; obstacle-avoidance routes around wall.

### Stage 8 — L0 / L3 calibration

No new code. Simulation run: L0 trigger rate vs L3 clearance margin.
Target: 0 L0 triggers when trajectory `d_min > 2 m`.

### Dependencies

```
Stage 1 → Stage 2 → Stage 2.5 ─────────────────────────→ Stage 7 → Stage 8
                       │                                      ↑
                       └──────────────────→ Stage 5      Stage 4
                                                             ↑
Stage 3 ──────────────────────────────────────────→ Stage 4 → Stage 6

[L1 OctoMap: gate before any L2 structural change; defer until needed]
```

Stages 1–2 and Stage 3 are independent and can proceed in parallel.
Stage 2.5 must follow Stage 2 (needs the bounded in-memory store).
Stage 7 depends on both Stage 2.5 (ray_cast/bbox queries) and Stage 4 (update_tube).

---

## 5. Architecture Boundaries

```
WorldSnapshot           autonomy state
PerceptionPipelineOutput  evidence
Ghost detections        enter through same Observation3D path as real detections
Artifacts               evidence/debug outputs, not IPC
Overlay                 subscriber/renderer only
L0                      reflexive avoidance — no planner coupling
L3                      planning primitive — no flight command coupling until Stage 8 scoped
```

Do not:
- Add planner blocking, replanning, or command-sink avoidance unless explicitly scoped.
- Use YOLO/DETR/classifier outputs as prerequisite for obstacle avoidance.
- Derive visual obstacle coverage from vehicle yaw alone.
- Couple obstacle persistence, map-building, or sensing coverage to a flight command sink.
- Name files, validators, scripts, or symbols after planning labels or temporary session shorthand.

---

## 6. Patch Output and Safety Policy

```
- Patch scripts: concise OK:/ERROR: lines only. No grep/diff dumps after patches.
- Do not call sys.exit(), raise SystemExit, shell exit from generated patch snippets.
- Do not rely on set -e for patch control flow.
- Do not assume out/, generated artifacts, runtime logs, or validation dirs exist
  inside patch logic unless explicitly provided for that patch.
- If anchors do not match, print ERROR and do not write partial changes.
```
