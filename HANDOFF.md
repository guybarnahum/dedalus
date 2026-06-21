# Dedalus Current Handoff

You are continuing work on the Dedalus repo.
Repository: `guybarnahum/dedalus`

First read: **LLM.md** (current state + L3 plan)
Then read: `docs/two-level-obstacle-map.md` and any docs listed below that are relevant to the task.
Historical context only if needed: `LLM.back.md`

Run `git log --oneline -5` to confirm current commit before starting work.

---

## Active Development State

```
Viewer (mission_unified_viewer.py → build/viewer.html):
  COMPLETE and operator-validated:
  - L0 TTC radar inset (top-right): TTC-radial, escape/flight vectors, HOVER mode.
  - L0 sensor cone scope (top-left): az×el panel, source-tagged (white/gray/black),
    TTC halo coloring, zeroed when no data.
  - Takeoff/home marker: green diamond "H".
  - Unified snap-to-preset (45°/Top/Side): canonical yaws 45/135/225/315°,
    zoom preserved, zoom display updated during animation.
  - Color mode toggle: visibility:hidden (no layout resize).
  - Viewer contract validator: 0 violations (validate-mission-unified-viewer.py).

L0 C++ (complete):
  compute_l0_polar_risk() + collect_l0_sensor_observations() in local_flight_map.cpp.
  Polar sectors (1-D) + spherical bins (2-D az×el) + sensor observations serialized
  in world_snapshot SSE stream. JSON bool first_bin bug fixed.

L1/L2 (complete):
  L1 MissionLocalTraversabilityMap: 0.5 m, decay 0.05/s, streamed to viewer.
  L2 MissionLocalPlanningMap: 1 m×2 m, plain-text save/load, streamed to viewer.
  All 44/44 ctests pass.

Next: Stage 1 — L2 SQLite persistence (see plan below).
```

---

## Four-Level Obstacle Map Architecture

See `docs/two-level-obstacle-map.md` for full detail.

```
L0  LocalFlightMapSnapshot        ego-local, 0.5 m cells, rebuilt every tick
      Polar risk: compute_l0_polar_risk() → 36 az sectors + 36×9 spherical bins
      Sensor obs: collect_l0_sensor_observations() → source-tagged evidence
      Viewer: TTC radar inset (top-right) + cone scope inset (top-left)
      Role: reflexive avoidance, TTC, escape vector. No planner coupling.

L1  MissionLocalTraversabilityMap  per-flight accumulator, 0.5 m, decay 0.05/s
      Feeds L2 via update_from_traversability() each tick.
      Streamed to viewer as exterior voxel face surface (LOD).
      Lifetime: per-flight session.

L2  MissionLocalPlanningMap        persistent site map, 1 m × 1 m × 2 m voxels
      Current: flat vector + hashmap, plain-text file (planning_map_v1).
      Evidence-keyed: occupied max-merges in, free-space evicts.
      No time decay. Saved at finalize_mission_map_after_landing().
      Loaded at process start from planning_map_persistence_path_.
      Target: SQLite + WAL + R-tree (Stage 1).

L3  LocalESDFMap                   ESDF computed from local L2 window
      Not yet implemented. Plan: Stages 3–4.
      Role: distance field + gradient = planning primitive for trajectory optimizer.
      Derived from L2, not stored persistently.
```

Representation boundaries — keep these distinct:
- `ObstacleEvidence` — raw temporary input to map update, not stored
- `LocalFlightMapSnapshot` (L0) — current-hazard working set, ego-local
- `MissionLocalTraversabilityMap` (L1) — per-flight filtered accumulator
- `MissionLocalPlanningMap` (L2) — cross-mission persistent store
- `LocalESDFMap` (L3) — planning primitive, derived from L2
- Visualization/debug/replay — not the runtime/control storage format

---

## L2 SQLite + L3 ESDF Implementation Plan

Full plan with validation criteria is in LLM.md Section 4. Summary:

```
Stage 1    L2 SQLite persistence
             Replace plain-text save/load with SQLite + WAL + R-tree.
             Add dirty-cell set; background flush thread (10 s, dirty cells only).
             Startup: spatial query for local window around drone.
             Validate: roundtrip, partial load, flush < 50 ms, crash recovery, ctests.

Stage 2    Bounded in-memory L2 + spatial eviction
             Cap in-memory to ±150 m of drone; slide_window() on movement > 37 m.
             Evict far cells; stream in new cells from SQLite.
             Validate: memory bounded over 2 km flight; slide < 5 ms.

Stage 2.5  L2 planning API: ray_cast + query_occupied_in_box
             ray_cast(origin, dir, max_range) → optional<Vec3>: first occupied hit.
             query_occupied_in_box(bbox) → vector<Vec3>: all occupied cells in box.
             Pure query, no side effects on L2 state.
             Validate: hit at correct distance; nullopt through free space; < 1 ms.
             [L1 OctoMap: insert here before any L2 structural change; defer until needed]

Stage 3    L3 EDT compute (free function, no IPC)
             compute_esdf(l2, centre, window, d0) → LocalESDFMap.
             Meijster 3D separable EDT, O(N). Sparse hash map (shell cells only).
             Signed field (negative inside occupied, clamped −0.5 m).
             Validate: flat-wall + corner correctness; perf < 5 ms on 80×80×20 m.

Stage 4    L3 incremental updates
             update_incremental(dirty_voxels): recompute within d0 of dirty cells.
             update_tube(trajectory, radius): JIT narrow-band for path validation.
             Validate: incremental ≡ full (< 0.01 m); < 1 ms for 50 dirty cells.

Stage 5    Incremental L2 SSE streaming
             Distance-sorted chunked initial load (500 cells/msg, 50 ms yield).
             Delta mode after initial load (sequence watermark).
             Validate: no spike > 20 ms per message; complete after full stream.

Stage 6    L3 SSE streaming + viewer
             a) esdf_delta SSE event; drawESDFArrows() — 3D arrows colored by clearance;
                toggle in layer controls.
             b) L3 net repulsion vector in L0 radar inset (amber), alongside L0 escape
                vector (green). Both reflexive and planned guidance in one view.
             Validate: arrows correct; render < 4 ms; net vector consistent with arrows;
                       contract validator green.

Stage 7    Navigation function + trajectory optimizer
             compute_navigation_function(l2, goal) → NavigationMap (Dijkstra on L2,
               uses ray_cast + query_occupied_in_box from Stage 2.5).
             optimize_trajectory(nav, l3, start, goal, config) → Trajectory.
               Minimum-snap + ESDF soft penalty; uses update_tube() (Stage 4).
             TrajectoryConfig: energy_weight, clearance_weight, time_weight.
             Validate: topology, collision-free, snap bounded, weight monotonicity.

Stage 8    L0/L3 calibration (no new code)
             Sim run: L0 trigger rate vs L3 clearance margin.
             Target: 0 L0 triggers when d_min > 2 m on planned trajectory.

Dependencies:
  Stage 1 → Stage 2 → Stage 2.5 → Stage 7 → Stage 8
                     → Stage 5         ↑
  Stage 3 → Stage 4 → Stage 6    Stage 4 ↗
  Stages 1 and 3 independent (can be parallelised).
  Stage 2.5 must follow Stage 2.
  Stage 7 depends on Stage 2.5 (queries) AND Stage 4 (update_tube).
  [L1 OctoMap: gate before L2 structural change; defer until needed]
```

---

## Runtime Commands

Build and test:
```bash
cmake --build build-staging -j$(sysctl -n hw.logicalcpu)
ctest --test-dir build-staging --output-on-failure
```

Generate and validate viewer:
```bash
python3 tools/visualization/mission_unified_viewer.py --output build/viewer.html
python3 tools/validation/validate-mission-unified-viewer.py build/viewer.html
```

Start dedalus_viewer sidecar (live mode):
```bash
./build-staging/apps/dedalus_viewer \
  --host 127.0.0.1 --port 47770 \
  --http-port 8090 --static-root build-staging
```

Start dedalus_viewer sidecar (replay mode):
```bash
./build-staging/apps/dedalus_viewer \
  --replay-dir out/<run> --http-port 8090 --static-root build-staging
```

Start AirSim mission with full obstacle pipeline:
```bash
DEDALUS_AIRSIM_ENABLE_DEPTH_OBSTACLES=1 \
DEDALUS_AIRSIM_DEPTH_OBSTACLE_MAX_POINTS=4096 \
DEDALUS_AIRSIM_DEPTH_OBSTACLE_STRIDE=8 \
DEDALUS_AIRSIM_DEPTH_OBSTACLE_MAX_RANGE_M=30 \
DEDALUS_AIRSIM_DEPTH_OBSTACLE_MIN_RANGE_M=0.5 \
DEDALUS_AIRSIM_DEPTH_OBSTACLE_CONFIDENCE=0.8 \
simulation/airsim/run_mission.sh \
  --output-dir out/<run> \
  --merge-obstacle-map \
  --obstacle-map-site-id <site_id> \
  --obstacle-map-site-frame-id airsim_world \
  --obstacle-map-mission-id <mission_id> \
  --runtime-event-http-port 8080 \
  --runtime-event-static-root out/<run>
```

SSH browser access:
```bash
ssh -L 8090:127.0.0.1:8090 <ec2-host>
open http://127.0.0.1:8090/   # unified viewer (viewer.html default)
```

Evidence retention dry-run:
```bash
python3 tools/mission/mission-evidence-retention.py \
  --output-dir out/<run> --maps-dir maps --keep-every-n 100 --json
```

Obstacle memory manifest validation:
```bash
python3 tools/avoidance/validate_obstacle_memory_manifest.py \
  out/<run>/obstacle_memory_manifest.json \
  --site-id <site_id> --site-frame-id airsim_world \
  --mission-id <mission_id> --site-map-format sqlite
```

Git commit (FUSE lock):
```bash
rm -f /Users/titan/projects/dedalus/.git/*.lock
git add <files>
git commit -m "<message>"
git push
```

---

## Do Not

- Do not use YOLO/DETR/classifier outputs as a prerequisite for obstacle avoidance.
- Do not derive visual obstacle coverage from vehicle yaw alone.
- Do not use AirSim object-GT as the obstacle detector.
- Do not add named-object class filters to obstacle detection.
- Do not couple obstacle persistence, map-building policy, or sensing coverage to a flight command sink.
- Do not add avoidance/replanning/control behavior from persistent memory until explicitly scoped and validated.
- Do not add planner/control coupling at L3 until Stage 8 is explicitly scoped.
- Do not change L2's in-memory voxel structure without first implementing L1 OctoMap (Stage 2.5 gate).
- Do not introduce AirSim detector-side coalescing/flags (removed in 4.3A — map-level compaction owned by MissionLocalObstacleMap).
- Do not merge L0/L1/L2/L3 representations or use one layer's storage format for another's role.
- Do not name files, validators, scripts, or symbols after planning labels or temporary session shorthand.
- Do not commit build/viewer.html (built from tools/visualization/mission_unified_viewer.py; build/ is in .gitignore).
