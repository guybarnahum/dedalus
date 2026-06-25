# Dedalus Current Handoff

You are continuing work on the Dedalus repo.
Repository: `guybarnahum/dedalus`

First read: **LLM.md** (goals, current state, architecture, next slice)
Then read: `docs/two-level-obstacle-map.md` for L0–L3 detail.
Historical context only if needed: `LLM.back.md`

Run `git log --oneline -5` to confirm current commit before starting work.

---

## Active Development State

### Obstacle map stack — COMPLETE

All stages implemented, committed, tested.

```
Stage 1  L2 SQLite persistence       DONE — SQLite + WAL + dirty-cell flush (10 s)
Stage 2  Bounded in-memory L2        DONE — slide_window() ±150 m, wired in run_once()
Stage 2.5 L2 planning API            DONE — ray_cast + query_occupied_in_box
Stage 3  L3 EDT compute              DONE — Meijster 3D, sparse shell cells, signed field
Stage 4  L3 incremental updates      DONE — update_incremental() + update_tube() in tick loop
Stage 5  Incremental L2 SSE          DONE — write_seq watermark, delta publish wired
Stage 6  L3 SSE + viewer             DONE — esdf_delta events, arrows, net rep in radar inset
```

L3 is always recomputed from L2. It is never saved to disk.
`esdf_persistence_path` in CoreStackRunnerConfig is intentionally unwired.

### Viewer — COMPLETE

`tools/visualization/mission_unified_viewer.py` → `build/viewer.html`

All layers rendering: L0 TTC radar + cone scope, L1 trav surface, L2 planning cells
(age-dimmed), L3 ESDF gradient arrows + amber net-repulsion vector in radar inset.
Age dimming: 0–2 s full brightness, 2–10 s linear decay to 80%.
L3 lattice: R-spaced Z layers, alternating (R/2, R/2) XY offset on odd layers.
DB/live ESDF merge: DB baseline preserved through live full snapshots (map_seq=0 gate).
Contract validator: 0 violations.

### Deferred planning work

```
Stage 7  Navigation function + trajectory optimizer   DEFERRED — see LLM.md §5
Stage 8  L0/L3 calibration (sim run)                 DEFERRED — blocked on Stage 7
```

---

## Next Active Slice: Perception

See LLM.md §3 for full plan. Summary:

```
P1  Actor detection baseline
      ActorObservation type in PerceptionPipelineOutput (separate from ObstacleEvidence).
      Do NOT feed actors into L2. Backend TBD: AirSim GT oracle → depth clustering → vision model.

P2  Actor tracking
      ActorTrack: id, class, position, velocity, heading, last_seen_ns, confidence.
      Kalman / constant-velocity predictor. Lifecycle: init → update → age-out (2 s).
      actor_tracks SSE event. Viewer: labeled dots + velocity vectors.

P3  Exploration planner
      FrontierMap from L2. ExplorationPlanner → next waypoint.
      New mission step type: ExploreStep.

P4  Tag / chase / inspect behavior
      ApproachActorStep, OrbitActorStep, InspectActorStep.
      Close loop on ActorTrack position. Plugs into existing BehaviorSpec runtime.

P5  Multi-drone coordination (future, not started)
```

---

## Four-Level Obstacle Map Architecture (reference)

```
L0  LocalFlightMapSnapshot        ego-local, 0.5 m cells, rebuilt every tick
      Polar risk + sensor obs: compute_l0_polar_risk() + collect_l0_sensor_observations()
      Viewer: TTC radar inset (top-right) + cone scope inset (top-left)
      Role: reflexive avoidance, TTC, escape vector. No planner coupling.

L1  MissionLocalTraversabilityMap  per-flight accumulator, 0.5 m, decay 0.05/s
      Feeds L2 via update_from_traversability() each tick.
      Viewer: exterior voxel face surface (LOD, throttled 2 s).

L2  MissionLocalPlanningMap        persistent site map, 1 m × 1 m × 2 m voxels
      SQLite + WAL. slide_window(±150 m). ray_cast + query_occupied_in_box.
      Delta SSE streaming (write_seq watermark).
      DB path: maps/$DEDALUS_SITE_ID/l2_map.db (auto-derived in mission loop).

L3  LocalESDFMap                   ESDF computed from in-memory L2 window every tick
      Meijster 3D EDT. Sparse hash map (shell cells |d| < d0 = 5 m).
      Signed field; gradient ∇d precomputed per shell cell.
      update_incremental() on dirty L2 cells; full recompute on slide_window().
      SSE: esdf_delta (full on startup, delta on incremental change).
      NOT saved to disk — always recomputed from L2 (recompute ~6 ms).
```

Representation boundaries — keep these distinct:
- `ObstacleEvidence` — static geometry evidence, feeds L1/L2
- `ActorObservation` — dynamic agent evidence, feeds tracker ONLY (not L2)
- `LocalFlightMapSnapshot` (L0) — current-hazard working set, ego-local
- `MissionLocalTraversabilityMap` (L1) — per-flight filtered accumulator
- `MissionLocalPlanningMap` (L2) — cross-mission persistent store
- `LocalESDFMap` (L3) — planning primitive, derived from L2, not persisted

---

## Runtime Commands

Build and test (on Linux EC2 — Mac sandbox does not have cmake/ctest in PATH):
```bash
cmake --build build-staging -j$(nproc)
ctest --test-dir build-staging --output-on-failure
```

Generate and validate viewer (Mac or Linux):
```bash
python3 tools/visualization/mission_unified_viewer.py
cp build/viewer.html build-staging/viewer.html
python3 tools/validation/validate-mission-unified-viewer.py build/viewer.html
```

Start dedalus_viewer sidecar (live mode):
```bash
DEDALUS_SITE_ID=airsim_47.641N_122.140W \
./build-staging/apps/dedalus_viewer \
  --host 127.0.0.1 --port 47770 \
  --http-port 8090 --static-root build-staging
# L2 DB auto-derived: maps/$DEDALUS_SITE_ID/l2_map.db
```

Start AirSim mission:
```bash
DEDALUS_SITE_ID=airsim_47.641N_122.140W \
DEDALUS_AIRSIM_ENABLE_DEPTH_OBSTACLES=1 \
simulation/airsim/run_mission.sh \
  --config config/core_stack_object_behavior_airsim_existing_object_circle.yml \
  --runtime-event-http-port 8080 \
  --runtime-event-static-root build-staging
```

SSH browser access:
```bash
ssh -L 8090:127.0.0.1:8090 <ec2-host>
open http://127.0.0.1:8090/
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

- Feed dynamic actor evidence into L1 or L2 (actors are not static geometry).
- Use YOLO/DETR as a prerequisite for obstacle *avoidance* (fine for actor identification).
- Add planner/control coupling at L3 until Stage 7 is explicitly resumed.
- Save L3 to disk — always recompute from L2.
- Derive visual obstacle coverage from vehicle yaw alone.
- Couple obstacle persistence or map-building to a flight command sink.
- Change L2's in-memory voxel structure without first implementing L1 OctoMap (gate).
- Merge L0/L1/L2/L3 representations.
- Commit build/viewer.html (built from tools/visualization/mission_unified_viewer.py).
- Name files or symbols after planning labels or temporary session shorthand.
