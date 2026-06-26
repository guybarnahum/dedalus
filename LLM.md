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

**Development context:** Running in AirSim/PX4 SITL. The AirSim ground-truth depth API
(`DepthPlanar`) is being replaced by a real visual depth pipeline. GT depth remains available
as a validation oracle only — it is not the operational source. The front camera is
**gimbaled** (mission-controlled): pointing mode is set per-mission (stare-at-target,
angle-from-velocity, landing-approach). Gimbal state flows into `ObstacleSensingVolume`
every tick via encoder reading at frame timestamp (not commanded position).

```
AirSim live RGB frame + ego sidecar
  -> AirSimFrameSource (RGB; DepthPlanar disabled in production)
  -> FrameHintEgoProvider
  -> CoreStackRunner
       -> GhostTargetProvider -> GhostDetectionsPublisher
       -> SensingCoverageProvider (gimbal-aware) -> ObstacleSensingVolume (per tick)
       -> VisualDepthObstacleDetector            [replaces AirSimDepthObstacleDetector]
            -> DepthEngineInterface (ONNXDepthEngine / TensorRTDepthEngine)
                 infer_device() -> depth_relative[] (device ptr, never host-copied)
            -> project_depth_to_device_evidence() (CUDA kernel / CPU fallback)
                 ProjectionParams from ObstacleSensingVolume (gimbal-corrected R, T)
                 MetricScaleEstimate (fixed AirSim scale V0; VIO-coupled V1)
                 -> DeviceObstacleEvidence[] (POD, unified/pinned memory)
            -> fit_surface_patches_device()   -> SurfacePatch evidence (is_surface_hint)
            -> detect_thin_structures_device() -> LineSegment evidence (is_thin_structure_hint)
            -> inflate() -> ObstacleEvidence[]  (host-side string fields stamped in)
       -> PerceptionPipelineOutput.obstacle_evidence
  -> MissionLocalObstacleMap (raw evidence, per-tick)
  -> MissionMapAssimilator
       -> L1: MissionLocalTraversabilityMap (0.5 m, decay 0.05/s)
              -> traversability_map_publisher -> SSE (throttled 2 s)
       -> L2: MissionLocalPlanningMap (1 m×2 m, SQLite-backed, slide_window)
              -> MissionLocalPlanningMapPublisher -> SSE (delta, watermark)
  -> L3: LocalESDFMap (derived from L2 each tick, never saved to disk)
              -> LocalESDFMapPublisher -> SSE (esdf_delta)
  -> PerchCandidateEvaluator (non-realtime, reads L1 SurfacePatch + L2 clearance)
       -> ranked PerchCandidate list -> WorldSnapshot
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

## 3. Active Next Slice: Visual Depth + Perception

Two parallel tracks: **VD** (visual depth pipeline, replacing AirSim GT) and **P** (actor
perception, builds on VD). Planning work (Stages 7–8) is deferred; see Section 5.

### VD series — Visual Depth Pipeline (AirSim without GT)

Replace `AirSimDepthObstacleDetector` (ground-truth DepthPlanar API) with a real
vision-based depth pipeline. Development in AirSim using RGB camera only. AirSim GT
depth is available as a **validation oracle only** — run both detectors in parallel
during VD2–VD3 to measure delta, then disable GT permanently at VD4.

#### VD1 — Types and headers
New files: `visual_depth_frame.hpp`, `metric_scale_estimate.hpp`, `depth_engine.hpp`,
`depth_projection_kernel.hpp` (ProjectionParams, DeviceObstacleEvidence POD).
Add `is_surface_hint` flag to `ObstacleEvidence` in `occupancy_types.hpp`.
No engine, no inference yet.
**Milestone:** all headers compile clean; existing 44+ ctests pass unchanged.

#### VD2 — ONNXDepthEngine + CPU projection
`ONNXDepthEngine` (ONNX Runtime, CPU/MPS, Mac-compatible).
`project_depth_to_device_evidence()` CPU fallback path.
`VisualDepthObstacleDetector` class + `detect_visual_depth_obstacles()` free function.
`tools/perception/export_depth_anything.py` — exports DepthAnythingV2-Small `.onnx`
from HuggingFace checkpoint. Engine files never committed to repo.
Fixed scale from AirSim camera config (V0 scale source).
**Milestone:** static AirSim RGB frame → ObstacleEvidence grid matches AirSim GT baseline
within ±1 voxel for major obstacles (wall, building faces). Delta logged and within threshold.

#### VD3 — Surface patch + thin structure (CPU path)
`fit_surface_patches_device()` CPU path — RANSAC plane fitting on projected point cloud.
`detect_thin_structures_device()` CPU path — Sobel gradient + NMS + connected components.
No OpenCV dependency anywhere in the production path.
**Milestone:** AirSim scene containing a flat wall + a vertical pole → SurfacePatch evidence
emitted with upward-ish normal for wall; ThinStructureRisk LineSegment evidence emitted for pole.

#### VD4 — CoreStackRunner integration, GT removal
Wire `VisualDepthObstacleDetector` into `CoreStackRunner` behind
`visual_depth_engine != nullptr`. Disable `DEDALUS_AIRSIM_ENABLE_DEPTH_OBSTACLES` GT path.
`update_metric_scale(MetricScaleEstimate)` public method on `CoreStackRunner` (called by
future VIO subscriber; for now, set once from AirSim camera config).
Gimbal-aware: `ProjectionParams` populated from `ObstacleSensingVolume` encoder reading
at frame timestamp — not commanded gimbal position.
**Milestone:** Full AirSim mission runs with visual depth only. L1/L2 map builds. Viewer
shows obstacle cells and L3 ESDF. Mission completes without AirSim GT depth.

#### VD5 — PerchCandidateEvaluator
Non-realtime evaluator (runs outside the 30 Hz tick loop). Queries L1 for `SurfacePatch`
evidence with `is_surface_hint=true` and normal within threshold of world-up. Applies
flatness, minimum area (drone footprint + margin), clearance (L2 `ray_cast` upward), and
depth stability scoring. Outputs ranked `PerchCandidate` list into `WorldSnapshot`.
Viewer: perch candidates rendered as green landing-zone pads in the 3D scene.
**Milestone:** AirSim scene with a rooftop → evaluator identifies and ranks it correctly.
Clearance check via L2 ray_cast passes. Viewer shows it.

#### VD6 — CUDA kernel path (Jetson, when hardware is available)
`depth_projection_kernel.cu` — full CUDA projection + voxelization kernel.
`detect_thin_structures_device.cu` — Sobel + NMS + CC CUDA kernel.
`fit_surface_patches_device.cu` — RANSAC CUDA kernel.
`TensorRTDepthEngine` — loads `.engine` file; DLA offload optional.
`tools/perception/compile_depth_engine.sh` — `trtexec` INT8 calibration pipeline.
cudaMallocManaged buffers: zero-copy on Jetson unified memory by construction.
**Milestone:** ≥ 30 Hz end-to-end on Orin Nano. Total latency (infer + project + surface +
thin structure) ≤ 20 ms at 30 Hz. No host-device copy of depth buffer.

#### VD7 — VIO scale coupling (deferred)
`MetricScaleEstimator`: velocity + feature tracking → `MetricScaleEstimate` at ~50 Hz.
Altitude fallback when VIO invalid. Deferred until VD1–VD5 stable.

---

### P series — Actor Perception (after VD2 provides the depth foundation)

#### P1 — Actor detection baseline
`ActorObservation` type in `PerceptionPipelineOutput` (separate from `ObstacleEvidence`).
Do NOT feed actors into L2. Backend for AirSim: depth-based clustering on visual depth
output (not GT). Long term: lightweight YOLOv8n or DETR.

#### P2 — Actor tracking
`ActorTrack`: id, class, position, velocity, heading, last_seen_ns, confidence.
Kalman / constant-velocity predictor. Lifecycle: init → update → age-out (2 s).
`actor_tracks` SSE event. Viewer: labeled dots + velocity vectors.

#### P3 — Exploration planner (curiosity flight)
`FrontierMap` from L2. `ExplorationPlanner` → next waypoint ranked by information gain + cost.
New mission step type: `ExploreStep`. Plugs into existing `BehaviorSpec` runtime.

#### P4 — Tag / chase / inspect behavior
`ApproachActorStep`, `OrbitActorStep`, `InspectActorStep`.
Each closes loop on `ActorTrack` position. Gimbal `CameraPointingCommand` stares at target.

#### P5 — Multi-drone coordination (future, not started)
Peer discovery, map-partition, actor-ownership protocol.
Shared L2 DB merge: `tools/mission/merge_l2_maps.py`. Out of scope until P1–P4 stable.

---

## 4. Architecture Boundaries

```
WorldSnapshot              autonomy state (includes PerchCandidate list)
PerceptionPipelineOutput   evidence + actor observations
ObstacleEvidence           static geometry evidence → feeds L1/L2
  SurfacePatch shape       flat surface hit; is_surface_hint=true; not yet "landable"
  LineSegment shape        thin obstacle (wire, pole); is_thin_structure_hint=true
ActorObservation           dynamic agent evidence → feeds actor tracker, NOT L2
Ghost detections           enter through same Observation3D path as real detections
Artifacts                  evidence/debug outputs, not IPC
Overlay                    subscriber/renderer only
L0                         reflexive avoidance — no planner coupling
L3                         planning primitive — no flight command coupling until Stage 7 scoped
DepthEngineInterface       platform-transparent depth inference (ONNX/TensorRT)
DeviceObstacleEvidence     GPU-side POD intermediate; never a stored type; inflated to ObstacleEvidence
ProjectionParams           kernel input — all POD, no host ptrs; populated from ObstacleSensingVolume
MetricScaleEstimate        single global scale factor (V0: fixed from AirSim config; V1: VIO-coupled)
PerchCandidateEvaluator    non-realtime; reads L1 + queries L2; outputs to WorldSnapshot
```

Do not:
- Feed dynamic actor evidence into L2 (actors are not static obstacles).
- Mark SurfacePatch evidence as "landable" in the detector — that is a behavior tree semantic.
- Suppress APF repulsion in the detector — repulsion override during landing is a BT gain decision.
- Use the commanded gimbal position for ProjectionParams — always use the encoder reading at frame timestamp.
- Add planner blocking, replanning, or command-sink avoidance unless explicitly scoped.
- Use YOLO/DETR outputs as prerequisite for *obstacle avoidance* (fine for actor ID).
- Couple obstacle persistence, map-building, or sensing coverage to a flight command sink.
- Commit `.onnx` or `.engine` model files to the repo — generate on target hardware.
- Add OpenCV as a production dependency — use CUDA kernels or pure C++ alternatives.
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
