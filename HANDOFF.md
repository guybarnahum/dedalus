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

## Development Context: AirSim Without Ground Truth

**AirSim GT depth is replaced by visual algorithms.** `DepthPlanar` API is the validation
oracle only — not the operational source. `DEDALUS_AIRSIM_ENABLE_DEPTH_OBSTACLES` will be
disabled permanently after VD4.

**Front camera is gimbaled.** Mission controls pointing mode (stare-at-target, angle-from-
velocity, landing-approach). `ObstacleSensingVolume` carries gimbal-corrected extrinsics
from encoder reading at frame timestamp — not commanded position.

**Validation strategy during VD2–VD3:** run both VisualDepthObstacleDetector and the GT
AirSim detector in parallel. Log delta between their ObstacleEvidence outputs. Threshold:
major obstacles match within ±1 voxel on 90%+ of frames before GT is disabled.

---

## Next Active Slice: Visual Depth + Perception

See LLM.md §3 for full plans. Summary:

```
VD1  Types and headers
       VisualDepthFrame, MetricScaleEstimate, DepthEngineInterface,
       ProjectionParams, DeviceObstacleEvidence (POD).
       is_surface_hint added to ObstacleEvidence.
       Milestone: headers compile; all 44+ ctests unchanged.

VD2  ONNXDepthEngine + CPU projection
       ONNXDepthEngine (ONNX Runtime, Mac CPU/MPS).
       project_depth_to_device_evidence() CPU fallback.
       VisualDepthObstacleDetector + free function.
       tools/perception/export_depth_anything.py (no engine files committed).
       Fixed scale from AirSim camera config.
       Milestone: static RGB frame → ObstacleEvidence within ±1 voxel of GT baseline.

VD3  Surface patch + thin structure (CPU)
       fit_surface_patches_device() — RANSAC plane fitting. No OpenCV.
       detect_thin_structures_device() — Sobel + NMS + CC. No OpenCV.
       Milestone: wall → SurfacePatch; pole → ThinStructureRisk LineSegment.

VD4  CoreStackRunner integration, GT removal
       Wire VisualDepthObstacleDetector into CoreStackRunner.
       Disable AirSim GT depth path permanently.
       Gimbal-aware ProjectionParams from ObstacleSensingVolume.
       Milestone: full AirSim mission, visual depth only, L1/L2/L3 build, viewer renders.

VD5  PerchCandidateEvaluator
       Non-realtime. L1 SurfacePatch + L2 clearance + stability scoring.
       PerchCandidate list in WorldSnapshot. Viewer: green landing pads.
       Milestone: AirSim rooftop identified and ranked.

VD6  CUDA kernel path (Jetson, deferred until hardware available)
       depth_projection_kernel.cu, thin structure .cu, surface patch .cu.
       TensorRTDepthEngine. cudaMallocManaged (zero-copy on Jetson).
       Milestone: ≥30 Hz on Orin Nano; total latency ≤20 ms.

VD7  VIO scale coupling (deferred — after VD1-VD5 stable)

P1   Actor detection baseline      (after VD2 provides depth foundation)
P2   Actor tracking                Kalman. actor_tracks SSE. Viewer dots + velocity.
P3   Exploration planner           FrontierMap + ExploreStep behavior.
P4   Tag / chase / inspect         ApproachActorStep, OrbitActorStep, InspectActorStep.
P5   Multi-drone coordination      Future. Not started.
```

---

## Visual Depth Component Map (reference)

```
include/dedalus/sensing/
  visual_depth_frame.hpp       VisualDepthFrame, CameraIntrinsics, LensDistortion
  metric_scale_estimate.hpp    MetricScaleEstimate (scale, confidence, age_s)
  depth_engine.hpp             DepthEngineInterface, DepthInferenceResult
  depth_projection_kernel.hpp  ProjectionParams (POD), DeviceObstacleEvidence (POD)
  visual_depth_obstacle_detector.hpp  VisualDepthObstacleDetectorConfig, VisualDepthObstacleDetector

src/sensing/
  onnx_depth_engine.cpp        ONNXDepthEngine (ONNX Runtime, CPU/MPS)
  depth_projection_kernel.cpp  project_depth_to_device_evidence() CPU fallback
                               fit_surface_patches_device() CPU path
                               detect_thin_structures_device() CPU path
  visual_depth_obstacle_detector.cpp
  depth_projection_kernel.cu   CUDA kernel (VD6, Jetson only)
  thin_structure_kernel.cu     CUDA kernel (VD6, Jetson only)
  surface_patch_kernel.cu      CUDA kernel (VD6, Jetson only)
  tensorrt_depth_engine.cpp    TensorRTDepthEngine (VD6, Jetson only)

tools/perception/
  export_depth_anything.py     Export DepthAnythingV2-Small .onnx from HuggingFace
  compile_depth_engine.sh      trtexec INT8 calibration pipeline (run on Jetson)
```

Key type constraints:
- `DeviceObstacleEvidence` — 48-byte POD, GPU-side only; inflated to `ObstacleEvidence` host-side
- `ProjectionParams` — all POD, populated fresh each frame from `ObstacleSensingVolume`
- Depth buffer (`depth_relative[]`) — device ptr only; CPU never reads it
- Model files — never committed; generated on target hardware via `export_depth_anything.py` + `compile_depth_engine.sh`

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
- Derive visual obstacle coverage from vehicle yaw alone — use ObstacleSensingVolume (gimbal-aware).
- Use commanded gimbal position for ProjectionParams — use encoder reading at frame timestamp.
- Mark SurfacePatch evidence as "landable" in the detector — landability is a behavior tree semantic.
- Suppress APF repulsion from within the detector — repulsion override during landing is a BT gain decision.
- Add OpenCV as a production dependency — use CUDA kernels or pure C++ for image processing.
- Commit `.onnx`, `.engine`, or `.plan` model files — generate on target hardware.
- Couple obstacle persistence or map-building to a flight command sink.
- Change L2's in-memory voxel structure without first implementing L1 OctoMap (gate).
- Merge L0/L1/L2/L3 representations.
- Commit build/viewer.html (built from tools/visualization/mission_unified_viewer.py).
- Name files or symbols after planning labels or temporary session shorthand.
