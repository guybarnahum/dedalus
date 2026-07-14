# Dedalus Current Handoff

You are continuing work on the Dedalus repo.
Repository: `guybarnahum/dedalus`

First read: **LLM.md** (goals, current state, architecture, next slice)
Then read: `docs/two-level-obstacle-map.md` for L0–L3 detail.
Historical context only if needed: `LLM.back.md`

Run `git log --oneline -5` to confirm current commit before starting work.

---

## ⚠ Never Guess — Always Verify From Code

This rule is absolute. Before using any env var, CLI flag, config key, function signature,
struct field, CMake option, or script argument, **read the source that defines it**.

| What you need | Where to verify |
|---|---|
| Env var name | `grep 'getenv\|env_str_or' src/**/*.cpp` — or see §Runtime Env Var Reference in LLM.md |
| CLI flag | Read the `add_argument` / `CLI11` block in the relevant `apps/*.cpp` |
| Config key | Read `apply_config_value()` in `src/runtime/config_loader.cpp` |
| CMake option | Read the top-level `CMakeLists.txt` and `src/CMakeLists.txt` |
| Script flag | Read the `getopts` / argparse block in the wrapper `.sh` or `.py` |
| Struct field / function sig | Read the header in `include/dedalus/` |

A wrong env var is **silently ignored** and burns a full mission run to diagnose.
`warn_unknown_dedalus_vars()` (config_loader.cpp) now prints a startup warning for
unknown `DEDALUS_` vars, but that requires a rebuilt binary — do not rely on it as a
substitute for checking the code first.

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

Interaction: plain drag → pan (translates viewCenter in view-aligned NED world space);
shift+drag → rotate (yaw/pitch). y-origin at 0.80*canvas.height. Bounds use trimmed-mean
(10% outlier cut per axis). L1 LOD slider changes display resolution only — does not rescale
the scene (trav cells are included in recomputeBounds; both snapshot and delta handlers call it).

### Recent fixes (post-VD4 contract refactor)

```
H19.21a  L0 sensor-obs cone truncation — FIXED
         collect_l0_sensor_observations() called with default max_obs=256. Evidence is stored
         row-major (top elevation → bottom), so 256 entries covered only the top ~6 grid rows
         (positive elevation only), leaving the lower half of the cone empty in the viewer.
         Fix: pass evidence.size() at the call site in core_stack_runner_run.cpp.

H19.21b  AirSim overlay elevation bias — FIXED
         obstacle_evidence_from_snapshot() used evidence[:n] (head-slice). Same row-major
         order → only upper-elevation markers injected into the AirSim scene.
         Fix: stride-subsample (evidence[::stride][:n]) in airsim-world-overlay.py.

H19.21c  AirSim overlay flags split
         WITH_SENSING_EVIDENCE_OVERLAY split into two independent vars in run_mission.sh:
           WITH_SENSING_VOLUMES_OVERLAY=0   (camera FOV cone frustums)
           WITH_OBSTACLE_EVIDENCE_OVERLAY=1 (depth-projected evidence tiles)
         New CLI flags: --[no-]sensing-volumes-overlay, --[no-]obstacle-evidence-overlay
         Old --[no-]sensing-evidence-overlay kept as backward-compatible alias (sets both).
         YAML config keys: with_sensing_volumes_overlay, with_obstacle_evidence_overlay
```

### Deferred planning work

```
Stage 7  Navigation function + trajectory optimizer   DEFERRED — see LLM.md §5
Stage 8  L0/L3 calibration (sim run)                 DEFERRED — blocked on Stage 7
```

---

## Development Context: AirSim Without Ground Truth

**AirSim GT depth is replaced by visual algorithms.** `DepthPlanar` API is the validation
oracle only — not the operational source. Use `depth_eval: airsim_gt` (or `DEDALUS_DEPTH_EVAL=airsim_gt`)
to keep it as the reference slot. `DEDALUS_AIRSIM_ENABLE_DEPTH_OBSTACLES` is a dead env var — never
checked by the runtime; ignore it.

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

VD4  Provider contract + shared pipeline (IMPLEMENTED)
       Provider responsibility: resolve intrinsics + build DepthPipelineInput.
       run_depth_pipeline() owns all downstream: kernels + CUDA + inflate(source_kind) + bearing/elevation.
       Uniform ObstacleEvidenceProvider base: last_inverse_depth/last_params/last_pitch_deg.
       Runner: fill_panel lambda with base-class accessors; no typed casts.
       AirSim GT kept as validation oracle; not disabled — switchable via slot A/B config.
       Milestone: Slot A + Slot B running simultaneously. Agreement logged per frame.

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
  depth_projection_kernel.hpp  ProjectionParams (POD), DeviceObstacleEvidence (POD),
                               DepthPipelineInput, DepthPipelineConfig,
                               run_depth_pipeline(), metric_to_inverse_depth()
  obstacle_evidence_provider.hpp       ObstacleEvidenceProvider base: detect(EgoSensingFrame) +
                                        uniform introspection (last_inverse_depth/last_params/last_pitch_deg)
  visual_depth_obstacle_detector.hpp    VisualDepthObstacleDetectorConfig, VisualDepthObstacleDetector
  airsim_emulation_depth_obstacle_detector.hpp  AirSimEmulationDepthObstacleDetectorConfig,
                                        AirSimEmulationDepthObstacleDetector

src/sensing/
  onnx_depth_engine.cpp        ONNXDepthEngine (ONNX Runtime, CPU/MPS)
  depth_projection_kernel.cpp  project_depth_to_device_evidence(), fit_surface_patches_device(),
                               detect_thin_structures_device() (CPU paths);
                               run_depth_pipeline(), metric_to_inverse_depth() (shared pipeline);
                               CudaDepthDispatcher singleton (CUDA dispatch for both providers)
  visual_depth_obstacle_detector.cpp  provider-only: ONNX inference + scaled calibrated intrinsics
  airsim_emulation_depth_obstacle_detector.cpp  provider-only: GT metric depth + FoV-based intrinsics
  depth_projection_kernel.cu   CUDA kernel (VD6, Jetson only)
  thin_structure_kernel.cu     CUDA kernel (VD6, Jetson only)
  surface_patch_kernel.cu      CUDA kernel (VD6, Jetson only)
  tensorrt_depth_engine.cpp    TensorRTDepthEngine (VD6, Jetson only)

tools/perception/
  export_depth_anything.py     Export DepthAnythingV2-Small .onnx from HuggingFace
  compile_depth_engine.sh      trtexec INT8 calibration pipeline (run on Jetson)
```

Key type constraints:
- `DepthPipelineInput` — provider-filled struct: inverse_depth buffer + fully-resolved fx/fy/cx/cy + source_kind; no provider-type branching downstream
- `DeviceObstacleEvidence` — 48-byte POD, GPU-side only; inflated to `ObstacleEvidence` host-side
- `ProjectionParams` — all POD, built by run_depth_pipeline() from provider-resolved intrinsics + ObstacleSensingVolume
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

## Runtime Env Var Reference

All `DEDALUS_` env vars consumed by the C++ binary. Source of truth: `grep getenv` across `src/`.
Any unknown `DEDALUS_` var triggers a startup warning from `warn_unknown_dedalus_vars()` in `config_loader.cpp`.

| Env var | Read in | Effect |
|---|---|---|
| `DEDALUS_DEPTH` | config_loader.cpp | depth provider name (overrides YAML) |
| `DEDALUS_DEPTH_EVAL` | config_loader.cpp | depth eval provider name |
| `DEDALUS_EGO_PROVIDER` | config_loader.cpp | ego provider name (overrides YAML) |
| `DEDALUS_EGO_PROVIDER_EVAL` | config_loader.cpp | ego eval provider name |
| `DEDALUS_DETECTOR_EVAL` | config_loader.cpp | detector eval name |
| `DEDALUS_CAMERA_STABILIZER_EVAL` | config_loader.cpp | camera stabilizer eval name |
| `DEDALUS_TRACKER_EVAL` | config_loader.cpp | tracker eval name |
| `DEDALUS_IDENTITY_RESOLVER_EVAL` | config_loader.cpp | identity resolver eval name |
| `DEDALUS_PROJECTOR_EVAL` | config_loader.cpp | projector eval name |
| `DEDALUS_SITE_ID` | mission_loop + viewer | site identifier; derives L2 DB path `maps/$DEDALUS_SITE_ID/l2_map.db` |
| `DEDALUS_L2_NO_PERSIST` | dedalus_mission_loop.cpp | `1` = run L2 map in-memory only (no SQLite write) |
| `DEDALUS_DEPTH_DEBUG_DIR` | onnx_depth_engine.cpp | write per-frame PGM depth maps to this dir |
| `DEDALUS_AIRSIM_SCENE_INVENTORY` | provider_registry.cpp | path to AirSim scene inventory JSON |
| `DEDALUS_AIRSIM_GT_NEARBY_RADIUS_M` | provider_registry.cpp | AirSim GT detector search radius |
| `DEDALUS_AIRSIM_GT_MAX_OBJECTS_PER_FRAME` | provider_registry.cpp | AirSim GT max objects |
| `DEDALUS_AIRSIM_GT_STATIC_REFRESH_EVERY_N_FRAMES` | provider_registry.cpp | AirSim GT static refresh rate |
| `DEDALUS_MISSION_TRAVERSABILITY_MAP_ARTIFACT` | trav_map_artifact_writer_env.cpp | `1` = enable trav map artifact write |
| `DEDALUS_MISSION_TRAVERSABILITY_MAP_PATH` | trav_map_artifact_writer_env.cpp | trav map artifact path |
| `DEDALUS_MISSION_TRAVERSABILITY_MAP_SITE_ID` | trav_map_artifact_writer_env.cpp | site ID for trav map artifact |
| `DEDALUS_MISSION_TRAVERSABILITY_MAP_SITE_FRAME_ID` | trav_map_artifact_writer_env.cpp | site frame ID |
| `DEDALUS_MISSION_TRAVERSABILITY_MAP_MISSION_ID` | trav_map_artifact_writer_env.cpp | mission ID |
| `DEDALUS_MISSION_TRAVERSABILITY_MAP_WRITE_EVERY_UPDATES` | trav_map_artifact_writer_env.cpp | write frequency |
| `DEDALUS_MISSION_TRAVERSABILITY_MAP_MAX_CELLS` | trav_map_artifact_writer_env.cpp | max cells |
| `DEDALUS_MISSION_OBSTACLE_MAP_ARTIFACT` | obstacle_map_artifact_writer_env.cpp | `1` = enable full obstacle map artifact |
| `DEDALUS_MISSION_OBSTACLE_MAP_PATH` | obstacle_map_artifact_writer_env.cpp | full artifact path |
| `DEDALUS_MISSION_OBSTACLE_MAP_SITE_ID` | obstacle_map_artifact_writer_env.cpp | site ID |
| `DEDALUS_MISSION_OBSTACLE_MAP_SITE_FRAME_ID` | obstacle_map_artifact_writer_env.cpp | site frame ID |
| `DEDALUS_MISSION_OBSTACLE_MAP_MISSION_ID` | obstacle_map_artifact_writer_env.cpp + core_stack_runner.cpp | mission ID |
| `DEDALUS_MISSION_OBSTACLE_MAP_WRITE_EVERY_UPDATES` | obstacle_map_artifact_writer_env.cpp | write frequency |
| `DEDALUS_MISSION_OBSTACLE_MAP_DELTAS` | obstacle_map_delta_writer.cpp | `1` = enable delta write |
| `DEDALUS_MISSION_OBSTACLE_MAP_DELTAS_PATH` | obstacle_map_delta_writer.cpp | delta output path |
| `DEDALUS_MISSION_OBSTACLE_MAP_DELTAS_WRITE_EVERY_UPDATES` | obstacle_map_delta_writer.cpp | delta write frequency |

**Dead vars (do not use):** `DEDALUS_AIRSIM_ENABLE_DEPTH_OBSTACLES` — never read by runtime; silently ignored and will now trigger a startup warning.

**Mission logs:** `simulation/airsim/logs/mission_<YYYYMMDD_HHMMSS>.log` — written by `run_mission.sh`.

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
simulation/airsim/run_mission.sh \
  --config config/ci/core_stack_object_behavior_airsim_existing_object_circle.yaml \
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
