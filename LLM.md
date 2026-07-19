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

NEVER GUESS. This is an absolute rule with no exceptions:
  - Do not guess env var names. Always grep getenv/env_str_or in src/.
  - Do not guess CLI flag names. Always read the argparse/CLI11 block in the binary.
  - Do not guess config key names. Always read apply_config_value() or equivalent.
  - Do not guess function signatures, struct fields, or enum values. Always read the header.
  - Do not guess CMake option names. Always read CMakeLists.txt.
  - Do not guess script flag pass-through. Always read the wrapper's parser block.
  - Do not guess artifact paths, schema formats, or test expectations. Always inspect.

If you are not certain of the exact name, stop. Grep or read the source. Then proceed.
"I think it might be called X" is not acceptable — verify before stating.

A wrong env var (e.g. DEDALUS_PIPELINE_EGO instead of DEDALUS_EGO_PROVIDER) is
silently ignored and wastes an entire mission run. The cost of guessing is high.

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
       -> ObstacleEvidenceProvider (slot A primary / slot B reference)
            VisualDepthObstacleDetector — ONNX inference + scaled calibrated intrinsics
                 -> DepthEngineInterface (ONNXDepthEngine / TensorRTDepthEngine)
                      infer_device() -> depth_relative[] (device ptr, never host-copied)
            AirSimEmulationDepthObstacleDetector — GT metric depth + FoV-based intrinsics
            Each provider → DepthPipelineInput (fully-resolved fx/fy/cx/cy + inverse_depth)
            → run_depth_pipeline() [depth_projection_kernel.cpp]
                 ProjectionParams from provider-resolved intrinsics + ObstacleSensingVolume axes
                 MetricScaleEstimate (fixed AirSim scale V0; VIO-coupled V1)
                 -> project_depth_to_device_evidence() (CPU path; full CUDA pending VD6)
                      -> DeviceObstacleEvidence[] (POD, unified/pinned memory)
                 -> fit_surface_patches_device()   -> SurfacePatch evidence (is_surface_hint)
                 -> detect_thin_structures_device() -> LineSegment evidence (is_thin_structure_hint)
                 -> inflate(source_kind) -> ObstacleEvidence[]  (host-side string fields stamped in)
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
       -> collect_l0_sensor_observations(snap.local_flight_map, snap.obstacle_evidence, vel_body,
                                         snap.obstacle_evidence.size())
          (max_obs must be explicit — default 256 covers only top ~6 grid rows of row-major evidence)
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
       Renders: L0 TTC radar + cone scope, L1 trav surface (LOD slider — display-only,
                does not rescale scene), L2 planning cells, L3 ESDF arrows + net repulsion,
                obstacle evidence, ghost detections, trajectory, sensing volumes, mission
                event log, orientation gizmo, takeoff marker.
       Interaction: drag → pan (translates viewCenter in view-aligned NED space);
                    shift+drag → rotate (yaw/pitch).
       Bounds: trimmed-mean (10% outlier cut per axis); y-origin at 0.80×canvas height.
  -> tools/validation/validate-mission-unified-viewer.py (0 violations gate)
```

---

## 3. Active Next Slice: Visual Depth + Perception

Three parallel tracks: **VD** (visual depth), **VL** (visual localization — camera-only ego), and **P** (actor perception). Planning work (Stages 7–8) is deferred; see Section 5.

### Visual Depth Diagnosis (as of 2026-07-16)

The ONNX monocular depth model (DepthAnything V2 ViT-S metric) achieves only **11% voxel recall** against AirSim GT across a full circle mission. Voxel size sweep (0.5m → 2m → 5m → 15m) showed completely flat recall — confirming that ONNX and GT evidence are consistently >15m apart in 3D. This is a total domain gap, not a scale or noise issue that can be fixed by pipeline parameters.

Root cause: DepthAnything V2 metric small is trained on ground-level imagery. At aerial 52° downward pitch, the model compresses 15–30m GT geometry into the 5–15m output range. The block-minimum sampler (276 px/cell for ONNX vs 36 px/cell for GT) amplifies single-pixel noise, further displacing evidence angular positions.

**Two distinct failure modes:**
1. **Block-sampler noise** — single close pixel in 276-px ONNX block dominates evidence position. Fix: p5-percentile block sampler in `depth_projection_kernel.cpp` lines 215–268 (pending).
2. **Model domain gap** — scale compression 2-3× due to no aerial training data. Fix: fine-tune or replace model.

**Architecture audit finding (2026-07-16):**
L0 and L1 currently share the same sensing source via `MissionLocalObstacleMap`. Both layers are downstream of the 264ms ONNX inference. `detect_thin_structures_device` uses depth gradient (not absolute depth), so it is quality-resilient — a fence at 20m with ONNX placing it at 8m still generates a thin-structure event if the depth discontinuity is sharp. However, its 3D localization inherits the scale error, and it still waits for the full ONNX inference. L0 does not have an independent fast sensing path today.

**Depth improvement plan (VD series additions):**

| Step | What | Result / Expected recall | Effort |
|---|---|---|---|
| VD4a | p5-percentile block sampler (depth_projection_kernel.cpp L215-268) | **DONE** — 11.1% → 11.5% (not root cause) | low |
| VD4b | AGL scale anchoring | **SKIPPED** — simulation shows direction error, not scale error | low |
| VD4c | UniDepth V2 ViT-S with known intrinsics (fx=141.6, fy=141.2) — ONNX drop-in | **DONE** — scale p50=1.002×; per-frame F1 p50=23.4% (domain gap) | medium |
| VD4e | Log-odds probabilistic L1 accumulation — ray-casting multi-frame fusion | +20–30 pp over VD4c (estimate) | high |
| VD4d | Fine-tune DepthAnything V2 or UniDepth V2 on (RGB, GT depth) pairs from profiler runs | ~60–80% est. | high |
| VD8  | Decouple L0 from ONNX latency — dedicated proximity path (<50ms) | — | future |

### EgoStateProvider — first-class A/B provider (IMPLEMENTED)

`EgoStateProvider` follows the same pattern as every other pipeline stage:

| Aspect | Detail |
|---|---|
| Primary config key | `ego_provider: frame_hint` (YAML) |
| Env var override | `DEDALUS_EGO_PROVIDER` |
| Slot B eval config | `ego_provider_eval: frame_hint` |
| Slot B env var | `DEDALUS_EGO_PROVIDER_EVAL` |
| Agreement metric | `ego_provider.slot.agreement_ppt` — position distance, 1000=same, 0=≥1 m |
| Runner field | `ego_provider_reference_` (null = inactive) |
| Valid names | `frame_hint`, `no_telemetry`, `airsim`, `visual_odometry` (VL1, pending) |

Typical usage once VL1 lands:
```yaml
ego_provider: visual_odometry        # camera-only primary
ego_provider_eval: frame_hint        # AirSim telemetry as reference oracle
```
Agreement metric then measures VO drift vs AirSim ground truth per frame.

### VL series — Visual Localization (camera-only ego, no external sensors)

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

#### VD4 — Provider contract + shared pipeline (IMPLEMENTED)

Provider responsibility is strictly: acquire depth data, resolve camera intrinsics, build
`DepthPipelineInput`, call `run_depth_pipeline()`. No provider-type branching downstream.

`ObstacleEvidenceProvider` base — `detect(EgoSensingFrame)` + uniform introspection:
  `last_inverse_depth() / last_depth_width() / last_depth_height() / last_params() / last_pitch_deg()`
  Runner uses base-class accessors; no typed casts or `dynamic_cast` in run loop.

`VisualDepthObstacleDetector`:
  ONNX inference → scaled calibrated intrinsics (`fx_cal * inferred.width / image.width`).
  `DepthPipelineInput.source_kind = VisualObstacleDetector`. Diagnostics kept in detect().

`AirSimEmulationDepthObstacleDetector`:
  GT metric depth → `metric_to_inverse_depth()` → FoV-based intrinsics (`cx / tan(hfov/2)`).
  `DepthPipelineInput.source_kind = AirSimGroundTruthVisualEmulation`.
  Not disabled — kept as validation oracle; switchable via slot A/B config.

`DepthPipelineInput` — provider-filled struct:
  `inverse_depth[]` (disparity convention, HIGH=CLOSE), `width/height`, `scale`,
  `fx/fy/cx/cy`, `k1/k2`, `source_kind`, `sensor_name`, `source_provider`.

`run_depth_pipeline()` [depth_projection_kernel.cpp] — shared downstream for both providers:
  Builds `ProjectionParams` from provider-resolved intrinsics + sensing volume axes.
  CUDA dispatch (both providers now covered) for all 3 kernels.
  `inflate(source_kind)` → bearing/elevation loop → `ObstacleEvidence[]`.

`metric_to_inverse_depth()` — shared helper replacing private `invert_gt_depth()`.
`inflate()` takes `OccupancySourceKind` explicitly (was hardcoded `VisualObstacleDetector`).

`CoreStackRunner` two-slot: `depth_slot_a_` (primary → L1/L2), `depth_slot_b_` (reference,
delta-log only). Annotator block: `fill_panel` lambda via base-class accessors only.
Agreement metric: fraction of slot-A voxels with slot-B voxel within ±1 voxel.
Logged as `depth_slot.agreement_ppt` (parts per thousand) via `timing_writer_`.
`tools/perception/depth_provider_report.py` — HTML report: time-series, histogram, IoU distribution.
**Milestone:** Slot A + Slot B running simultaneously. Agreement logged per frame. Report generated.

A/B results (circle mission, UniDepth V2 ViT-S vs AirSim GT): scale p50 = 1.002× (correct); per-frame F1 p50 = 23.4% (domain gap — ViT-S trained on ground-level data). Per-frame F1 is the wrong metric for an accumulation-based system; see VD4e.

#### VD4e — Log-odds probabilistic L1 accumulation

Probabilistic ray-casting fusion across multiple depth frames from multiple viewing angles.

**Model:** Each depth observation makes two log-odds updates to L1 voxels:
- **Occupied** — voxels at the endpoint ± `depth_sigma_m` receive: `log_odds[v] += log(p_hit / (1-p_hit))` with `p_hit ≈ 0.3` (low per-frame weight; accumulation does the work).
- **Free** — every voxel the ray traverses before the endpoint receives: `log_odds[v] += log(p_free / (1-p_free))` with `p_free ≈ 0.1`.

**Multi-view convergence:** True obstacle voxels accumulate occupied evidence from all viewing angles. False-candidate voxels (only occupied along one ray) get free evidence when a crossing ray traverses them — they are cancelled. After N frames from N directions, true obstacles converge to high log-odds; artefacts decay.

**L0/L1 boundary:** L0 uses direct per-frame evidence (direction + approximate distance, ego frame). L0 does not need ray-casting — only L1 accumulates world-frame log-odds.

**Implementation delta:**
- `MissionLocalTraversabilityMap` cells: add `log_odds` float. `occupied_score = sigmoid(log_odds)` for backward-compat.
- `MissionMapAssimilator`: new `ray_cast_update(origin, endpoint, sigma_m, p_hit, p_free)` replaces direct cell increment.
- `ObstacleEvidenceProvider::confidence()` drives `p_hit`. Set `unidepth.confidence: 0.30` (was 0.75).
- WorldSnapshot L1 delta stream: carry `log_odds` alongside `occupied_score`.

**Viewer changes:**
- L1 voxel alpha = `sigmoid(log_odds)`, clamped [0, 1].
- `sigmoid(log_odds) < 0.3` → invisible (configurable `traversability_map.visibility_threshold`).
- Voxels materialize from transparent to opaque as evidence accumulates.
- Voxel centroid drifts toward the weighted-mean estimate as multi-view data refines it.

**Evaluation:** End-of-flight L1 map recall against GT occupancy grid — not per-frame voxel F1. Run a full circle mission; compare accumulated L1 map vs AirSim GT depth accumulated over the same trajectory.

**Milestone:** Full circle mission → L1 map recall against GT ≥ 50%. Viewer shows voxels materializing and drifting in real time as log-odds accumulates.

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

#### VD7 — Metric scale from VO (merged into VL2; see VL series)
`MetricScaleEstimate.scale` derived from VO velocity + monocular depth disparity.
Replaces fixed AirSim-config scale. Deferred until VL1 stable.

---

### VL series — Visual Localization (camera-only ego)

Core design goal: navigate entirely from the RGB camera without AirSim pose API,
GPS, or any external positioning sensor. `VisualEgoStateProvider` implements
`EgoStateProvider` and slots into the existing A/B eval framework.
`frame_hint` (AirSim telemetry) remains available as slot B oracle to measure
VO drift per frame via `ego_provider.slot.agreement_ppt`.

#### VL1 — VisualEgoStateProvider: frame-to-frame VO
`VisualOdometryState` — running position/orientation estimate in L2-frame NED.
Feature tracking: FAST corner detection + Lucas-Kanade optical flow (no OpenCV).
Essential matrix decomposition → delta R, delta t (direction only).
Apply `MetricScaleEstimate.scale` → metric translation per frame.
Integrate into cumulative pose, output as `EgoState`.
Config: `ego_provider: visual_odometry`.  Registered in `ProviderRegistry::ego_providers()`.
`EgoStateEstimate.confidence` = 1/(1 + cumulative_drift_estimate).
**Milestone:** AirSim straight-flight segment → VO position drift < 2 m over 30 s.
`ego_provider.slot.agreement_ppt` logged with `ego_provider_eval: frame_hint`.

#### VL2 — Scale from depth + L2 re-localization
Derive metric scale: match depth-map obstacle distances to nearest L2 occupied voxel
distances. No GPS required — L2 map itself is the metric reference.
L2 re-localization: when `EgoState.confidence` drops below threshold, run ICP-lite
against L2 occupied cells visible from current frame → correct accumulated drift.
`MetricScaleEstimate` updated at re-localization frequency (~1 Hz).
**Milestone:** VO drift < 0.5 m over a full 3-minute mission circuit. Scale estimate
converges within 10 s of first L2 re-localization hit.

#### VL3 — Uncertainty propagation + AirSim fallback
Propagate uncertainty through each VO step (covariance or lightweight scalar).
`EgoState.confidence` reflects integrated uncertainty.
In AirSim context: fall back to `frame_hint` when `confidence < 0.3` (configurable).
In real-world context: no fallback — confidence drives conservative L0 avoidance margins.
**Milestone:** Simulated feature-poor corridor → confidence drops, wider avoidance margins
activate, system stays stable.

#### VL4 — Cross-session L2 frame anchoring and active relocalization

**Design concept (conceived Jun 27 2026; see IP-13).**

Replace the static per-session `fallback_map_frame_id` origin with a dynamic frame whose
anchor is the persistent L2 map itself. The `fallback_map_frame_id` config key becomes a
human-readable site label only; the geometric origin is resolved at runtime by matching
the first depth observations against existing L2 voxels.

Key design properties (validated in design session):

- **Resolution is sufficient.** 1 m × 1 m × 2 m voxels support relocalization at UAV
  operational scales. Sub-meter accuracy is not required for avoidance or target pursuit.
- **Active disambiguation.** When the initial match is ambiguous (sparse map, repeated
  structure), the drone moves deliberately to reduce the pose hypothesis set — the same
  strategy biological agents use in uncertain environments.
- **Site-relative, cold-start safe.** The L2 origin is wherever the first session started.
  Re-anchoring to a better georeference later is a single affine DB transform. An empty
  L2 map behaves identically to the current static-frame system.
- **Elastic local correction.** Internal frame consistency matters more than external
  ground truth. When a region is revisited and displacement Δ is measured, the correction
  is applied locally with stiffness proportional to evidence density — dense regions resist
  deformation, sparse regions flex. No full pose graph required.

```
VL4a  Startup relocalization
      On open_db() with non-empty L2, project first N depth frames → candidate cloud.
      ICP-lite match against L2 occupied cells within search radius.
      If match score > threshold: adopt T_new→L2 as session pose offset.
      If match score < threshold: feed ambiguity signal to exploration planner (P3).
      Milestone: second session on same site re-enters L2 frame within 1.5 m.

VL4b  Loop closure correction (mid-flight)
      On strong L2 re-localization match: compute Δ = observed − predicted.
      Apply Δ to running pose offset, weighted by match confidence.
      Lightweight alternative to graph-SLAM loop closure.
      Milestone: 5-minute circuit returns within 0.5 m of departure in L2 coords.

VL4c  Elastic local voxel correction
      When Δ suggests local map error (not just drift): shift L2 voxel positions
      in radius R = f(Δ) around match site, linear falloff to zero at R.
      Weight by cell mission_count (denser = stiffer).
      Milestone: corrected region aligns with re-observed geometry within 1 voxel.

VL4d  Re-anchoring API
      MissionLocalPlanningMap::re_anchor(Vec3 translation, Quat rotation):
        full DB rigid transform (UPDATE cells SET cx=..., cy=..., cz=...).
      Exposed via l2_editor.py /api/re-anchor for manual correction from viewer.
      Milestone: editor can shift entire L2 map to a new georeference origin.
```

**Dependencies:** VL2 (ICP-lite machinery), VL3 (confidence signal for disambiguation
trigger), exploration planner P3 (executes disambiguation trajectory).
**IP:** IP-13. Consider combined provisional with IP-05 and IP-03.

---

### EP series — Evaluation Pipeline (A/B reference slots for all perception stages)

A/B evaluation is a first-level design goal. Every pipeline stage has a reference slot B
that receives the same inputs as slot A but is never fed downstream. Agreement logged per frame.

#### EP1 — Perception stage evaluation slots (IMPLEMENTED)
`include/dedalus/runtime/evaluation_slot.hpp`:
  `EvaluationSlotPair<T>` — typed container for primary + reference provider.
  Agreement free functions: `detection_agreement()` (IoU > 0.5 fraction),
  `stabilizer_agreement()` (normalized ΔT ppt), `tracker_agreement()` (class-label match),
  `identity_agreement()` (label match on same track), `observation_agreement()` (< 1 m fraction).
`CoreStackRunnerConfig`: five reference slot fields (`detector_reference`, `stabilizer_reference`,
  `tracker_reference`, `identity_resolver_reference`, `projector_reference`).
`CoreStackRunner::run_once()`: for each active reference slot, runs provider on primary-slot inputs,
  computes agreement, logs as `<stage>.slot.agreement_ppt`.
Zero overhead when all reference slots are null.

#### EP2 — Unified perception slot report (planned)
Extend `tools/perception/depth_provider_report.py` → `tools/perception/perception_slot_report.py`
All active slot agreement metrics in one dashboard: time-series per stage, cross-stage correlation.

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
                           NOTE: L0 evidence currently derives from the same MissionLocalObstacleMap
                           as L1. Both layers share the ONNX depth source. True L0/L1 sensing
                           separation (VD8) requires a dedicated sub-50ms proximity path.
L3                         planning primitive — no flight command coupling until Stage 7 scoped
DepthEngineInterface       platform-transparent depth inference (ONNX/TensorRT)
DepthPipelineInput         provider-filled struct: inverse_depth + fully-resolved fx/fy/cx/cy + source_kind; no provider branching downstream
DepthPipelineConfig        shared grid/depth/voxel config consumed by run_depth_pipeline()
run_depth_pipeline()       shared downstream owned by depth_projection_kernel.cpp: kernels + CUDA + inflate(source_kind) + bearing/elevation
metric_to_inverse_depth()  shared helper: metric metres → disparity inverse_depth (HIGH=CLOSE)
DeviceObstacleEvidence     GPU-side POD intermediate; never a stored type; inflated to ObstacleEvidence
ProjectionParams           kernel input — all POD, no host ptrs; built by run_depth_pipeline() from provider-resolved intrinsics
MetricScaleEstimate        single global scale factor (V0: fixed from AirSim config; V1: VIO-coupled)
PerchCandidateEvaluator    non-realtime; reads L1 + queries L2; outputs to WorldSnapshot
```

### A/B Evaluation — First-Class Design Primitive

Every perception pipeline stage supports an optional **slot B (reference)** provider alongside slot A (primary). This is a first-level design goal, not a debug feature.

```
Stage            Slot A (primary)              Slot B (reference)           Agreement metric
───────────────  ────────────────────────────  ──────────────────────────   ────────────────────────────
EgoStateProvider EgoStateProvider              EgoStateProvider             position distance ppt of 1 m (done)
Depth provider   ObstacleEvidenceProvider      ObstacleEvidenceProvider     voxel overlap ±1 (done)
Detector         Detector                      Detector                     IoU-matched fraction > 0.5
CameraStabilizer CameraStabilizer              CameraStabilizer             normalized ΔT magnitude (ppt)
Tracker          Tracker                       Tracker                      class-label agreement on matched tracks
IdentityResolver IdentityResolver              IdentityResolver             identity label match on same track
Projector3D      Projector3D                   Projector3D                  fraction within 1 m in local frame
```

Rules:
- Slot B always receives the **same primary-slot inputs** as slot A (not slot B's own upstream outputs).
  This gives clean calibration semantics: "what would the reference model do on the same data?"
- Slot B output is **never fed downstream** — only logged.
- Agreement is logged per-frame via `timing_writer_->record_stage("<stage>.slot.agreement_ppt", <0-1000>)`.
- All reference slots default to `nullptr` — zero overhead when unused.
- `EvaluationSlotPair<T>` in `include/dedalus/runtime/evaluation_slot.hpp` is the typed container.
  Agreement free functions live in the same header.

Implemented:
- `ego_provider_reference_` in `CoreStackRunner`; `DEDALUS_EGO_PROVIDER`/`_EVAL` env vars ✓
- `depth_slot_a_` / `depth_slot_b_` in `CoreStackRunner` (VD4) ✓
- All five perception stages: fields in `CoreStackRunnerConfig` + run loop in `run_once()` (EP1) ✓

### Provider Building-Block Contract (applies to every provider type)

Every provider — existing or future — MUST satisfy all of the following:

**1. Configurable via YAML**
```yaml
<provider_type>: <name>           # primary slot
<provider_type>_eval: <name>      # slot B (optional; "" = inactive)
```

**2. Env var overrides**
```
DEDALUS_<PROVIDER_TYPE>           # primary — overrides YAML value
DEDALUS_<PROVIDER_TYPE>_EVAL      # slot B
```
Applied in `apply_eval_env_overrides()` in `config_loader.cpp`.

**3. Validated names**
`validate_provider_names()` calls `check()` for primary and `check_opt()` for eval.
`registry.<type>s()` returns the list of valid names.

**4. Runner config fields**
`CoreStackRunnerConfig` holds a `<type>_reference` shared/unique ptr (null = inactive).
Initialized from config in `CoreStackRunner` constructor.

**5. `populate_runner_eval_slots()`**
Resolves eval name → concrete instance; stores in runner.

**6. `run_once()` evaluation**
If reference slot is non-null: run on same inputs as primary; compute agreement; log:
```cpp
timing_writer_->record_stage("<type>.slot.agreement_ppt",
    static_cast<int64_t>(agreement * 1000));
```

**7. Agreement function in `evaluation_slot.hpp`**
One `<type>_agreement(const A&, const B&) → float [0,1]` inline free function.
0 = no agreement / no data; 1 = perfect agreement.

**8. `provider_registry.hpp` comment**
`CoreStackProviderConfig` documents valid names and env vars for the provider type.

**9. Zero overhead when inactive**
Reference slot null check is the only cost when slot B is unused.

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
- Feed slot B outputs downstream or let slot B write world state.

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

## 7. Runtime Env Var Reference

Authoritative list of all `DEDALUS_` env vars consumed by the C++ runtime.
**Never guess an env var name — always verify against this table or grep `getenv("DEDALUS_` in `src/`.**
Any unknown `DEDALUS_` var triggers a startup warning (`warn_unknown_dedalus_vars()` in `config_loader.cpp`).

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
| `DEDALUS_SITE_ID` | dedalus_mission_loop.cpp, dedalus_viewer.cpp | site ID; derives L2 DB path `maps/$DEDALUS_SITE_ID/l2_map.db` |
| `DEDALUS_L2_NO_PERSIST` | dedalus_mission_loop.cpp | `1` = L2 map in-memory only (no SQLite write) |
| `DEDALUS_DEPTH_DEBUG_DIR` | onnx_depth_engine.cpp | write per-frame PGM depth maps to this dir |
| `DEDALUS_AIRSIM_SCENE_INVENTORY` | provider_registry.cpp | path to AirSim scene inventory JSON |
| `DEDALUS_AIRSIM_GT_NEARBY_RADIUS_M` | provider_registry.cpp | AirSim GT detector search radius |
| `DEDALUS_AIRSIM_GT_MAX_OBJECTS_PER_FRAME` | provider_registry.cpp | AirSim GT max objects per frame |
| `DEDALUS_AIRSIM_GT_STATIC_REFRESH_EVERY_N_FRAMES` | provider_registry.cpp | AirSim GT static refresh rate |
| `DEDALUS_MISSION_TRAVERSABILITY_MAP_ARTIFACT` | trav_map_artifact_writer_env.cpp | `1` = enable trav map artifact write |
| `DEDALUS_MISSION_TRAVERSABILITY_MAP_PATH` | trav_map_artifact_writer_env.cpp | trav map artifact path |
| `DEDALUS_MISSION_TRAVERSABILITY_MAP_SITE_ID` | trav_map_artifact_writer_env.cpp | site ID for trav artifact |
| `DEDALUS_MISSION_TRAVERSABILITY_MAP_SITE_FRAME_ID` | trav_map_artifact_writer_env.cpp | site frame ID |
| `DEDALUS_MISSION_TRAVERSABILITY_MAP_MISSION_ID` | trav_map_artifact_writer_env.cpp | mission ID |
| `DEDALUS_MISSION_TRAVERSABILITY_MAP_WRITE_EVERY_UPDATES` | trav_map_artifact_writer_env.cpp | write frequency |
| `DEDALUS_MISSION_TRAVERSABILITY_MAP_MAX_CELLS` | trav_map_artifact_writer_env.cpp | max cells |
| `DEDALUS_MISSION_OBSTACLE_MAP_ARTIFACT` | obstacle_map_artifact_writer_env.cpp | `1` = enable obstacle map artifact |
| `DEDALUS_MISSION_OBSTACLE_MAP_PATH` | obstacle_map_artifact_writer_env.cpp | artifact path |
| `DEDALUS_MISSION_OBSTACLE_MAP_SITE_ID` | obstacle_map_artifact_writer_env.cpp | site ID |
| `DEDALUS_MISSION_OBSTACLE_MAP_SITE_FRAME_ID` | obstacle_map_artifact_writer_env.cpp | site frame ID |
| `DEDALUS_MISSION_OBSTACLE_MAP_MISSION_ID` | obstacle_map_artifact_writer_env.cpp, core_stack_runner.cpp | mission ID |
| `DEDALUS_MISSION_OBSTACLE_MAP_WRITE_EVERY_UPDATES` | obstacle_map_artifact_writer_env.cpp | write frequency |
| `DEDALUS_MISSION_OBSTACLE_MAP_DELTAS` | obstacle_map_delta_writer.cpp | `1` = enable delta write |
| `DEDALUS_MISSION_OBSTACLE_MAP_DELTAS_PATH` | obstacle_map_delta_writer.cpp | delta output path |
| `DEDALUS_MISSION_OBSTACLE_MAP_DELTAS_WRITE_EVERY_UPDATES` | obstacle_map_delta_writer.cpp | delta write frequency |

**Dead vars (never read by runtime — will trigger startup warning):**
- `DEDALUS_AIRSIM_ENABLE_DEPTH_OBSTACLES` — removed; always triggers unknown-var warning now

**Mission logs:** `simulation/airsim/logs/mission_<YYYYMMDD_HHMMSS>.log` — written by `run_mission.sh`.

---

## 8. Patch Output and Safety Policy

```
- Patch scripts: concise OK:/ERROR: lines only. No grep/diff dumps after patches.
- Do not call sys.exit(), raise SystemExit, shell exit from generated patch snippets.
- Do not rely on set -e for patch control flow.
- Do not assume out/, generated artifacts, runtime logs, or validation dirs exist
  inside patch logic unless explicitly provided for that patch.
- If anchors do not match, print ERROR and do not write partial changes.
```
