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

---

## H19. Visual Depth Perception Pipeline — Locked Design Decisions

Locked during design session (2026-06-24). These decisions close the architecture for VD1–VD5; do not relitigate without explicit scope.

### H19.1 AirSim-Without-GT Development Context

`DEDALUS_AIRSIM_ENABLE_DEPTH_OBSTACLES` is disabled in operational configuration. The AirSim `DepthPlanar` ground-truth API is the **validation oracle only** — not the operational depth source. It remains runnable in parallel with `VisualDepthObstacleDetector` during VD2–VD3 for delta-logging. GT is permanently disabled after VD4 passes the 90%-within-±1-voxel threshold.

### H19.2 Gimbaled Camera Architecture

The front camera is gimbaled — mission controls pointing mode per-flight:

| Mode | Gimbal state | Coverage role |
|---|---|---|
| Stare-at-target | Locked on track | Actor follow/circle |
| Angle-from-velocity | Yaw-aligned ± tilt | Forward obstacle sensing |
| Landing-approach | Tilted down | Floor / landing zone |
| Fixed-forward | Boresight | Default |

**Critical constraint:** `ObstacleSensingVolume` carries **encoder reading at frame timestamp**, not commanded gimbal position. Actuator lag is 10–40 ms, causing up to 40 cm projection error into a 50 cm voxel grid. `ProjectionParams` must be populated from the encoder-stamped extrinsics — never from commanded position.

### H19.3 Depth Estimation Paradigm

Monocular depth estimation (MDE) selected for V0. Stereo and SfM are deferred.

Rationale:
- Single gimbaled front camera — no stereo baseline without hardware change.
- SfM requires sufficient parallax; hovering or slow approach provides little.
- MDE (DepthAnythingV2-Small) runs at >30 fps on Jetson Orin Nano INT8 and on Mac dev via ONNX Runtime.

**Model:** DepthAnythingV2-Small (25M params, ViT-S backbone).
**Export:** `tools/perception/export_depth_anything.py` — exports `.onnx` from HuggingFace weights.
**Compilation:** `tools/perception/compile_depth_engine.sh` — runs `trtexec` INT8 calibration on target hardware.
**Constraint:** `.onnx`, `.engine`, `.plan` files are **never committed**. Generated on target.

### H19.4 Scale Alignment

Single global scale factor, fixed from AirSim camera config, is acceptable for V0. Rationale: L1 map persistence covers peripheral geometry from prior frames. Single scale avoids per-region complexity without measurable mission impact at this stage.

`MetricScaleEstimate { float scale; float confidence; float age_s; }` — populated once at startup from config. VIO-coupled scale is deferred to VD7.

### H19.5 Zero-Copy Memory Architecture

Depth buffer (`depth_relative[]`) lives in device memory only. Host never reads it.

Two-stage pipeline:
1. `DeviceObstacleEvidence` — 48-byte POD struct, GPU-side intermediate. No `std::string`, no dynamic allocation. All CUDA kernels emit this type.
2. `inflate()` — host-side function. Stamps `std::string` fields (sensor_name, source_provider, map_frame_id) from compile-time constants. Outputs `ObstacleEvidence[]`.

`ProjectionParams` — all-POD struct. Populated fresh each frame from `ObstacleSensingVolume`. Passed to kernels by value.

On Jetson: `cudaMallocManaged` provides unified memory — zero physical copy on device→host transition.
On discrete GPU (dev): only the compact POD result array crosses the bus (≤49 KB for 1024 evidence points). Depth buffer (1.07 MB) stays on device.

### H19.6 No OpenCV in Production Path

All image processing uses CUDA kernels or plain C++. No `cv::Mat` allocations on the hot path.

Thin structure detection replaces `cv::createLineSegmentDetector` with a custom CUDA kernel (~300 lines):
1. Sobel gradient (Gx, Gy) per pixel.
2. Non-maximum suppression along gradient direction.
3. Hysteresis thresholding (strong / weak edges).
4. Connected components → `LineSegment` candidates.
5. Filter by aspect ratio (length >> width).

Emits: `ObstacleEvidenceShape::LineSegment`, `ObstacleEvidenceState::ThinStructureRisk`, `is_thin_structure_hint=true`.

### H19.7 Surface and Perch Detection

`fit_surface_patches_device()` runs RANSAC plane fitting on the projected point cloud in the same CUDA stream as the projection kernel (no re-inference). Emits `ObstacleEvidenceShape::SurfacePatch`, `is_surface_hint=true`.

**`is_surface_hint` not `is_landable_hint`** — landability is a behavior tree semantic, not a detector output.
**Repulsion override during landing** — APF gain suppression for below-horizon sector is a BT gain decision, not detector logic. Do not suppress repulsion from within any detector.

### H19.8 PerchCandidateEvaluator

Non-realtime evaluator. Reads L1 `SurfacePatch` accumulation + L2 `ray_cast` for clearance above each candidate. Scores: flatness, area, roughness, clearance. Outputs ranked `PerchCandidate` list to `WorldSnapshot`.

Does not run in the tick loop. Triggered on behavior tree request or on a configurable interval.

### H19.9 No Rectify-Then-Infer

Training and inference operate on raw fisheye frames. Distortion is absorbed into network weights. `LensDistortion` struct is used only in the unproject step of `project_depth_to_device_evidence()` — not as a preprocessing step before inference.

### H19.10 `ObstacleEvidence` Extensions

`is_surface_hint bool` added alongside the existing `is_thin_structure_hint bool`. No other fields added at VD1. `is_landable_hint` is explicitly rejected — use `is_surface_hint` and let the BT evaluate landability from `PerchCandidate` scoring.

### H19.11 VD Stage Definitions

| Stage | Deliverable | Milestone gate |
|---|---|---|
| VD1 | Types and headers (no impl) | All 44+ ctests pass unchanged |
| VD2 | ONNXDepthEngine + CPU projection | Static frame → evidence within ±1 voxel of GT baseline |
| VD3 | Surface patch + thin structure (CPU) | Wall → SurfacePatch; pole → ThinStructureRisk LineSegment |
| VD4 | CoreStackRunner integration; GT disabled | Full AirSim mission, visual depth only, L1/L2/L3 build |
| VD5 | PerchCandidateEvaluator | AirSim rooftop identified and ranked in viewer |
| VD6 | CUDA kernels + TensorRT (Jetson) | ≥30 Hz on Orin Nano; total latency ≤20 ms |
| VD7 | VIO scale coupling | Deferred — after VD1–VD5 stable |

### H19.12 Validation Strategy

VD2–VD3: `VisualDepthObstacleDetector` and `AirSimDepthObstacleDetector` run in parallel. Delta between their `ObstacleEvidence` outputs is logged. Gate: major obstacles match within ±1 voxel on ≥90% of frames before GT is disabled at VD4.

VD4+: `DEDALUS_AIRSIM_ENABLE_DEPTH_OBSTACLES` is permanently off in `CoreStackRunner`.

### H19.13 Metric Model Output Convention (locked, d750ceb)

DepthAnythingV2-Metric-Outdoor graph tail is `Sigmoid → Mul(×80)`. Output is **linear metric depth in metres, HIGH=FAR** in [0, 80m].

The pipeline convention is **inverse_depth, HIGH=CLOSE** (disparity). All engines convert at inference time:
```
inverse_depth = scale / raw        (scale=1.0 → 1/raw)
depth_m = ProjectionParams.scale / inverse_depth = raw  ✓
```

`scale ≈ 1.0` kept in engine. Calibrated per-scene scale lives in `MetricScaleEstimate` (YAML: `visual_onnx.scale`).

TRT path had a silent bug (raw metres stored directly as inverse_depth). Fixed in d750ceb by adding `metric_depth{true}` + `scale{1.0}` fields to `TensorRTDepthEngineConfig` and implementing the same conversion as the ONNX engine. This was a production bug (Jetson uses TRT, dev uses ONNX Runtime).

Sky pixels saturate at ~80m → stored as `inverse_depth ≈ 0.0125` → raw < 1cm threshold → stored as 0 (invalid). They are filtered by `max_depth_m=60.0` in the projection step.

### H19.14 N×M Block-Minimum Grid Sampling (locked, b7233a6)

**Problem:** stride=4 on 518×518 gives ~0.30m sample spacing at 30m. With `voxel_size_m=1.5m`, adjacent samples collapsed into the same voxel → only 10–50 unique evidence from 16,900 sampled pixels. GT ran on a 640×360 map pre-filled at stride=16 → 40×22=880 samples → no dedup problem.

**Solution:** Replace `pixel_stride` with `depth_grid_cols` × `depth_grid_rows`. Divide the depth map into N×M cells; project the pixel with the highest inverse_depth (= closest obstacle) in each cell. One evidence point per cell — no cross-cell voxel deduplication.

**Chosen values: N=40, M=22** (16:9). Matches AirSim bridge stride=16 on 640×360 exactly — each 16×16 block has exactly one GT sample → both providers produce 40×22=880 evidence with identical FOV coverage. `voxel_size_m` returned to 0.5m (the workaround coarsening to 1.5m is no longer needed). `max_evidence` raised to 1024 (880 + patches + thin headroom).

**GT equalization:** `airsim_gt_vd` was constructed with no config (always C++ defaults). Now receives config mirrored from `visual_onnx_config` in `make_depth_provider()` — grid, depth range, voxel size, max_evidence, surface/thin flags — so both providers are always in lockstep.

**Future:** to increase to 80×45, change the AirSim bridge `--depth-stride` from 16 → 8 and update `visual_onnx.depth_grid_cols/rows` in YAML. No C++ changes required.

### H19.15 GPU Projection Delegates to CPU (locked, b7233a6)

`CudaDepthDispatcher::project()` now calls `project_depth_to_device_evidence()` on the CPU rather than launching the `project_depth_kernel` CUDA kernel.

**Why:**

1. **Kernel architecture mismatch.** The old kernel did one thread per strided pixel (independent writes). Block-minimum requires a reduction across a pixel block before writing output — fundamentally different kernel shape (shared-memory reduction per block). Adapting the existing kernel would mean a rewrite, not a tweak.

2. **Workload is trivially small.** Old: 16,900 strided pixels. New: 880 cells × ~130 pixels each = ~114,400 multiply-adds. CPU does this in < 1ms; PCIe round-trip alone would exceed that.

3. **GPU stream is better used elsewhere.** `detect_thin` (Sobel on full 518×518) and `fit_patches` (RANSAC) are the legitimate GPU workloads — dense parallel operations that justify the transfer. Keeping `project()` on CPU avoids serialization stalls on the single CUDA stream.

**Reopen if:** grid grows beyond ~80×45 (3,600 cells) or multi-scale grids are added. At that point, write a proper reduction kernel — one thread block per grid cell, shared-memory min-depth reduction. Until then, CPU projection is faster end-to-end.

### H19.16 Surface Normals Deferred from L0 (decision)

`ObstacleEvidence` already carries `has_surface_normal` / `surface_normal_local`, populated by:
- `fit_surface_patches_device` → RANSAC plane normal (unit vector toward camera)
- `detect_thin_structures_device` → unit direction vector along the structure axis

`inflate()` wires these through faithfully. All three providers (`airsim_gt_detector`, `airsim_gt_vd`, `visual_onnx`) produce evidence with populated normals when the relevant detection stages are enabled.

**L0 does not currently aggregate or store normals.** Evidence is ingested into occupancy/risk bins (az×el spherical bins, polar sectors); the normal field is consumed downstream only by L1/L2 surface-hint logic.

**Rationale for deferral:** L0 is the reflexive tick-rate avoidance layer, rebuilt every frame. Adding per-cell normals would require extending `LocalFlightMapSnapshot` data structures, an aggregation pass during ingest, and a reflex policy that acts on orientation — a non-trivial scope addition.

**Future trigger:** when the reflex maneuver policy needs an optimal exit vector on emergency close-range obstacles (e.g., "fly perpendicular to the wall surface to maximise clearance gain rate"). At that point, L0 cells or az×el risk bins can be extended with a dominant-normal field. The normal is already in the evidence; only the L0 aggregation and policy layer need adding.
