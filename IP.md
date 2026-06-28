# Dedalus IP Candidates

Potential patentable inventions identified from the codebase. Each entry describes the technical contribution, the novelty argument against known prior art, and the boundary of what a claim would cover. This document is not a legal opinion — prior art searches and attorney review required before filing.

---

## IP-01 · Spatially-Relative Temporal Decay for Persistent Obstacle Maps

**Status:** Designed, not yet implemented.

**Technical description:**
A cell's decay rate is proportional to how much older it is than its spatial neighbours, not to its absolute age. Formally:

```
relative_age(c) = max(0, median(updated_ns, N6(c)) − c.updated_ns)
decay_factor(c) = base_decay × (1 + relative_age(c) / τ)
```

where `N6(c)` is the 6-connected neighbourhood and `τ` is a characteristic time scale (e.g. 30 days in nanoseconds). A cell last observed 6 months ago surrounded by neighbours also 6 months old is treated as current (the region simply has not been flown recently). The same cell surrounded by neighbours updated yesterday is treated as anomalously stale and decays aggressively, because the area was actively observed and the cell was specifically not re-seen.

**Novelty argument:**
Standard occupancy map decay uses absolute time (exponential forgetting or age thresholding). The specific formulation of spatially-relative decay — normalising for flight-path coverage gaps by comparing each cell against its local temporal context — is not present in OctoMap, Voxblox, FIESTA, or any known UAV mapping literature. It elegantly separates "unobserved because nobody flew here" from "unobserved while the area was being observed", the latter being genuine evidence of absence.

**Claim boundary:**
A method for decaying occupancy in a persistent 3D voxel map where the decay rate of a voxel is a function of the difference between its last-observation timestamp and the median (or percentile) last-observation timestamp of its spatial neighbours, applied to autonomous vehicle mapping.

**Evidence of conception:** Design session between developer and AI assistant, June 27 2026.

---

## IP-02 · Velocity-Adaptive Spatially-Averaged ESDF Repulsion Using Stopping Distance

**Status:** Implemented — `include/dedalus/avoidance/local_esdf_map.hpp`, `repulsion_smoothed()`.

**Technical description:**
`repulsion_smoothed(pos, vel, d0, k, a_max)` computes APF repulsion from a sparse ESDF where the spatial averaging kernel radius adapts to current velocity via the kinematic stopping distance:

```
R(v) = clamp(v² / (2 · a_max), R_min, d0)
```

At low speed R collapses to a point query; at high speed R expands and the repulsion direction is a Gaussian-weighted centroid (`w_i = exp(−‖Δx‖² / (2σ²))`, σ = R/2) over all shell cells within R. This ensures the trajectory planner cannot react to ESDF features smaller than the vehicle's braking footprint. A pre-baked `sgrad` field in `LocalESDFCell` stores the 6-neighbour Gaussian-averaged APF direction at zero runtime cost for the non-velocity-aware query path.

**Novelty argument:**
Standard APF robotics uses a single-point ESDF query. Adapting the spatial averaging radius to stopping distance — so that field features below braking resolution are automatically smoothed out — is not in Voxblox, FIESTA, or prior UAV avoidance literature. The two-level smoothing (stored `sgrad` at construction + runtime Gaussian kernel scaling with velocity) is architecture-novel.

**Claim boundary:**
A method for computing artificial potential field repulsion from a signed distance field where the spatial integration kernel radius is determined by the vehicle's kinematic stopping distance `v²/(2·a_max)`, applied to autonomous vehicle motion planning.

---

## IP-03 · Mission-Provenance Vote Table with Surgical Per-Mission Retraction in Persistent Voxel Maps

**Status:** Implemented — `src/avoidance/mission_local_planning_map_sqlite.cpp`, `cell_votes` schema.

**Technical description:**
A persistent 3D obstacle voxel map (`MissionLocalPlanningMap`) maintains a join table `cell_votes(xi, yi, zi, mission_id, method_id, hit_count, first_ns, last_ns)` tracking which flight mission contributed evidence to each voxel. Database triggers maintain a denormalised `mission_count` on each cell. `remove_mission(mission_id)` atomically deletes all votes from that mission, decrements `mission_count` per affected voxel via trigger, and cascade-deletes any voxel whose count reaches zero — surgically removing all geometry from one identified bad flight without disturbing voxels corroborated by other missions.

**Novelty argument:**
No known persistent obstacle mapping system (OctoMap, RTAB-Map, ElasticFusion) tracks per-mission evidence attribution. The ability to retract geometry from a single identified flight is unique. The vote-table with trigger-maintained denormalised count enables O(1) per-vote operations and O(affected_cells) retraction, not O(full_map) reprocessing.

**Claim boundary:**
A persistent obstacle voxel map system comprising: a vote table mapping voxel coordinates to the identities of observation sources (missions, sensors, or sessions); a denormalised source count per voxel maintained by database triggers; and a retraction operation that atomically removes all observations from a specified source identity and deletes voxels whose source count reaches zero.

---

## IP-04 · Four-Layer Heterogeneous Obstacle Map with Per-Layer Decay Contracts for Edge-Compute UAVs

**Status:** Implemented — L0 (`local_flight_map.hpp`), L1 (`mission_local_traversability_map.hpp`), L2 (`mission_local_planning_map.hpp`), L3 (`local_esdf_map.hpp`).

**Technical description:**
Four spatially-overlapping representations with deliberately heterogeneous decay/eviction contracts:

| Layer | Resolution | Decay | Lifetime | Eviction rule |
|-------|-----------|-------|----------|---------------|
| L0 | ego-local 2D | rebuilt every tick (∞ decay) | 1 tick | always |
| L1 | 0.5 m isotropic | 0.05 score/s time decay | per-flight | score floor 0.1 |
| L2 | 1 m × 1 m × 2 m anisotropic | none; free-space keyed only | cross-mission, SQLite-persisted | explicit free evidence only |
| L3 | 2 m sample spacing | always recomputed from L2; never persisted | on-demand | always |

The invariant that L3 is always recomputed (~6 ms from L2 window) and never persisted prevents stale distance-field data from diverging from L2.

**Novelty argument:**
Prior systems use one or two uniform-decay occupancy representations. The specific four-layer hierarchy with heterogeneous decay contracts, anisotropic L2 voxel sizing (taller voxels matching UAV vertical obstacle profiles), the "absence of observation ≠ free space" L2 eviction rule, and the always-recomputed L3 derivation invariant, as a unified system for edge-compute autonomous UAVs, is novel.

**Claim boundary:**
A multi-layer obstacle representation for autonomous vehicles comprising at least three layers with distinct decay contracts: a short-lived ego-local layer rebuilt each cycle, a per-session accumulator with time-based decay, and a cross-session persistent layer with free-space-keyed eviction; plus a derivative layer always recomputed from the persistent layer and never independently persisted.

---

## IP-05 · L2 Map-Assisted Visual Odometry Scale Disambiguation and ICP Re-Localisation

**Status:** Implemented — `include/dedalus/sensors/visual_ego_state_provider.hpp`, `update_scale_vl2()`, `relocalize_vl2()`.

**Technical description:**
`VisualEgoStateProvider` resolves monocular depth scale ambiguity by comparing metric depth of tracked features against distances to occupied voxels in the persistent L2 `MissionLocalPlanningMap` visible from the current pose. When pose confidence drops below a threshold (`relocalization_confidence_threshold = 0.5`), ICP-lite alignment of the current feature point cloud against L2 occupied cells within a `relocalization_search_radius_m = 5.0 m` sphere is performed. Scale estimate is updated via EMA (`scale_update_alpha = 0.1`). The L2 map injected is the same map driving obstacle avoidance, closing the loop between VIO scale accuracy and map quality.

**Novelty argument:**
Prior map-assisted VIO (VINS-Mono, LVI-SAM) uses pre-built point cloud maps. Using the incrementally-accumulated flight-stack obstacle voxel map — which is the same representation used for avoidance — as the metric scale reference creates a virtuous loop not present in prior literature: better scale → better L2 → better scale.

**Claim boundary:**
A method for monocular visual odometry scale disambiguation wherein metric scale is estimated by comparing observed depth to occupied cells in a persistent cross-mission obstacle voxel map, and re-localisation is performed by aligning the observed feature point cloud to said voxel map.

---

## IP-06 · Unified Three-Pass Monocular Depth Kernel Producing Three Semantically Distinct Evidence Types

**Status:** Implemented — `include/dedalus/sensing/depth_projection_kernel.hpp`.

**Technical description:**
A single monocular depth inference output feeds three sequential GPU-portable passes without OpenCV:

1. `project_depth_to_device_evidence()` — back-projects depth pixels to `ObstacleEvidence{shape=Voxel}` using Brown-Conrady undistortion.
2. `fit_surface_patches_device()` — 32-iteration RANSAC on the projected point cloud with 3-point minimal plane fitter, emitting `ObstacleEvidence{shape=SurfacePatch, is_surface_hint=true}` for flat areas ≥ 0.4 m².
3. `detect_thin_structures_device()` — per-pixel depth contrast against 5×5 neighbourhood max (not image luma gradient), 4-connected BFS component labelling, aspect ratio filtering, emitting `ObstacleEvidence{shape=LineSegment, is_thin_structure_hint=true}`.

All three outputs enter the identical `ObstacleEvidence` ingest pipeline as real sensors. Architecture is a 48-byte GPU-portable `DeviceObstacleEvidence` POD (`depth_projection_kernel.hpp`).

**Novelty argument:**
The depth-contrast thin structure detector compares metric depth against neighbourhood max (not image gradient), making it texture-invariant and metric — this is not in prior thin-obstacle detection literature (which typically uses image-space edge detection). Producing three semantically distinct evidence shapes from one depth frame in a GPU-portable classless architecture is novel.

**Claim boundary:**
A method for detecting thin structural obstacles from monocular depth wherein thin obstacle evidence is identified by comparing each pixel's metric depth against a neighbourhood maximum depth value, and the resulting evidence is expressed in the same typed format as dense volumetric obstacle evidence.

---

## IP-07 · Cross-Mission Persistent Voxel Clearance Gate for Landing Zone Selection

**Status:** Implemented — `include/dedalus/runtime/perch_candidate_evaluator.hpp`.

**Technical description:**
`PerchCandidateEvaluator::evaluate()` scores monocular-depth-derived surface patches (`is_surface_hint=true`, `|n.z| ≥ 0.866`, area ≥ 0.4 m²) and then fires an upward `ray_cast()` into the cross-mission persistent L2 `MissionLocalPlanningMap`, rejecting any candidate where an obstacle exists within `min_clearance_m = 2.0 m`. The clearance check uses obstacle memory from prior missions, meaning a landing zone that was previously observed to be obstructed (in a prior flight, even if not currently in camera view) is correctly rejected.

**Novelty argument:**
Prior safe landing zone detection (stereo-based, lidar-based, monocular-based) uses only current-frame or current-flight geometric evidence for clearance gating. Using a cross-mission persistent obstacle map for the overhead clearance check is novel — it prevents landing on zones that were temporarily clear but are historically obstructed.

**Claim boundary:**
A method for selecting autonomous vehicle landing or perch zones wherein candidate zones derived from current sensor evidence are validated against a cross-session persistent obstacle map using a clearance ray cast, and candidates are rejected if said map indicates historical obstruction within the clearance envelope.

---

## IP-08 · Typed Real-Time A/B Dual-Provider Evaluation Embedded in UAV Perception Pipeline

**Status:** Implemented — `include/dedalus/runtime/evaluation_slot.hpp`, `core_stack_runner.hpp`.

**Technical description:**
Every perception stage (ego state, detection, tracking, 3D projection, depth) holds a typed `EvaluationSlotPair<Provider>` with a primary slot A (feeds all downstream systems) and a reference slot B (logs agreement metrics only, zero downstream effect). Agreement metrics are stage-semantics-specific: detection uses IoU > 0.5 fraction over `Detection2D`; 3D observation uses position distance < 1 m fraction over `Observation3D`; ego state uses normalised position distance. Null slot B = zero overhead (branch eliminated at compile time via `nullptr` check). The framework enables continuous simulation-to-production migration validation without test code branching in the hot path.

**Novelty argument:**
ML A/B testing frameworks are offline. Embedding a typed A/B provider pair at each stage of a real-time flight-critical perception pipeline, with stage-semantics-specific agreement metrics and compile-time zero cost for inactive slot B, is specific to safety-critical autonomous systems.

**Claim boundary:**
A real-time autonomous vehicle perception pipeline wherein each processing stage contains a primary provider feeding downstream systems and a reference provider whose output is compared against the primary using stage-semantics-specific agreement metrics, with zero downstream effect and zero runtime cost when the reference provider is inactive.

---

## IP-09 · Dual 1D + 2D Spherical TTC Risk Map with Per-Bin Sensor Source Attribution from Occupancy Map

**Status:** Implemented — `include/dedalus/avoidance/local_flight_map.hpp`, `L0PolarRiskSector`, `L0SphericalRiskBin`.

**Technical description:**
A single pass over L0 occupied cells computes simultaneously: (1) a 1-D azimuth risk array (36 × 10° sectors) with `min_ttc_s`, `max_closing_speed_mps`, and `nearest_range_m`; and (2) a 2-D `az × el` spherical grid (36 × 9 bins, ±45° elevation) where each bin carries a `source_mask` bitmask encoding which sensor modalities contributed evidence (bit 0 = AirSim GT, bit 1 = DepthProvider, bit 2 = VisualObstacleDetector). Closing speed is computed cell-by-cell as `(vel_body · bearing) / range`. A parallel `collect_l0_sensor_observations()` preserves pre-aggregation raw bearing and source identity for per-modality comparison.

**Novelty argument:**
Prior UAV collision avoidance computes distance-to-nearest or azimuth risk only. The combination of per-bin TTC from occupancy map cells (not tracker estimates) with 2D spherical layout and per-bin sensor source attribution bitmask, enabling both risk display and per-modality agreement analysis in one pass, is novel.

**Claim boundary:**
A risk computation method for autonomous vehicles that, in a single pass over an occupancy map, produces both a 1-D azimuth TTC sector array and a 2-D elevation-azimuth spherical risk grid with per-bin sensor source attribution bitmasks.

---

## IP-10 · Hysteresis Sliding Window over Cross-Mission SQLite-Backed Voxel Map with Async Dirty-Cell Flush

**Status:** Implemented — `include/dedalus/avoidance/mission_local_planning_map.hpp`, `slide_window()`.

**Technical description:**
`slide_window(drone_pos)` maintains a bounded in-memory L2 footprint: trigger at `horizon_m/4` (37.5 m) movement, load cells within `horizon_m` (150 m), evict cells beyond `2 × horizon_m` (300 m). The hysteresis band (150–300 m) prevents load/evict oscillation when the drone hovers near a boundary. Cells are not deleted from SQLite on eviction — they re-enter memory when the drone returns. Writes are accumulated in `dirty_cells_` behind a mutex and flushed asynchronously by a background thread every 10 seconds, avoiding SQLite I/O jitter in the 30 Hz flight control loop.

**Claim boundary:**
A sliding window method for a cross-session persistent obstacle map wherein cells are loaded at a first radius threshold and evicted at a larger second radius threshold (hysteresis band), and write operations are accumulated in a dirty set and asynchronously flushed to persistent storage by a background thread decoupled from the primary control loop.

---

## IP-11 · Ghost Detections as First-Class Typed Perception Primitives with Live Scene Object Binding

**Status:** Implemented — `include/dedalus/perception/ghost_targets.hpp`.

**Technical description:**
`GhostTargetProvider` produces `PerceptionPipelineOutput` and `Observation3D` instances with identical typed contracts to real camera detectors — same struct types, same ingest path, same world-model update logic. No branching anywhere in the pipeline. The AirSim variant (`AirSimGhostObjectPatternBinding`) binds ghost `source_track_id`s to live AirSim scene objects by name or wildcard prefix, streaming actual object poses from the simulation via the bridge transport — enabling oracle-quality ground truth injected as "real" detections for calibration and regression testing.

**Claim boundary:**
A simulation framework for autonomous vehicle perception wherein synthetic agent observations implement the same typed output interface as physical sensor detectors and are processed through the identical production pipeline without code branching, combined with live binding of synthetic agent trajectories to simulation scene object poses via pattern matching.

---

## IP-12 · Decoupled EDT/ESDF Computation and Storage Resolution with L2-Aligned Grid Snapping

**Status:** Implemented — `src/avoidance/compute_esdf.cpp`, `local_esdf_map.hpp`.

**Technical description:**
`compute_esdf()` runs Felzenszwalb–Huttenlocher 3-phase separable EDT at L2 native resolution (1 m × 2 m) for accuracy, but stores output cells only at a coarser `sample_spacing_m = 2.0 m` stride. APF direction at each stored cell is a Gaussian-weighted average over fine-resolution EDT cells within ±1 coarse step. Grid origin is snapped to the nearest L2 cell boundary (`xi_origin = floor(centre/sx) * sx`) so L3 and L2 cell keys are aligned — enabling O(1) key-to-key correspondence between layers without floating-point tolerance. `update_incremental()` refreshes only the dirty L2 sub-volume: erase stale ESDF cells in the bounding box, recompute EDT over the box expanded by `d0 + 1 margin`, merge result.

**Claim boundary:**
A signed distance field computation method wherein EDT is computed at a first (fine) spatial resolution and output cells are stored at a second (coarser) resolution aligned to boundaries of the source occupancy map grid, with APF direction values computed as Gaussian-weighted averages over fine-resolution cells within the coarse cell footprint.

---

## IP-13 · L2-Anchored Cross-Session Relocalization with Active Disambiguation and Elastic Local Frame Correction

**Status:** Designed, not yet implemented.

**Technical description:**
On session start — and optionally mid-flight — the drone matches incoming depth observations against occupied voxels in the persistent L2 `MissionLocalPlanningMap` to estimate the rigid transform T_new→L2. Once this transform is known, all subsequent ego poses are expressed directly in the existing L2 frame rather than in a freshly-initialised per-session coordinate frame. The `fallback_map_frame_id` config key degenerates to a human-readable site label; it no longer defines an absolute origin.

Four key properties of the design, validated through discussion:

1. **Coarse resolution is sufficient** — 1 m × 1 m × 2 m voxels are adequate for relocalization at UAV operational scales. Sub-meter precision is not required for avoidance, target pursuit, or landing handoff.

2. **Active disambiguation** — when the initial L2 observation match is ambiguous (sparse map, repeated structure), the drone actively moves to reduce the pose hypothesis set, exactly as biological agents do. This is treated as a first-class strategy, not a fallback.

3. **Site-relative frame, cold-start safe** — the L2 origin is wherever the first session started. The frame is not geographically fixed. If a better georeference is later available (GPS fix, known landmark), the entire DB can be re-anchored via a single affine transform (`UPDATE cells SET cx=cx+dx, cy=cy+dy, cz=cz+dz`). Cold start on an empty map behaves identically to the current static-frame system.

4. **Elastic local correction** — internal frame consistency matters more than agreement with external ground truth. When a region is revisited and the match displacement is Δ, the correction is applied locally with stiffness proportional to evidence density (densely-observed regions resist deformation; sparse regions flex). This is analogous to graph-SLAM loop closure but without a full pose graph: a lightweight "apply delta to cells in radius R, with linear falloff" suffices at L2 resolution. The L2 frame can stretch locally.

Loop closure is the correction mechanism: when the drone re-observes a region, the measured displacement between the predicted L2 key and the matched occupied cells is the drift correction signal. Even a simple "shift the running pose offset by Δ, weighted by match confidence" gives progressive drift correction without a formal SLAM backend.

**Novelty argument:**
Prior map-based relocalization (Monte Carlo localization, visual place recognition, LiDAR ICP) uses a separate, dedicated localization map. Here the obstacle avoidance voxel map — the same structure driving L0/L1/L3 and landing zone selection — is the localization reference, creating a virtuous loop: better pose → better L2 → better pose. The elastic local correction model (local rigidity proportional to evidence density, applied directly to voxel coordinates without a pose graph) is novel. The treatment of active motion as a first-class disambiguation strategy rather than a SLAM failure mode is novel for persistent-map UAV navigation.

**Claim boundary:**
A method for autonomous vehicle session initialization wherein the vehicle's coordinate frame is established by matching current sensor observations against a pre-existing persistent obstacle voxel map, with ambiguity resolved by deliberate vehicle motion; and wherein mid-session drift corrections are applied locally to the voxel map with spatial stiffness proportional to evidence density in each region.

**Evidence of conception:** Design session between developer and AI assistant, June 27 2026.

---

## Summary and Priority

| ID | Title | Priority | Status |
|----|-------|----------|--------|
| IP-01 | Spatially-relative temporal decay | **Highest** | Not implemented — conceived Jun 2026 |
| IP-02 | Velocity-adaptive ESDF APF repulsion | **Highest** | Implemented |
| IP-03 | Mission-provenance vote table + retraction | **Highest** | Implemented |
| IP-04 | Four-layer heterogeneous obstacle map | **High** | Implemented |
| IP-05 | L2 map-assisted VIO scale disambiguation | **High** | Implemented |
| IP-06 | Three-pass depth kernel → 3 evidence types | **High** | Implemented |
| IP-07 | Cross-mission clearance gate for landing zones | **High** | Implemented |
| IP-08 | Typed real-time A/B dual-provider framework | **Medium** | Implemented |
| IP-09 | Dual 1D+2D spherical TTC with source mask | **Medium** | Implemented |
| IP-10 | Hysteresis sliding window + async flush | **Medium** | Implemented |
| IP-11 | Ghost detections as first-class primitives | **Lower** | Implemented |
| IP-12 | Decoupled EDT resolution + L2-aligned snapping | **Lower** | Implemented |
| IP-13 | L2-anchored cross-session relocalization + elastic frame | **Highest** | Not implemented — conceived Jun 2026 |

---

## Notes for Patent Counsel

- IP-01 and IP-13 have no code yet — conception date June 27 2026, documented in this session. Provisional filing recommended before implementation for both.
- IP-13 (L2-anchored relocalization) has broadest architectural scope and should be treated as a system claim covering the unified localization+mapping loop. Consider combined filing with IP-05 (VIO scale from L2) and IP-03 (mission provenance), which are sub-claims of the same system.
- IP-01 has no code yet — conception date is June 27 2026, documented in this session. Provisional filing recommended before implementation.
- IP-02 through IP-04 are the strongest candidates for claims with clear formula novelty and architecture scope.
- IP-03 (mission retraction) has broadest commercial applicability: applies to any autonomous system with long-duration persistent maps (self-driving, marine, warehouse robotics) not just UAVs.
- IP-05 creates prior art defensively against any future claims on "map-assisted monocular VIO".
- All items should be checked against ICRA/IROS/RA-L 2018-2025 proceedings and relevant ROS/PX4 open-source repos before filing.
