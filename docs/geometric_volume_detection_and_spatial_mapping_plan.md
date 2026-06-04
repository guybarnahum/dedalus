# Geometric Volume Detection and Spatial Flight Mapping

This document records the classless geometric volume detection and layered spatial mapping stages that follow the AirSim GT obstacle-source, swept-volume, overlay, and runtime profiling work.

It extends:

- `docs/classless_geometric_occupancy_and_avoidance_plan.md`
- `docs/reflexive_obstacle_avoidance_architecture.md`

The central constraint is unchanged: **Track 4 is classless geometric safety, not semantic object recognition**. Do not make YOLO/DETR-style classifiers a prerequisite for obstacle avoidance. Track 4 should detect occupied, free, and unknown space from arbitrary geometric volume evidence: depth, freespace, obstacle masks, optical flow, looming cues, AirSim depth, stereo, monocular depth, synthetic fixtures, or GT visual-emulation. Semantic labels may decorate cells later, but collision avoidance must not depend on them.

---

## Current state before 4.1

The repo now has a working AirSim GT obstacle source that can populate `ego_occupancy` and `latest_swept_volume` in `WorldSnapshot` and render occupancy/swept-volume OSD markers through `airsim-world-overlay.py`.

Recent performance stages established:

- AirSim GT object pose bridge is persistent.
- Nearby inventory selection limits the GT pose stream.
- Static obstacle pose refresh is throttled with `DEDALUS_AIRSIM_GT_STATIC_REFRESH_EVERY_N_FRAMES`.
- `run_mission.sh` supports provider-neutral `--source-frame-rate-hz`, `--pipeline-timing`, and `--frame-producer-timing`.
- EFF profiles show GT source cost is sub-millisecond; AirSim `simGetImages` is now the dominant live producer cost.

Important existing limitation:

- The current occupancy/swept-volume debug path is still too global/simple. It mostly treats obstacles as map-frame occupied cells and forward checks as local +X / forward corridor approximations. For visual obstacle work, the system must explicitly represent what the current sensor could actually see.

---

# 4.1 — Classless Arbitrary Volume Detector Contract

## Goal

Introduce a provider-neutral geometric obstacle evidence contract so GT, AirSim depth, synthetic fixtures, and the future visual detector are interchangeable downstream.

The output of 4.1 should answer:

> Given the current camera/body pose and a geometric sensing model, what volumes of space are occupied, free, or unknown, and with what confidence?

It should **not** answer:

> What class is this object?

A later semantic stage can annotate the evidence, but the Track 4 safety path should work with `occupied`, `free`, `unknown`, `thin-structure-risk`, and geometric confidence.

## 4.1A — Sensing volume / detection capability model

Add an explicit sensing-volume description to artifacts/world snapshot sidecars. This is the volume in which a provider can plausibly produce evidence.

Candidate contract:

```cpp
struct ObstacleSensingVolume {
    TimePoint timestamp;
    FrameId source_frame_id;
    std::string sensor_name;       // e.g. front_center
    std::string provider_name;     // airsim_depth, visual_volume_detector, airsim_gt_visual_emulation
    MapFrameId map_frame_id;

    Vec3 origin_local;
    Vec3 forward_axis_local;
    Vec3 right_axis_local;
    Vec3 up_axis_local;

    double near_range_m;
    double far_range_m;
    double horizontal_fov_rad;
    double vertical_fov_rad;
    double min_reliable_range_m;
    double max_reliable_range_m;

    // Optional implementation gates.
    double min_surface_area_m2;
    double min_angular_size_rad;
    double min_confidence;
};
```

For early AirSim/live validation, this can be a frustum or frustum-like band in front of the camera/body.

The OSD should render this as the **detector capability band/frustum**, not as the swept-volume collision query.

## 4.1B — Common obstacle evidence contract

Introduce a classless evidence contract that can represent voxels, rays, frustum bins, capsules/line segments, or surface patches.

Candidate contract:

```cpp
enum class ObstacleEvidenceState {
    Unknown,
    Free,
    Occupied,
    ThinStructureRisk,
};

enum class ObstacleEvidenceShape {
    Voxel,
    FrustumBin,
    RaySegment,
    SurfacePatch,
    LineSegment,
    Capsule,
};

struct ObstacleEvidence {
    TimePoint timestamp;
    FrameId source_frame_id;
    std::string sensor_name;
    std::string source_provider;
    OccupancySourceKind source_kind;
    MapFrameId map_frame_id;

    ObstacleEvidenceState state;
    ObstacleEvidenceShape shape;

    Vec3 center_local;
    Vec3 size_m;
    Vec3 endpoint_a_local;
    Vec3 endpoint_b_local;
    double radius_m;

    double occupancy_probability;
    double free_probability;
    double confidence;
    double range_m;
    double bearing_rad;
    double elevation_rad;

    bool inside_sensing_volume;
    bool inside_swept_volume;
    bool is_static_hint;
    bool is_thin_structure_hint;
};
```

Names can be adjusted to match repo conventions. The important point is that downstream `InMemoryWorldModel`, ERODF, swept-volume queries, OSD, and future map builders consume this normalized contract rather than source-specific GT or visual objects.

## 4.1C — AirSim GT visual-emulation source

The current AirSim GT source is useful as an oracle, but it should have a detector-emulation mode for fair comparison against classless visual volume detection.

Modes:

```text
airsim_gt_global_oracle:
  Uses nearby inventory selection and emits selected GT obstacles, even if not in the current camera frustum.
  Useful for safety regression and map oracle validation.

airsim_gt_visual_emulation:
  Clips GT objects against ObstacleSensingVolume.
  Emits only what the configured visual/volume detector could plausibly observe.
  Useful for recall/precision-style validation of visual geometry providers.

visual_volume_detector:
  Uses image/depth/freespace/flow evidence to emit the same ObstacleEvidence contract.
```

GT visual emulation should optionally enforce:

- inside frustum / sensing band
- min/max range
- image projection in bounds
- min apparent angular size or approximate projected area
- optional thin-structure mode for cables/wires using conservative inflation

The visual detector should be compared against `airsim_gt_visual_emulation`, not against the global oracle.

## 4.1D — Arbitrary volume detector stub

Before integrating expensive CV/depth models, add a deterministic provider that emits arbitrary geometric volume evidence from configured fixtures or AirSim depth-like input.

The stub should support:

- occupied frustum bins
- free-space rays
- unknown outside sensing volume
- thin line/capsule fixtures for cable-like obstacles
- deterministic CI tests with no AirSim dependency

This creates the interface and validation harness for the real detector.

## 4.1E — Real classless visual volume detector

Only after 4.1A–D, add a real provider. Initial candidates:

- AirSim depth-to-volume provider for simulation
- RGB-D/depth-frame volume provider
- monocular-depth-to-volume provider
- freespace/obstacle-mask-to-volume provider
- optical-flow/looming risk provider
- thin-line/cable-risk provider

Do not start by adding a YOLO classifier. The detector should produce arbitrary occupied/free/unknown volumes and optional thin-structure risk, not object classes.

---

# OSD / Visualization requirements for 4.1

The overlay must make detector capability explicit.

Show these as distinct layers:

```text
SENSOR BAND / FRUSTUM:
  What this provider could plausibly detect.

OCCUPIED/FREE/UNKNOWN EVIDENCE:
  What the source actually emitted.

SWEPT VOLUME:
  Collision query along nominal or current planned motion.

BLOCKING CELLS / PRIMITIVES:
  Evidence intersecting the swept volume.

OUT-OF-BAND GT:
  Optional debug-only oracle markers, faint/hidden by default, not counted as visual misses.
```

This avoids misleading OSD where GT obstacles behind/outside the camera appear as if a visual detector should have seen them.

---

# 4.2 — Russian-Doll Flight Map Builder

## Goal

Upgrade `rough_flight_map_builder.cpp` from placeholder output to a persistent map builder that consumes the real-time reflexive obstacle field and creates multi-resolution traversability memory for local planning, faster flight through known corridors, and eventual ego localization.

Current status: `rough_flight_map_builder.cpp` is a stub that emits hardcoded structures/corridors/landmarks. 4.2 should replace the placeholder logic with stateful map accumulation.

## Three-layer Russian-doll map

Use 3 detail levels because each level has a distinct job.

### L0 — Reflexive near field

```text
range:      0–15 m or next 1–2 seconds of motion
resolution: 0.25–0.5 m equivalent
lifetime:   0.2–2 s
frame:      ego/body/camera or local map
purpose:    immediate collision avoidance, swept-volume blocking, emergency stop/slow/bypass
```

Input: current `ego_occupancy`, `latest_swept_volume`, and/or normalized `ObstacleEvidence`.

### L1 — Local maneuver map

```text
range:      15–80 m or next 2–6 seconds of motion
resolution: 1–2 m equivalent
lifetime:   5–30 s
frame:      active local map frame
purpose:    local replanning, avoid rediscovering obstacles, select bypass corridors
```

This should be the first real implementation target for `RoughFlightMapBuilder`.

### L2 — Global rough map

```text
range:      mission area / repeated observations
resolution: 4–10 m equivalent or semantic structures instead of dense voxels
lifetime:   mission/session, eventually persistent
frame:      active global/map frame
purpose:    route planning, known blocked zones, known flyable corridors, geometric landmarks for localization
```

Do not use L2 for reflexive safety decisions. Use it for planning bias and memory.

## Variable resolution and speed awareness

Use distance/time-to-collision shells instead of one uniform voxel grid.

Recommended rule:

```text
near shell = next 1–2 seconds of travel, fine detail
mid shell  = next 2–6 seconds, medium detail
far shell  = next 6–15 seconds, coarse detail / semantic structure
```

At higher speeds:

- extend forward horizon
- widen/inflate swept volume
- rely more on coarse early warnings
- increase unknown-space cost

At lower speeds/hover:

- allow finer local detail
- reduce horizon
- improve local map confidence with repeated observations

## Traversability, not just occupancy

Flight planning needs more than free/occupied.

Each map cell/primitive should maintain:

```cpp
struct FlightMapCell {
    Vec3 center_local;
    Vec3 size_m;
    float occupancy_probability;
    float free_probability;
    float traversability_cost;
    float confidence;
    TimePoint first_seen;
    TimePoint last_seen;
    TimePoint last_free_observed;
    OccupancySourceKind last_source_kind;
    bool thin_structure_risk;
};
```

Possible costs:

```text
occupied: hard block
unknown: soft risk, grows with speed and cable-prone mode
free: low cost, decays over time
thin-structure-risk: high cost, conservative inflation, longer persistence
```

## Cable/wire handling

Cables and wires should be represented as line/capsule primitives, not only ordinary voxels.

For Track 4 they remain classless: the system does not need to label a cable semantically, but it should recognize thin-structure geometry/risk.

Behavior:

- accumulate weak thin-line evidence over frames
- inflate conservatively
- persist longer than ordinary uncertain blobs
- raise unknown-space cost at speed in cable-prone environments
- rasterize into L0/L1 occupancy for swept-volume checks

## RoughFlightMapBuilder evolution

Recommended refactor:

```cpp
class RoughFlightMapBuilder {
public:
    RoughFlightMapUpdate update(
        const EgoOccupancyMapSnapshot& reflexive_occupancy,
        const std::vector<ObstacleEvidence>& evidence,
        const ObstacleSensingVolume& sensing_volume,
        const EgoState& ego,
        TimePoint now,
        MapFrameId map_frame_id);

private:
    MultiResolutionFlightMap map_;
};
```

The current method can remain as an adapter during migration, but the real builder should become stateful because a global/local map cannot be rebuilt from only the current frame.

## 4.2 implementation stages

### 4.2A — Consume reflexive occupancy

Make `RoughFlightMapBuilder` consume `ego_occupancy`/`ObstacleEvidence` and emit a debug-only multi-resolution map summary.

Acceptance:

- no behavior changes
- artifacts include L0/L1/L2 map summaries
- deterministic synthetic test updates a few cells and validates aging/costs

### 4.2B — Rolling local traversability map

Add L1 map memory with aging, confidence, traversability cost, and simple corridor extraction.

Acceptance:

- repeated observations increase confidence
- stale cells decay
- known-free corridors are emitted as `FlightCorridor` candidates
- blocked zones appear in artifacts/OSD

### 4.2C — Global rough map promotion

Promote persistent repeated local evidence into coarse L2 structures/corridors/landmarks.

Acceptance:

- repeated wall/fence/tree-line evidence creates stable structures
- free corridors persist with confidence and decay
- map updates are bounded in size and artifact-visible

### 4.2D — Planner hooks

Expose map outputs useful to mission/trajectory planning:

- preferred local corridor
- blocked sectors
- unknown-risk sectors
- speed recommendations by corridor confidence
- landmarks/structures for future ego localization

Do not put this logic in the flight sink.

---

# Validation principles

For 4.1/4.2 validation, use staged source comparisons:

```text
synthetic volume fixtures:
  deterministic CI, no AirSim.

airsim_gt_global_oracle:
  regression that GT source and nearby inventory selection work.

airsim_gt_visual_emulation:
  expected detections inside the sensing volume.

visual_volume_detector:
  actual geometry provider output.
```

The core comparison is:

```text
visual_volume_detector vs airsim_gt_visual_emulation
```

not visual detector vs all GT objects.

Expected artifacts:

- `WorldSnapshot.ego_occupancy`
- `WorldSnapshot.latest_swept_volume`
- sensing-volume summary
- obstacle evidence summary
- flight-map level summaries
- OSD sensor band / swept volume / blocking evidence markers

---

# Do not

- Do not require YOLO/DETR/semantic classifiers for Track 4 obstacle avoidance.
- Do not treat AirSim global GT as the same thing as visual detector capability.
- Do not make the overlay infer detector semantics; publish source/capability metadata from the world model.
- Do not put avoidance or mapping policy in command sinks.
- Do not duplicate occupancy logic separately for GT and visual sources.
- Do not judge visual detector recall against obstacles outside the configured sensing volume.
- Do not make `rough_flight_map_builder.cpp` a second perception pipeline; it should consume normalized obstacle evidence/reflexive occupancy.
