# Track 4.x — Visualization, Ground Truth, and 4.0A Implementation Plan

This document records the Track 4.x implementation plan with world-model visualization in mind. It complements `docs/track_4_obstacle_avoidance_and_mapping_plan.md` and `docs/erodf_reflexive_avoidance_whitepaper.md`.

Track 4.x is the classless geometric safety layer. It must remain independent of semantic object detection for collision safety, but it should support multiple selectable obstacle evidence sources for validation and comparison.

## Architectural rule

Do not create a parallel obstacle visualization or avoidance IPC stack.

Reuse the existing repo seams:

```text
Perception / obstacle evidence
  -> InMemoryWorldModel
  -> WorldSnapshot
  -> ArtifactSnapshotWriter snapshot JSON
  -> RuntimeEventStreamServer JSONL
  -> AirSim world overlay subscriber
  -> PPM frame annotation + sidecar JSON
  -> existing PPM -> MP4 export script
```

Track 4.x products should be visible through the world model first, then rendered by existing subscribers/renderers.

## Source selection model

All obstacle sources should produce the same occupancy/world-model contract.

```text
obstacle_source = synthetic_fixture
obstacle_source = airsim_ground_truth
obstacle_source = visual_obstacle_detector
obstacle_source = depth_provider
obstacle_source = fused
```

The selected source must be visible in `WorldSnapshot.ego_occupancy` through fields such as:

```text
source_kind
source_provider
source_object_name per debug cell / primitive where available
```

The source switch should live in Track 4 occupancy/provider configuration, not in the AirSim overlay and not in flight sinks.

## Stage 4.0A-Viz — World-model occupancy contract

Status: started.

Goal: make occupancy a first-class `WorldSnapshot` product before adding swept-volume query or command correction.

Deliverables:

```text
include/dedalus/occupancy/occupancy_types.hpp
  - OccupancyCellState
  - OccupancySourceKind
  - OccupancyCellSummary
  - EgoOccupancyMapSnapshot

include/dedalus/world_model/world_snapshot.hpp
  - WorldSnapshot::has_ego_occupancy
  - WorldSnapshot::ego_occupancy

src/world_model/world_snapshot.cpp
  - serialize ego_occupancy into existing snapshot JSON

src/world_model/in_memory_world_model.cpp
  - publish deterministic synthetic ego occupancy for early artifacts/streaming

tests/unit/test_world_snapshot_json.cpp
  - assert ego_occupancy JSON contract
```

Non-goals for 4.0A:

```text
- no swept-volume collision query yet
- no avoidance policy yet
- no flight command correction
- no AirSim object query yet
- no visual detector integration yet
```

Success criteria:

```text
snapshot JSON contains ego_occupancy
runtime stream carries ego_occupancy because it embeds WorldSnapshot::to_json()
artifact snapshots carry ego_occupancy because ArtifactSnapshotWriter uses WorldSnapshot::to_json()
existing tests still pass
```

## Stage 4.0B — Synthetic ego occupancy visualization

Goal: render deterministic synthetic occupancy from `WorldSnapshot.ego_occupancy`.

AirSim overlay:

```text
simulation/airsim/scripts/airsim-world-overlay.py
  --show-occupancy-summary
  --show-occupancy-cells
  --max-occupancy-cells
```

Render rules:

```text
occupied debug cells: red/orange sparse points or boxes
unknown debug cells: yellow/grey sparse points or outline
free debug cells: optional low-density green/blue samples
summary OSD: OCC src=<source> occ=<n> free=<n> unk=<n> clear=<m>
```

MP4/offline annotation:

```text
src/visualization/frame_annotator.cpp
  draw occupancy summary in HUD
  project and draw selected occupied/high-risk cells

src/visualization/world_overlay_sidecar.cpp
  write occupancy summary and projected occupancy cells
```

Use existing `WorldToImageProjector`. Do not add another projection implementation.

## Stage 4.0C — World-overlay sidecar extension

Goal: make offline MP4/debug artifacts useful without overloading pixels.

Extend each `frame_XXXXXX.world_overlay.json` with:

```json
{
  "occupancy": {
    "map_frame_id": "map_local_0001",
    "source_kind": "synthetic_fixture",
    "summary": {
      "occupied_count": 1,
      "free_count": 1,
      "unknown_count": 1,
      "nearest_obstacle_distance_m": 5.0,
      "forward_corridor_clearance_m": 4.0
    },
    "projected_cells": []
  }
}
```

The frame image should show only selected readable markers. The sidecar should carry richer structured debug details.

## Stage 4.0D — Swept-volume query visualization, dry-run only

Goal: visualize the query before it changes flight.

Add:

```text
include/dedalus/occupancy/swept_volume_query.hpp
src/occupancy/swept_volume_query.cpp
```

Expose in snapshot/artifacts:

```text
latest_swept_volume.status
latest_swept_volume.start_local
latest_swept_volume.end_local
latest_swept_volume.radius_m
latest_swept_volume.min_clearance_m
latest_swept_volume.time_to_collision_s
latest_swept_volume.blocking_cell_centers
```

Visualization:

```text
AirSim: centerline + sparse radius rings; green/yellow/red by status
MP4: projected centerline + status text
```

No command correction in this stage.

## Stage 4.0E — Avoidance policy dry-run

Goal: compute what the avoidance layer would do, but still send nominal commands.

Add:

```text
AvoidancePolicy
AvoidanceCorrection
avoidance_query / avoidance_would_correct / avoidance_clear events
```

Visualization:

```text
nominal velocity arrow
would-correct velocity arrow
optional repulsion hint vector
reason and clearance in OSD/MP4 HUD
```

The runtime event stream already carries mission events, so dry-run avoidance events should use that existing path.

## Stage 4.0F-GT — AirSim ground-truth obstacle provider

Goal: use AirSim/Unreal ground-truth object poses as a classless obstacle source.

This stage bridges deterministic synthetic fixtures and visual/depth detector output.

Flow:

```text
AirSim object query by name/pattern
  -> AirSimGroundTruthObstacleProvider
  -> OccupancyObservation / OccupancyCellSummary / primitive
  -> InMemoryWorldModel
  -> WorldSnapshot.ego_occupancy
  -> artifacts / stream / overlay / MP4
```

Example source config:

```yaml
track4_occupancy:
  enabled: true
  obstacle_source: airsim_ground_truth

  airsim_ground_truth:
    object_name_patterns:
      - "Tree*"
      - "Pole*"
      - "Fence*"
      - "Wire*"
      - "Cable*"
      - "Wall*"
      - "Rock*"
      - "Building*"
    default_inflation_radius_m: 0.5
    thin_object_inflation_radius_m: 0.75
    max_range_m: 60.0
```

The first pass may use object pose plus approximate bounding boxes/dimensions. If exact dimensions are unavailable, allow explicit overrides:

```yaml
object_overrides:
  UtilityPole_01:
    shape: capsule
    radius_m: 0.25
    height_m: 8.0

  Cable_01:
    shape: line_segment
    radius_m: 0.05
    inflation_radius_m: 0.75
```

Ground truth cells/primitives must be marked with:

```text
source_kind = airsim_ground_truth
source_provider = AirSimGroundTruthObstacleProvider
source_object_name = <AirSim object name>
```

## Stage 4.0F-GT.2 — Detector versus ground-truth comparison

Goal: compare visual/depth obstacle detection against AirSim ground truth.

Run sources in parallel:

```text
AirSimGroundTruthObstacleProvider
VisualObstacleDetector / DepthProvider
```

Emit an evaluation product into `WorldSnapshot`/artifacts later:

```text
gt_obstacle_count
detector_obstacle_count
matched_count
missed_gt_count
false_positive_count
mean_center_error_m
mean_depth_error_m
occupancy_iou_3d
forward_corridor_recall
forward_corridor_precision
```

For avoidance, the key metric is not generic IoU. The key metric is:

```text
Did detector evidence identify obstacles that intersect the drone's swept path?
```

Visualization:

```text
GT obstacle: cyan/blue
Detector obstacle: yellow/orange
Matched: green
Missed GT: red
False positive: purple
```

## Stage 4.0G — AirSim/depth-derived visual obstacle detector

Goal: generate occupancy from real frame-derived evidence while preserving the same world-model contract.

Sources:

```text
AirSim depth
monocular depth
stereo depth
freespace masks
optical-flow / looming cues
```

All providers feed the same occupancy contract. Avoidance should not know which detector generated the evidence except through provenance/confidence fields.

## Stage 4.0H — Thin obstacle primitives

Goal: represent cables/wires/poles without requiring a dense voxel explosion.

Add primitive support:

```text
Voxel
Capsule
LineSegment
SurfacePatch
```

Wire/cable strategy:

```text
store suspected cable as inflated line/capsule primitive
rasterize into occupancy only for query/debug
visualize primitive directly in AirSim/MP4
```

## Stage 4.0I — Avoidance mission adapter, simulation-only correction

Goal: enable actual correction in simulation without modifying flight sinks.

Preferred first integration:

```text
AvoidanceFlightCommandSink wrapper
  reads LatestWorldSnapshot
  computes correction
  emits mission events
  forwards corrected command to inner sink
```

This preserves the current `MissionRuntime` and command sink interfaces. Later, this can become a cleaner mission adapter.

## Stage 4.0J — Global map promotion and known-free corridors

Goal: promote stable ego-local occupancy into mission/global map products.

Promote only repeated, well-localized, non-dynamic geometry:

```text
OccupiedVoxel
SurfacePatch
LineSegment
KnownFreeCorridor
```

Known-free corridors are as important as obstacles because they allow faster flight through previously validated space.

## Stage 4.0K — Map-aware speed and route bias

Goal: use global geometric memory to fly better and faster.

Map-risk query output:

```text
max_recommended_speed_mps
known_free_confidence
obstacle_risk
unknown_risk
reason
```

Policy:

```text
known-free corridor -> allow higher speed
unknown corridor -> cap speed
known clutter -> slow down and inflate safety radius
wire-risk region -> slow down aggressively
```

## Stage 4.0L — Ego localization hooks

Goal: expose geometric constraints for future VIO/SLAM/localization modules.

Candidate constraints:

```text
PointToPlane
PointToLine
PlaneToPlane
LineToLine
OccupancyAlignment
```

Do not implement full SLAM in early Track 4.x. Preserve geometric products and confidence so a later localization module can consume them.

## Validation ladder

```text
1. WorldSnapshot JSON contract tests.
2. Synthetic occupancy fixtures.
3. AirSim overlay dry-run rendering from snapshot stream.
4. PPM annotation + sidecar validation.
5. MP4 export from PPM sequence.
6. Swept-volume query unit tests.
7. Avoidance dry-run event tests.
8. AirSim ground-truth provider comparison tests.
9. Visual/depth detector versus ground truth comparison.
10. Simulation-only avoidance correction.
```

## Hard boundaries

```text
- no obstacle logic inside flight sinks
- no overlay-side occupancy inference
- no separate occupancy IPC stack
- no bypass of WorldSnapshot
- no command correction before dry-run visualization and artifacts are validated
- no semantic detector dependency for collision safety
```
