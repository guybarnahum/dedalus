# Track 4.x — Visualization, Ground Truth, and 4.0A Implementation Plan

This document records the implementation plan with world-model visualization in mind. It complements `docs/classless_geometric_occupancy_and_avoidance_plan.md` and `docs/reflexive_obstacle_avoidance_architecture.md`.

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

Status: complete.

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

Status: complete.

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
occupied debug cells: red/orange semi-transparent wireframe cuboids
unknown debug cells: yellow semi-transparent wireframe cuboids
free debug cells: blue semi-transparent wireframe cuboids
occupancy cells are spatial/debug context only; do not label them as visual detections
summary OSD: OCC src=<source> occ=<n> free=<n> unk=<n> clear=<m>
```

For AirSim visual validation, render current camera coverage and visual obstacle evidence together:

```bash
simulation/airsim/run_mission.sh
# passes --show-sensing-volumes --show-obstacle-evidence to airsim-world-overlay.py by default
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

Status: complete.

Goal: make offline MP4/debug artifacts useful without overloading pixels.

Each `frame_XXXXXX.world_overlay.json` now carries occupancy summary and projected occupancy cells. The frame image should show only selected readable markers. The sidecar carries richer structured debug details.

Validator support:

```text
tools/mission/validate-mission-artifacts.py
  --expect-occupancy
  --expect-occupancy-sidecars
```

## Stage 4.0D — Swept-volume query visualization, dry-run only

Status: complete.

Goal: visualize the query before it changes flight.

Snapshot/artifact contract:

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
AirSim: swept segment, status OSD, blocker markers
MP4/sidecar: projected start/end and projected blocker cells
```

No command correction in this stage.

Validation:

```bash
python3 tools/mission/validate-mission-artifacts.py \
  out/object_behavior_airsim_existing_object_circle \
  --expect-complete \
  --expect-behavior \
  --expect-camera-pointing \
  --expect-camera-modes neutral,target,home,landing_area \
  --camera-frames-dir out/object_behavior_airsim_existing_object_circle/camera_pointing_frames \
  --expect-camera-proof-frames \
  --expect-occupancy \
  --expect-swept-volume \
  --safe-height-m 16 \
  --landed-height-m 1.0
```

Overlay evidence:

```bash
OVERLAY_LOG="$(ls -t simulation/airsim/logs/overlay_*.log | head -1)"
grep -nE "connected|SWEEP|swept_volume|Traceback|error|exception" -i "$OVERLAY_LOG" | tail -80
```

## Stage 4.0E — Swept-volume sidecar-only validation cleanup

Status: complete.

Goal: validate PPM/world-overlay sidecars without requiring a mission event log or snapshot JSON.

This supports deterministic offline validation of:

```text
PPM annotation -> frame_*.world_overlay.json -> validator
```

Validator command:

```bash
python3 tools/mission/validate-mission-artifacts.py \
  /tmp/dedalus_ppm_frame_annotation_sink_test \
  --expect-occupancy-sidecars \
  --expect-swept-volume-sidecars \
  --allow-missing-snapshots
```

Expected summary shape:

```text
snapshots: 0
occupancy_artifacts:
  sidecars_checked: 1
  projected_cells_checked: 1
swept_volume_artifacts:
  sidecars_checked: 1
  blocking_cells_checked: 1
failures: 0
```

Targeted tests:

```bash
ctest --test-dir build-staging --output-on-failure -R \
  'world_snapshot_json|ppm_frame_annotation_sink|mission_artifact_validator|replay_mission_smoke'
```

## Stage 4.1A — Avoidance advisory dry-run

Goal: compute what the avoidance layer would recommend, but still send nominal commands.

Add advisory-only events such as:

```text
avoidance_query
avoidance_would_correct
avoidance_clear
```

Visualization:

```text
nominal velocity arrow
advisory velocity arrow
reason and clearance in OSD/MP4 HUD
```

The runtime event stream already carries mission events, so dry-run advisory events should use that existing path.

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

The first pass may use object pose plus approximate bounding boxes/dimensions. If exact dimensions are unavailable, allow explicit overrides.
