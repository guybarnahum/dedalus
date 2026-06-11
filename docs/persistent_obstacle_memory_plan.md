# Persistent obstacle memory plan

## Purpose

Mission-local obstacle mapping accumulates classless depth/obstacle evidence in a stable takeoff-relative frame and derives the current ego-local `LocalFlightMapSnapshot` from that accumulated map.

The next layer is persistent obstacle memory: saving mission maps, merging them into a longer-lived site map, aging evidence carefully, and preloading relevant prior map evidence at the start of future missions.

This document records the post-5F/5G plan and the 5H/5H.1/5I implementation direction.

## Current status

Implemented and validated:

- AirSim depth obstacle evidence enters `PerceptionPipelineOutput.obstacle_evidence`.
- `MissionLocalObstacleMap` accumulates classless obstacle evidence in a mission-local map frame.
- `LocalFlightMapAccumulator::update_from_mission_local_map(...)` derives the current ego-local flight-map crop from mission-local evidence.
- `CoreStackRunner` wires depth evidence through mission-local accumulation and derives read-only local flight map / trajectory safety diagnostics.
- `WorldSnapshot` publishes `mission_local_obstacle_map` diagnostics with capped debug cells.
- `tools/visualization/mission_local_obstacle_viewer.py` provides offline mission-local map visualization.
- `tools/avoidance/export_mission_obstacle_map.py` exports mission obstacle map artifacts from snapshot diagnostics.
- Runtime full mission obstacle map artifact writing emits `mission_obstacle_map_full.json` directly from the full `MissionLocalObstacleMapSnapshot`.
- `tools/avoidance/merge_site_obstacle_map.py` is the planned/active 5I post-process tool for merging mission artifacts into persistent site memory.

Validated AirSim examples:

```text
WorldSnapshot diagnostic map:
  map_frame_id: map_airsim_mission_0001
  observed_cell_count: 40133
  occupied_cell_count: 16805
  update_count: 915
  debug_cells: 128

Full mission artifact:
  source_cells_are_debug_capped: false
  exported_cell_count: 41480
  mission_summary.observed_cell_count: 41480
  output: out/object_behavior_airsim_existing_object_circle/mission_obstacle_map_full.json
```

## Frame model

Use three map layers:

```text
mission-local map
  short-lived, one flight/run
  frame: takeoff / first trusted local pose

mission obstacle map artifact
  durable artifact emitted by a run
  frame: mission-local plus explicit site anchor metadata

persistent site obstacle map
  long-lived map merged across missions
  frame: site-local, AirSim world, ENU, RTK, SLAM map, or other stable site frame
```

Avoid calling the persistent map “global” until it has a real geodetic/world anchor. Prefer `site-local` or `persistent site map`.

Required transform relation:

```text
site_T_mission
  transforms mission-local cell centers into the persistent site frame
```

In AirSim this can initially be identity or the AirSim world/takeoff transform. In real flight it can later come from GPS/ENU, RTK, SLAM, fiducials, or another localization source.

## Persistent timestamp policy

Persistent maps must use absolute wall-clock timestamps with explicit units.

Use:

```text
time_unit: unix_ns
```

Mission-relative timestamps are acceptable only inside live runtime and per-frame snapshots. Persistent map artifacts should store absolute `unix_ns` times so age can be computed across missions.

Required site-level timestamps:

```text
created_at_unix_ns
updated_at_unix_ns
site_last_visited_unix_ns
```

Required mission-level timestamps:

```text
mission_start_unix_ns
mission_end_unix_ns
```

Required cell-level timestamps:

```text
first_seen_unix_ns
last_seen_unix_ns
last_confirmed_occupied_unix_ns
last_observed_free_unix_ns
last_in_sensor_frustum_unix_ns
```

Also retain primitive counts rather than only derived scores:

```text
positive_observation_count
negative_observation_count
mission_observation_count
source_stats
source_missions
```

Known current limitation: `mission_observation_count` is currently written as `1` per cell in the full runtime artifact, even when the accumulated cell score implies repeated hits. This is acceptable for the first site-map merge because raw scores and timestamps are preserved, but a later refinement should add true per-cell observation counters to `MissionLocalObstacleCell`.

## Decay philosophy

Do not blindly erase obstacle memory because calendar time passed.

Separate:

```text
persistent_score
  belief from historical evidence

freshness_score
  confidence modifier based on revisits and staleness

active_score
  score used for preload / local map / avoidance policy
```

Strong decay should be driven by contradiction or repeated revisit without confirmation, not by the site simply being unvisited.

Important ages:

```text
cell_age_seconds =
  now - cell.last_seen_unix_ns

site_staleness_seconds =
  now - site.site_last_visited_unix_ns

local_revisit_age_seconds =
  now - last time this cell/neighborhood was inside sensing coverage

contradiction_age_seconds =
  now - last explicit free/negative observation
```

Key normalization:

```text
relative_gap_seconds = max(0, cell_age_seconds - site_staleness_seconds)
```

If an entire site has not been visited for 30 days, `relative_gap_seconds` is near zero for most cells. Those cells should become stale but should not disappear.

If the site has been revisited repeatedly and one cell has not been reconfirmed, `relative_gap_seconds` grows and the cell can decay.

## Initial scoring model

Store primitive values and compute derived fields experimentally.

Suggested derived scoring:

```text
occupancy_score = sigmoid(occupied_log_odds)

freshness_score = exp(-relative_gap_days / 30)
freshness_score = clamp(freshness_score, 0.35, 1.0)

active_score = occupancy_score * freshness_score
```

Contradiction should be stronger:

```text
if last_observed_free_unix_ns > last_confirmed_occupied_unix_ns:
    active_score *= exp(-free_observations_since_occupied / 3)
```

This yields:

```text
old because site not visited -> mostly preserved
old despite site revisits -> decays
explicitly contradicted -> decays fast
```

## Cell states

Suggested derived states:

```text
active
  strong current or persistent evidence

stale
  old relative to site visits, not contradicted

suppressed
  recently contradicted by free-space evidence

probationary
  newly observed, low observation count

retired
  decayed/contradicted enough to ignore in active map, retained only for archive/debug
```

Avoidance policy should initially treat these as diagnostics only:

```text
active      -> can seed local flight map
stale       -> seed with reduced confidence
suppressed  -> do not block unless live sensor confirms
probationary-> use cautiously if close/high confidence
retired     -> ignore
```

## Artifact schemas

### Mission obstacle map

`mission_obstacle_map_full.json`:

```json
{
  "schema": "dedalus.mission_obstacle_map.v1",
  "time_unit": "unix_ns",
  "site_id": "airsim_neighborhood",
  "site_frame_id": "airsim_world",
  "mission_id": "object_behavior_airsim_existing_object_circle",
  "mission_map_frame_id": "map_airsim_mission_0001",
  "site_T_mission": {
    "position": {"x": 0, "y": 0, "z": 0},
    "rotation_rpy": {"x": 0, "y": 0, "z": 0}
  },
  "mission_start_unix_ns": 1781212111647270000,
  "mission_end_unix_ns": 1781212337423149000,
  "cell_size_m": 0.5,
  "vertical_cell_size_m": 0.5,
  "score_scale": 20,
  "mission_summary": {
    "observed_cell_count": 41480,
    "occupied_cell_count": 15677,
    "free_cell_count": 0,
    "update_count": 861
  },
  "export_summary": {
    "exported_cell_count": 41480,
    "source_cells_are_debug_capped": false
  },
  "cells": []
}
```

### Site obstacle map

`maps/<site_id>/site_obstacle_map.json`:

```json
{
  "schema": "dedalus.site_obstacle_map.v1",
  "time_unit": "unix_ns",
  "site_id": "airsim_neighborhood",
  "site_frame_id": "airsim_world",
  "created_at_unix_ns": 0,
  "updated_at_unix_ns": 0,
  "site_last_visited_unix_ns": 0,
  "cell_size_m": 0.5,
  "vertical_cell_size_m": 0.5,
  "merge_summary": {},
  "score_summary": {},
  "decay_policy": {
    "relative_gap_seconds_formula": "max(0, cell_age_seconds - site_staleness_seconds)",
    "freshness_half_life_days": 30.0,
    "freshness_floor": 0.35,
    "calendar_age_is_not_erasure": true
  },
  "cells": []
}
```

## Planned slices

### 5H — Export mission obstacle map artifact

Implemented as `tools/avoidance/export_mission_obstacle_map.py`.

Input:

```text
out/<run>/snapshot_*.json
```

Output:

```text
out/<run>/mission_obstacle_map.json
out/<run>/mission_obstacle_map.meta.json
```

This exporter is useful for schema/timestamp validation, but it reads `mission_local_obstacle_map.debug_cells` from snapshots and is therefore capped.

### 5H.1 — Runtime full mission obstacle map artifact writer

Implemented as a runtime artifact writer.

Output:

```text
out/<run>/mission_obstacle_map_full.json
out/<run>/mission_obstacle_map_full.json.meta.json
```

Properties:

```text
source_cells_are_debug_capped: false
exported_cell_count == mission_summary.observed_cell_count
```

This is the artifact that should feed persistent site-map merge.

### 5I — Merge mission map into persistent site map

Add or use `tools/avoidance/merge_site_obstacle_map.py`.

Inputs:

```text
out/<run>/mission_obstacle_map_full.json
maps/<site_id>/site_obstacle_map.json
```

Output:

```text
maps/<site_id>/site_obstacle_map.json
maps/<site_id>/site_obstacle_map.meta.json
```

Behavior:

```text
transform mission cells through site_T_mission
quantize into site grid
merge log-odds/counts/timestamps/source stats
write atomically
compute derived site-relative freshness fields
```

Important caveat: repeated merges of the same mission artifact should not double-count evidence. The merge tool should either track merged mission ids globally or track `source_missions` per cell.

### 5J — Score / age persistent site map

Add or refine `tools/avoidance/score_site_obstacle_map.py`.

Compute derived fields:

```text
age_seconds
site_staleness_seconds
relative_gap_seconds
site_relative_age_percentile
freshness_score
active_score
status
```

Do not overwrite raw evidence primitives. Allow formula experiments without losing history.

### 5K — run_mission.sh post-process

Add optional post-process flags:

```text
--site-id <id>
--export-obstacle-map
--merge-obstacle-map
--site-map-path maps/<site_id>/site_obstacle_map.json
```

or equivalent config entries.

At first, post-process after the mission. Later, add periodic/streaming writers.

### 5L — Runtime preload from site map

At mission startup:

```text
load persistent site map
crop near takeoff
transform site cells into current mission-local frame
seed MissionLocalObstacleMap
derive LocalFlightMapSnapshot
keep read-only diagnostics first
```

Only after validation should this feed planner/control.

### 5M — Streaming map deltas

Emit lightweight map snapshots/deltas during flight:

```text
mission_obstacle_map_latest.json.tmp
atomic rename to mission_obstacle_map_latest.json
```

or JSONL event stream.

This enables a viewer or external map service to watch the accumulating map while flying without coupling persistence to command sinks.

### 5N — Planner integration

Only after persistent map confidence is validated:

```text
active/stale prior map
  -> seeded local flight map
  -> trajectory safety
  -> avoidance/replanning policy
```

No command blocking or replanning should be added until explicitly requested.

## Non-goals

- Do not store only a single decayed score.
- Do not delete obstacles solely because the site was not visited recently.
- Do not require geodetic/global coordinates for AirSim validation.
- Do not couple map persistence to flight command sinks.
- Do not use semantic object-GT as the obstacle memory source.
- Do not make persistence a runtime database before the post-process tools are proven.
- Do not expand every per-frame `snapshot_*.json` with the full obstacle map.
