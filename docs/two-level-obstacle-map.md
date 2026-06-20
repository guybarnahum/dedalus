# Two-Level Obstacle Map Architecture

## Overview

Dedalus maintains two complementary obstacle maps with different roles, lifetimes, and resolution. Together they give the system both the speed it needs for reflexive in-flight avoidance and the persistence it needs for flight planning across missions.

```
Depth sensor evidence
        │
        ▼
┌──────────────────────────────┐
│  MissionLocalObstacleMap     │  raw per-tick accumulator, ego-local crop
│  (obstacle cells, ~0.5 m)    │  → LocalFlightMapSnapshot → TrajectorySafetyEvaluator
└───────────────┬──────────────┘
                │  assimilator drains periodically
                ▼
┌──────────────────────────────┐
│  Level 1: Traversability Map │  mission-local, time-decaying accumulator
│  MissionLocalTraversabilityMap  │  0.5 m voxels, all states
│  (per-flight, ~20 s memory)  │
└───────────────┬──────────────┘
                │  update_from_traversability()
                ▼
┌──────────────────────────────┐
│  Level 2: Planning Map       │  cross-mission persistent obstacle store
│  MissionLocalPlanningMap     │  1 m × 1 m × 2 m voxels, occupied only
│  (no time decay, disk-backed)│
└──────────────────────────────┘
```

The `LocalFlightMap` / `TrajectorySafetyEvaluator` pipeline that drives the existing reflexive avoidance is a separate ego-local crop of the obstacle map (not shown in the two-level numbering) and is described in `reflexive_obstacle_avoidance_architecture.md`.

---

## Level 1 — Traversability Accumulator

**Class:** `MissionLocalTraversabilityMap`  
**Header:** `include/dedalus/avoidance/mission_local_traversability_map.hpp`

### What it is

Level 1 is a mission-local voxel accumulator that takes raw obstacle evidence from the depth detector and builds a persistent 3-D occupancy picture for the current flight. It stores all evidence states: Occupied, ObservedFree, Unknown, Mixed, Stale.

It is the primary source for the flight-planning layer and for the viewer's traversability overlay.

### Resolution

| Axis | Cell size |
|------|-----------|
| XY   | 0.5 m     |
| Z    | 0.5 m     |

At a typical flight area of 100 m × 100 m × 20 m this is ~800 K potential cells; in practice the occupied set is much smaller because only observed cells are stored.

### Score update rule

When a new obstacle observation arrives, L1 max-merges the evidence score:

```
cell.occupied_score = max(cell.occupied_score, source.occupied_score)
```

This means evidence can only strengthen a cell through direct observation, not weaken it passively.

### Decay

L1 has configurable time-based decay so that evidence fades when a region leaves the sensor field of view:

```cpp
occupied_score_decay_per_second = 0.05   // decays 1.5 → 0.1 in ~28 s
```

This is intentional: L1 is a *recent-evidence* map. A cell the drone flew past 30 seconds ago should fade unless re-observed, because the world may have changed. Moving obstacles (people, vehicles) are handled naturally by decay without explicit tracking.

### Pruning

Cells whose `occupied_score` falls below `prune_min_occupied_score` (default 0.1) are evicted from the vector and index. Pruning runs every `prune_interval_ticks` updates (default every 10) to amortise the O(N) compaction cost.

### Lifecycle

| Event | Action |
|-------|--------|
| Mission start | L1 starts empty (or optionally reset) |
| Each tick | `apply_aging()` → ingest new evidence → `prune_weak_cells()` |
| Between missions (parked) | Scores continue to decay — this is expected |
| Mission end / landing | `finalize_mission_map_after_landing()` flushes and writes artifact |

### State published to viewer

L1 is what the viewer's traversability overlay shows. It is streamed over SSE as `traversability_map_snapshot` (full) and `traversability_map_delta` (incremental), throttled to 1 publish per 2 s.

---

## Level 2 — Persistent Planning Map

**Class:** `MissionLocalPlanningMap`  
**Header:** `include/dedalus/avoidance/mission_local_planning_map.hpp`

### What it is

Level 2 is the long-lived obstacle store used for flight path planning. It accumulates confirmed obstacle evidence from L1 but is never erased by time — only by direct free-space observation. It survives across power cycles via disk persistence.

### Resolution

| Axis | Cell size | vs L1 |
|------|-----------|-------|
| XY   | 1.0 m     | 2×    |
| Z    | 2.0 m     | 4×    |

Each L2 voxel covers 1 m × 1 m × 2 m = 2 m³, which is 16× the volume of an L1 voxel. A typical occupied L1 set of ~38 K cells projects to ~3–6 K L2 cells.

### Evidence update rule (incremental, not rebuild)

`update_from_traversability()` applies per-cell evidence rules rather than rebuilding from scratch:

```
L1 Occupied (score >= min_occupied_score)
    → max-merge occupied_score + confidence into L2 voxel
    → create voxel if it doesn't exist

L1 ObservedFree
    → reduce L2 voxel score: new_score = old_score * (1 - free_evidence_weight)
    → evict L2 voxel if score drops below min_occupied_score

L1 Unknown / Stale / absent
    → no change to L2
```

The critical property: **absence of observation does not clear a cell**. If the drone does not fly past a region this mission, its obstacles remain in L2 from prior missions. Only confirmed free-space observation removes them.

### Free-space eviction dynamics

`free_evidence_weight = 0.5` (default) means:

- A cell at score 1.5 (barely occupied) clears after **1** free observation
- A cell at score 6.0 (well-confirmed) clears after **3** free observations
- A cell at score 15.0 (max evidence) clears after **4** free observations

This gives well-mapped regions a higher "inertia" that resists false-free readings from sensor noise.

### No time decay

L2 cells do not decay with time. A building mapped last month is still there next month unless the drone explicitly observes free space at that location. This is correct for static environments.

For environments with known changes (construction, temporary structures) the map can be partially or fully reset via `reset()` at mission start if required.

### Lifecycle

| Event | Action |
|-------|--------|
| Process start | Load from `planning_map_persistence_path` if file exists |
| Between missions (parked) | Cells unchanged — no time decay |
| Each tick (L1 updated) | `update_from_traversability(l1_snapshot)` — incremental evidence rule |
| Mission end / landing | `save_to_file()` — atomic write to disk |

### Disk persistence format

Plain-text, one cell per line after a version header:

```
planning_map_v1 cell_size=1 vcell_size=2 min_score=1 free_weight=0.5
<cx> <cy> <cz> <occupied_score> <confidence> <source_cell_count>
...
```

The file is written via temp-then-rename so a partial write never corrupts the prior mission's map.

### Configuration

| Parameter | Default | Description |
|-----------|---------|-------------|
| `cell_size_m` | 1.0 m | XY voxel size |
| `vertical_cell_size_m` | 2.0 m | Z voxel size |
| `min_occupied_score` | 1.0 | Score floor; cells below this are evicted |
| `free_evidence_weight` | 0.5 | Fraction of score removed per free observation |
| `planning_map_persistence_path` | `""` | If empty, persistence is disabled |

---

## Evidence Flow

```
Depth frame
    │
    │ AirSimDepthObstacleDetector.detect()
    ▼
ObstacleEvidence[]
    │
    │ MissionLocalObstacleMap.update()
    ▼
MissionLocalObstacleMapSnapshot
    │                      │
    │ assimilator           │ MissionObstacleMapDeltaWriter → SSE stream
    │ .enqueue()            │   (raw obstacle overlay in viewer)
    ▼
MissionMapAssimilator.tick()
    │
    │ traversability_map_.update_from_mission_obstacle_map()
    ▼
MissionLocalTraversabilityMap  [Level 1]
    │
    │ update_from_traversability()        also: traversability_map_publisher_ → SSE
    ▼
MissionLocalPlanningMap  [Level 2]
    │
    │ (future) path planner queries
    ▼
Flight plan
```

---

## Viewer Extension — Ego L1 View (Reflexive Avoidance)

The viewer currently shows the full L1 traversability overlay. A planned extension will add a second rendering layer showing only the **ego-local window** of L1 — the bounded crop around the drone that drives reflexive emergency avoidance.

This ego L1 view will:

- Show a fixed-radius sphere or box around the drone position (e.g. 20 m)
- Use a distinct color scheme to distinguish from the global L1 overlay (red intensity proportional to proximity, not height)
- Update at full SSE rate (not the 2 s throttle) to reflect real-time obstacle state
- Be toggleable independently from the global trav overlay

This view gives operators direct visibility into the information the reflexive avoidance layer is acting on, making emergency stop and avoidance correction events interpretable without post-hoc log analysis.

The ego L1 view is connected to the `LocalFlightMapSnapshot` / `TrajectorySafetyEvaluator` pipeline, not the full L1 accumulator, since that is what drives the actual avoidance commands.

---

## Configuration Reference

### Level 1 (`MissionLocalTraversabilityMapConfig`)

| Parameter | Default | Notes |
|-----------|---------|-------|
| `cell_size_m` | 0.5 m | XY voxel size |
| `vertical_cell_size_m` | 0.5 m | Z voxel size |
| `occupied_threshold` | 1.0 | Score to classify cell as Occupied |
| `occupied_score_decay_per_second` | 0.05 | Time decay rate |
| `prune_min_occupied_score` | 0.1 | Score floor below which cells are evicted |
| `prune_interval_ticks` | 10 | Prune every N assimilator ticks |
| `stale_after_seconds` | 300 | Age flag threshold (does not prune) |

### Level 2 (`MissionLocalPlanningMapConfig`)

| Parameter | Default | Notes |
|-----------|---------|-------|
| `cell_size_m` | 1.0 m | XY voxel size |
| `vertical_cell_size_m` | 2.0 m | Z voxel size |
| `min_occupied_score` | 1.0 | Project only L1 cells at/above this |
| `free_evidence_weight` | 0.5 | Fraction of score cleared per free obs |

---

## Future Work

- **Path planner integration:** wire L2 snapshot into the trajectory planner so pre-mission route computation uses the persistent obstacle map.
- **L2 SSE streaming:** publish L2 cells to the viewer as a separate layer (much smaller than L1, suitable for a higher refresh rate).
- **Ego L1 reflexive view:** viewer overlay showing the ego-local crop driving emergency avoidance.
- **L2 selective reset:** API to clear L2 cells within a bounding box (useful after known environment changes).
- **Confidence-weighted aggregation:** replace max-merge with a weighted Bayesian update for better handling of low-confidence sensors.
- **Multi-resolution L2:** separate horizontal and vertical resolution to reflect the asymmetric cost of vertical vs horizontal route changes.
