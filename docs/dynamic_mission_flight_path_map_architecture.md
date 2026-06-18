# Dynamic mission flight-path map architecture

## Slice names

1. **S1 foundational traversability map**
   - Add a trajectory-independent `MissionLocalTraversabilityMap` built from compacted mission-local obstacle evidence.
   - Store classless occupied/free/unknown/stale belief, clearance, vertical-clearance, and planning-cost hints.

2. **S2 background map assimilator / compactor**
   - Add `MissionMapAssimilator` as a bounded, non-control-path background component.
   - Drain compact mission obstacle snapshots into the foundational map.
   - Keep raw/reflexive evidence temporary and keep the foundational map compact.

3. **S3 post-landing finalization barrier**
   - After landing/disarm, raise assimilation priority and wait for bounded map finalization before mission exit.
   - The mission should not silently discard sensed obstacle evidence; it should either compact it or preserve replayable evidence.

4. **S4 viewer foundational-map layers**
   - Stream/display the foundational map alongside raw live evidence.
   - Show age, confidence, stale/unknown cost, clearance bands, overhead/under-branch risk, and compaction status.

5. **S5 trajectory corridor evaluator**
   - Once a target trajectory exists, sample the foundational map around it.
   - Produce a clearance-aware nominal trajectory candidate and deviation/clearance metrics.
   - Keep this read-only/diagnostics-first until planner integration is explicitly scoped.

## Representation boundary

The dynamic mission map work must keep four data representations distinct. The
full representation and raw-evidence retention policy is maintained in:

```text
docs/mission_map_data_representations_and_retention.md
```

Summary:

```text
Detector raw evidence
  -> normalized source-attributed `ObstacleEvidence`
  -> temporary live input for emergency/reflexive avoidance and map update

Ego-relative reflexive avoidance map
  -> bounded `LocalFlightMapSnapshot` working set around the drone
  -> current hazard / emergency diagnostics and future reflexive control path
  -> not a persistent site map

Mission/site foundational planning map
  -> mission-derived traversability update
  -> future persistent site-frame planning memory
  -> queryable by planner/directive-specific cost functions

Visualization/debug/replay representation
  -> streamable/exportable view of evidence, trajectory, local map, and site map
  -> should not dictate runtime/control storage formats
  -> should eventually be served by sidecar/offline tooling, not only mission loop
```

Raw detector evidence is temporary. It can be forgotten only after it has served
both purposes:

1. emergency/reflexive avoidance during the active flight window,
2. mission/site map update for future planning.

Forgetting must be controlled by an explicit retention gate. Do not prune raw or
debug evidence merely because a queue is empty; require a successful compaction
or replayable output and a retention manifest.

## Runtime ownership

```text
Raw depth / geometry evidence
  -> reflexive obstacle avoidance
      high-rate, short-lived, emergency safety path

MissionLocalObstacleMap
  -> provider-neutral same-update compaction and primitive counters

MissionMapAssimilator
  -> bounded compaction into mission-derived traversability form

MissionLocalTraversabilityMap
  -> trajectory-independent mission-derived foundational map
      occupied/free/unknown/stale belief
      clearance and vertical-clearance fields
      age/confidence/cost hints

Persistent site traversability map, future/planned
  -> stable site-frame, cross-mission planning memory
      receives mission-derived traversability/obstacle updates
      supports planner directive-specific cost functions

TrajectoryCorridorEvaluator, future only
  -> samples foundational/site map after target trajectory is known
```

The foundational map is not a command sink and does not replace reflexive
avoidance. It is a long-lived planning/debug memory that ages gracefully, caps
evidence, and avoids storing raw depth detections forever. The current
`MissionLocalTraversabilityMap` is a mission-derived artifact; it is not yet the
complete cross-mission persistent site planning map.

## Current implementation boundary

The current implementation keeps runtime/control coupling out of scope. It adds:

- in-memory data structures,
- bounded assimilation semantics,
- post-landing flush status semantics,
- final traversability artifact writing,
- foundational-map HTML viewer and validator,
- unit tests.

Live viewer deltas for traversability, raw-evidence retention/pruning, offline
map serving, persistent site-map planner queries, and trajectory evaluation are
later slices unless explicitly scoped.
