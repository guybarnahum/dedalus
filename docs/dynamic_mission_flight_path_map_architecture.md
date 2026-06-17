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

## Runtime ownership

```text
Raw depth / geometry evidence
  -> reflexive obstacle avoidance
      high-rate, short-lived, emergency safety path

MissionLocalObstacleMap
  -> provider-neutral same-update compaction and primitive counters

MissionMapAssimilator
  -> background bounded compaction into persistent/planning-friendly form

MissionLocalTraversabilityMap
  -> trajectory-independent foundational map
      occupied/free/unknown/stale belief
      clearance and vertical-clearance fields
      age/confidence/cost hints

TrajectoryCorridorEvaluator, future only
  -> samples foundational map after target trajectory is known
```

The foundational map is not a command sink and does not replace reflexive avoidance. It is a long-lived planning/debug memory that ages gracefully, caps evidence, and avoids storing raw depth detections forever.

## First implementation boundary

The first implementation keeps runtime/control coupling out of scope. It adds only:

- in-memory data structures,
- bounded assimilation semantics,
- post-landing flush status semantics,
- unit tests.

Persistence writers, live viewer deltas, runtime shutdown barriers, and trajectory evaluation are intentionally later slices.
