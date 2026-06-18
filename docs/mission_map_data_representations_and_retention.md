# Mission map data representations and raw-evidence retention

## Purpose

This document defines the representation boundaries for obstacle evidence, live
avoidance maps, durable planning maps, and visualization/debug exports. It also
records the retention gate for forgetting raw detector evidence after it has
served both safety and map-update purposes.

The goal is to avoid conflating four different data products:

1. detector raw evidence,
2. ego-relative reflexive avoidance state,
3. persistent site/foundational planning memory,
4. visualization/debug/replay representation.

Raw evidence is valuable during flight, but it should not become the long-lived
site memory. Once it has been consumed by the live safety path and compacted into
an authoritative map update, it can be forgotten or reduced to a replay/debug
sample according to an explicit retention policy.

## Representation 1: detector raw evidence

Current normalized contract:

- `dedalus::ObstacleEvidence`
- carried by `PerceptionPipelineOutput::obstacle_evidence`
- serialized in `WorldSnapshot::obstacle_evidence` for diagnostics

Current and planned producers:

- AirSim ground-truth visual emulation,
- AirSim depth obstacle detector / depth provider,
- future visual obstacle detector,
- future fused sources.

Role:

```text
sensor/source output
  -> normalized geometric evidence
  -> live emergency/reflexive avoidance input
  -> mission/local map update input
```

Properties:

- temporary,
- high-rate/high-volume,
- source attributed,
- provider-neutral enough to feed map update code,
- not a long-lived planning map,
- not the representation a mission planner should query directly.

Retention intent:

Raw detector evidence should survive only long enough to satisfy:

1. live reflexive/emergency avoidance,
2. mission-local and site/foundational map update,
3. validation/debug retention policy.

It should not be kept forever merely because it was observed.

## Representation 2: ego-relative reflexive avoidance map

Current implementation:

- `LocalFlightMapAccumulator`
- `LocalFlightMapSnapshot`
- `TrajectorySafetyEvaluator` diagnostics

Role:

```text
live evidence and/or compact mission-local cells
  -> bounded ego-relative map around the drone
  -> immediate hazard / emergency diagnostics
```

Properties:

- ego-relative or mission-local cropped around current ego pose,
- bounded in range and memory,
- low-latency,
- current-flight/current-hazard oriented,
- should be safe to reset/recompute from compact current evidence,
- not a persistent site map,
- not a cross-mission planning database.

Safety boundary:

This map is the correct place for emergency/reflexive behavior. The persistent
site/foundational map may inform planning later, but it must not replace live
reflexive obstacle avoidance.

Current state:

The runtime already derives local flight-map snapshots from mission-local map
evidence and runs read-only trajectory-safety diagnostics. This is still not a
closed-loop command sink unless explicitly scoped later.

## Representation 3: persistent site / foundational planning map

Current related implementations:

- `MissionLocalObstacleMap`
- `MissionMapAssimilator`
- `MissionLocalTraversabilityMap`
- `MissionTraversabilityMapArtifactWriter`
- `tools/avoidance/merge_site_obstacle_map.py`
- `maps/<site_id>/site_obstacle_map.sqlite` / JSON export path

Important distinction:

`MissionLocalTraversabilityMap` is currently a mission-derived foundational
artifact. It is useful and validated, but it is not yet the complete cross-mission
site planning map. The true durable planning layer should be a stable site-frame
map that can be merged across missions and queried by a planner.

Role:

```text
mission-local compact obstacle/map evidence
  -> mission-derived traversability update
  -> persistent site-frame planning map
  -> future mission trajectory planner queries
```

Planner-facing intent:

The planner should eventually query the site/foundational map using directive
specific cost functions, for example:

- high and safe,
- low and stealthy,
- smooth and energy saving,
- clearance maximizing,
- risk minimizing.

Those directives should change scoring/query policy, not mutate raw evidence.

Required durable-map properties:

- stable `site_id`,
- stable `site_frame_id`,
- explicit `site_T_mission` for mission-derived updates,
- idempotent mission merge behavior,
- primitive counts retained separately from derived scores,
- timestamps suitable for cross-mission aging,
- contradiction/free-space handling,
- status fields such as active/stale/suppressed/probationary/retired,
- queryable cost fields for planner experiments.

Current state:

The current AirSim-validated path writes:

```text
out/<mission>/mission_traversability_map_full.json
out/<mission>/mission_traversability_map_full.json.meta.json
```

from the normal `run_mission.sh --merge-obstacle-map` path. This proves that raw
mission evidence can be compacted into a mission-local foundational artifact. It
does not yet prove that a durable site planning map has been updated unless the
site merge step is also run and validated.

## Representation 4: visualization/debug/replay representation

Current implementations:

- `tools/visualization/mission_local_obstacle_viewer.py`
- `tools/visualization/mission_traversability_map_viewer.py`
- runtime HTTP/SSE/static server (`/healthz`, `/events`, static root)
- per-frame `snapshot_*.json`
- mission delta JSONL/SQLite artifacts

Role:

```text
runtime/debug data products
  -> live viewer and offline replay
  -> operator validation and analysis
```

Properties:

- may be verbose,
- may contain raw or sampled evidence,
- may contain trajectory and ego state,
- may contain derived map layers,
- should not dictate runtime/control storage formats,
- should not be the only durable planning memory.

Serving model:

The mission loop may continue to serve a lightweight live validation viewer, but
longer-term map/debug visualization should be available when the drone and
mission loop are offline. Prefer a sidecar/offline map server or static viewer
that reads durable artifacts and site maps from disk.

Future desired tool/service:

```text
tools/visualization/serve_mission_map.py
or apps/dedalus_map_server
```

with layers for:

- raw detector evidence summaries/samples,
- ego trajectory,
- current drone state when live,
- sensing/camera vectors,
- ego-relative local flight map,
- mission-local obstacle cells,
- mission-derived traversability map,
- persistent site/foundational map,
- derived visual footprints/boundaries/contours.

## Retention / forgetting gate

Do not delete raw evidence merely because the assimilator queue is empty. Raw
evidence can be forgotten only after the system has satisfied both safety and
persistence requirements.

A conservative retention gate is:

```text
raw evidence is forgettable if:
  active flight/emergency window is clear
  and mission-local evidence has been compacted
  and mission-derived traversability output has been persisted
  and either:
      a replayable mission delta/artifact stream is retained
      or the persistent site map merge has succeeded idempotently
  and a retention manifest records the decision
```

Initial implementation should be dry-run only. It should write a manifest that
reports what would be retained or removed, without deleting anything.

Suggested manifest:

```json
{
  "schema": "dedalus.mission_evidence_retention.v1",
  "site_id": "validate_r3b1",
  "mission_id": "validate_r3b1_mission",
  "raw_evidence_forget_state": "dry_run",
  "reason": "foundational_map_finalized",
  "active_emergency_window_clear": true,
  "mission_local_compaction_complete": true,
  "traversability_artifact_written": true,
  "site_map_merge_succeeded": false,
  "replayable_delta_stream_retained": true,
  "can_forget_raw_evidence": true,
  "raw_snapshots_before": 968,
  "raw_snapshots_retained": 10,
  "raw_snapshots_would_remove": 958,
  "retained_outputs": [
    "mission_traversability_map_full.json",
    "mission_traversability_map_full.json.meta.json",
    "mission_obstacle_map_deltas.sqlite",
    "obstacle_memory_manifest.json"
  ]
}
```

Conservative first retention policy:

Retain:

- mission traversability artifact and meta file,
- mission obstacle delta SQLite/JSONL until site merge is fully trusted,
- obstacle memory manifest,
- validation logs,
- first snapshot,
- final snapshot,
- every Nth snapshot for debug/replay, configurable.

Do not retain indefinitely:

- every per-frame `snapshot_*.json`,
- raw-heavy debug images/frames unless explicitly requested,
- temporary detector-specific dumps.

Deletion must remain behind an explicit flag until dry-run manifests are
validated.

Suggested knobs:

```text
DEDALUS_MISSION_EVIDENCE_RETENTION=0/1
DEDALUS_MISSION_EVIDENCE_RETENTION_DRY_RUN=1
DEDALUS_MISSION_EVIDENCE_RETENTION_DELETE_SNAPSHOTS=0/1
DEDALUS_MISSION_EVIDENCE_RETENTION_KEEP_EVERY_N=100
```

## Implementation order

1. Keep the four representation boundaries explicit in code and docs.
2. Add a dry-run retention manifest generator.
3. Validate it on AirSim mission output without deleting files.
4. Add optional pruning only after the manifest is trusted.
5. Only then consider stronger deletion once persistent site-map merge is
   idempotent and planner-facing durable memory is validated.

## Current status summary

Implemented:

- normalized detector evidence contract,
- mission-local obstacle-map compaction,
- ego-local flight-map diagnostic projection,
- mission traversability artifact writer,
- artifact viewer and validator,
- runtime defaulting so `run_mission.sh --merge-obstacle-map` emits the
  traversability artifact.

Not implemented yet:

- dry-run raw evidence retention manifest,
- active artifact pruning/deletion,
- explicit emergency-window retention holdback,
- true cross-mission site traversability planner map,
- offline/sidecar unified map server,
- planner directive cost querying.
