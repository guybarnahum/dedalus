# Obstacle memory plugin architecture

## Purpose

Obstacle memory has two conflicting needs:

1. The flight stack needs low-latency, deterministic, in-memory map access.
2. Persistent memory needs compact, durable, queryable storage across missions.

The architecture therefore uses a three-tier design with stable contracts between tiers. Backend choices such as JSON, SQLite, LMDB, or custom binary/mmap must be replaceable without changing runtime map users, behavior code, or visualization consumers.

## Three-tier design

```text
Live obstacle evidence
  -> Tier A: hot runtime maps
  -> Tier B: mission delta stream
  -> Tier C: persistent site memory
  -> Debug/export codecs
```

### Tier A — hot runtime maps

Tier A is the real-time, in-memory obstacle map used during flight.

Current examples:

```text
MissionLocalObstacleMap
LocalFlightMap
TrajectorySafety diagnostics
```

Requirements:

```text
- No blocking disk I/O in the hot path.
- Deterministic bounded work per frame.
- Efficient local spatial lookup.
- Suitable for diagnostics now and future avoidance policy later.
- Stores only the active/cropped runtime working set.
```

Non-goals:

```text
- Long-term persistence.
- Full-site scan.
- Human-readable serialization.
```

Candidate implementations:

```text
- Current C++ MissionLocalObstacleMap.
- Sparse voxel hash map.
- flat_hash_map keyed by quantized cell coordinates.
- Rolling local grid / ring buffer.
- Future GPU or SIMD-backed grid.
```

### Tier B — mission delta stream

Tier B is the bridge between live runtime maps and durable site memory.

It should receive compact changed-cell batches from Tier A and append them asynchronously or in coarse batches.

Requirements:

```text
- Append-only during flight.
- Crash-tolerant.
- Does not block the flight loop.
- Can be replayed into Tier C.
- Can be disabled or replaced by a debug sink.
```

Important rule:

```text
Do not write the full persistent site map from the control loop.
```

Candidate implementations:

```text
- Debug JSONL delta log.
- SQLite mission delta table.
- LMDB append/update store.
- Custom binary log.
```

### Tier C — persistent site memory

Tier C stores durable cross-mission obstacle memory.

Requirements:

```text
- Efficient spatial query by site and cell bounds.
- Efficient status query: active, stale, probationary, suppressed, retired.
- Idempotent mission merges.
- Durable and crash-safe.
- Supports scoring/aging independent from raw evidence.
- Supports debug export/import.
```

Candidate implementations:

```text
- Current debug JSON site map.
- SQLite site map store.
- LMDB key-value store.
- RocksDB/LevelDB for very large logs.
- Custom binary/mmap format for long-term optimized runtime use.
```

## Format policy

There are two format classes.

### Efficient default format

Used by runtime preload, persistent storage, scoring, and post-mission merge.

Examples:

```text
maps/<site_id>/site_obstacle_map.sqlite
out/<mission>/mission_obstacle_deltas.sqlite
out/<mission>/mission_obstacle_deltas.binlog
```

Properties:

```text
- Compact.
- Indexed or directly addressable.
- Fast to parse/load.
- Not necessarily human-readable.
```

### Debug/export format

Used for inspection, tests, visualization, and interoperability.

Examples:

```text
maps/<site_id>/site_obstacle_map.debug.json
maps/<site_id>/site_obstacle_map.debug.json.gz
out/<mission>/mission_obstacle_map_full.debug.json
out/<mission>/mission_obstacle_deltas.debug.jsonl
```

Properties:

```text
- Human-readable or universal.
- jq-friendly when JSON.
- Not the default runtime storage.
- Generated on demand by conversion tools.
```

Current 5H-5K JSON files are debug/export artifacts. They are valuable for validation and review, but they should not remain the default persistent runtime format.

## Interface contracts

The rest of the system should depend on contracts, not storage backends.

### Hot runtime map contract

```text
ingest live obstacle evidence
query/crop local region
produce snapshot/changed-cell batch
```

The current MissionLocalObstacleMap already fulfills most of this role. Future work should avoid exposing storage details to it.

### Delta sink contract

```text
open mission delta stream
append changed-cell batch
flush/close
```

Implementations may write JSONL, SQLite, LMDB, or binary logs.

### Persistent store contract

```text
open site map
merge mission deltas or full mission artifact
load region by bounds/status
score derived state
export/import debug representation
```

The store must expose query semantics, not file-format semantics.

## Runtime access policy

The flight loop should access only Tier A memory structures.

Allowed during flight:

```text
- Update in-memory mission/local maps from live evidence.
- Append compact delta batches asynchronously.
- Query a preloaded/cropped persistent region through an in-memory cache.
```

Not allowed in the flight hot path:

```text
- Parse full JSON maps.
- Rewrite full site map files.
- Run large SQL scans synchronously.
- Block on compaction/export.
```

SQLite or LMDB can be used by a background/preload path, but the behavior/controller path should see an in-memory obstacle working set.

## Persistence and scoring policy

Raw evidence is preserved separately from derived state.

Raw evidence examples:

```text
occupied_log_odds
free_log_odds
observation counts
first_seen_unix_ns
last_seen_unix_ns
source mission ids
source providers
```

Derived state examples:

```text
occupancy_score
freshness_score
active_score
status
site_relative_age_percentile
relative_gap_seconds
```

Derived scoring may be recomputed without changing raw evidence.

Calendar age alone is not erasure:

```text
relative_gap_seconds = max(0, cell_age_seconds - site_staleness_seconds)
```

Strong decay/suppression should come from contradiction, revisits without reconfirmation, or newer free-space evidence.

## Backend choice guidance

### SQLite

Good first efficient backend.

Use for:

```text
- Persistent site map.
- Mission delta tables.
- Region preload queries.
- Post-mission merge and scoring.
```

Do not use for:

```text
- Per-frame occupancy lookups in the control loop.
```

### LMDB

Strong candidate for read-heavy persistent maps with compact binary values.

Use when:

```text
- Direct key-value lookups dominate.
- We want mmap-backed fast reads.
- We are comfortable building our own spatial/status indexes.
```

### Custom binary/mmap

Best long-term size/runtime efficiency, higher engineering cost.

Use when:

```text
- SQLite/LMDB overhead becomes limiting.
- The cell schema stabilizes.
- We need fast preload of millions of cells with minimal parsing.
```

### JSON / JSONL

Use as debug/export only.

Good for:

```text
- jq inspection.
- development validation.
- viewer input.
- small fixtures.
```

Bad for:

```text
- default persistent storage at scale.
- random spatial queries.
- runtime preload of large maps.
```

## Near-term implementation plan

### 5L.0 — contracts and docs

Add stable C++ interfaces for:

```text
ObstacleDeltaSink
PersistentObstacleStore
DebugObstacleMapCodec
```

No backend implementation yet.

### 5L.1 — SQLite site store

Implement `SQLitePersistentObstacleStore` as the first efficient backend.

### 5L.2 — conversion tools

Add debug conversion:

```text
sqlite -> debug JSON
debug JSON -> sqlite
```

### 5M — streaming mission deltas

Runtime appends changed-cell batches to a delta sink.

### 5N — diagnostics-only preload

Preload active/probationary cells from persistent store into mission-local/local-flight diagnostics.

### 5O — backend alternatives

Prototype LMDB or custom binary/mmap behind the same contracts.


### 5L.1 — SQLite site store prototype

The first efficient backend prototype is `tools/avoidance/site_obstacle_map_sqlite.py`.

It is intentionally a tool/backend prototype and is not yet used by the flight loop. It supports:

```text
import-json    debug JSON site map -> SQLite
score          recompute derived scores inside SQLite
summary        inspect cell/status counts and DB size
query-region   query indexed site/cell bounds and status
export-json    SQLite -> debug JSON
```

This validates the efficient/default storage direction without changing runtime behavior. Runtime integration should come later through the `PersistentObstacleStore` contract.


### 5L.2 — direct SQLite post-mission merge

The efficient post-mission merge path is `tools/avoidance/merge_site_obstacle_map_sqlite.py`.

It consumes full mission debug artifacts and updates the SQLite site store directly:

```text
out/<run>/mission_obstacle_map_full.json
  -> maps/<site_id>/site_obstacle_map.sqlite
```

This avoids writing the large debug JSON site map during normal persistence. JSON remains available through explicit export:

```text
tools/avoidance/site_obstacle_map_sqlite.py export-json
```

Runtime integration is still deferred. The tool validates the efficient default format and the storage contract before wiring SQLite into `run_mission.sh` or C++ preload paths.


### 5M.1 — compact persistence-essential delta payload

The mission obstacle delta JSONL stream now emits a compact v2 payload. Each changed cell carries only persistence-essential fields:

```text
center_mission
occupied_score
free_score
confidence
first_seen_unix_ns
last_seen_unix_ns
source_kind
source_provider
```

The stream intentionally omits debug/derived fields such as thresholded occupied/free booleans, normalized scores, log odds, risk score, repeated cell size, and placeholder observation counters. These can be recomputed during replay, scoring, or backend-specific compaction. This keeps the Tier-B stream focused on durable evidence rather than runtime visualization state.


### 5N — SQLite mission delta log

The compact mission obstacle delta JSONL stream can now be imported into a SQLite-backed Tier-B delta log:

```text
tools/avoidance/mission_obstacle_delta_sqlite.py import-jsonl \
  out/<run>/mission_obstacle_map_deltas.jsonl \
  --db out/<run>/mission_obstacle_map_deltas.sqlite \
  --replace
```

This database is not the persistent site map. It is a replay/queryable storage backend for the append-only mission delta stream. It validates the Tier-B persistence boundary before wiring delta replay directly into Tier-C site-map compaction.


### 5O — compact delta SQLite to persistent site SQLite

The Tier-B delta SQLite log can now be compacted into the Tier-C persistent site SQLite map without reading the full mission JSON artifact:

```text
tools/avoidance/merge_site_obstacle_map_from_delta_sqlite.py \
  --delta-db out/<run>/mission_obstacle_map_deltas.sqlite \
  --site-db maps/<site_id>/site_obstacle_map.sqlite \
  --site-id <site_id> \
  --site-frame-id <site_frame_id>
```

This keeps the runtime unchanged while proving the persistence path:

```text
runtime in-memory map -> compact JSONL delta stream -> delta SQLite log -> persistent site SQLite
```

The merge is idempotent at the mission-id level. If all mission IDs from the delta log are already present in the site database, a repeated merge skips without duplicating cells.


### 5P — post-mission merge from delta SQLite by default

The AirSim post-mission merge path now uses the compact delta stream for SQLite site memory by default:

```text
mission_obstacle_map_deltas.jsonl
  -> mission_obstacle_map_deltas.sqlite
  -> maps/<site_id>/site_obstacle_map.sqlite
```

The full mission JSON artifact remains available as a debug/export artifact and remains the source for `--site-map-format json`. The legacy full-JSON-to-SQLite merge path is retained under `sqlite-full-json` for comparison and debugging.


### 5Q — full mission JSON artifact is debug-only by default

The default AirSim post-mission SQLite path no longer requires the full mission obstacle JSON artifact. By default, runtime emits the compact delta stream and post-mission processing uses:

```text
mission_obstacle_map_deltas.jsonl
  -> mission_obstacle_map_deltas.sqlite
  -> maps/<site_id>/site_obstacle_map.sqlite
```

The large full mission JSON artifact is debug/export-only. It is enabled explicitly with:

```text
--write-full-obstacle-map-artifact
```

It is also enabled automatically for formats that require the full JSON artifact:

```text
--site-map-format json
--site-map-format both
--site-map-format sqlite-full-json
```
