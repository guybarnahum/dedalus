# Mission-local obstacle mapping architecture

## Purpose

Obstacle evidence is produced from camera/depth observations while the drone is moving. A local flight map cannot simply keep adding evidence in the current ego frame; doing so would smear obstacles as the ego pose changes.

The mapping layer uses a stable **mission-local** frame, anchored at takeoff or the first trusted local pose, and derives a current ego-local crop from that accumulated map.

## Existing contracts reused

This design intentionally reuses existing Dedalus contracts rather than adding parallel types:

- `Pose3` represents pose as `position` plus `rotation_rpy`.
- `EgoState.local_T_body` is the current body pose in the local/map frame.
- `EgoState.map_frame_id` names the active local frame.
- `FramePacket` carries per-frame camera/depth/intrinsics/extrinsics/ego hints.
- `ObstacleEvidence` is the provider-neutral classless obstacle contract.

## Frames

### Camera frame

The sensor/depth frame. Raw depth pixels, rays, and visual obstacle detector products start here.

### Body frame

The drone body frame at the capture time.

### Mission-local map frame

A stable local frame for the mission. This is normally anchored at takeoff / first trusted local odometry pose. It is not a global geodetic map and does not need to survive across missions.

Obstacle evidence should be accumulated here.

### Current ego frame

The current body-centered planning frame. The local flight map is a crop/view of the mission-local map transformed into this current ego frame.

## Transform chain

For each depth/visual obstacle observation:

```text
camera point/evidence
  -> body frame at capture time
  -> mission-local map frame
  -> accumulated sparse obstacle map
```

At each runtime tick:

```text
mission-local obstacle map
  -> transform through inverse(current map_T_body)
  -> crop around current ego
  -> LocalFlightMapSnapshot
  -> TrajectorySafetyEvaluator
```

## Provider-neutral detector rule

All obstacle sources must emit the same contract:

```cpp
std::vector<ObstacleEvidence>
```

This applies to:

- AirSim GT visual emulation
- AirSim DepthPlanar obstacle detector
- future real visual/depth obstacle detector

Downstream mapping, trajectory safety, and visualization should not depend on whether evidence came from GT, simulated depth, or a real detector, except for provenance/confidence/debug tuning.

## Non-goals for the first mission-local slices

- No global long-term map.
- No GPS/geodetic anchoring requirement.
- No replanning or command blocking.
- No detector-specific map logic.
- No duplicate pose or obstacle-evidence contracts.

## First implementation path

1. Add pose transform helpers over existing `Pose3`.
2. Add a mission-local obstacle map that stores sparse cells in `map_frame_id`.
3. Derive the ego-local `LocalFlightMapSnapshot` by transforming/cropping the mission-local map.
4. Keep `TrajectorySafetyEvaluator` ego-local and read-only.
5. Add offline 3D visualization and AirSim static replay of mission-local evidence.

## Persistent obstacle memory extension

Mission-local mapping is the live, one-mission accumulation layer. Persistent obstacle memory is the next layer and is intentionally separate.

```text
mission-local obstacle map
  -> mission obstacle map artifact
  -> persistent site obstacle map
  -> future mission preload
```

Persistent memory must record absolute time units (`unix_ns`) and primitive evidence fields so decay formulas can be changed later. Do not store only a single decayed score.

Decay policy:

- Calendar age alone should not erase obstacles.
- Cells should decay strongly only when new observations contradict them or when the site has been revisited but the cell/neighborhood was not reconfirmed.
- A site that has not been visited for a long time should retain stale obstacle memory rather than losing the entire map.
- Compute `relative_gap_seconds = max(0, cell_age_seconds - site_staleness_seconds)` to normalize cell age against whole-site staleness.

See `docs/persistent_obstacle_memory_plan.md` for the current 5H+ plan.


## Three-tier plugin storage model

Persistent obstacle memory now follows the storage/plugin architecture defined in `docs/obstacle_memory_plugin_architecture.md`.

The stable model is:

```text
Tier A — hot runtime maps
  In-memory MissionLocalObstacleMap / LocalFlightMap / future sparse grids.
  No blocking disk I/O in the flight hot path.

Tier B — mission delta stream
  Append-only changed-cell batches from runtime to durable mission logs.
  Backend can be JSONL, SQLite, LMDB, or binary.

Tier C — persistent site memory
  Durable, indexed, cross-mission map store.
  Backend can be current debug JSON, SQLite, LMDB, or custom binary/mmap.
```

The rest of the runtime should depend on contracts, not file formats. JSON remains a debug/export format; efficient storage should become the default for runtime preload, scoring, and persistence.
