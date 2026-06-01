# Track 4.x — Reflexive Obstacle Avoidance and Spatial Mapping

Track 4.x is the obstacle-avoidance and spatial-mapping track. It builds on the existing Dedalus world-model architecture and on Track 3.x perception/spatial evidence products.

This track intentionally does **not** put obstacle avoidance inside the flight sink. Flight sinks execute commands. Avoidance owns a policy layer that consumes world-model state, proposes bounded trajectory corrections, and hands corrected command intent to the normal command sink path.

## Goal

Produce real-time obstacle awareness in the drone's direction of movement, keep a short-memory representation of obstacles in the world model, generate reflexive trajectory corrections that bypass obstacles, recover the original behavior trajectory when possible, and accumulate obstacle memory into an ego/local/global map that improves future tracking and localization.

High-level flow:

```text
frames + ego pose
  -> Track 3.x perception providers
       -> object detections / segmentation / depth / optical flow / freespace evidence
       -> obstacle observations
       -> ego-local obstacle map
  -> InMemoryWorldModel
       -> short-memory obstacle field
       -> persistent local/global obstacle map
       -> map landmarks / constraints for ego localization
  -> Track 4.x avoidance policy
       -> swept path query in direction of movement
       -> reflexive bypass correction
       -> recover nominal trajectory when clear
  -> normal mission command sink path
```

## Track split

```text
Track 2.x — Behavior / mission / object-conditioned control
Track 3.x — Perception and spatial world-model evidence
Track 4.x — Reflexive obstacle avoidance, obstacle memory, mapping, and ego localization
```

Track 4.x depends on Track 3.x but can start with conservative synthetic or simple CV obstacle evidence while the full perception stack matures.

## Design principles

```text
1. Obstacle evidence belongs in perception/world-model products, not behavior controllers.
2. Avoidance policy consumes WorldSnapshot / local obstacle map state.
3. Flight sinks do not contain obstacle semantics.
4. The first avoidance layer should be reflexive, bounded, and explainable.
5. Recovery to the nominal behavior trajectory is part of avoidance, not an afterthought.
6. Every correction must be observable and validated from artifacts/events.
7. Use short memory for immediate safety and longer map memory for planning/localization.
```

## Perception inputs: what models and why

Initial Track 4.x should use provider-based CV stages so models can be swapped without changing the world model or avoidance policy.

Recommended perception evidence stack:

```text
1. Object detector / segmenter
   Purpose: detect explicit obstacles such as trees, poles, buildings, people, vehicles, animals, walls, rocks, and terrain structures.
   Initial providers:
     - scripted/synthetic obstacle provider for tests
     - YOLO-family detector for real-time classes
     - segmentation model for obstacle masks / freespace masks

2. Monocular depth or stereo/depth provider
   Purpose: estimate obstacle range and freespace from camera frames.
   Initial providers:
     - AirSim depth image provider where available
     - monocular depth provider for RGB-only fallback
     - flat-ground projection only for low-risk/simple cases

3. Optical flow / frame-to-frame motion provider
   Purpose: identify looming obstacles and moving obstacles in the direction of travel.
   Initial providers:
     - sparse feature flow for lightweight motion cues
     - dense flow later if needed

4. Ego pose / ego-motion provider
   Purpose: transform per-frame obstacle evidence into ego-local and map frames.
   Initial providers:
     - existing FrameHintEgoProvider / AirSim ego sidecar
     - later VIO/SLAM-compatible ego estimate
```

Track 3.x should expose the output of these stages as world-model evidence products. Track 4.x should consume those products; it should not run CV directly inside the avoidance controller.

## Obstacle representation

Start with three layers of obstacle state:

```text
1. Immediate swept-path obstacles
   Lifetime: seconds.
   Frame: ego/local movement corridor.
   Purpose: reflexive avoidance decisions.

2. Ego-local obstacle memory
   Lifetime: current flight / recent local area.
   Frame: active local map frame.
   Purpose: avoid repeatedly rediscovering the same obstacle and support recovery around it.

3. Global obstacle map
   Lifetime: mission/session or persisted map.
   Frame: global/map frame where available.
   Purpose: future path planning, known-risk areas, and self-localization landmarks.
```

Candidate data contracts:

```cpp
struct ObstacleObservation {
    TimePoint timestamp;
    FrameId source_frame_id;
    std::string camera_name;
    MapFrameId map_frame_id;

    Vec3 position_local;
    Vec3 velocity_local;
    Vec3 dimensions_m;
    double range_m;
    double bearing_rad;
    double elevation_rad;

    double confidence;
    std::string source;        // yolo, segmentation, depth, flow, synthetic, airsim_depth
    std::string obstacle_type; // tree, wall, person, unknown, terrain, building, vehicle...
    bool dynamic;
};

struct EgoLocalObstacleCell {
    Vec3 center_local;
    Vec3 size_m;
    double occupancy_probability;
    double last_observed_s;
    double confidence;
};

struct CollisionCorridorQuery {
    Vec3 ego_position_local;
    Vec3 ego_velocity_local;
    Vec3 nominal_velocity_local;
    double horizon_s;
    double corridor_radius_m;
    double vertical_margin_m;
};

struct AvoidanceCorrection {
    bool active;
    Vec3 corrected_velocity_local;
    Vec3 lateral_escape_vector_local;
    double speed_scale;
    std::string reason;
    std::string obstacle_id;
    double time_to_collision_s;
    double clearance_m;
};
```

These names are illustrative. The implementation should use the repo's current type conventions after inspection.

## Swept-path detection

The first reflexive query should be a swept volume in the direction of intended movement.

Inputs:

```text
ego pose
ego velocity
nominal behavior command velocity
camera field of view / frame evidence
short-memory obstacle map
behavior envelope: radius, altitude, max speed, max vertical speed
```

Query:

```text
Look ahead along the nominal movement vector for horizon_s seconds.
Create a corridor with radius based on vehicle radius + margin + speed margin.
Intersect corridor with obstacle observations / occupancy cells.
Rank obstacles by time-to-collision, distance, confidence, and persistence.
```

Initial defaults:

```text
horizon_s: 2.0 to 4.0
corridor_radius_m: vehicle_radius + 1.0m margin
vertical_margin_m: 1.0 to 2.0m
minimum_confidence: conservative but not brittle
```

## Reflexive correction policy

The first policy should be simple and explainable:

```text
if no obstacle in swept corridor:
  return nominal velocity

if obstacle ahead:
  choose lateral bypass direction with greater free clearance
  reduce speed if clearance is low
  optionally climb if lateral clearance is blocked and climb is safe
  produce corrected velocity = nominal forward component + lateral escape + optional vertical escape

while bypassing:
  keep querying swept path
  keep obstacle memory active
  keep target/behavior context alive

when corridor to nominal trajectory is clear:
  blend back to nominal behavior command over recovery_s seconds
```

Avoidance must be bounded:

```text
max_lateral_correction_mps
max_vertical_correction_mps
max_speed_scale
max_avoidance_duration_s before abort/go-home policy
minimum_clearance_m
```

The controller should emit events such as:

```text
avoidance_query
avoidance_active
avoidance_correction
avoidance_clear
avoidance_recovering
avoidance_failed_no_clear_path
```

## Integration with behavior runtime

Recommended control boundary:

```text
ObjectBehaviorMissionController
  -> computes nominal VelocityCommand

AvoidancePolicy / AvoidanceMissionAdapter
  -> consumes nominal VelocityCommand + WorldSnapshot obstacle products
  -> emits corrected VelocityCommand plus avoidance events

Px4BridgeCommandSink
  -> executes corrected command only
```

Do not embed obstacle logic inside `Px4BridgeCommandSink` or AirSim/PX4 bridge scripts.

## World-model responsibilities

The world model should own the memory and coordinate-frame semantics:

```text
- ingest obstacle observations from perception
- fuse/update short-memory obstacle state
- expose ego-local obstacle map in WorldSnapshot
- maintain persistent/global obstacle map if configured
- attach provenance and confidence
- age out stale obstacles
- mark dynamic obstacles separately from static map obstacles
```

Obstacle memory should support:

```text
short_memory_ttl_s
persistent_static_obstacle_threshold
dynamic_obstacle_decay_s
map_frame_id consistency
uncertainty growth when not observed
```

## Ego map and global map

Ego map:

```text
A local obstacle/freespace memory centered around the drone's recent trajectory.
Useful for immediate reflexes, recovery, and not oscillating around the same obstacle.
```

Global map:

```text
A persistent obstacle/landmark map in a stable map frame.
Useful for future flight tracks, known-risk corridors, and pre-avoidance planning.
```

The same obstacle observations can feed localization:

```text
known obstacle/landmark observations
  -> match against global map
  -> estimate ego drift / map-frame correction
  -> improve active_map_frame consistency
```

Track 4.x should not try to solve full SLAM in the first slice. It should define the data contracts so later VIO/SLAM/global localization can plug in.

## Suggested milestones

```text
4.0A — Architecture/data-contract plan
  Document obstacle observations, short-memory obstacle map, avoidance query, and correction events.

4.0B — Synthetic obstacle memory in WorldSnapshot
  Add obstacle observations/cells to world model and artifacts using deterministic synthetic inputs.

4.0C — Swept-path query validator
  Given ego pose, nominal velocity, and obstacle cells, validate collision/no-collision classification.

4.0D — Reflexive correction policy unit tests
  Generate lateral/vertical correction around synthetic obstacles and recover to nominal velocity.

4.0E — Mission-runtime integration in simulation
  Insert AvoidancePolicy between behavior command and flight sink; emit avoidance events.

4.0F — Frame-derived obstacle evidence
  Feed obstacle observations from CV/depth/segmentation providers into the same world-model path.

4.0G — Ego-local obstacle memory and global obstacle map
  Persist encountered obstacles and expose map artifacts.

4.0H — Localization hooks
  Use known obstacle/landmark matches as constraints for ego/map-frame correction.
```

## First implementation slice recommendation

Do not start by wiring a large CV model into the flight loop. Start with 4.0A/4.0B/4.0C:

```text
1. Add obstacle observation and ego-local obstacle map contracts.
2. Add synthetic obstacle provider or fixture for deterministic tests.
3. Add world-model ingestion and WorldSnapshot serialization.
4. Add swept-path query library with unit tests.
5. Add artifact validator support.
```

Then add reflexive correction policy in 4.0D/4.0E. Only after that should real CV providers be allowed to influence flight.

## Non-goals for early Track 4.x

```text
- No obstacle logic inside flight sinks.
- No full global route planner in the first slice.
- No brittle image-only avoidance without ego pose and depth/range confidence.
- No hard dependency on a single CV model.
- No hidden corrections without events/artifacts.
- No bypass of WorldSnapshot.
- No direct overlay-side avoidance inference.
```

## Validation expectations

Every Track 4.x run should prove:

```text
obstacle observations are present with source/provenance
obstacle memory updates over time
swept-path query detects obstacles in the movement corridor
avoidance events explain the correction
corrected command stays within configured limits
vehicle avoids synthetic obstacle in simulation
controller recovers nominal behavior when clear
world snapshot/artifacts expose the obstacle map state
```

Suggested test groups:

```text
ctest --test-dir build-staging --output-on-failure -R \
  'world_model|obstacle|avoidance|core_stack|mission_runtime|mission_artifact_validator'
```
