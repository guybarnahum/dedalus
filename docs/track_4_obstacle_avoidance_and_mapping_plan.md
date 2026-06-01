# Track 4.x — Classless Geometric Occupancy, Reflexive Avoidance, and Spatial Mapping

Track 4.x is the classless geometric safety layer. It is about detecting where the drone can and cannot safely move, not identifying what an object is.

Track 4.x should **not** depend on YOLO/DETR-style class detections or semantic tracking. Those belong to Track 3.x. Track 4.x may use depth, freespace, optical flow, obstacle masks, AirSim depth, stereo, monocular depth, or other classless geometric cues to produce occupancy/freespace evidence in ego coordinates.

This track intentionally does **not** put obstacle avoidance inside the flight sink. Flight sinks execute commands. Avoidance owns a policy layer that consumes world-model geometric state, proposes bounded trajectory corrections, and hands corrected command intent to the normal command sink path.

## Goal

Produce real-time classless obstacle awareness in the drone's direction of movement, keep a short-memory ego-frame representation of occupied/free space, generate reflexive trajectory corrections that bypass obstacles, recover the original behavior trajectory when possible, and accumulate obstacle/freespace memory into ego-local and global maps that improve future tracks and ego localization.

High-level flow:

```text
frames + ego pose
  -> classless geometric perception
       -> depth / freespace / optical-flow / obstacle-mask evidence
       -> swept-frame occupancy observations
       -> ego-frame voxel or occupancy map
  -> InMemoryWorldModel
       -> short-memory ego obstacle/freespace field
       -> persistent ego-local/global geometric map
       -> geometric landmarks / constraints for ego localization
  -> Track 4.x avoidance policy
       -> front swept-volume query in direction of movement
       -> reflexive bypass correction
       -> recover nominal trajectory when clear
  -> normal mission command sink path
```

## Track split

```text
Track 2.x — Behavior / mission / object-conditioned control
Track 3.x — Semantic perception for actors and objects of interest
Track 4.x — Classless geometric occupancy, reflexive avoidance, mapping, and ego localization
```

Track 4.x is the background geometric layer. Track 3.x is the semantic foreground layer. Track 3.x may classify people, animals, cars, and other actors of interest; Track 4.x should still avoid a wall, tree branch, pole, rock, wire, or unknown blob without knowing its class.

## Design principles

```text
1. Avoidance uses classless geometry first: occupied, free, unknown, dynamic/moving if inferable.
2. Semantic class labels are optional annotations, not required for collision avoidance.
3. Occupancy/freespace evidence belongs in perception/world-model products, not behavior controllers.
4. Avoidance policy consumes WorldSnapshot / geometric map state.
5. Flight sinks do not contain obstacle semantics.
6. The first avoidance layer should be reflexive, bounded, and explainable.
7. Recovery to the nominal behavior trajectory is part of avoidance, not an afterthought.
8. Every correction must be observable and validated from artifacts/events.
9. Use short memory for immediate safety and longer map memory for planning/localization.
```

## Classless perception inputs: what models and why

Initial Track 4.x should use provider-based geometric stages so models can be swapped without changing the world model or avoidance policy.

Recommended classless evidence stack:

```text
1. Depth provider
   Purpose: estimate range along pixels/rays.
   Initial providers:
     - AirSim depth frames for simulation validation
     - stereo depth if available
     - monocular depth for RGB-only fallback

2. Freespace / obstacle-mask provider
   Purpose: identify navigable vs non-navigable image regions without requiring class labels.
   Initial providers:
     - simple depth threshold / geometric mask from depth
     - classless segmentation/freespace model later

3. Optical-flow / looming provider
   Purpose: detect approaching obstacles and moving geometry in the flight direction.
   Initial providers:
     - sparse feature flow for lightweight motion cues
     - dense flow later if needed

4. Multi-frame broom/sweep integrator
   Purpose: integrate depth/freespace rays over ego motion into ego-frame voxels or occupancy cells.
   Initial providers:
     - deterministic synthetic occupancy provider for tests
     - AirSim depth-to-voxel provider for simulation
     - monocular/stereo depth-to-voxel provider later

5. Ego pose / ego-motion provider
   Purpose: transform per-frame obstacle/freespace evidence into ego-local and map frames.
   Initial providers:
     - existing FrameHintEgoProvider / AirSim ego sidecar
     - later VIO/SLAM-compatible ego estimate
```

Track 4.x should not run YOLO/DETR-class detectors as a prerequisite for safety. Semantic detections can decorate or cross-check geometric occupancy, but the primary avoidance map is classless.

## Broom / swept occupancy concept

The core Track 4.x perception primitive is a moving ego-frame sweep:

```text
for each frame:
  project depth/freespace rays into ego coordinates
  mark free cells along each ray up to measured depth
  mark occupied cells near depth termination / obstacle surface
  mark unknown cells outside observed FOV or beyond confidence horizon
  age previous cells over time
  transform/update cells using ego motion
```

This creates a local geometric memory of what the drone recently saw, not just what is visible in the latest frame.

The map can be implemented as one of:

```text
sparse voxel grid
2.5D ego-local occupancy grid
height-banded occupancy grid
frustum-aligned swept corridor grid
point-cloud / surfel cache with occupancy projection
```

For early Track 4.x, a sparse ego-local voxel/occupancy grid is preferred because it is explicit, testable, and easy to query for swept-volume collision checks.

## Obstacle / occupancy representation

Start with three layers of classless geometric state:

```text
1. Immediate swept-path occupancy
   Lifetime: seconds.
   Frame: ego-local movement corridor.
   Purpose: reflexive avoidance decisions.

2. Ego-local occupancy memory
   Lifetime: current flight / recent local area.
   Frame: active local map frame or rolling ego map.
   Purpose: avoid repeatedly rediscovering the same obstacle and support recovery around it.

3. Global/background geometric map
   Lifetime: mission/session or persisted map.
   Frame: global/map frame where available.
   Purpose: future path planning, known-risk corridors, and self-localization landmarks.
```

Candidate data contracts:

```cpp
enum class OccupancyState {
    Unknown,
    Free,
    Occupied,
};

struct OccupancyObservation {
    TimePoint timestamp;
    FrameId source_frame_id;
    std::string camera_name;
    MapFrameId map_frame_id;

    Vec3 center_local;
    Vec3 size_m;
    OccupancyState state;
    double occupancy_probability;
    double confidence;
    double range_m;

    std::string source; // synthetic, airsim_depth, stereo_depth, monocular_depth, freespace, flow
};

struct EgoVoxelCell {
    Vec3 center_local;
    Vec3 size_m;
    double occupied_probability;
    double free_probability;
    double last_observed_s;
    double confidence;
};

struct SweptVolumeQuery {
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
    std::string obstacle_cell_id;
    double time_to_collision_s;
    double clearance_m;
};
```

These names are illustrative. The implementation should use the repo's current type conventions after inspection.

## Swept-volume query

The first reflexive query should be a swept volume in the direction of intended movement.

Inputs:

```text
ego pose
ego velocity
nominal behavior command velocity
ego-local occupancy / voxel map
short-memory occupied/free/unknown cells
behavior envelope: radius, altitude, max speed, max vertical speed
```

Query:

```text
Look ahead along the nominal movement vector for horizon_s seconds.
Create a swept volume with radius based on vehicle radius + margin + speed margin.
Intersect the swept volume with occupied and unknown cells.
Prefer paths through known-free space.
Rank threats by time-to-collision, distance, occupancy probability, confidence, and persistence.
```

Initial defaults:

```text
horizon_s: 2.0 to 4.0
corridor_radius_m: vehicle_radius + 1.0m margin
vertical_margin_m: 1.0 to 2.0m
minimum_occupancy_probability: conservative but not brittle
unknown_policy: slow_down_or_avoid when speed/risk is high
```

## Reflexive correction policy

The first policy should be simple and explainable:

```text
if swept volume is clear through known-free / low-occupancy space:
  return nominal velocity

if occupied or high-risk unknown cells are ahead:
  choose lateral bypass direction with greater free clearance
  reduce speed if clearance is low or map confidence is poor
  optionally climb if lateral clearance is blocked and vertical free space is available
  produce corrected velocity = nominal forward component + lateral escape + optional vertical escape

while bypassing:
  keep querying swept volume
  keep occupancy memory active
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
unknown_space_speed_limit_mps
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
  -> consumes nominal VelocityCommand + WorldSnapshot occupancy products
  -> emits corrected VelocityCommand plus avoidance events

Px4BridgeCommandSink
  -> executes corrected command only
```

Do not embed obstacle logic inside `Px4BridgeCommandSink` or AirSim/PX4 bridge scripts.

## World-model responsibilities

The world model should own geometric memory and coordinate-frame semantics:

```text
- ingest occupancy observations from classless perception
- fuse/update short-memory ego-local occupancy state
- expose ego-local occupancy/voxel map in WorldSnapshot
- maintain persistent/global geometric map if configured
- attach provenance and confidence
- age out stale cells
- distinguish occupied/free/unknown
- optionally flag dynamic/moving geometry when inferred from flow or temporal inconsistency
```

Occupancy memory should support:

```text
short_memory_ttl_s
persistent_static_cell_threshold
dynamic_cell_decay_s
map_frame_id consistency
uncertainty growth when not observed
free-space clearing along rays
unknown-space policy
```

## Ego map and global map

Ego map:

```text
A rolling local occupied/free/unknown memory centered around the drone's recent trajectory.
Useful for immediate reflexes, recovery, and not oscillating around the same obstacle.
```

Global map:

```text
A persistent classless geometric map in a stable map frame.
Useful for future flight tracks, known-risk corridors, and pre-avoidance planning.
```

The same geometric cells can feed localization:

```text
known obstacle surfaces / geometric landmarks
  -> match against global map
  -> estimate ego drift / map-frame correction
  -> improve active_map_frame consistency
```

Track 4.x should not try to solve full SLAM in the first slice. It should define the data contracts so later VIO/SLAM/global localization can plug in.

## Relationship to Track 3.x

```text
Track 4.x:
  classless background geometry
  depth / freespace / occupancy / voxels
  collision avoidance
  geometric map memory
  localization constraints

Track 3.x:
  semantic foreground actors and objects of interest
  YOLO/DETR-style classes
  tracking / identity / re-ID
  selected targets for object-conditioned behavior
```

Track 3.x can use Track 4.x maps as context: for example, to reject impossible tracks, constrain object depth, or understand occlusion. Track 4.x should not require Track 3.x labels to avoid collisions.

## Suggested milestones

```text
4.0A — Classless occupancy / avoidance architecture and data contracts
  Document occupancy observations, ego voxel map, swept-volume query, and correction events.

4.0B — Synthetic ego-frame occupancy memory in WorldSnapshot
  Add occupancy cells to world model and artifacts using deterministic synthetic inputs.

4.0C — Swept-volume query validator
  Given ego pose, nominal velocity, and occupancy cells, validate collision/no-collision classification.

4.0D — Reflexive correction policy unit tests
  Generate lateral/vertical correction around synthetic occupancy and recover to nominal velocity.

4.0E — Mission-runtime integration in simulation
  Insert AvoidancePolicy between nominal behavior command and flight sink; emit avoidance events.

4.0F — Frame-derived depth/freespace occupancy evidence
  Feed occupancy observations from AirSim depth / depth providers into the same world-model path.

4.0G — Ego-local occupancy memory and global geometric map
  Persist encountered geometry and expose map artifacts.

4.0H — Localization hooks
  Use known geometric landmark/surface matches as constraints for ego/map-frame correction.
```

## First implementation slice recommendation

Do not start by wiring a large semantic detector into the flight loop. Start with 4.0A/4.0B/4.0C:

```text
1. Add classless occupancy observation and ego-local voxel/occupancy map contracts.
2. Add synthetic occupancy provider or fixture for deterministic tests.
3. Add world-model ingestion and WorldSnapshot serialization.
4. Add swept-volume query library with unit tests.
5. Add artifact validator support.
```

Then add reflexive correction policy in 4.0D/4.0E. Only after that should real depth/freespace providers influence flight.

## Non-goals for early Track 4.x

```text
- No obstacle logic inside flight sinks.
- No full global route planner in the first slice.
- No brittle image-only avoidance without ego pose and depth/range confidence.
- No dependency on YOLO/DETR/object-class detections.
- No hidden corrections without events/artifacts.
- No bypass of WorldSnapshot.
- No direct overlay-side avoidance inference.
```

## Validation expectations

Every Track 4.x run should prove:

```text
occupancy observations are present with source/provenance
occupied/free/unknown memory updates over time
swept-volume query detects occupied/unknown space in the movement corridor
avoidance events explain the correction
corrected command stays within configured limits
vehicle avoids synthetic occupancy in simulation
controller recovers nominal behavior when clear
world snapshot/artifacts expose the occupancy map state
```

Suggested test groups:

```text
ctest --test-dir build-staging --output-on-failure -R \
  'world_model|occupancy|obstacle|avoidance|core_stack|mission_runtime|mission_artifact_validator'
```
