# ERODF: Ego Rolling Occupancy Distance Field for Reflexive Drone Avoidance

## Abstract

Dedalus needs a fast, classless obstacle-avoidance layer that can keep a drone from colliding with geometry without depending on semantic object detection. The system must avoid a wall, branch, pole, rock, cable, wire, or unknown blob even when no YOLO/DETR-style detector has assigned a class label.

This white paper proposes **ERODF**, an **Ego Rolling Occupancy Distance Field**. ERODF maintains a short-lived, ego-local 3D occupancy field in the world model, updates it from classless geometric evidence such as depth, freespace, optical flow, looming cues, obstacle masks, AirSim depth, stereo depth, monocular depth, or synthetic fixtures, and exposes that field through `WorldSnapshot` for reflexive avoidance and future mapping.

The avoidance policy consumes the nominal velocity command produced by the mission behavior controller, queries the ego-local occupancy field using swept-volume collision checks, and emits a bounded corrected velocity command only when necessary. The flight sink remains a command sink; it does not own obstacle semantics or avoidance policy.

The recommended first implementation combines:

1. **EWOK-style rolling ego-local voxel occupancy** for bounded-memory real-time mapping.
2. **FIESTA/Voxblox-style local distance-field reasoning** for clearance queries.
3. **DWA-lite velocity-space sampling** for bounded reflexive correction.
4. **Explicit events and artifacts** so every correction is observable and testable.

ERODF is intended as the Track 4 foundation for classless geometric occupancy, reflexive avoidance, global map accumulation, and future ego localization against geometric structure.

---

## 1. Problem Statement

Dedalus already has a mission and behavior pipeline that produces nominal drone commands from behavior specifications. Examples include approach, circle, altitude profile, and behavior sequences. That pipeline answers:

> What object or behavior target do I care about, and how should I move relative to it?

Obstacle avoidance is a different problem. It asks:

> Is the space the drone is about to occupy safe?

That question must be answered without requiring semantic object identity. A safe drone must avoid geometry whether the geometry is a wall, pole, tree branch, cable, wire, fence, rock, vehicle, person, or unknown artifact.

Therefore, Track 4 should be a **classless geometric safety layer** that:

- Processes frames plus ego pose into ego-frame occupied, free, and unknown space.
- Maintains short-memory ego-local occupancy state in the world model.
- Supports swept-volume queries in the drone's intended movement direction.
- Generates bounded reflexive trajectory corrections around obstacles.
- Recovers to the nominal behavior trajectory when clear.
- Accumulates ego-local observations into longer-lived global map memory.
- Creates hooks for future ego localization against known geometric obstacles or landmarks.

---

## 2. Design Goals

### 2.1 Classless

Track 4 avoidance must not depend on semantic object classes, actor identity, or object-conditioned behavior targets.

It may consume classless geometric evidence from:

- AirSim depth.
- Stereo depth.
- Monocular depth.
- Optical flow.
- Looming cues.
- Obstacle masks.
- Freespace segmentation.
- Multi-frame swept or broom-style occupancy integration.
- Synthetic occupancy fixtures.

Semantic Track 3 perception may later decorate the same cells or regions with class labels, but Track 4 must not require those labels to avoid collisions.

### 2.2 World-Model-Native

The output of obstacle perception and avoidance must be represented in the world model. The system should expose occupancy state through:

- `WorldSnapshot`.
- Mission artifacts.
- Runtime event stream summaries.
- Validation tools.

Avoidance must not become a hidden image-processing sidecar and must not be buried inside `Px4BridgeCommandSink`.

### 2.3 Reflexive and Fast

The first layer should answer a bounded local safety question:

> Given the current nominal command, is the swept volume over the next short horizon clear, blocked, or unknown-risk?

This should run quickly enough for near-real-time command correction without starting with a heavy global planner or nonlinear optimizer.

### 2.4 Deterministic and Testable

The first implementation should be validated with synthetic occupancy fixtures before real CV, depth models, AirSim depth, or flight-command integration influence live behavior.

### 2.5 Observable

Every correction should produce events and artifact records. There should be no hidden avoidance behavior.

Required event concepts:

- `avoidance_query`
- `avoidance_active`
- `avoidance_correction`
- `avoidance_clear`
- `avoidance_recovering`
- `avoidance_failed_no_clear_path`

---

## 3. Recommended Algorithm

The recommended approach is:

> An ego-local rolling 3D voxel occupancy grid with a local distance field and swept-volume velocity sampling.

This has three layers.

### 3.1 Rolling Ego-Local Occupancy Grid

Maintain a fixed-size 3D voxel grid centered around the drone or a stable short-lived local map frame.

Each cell stores:

- `Free`
- `Occupied`
- `Unknown`
- confidence
- last observation time
- provenance
- optional distance to nearest occupied cell

The first implementation should use a dense circular buffer rather than an octree. This keeps memory bounded, indexing simple, and queries deterministic.

Example starting configuration:

```text
resolution: 0.20 m
window:     12 m x 12 m x 4 m
cells:      60 x 60 x 20 = 72,000 cells
```

A higher-resolution near-field configuration is also reasonable:

```text
resolution: 0.10 m
window:     8 m x 8 m x 3 m
cells:      80 x 80 x 30 = 192,000 cells
```

Both are small enough for CPU tests and early runtime validation.

### 3.2 Local Occupancy Distance Field

For each cell, maintain an approximate distance to the nearest occupied cell. This allows fast clearance checks:

- How close is the nearest obstacle?
- Is the forward corridor clear by at least the drone body radius plus safety margin?
- Which direction increases clearance?
- Is the nominal path clear, blocked, or unknown-risk?

The first implementation can recompute the local distance field over the bounded grid after updates. Later, it can upgrade to incremental ESDF-style update queues.

### 3.3 Swept-Volume Velocity Sampling

The avoidance policy receives a nominal velocity command from the mission behavior layer. It tests the swept volume of that command over a short horizon.

If the nominal command is clear, it passes through unchanged.

If blocked or risky, the policy samples nearby candidate velocities:

- slow forward
- stop or hover
- lateral left
- lateral right
- up
- down, when allowed
- slow plus lateral
- slow plus vertical

Each candidate is evaluated against the occupancy map. The selected correction is the lowest-cost safe candidate.

---

## 4. Thin Obstacles: Cables and Wires

Thin and long obstacles are the hardest classless-avoidance case. A wire may occupy less than one pixel, be difficult for monocular depth, and disappear in sparse depth. ERODF should therefore treat cables and wires as a special geometric risk category even though it remains classless.

### 4.1 Representation

Do not represent likely cables only as ordinary occupied voxels. Add a second primitive:

```cpp
struct OccupancyPrimitive {
  enum class Shape {
    Voxel,
    Capsule,
    LineSegment,
    SurfacePatch
  } shape;

  Vec3 center_ego_m;
  Vec3 endpoint_a_ego_m;
  Vec3 endpoint_b_ego_m;

  float radius_m;
  float confidence;
  float persistence_score;

  std::string source_provider;
  std::string source_frame_id;
  uint64_t last_observed_time_ns;
};
```

A cable-like feature should be stored as a **line segment or capsule** with a conservative radius, then rasterized into the occupancy grid for swept-volume queries.

### 4.2 Detection Cues

Potential cable evidence can come from multiple weak cues:

- Image-space line detection.
- Depth discontinuities.
- Stereo mismatch or sparse stereo returns.
- Optical-flow inconsistency.
- Repeated thin edge observations across frames.
- Looming or expansion of a thin feature during approach.
- Negative evidence: expected freespace ray suddenly terminates or becomes inconsistent.

No single cue is reliable enough. The map should accumulate cable suspicion over frames.

### 4.3 Conservative Inflation

Because the physical cable is thin but the perception uncertainty is large, wire-like primitives should be inflated more aggressively than ordinary obstacle surfaces.

Recommended default:

```text
wire/cable primitive radius = physical_estimate + perception_uncertainty + safety_margin
minimum inflated radius     = 0.25 m to 0.50 m for early simulation
```

This is intentionally conservative. It is better for the first safety layer to slow down or avoid a suspected wire than to treat it as free space.

### 4.4 Unknown-Space Rule for Wires

If the system is in a cable-prone environment, unknown forward space should become more expensive at speed.

Suggested policy:

```text
if forward corridor has weak thin-line evidence:
  slow down
  increase corridor radius
  require stronger known-free evidence
  prefer lateral/vertical paths with better visibility
```

### 4.5 Validation Fixtures

Add synthetic tests for:

- horizontal wire across the path
- vertical cable or pole-like line
- diagonal wire crossing the swept volume
- sparse intermittent cable observations
- false positive thin line outside the corridor
- cable seen once then aging out
- cable-prone mode where unknown forward corridor forces slowdown

---

## 5. Variable-Sized Voxels and Polar Occupancy

A fixed Cartesian voxel grid is the best first implementation because it is simple, deterministic, and easy to test. However, variable-sized cells are useful because nearby space needs high resolution while distant space can be coarser.

### 5.1 Recommendation

Use a staged approach:

1. **Start with fixed-size Cartesian ego voxels.**
2. **Add distance-banded resolution once the fixed map is validated.**
3. **Consider frustum/polar occupancy for sensor-native evidence and forward corridor risk.**
4. **Keep the public `WorldSnapshot` contract independent of internal cell layout.**

### 5.2 Distance-Banded Cartesian Grid

A practical intermediate design is a multi-resolution Cartesian grid:

```text
0-5 m:    0.10 m cells
5-15 m:   0.25 m cells
15-40 m:  0.50 m or 1.00 m cells
```

This gives high near-field precision without wasting memory far away.

### 5.3 Polar or Frustum Grid

A polar/frustum grid may be better aligned with camera observations:

```text
range bins:   near fine, far coarse
azimuth bins: angular resolution based on camera intrinsics
vertical bins or elevation bands
```

This is attractive because camera pixels naturally represent angular rays. Farther cells can become larger in meters because the same angular pixel covers more world space at range.

However, polar grids complicate:

- multi-camera fusion
- ego motion integration
- lateral movement queries
- global map accumulation
- distance-field computation
- debugging artifacts

### 5.4 Recommended Internal Representation

Use **two internal forms** once the system matures:

```text
Sensor/frustum occupancy
  -> efficient projection from depth/freespace rays
  -> preserves angular uncertainty

Cartesian ego-local occupancy
  -> swept-volume queries
  -> distance field
  -> world-model snapshot
  -> map accumulation
```

The frustum form is good for ingestion. The Cartesian form is good for planning and control.

### 5.5 Snapshot Contract

The world model should expose cells generically:

```cpp
struct EgoLocalOccupancyCell {
  Vec3 center_ego_m;
  Vec3 size_m;
  OccupancyCellState state;
  float confidence;
  float distance_to_nearest_occupied_m;
};
```

`size_m` allows variable cells without changing the rest of the pipeline.

---

## 6. Physical Repulsion Field vs Swept-Volume Policy

A repulsion field is useful, but it should not be the only avoidance method.

The idea is:

> Each obstacle contributes a repulsive velocity or force vector. The vector sum is added to the planned velocity to push the drone away.

This is attractive because it is smooth, cheap, and intuitive. However, pure potential-field methods can fail in local minima, oscillate near narrow passages, push the drone into unknown space, or cancel forces symmetrically in front of obstacles.

### 6.1 Recommendation

Use repulsion as a **soft scoring term or candidate generator**, not as the safety authority.

Recommended control structure:

```text
nominal velocity
  + repulsion-biased candidate generation
  -> swept-volume safety query
  -> bounded corrected velocity
```

The swept-volume query remains authoritative. A repulsion vector may suggest where to search, but a candidate is only accepted if its swept volume is safe.

### 6.2 Repulsion Vector

For occupied cells or primitives within an influence radius:

```text
repulsion_i = weight_i * direction_away_i * falloff(distance_i)
```

Possible falloff:

```text
falloff(d) = max(0, 1/d - 1/influence_radius)^2
```

The accumulated vector:

```text
repulsion = clamp(sum(repulsion_i), max_repulsion_mps)
```

Then generate candidates around:

```text
biased_velocity = nominal_velocity + repulsion
```

### 6.3 Required Guards

A repulsion field must be bounded by:

- maximum lateral correction
- maximum vertical correction
- maximum speed reduction or acceleration
- minimum clearance
- unknown-space policy
- swept-volume safety check
- oscillation damping
- recovery blending

### 6.4 When Repulsion Works Well

Repulsion is especially useful for:

- smooth wall following
- pushing away from isolated obstacles
- maintaining clearance from nearby geometry
- biasing candidate velocity search
- recovering around an obstacle without a full planner

### 6.5 When Repulsion Is Dangerous

Do not let pure repulsion command the drone when:

- facing a concave obstacle
- navigating a narrow doorway or passage
- obstacles exist on both sides
- the best action is braking rather than lateral motion
- unknown space dominates the proposed escape direction
- localization or occupancy confidence is stale

---

## 7. From Short-Lived Ego Field to Global Map

The ego-local occupancy field is optimized for immediate safety. The global map is optimized for memory, planning, speed, and localization. They should not be the same object.

### 7.1 Three-Layer Map Model

Use three map layers:

```text
1. Immediate swept-path occupancy
   Lifetime: milliseconds to seconds
   Purpose: reflexive collision checks

2. Ego-local rolling occupancy memory
   Lifetime: seconds to current local maneuver
   Purpose: avoid rediscovering the same obstacle, support bypass and recovery

3. Global/background geometric map
   Lifetime: mission, session, or persisted map
   Purpose: better route planning, faster flight through known-safe areas, localization constraints
```

### 7.2 Promotion from Ego Map to Global Map

Do not promote every short-lived cell. Promote only stable, repeated, well-localized geometry.

Promotion criteria:

```text
cell or primitive observed from multiple frames
confidence above threshold
low temporal inconsistency
map-frame transform quality above threshold
not likely dynamic
not only a one-frame artifact
not contradicted by later freespace rays
```

Promoted map elements should include provenance and uncertainty:

```cpp
struct GlobalGeometricMapElement {
  enum class Type {
    OccupiedVoxel,
    SurfacePatch,
    LineSegment,
    LandmarkPrimitive,
    KnownFreeCorridor
  } type;

  Vec3 position_map_m;
  Vec3 size_m;
  Vec3 normal_or_axis_map;

  float occupancy_probability;
  float confidence;
  float localization_uncertainty_m;
  float persistence_score;

  uint64_t first_seen_time_ns;
  uint64_t last_confirmed_time_ns;
  uint32_t observation_count;

  bool dynamic_suspected;
};
```

### 7.3 Free Space Is Also Valuable

The global map should not only remember obstacles. It should remember known-free corridors.

Known-free memory enables faster flight because the drone can distinguish:

- unobserved unknown space
- recently observed free space
- historically reliable free corridor
- known obstacle region

This is critical for flying better and faster.

### 7.4 Aging and Contradiction

Global map memory should age more slowly than the ego map but still support contradiction.

Rules:

```text
repeated obstacle evidence increases persistence
repeated freespace rays through a cell decrease occupancy
dynamic inconsistency reduces persistence
old low-confidence cells become background hints, not hard blockers
known-free corridors expire or degrade if not refreshed
```

---

## 8. Using the Global Map to Fly Better and Faster

The global map improves flight by changing the system from purely reactive to anticipatory.

### 8.1 Speed Policy

Speed should depend on map confidence:

```text
known-free corridor: allow higher speed
unknown corridor: limit speed
known clutter: slow down and increase margin
stale map: moderate speed, require confirmation
known cable/wire risk: slow down and increase safety radius
```

This lets the drone fly faster in space it has already validated and slower where uncertainty is high.

### 8.2 Pre-Avoidance Planning

Instead of waiting until the immediate swept path is blocked, the global map can bias nominal behavior earlier:

```text
if upcoming nominal path intersects known obstacle region:
  adjust path before reflexive avoidance triggers
```

This reduces abrupt correction and makes behavior look intentional.

### 8.3 Route Reuse

Known-free corridors can become reusable local routes:

```text
launch area -> yard corridor -> around tree -> target observation point
```

The drone can prefer previously validated corridors over unknown shortcuts.

### 8.4 Risk-Aware Behavior Envelope

Behavior controllers can ask the map for constraints:

- maximum safe speed in direction
- preferred lateral side
- altitude band risk
- known no-fly cells
- known-free recovery corridor

The behavior policy remains mission-driven, but the map informs safe aggressiveness.

---

## 9. Using the Map for Ego Localization

Track 4 should not attempt full SLAM in the first slice. However, it should preserve the right geometric products so later localization can plug in.

The core idea:

> Match currently observed geometric structure against the global geometric map to estimate or correct the drone's pose in the map frame.

### 9.1 Useful Geometric Features

Good localization features include:

- corners
- wall planes
- poles
- tree trunks
- fence lines
- roof edges
- ground-plane boundaries
- persistent line segments
- persistent surface patches
- obstacle constellations

Cable-like features may be useful if persistent, but should be treated carefully because they are thin and often uncertain.

### 9.2 Localization Constraint Output

The map should be able to emit constraints, not necessarily solve localization itself:

```cpp
struct GeometricLocalizationConstraint {
  enum class Type {
    PointToPlane,
    PointToLine,
    PlaneToPlane,
    LineToLine,
    OccupancyAlignment
  } type;

  Vec3 observed_feature_ego_m;
  Vec3 matched_feature_map_m;
  Vec3 normal_or_axis_map;

  float residual_m;
  float confidence;
  float covariance_hint;

  std::string map_element_id;
};
```

These constraints can later feed a VIO, SLAM, EKF, pose-graph, or scan-matching module.

### 9.3 Occupancy Alignment

A simple first localization hook is occupancy alignment:

```text
current local occupied/free pattern
  -> compare against nearby global map hypotheses
  -> estimate small pose correction
  -> emit map_frame correction hint
```

This is not full SLAM, but it allows the map to say:

> The current obstacle pattern best matches the global map if ego pose is shifted slightly left and rotated two degrees.

### 9.4 Localization-Aware Map Confidence

Do not promote local obstacles aggressively when ego localization is poor. Map confidence should depend on pose confidence.

```text
if ego pose covariance is high:
  reduce global promotion confidence
  increase map element uncertainty
  avoid hard planning commitments from those elements
```

---

## 10. Proposed Dedalus Runtime Architecture

Recommended flow:

```text
AirSim live frame + ego sidecar
  -> AirSimFrameSource
  -> FrameHintEgoProvider
  -> CoreStackRunner
       -> geometric evidence providers
       -> OccupancyObservation
  -> InMemoryWorldModel
       -> EgoLocalOccupancyMap
       -> optional GlobalGeometricMap
  -> WorldSnapshotPublisher
       -> LatestWorldSnapshotSubscriber
       -> ArtifactSnapshotWriter
       -> RuntimeEventStreamServer
  -> LatestWorldSnapshot
  -> MissionRuntime async loop
  -> ObjectBehaviorMissionController
       -> nominal VelocityCommand
  -> AvoidanceMissionAdapter
       -> SweptVolumeQuery
       -> AvoidancePolicy
       -> corrected VelocityCommand
  -> Px4BridgeCommandSink
  -> PX4 / AirSim
```

This preserves the intended boundary:

- Behavior controller emits nominal command.
- Avoidance adapter reads world-model occupancy and corrects if needed.
- Flight sink executes corrected command only.
- Artifacts and events explain what happened.

---

## 11. Proposed Data Contracts

### 11.1 `OccupancyObservation`

```cpp
enum class OccupancyEvidenceType {
  DepthPoint,
  FreeRay,
  OccupiedVoxel,
  ObstacleMask,
  FreespaceMask,
  FlowLooming,
  ThinLine,
  SyntheticFixture
};

struct OccupancyObservation {
  uint64_t timestamp_ns;

  std::string source_frame_id;
  std::string camera_name;
  std::string map_frame_id;
  std::string source_provider;

  Pose3D ego_pose_used;

  OccupancyEvidenceType evidence_type;

  Vec3 position_ego_m;
  Vec3 ray_origin_ego_m;
  Vec3 ray_direction_ego_unit;

  float range_m;
  float confidence;

  float occupied_update_log_odds;
  float free_update_log_odds;

  bool marks_free_space;
  bool marks_occupied_space;
};
```

### 11.2 `EgoLocalOccupancyCell`

```cpp
enum class OccupancyCellState {
  Unknown,
  Free,
  Occupied
};

struct EgoLocalOccupancyCell {
  int16_t ix;
  int16_t iy;
  int16_t iz;

  Vec3 center_ego_m;
  Vec3 size_m;

  OccupancyCellState state;

  float log_odds;
  float confidence;

  float distance_to_nearest_occupied_m;
  float occupancy_radius_m;

  uint64_t last_observed_time_ns;
  uint32_t age_frames;

  std::string source_frame_id;
  std::string camera_name;
  std::string source_provider;
};
```

### 11.3 `EgoLocalOccupancyMap`

```cpp
struct EgoLocalOccupancyMap {
  std::string map_frame_id;
  uint64_t timestamp_ns;

  float resolution_m;

  float size_x_m;
  float size_y_m;
  float size_z_m;

  uint32_t cells_x;
  uint32_t cells_y;
  uint32_t cells_z;

  Pose3D ego_pose_used;
  Vec3 ego_velocity_mps;

  std::vector<EgoLocalOccupancyCell> cells;

  uint32_t occupied_count;
  uint32_t free_count;
  uint32_t unknown_count;
  uint32_t stale_count;

  float nearest_obstacle_distance_m;
  float forward_corridor_clearance_m;
};
```

### 11.4 `SweptVolumeQuery`

```cpp
enum class UnknownSpacePolicy {
  TreatUnknownAsBlocked,
  TreatUnknownAsRisk,
  AllowUnknownIfSlowing
};

struct SweptVolumeQuery {
  uint64_t timestamp_ns;

  Vec3 ego_position_m;
  Vec3 ego_velocity_mps;
  Vec3 nominal_velocity_mps;

  float horizon_s;
  float corridor_radius_m;
  float vertical_margin_m;
  float braking_margin_m;

  UnknownSpacePolicy unknown_policy;
};
```

### 11.5 `SweptVolumeResult`

```cpp
enum class SweptVolumeStatus {
  Clear,
  OccupiedBlocked,
  UnknownRisk,
  StaleMap,
  NoValidData
};

struct SweptVolumeResult {
  SweptVolumeStatus status;

  float min_clearance_m;
  float time_to_collision_s;

  Vec3 nearest_obstacle_point_ego_m;
  Vec3 suggested_escape_direction_ego_m;

  uint32_t occupied_cells_in_corridor;
  uint32_t unknown_cells_in_corridor;
};
```

### 11.6 `AvoidanceCorrection`

```cpp
enum class AvoidanceCorrectionStatus {
  PassThrough,
  Corrected,
  Braking,
  HoverFallback,
  NoClearPath
};

struct AvoidanceCorrection {
  uint64_t timestamp_ns;

  AvoidanceCorrectionStatus status;

  VelocityCommand nominal_command;
  VelocityCommand corrected_command;

  SweptVolumeResult nominal_query_result;
  SweptVolumeResult selected_query_result;

  Vec3 repulsion_hint_mps;

  float correction_cost;
  float recovery_blend;

  std::string reason;
};
```

---

## 12. World Model Integration

The world model should own the short-memory occupancy state.

### 12.1 Ingestion

`InMemoryWorldModel` receives `OccupancyObservation` packets and updates the rolling occupancy grid.

For ray-like observations:

- Cells along the ray are updated toward `Free`.
- Endpoint cells are updated toward `Occupied` if the observation marks occupied space.
- Confidence affects update magnitude.

For mask-like observations:

- Mask pixels are projected into ego-frame rays or points.
- Occupied evidence updates obstacle cells.
- Freespace evidence clears traversable cells.

For synthetic observations:

- Tests directly create occupied, free, and unknown regions.

### 12.2 Aging

Each cell maintains `last_observed_time_ns` and `age_frames`.

Suggested aging behavior:

- Occupied cells decay toward `Unknown` when not reobserved.
- Free cells decay toward `Unknown` when stale.
- Unknown remains unknown.
- Cells outside the rolling window are dropped.
- Confidence decreases with age.

This prevents stale obstacles from permanently blocking motion while preserving short-memory persistence through temporary occlusion.

### 12.3 Frame Consistency

Every occupancy map and observation must carry:

- `map_frame_id`
- `source_frame_id`
- `timestamp_ns`
- `ego_pose_used`
- `source_provider`
- `camera_name`, when relevant

The world model should reject or quarantine observations whose frame identity is inconsistent with the current map state.

### 12.4 Snapshot Exposure

`WorldSnapshot` should expose a compact occupancy summary first, then optionally sampled cells.

Minimum snapshot fields:

```cpp
struct OccupancySnapshotSummary {
  std::string map_frame_id;
  uint64_t timestamp_ns;

  float resolution_m;
  uint32_t occupied_count;
  uint32_t free_count;
  uint32_t unknown_count;
  uint32_t stale_count;

  float nearest_obstacle_distance_m;
  float forward_corridor_clearance_m;

  bool has_valid_occupancy;
};
```

---

## 13. Swept-Volume Query Design

The swept-volume query tests the space the drone body would occupy if it executed a candidate velocity for a short horizon.

### 13.1 Inputs

- Current ego pose.
- Current ego velocity.
- Nominal command velocity.
- Occupancy map.
- Horizon in seconds.
- Drone radius plus safety margin.
- Vertical margin.
- Unknown-space policy.
- Braking margin.

### 13.2 Swept Geometry

For the first implementation, represent near-future motion as a swept capsule or cylinder:

```text
start  = ego position
end    = ego position + candidate_velocity * horizon_s
radius = drone_body_radius + safety_margin
```

The query samples cells intersecting the swept volume.

### 13.3 Output Classification

The swept path is classified as:

- `Clear`: no occupied cells and acceptable unknown-space exposure.
- `OccupiedBlocked`: occupied cells intersect the swept volume.
- `UnknownRisk`: no known obstacle, but unknown space exceeds policy threshold.
- `StaleMap`: occupancy data is too old.
- `NoValidData`: no occupancy data is available.

---

## 14. Reflexive Avoidance Policy

The first avoidance policy should be deliberately small and bounded.

### 14.1 Pass-Through Case

If the nominal command's swept volume is clear:

```text
corrected_command = nominal_command
status = PassThrough
```

### 14.2 Correction Case

If the nominal command is blocked or unknown-risk:

1. Generate candidate velocities.
2. Optionally bias candidates with a repulsion hint.
3. Query each candidate's swept volume.
4. Reject blocked candidates.
5. Score remaining candidates.
6. Emit the lowest-cost safe correction.

Candidate set example:

```text
nominal
0.75x nominal speed
0.50x nominal speed
0.25x nominal speed
hover
left strafe
right strafe
up
down
slow + left
slow + right
slow + up
```

### 14.3 Candidate Cost

Suggested cost:

```text
cost =
  nominal_deviation_weight * distance(candidate, nominal)
+ clearance_weight          * inverse_clearance(candidate)
+ smoothness_weight         * distance(candidate, previous_corrected)
+ recovery_weight           * distance(candidate, recovery_direction)
+ vertical_penalty
+ unknown_space_penalty
```

### 14.4 No-Clear-Path Case

If no safe candidate exists:

```text
corrected_command = brake_or_hover
status = NoClearPath
event = avoidance_failed_no_clear_path
```

### 14.5 Recovery

When the nominal path becomes clear again, the adapter should blend back rather than snap back:

```text
corrected = lerp(previous_corrected, nominal, recovery_blend)
```

---

## 15. Observability and Artifacts

Avoidance must be observable.

### 15.1 Events

#### `avoidance_query`

Emitted whenever the avoidance adapter evaluates a command.

```json
{
  "event": "avoidance_query",
  "timestamp_ns": 0,
  "nominal_velocity_mps": [0, 0, 0],
  "horizon_s": 1.0,
  "corridor_radius_m": 0.5,
  "status": "clear"
}
```

#### `avoidance_correction`

```json
{
  "event": "avoidance_correction",
  "status": "corrected",
  "nominal_velocity_mps": [1.0, 0.0, 0.0],
  "corrected_velocity_mps": [0.3, 0.5, 0.0],
  "min_clearance_m": 0.8,
  "reason": "occupied_blocked"
}
```

Other event names:

- `avoidance_active`
- `avoidance_clear`
- `avoidance_recovering`
- `avoidance_failed_no_clear_path`

### 15.2 Artifact Summary

Mission artifacts should include:

```json
{
  "occupancy": {
    "map_frame_id": "ego_local",
    "timestamp_ns": 0,
    "resolution_m": 0.2,
    "occupied_count": 35,
    "free_count": 1024,
    "unknown_count": 70941,
    "nearest_obstacle_distance_m": 1.4,
    "forward_corridor_clearance_m": 0.9
  },
  "avoidance": {
    "active": true,
    "last_status": "corrected",
    "last_reason": "occupied_blocked",
    "last_min_clearance_m": 0.9
  }
}
```

---

## 16. Proposed File-Level Implementation Plan

### 16.1 New Occupancy Types

Add:

```text
include/dedalus/occupancy/occupancy_types.hpp
include/dedalus/occupancy/ego_local_occupancy_map.hpp
include/dedalus/occupancy/swept_volume_query.hpp

src/occupancy/ego_local_occupancy_map.cpp
src/occupancy/swept_volume_query.cpp
```

### 16.2 World Model Extensions

Extend:

```text
include/dedalus/world_model/world_snapshot.hpp
src/world_model/world_snapshot.cpp

include/dedalus/world_model/in_memory_world_model.hpp
src/world_model/in_memory_world_model.cpp
```

Add support for:

- Ingesting `OccupancyObservation`.
- Maintaining `EgoLocalOccupancyMap`.
- Exposing occupancy summary in `WorldSnapshot`.
- Emitting occupancy artifact fields.

### 16.3 Avoidance Layer

Add after the occupancy map and swept query are tested:

```text
include/dedalus/avoidance/avoidance_policy.hpp
include/dedalus/avoidance/avoidance_mission_adapter.hpp

src/avoidance/avoidance_policy.cpp
src/avoidance/avoidance_mission_adapter.cpp
```

### 16.4 Mission Runtime Integration

Insert the adapter between nominal command generation and the sink:

```text
ObjectBehaviorMissionController
  -> AvoidanceMissionAdapter
  -> Px4BridgeCommandSink
```

The flight sink should not contain obstacle logic.

---

## 17. Validation Strategy

### 17.1 Synthetic Occupancy Fixtures

Before real depth or CV evidence, create deterministic synthetic scenes:

1. Empty map.
2. Wall directly ahead.
3. Pole near the forward corridor.
4. Unknown corridor.
5. Obstacle left, clear right.
6. Obstacle right, clear left.
7. Narrow passage.
8. Stale obstacle that ages out.
9. Moving ego frame with rolling grid.
10. No-clear-path enclosure.
11. Horizontal cable across path.
12. Diagonal cable crossing the swept volume.
13. Sparse intermittent cable evidence.
14. Known-free corridor reused from global map.
15. Global map contradiction by freespace rays.

### 17.2 Unit Tests

Add tests for:

```text
tests/unit/test_ego_local_occupancy_map.cpp
tests/unit/test_swept_volume_query.cpp
tests/unit/test_avoidance_policy.cpp
tests/unit/test_global_geometric_map.cpp
```

Core expectations:

- Occupied cells block swept-volume query.
- Free cells do not block.
- Unknown cells follow unknown-space policy.
- Stale maps produce `StaleMap`.
- Aging decays cells toward unknown.
- Candidate velocity selection chooses a safe correction.
- Repulsion hints do not bypass swept-volume safety.
- No-clear-path produces hover/brake fallback.
- Recovery blends back to nominal command.
- Thin-line obstacles are inflated and block the corridor.
- Stable local geometry can be promoted to the global map.
- Contradicted global map cells lose confidence.

### 17.3 Artifact Validation

Extend:

```text
tools/mission/validate-mission-artifacts.py
```

Validator should check:

- Occupancy summary exists when Track 4 is enabled.
- `map_frame_id` is present.
- Occupied/free/unknown counts are nonnegative.
- Avoidance events are internally consistent.
- Corrected command differs from nominal only when avoidance is active.
- `avoidance_failed_no_clear_path` includes fallback command.
- Thin-line/cable fixtures produce expected blocked corridors.
- Global map promotion includes observation count, confidence, and uncertainty.

---

## 18. Milestone Breakdown

### 4.0A — Architecture and Data Contracts

Deliver this design and define the minimal contracts.

### 4.0B — Synthetic Ego-Frame Occupancy Memory

Add `EgoLocalOccupancyMap` to the world model and expose it through `WorldSnapshot`.

### 4.0C — Front-Swept Occupancy Query Validator

Implement swept-volume query over synthetic occupancy cells.

### 4.0D — Reflexive Correction Policy Unit Tests

Implement `AvoidancePolicy` and test candidate velocity correction, including optional repulsion hints.

### 4.0E — Mission Runtime Integration

Insert `AvoidanceMissionAdapter` between behavior command generation and the flight sink.

### 4.0F — Frame-Derived Occupancy Evidence

Add AirSim depth and/or monocular depth projection into `OccupancyObservation`.

### 4.0G — Ego-Local and Global Obstacle Maps

Persist selected stable geometry and known-free corridors beyond the short-memory ego map.

### 4.0H — Localization Hooks

Use known geometric landmark, line, surface, and occupancy matches as constraints for ego/map-frame correction.

---

## 19. Non-Goals

The first implementation should avoid:

- YOLO/DETR dependency for safety.
- Semantic object-class requirement for collision avoidance.
- Obstacle logic inside flight sinks.
- Full global route planning.
- Brittle image-only avoidance without ego pose and depth/range confidence.
- Hidden command corrections without events or artifacts.
- Bypassing `WorldSnapshot`.
- Overlay-side avoidance inference.
- Large CV model wired directly into the flight loop as the first step.
- Treating pure repulsion as the safety authority.
- Treating unknown space as free space at speed.

---

## 20. Summary

ERODF gives Dedalus a practical Track 4 foundation:

- Classless geometric obstacle avoidance.
- Short-memory ego-local occupancy in the world model.
- Thin-obstacle support through inflated line/capsule primitives.
- Optional variable-size cells through `size_m` and later distance-banded grids.
- Fast swept-volume safety queries.
- Repulsion-biased but safety-checked command correction.
- Global map promotion for known obstacles and known-free corridors.
- Future ego-localization hooks from geometric map constraints.
- Explicit events and artifacts.
- Synthetic-first validation.

The first build should not be a full global planner. It should be a narrow, fast, deterministic safety layer that answers:

> Is the space the drone is about to occupy safe, and if not, what is the smallest bounded correction that keeps it safe while preserving mission intent?

That is the right foundation for Track 4 classless obstacle avoidance, spatial memory, and later map-aware autonomous flight.
