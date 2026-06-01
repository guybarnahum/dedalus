# Dedalus Development Tracks

Dedalus now tracks three parallel architecture lines:

```text
Track 2.x — Behavior / mission / object-conditioned control
Track 3.x — Semantic perception for actors and objects of interest
Track 4.x — Classless geometric occupancy, reflexive avoidance, mapping, and ego localization
```

## Track 2.x — Behavior / Mission / Object-Conditioned Control

Track 2.x owns mission lifecycle and behavior command generation.

Examples:

```text
follow
approach
circle/orbit
behavior sequences
altitude profiles
yaw/camera policy
mission lifecycle validation
```

Track 2.x consumes `WorldSnapshot` and emits nominal command intent. It should not own CV/perception semantics and should not contain obstacle-map fusion logic.

## Track 3.x — Semantic Perception for Actors and Objects of Interest

Track 3.x owns semantic/class-aware perception above the geometric background layer.

Scope:

```text
object-class detection, segmentation, and classification
semantic actor/object tracking
identity and re-identification
Observation3D generation for objects of interest
class-specific confidence/provenance
TargetSelector-compatible agents
world-model ingestion of semantic actors
snapshot/artifact validation for semantic perception products
```

Examples of Track 3.x models:

```text
YOLO / DETR / segmentation models for objects of interest
pose/keypoint models for people/animals when behavior depends on them
re-ID / embedding models for persistent identities
trackers for semantic actors and selected targets
```

Track 3.x should build on the current pipeline boundary:

```text
AirSimFrameSource / other FrameSource
  -> FrameHintEgoProvider / ego provider
  -> CoreStackRunner
  -> PerceptionPipelineOutput
  -> InMemoryWorldModel
  -> WorldSnapshotPublisher
```

Track 3.x produces semantic evidence and world-model actor state. It does not perform closed-loop avoidance control and it is not the primary obstacle-avoidance layer.

## Track 4.x — Classless Geometric Occupancy, Reflexive Avoidance, Mapping, and Ego Localization

Track 4.x is the background geometric safety layer. It should not depend on YOLO/DETR-style object classes or semantic tracking.

Scope:

```text
classless depth / freespace / occupancy estimation
front-swept obstacle detection in direction of movement
ego-frame voxel or occupancy maps
short-memory obstacle/freespace state
reflexive bypass trajectory corrections
recovery to nominal behavior trajectory
ego-local obstacle memory
global obstacle/background map
geometric landmark/obstacle constraints for future ego self-localization
```

Possible Track 4.x model inputs:

```text
AirSim depth frames for simulation validation
monocular depth / stereo depth
optical flow / looming cues
freespace segmentation
classless obstacle masks
multi-frame broom/sweep integration into voxels or occupancy cells
```

Canonical Track 4.x plan:

```text
docs/track_4_obstacle_avoidance_and_mapping_plan.md
```

Track 4.x must preserve these boundaries:

```text
Classless geometric perception produces occupancy/freespace evidence.
WorldModel publishes autonomy state and geometric memory.
Behavior produces nominal command intent.
Avoidance adapts command intent based on geometric obstacle state.
Flight sinks execute commands only.
Overlay renders state; it does not infer avoidance semantics.
```

## Track relationship

```text
Track 4.x is the classless background safety layer:
  obstacles, freespace, occupancy, depth, ego/global maps, collision avoidance.

Track 3.x is the semantic foreground layer:
  people, animals, vehicles, selected targets, identities, class-aware behaviors.
```

Track 4.x can begin with synthetic occupancy observations or AirSim depth before Track 3.x semantic detection is complete. Track 3.x can use Track 4.x maps as background context, but Track 4.x must not require class labels to avoid collisions.

## Recommended immediate order

```text
4.0A — Classless occupancy / avoidance architecture and data contracts
4.0B — Synthetic ego-frame occupancy memory in WorldSnapshot
4.0C — Front-swept occupancy query validator
4.0D — Reflexive correction policy unit tests
4.0E — Mission-runtime integration in simulation
4.0F — Frame-derived depth/freespace occupancy evidence
4.0G — Ego-local/global obstacle maps
4.0H — Localization hooks from geometric map constraints
```
