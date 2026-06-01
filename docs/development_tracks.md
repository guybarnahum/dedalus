# Dedalus Development Tracks

Dedalus now tracks three parallel architecture lines:

```text
Track 2.x — Behavior / mission / object-conditioned control
Track 3.x — Perception and spatial world-model evidence
Track 4.x — Reflexive obstacle avoidance, obstacle memory, mapping, and ego localization
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

## Track 3.x — Perception and Spatial World-Model Evidence

Track 3.x owns frame-derived evidence and world-model spatial products.

Scope:

```text
frame processing
detections / segmentation / depth / optical flow providers
Observation3D generation
obstacle observations
ego-local spatial products
local occupancy / obstacle evidence
world-model ingestion
snapshot/artifact validation for perception products
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

Track 3.x produces evidence and world-model state. It does not perform closed-loop avoidance control.

## Track 4.x — Reflexive Obstacle Avoidance and Mapping

Track 4.x consumes Track 3.x perception/spatial products and generates bounded trajectory corrections.

Scope:

```text
swept-path obstacle queries in the direction of movement
short-memory obstacle state
reflexive bypass trajectory corrections
recovery to nominal behavior trajectory
ego-local obstacle memory
global obstacle map
landmark/obstacle constraints for future ego self-localization
```

Canonical Track 4.x plan:

```text
docs/track_4_obstacle_avoidance_and_mapping_plan.md
```

Track 4.x must preserve these boundaries:

```text
Perception produces evidence.
WorldModel publishes autonomy state.
Behavior produces nominal command intent.
Avoidance adapts command intent based on obstacle state.
Flight sinks execute commands only.
Overlay renders state; it does not infer avoidance semantics.
```

## Track dependency order

```text
Track 3.x evidence can be developed independently using synthetic/smoke inputs.
Track 4.x can begin with synthetic obstacle observations before real CV models are complete.
Real CV-derived avoidance should wait until obstacle observations, world-model memory, and avoidance validators are in place.
```

## Recommended immediate order

```text
4.0A — Architecture/data-contract plan
4.0B — Synthetic obstacle memory in WorldSnapshot
4.0C — Swept-path query validator
4.0D — Reflexive correction policy unit tests
4.0E — Mission-runtime integration in simulation
4.0F — Frame-derived obstacle evidence
4.0G — Ego-local/global obstacle maps
4.0H — Localization hooks
```
