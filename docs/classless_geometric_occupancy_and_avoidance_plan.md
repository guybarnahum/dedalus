# Classless Geometric Occupancy, Reflexive Avoidance, and Spatial Mapping

This is the classless geometric safety layer. It is about detecting where the drone can and cannot safely move, not identifying what an object is.

The geometric safety layer should **not** depend on YOLO/DETR-style class detections or semantic tracking. Semantic perception may annotate actors or objects of interest, but geometric safety must be able to avoid a wall, tree branch, pole, rock, wire, or unknown blob without knowing its class.

Obstacle sensing starts at the camera coverage boundary. Read `docs/sensing_coverage_architecture.md` before modifying visual obstacle detection, visual-emulation, ego occupancy mapping, or global spatial mapping.

This layer intentionally does **not** put obstacle avoidance inside the flight sink. Flight sinks execute commands. Avoidance owns a policy layer that consumes world-model geometric state, proposes bounded trajectory corrections, and hands corrected command intent to the normal command sink path.

## Goal

Produce real-time classless obstacle awareness from current sensing coverage, keep a short-memory ego-frame representation of occupied/free/unknown space, generate reflexive trajectory corrections that bypass obstacles, recover the original behavior trajectory when possible, and accumulate obstacle/freespace memory into ego-local and global maps that improve future tracks and ego localization.

High-level flow:

```text
FramePacket + EgoState + CameraSensingVolume
  -> EgoSensingFrame
  -> classless geometric perception
       -> depth / freespace / optical-flow / obstacle-mask evidence
       -> obstacle evidence in camera/ego coordinates
  -> EgoOccupancyMapper
       -> short-memory ego obstacle/freespace field
  -> GlobalSpatialMapper
       -> persistent ego-local/global geometric map
       -> geometric landmarks / constraints for ego localization
  -> reflexive avoidance policy
       -> swept-volume query in direction of intended movement
       -> bounded bypass correction
       -> recover nominal trajectory when clear
  -> normal mission command sink path
```

## Layer split

```text
Object behavior / mission control:
  target-conditioned movement, yaw/camera intent, mission lifecycle

Semantic perception:
  actors and objects of interest, optional labels, identity, tracking

Classless geometric safety:
  sensing coverage, obstacle evidence, occupancy, reflexive avoidance, mapping, ego localization support
```

The geometric safety layer is the background safety layer. Semantic perception is the foreground actor/object layer. Semantic labels can enrich evidence, but must not be required for collision avoidance.

## Design principles

```text
1. Sensing coverage is camera optical coverage, not vehicle heading.
2. Avoidance uses classless geometry first: occupied, free, unknown, dynamic/moving if inferable.
3. Semantic class labels are optional annotations, not required for collision avoidance.
4. Occupancy/freespace evidence belongs in perception/world-model products, not behavior controllers.
5. Avoidance policy consumes WorldSnapshot / geometric map state.
6. Flight sinks do not contain obstacle semantics.
7. The first avoidance layer should be reflexive, bounded, and explainable.
8. Recovery to the nominal behavior trajectory is part of avoidance, not an afterthought.
9. Every correction must be observable and validated from artifacts/events.
10. Use short memory for immediate safety and longer map memory for planning/localization.
```

## Classless perception inputs: what models and why

Initial geometric safety work should use provider-based geometric stages so models can be swapped without changing the world model or avoidance policy.

Recommended classless evidence stack:

```text
1. Sensing coverage provider
   Purpose: compute the current optical coverage volume for every sensing camera.
   Initial providers:
     - configured camera intrinsics/FOV/range
     - camera mount/extrinsics
     - commanded camera_pointing_intent
     - later measured gimbal feedback

2. Depth provider
   Purpose: estimate range along pixels/rays.
   Initial providers:
     - AirSim depth frames for simulation validation
     - stereo depth if available
     - monocular depth for RGB-only fallback

3. Freespace / obstacle-mask provider
   Purpose: identify navigable vs non-navigable image regions without requiring class labels.
   Initial providers:
     - simple depth threshold / geometric mask from depth
     - classless segmentation/freespace model later

4. Optical-flow / looming provider
   Purpose: detect approaching obstacles and moving geometry in the flight direction.
   Initial providers:
     - sparse feature flow for lightweight motion cues
     - dense flow later if needed
```
