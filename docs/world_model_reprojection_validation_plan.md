# World-Model Reprojection Validation Plan

This document defines the planned validation layer for projecting WorldSnapshot state back into the camera viewport.

It is not only for ghost/scripted detections. It is a general perception-consistency tool:

```text
2D camera detection / track
  -> projected to 3D ego/local/world state
  -> captured, fused, processed, and enriched by the world model
  -> exposed through WorldSnapshot.agents
  -> reprojected back into the current camera viewport
  -> compared against the original or current 2D evidence
```

The same machinery also supports ghost/scripted targets, selected-target overlays, behavior visualization, MP4 review artifacts, and optional AirSim viewport debugging.

---

## 1. Motivation

Object-conditioned behavior depends on the world model being spatially coherent. A detector box in the camera frame is not enough; the system must know where that object is relative to the drone and map frame. But once a detection has been lifted into 3D and fused into `WorldSnapshot`, we need a way to validate that the world-model position still explains what the camera sees.

The core loop is:

```text
camera viewport observation
  -> 2D detector / tracker
  -> 3D projection into ego/local/world frame
  -> WorldSnapshot agent
  -> reproject WorldSnapshot agent into camera viewport
  -> overlay / compare / measure residual
```

This creates a powerful validation signal:

```text
If the world-model agent is correct, its reprojected image position should remain consistent with image evidence as the drone moves.
```

---

## 2. Architecture Boundary: Perception Evidence, World-Model State, Behavior Input

All perception-derived information that may influence autonomy must be captured into the world model before behavior consumes it.

```text
Perception pipeline:
  Produces evidence and measurements.
  Examples: detections, boxes, tracks, features, projected 3D observations, source frame metadata.

World model:
  Owns fusion, provenance, enrichment, consistency checks, reprojection residuals, lifecycle state, and stable agent handles.
  It converts perception evidence into stateful WorldSnapshot products.

Behavior / planning clients:
  Consume WorldSnapshot state only.
  They should not reach backward into raw PerceptionPipelineOutput, detector boxes, or tracker internals.
```

Architectural invariant:

```text
PerceptionPipelineOutput is evidence.
WorldSnapshot is autonomy state.
Behavior consumes WorldSnapshot, not perception internals.
```

This applies to reprojection too:

```text
Detection2D / Track2D / Observation3D provenance
  -> AgentState provenance and enriched reprojection metadata
  -> WorldSnapshot / world-overlay artifacts
  -> behavior and validation clients consume the world-model product
```

A behavior controller may care that a target has good geometry, fresh evidence, low reprojection residual, or stable identity. It should receive those as world-model fields or derived TargetSelection fields, not recompute them from detector output.

---

## 3. Coordinate Products and Consumers

Keep three coordinate products separate. They are connected by transforms, but they serve different system layers.

```text
1. Camera viewport projection
   Raw sensor/image coordinates.
   Typical representation: pixel coordinates, bounding boxes, image-space tracks.
   Consumer: detector, tracker, visualization, reprojection residual validation.

2. Drone ego-relative 3D
   Tactical object state relative to the current drone/body/ego frame.
   Typical representation: Cartesian body-frame vector plus optional polar/range-bearing form.
   Consumer: object-conditioned behavior, standoff/follow/circle math, local obstacle avoidance.

3. Global/map 3D
   Scene/map state in a local or global map frame.
   Typical representation: Cartesian local map coordinates, optionally anchored to absolute latitude/longitude/altitude.
   Consumer: search, route memory, go-home, mission geography, multi-frame/world-model consistency.
```

The global/map frame may be either:

```text
floating local patch:
  A mission-local Cartesian map with no absolute geodetic anchor.

anchored map:
  A local/global frame tied to latitude/longitude/altitude or another absolute reference.
```

Design rule:

```text
Do not treat camera pixels, ego-relative 3D, and global/map 3D as interchangeable.
Every transition between them must pass through an explicit transform and camera model.
```

Practical data flow:

```text
Camera viewport
  -> detector/tracker evidence
  -> ego-relative 3D estimate
  -> global/map 3D WorldSnapshot agent
  -> current-ego/camera reprojection back to viewport for validation
```

Behavior should generally consume ego-relative 3D or a target expressed relative to ego at the current tick. Global navigation should consume map/global 3D. Reprojection validation consumes map/global or ego-relative 3D plus the current ego/camera transform to produce viewport pixels.

---

## 4. Scope

This validation layer applies to:

```text
- ghost/scripted targets
- AirSim ground-truth-derived observations
- camera-derived 2D detections lifted into 3D
- tracked objects with persistent track_id/source_track_id
- selected target overlays
- behavior debug overlays
```

It should support two classes of visualization:

```text
Dedalus artifact overlays:
  Deterministic PPM/MP4 outputs used for validation and review.

AirSim viewport overlays:
  Optional operator/debug display mirrored back into the simulator view.
```

Dedalus artifact overlays are validation truth. AirSim viewport overlays are operator convenience.

---

## 5. Core Architecture

The projection/reprojection layer should be read-only with respect to behavior execution, but the metadata it computes belongs to the world-model validation/enrichment surface.

```text
WorldSnapshot + camera model + current ego pose
  -> WorldToImageProjector
  -> projected viewport coordinates / residuals / visibility reason
  -> FrameAnnotationSink / sidecar JSON / MP4 export
```

It must not feed directly back into target selection or behavior control as an out-of-band perception dependency. If behavior needs this signal, the world model or TargetSelector should expose it as world-model-derived target quality.

Keep the autonomy and visualization chains separate:

```text
Autonomy:
  WorldSnapshot -> TargetSelector -> BehaviorRuntime -> VelocityCommand

Visualization/validation:
  WorldSnapshot -> WorldToImageProjector -> Annotator -> PPM/MP4/AirSim debug overlay
```

---

## 6. Coordinate-Frame Contract

The projection path should be explicit and unit-tested.

```text
point_local_or_map
  -> ego/body frame
  -> camera frame
  -> normalized image plane
  -> pixel coordinate
```

Inputs:

```text
WorldSnapshot.active_map_frame_id
WorldSnapshot.ego.local_T_body
AgentState.position_local
Camera intrinsics
Camera extrinsics relative to body
```

Projection math, conceptually:

```text
p_body = R_body_from_local * (p_local_agent - p_local_body)
p_camera = R_camera_from_body * p_body + t_camera_from_body
u = fx * (x_camera / z_camera) + cx
v = fy * (y_camera / z_camera) + cy
```

The exact AirSim/PX4/Dedalus axis convention must be isolated in one module and locked down by tests. Do not spread axis swaps throughout annotation code.

---

## 7. Camera Model

Start with explicit config. Do not depend on live AirSim camera introspection for the first deterministic implementation.

Example future config shape:

```yaml
camera_model:
  image_width: 640
  image_height: 360
  horizontal_fov_deg: 90
  body_T_camera:
    translation_m: [0.0, 0.0, 0.0]
    rotation_rpy_deg: [0.0, 0.0, 0.0]
```

Derived intrinsics:

```text
fx = (width / 2) / tan(horizontal_fov / 2)
fy = fx, or derived from vertical FOV/aspect ratio
cx = (width - 1) / 2
cy = (height - 1) / 2
```

Later, AirSim-specific providers may fill these values from simulator settings or camera info, but the validation contract should not depend on that.

---

## 8. Proposed Reprojection Types

Current/projected files:

```text
include/dedalus/visualization/world_to_image_projector.hpp
src/visualization/world_to_image_projector.cpp
include/dedalus/visualization/world_overlay_sidecar.hpp
src/visualization/world_overlay_sidecar.cpp
```

Current/projected types:

```cpp
struct WorldImageCameraIntrinsics {
    int width;
    int height;
    double fx;
    double fy;
    double cx;
    double cy;
    double near_plane_m;
};

struct WorldImageCameraExtrinsics {
    Pose3 body_T_camera;
};

struct ProjectedWorldPoint {
    bool visible;
    double u_px;
    double v_px;
    double depth_m;
    Vec3 position_ego_relative;
    Vec3 position_body;
    Vec3 position_camera;
    double range_m;
    double bearing_deg;
    std::string reason;
};
```

Core function:

```cpp
ProjectedWorldPoint project_local_point_to_image(
    const Vec3& point_local,
    const EgoState& ego,
    const WorldToImageProjectionConfig& config);
```

Visibility reasons:

```text
visible
invalid_input
behind_camera
too_close
outside_image
map_frame_mismatch
missing_camera_model
```

---

## 9. Reprojection Residual Semantics

A reprojection residual is the pixel error between where an object was originally observed in the camera image and where the world model predicts that object should appear after projecting the world-model agent back into the camera image.

For a camera-derived detection:

```text
source_center_px:
  The center of the original detector or tracker box in the camera viewport.

reprojected_center_px:
  The pixel coordinate produced by projecting AgentState.position_local through the current ego pose and camera model.

residual_px:
  Euclidean pixel distance between source_center_px and reprojected_center_px.
```

Formula:

```text
residual_px = sqrt((u_reprojected - u_source)^2 + (v_reprojected - v_source)^2)
```

Example:

```text
source_bbox_px:
  x=260, y=160, width=80, height=180

source_center_px:
  [300, 250]

reprojected_center_px:
  [306, 247]

residual_px:
  sqrt((306 - 300)^2 + (247 - 250)^2)
  = 6.7 px
```

Interpretation:

```text
small residual:
  The world-model 3D position is consistent with the image evidence.

large residual:
  The world-model projection and the image evidence disagree.
```

A large residual is a diagnostic signal, not a single root cause. It can indicate:

```text
- wrong or stale depth estimate
- wrong camera intrinsics or FOV
- wrong camera/body extrinsics
- wrong ego pose or orientation
- stale track/world-model position
- detection-to-track association error
- object moved but world state was not refreshed
- coordinate-frame convention bug
```

Residuals can still be useful when the reprojected point is outside the image, as long as the projection produced finite pixel coordinates. In that case, the sidecar can report `reason: outside_image` and still include `residual_px`. That tells us how far the world-model prediction is from the original image evidence, even though the projected point is not currently drawable inside the viewport.

Ghost/scripted agents do not have source image evidence, so they should normally omit `source_bbox_px`, `source_center_px`, and `residual_px`. They can still report projected coordinates, visibility, depth, range, and bearing.

Architectural rule:

```text
Residuals are world-model validation/enrichment signals.
Behavior should not recompute residuals from detector boxes.
If behavior needs target-quality information, expose it as WorldSnapshot / TargetSelection quality metadata.
```

---

## 10. Detection Provenance Needed for Residuals

For true perception validation, `Observation3D` and `AgentState` preserve enough provenance to compare reprojection against image evidence.

Current links:

```text
Detection2D.detection_id
Detection2D.bbox_px
Detection2D.frame_id
Track2D.track_id
Track2D.source_detection_id
Observation3D.track_id
Observation3D.source_detection_id
Observation3D.source_bbox_px
Observation3D.source_frame_id
AgentState.source_track_id
AgentState.source_detection_id
AgentState.source_bbox_px
AgentState.source_frame_id
AgentState.agent_id
```

This allows validation like:

```text
original detection center_px:  [u0, v0]
world-model reprojection_px:  [u1, v1]
residual_px:                 sqrt((u1-u0)^2 + (v1-v0)^2)
```

The key rule is that residuals and quality indicators must become world-model-side metadata. They should not remain as private detector/tracker implementation details if behavior or validation will depend on them.

---

## 11. Artifact Format

For each annotated frame, write a sidecar JSON file:

```text
out/<annotation_dir>/
  frame_000001.ppm
  frame_000001.world_overlay.json
  manifest.txt
```

Example sidecar:

```json
{
  "frame_id": "frame_000001",
  "timestamp_ns": 1234567890,
  "coordinate_products": {
    "viewport": "camera pixel coordinates",
    "ego_relative_3d": "current drone/body-relative tactical coordinates",
    "map_3d": "floating or anchored local/global Cartesian map coordinates"
  },
  "camera_model": {
    "width": 640,
    "height": 360,
    "fx": 320.0,
    "fy": 320.0,
    "cx": 319.5,
    "cy": 179.5
  },
  "agents": [
    {
      "agent_id": "agent_ghost_person_001",
      "source_track_id": "ghost_person_001",
      "identity_id": "identity_ghost_person_001",
      "class": "person",
      "confidence": 0.82,
      "position_local": [12.0, -4.0, 0.0],
      "position_ego_relative": [12.0, -4.0, 2.0],
      "range_m": 12.8,
      "bearing_deg": -18.4,
      "visible": true,
      "u_px": 421.3,
      "v_px": 180.0,
      "depth_m": 12.6,
      "reason": "visible"
    }
  ]
}
```

For camera-derived detections, include residual fields:

```json
{
  "source_detection_id": "det_000123",
  "source_track_id": "track_0007",
  "source_bbox_px": {"x": 375.0, "y": 130.0, "width": 80.0, "height": 100.0},
  "source_center_px": [415.0, 181.0],
  "reprojected_center_px": [421.3, 180.0],
  "residual_px": 6.4
}
```

---

## 12. Visual Overlay Semantics

Use distinct overlay prefixes:

```text
D:
  detector-space 2D evidence

T:
  tracker-space 2D track

AG:
  world-model agent reprojected into the camera frame

SEL:
  selected target from TargetSelector

BH:
  behavior/runtime state
```

For ghost/scripted targets:

```text
AG:ghost_person_001 person C0.82
```

For camera-derived detections:

```text
D:person det_000123
T:track_0007
AG:track_0007 person err=6.4px
```

This makes it visually obvious whether the overlay is detector evidence or world-model reprojection.

---

## 13. Stationary-Object / Moving-Drone Stress Case

A key stress case is a stationary object in the world while the drone moves.

Expected behavior:

```text
Object world/local position:
  approximately constant

Drone ego-relative vector to object:
  changes as the drone translates/yaws

Reprojected viewport coordinate:
  changes significantly, sometimes rapidly
```

This validates that reprojection uses the current ego pose and camera model, not stale image coordinates.

Test scenario:

```text
Frame 1:
  ego at x=0, object at local [10, 0, 0]
  ego-relative vector is [10, 0, 0]
  reprojection near image center

Frame 2:
  ego translates right/left or yaws
  object local/map position remains [10, 0, 0]
  ego-relative vector changes
  reprojection moves across the viewport

Frame 3:
  ego passes object or turns away
  object local/map position remains stable
  object becomes near edge, outside image, or behind camera
```

Assertions:

```text
- object agent_id/source_track_id remains stable
- map/global position remains constant or changes only within tolerance
- ego-relative vector changes with drone motion
- projected u/v changes as ego pose changes
- visibility flips to outside_image/behind_camera when appropriate
- no target identity switch occurs due solely to image motion
```

This is especially important because a static object can have highly dynamic image motion under drone translation, yaw, and altitude changes. The world model should preserve the object while the viewport projection moves.

---

## 14. Ghost Targets vs Camera-Derived Detections

Ghost targets are the easiest deterministic starting point because their 3D positions are known.

```text
Ghost validation:
  scripted local position -> WorldSnapshot -> reproject to viewport
```

Camera-derived detection validation adds an extra loop:

```text
2D detection -> projected 3D observation -> WorldSnapshot -> reproject to viewport -> compare with 2D detection/track
```

Both should use the same reprojection module.

Do not create a ghost-only projection path. Ghosts are just one source of WorldSnapshot agents.

---

## 15. AirSim Viewport Overlay

AirSim viewport overlay is a later optional adapter.

Two options:

```text
AirSim debug drawing:
  Convert WorldSnapshot local position to AirSim world coordinates and call debug draw point/sphere/text APIs.

Spawned debug actors:
  Create visible debug-only objects in the simulator scene.
```

Preferred first approach:

```text
AirSim debug drawing, if supported by the Colosseum/AirSim build.
```

Avoid spawned actors initially because they can contaminate the simulation scene and perception pipeline.

Rules:

```text
- optional only
- operator/debug display only
- never validation truth
- never modifies mission state
- never contaminates binary bridge stdout
- must be easy to disable
```

---

## 16. Implementation Order

```text
2.24G.1 — Add WorldToImageProjector contract and math tests. DONE.

2.24G.2 — Add camera intrinsics/extrinsics config defaults. DONE for deterministic default camera model.

2.24G.3 — Reproject WorldSnapshot.agents into PPM annotation artifacts. DONE.

2.24G.4 — Emit frame_XXXXXX.world_overlay.json sidecars. DONE.

2.24G.5 — Strengthen world_reprojection_artifacts test to verify projected visible agents and sidecars. DONE.

2.24G.6 — Add stationary-object / moving-drone reprojection stress test. DONE at unit level.

2.24G.7 — Add detection residual fields once Observation3D preserves source 2D detection provenance. DONE baseline.

2.24G.8 — Keep MP4 export path as review artifact generation from annotated PPM sequences. DONE baseline.

Next — promote reprojection/residual quality from sidecar-only artifact into explicit world-model quality fields if behavior needs to depend on it.

Later optional adapter — AirSim viewport debug overlay from WorldSnapshot agents.
```

---

## 17. Validation Commands

After implementation:

```bash
cmake -S . -B build-staging
cmake --build build-staging -j$(nproc)
ctest --test-dir build-staging --output-on-failure -L contracts
ctest --test-dir build-staging --output-on-failure -L unit
ctest --test-dir build-staging --output-on-failure -L synthetic
```

Focused expected tests:

```bash
ctest --test-dir build-staging --output-on-failure -R 'world_to_image_projector|world_reprojection_artifacts|ppm_frame_annotation_sink'
```

Full validation:

```bash
ctest --test-dir build-staging --output-on-failure
```
