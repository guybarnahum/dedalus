# Visual Obstacle Detection Transition Plan

This document defines the transition from AirSim GT visual-emulation to actual visual obstacle detection.

The core point:

```text
4.1B made sensing coverage real.
4.1C starts actual visual obstacle detection.
```

AirSim GT visual-emulation remains useful, but only as a validation oracle clipped by explicit camera sensing coverage. It must not become the detector architecture.

## 3.x object-GT versus 4.x obstacle sensing

AirSim object-GT is not the 4.x obstacle detector.

```text
3.x perception / object world model:
  object detection
  class labels
  re-ID
  target selection
  approximate 3D object pose
  AirSim named-object GT for semantic/object validation

4.x obstacle sensing / avoidance:
  classless geometry
  depth/ray/mesh returns
  occupied/free/unknown evidence volumes
  ego occupancy
  swept-volume risk
  obstacle avoidance
```

Do not expand 4.x obstacle detection by adding more AirSim object patterns for trees, cars, roofs, floors, walls, or terrain. Those are semantic/object approximations and still miss geometry. The 4.x simulation detector must consume classless geometry inside the current sensing volume.

## Current state after 4.1B

The 4.1B stack now has the required detector-side geometry boundary:

```text
mission_options.obstacle_sensing.cameras.*
  -> CameraSensingConfig
  -> SensingCoverageProvider
  -> CameraSensingVolume
  -> ObstacleSensingVolume in WorldSnapshot
```

Camera/gimbal pitch is propagated through:

```text
CameraPointingCommand
  -> CameraPointingStateStore
  -> CoreStackRunner.update_camera_pointing_states(...)
  -> SensingCoverageProvider
  -> pitched CameraSensingVolume
```

AirSim GT visual-emulation now behaves as:

```text
AirSim GT global oracle:
  all configured/query-scope GT objects

AirSim GT visual-emulation:
  only named GT objects inside current explicit camera sensing coverage

No sensing coverage:
  no visual-emulation evidence
```

That was useful to validate sensing coverage, but it is not a 4.x obstacle detector because it does not include arbitrary scene geometry such as floor, roofs, walls, terrain, tree geometry, or unlabeled structures.

## What actual visual obstacle detection means

Actual visual obstacle detection is a provider that consumes sensor-derived frame information and current camera coverage, then emits normalized classless `ObstacleEvidence`.

Detector input:

```text
EgoSensingFrame
  FramePacket
  EgoState
  CameraSensingVolume for FramePacket.camera_id
```

Detector output:

```text
std::vector<ObstacleEvidence>
```

The detector must not:

```text
infer coverage from ego yaw
use AirSim named-object GT as obstacle geometry
query global AirSim GT as if it were visual perception
require YOLO/DETR/semantic class labels
emit evidence for pixels/rays outside current camera coverage
emit evidence when no valid sensing coverage exists
```

The detector should:

```text
use the current frame camera_id
use per-frame intrinsics and sensing volume
produce occupied / free / unknown geometric evidence
populate range, bearing, elevation, sensor_name, source_frame_id
mark source_kind/source_provider as a real visual provider, not AirSim GT visual-emulation
```

## First detector: AirSim depth-frame geometric evidence

The first actual detector should be an AirSim depth-frame provider, not an RGB-only learned model.

Reason:

```text
AirSim depth is still sensor-derived visual geometry.
It is deterministic enough for CI/simulation validation.
It avoids mixing object semantics into obstacle avoidance.
It validates the obstacle evidence and occupancy contracts before ML complexity.
```

Initial provider name:

```text
airsim_depth_obstacle_detector
```

Suggested source kind/provider:

```text
source_kind: visual_depth
source_provider: airsim_depth_obstacle_detector
```

If `OccupancySourceKind` does not yet have a suitable enum, add one explicitly rather than overloading `AirSimGroundTruthVisualEmulation`.

## 4.1C implementation stages

### 4.1C.1 — visual obstacle detector contract

Add a provider contract that is independent of semantic perception:

```cpp
class VisualObstacleDetector {
public:
    virtual ~VisualObstacleDetector() = default;
    virtual std::vector<ObstacleEvidence> detect(const EgoSensingFrame& frame) = 0;
};
```

The detector should live near the sensing/geometric layer, not inside:

```text
FlightCommandSink
ObjectBehaviorMissionController
rough_flight_map_builder.cpp
semantic Detector / Tracker / IdentityResolver
```

Suggested files:

```text
include/dedalus/sensing/visual_obstacle_detector.hpp
src/sensing/visual_obstacle_detector.cpp
tests/unit/test_visual_obstacle_detector_contract.cpp
```

### 4.1C.2 — AirSim depth obstacle detector

Status: validated end-to-end for the AirSim depth sidecar path. See `docs/airsim_depth_obstacle_detector_validation.md`.

Implement a simulation provider that uses depth frames/rays to create classless obstacle evidence.

Minimum viable behavior:

```text
sample depth image on a coarse grid
reject invalid/no-return depth
back-project depth samples through camera intrinsics
transform samples into local/map frame using CameraSensingVolume axes
emit occupied evidence for depth returns
optionally emit free-space evidence along rays before returns
```

Start simple:

```text
one occupied voxel per Nth pixel/ray
confidence based on valid depth and range
range/bearing/elevation filled from camera ray
source_frame_id preserved
inside_sensing_volume = true by construction
```

Do not start with dense occupancy mapping. First prove evidence emission.

The first implementation slice should be the detector core, independent of the AirSim RPC transport:

```text
DepthFrame + ObstacleSensingVolume
  -> airsim_depth_obstacle_detector
  -> std::vector<ObstacleEvidence>
```

The live AirSim integration follows after the core contract compiles and validates:

```text
AirSim depth image request
  -> DepthFrame
  -> airsim_depth_obstacle_detector
  -> CoreStackRunner obstacle evidence handoff
  -> WorldSnapshot.obstacle_evidence
  -> OSD volumetric evidence renderer
```

Validated live dataflow:

```text
AirSim DepthPlanar
  -> simulation/airsim/scripts/airsim-stream-frames-binary.py --include-depth
  -> RGB binary frame + JSON sidecar depth grid
  -> AirSimFrameSource FramePacket.depth_frame
  -> CoreStackRunner current obstacle_sensing_volumes
  -> AirSimDepthObstacleDetector
  -> PerceptionPipelineOutput.obstacle_evidence
  -> InMemoryWorldModel
  -> WorldSnapshot.obstacle_evidence
  -> RuntimeEventStreamServer
  -> simulation/airsim/scripts/airsim-world-overlay.py volumetric evidence renderer
```

Representative validated run:

```text
out/object_behavior_airsim_existing_object_circle_depth_probe
```

Observed validation summary:

```text
bridge:
  include_depth=True
  stride=16
  sidecar grid=16x9
  valid samples ~= 118-126 per frame

snapshots:
  count=908
  obstacle evidence provider:
    airsim_depth_obstacle_detector: 112275
  obstacle evidence source kind:
    depth_provider: 112275
  depth evidence first=snapshot_0001.json
  depth evidence last=snapshot_0908.json

latest snapshots:
  provider=airsim_depth_obstacle_detector only
  evidence count ~= 46-53 per frame

OSD:
  obstacle_evidence render providers={'airsim_depth_obstacle_detector': 46-96}
```

Sampling invariant:

```text
Bridge --depth-stride controls acquisition / transport downsampling.
Detector pixel_stride controls optional detector-side second-pass decimation.
Detector default pixel_stride must be 1 so every received sidecar sample is consumed.
```

The first implementation incorrectly double-strided a 16x9 sidecar grid with detector `pixel_stride=16`, producing only one stable corner sample per frame. The validated fix consumes every received sidecar sample by default.

### 4.1C.3 — CoreStackRunner integration

Insert the real visual detector after sensing coverage is computed and before world-model ingest:

```text
FramePacket + EgoState
  -> SensingCoverageProvider
  -> EgoSensingFrame
  -> VisualObstacleDetector
  -> ObstacleEvidence
  -> EgoOccupancyMapper / WorldSnapshot
```

AirSim GT visual-emulation remains a parallel validation source, not the detector.

For 4.x obstacle sensing, fallback semantics must be explicit:

```text
If airsim_depth_obstacle_detector is enabled:
  WorldSnapshot.obstacle_evidence should represent the depth detector result.
  Empty depth evidence means no observed depth evidence for that frame.
  Do not silently replace empty depth evidence with AirSim named-object GT boxes.
```

AirSim object-GT remains useful for:

```text
3.x semantic/object perception validation
target behavior validation
approximate object pose validation
debugging object-conditioned behaviors
```

It is not the 4.x obstacle detector.

Next configuration work:

```text
obstacle_sensing.detectors.airsim_depth.enabled
obstacle_sensing.detectors.airsim_depth.depth_stride
obstacle_sensing.detectors.airsim_depth.max_range_m
obstacle_sensing.detectors.airsim_depth.voxel_size_m
obstacle_sensing.detectors.airsim_depth.max_evidence
obstacle_sensing.detectors.airsim_depth.disable_object_gt_fallback
```

### 4.1C.4 — artifact validation

Extend artifact validators to distinguish:

```text
airsim_gt_visual_emulation:
  oracle-clipped validation evidence

airsim_depth_obstacle_detector:
  real sensor-derived visual evidence
```

Validators should ensure:

```text
real detector evidence has source_frame_id
real detector evidence references a declared sensing camera
real detector evidence is inside sensing coverage
real detector evidence has valid range/bearing/elevation
real detector evidence does not require semantic class labels
```

### 4.1C.5 — ego occupancy mapper handoff

Once real evidence is emitted, normalize both sources into the same occupancy contract:

```text
ObstacleEvidence
  -> EgoOccupancyMapper
  -> EgoOccupancyMapSnapshot
  -> WorldSnapshot.ego_occupancy
```

The mapper should not care whether evidence came from:

```text
airsim_gt_visual_emulation
airsim_depth_obstacle_detector
future stereo provider
future monocular depth provider
future freespace/segmentation provider
```

## Later detectors

After the AirSim depth detector is validated:

```text
stereo depth provider
monocular depth provider
classless freespace / obstacle-mask provider
optical-flow / looming provider
thin-structure specialized provider
```

Semantic detectors may enrich evidence but must not be required for obstacle avoidance.

## Completion criteria for leaving 4.1B

4.1B is complete when:

```text
explicit sensing coverage is required and validated
camera pitch affects sensing coverage and evidence clipping
AirSim GT visual-emulation has no fake fallback coverage
live AirSim configs declare obstacle_sensing.cameras.*
artifact validator rejects fake visual-emulation sensing volumes
this transition plan is in the required read set
```

Then the next active work should be:

```text
4.1C.1 — visual obstacle detector contract
4.1C.2 — AirSim depth-frame classless obstacle detector
```
