# Visual Obstacle Detection Transition Plan

This document defines the transition from AirSim GT visual-emulation to actual visual obstacle detection.

The core point:

```text
4.1B made sensing coverage real.
4.1C starts actual visual obstacle detection.
```

AirSim GT visual-emulation remains useful, but only as a validation oracle clipped by explicit camera sensing coverage. It must not become the detector architecture.

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
  only GT objects inside current explicit camera sensing coverage

No sensing coverage:
  no visual-emulation evidence
```

That is the correct precondition for implementing the real detector.

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
