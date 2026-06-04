# Sensing Coverage Architecture

This document defines the obstacle-sensing coverage boundary for visual obstacle detection, ego occupancy mapping, and global spatial mapping.

## Core rule

Obstacle sensing volume is **camera optical coverage**, not vehicle heading.

A drone can fly forward while the camera is pitched down, looking at a target, looking at a landing area, or pointed by a gimbal. The obstacle detector can only produce visual evidence for what the active sensing camera can currently see. Therefore the sensing volume must be derived from the active camera geometry and pointing state, not from ego yaw alone.

```text
vehicle motion / swept volume
  !=
camera optical coverage / sensing volume
```

The avoidance and mapping stack must keep those volumes separate:

```text
Sensing coverage volume:
  what the configured camera can currently observe

Obstacle evidence:
  what a detector or visual-emulation adapter observed inside that coverage

Ego swept volume:
  where the vehicle may travel over the short horizon

Unknown-risk region:
  swept space that is not sufficiently covered by current sensing evidence
```

## Existing repo boundary to build on

`FramePacket` already carries the correct per-frame fields:

```text
FramePacket
  frame_id
  timestamp
  camera_id
  image
  intrinsics
  optional camera_T_world
  optional camera_T_body
  optional ego_hint
```

Camera/gimbal pointing is also already a separate actuator path from vehicle yaw:

```text
vehicle yaw intent
  -> PX4 velocity/yaw command path

camera/gimbal pitch intent
  -> CameraPointingCommand / camera_pointing_intent
  -> AirSim or MAVLink gimbal sink
```

The sensing coverage layer must compose both boundaries.

## Proposed dataflow

```text
FrameSource
  -> FramePacket(camera_id, image, intrinsics, optional camera_T_body)

EgoProvider
  -> EgoState

CameraPointingStateProvider
  -> latest commanded/measured pointing state per camera

SensingCoverageProvider
  -> CameraSensingVolume per camera
  -> SensingCoverageSnapshot for all cameras
  -> EgoSensingFrame for each frame

VisualObstacleDetector / visual-emulation adapter
  -> ObstacleEvidence

EgoOccupancyMapper
  -> EgoOccupancyMapSnapshot

GlobalSpatialMapper
  -> global spatial map / long-term memory

WorldSnapshotPublisher
  -> obstacle_sensing_volumes
  -> obstacle_evidence
  -> ego_occupancy
  -> latest_swept_volume
```

The visual detector should receive the frame and its camera coverage together:

```text
EgoSensingFrame
  FramePacket
  EgoState
  CameraSensingVolume for FramePacket.camera_id
```

The detector or adapter should not infer camera coverage from world-model state after the fact.

## Main contracts

### CameraSensingConfig

Stable configuration for each sensing camera:

```cpp
struct CameraSensingConfig {
    CameraId camera_id;
    std::string camera_name;
    std::string role{"visual_obstacle_detector"};

    double horizontal_fov_rad{0.0};
    double vertical_fov_rad{0.0};
    double near_range_m{0.0};
    double far_range_m{0.0};
    double min_reliable_range_m{0.0};
    double max_reliable_range_m{0.0};

    Vec3 body_T_camera_xyz_m;
    Vec3 body_T_camera_rpy_rad;

    std::string pointing_source{"camera_pointing_intent"};
};
```

### CameraPointingState

Current commanded or measured pointing state for each camera:

```cpp
struct CameraPointingState {
    CameraId camera_id;
    std::string camera_name;
    TimePoint timestamp;

    double pitch_rad{0.0};
    double yaw_rad{0.0};
    double roll_rad{0.0};

    bool valid{false};
    bool measured{false};
    std::string source{"neutral"};
};
```

The first implementation may use commanded `camera_pointing_intent`. The contract must leave room for measured gimbal feedback later:

```text
source = camera_pointing_intent
measured = false
```

Later hardware integration can use:

```text
source = mavlink_gimbal_feedback
measured = true
```

### CameraSensingVolume

Runtime geometry for one camera at one time:

```cpp
struct CameraSensingVolume {
    TimePoint timestamp;
    FrameId frame_id;
    bool has_frame_id{false};

    CameraId camera_id;
    std::string camera_name;
    std::string role;
    MapFrameId map_frame_id;

    Pose3 body_T_camera_mount;
    Pose3 body_T_camera_current;
    Pose3 map_T_camera_current;

    CameraIntrinsics intrinsics;
    double horizontal_fov_rad{0.0};
    double vertical_fov_rad{0.0};
    double near_range_m{0.0};
    double far_range_m{0.0};
    double min_reliable_range_m{0.0};
    double max_reliable_range_m{0.0};

    Vec3 origin_local;
    Vec3 forward_axis_local;
    Vec3 right_axis_local;
    Vec3 up_axis_local;

    CameraPointingState pointing_state;
};
```

`ObstacleSensingVolume` in `WorldSnapshot` can remain the published/debug-facing subset of this richer camera coverage state.

### SensingCoverageSnapshot

All active sensing volumes for the current tick:

```cpp
struct SensingCoverageSnapshot {
    TimePoint timestamp;
    MapFrameId map_frame_id;
    std::vector<CameraSensingVolume> camera_volumes;
};
```

### EgoSensingFrame

The per-camera detector input:

```cpp
struct EgoSensingFrame {
    FramePacket frame;
    EgoState ego;
    CameraSensingVolume sensing_volume;
};
```

## Configuration shape

Prefer indexed camera config so the system can support multiple visual sensing cameras:

```yaml
mission_options.obstacle_sensing.cameras.0.name: front_center
mission_options.obstacle_sensing.cameras.0.role: visual_obstacle_detector
mission_options.obstacle_sensing.cameras.0.horizontal_fov_deg: 90
mission_options.obstacle_sensing.cameras.0.vertical_fov_deg: 60
mission_options.obstacle_sensing.cameras.0.near_range_m: 0.5
mission_options.obstacle_sensing.cameras.0.far_range_m: 80
mission_options.obstacle_sensing.cameras.0.min_reliable_range_m: 1
mission_options.obstacle_sensing.cameras.0.max_reliable_range_m: 60
mission_options.obstacle_sensing.cameras.0.body_T_camera_xyz_m: [0, 0, 0]
mission_options.obstacle_sensing.cameras.0.body_T_camera_rpy_deg: [0, 0, 0]
mission_options.obstacle_sensing.cameras.0.pointing_source: camera_pointing_intent
```

Optional additional cameras:

```yaml
mission_options.obstacle_sensing.cameras.1.name: downward
mission_options.obstacle_sensing.cameras.1.role: landing_area_detector
```

## AirSim ground-truth visual-emulation

AirSim global ground truth and visual emulation are different sources:

```text
AirSim global oracle:
  all named objects that exist in the scene or configured query scope

AirSim visual-emulation adapter:
  only objects inside the current CameraSensingVolume for the configured camera
```

The adapter should consume:

```text
AirSim named-object candidates
+ SensingCoverageSnapshot
-> ObstacleEvidence(source_kind = airsim_gt_visual_emulation)
```

It should not compare a visual detector against global scene objects outside the sensing volume.

## Staged implementation plan

### Stage 1 — Contract and geometry library

Add:

```text
include/dedalus/sensing/sensing_coverage.hpp
src/sensing/sensing_coverage.cpp
tests/unit/test_sensing_coverage.cpp
```

Implement:

```text
CameraSensingConfig
CameraPointingState
CameraSensingVolume
SensingCoverageSnapshot
EgoSensingFrame
SensingCoverageProvider
point-in-camera-volume helper
```

Validation:

```text
pitch = 0 deg:
  forward-level point is inside
  lateral out-of-FOV point is outside

pitch = negative/downward:
  forward-down point is inside
  forward-level point moves toward edge or outside depending FOV

camera mount yaw offset:
  inclusion follows camera optical axis, not ego yaw

two cameras:
  each camera produces its own named sensing volume
```

### Stage 2 — Config loader support

Add indexed `mission_options.obstacle_sensing.cameras.*` parsing into typed config.

Validation:

```text
config loader accepts one or more cameras
invalid FOV/range values fail clearly
missing camera config falls back to front_center defaults only for compatibility
```

### Stage 3 — Runtime state provider

Add an in-process `CameraPointingStateProvider` or equivalent runtime state store.

Initial source:

```text
CameraPointingCommand / camera_pointing_intent
```

Future source:

```text
MAVLink gimbal feedback or AirSim simGetCameraInfo verification
```

Validation:

```text
neutral mode yields configured mount orientation
camera_pointing_intent updates per-camera pitch
stale/invalid pointing state is marked in coverage output
```

### Stage 4 — CoreStackRunner integration

Produce `EgoSensingFrame` for each camera frame before visual obstacle detection.

Dataflow becomes:

```text
FramePacket + EgoState + CameraPointingState
  -> SensingCoverageProvider
  -> EgoSensingFrame
  -> VisualObstacleDetector / visual-emulation adapter
```

Validation:

```text
FramePacket.camera_id selects the matching CameraSensingVolume
WorldSnapshot.obstacle_sensing_volumes mirrors the active coverage
coverage timestamp and frame_id are preserved
```

### Stage 5 — Visual-emulation adapter migration

Move the current AirSim ground-truth visual-emulation clipping out of `InMemoryWorldModel` and into the sensing/evidence path.

Validation:

```text
global AirSim GT occupancy still contains all candidate objects
visual-emulation evidence contains only objects inside current camera coverage
pitching camera down changes which objects become evidence
```

### Stage 6 — Ego occupancy mapper

Add `EgoOccupancyMapper` that consumes obstacle evidence and sensing coverage:

```text
SensingCoverageSnapshot
+ ObstacleEvidence
+ EgoState
-> EgoOccupancyMapSnapshot
```

Validation:

```text
occupied evidence marks occupied cells
free-space rays/frustum bins can mark free cells
unobserved swept space remains unknown
inside_swept_volume is derived from swept-volume query, not detector source
```

### Stage 7 — Global spatial mapper

Add global map accumulation with aging/decay:

```text
EgoOccupancyMapSnapshot
+ EgoState
+ time
-> global spatial map / long-term obstacle memory
```

Validation:

```text
repeated observations strengthen confidence
stale observations decay
rarely seen objects/obstacles age out or become uncertain
```

## Do not

```text
Do not derive visual obstacle coverage from vehicle yaw alone.
Do not make the world model infer camera coverage after perception.
Do not compare visual detector output to AirSim global GT outside the active camera volume.
Do not put sensing coverage or mapping policy inside a flight command sink.
Do not require semantic object classes for obstacle avoidance evidence.
```

## Immediate next code slice

Implement Stage 1 only:

```text
sensing_coverage.hpp / sensing_coverage.cpp
test_sensing_coverage.cpp
```

Keep runtime behavior unchanged until the geometry contract is tested.
