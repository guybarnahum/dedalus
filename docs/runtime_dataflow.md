# Runtime Dataflow

This document captures the current Dedalus runtime dataflow from sources, through publishers, through the optional runtime event server, to subscribers and sinks.

The core rule is:

```text
Control-critical consumers should subscribe in-process.
External tools should consume typed runtime events.
Artifact files are evidence/debug outputs, not IPC.
```

---

## 1. Main live mission dataflow

```text
AirSim / PX4
  -> simulation/airsim/scripts/airsim-stream-frames-binary.py
       stdout: binary frame protocol only
       stderr: human diagnostics
  -> AirSimFrameSource
  -> FrameHintEgoProvider
  -> CoreStackRunner
       -> PerceptionPipeline
       -> optional GhostTargetProvider::frame_at(...)
            -> GhostDetectionsFrame
            -> Observation3D list
       -> InMemoryWorldModel
       -> WorldSnapshot
  -> publishers and subscribers
  -> MissionRuntime
  -> FlightCommandSink
  -> PX4 / AirSim
```

Expanded current view:

```text
Sources
  AirSim RGB frame + ego sidecar
    -> AirSimFrameSource
    -> FrameHintEgoProvider

Perception / simulation injection
  CoreStackRunner
    -> PerceptionPipeline.process(frame, ego)
    -> GhostTargetProvider::frame_at(timestamp, map_frame, scenario_start)
         evaluates one configured ghost target source at frame time
         returns GhostDetectionsFrame
         returns matching Observation3D objects
    -> append ghost Observation3D objects to PerceptionPipelineOutput.observations

World state
  InMemoryWorldModel.update_ego(ego)
  InMemoryWorldModel.ingest(perception_output)
  InMemoryWorldModel.snapshot()

Publishers
  GhostDetectionsPublisher.publish(GhostDetectionsFrame)
  WorldSnapshotPublisher.publish(WorldSnapshot)
  MissionEventPublisher.publish(MissionEvent)

In-process subscribers
  LatestWorldSnapshotSubscriber
    -> LatestWorldSnapshot
    -> MissionRuntime
    -> ObjectBehaviorMissionController / TrajectoryMissionController
    -> Px4BridgeCommandSink
    -> tools/px4/px4-command-bridge.py
    -> PX4 / AirSim

Evidence subscribers
  ArtifactSnapshotWriter
    -> snapshot_XXXX.json
    -> snapshot_manifest.txt

Mission event artifact path
  MissionRuntime.write_event(...)
    -> MissionEventPublisher
    -> mission_events.jsonl

External live subscribers
  RuntimeEventStreamServer
    subscribes to GhostDetectionsPublisher, WorldSnapshotPublisher, and MissionEventPublisher
    emits TCP JSONL runtime events
    -> simulation/airsim/scripts/airsim-world-overlay.py
    -> external/debug/customer tools
```

---

## 2. Planned sensing coverage boundary

The obstacle-sensing path must treat camera coverage as a first-class runtime input. Vehicle yaw and camera optical coverage are different volumes: the vehicle may fly forward while the camera is pitched down or pointed by a gimbal.

The planned in-process dataflow is:

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

The detector-facing input should be:

```text
EgoSensingFrame
  FramePacket
  EgoState
  CameraSensingVolume for FramePacket.camera_id
```

Rules:

```text
Sensing coverage is derived from camera geometry and pointing state, not ego yaw alone.
World model publishes sensing volumes/evidence, but does not infer camera coverage after perception.
AirSim global GT is an oracle of scene candidates; AirSim visual emulation is clipped to current camera coverage.
```

See `docs/sensing_coverage_architecture.md` for the full staged plan.

---

## 3. Ghost target source modes

Ghost targets are simulation/debug inputs that deliberately enter at the same semantic boundary as real 3D detections: `Observation3D` appended to `PerceptionPipelineOutput.observations`.

Current implemented source modes:

```text
trajectory_scenario:
  config/behaviors/ghost_detections/*.json
    -> references config/behaviors/trajectories/*.json
    -> GhostScenario evaluates static and dynamic detections at frame time
    -> GhostDetectionsFrame + Observation3D list

airsim_objects:
  explicit existing AirSim scene object name
    -> simulation/airsim/scripts/airsim-object-poses.py
    -> AirSim simGetObjectPose(object_name)
    -> GhostDetectionState at that AirSim pose
    -> GhostDetectionsFrame + Observation3D list
```

Both modes must produce the same downstream products:

```text
GhostDetectionsFrame
  -> GhostDetectionsPublisher
  -> RuntimeEventStreamServer
  -> overlay PLAN / PLAN*

Observation3D list
  -> PerceptionPipelineOutput.observations
  -> InMemoryWorldModel
  -> WorldSnapshot.agents
  -> TargetSelector
  -> ObjectBehaviorMissionController
  -> MissionEventPublisher target_selected
  -> overlay SEL
```

The overlay must not discover scene objects or generate ghosts. Existing-object binding belongs in the mission-loop/provider side so visualization and behavior share one source of truth.

The implemented AirSim object mode supports explicit object selection:

```yaml
ghost_targets_enabled: true
ghost_targets_source: airsim_objects

ghost_targets_airsim.objects.0.source_track_id: ghost_person_001
ghost_targets_airsim.objects.0.airsim_object_name: BRPlayer_01_96
ghost_targets_airsim.objects.0.class: person
ghost_targets_airsim.objects.0.confidence: 0.82
ghost_targets_airsim.objects.0.size_m: [0.6, 0.6, 1.8]
```

Later convenience modes may add `all` and seeded `random`, but they should not precede deterministic explicit binding.

---

## 4. Publisher / subscriber topology

```text
                          +-----------------------------+
                          |       CoreStackRunner        |
                          +-----------------------------+
                                      |
                 +--------------------+--------------------+
                 |                                         |
                 v                                         v
      +-------------------------+              +-------------------------+
      | GhostDetectionsPublisher|              | WorldSnapshotPublisher  |
      +-------------------------+              +-------------------------+
                 |                                         |
                 |                                         +--> LatestWorldSnapshotSubscriber
                 |                                         |      -> LatestWorldSnapshot
                 |                                         |      -> MissionRuntime
                 |                                         |            -> MissionEventPublisher
                 |                                         |            -> mission_events.jsonl
                 |                                         |      -> behavior controller
                 |                                         |      -> FlightCommandSink
                 |                                         |
                 |                                         +--> ArtifactSnapshotWriter
                 |                                                -> snapshot files
                 |
                 +--------------------+--------------------+
                                      |
                                      v
                          +-----------------------------+
                          |  RuntimeEventStreamServer   |
                          |  TCP JSONL event stream     |
                          +-----------------------------+
                                      ^
                                      |
                          +-----------------------------+
                          |    MissionEventPublisher    |
                          +-----------------------------+
                                      |
                                      v
                          +-----------------------------+
                          | simulation/airsim-world-    |
                          | overlay.py and other tools  |
                          +-----------------------------+
```

Important boundary:

```text
GhostDetectionsPublisher is for simulation/debug visibility.
WorldSnapshotPublisher is the behavior-facing autonomy state boundary.
MissionEventPublisher exposes behavior/mission transitions and selected-target state.
Behavior does not consume GhostDetectionsFrame directly.
```

---

## 5. Runtime event stream records

The optional runtime event stream is enabled by:

```bash
./build-staging/apps/dedalus_mission_loop \
  --config config/core_stack_object_behavior_airsim_existing_object.yaml \
  --output-dir out/object_behavior_airsim_existing_object \
  --world-snapshot-stream-port 47770 \
  --progress
```

The flag name is currently retained for operator compatibility, but the server is now a generic runtime-event stream internally.

Current JSONL record types:

```json
{"type":"ghost_detections","seq":1,"timestamp_ns":123,"map_frame_id":"map_airsim_mission_0001","ghost_detections":{"timestamp_ns":123,"map_frame_id":"map_airsim_mission_0001","scenario_elapsed_s":0.0,"detections":[]}}
```

```json
{"type":"world_snapshot","seq":2,"timestamp_ns":123,"active_map_frame_id":"map_airsim_mission_0001","snapshot":{}}
```

```json
{"type":"mission_event","seq":3,"timestamp_ns":124,"mission_event":{"event":"target_selected","tick":42,"state":"ExecuteMission","display_state":"Mission","display_detail":"arriving","source_track_id":"ghost_person_001","agent_id":"agent_ghost_person_001"}}
```

Sequence numbers are stream-local and shared across event types. A client should use `type` to dispatch records and `seq` to detect gaps.

---

## 6. AirSim overlay subscriber dataflow

`simulation/airsim/scripts/airsim-world-overlay.py` is now a subscriber/renderer only.

```text
RuntimeEventStreamServer
  -> TCP JSONL stream
  -> simulation/airsim/scripts/airsim-world-overlay.py
       cache latest ghost_detections
       cache latest world_snapshot
       cache latest target_selected mission_event
       cache latest mission_event for OSD state
       render PLAN / PLAN* markers from ghost_detections
       render AG markers from world_snapshot.agents
       render EGO marker from world_snapshot.ego
       render SEL marker by matching target_selected source_track_id / agent_id to world_snapshot.agents
       render obstacle_sensing_volumes as camera coverage markers
       render obstacle_evidence as occupied/free/unknown/thin-risk evidence markers
       render latest_swept_volume as collision query marker
       render DEDALUS numeric OSD from world_snapshot ego motion
       render DEDALUS-STATE from mission_event display_state/display_detail
  -> AirSim simPlotPoints / simPlotStrings / simPrintLogMessage
```

Run:

```bash
python3 simulation/airsim/scripts/airsim-world-overlay.py \
  --stream-port 47770 \
  --host 127.0.0.1 \
  --rpc-port 41451 \
  --vehicle-name PX4 \
  --show-world-agents \
  --show-sensing-volumes \
  --show-obstacle-evidence \
  --show-swept-volume \
  --osd
```
