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
  -> simulation/airsim-stream-frames-binary.py
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

Expanded view:

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
    -> simulation/px4-command-bridge.py
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
    -> simulation/airsim-world-overlay.py
    -> external/debug/customer tools
```

---

## 2. Ghost target source modes

Ghost targets are simulation/debug inputs that deliberately enter at the same semantic boundary as real 3D detections: `Observation3D` appended to `PerceptionPipelineOutput.observations`.

Current implemented source modes:

```text
trajectory_scenario:
  simulation/ghost_detections/*.json
    -> references simulation/trajectories/*.json
    -> GhostScenario evaluates static and dynamic detections at frame time
    -> GhostDetectionsFrame + Observation3D list

airsim_objects:
  explicit existing AirSim scene object name
    -> simulation/airsim-object-poses.py
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

The implemented 2.26E AirSim object mode supports explicit object selection:

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

## 3. Publisher / subscriber topology

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

## 4. Runtime event stream records

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

## 5. AirSim overlay subscriber dataflow

`simulation/airsim-world-overlay.py` is now a subscriber/renderer only.

```text
RuntimeEventStreamServer
  -> TCP JSONL stream
  -> simulation/airsim-world-overlay.py
       cache latest ghost_detections
       cache latest world_snapshot
       cache latest target_selected mission_event
       cache latest mission_event for OSD state
       render PLAN / PLAN* markers from ghost_detections
       render AG markers from world_snapshot.agents
       render EGO marker from world_snapshot.ego
       render SEL marker by matching target_selected source_track_id / agent_id to world_snapshot.agents
       render DEDALUS numeric OSD from world_snapshot ego motion
       render DEDALUS-STATE from mission_event display_state/display_detail
  -> AirSim simPlotPoints / simPlotStrings / simPrintLogMessage
```

Run:

```bash
python3 simulation/airsim-world-overlay.py \
  --stream-port 47770 \
  --follow \
  --rate-hz 5 \
  --duration-s 180 \
  --clear \
  --label \
  --osd \
  --debug
```

The overlay must not:

```text
- evaluate GhostScenario locally
- discover/list AirSim objects for ghost binding
- poll snapshot_manifest.txt in normal mode
- read mission_events.jsonl for normal SEL state
- infer behavior semantics such as following/circling/positioned; mission/behavior events must publish display_state/display_detail
- decide between source modes such as combined/world_snapshot/artifact_snapshot
```

The overlay simply renders whichever event types arrive. Visibility controls should be simple renderer flags, such as `--hide-planned`, `--hide-world`, `--hide-ego`, or `--hide-selected`.

---

## 6. Behavior / flight sink dataflow

```text
WorldSnapshotPublisher
  -> LatestWorldSnapshotSubscriber
  -> LatestWorldSnapshot
  -> MissionRuntime async loop
  -> ObjectBehaviorMissionController
       -> TargetSelector reads WorldSnapshot.agents
       -> emits mission events such as target_selected / behavior_start / behavior_complete / behavior_tick_sample
       -> emits behavior display details such as arriving / following / positioned / circling / done
       -> behavior math emits bounded velocity intent
  -> Px4BridgeCommandSink
  -> simulation/px4-command-bridge.py
       PX4 shell: arm / takeoff / land / disarm
       pymavlink: OFFBOARD velocity setpoints
  -> PX4 / AirSim

MissionRuntime
  -> MissionEventPublisher
  -> RuntimeEventStreamServer
  -> mission_event JSONL records
```

Flight sinks receive bounded kinematic intent only. They must not own target selection, world-model reasoning, obstacle avoidance, or mission semantics.

---

## 7. Artifact dataflow

```text
WorldSnapshotPublisher
  -> ArtifactSnapshotWriter
  -> out/.../snapshot_XXXX.json
  -> out/.../snapshot_manifest.txt

MissionRuntime
  -> MissionEventPublisher
  -> mission_events.jsonl

FrameAnnotator
  -> PPM frames and world_overlay sidecars

Validators
  -> read artifacts after the run
```

Artifacts are still important for evidence, regression tests, and human debugging. They are not the normal communication path between runtime components.

---

## 8. Current live overlay markers

```text
PLAN:
  dynamic planned/evaluated ghost detection from ghost_detections event.

PLAN*:
  static planned/evaluated ghost detection from ghost_detections event.

AG:
  world-model agent from world_snapshot event.

EGO:
  ego pose from world_snapshot event.

SEL:
  selected target from mission_event target_selected, rendered by matching source_track_id or agent_id to the latest world_snapshot agent.
```

For AirSim existing-object binding, PLAN / AG / SEL should appear over the real visible AirSim object whose pose drives the ghost detection.
