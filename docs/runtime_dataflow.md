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
         evaluates GhostScenario once at frame time
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

External live subscribers
  RuntimeEventStreamServer
    subscribes to GhostDetectionsPublisher and WorldSnapshotPublisher
    emits TCP JSONL runtime events
    -> simulation/airsim-world-overlay.py
    -> external/debug/customer tools
```

---

## 2. Publisher / subscriber topology

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
Behavior does not consume GhostDetectionsFrame directly.
```

---

## 3. Runtime event stream records

The optional runtime event stream is enabled by:

```bash
./build-staging/apps/dedalus_mission_loop \
  --config config/core_stack_object_behavior_airsim_ghost.yaml \
  --output-dir out/object_behavior_airsim_ghost \
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

Sequence numbers are stream-local and shared across event types. A client should use `type` to dispatch records and `seq` to detect gaps.

---

## 4. AirSim overlay subscriber dataflow

`simulation/airsim-world-overlay.py` is now a subscriber/renderer only.

```text
RuntimeEventStreamServer
  -> TCP JSONL stream
  -> simulation/airsim-world-overlay.py
       cache latest ghost_detections
       cache latest world_snapshot
       render PLAN / PLAN* markers from ghost_detections
       render AG markers from world_snapshot.agents
       render EGO marker from world_snapshot.ego
  -> AirSim simPlotPoints / simPlotStrings
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
  --debug
```

The overlay must not:

```text
- evaluate GhostScenario locally
- poll snapshot_manifest.txt in normal mode
- read mission_events.jsonl for normal SEL state
- decide between source modes such as combined/world_snapshot/artifact_snapshot
```

The overlay simply renders whichever event types arrive. Visibility controls should be simple renderer flags, such as `--hide-planned`, `--hide-world`, or `--hide-ego`.

---

## 5. Behavior / flight sink dataflow

```text
WorldSnapshotPublisher
  -> LatestWorldSnapshotSubscriber
  -> LatestWorldSnapshot
  -> MissionRuntime async loop
  -> ObjectBehaviorMissionController
       -> TargetSelector reads WorldSnapshot.agents
       -> behavior math emits bounded velocity intent
  -> Px4BridgeCommandSink
  -> simulation/px4-command-bridge.py
       PX4 shell: arm / takeoff / land / disarm
       pymavlink: OFFBOARD velocity setpoints
  -> PX4 / AirSim
```

Flight sinks receive bounded kinematic intent only. They must not own target selection, world-model reasoning, obstacle avoidance, or mission semantics.

---

## 6. Artifact dataflow

```text
WorldSnapshotPublisher
  -> ArtifactSnapshotWriter
  -> out/.../snapshot_XXXX.json
  -> out/.../snapshot_manifest.txt

MissionRuntime
  -> mission_events.jsonl

FrameAnnotator
  -> PPM frames and world_overlay sidecars

Validators
  -> read artifacts after the run
```

Artifacts are still important for evidence, regression tests, and human debugging. They are not the normal communication path between runtime components.

---

## 7. Planned next event type

SEL markers should come from live mission state, not files. The next clean slice should publish either:

```text
MissionEventPublisher
  -> RuntimeEventStreamServer
  -> event type mission_event
```

or a narrower selected-target state stream:

```text
SelectedTargetPublisher
  -> RuntimeEventStreamServer
  -> event type selected_target
```

Until then, the overlay renders PLAN / PLAN* / AG / EGO, but not live SEL.
