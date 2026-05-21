# Object Behavior AirSim Ghost Runbook

This runbook validates object-conditioned behavior against live AirSim frames while using ghost detections as the object source.

Canonical runtime diagrams live in:

```text
docs/runtime_dataflow.md
```

As of 2.26D.7, `simulation/airsim-world-overlay.py` is a stream-only subscriber/renderer. It does not evaluate ghost scenarios locally, does not discover AirSim scene objects, and does not read `snapshot_manifest.txt` in normal operation.

2.26E direction:

```text
Bind ghost detections to existing visible AirSim scene objects.
Do not import custom meshes yet.
Do not make the overlay own ghost generation or object discovery.
```

---

## 1. Ghost target source modes

Dedalus ghost targets are simulation/debug inputs that enter at the same semantic boundary as real 3D detections:

```text
Ghost target source
  -> GhostDetectionsFrame
  -> Observation3D list
  -> PerceptionPipelineOutput.observations
  -> InMemoryWorldModel
  -> WorldSnapshot.agents
  -> TargetSelector / behavior
```

### 1.1 Implemented: trajectory_scenario

The current implemented ghost source is a deterministic JSON scenario:

```text
simulation/ghost_detections/person_pair_crossing.json
```

Each dynamic detection references an existing trajectory JSON file:

```text
simulation/trajectories/ghost_person_001_crossing.json
simulation/trajectories/ghost_person_002_crossing.json
```

Static detections use `trajectory_path: null`.

The trajectory files use the same `VelocityTrajectory` schema used by drone flight trajectory code. Do not embed trajectory segments inside the ghost detection JSON.

Current people trajectories are intentionally dynamic:

```text
cross -> wait -> cross back -> wait -> repeat
```

### 1.2 Planned 2.26E: airsim_scene_objects

The planned 2.26E source binds a ghost detection to an existing visible AirSim object:

```text
AirSim scene object name
  -> simGetObjectPose(object_name)
  -> GhostDetectionState at that pose
  -> GhostDetectionsFrame + Observation3D list
```

Initial implementation should support explicit object selection only:

```yaml
ghost_targets_enabled: true
ghost_targets_source: airsim_objects

ghost_targets_airsim:
  objects:
    - source_track_id: ghost_person_001
      airsim_object_name: BRPlayer_01_96
      class: person
      confidence: 0.82
      size_m: [0.6, 0.6, 1.8]
```

Later convenience modes may add `all` and seeded `random`, but deterministic explicit binding comes first.

Useful discovery commands:

```bash
python3 simulation/airsim-list-objects.py \
  --match-class person \
  --sort distance \
  --format table
```

```bash
python3 simulation/airsim-list-objects.py \
  --match-class animal \
  --sort distance \
  --format table
```

```bash
python3 simulation/airsim-list-objects.py \
  --only-matched \
  --output out/airsim_scene_objects.json
```

Coordinate contract:

```text
trajectory_scenario initial_position_local_m:
  map-local / AirSim-local NED coordinates, not ego-relative.

airsim_scene_objects simGetObjectPose position:
  AirSim-local NED coordinates from the simulator.

GhostDetectionState.position_local_m:
  same map-local / AirSim-local NED frame.

Observation3D.position_local:
  same value injected into perception.

WorldSnapshot.agents[].position_local:
  same map-local frame after world-model ingestion.

position_ego_relative:
  derived view of object relative to current ego, not the canonical ghost placement.
```

---

## 2. Runtime and dataflow boundaries

The AirSim ghost object-behavior run has six separate paths:

```text
1. Sensor/data input path
2. Ghost simulation/injection path
3. Runtime publish/subscribe path
4. AirSim visualization subscriber path
5. Command/control output path
6. Artifact/validation path
```

### 2.1 Sensor / data input path

Dedalus receives live camera frames and ego telemetry from AirSim/PX4 through a binary frame bridge:

```text
AirSim / PX4
  -> simulation/airsim-stream-frames-binary.py
  -> stdout binary frame protocol
  -> AirSimFrameSource
  -> FrameHintEgoProvider
  -> CoreStackRunner
```

The binary bridge stdout must remain protocol bytes only. Human diagnostics must go to stderr.

In the AirSim ghost config:

```yaml
bridge_mode: stream_binary_ego
bridge_transport: pipe
bridge_command: python3 simulation/airsim-stream-frames-binary.py --include-ego --rate-hz 5 --mavlink-armed-endpoints udpin:127.0.0.1:14540
```

### 2.2 Ghost simulation / perception injection path

Current trajectory scenario config:

```yaml
ghost_targets_enabled: true
ghost_targets_scenario_path: simulation/ghost_detections/person_pair_crossing.json
```

Runtime path:

```text
CoreStackRunner
  -> first valid frame establishes ghost source start/reference state
  -> GhostTargetProvider::frame_at(timestamp, map_frame, scenario_start)
       evaluates the configured ghost source exactly once for that frame time
       returns GhostDetectionsFrame for runtime-event subscribers
       returns matching Observation3D objects
  -> append Observation3D objects to PerceptionPipelineOutput.observations
  -> InMemoryWorldModel.ingest(...)
  -> WorldSnapshot.agents
```

The earliest possible WorldSnapshot ghost agent appears after one full frame pipeline pass:

```text
AirSim frame arrives
  -> ego estimate succeeds
  -> perception pipeline runs
  -> ghost observations are appended
  -> world_model.ingest(...)
  -> world_model.snapshot(...)
  -> WorldSnapshotPublisher.publish(...)
```

That delay is expected. PLAN markers can appear as soon as `ghost_detections` events arrive; AG markers appear after world-model ingestion and snapshot publication.

For future AirSim existing-object binding:

```text
PLAN / AG / SEL should appear over the real visible AirSim object whose pose drives the ghost detection.
```

### 2.3 Runtime publish / subscribe path

Current in-process and external consumers receive typed events through publisher boundaries:

```text
GhostDetectionsPublisher.publish(frame)
  -> RuntimeEventStreamServer

WorldSnapshotPublisher.publish(snapshot)
  -> LatestWorldSnapshotSubscriber
  -> ArtifactSnapshotWriter
  -> RuntimeEventStreamServer

MissionEventPublisher.publish(event)
  -> RuntimeEventStreamServer
  -> mission_events.jsonl
```

`LatestWorldSnapshotSubscriber` is the in-process behavior handoff:

```text
WorldSnapshotPublisher
  -> LatestWorldSnapshotSubscriber
  -> LatestWorldSnapshot
  -> MissionRuntime
```

`ArtifactSnapshotWriter` owns evidence files:

```text
WorldSnapshotPublisher
  -> ArtifactSnapshotWriter
  -> snapshot_XXXX.json
  -> snapshot_manifest.txt
```

`RuntimeEventStreamServer` is an optional live TCP JSONL stream for external tools/customers:

```text
GhostDetectionsPublisher + WorldSnapshotPublisher + MissionEventPublisher
  -> RuntimeEventStreamServer
  -> tcp://127.0.0.1:<port>
```

Enable it with the current compatibility flag:

```bash
--world-snapshot-stream-port 47770
```

The stream emits records shaped like:

```json
{"type":"ghost_detections","seq":1,"timestamp_ns":123,"map_frame_id":"map_airsim_mission_0001","ghost_detections":{"detections":[]}}
```

```json
{"type":"world_snapshot","seq":2,"timestamp_ns":123,"active_map_frame_id":"map_airsim_mission_0001","snapshot":{}}
```

```json
{"type":"mission_event","seq":3,"timestamp_ns":124,"mission_event":{"event":"target_selected","source_track_id":"ghost_person_001"}}
```

Sequence numbers are stream-local and shared across event types. Consumers should dispatch by `type` and use `seq` to detect gaps.

The stream server is opt-in, non-blocking, and drops broken/slow clients rather than blocking the mission loop. It reports stats on shutdown:

```text
published_seq
accepted_clients
connected_clients
dropped_clients
```

### 2.4 AirSim visualization subscriber path

`simulation/airsim-world-overlay.py` is display-only:

```text
RuntimeEventStreamServer
  -> TCP JSONL stream
  -> simulation/airsim-world-overlay.py
       caches latest ghost_detections
       caches latest world_snapshot
       caches latest target_selected mission_event
       renders PLAN / PLAN* from ghost_detections
       renders AG from world_snapshot.agents
       renders EGO from world_snapshot.ego
       renders SEL by matching selected target against world_snapshot.agents
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

The overlay does not modify Dedalus perception, world model, target selection, or mission state.

It must not:

```text
- evaluate GhostScenario locally
- discover/list AirSim objects for ghost binding
- poll snapshot_manifest.txt in normal mode
- read mission_events.jsonl for normal SEL state
- own source modes such as combined/world_snapshot/artifact_snapshot
```

Current marker semantics:

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

### 2.5 Mission / behavior runtime path

The autonomy loop consumes the latest world snapshot and emits mission commands:

```text
LatestWorldSnapshot
  -> MissionRuntime async loop
  -> ObjectBehaviorMissionController
  -> TargetSelector
  -> behavior velocity
  -> FlightCommandSink
```

For the ghost follow run:

```text
WorldSnapshot.agents[]
  agent_id: agent_ghost_person_001
  source_track_id: ghost_person_001
  class: person
  confidence: 0.82

TargetSelector
  -> selected target

ObjectBehaviorMissionController
  -> target_selected
  -> behavior_start
  -> behavior_tick_sample
  -> bounded follow velocity
  -> behavior_complete
```

The behavior controller does not talk to AirSim directly. It only emits mission commands:

```text
Arm
Takeoff
Velocity
Land
Disarm
```

### 2.6 Command/control output path

Commands go back to PX4/AirSim through the PX4 bridge command sink:

```text
ObjectBehaviorMissionController
  -> VelocityCommand / lifecycle command
  -> Px4BridgeCommandSink
  -> simulation/px4-command-bridge.py
  -> PX4 shell / pymavlink
  -> PX4 / AirSim
```

Command helper OK is not vehicle truth. Vehicle truth comes back through ego/world telemetry. The mission does not enter `ExecuteMission` until ego height reaches the configured safe height.

### 2.7 Artifact / validation path

`ArtifactSnapshotWriter` writes durable snapshot artifacts:

```text
out/object_behavior_airsim_ghost/
  mission_events.jsonl
  snapshot_manifest.txt
  snapshot_0001.json
  snapshot_0002.json
  ...

out/object_behavior_airsim_ghost_annotation/
  frame_000001.ppm
  frame_000001.world_overlay.json
  ...
```

The validator reads those artifacts after the run:

```text
simulation/validate-object-behavior-airsim-ghost.py
  -> mission_events.jsonl
  -> snapshots
  -> annotation sidecars
```

This is the validation truth path. It is not a simulation input path.

---

## 3. Run with AirSim/PX4 already running

Clean generated artifacts:

```bash
rm -rf out/object_behavior_airsim_ghost out/object_behavior_airsim_ghost_annotation
```

Terminal 1, run mission-loop with runtime event stream enabled:

```bash
./build-staging/apps/dedalus_mission_loop \
  --config config/core_stack_object_behavior_airsim_ghost.yaml \
  --output-dir out/object_behavior_airsim_ghost \
  --max-frames 900 \
  --shutdown-max-frames 400 \
  --world-snapshot-stream-port 47770 \
  --progress
```

Terminal 2, visualize live runtime events in AirSim:

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

Optional terminal 3, inspect the live stream:

```bash
nc 127.0.0.1 47770 | head -5
```

Expected stream event types:

```text
ghost_detections
world_snapshot
mission_event
```

Expected live overlay markers:

```text
PLAN / PLAN*
AG
EGO
SEL
```

Expected mission events:

```text
target_selected
behavior_start
behavior_tick_sample
behavior_complete
command_dispatch Arm
command_dispatch Takeoff
command_dispatch Velocity during ExecuteMission
command_dispatch Land
command_dispatch Disarm
runtime_stop state=Complete terminal_settled=true
```

Expected selected target:

```text
source_track_id: ghost_person_001
agent_id: agent_ghost_person_001
class: person
```

The behavior should not switch to `ghost_person_002` merely because it has higher confidence.

---

## 4. AirSim existing-object discovery for 2.26E

Use the listing tool while AirSim is running:

```bash
python3 simulation/airsim-list-objects.py \
  --match-class person \
  --sort distance \
  --format table
```

```bash
python3 simulation/airsim-list-objects.py \
  --match-class animal \
  --sort distance \
  --format table
```

```bash
python3 simulation/airsim-list-objects.py \
  --only-matched \
  --output out/airsim_scene_objects.json
```

Pick explicit object names first, such as `BRPlayer_*`, and bind them deterministically in config. Do not start with random/all selection modes.

Planned first existing-object validation:

```text
Select one visible AirSim object by name.
Bind it to ghost_person_001 or ghost_car_001.
Publish ghost_detections from its AirSim pose.
Inject the same pose into perception/world-model.
Verify PLAN / AG / SEL align over the visible object.
```

---

## 5. Validate the run artifacts

```bash
python3 simulation/validate-object-behavior-airsim-ghost.py \
  out/object_behavior_airsim_ghost \
  --annotation-dir out/object_behavior_airsim_ghost_annotation
```

The validator checks:

```text
mission_events.jsonl exists and is valid JSONL
snapshot_manifest.txt exists and references snapshots
snapshots contain ghost_person_001, ghost_person_002, and ghost_car_001
mission_events contains target_selected / behavior_start / behavior_complete
selected target is ghost_person_001
Arm / Takeoff / Land / Disarm command dispatch/result events exist
runtime_stop is Complete and terminal_settled=true
annotation PPM frames exist
frame_XXXXXX.world_overlay.json sidecars exist
sidecars include ghost_person_001
```

---

## 6. Export MP4 review artifact

```bash
python3 scripts/export-ppm-sequence-to-mp4.py \
  --annotation-dir out/object_behavior_airsim_ghost_annotation \
  --output-mp4 out/object_behavior_airsim_ghost/world_overlay.mp4
```

The MP4 is a human review artifact. Mission events and snapshots remain the validation truth.

---

## 7. Test commands

Targeted pub/sub and runtime stream regression:

```bash
ctest --test-dir build-staging --output-on-failure -R 'pubsub|world_snapshot_publisher|world_snapshot_stream_server|mission_runtime'
```

Targeted ghost/object behavior regression:

```bash
ctest --test-dir build-staging --output-on-failure -R 'ghost_scenario|ghost_scenario_eval_cli|perception_world_model_flow|object_behavior_mission_smoke'
```

Full regression:

```bash
ctest --test-dir build-staging --output-on-failure
```

---

## 8. What this proves

The current trajectory-scenario run proves:

```text
Live AirSim frames can drive the normal CoreStackRunner path.
Ghost detections are authored as JSON objects with referenced VelocityTrajectory JSON paths.
GhostScenario can evaluate static and dynamic ghost detections deterministically.
CoreStackRunner publishes GhostDetectionsFrame from the same evaluation injected into perception.
Mission-loop ghost injection enters PerceptionPipelineOutput.observations and then WorldSnapshot.agents.
WorldSnapshots are published through a reusable typed pub/sub boundary.
Mission events are published through the runtime event stream and written to mission_events.jsonl.
Behavior consumes WorldSnapshot through LatestWorldSnapshotSubscriber.
Artifacts are written by ArtifactSnapshotWriter as evidence/debug outputs.
A live runtime event TCP JSONL stream can be enabled for external customers/tools.
AirSim overlay consumes live runtime events and renders PLAN / PLAN* / AG / EGO / SEL without artifact polling.
TargetSelector can select the requested lower-confidence ghost target by source_track_id.
ObjectBehaviorMissionController can trigger behavior events from WorldSnapshot state.
Follow behavior can emit bounded non-zero velocity.
Mission lifecycle still reaches GoHome / Land / Disarm / Complete.
```

The planned 2.26E existing-object run should additionally prove:

```text
Existing visible AirSim objects can drive GhostDetectionsFrame positions.
The same object pose feeds perception/world-model and overlay visualization.
PLAN / AG / SEL align over a real visible object in the AirSim environment.
```

---

## 9. What this does not prove yet

This does not yet prove:

```text
implemented AirSim existing-object ghost binding
circle / approach / sequence motion
real camera detector quality
real 3D perception quality
obstacle avoidance
custom AirSim mesh/proxy actors for people/cars
viewport overlay as validation truth
```

Those are subsequent slices.

---

## 10. Engineering cleanup rule

For future work on this path:

```text
Review the implementation and look for legacy pre-refactor leftovers to clean up.
Always strive for state-of-the-art clean code, carefully balancing code structure, runtime efficiency, and complexity.
Do not carry legacy code through shims or other inelegant solutions due to momentum.
When appropriate, suggest refactors to avoid drift and bloat, and keep code streamlined.
```

Specific to this runbook:

```text
Do not use artifact files as runtime IPC when the live stream or in-process subscriber boundary is the right abstraction.
Do not reintroduce direct snapshot file writing into dedalus_mission_loop; keep it behind ArtifactSnapshotWriter.
Do not bypass WorldSnapshotPublisher for behavior-facing snapshot delivery.
Do not make simulation/airsim-world-overlay.py evaluate GhostScenario, discover AirSim objects, or poll artifacts in normal mode.
Keep generic event machinery generic: EventPublisher<T> / EventSubscriber<T>.
Keep domain consumers readable: WorldSnapshotSubscriber::on_snapshot(...), GhostDetectionsSubscriber::on_ghost_detections(...), MissionEventSubscriber::on_mission_event(...), not generic on_event(...) in domain code.
