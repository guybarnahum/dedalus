# Object Behavior AirSim Ghost Runbook

This runbook validates object-conditioned behavior against live AirSim frames while using a deterministic ghost-detection scenario as the object source.

Canonical runtime diagrams live in:

```text
docs/runtime_dataflow.md
```

As of 2.26D.6, `simulation/airsim-world-overlay.py` is a stream-only subscriber/renderer. It does not evaluate ghost scenarios locally and does not read `snapshot_manifest.txt` in normal operation.

---

## 1. Scenario and trajectory inputs

The ghost detection scenario describes objects, not motion internals:

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

Coordinate contract:

```text
initial_position_local_m:
  map-local / AirSim-local NED coordinates, not ego-relative.

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

Mission-loop ghost injection is configured with:

```yaml
ghost_targets_enabled: true
ghost_targets_scenario_path: simulation/ghost_detections/person_pair_crossing.json
```

Runtime path:

```text
CoreStackRunner
  -> first valid frame establishes ghost_scenario_start
  -> GhostTargetProvider::frame_at(timestamp, map_frame, scenario_start)
       evaluates GhostScenario exactly once for that frame time
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

### 2.3 Runtime publish / subscribe path

Current in-process and external consumers receive typed events through publisher boundaries:

```text
GhostDetectionsPublisher.publish(frame)
  -> RuntimeEventStreamServer

WorldSnapshotPublisher.publish(snapshot)
  -> LatestWorldSnapshotSubscriber
  -> ArtifactSnapshotWriter
  -> RuntimeEventStreamServer
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
GhostDetectionsPublisher + WorldSnapshotPublisher
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
       renders PLAN / PLAN* from ghost_detections
       renders AG from world_snapshot.agents
       renders EGO from world_snapshot.ego
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
  not live yet. Future slice should stream mission events or selected-target state.
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

## 4. Validate the run artifacts

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

## 5. Export MP4 review artifact

```bash
python3 scripts/export-ppm-sequence-to-mp4.py \
  --annotation-dir out/object_behavior_airsim_ghost_annotation \
  --output-mp4 out/object_behavior_airsim_ghost/world_overlay.mp4
```

The MP4 is a human review artifact. Mission events and snapshots remain the validation truth.

---

## 6. Test commands

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

## 7. What this proves

This run proves:

```text
Live AirSim frames can drive the normal CoreStackRunner path.
Ghost detections are authored as JSON objects with referenced VelocityTrajectory JSON paths.
GhostScenario can evaluate static and dynamic ghost detections deterministically.
CoreStackRunner publishes GhostDetectionsFrame from the same evaluation injected into perception.
Mission-loop ghost injection enters PerceptionPipelineOutput.observations and then WorldSnapshot.agents.
WorldSnapshots are published through a reusable typed pub/sub boundary.
Behavior consumes WorldSnapshot through LatestWorldSnapshotSubscriber.
Artifacts are written by ArtifactSnapshotWriter as evidence/debug outputs.
A live runtime event TCP JSONL stream can be enabled for external customers/tools.
AirSim overlay consumes live runtime events and renders PLAN / PLAN* / AG / EGO without artifact polling.
TargetSelector can select the requested lower-confidence ghost target by source_track_id.
ObjectBehaviorMissionController can trigger behavior events from WorldSnapshot state.
Follow behavior can emit bounded non-zero velocity.
Mission lifecycle still reaches GoHome / Land / Disarm / Complete.
```

---

## 8. What this does not prove yet

This does not yet prove:

```text
SEL marker consuming a live mission-event stream or selected-target state.
circle / approach / sequence motion
real camera detector quality
real 3D perception quality
obstacle avoidance
AirSim mesh/proxy actors for people/cars
viewport overlay as validation truth
```

Those are subsequent slices.

---

## 9. Engineering cleanup rule

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
Do not make simulation/airsim-world-overlay.py evaluate GhostScenario or poll artifacts in normal mode.
Keep generic event machinery generic: EventPublisher<T> / EventSubscriber<T>.
Keep domain consumers readable: WorldSnapshotSubscriber::on_snapshot(...), GhostDetectionsSubscriber::on_ghost_detections(...), not generic on_event(...) in domain code.
```