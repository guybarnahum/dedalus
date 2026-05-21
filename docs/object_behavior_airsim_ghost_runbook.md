# Object Behavior AirSim Ghost Runbook

This runbook validates object-conditioned behavior against live AirSim frames while using a deterministic ghost-detection scenario as the object source.

As of 2.26C, ghost simulation is no longer hardcoded and no longer depends on mission artifacts for visualization. The canonical source is:

```text
simulation/ghost_detections/person_pair_crossing.json
  -> references simulation/trajectories/*.json
  -> loaded by GhostScenario
  -> evaluated by VelocityTrajectory-backed motion
```

As of 2.26D, WorldSnapshot delivery is no longer a file-IO hack inside `dedalus_mission_loop`. The runtime publishes snapshots through a reusable typed pub/sub service:

```text
CoreStackRunner
  -> InMemoryWorldModel
  -> WorldSnapshotPublisher
       -> LatestWorldSnapshotSubscriber       behavior / MissionRuntime
       -> ArtifactSnapshotWriter              durable evidence/debug files
       -> optional WorldSnapshotStreamServer  TCP JSONL live stream
```

Artifacts remain important, but they are evidence/debug outputs, not runtime IPC.

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

position_local_m from GhostScenario.evaluate(t):
  same map-local / AirSim-local NED frame.

WorldSnapshot.agents[].position_local:
  same map-local frame after mission-loop injection.

position_ego_relative:
  derived view of object relative to current ego, not the canonical ghost placement.
```

---

## 2. Shared evaluator CLI

The evaluator CLI validates scenario authoring and feeds Python visualization without duplicating trajectory parsing logic:

```bash
./build-staging/apps/dedalus_ghost_scenario_eval \
  --scenario simulation/ghost_detections/person_pair_crossing.json \
  --time-s 10
```

Expected positions at `time-s 10`:

```text
ghost_person_001:
  start x=12.0 + 0.3*10 = 15.0

ghost_person_002:
  start x=8.0 - 0.2*10 = 6.0

ghost_car_001:
  static x=4.0
```

At `time-s 50`, both people have completed one cross/wait/back/wait cycle and are back near their start positions.

The CLI is covered by CTest:

```bash
ctest --test-dir build-staging --output-on-failure -R ghost_scenario_eval_cli
```

---

## 3. Runtime and dataflow boundaries

The AirSim ghost object-behavior run has five separate paths:

```text
1. Sensor/data input path
2. Ghost simulation path
3. WorldSnapshot publish/subscribe path
4. Command/control output path
5. Artifact/validation path
```

### 3.1 Sensor / data input path

Dedalus receives live camera frames and ego telemetry from AirSim/PX4 through a binary frame bridge:

```text
AirSim / PX4
  -> simulation/airsim-stream-frames-binary.py
  -> stdout binary frame protocol
  -> AirSimFrameSource
  -> FrameHintEgoProvider
  -> CoreStackRunner
  -> PerceptionPipelineOutput
  -> InMemoryWorldModel
  -> WorldSnapshotPublisher
```

The binary bridge stdout must remain protocol bytes only. Human diagnostics must go to stderr.

In the AirSim ghost config:

```yaml
bridge_mode: stream_binary_ego
bridge_transport: pipe
bridge_command: python3 simulation/airsim-stream-frames-binary.py --include-ego --rate-hz 5 --mavlink-armed-endpoints udpin:127.0.0.1:14540
```

### 3.2 Ghost simulation path

Mission-loop ghost injection is configured with:

```yaml
ghost_targets_enabled: true
ghost_targets_scenario_path: simulation/ghost_detections/person_pair_crossing.json
```

Runtime path:

```text
CoreStackRunner
  -> first valid frame establishes ghost_scenario_start
  -> GhostTargetProvider evaluates GhostScenario at frame elapsed time
  -> appends ghost observations to PerceptionPipelineOutput.observations
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

That delay is expected. It is not caused by AirSim visualization and does not depend on artifacts.

### 3.3 WorldSnapshot publish / subscribe path

Current in-process and external consumers receive snapshots through one publisher boundary:

```text
WorldSnapshotPublisher.publish(snapshot)
  -> LatestWorldSnapshotSubscriber
  -> ArtifactSnapshotWriter
  -> optional WorldSnapshotStreamServer
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

`WorldSnapshotStreamServer` is an optional live TCP JSONL stream for external customers/tools:

```text
WorldSnapshotPublisher
  -> WorldSnapshotStreamServer
  -> tcp://127.0.0.1:<port>
```

Enable it with:

```bash
--world-snapshot-stream-port 47770
```

The stream emits lines shaped like:

```json
{"type":"world_snapshot","seq":1,"timestamp_ns":123,"active_map_frame_id":"map_airsim_mission_0001","snapshot":{}}
```

The stream server is opt-in, non-blocking, and drops broken/slow clients rather than blocking the mission loop. It reports stats on shutdown:

```text
published_seq
accepted_clients
connected_clients
dropped_clients
```

### 3.4 AirSim visualization path

`simulation/airsim-world-overlay.py` is display-only. Its default mode is artifact-free for planned ghost visualization:

```bash
python3 simulation/airsim-world-overlay.py \
  --ghost-scenario simulation/ghost_detections/person_pair_crossing.json \
  --ghost-evaluator ./build-staging/apps/dedalus_ghost_scenario_eval \
  --source ghost_scenario \
  --follow \
  --rate-hz 2 \
  --duration-s 180 \
  --clear \
  --label
```

This path is:

```text
ghost detection JSON
  -> dedalus_ghost_scenario_eval
  -> simulation/airsim-world-overlay.py
  -> AirSim simPlotPoints / simPlotStrings
```

Static detections:

```text
trajectory_path: null
  -> drawn persistently once by default
```

Dynamic detections:

```text
trajectory_path present / non-zero evaluated velocity
  -> redrawn while --follow is active
```

The overlay does not modify Dedalus perception, world model, target selection, or mission state.

Current limitation:

```text
AG/EGO/SEL marker rendering in simulation/airsim-world-overlay.py still has artifact-comparison modes.
The next stage should make AG/EGO consume the live WorldSnapshot stream instead of snapshot_manifest.txt.
SEL should later consume a live mission-event stream or selected-target state instead of mission_events.jsonl.
```

### 3.5 Mission / behavior runtime path

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

### 3.6 Command/control output path

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

### 3.7 Artifact / validation path

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

## 4. Run with AirSim/PX4 already running

Clean generated artifacts:

```bash
rm -rf out/object_behavior_airsim_ghost out/object_behavior_airsim_ghost_annotation
```

Terminal 1, visualize planned ghost scenario in AirSim:

```bash
python3 simulation/airsim-world-overlay.py \
  --ghost-scenario simulation/ghost_detections/person_pair_crossing.json \
  --ghost-evaluator ./build-staging/apps/dedalus_ghost_scenario_eval \
  --source ghost_scenario \
  --follow \
  --rate-hz 2 \
  --duration-s 180 \
  --clear \
  --label
```

Terminal 2, run mission-loop with live WorldSnapshot stream enabled:

```bash
./build-staging/apps/dedalus_mission_loop \
  --config config/core_stack_object_behavior_airsim_ghost.yaml \
  --output-dir out/object_behavior_airsim_ghost \
  --max-frames 900 \
  --shutdown-max-frames 400 \
  --world-snapshot-stream-port 47770 \
  --progress
```

Optional terminal 3, inspect the live stream:

```bash
nc 127.0.0.1 47770 | head -5
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

## 5. Optional artifact comparison overlay

Normal planned-ghost visualization does not read artifacts. Until the overlay consumes the live WorldSnapshot stream, `combined` and `world_snapshot` modes remain debug/fallback artifact comparison modes.

Combined comparison:

```bash
python3 simulation/airsim-world-overlay.py \
  --ghost-scenario simulation/ghost_detections/person_pair_crossing.json \
  --ghost-evaluator ./build-staging/apps/dedalus_ghost_scenario_eval \
  --snapshot-dir out/object_behavior_airsim_ghost \
  --source combined \
  --follow \
  --rate-hz 2 \
  --duration-s 180 \
  --clear \
  --label \
  --debug
```

WorldSnapshot-only comparison:

```bash
python3 simulation/airsim-world-overlay.py \
  --snapshot-dir out/object_behavior_airsim_ghost \
  --source world_snapshot \
  --follow \
  --rate-hz 2 \
  --duration-s 180 \
  --clear \
  --label \
  --debug
```

Visual semantics in comparison modes:

```text
PLAN:
  planned/evaluated GhostScenario state.

PLAN*:
  static planned/evaluated GhostScenario state, drawn persistently by default.

AG:
  world-model agent from snapshot artifacts today; should move to live WorldSnapshot stream next.

EGO:
  ego pose from snapshot artifacts today; should move to live WorldSnapshot stream next.

SEL:
  selected target from mission_events.jsonl target_selected today; should move to live mission-event stream next.
```

---

## 6. Validate the run artifacts

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

## 7. Export MP4 review artifact

```bash
python3 scripts/export-ppm-sequence-to-mp4.py \
  --annotation-dir out/object_behavior_airsim_ghost_annotation \
  --output-mp4 out/object_behavior_airsim_ghost/world_overlay.mp4
```

The MP4 is a human review artifact. Mission events and snapshots remain the validation truth.

---

## 8. Test commands

Targeted pub/sub and stream regression:

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

## 9. What this proves

This run proves:

```text
Live AirSim frames can drive the normal CoreStackRunner path.
Ghost detections are authored as JSON objects with referenced VelocityTrajectory JSON paths.
GhostScenario can evaluate static and dynamic ghost detections deterministically.
AirSim planned-ghost visualization can consume the shared evaluator without reading mission artifacts.
Mission-loop ghost injection can consume the same scenario/evaluator path into WorldSnapshot agents.
WorldSnapshots are published through a reusable typed pub/sub boundary.
Behavior consumes WorldSnapshot through LatestWorldSnapshotSubscriber.
Artifacts are written by ArtifactSnapshotWriter as evidence/debug outputs.
A live WorldSnapshot TCP JSONL stream can be enabled for external customers/tools.
TargetSelector can select the requested lower-confidence ghost target by source_track_id.
ObjectBehaviorMissionController can trigger behavior events from WorldSnapshot state.
Follow behavior can emit bounded non-zero velocity.
Mission lifecycle still reaches GoHome / Land / Disarm / Complete.
```

---

## 10. What this does not prove yet

This does not yet prove:

```text
AirSim AG/EGO markers consuming the live WorldSnapshot stream.
SEL marker consuming a live mission-event stream.
circle / approach / sequence motion
real camera detector quality
real 3D perception quality
obstacle avoidance
AirSim mesh/proxy actors for people/cars
viewport overlay as validation truth
```

Those are subsequent slices.

---

## 11. Engineering cleanup rule

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
Keep generic event machinery generic: EventPublisher<T> / EventSubscriber<T>.
Keep domain consumers readable: WorldSnapshotSubscriber::on_snapshot(...), not generic on_event(...) in world-model code.
```