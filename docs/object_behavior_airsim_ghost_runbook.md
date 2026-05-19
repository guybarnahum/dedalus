# Object Behavior AirSim Ghost Runbook

This runbook validates object-conditioned behavior against live AirSim frames while still using ghost/scripted targets as deterministic target input.

The purpose is to prove this path:

```text
AirSim live frame + ego sidecar
  -> AirSimFrameSource
  -> ghost/scripted targets
  -> InMemoryWorldModel
  -> WorldSnapshot.agents
  -> TargetSelector
  -> ObjectBehaviorMissionController
  -> target_selected / behavior_start / behavior_tick_sample / behavior_complete
  -> Px4BridgeCommandSink
  -> PX4 / AirSim
  -> Dedalus annotated artifacts + sidecar JSON
```

As of 2.26A, `follow` emits bounded non-zero velocity during `ExecuteMission`. Circle/approach/sequence behavior math remains future work.

---

## 1. Runtime and Dataflow Boundaries

The AirSim ghost object-behavior run has three parallel paths. They intentionally do not depend on one another for correctness.

```text
1. Sensor/data input path
2. Command/control output path
3. Visualization/debug overlay path
```

### 1.1 Sensor / data input path

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
  -> WorldSnapshot
  -> LatestWorldSnapshot
```

The binary bridge stdout must remain protocol bytes only. Human diagnostics must go to stderr.

In the AirSim ghost config:

```yaml
bridge_mode: stream_binary_ego
bridge_transport: pipe
bridge_command: python3 simulation/airsim-stream-frames-binary.py --include-ego --rate-hz 5 --mavlink-armed-endpoints udpin:127.0.0.1:14540
```

Ghost/scripted targets enter before the world model, like real detections eventually will:

```text
GhostTargetProvider
  -> PerceptionPipelineOutput.observations
  -> InMemoryWorldModel
  -> WorldSnapshot.agents
```

This means behavior consumes normal `WorldSnapshot.agents`; it does not know or care whether the source was a real detector or a ghost/scripted validation target.

### 1.2 Mission / behavior runtime path

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

### 1.3 Command/control output path

Commands go back to PX4/AirSim through the PX4 bridge command sink:

```text
ObjectBehaviorMissionController
  -> VelocityCommand / lifecycle command
  -> Px4BridgeCommandSink
  -> simulation/px4-command-bridge.py
  -> PX4 shell / pymavlink
  -> PX4 / AirSim
```

The split remains:

```text
PX4 shell:
  commander arm
  commander takeoff
  commander land
  commander disarm

pymavlink:
  OFFBOARD setup
  SET_POSITION_TARGET_LOCAL_NED velocity commands
  LOCAL_POSITION_NED feedback for climb/safe-height behavior
```

Command helper OK is not vehicle truth. Vehicle truth comes back through ego/world telemetry. The mission does not enter `ExecuteMission` until ego height reaches the configured safe height.

### 1.4 Artifact / validation path

`dedalus_mission_loop` writes durable artifacts:

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

This is the validation truth path.

### 1.5 AirSim viewport/debug overlay path

`simulation/airsim-world-overlay.py` is not part of the autonomy loop. It is display-only.

It now supports three overlay sources:

```text
combined:
  Draw planned ghost-scenario markers immediately, then overlay world-model agents when snapshots exist.

world_snapshot:
  Draw only agents that Dedalus has actually written into WorldSnapshot artifacts.

ghost_scenario:
  Draw only planned ghost targets from simulation/ghost_targets/person_pair_crossing.yaml.
```

The visual semantics are:

```text
PLAN:
  faint planned ghost-scenario marker, read directly from simulation/ghost_targets/*.yaml.

AG:
  solid world-model agent marker, read from snapshot_XXXX.json.

SEL:
  selected target marker, read from mission_events.jsonl target_selected.
```

This makes the state explicit:

```text
planned ghost exists but no AG marker:
  AirSim/debug preview is working, but Dedalus has not yet put that ghost into WorldSnapshot.

AG marker exists but no SEL marker:
  Dedalus has the world-model agent, but TargetSelector has not selected it yet or mission_events are not available.

SEL marker exists:
  TargetSelector selected that WorldSnapshot agent and behavior can act on it.
```

Viewport overlay dataflow:

```text
simulation/ghost_targets/person_pair_crossing.yaml
  -> planned PLAN markers

snapshot_manifest.txt
  -> latest snapshot_XXXX.json
  -> WorldSnapshot.agents[]
  -> solid AG markers

mission_events.jsonl
  -> target_selected.source_track_id
  -> green SEL marker

AirSim debug draw API
  -> Unreal/AirSim viewport markers
```

It draws:

```text
position_local
  -> airsim.Vector3r(x, y, z - lift)
  -> client.simPlotPoints(...)
  -> client.simPlotStrings(...)
```

In `combined` mode, the overlay does not block planned ghost drawing while it waits for Dedalus snapshots. In `world_snapshot` mode, it waits for the required `WorldSnapshot` tracks because that mode explicitly means “show me what Dedalus knows.”

`Ctrl-C` exits either wait with no mission-state change.

Why this renders into the AirSim camera view:

```text
WorldSnapshot or ghost-scenario 3D position
  -> AirSim 3D debug marker
  -> Unreal renders marker in world
  -> AirSim camera sees marker if marker is inside the camera frustum
```

It does not paint camera pixels directly. Pixel-accurate reprojection validation remains in:

```text
frame_XXXXXX.world_overlay.json
  u_px
  v_px
  visible
  reason
```

Clean boundary:

```text
airsim-stream-frames-binary.py:
  AirSim/PX4 -> Dedalus sensor input

px4-command-bridge.py:
  Dedalus command output -> PX4/AirSim

airsim-world-overlay.py:
  Ghost fixture + Dedalus artifacts -> AirSim human debug display

validate-object-behavior-airsim-ghost.py:
  Dedalus artifacts -> validation result
```

---

## 2. AirSim ghost object-behavior config

The config is committed here:

```text
config/core_stack_object_behavior_airsim_ghost.yaml
```

The canonical behavior spec remains:

```text
config/behaviors/follow_specific_track.yaml
```

The sim-only ghost fixture remains:

```text
simulation/ghost_targets/person_pair_crossing.yaml
```

---

## 3. Run with AirSim/PX4 already running

```bash
rm -rf out/object_behavior_airsim_ghost out/object_behavior_airsim_ghost_annotation

./build-staging/apps/dedalus_mission_loop \
  --config config/core_stack_object_behavior_airsim_ghost.yaml \
  --output-dir out/object_behavior_airsim_ghost \
  --max-frames 900 \
  --shutdown-max-frames 400 \
  --progress
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

## 5. AirSim viewport overlay

The overlay can be started before or after the mission loop. In default `combined` mode, it immediately draws planned ghosts from the ghost fixture, then adds solid world-model agents as soon as Dedalus snapshots are available.

Terminal 1, mission loop:

```bash
rm -rf out/object_behavior_airsim_ghost out/object_behavior_airsim_ghost_annotation

./build-staging/apps/dedalus_mission_loop \
  --config config/core_stack_object_behavior_airsim_ghost.yaml \
  --output-dir out/object_behavior_airsim_ghost \
  --max-frames 900 \
  --shutdown-max-frames 400 \
  --progress
```

Terminal 2, viewport overlay:

```bash
python3 simulation/airsim-world-overlay.py \
  --snapshot-dir out/object_behavior_airsim_ghost \
  --source combined \
  --follow \
  --rate-hz 2 \
  --duration-s 180 \
  --clear \
  --label
```

Expected messages if started early:

```text
airsim-world-overlay: waiting for AirSim RPC at 127.0.0.1:41451 (...)
airsim-world-overlay: drawing planned ghosts; waiting for Dedalus snapshots under out/object_behavior_airsim_ghost
airsim-world-overlay: drawing planned ghosts; waiting for WorldSnapshot ghosts (...)
```

Expected ready message:

```text
airsim-world-overlay: WorldSnapshot ghost agents ready from snapshot_XXXX.json (...)
```

What should appear in AirSim/Unreal:

```text
faint PLAN markers:
  PLAN person ghost_person_001
  PLAN person ghost_person_002
  PLAN car ghost_car_001

solid AG markers once WorldSnapshot exists:
  AG person ghost_person_001
  AG person ghost_person_002
  AG car ghost_car_001

green SEL marker after selection:
  SEL person ghost_person_001
```

Draw only planned ghosts, independent of mission loop:

```bash
python3 simulation/airsim-world-overlay.py \
  --snapshot-dir out/object_behavior_airsim_ghost \
  --source ghost_scenario \
  --follow \
  --rate-hz 2 \
  --duration-s 180 \
  --clear \
  --label
```

Draw only what Dedalus has put into WorldSnapshot:

```bash
python3 simulation/airsim-world-overlay.py \
  --snapshot-dir out/object_behavior_airsim_ghost \
  --source world_snapshot \
  --follow \
  --rate-hz 2 \
  --duration-s 180 \
  --clear \
  --label
```

Dry-run without AirSim drawing:

```bash
python3 simulation/airsim-world-overlay.py \
  --snapshot-dir out/object_behavior_airsim_ghost \
  --source combined \
  --dry-run \
  --label
```

One-shot draw after a completed run:

```bash
python3 simulation/airsim-world-overlay.py \
  --snapshot-dir out/object_behavior_airsim_ghost \
  --source combined \
  --clear \
  --label \
  --persistent
```

Animate planned ghost positions using the fixture velocity fields:

```bash
python3 simulation/airsim-world-overlay.py \
  --snapshot-dir out/object_behavior_airsim_ghost \
  --source combined \
  --follow \
  --animate-planned \
  --rate-hz 2 \
  --duration-s 180 \
  --clear \
  --label
```

Clear persistent markers:

```bash
python3 - <<'PY'
import airsim
client = airsim.MultirotorClient(ip='127.0.0.1', port=41451)
client.confirmConnection()
client.simFlushPersistentMarkers()
PY
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

## 7. What this proves

This run proves:

```text
Live AirSim frames can drive the normal CoreStackRunner path.
Ghost targets can be injected into WorldSnapshot agents during a live AirSim run.
TargetSelector can select the requested lower-confidence ghost target by source_track_id.
ObjectBehaviorMissionController can trigger behavior events from WorldSnapshot state.
Follow behavior can emit bounded non-zero velocity.
Mission lifecycle still reaches GoHome / Land / Disarm / Complete.
Dedalus artifact overlays can show the ghost world-model agents on live AirSim camera frames.
AirSim viewport overlay can show planned ghosts, world-model agents, and selected target as distinct states without affecting autonomy.
```

---

## 8. What this does not prove yet

This does not yet prove:

```text
circle / approach / sequence motion
real camera detector quality
real 3D perception quality
obstacle avoidance
viewport overlay as validation truth
```

Those are subsequent slices.
