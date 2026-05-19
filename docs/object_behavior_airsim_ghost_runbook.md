# Object Behavior AirSim Ghost Runbook

This runbook validates the next object-conditioned behavior slice against live AirSim frames while still using ghost/scripted targets as deterministic target input.

The purpose is to prove this path:

```text
AirSim live frame + ego sidecar
  -> AirSimFrameSource
  -> ghost/scripted targets
  -> InMemoryWorldModel
  -> WorldSnapshot.agents
  -> TargetSelector
  -> ObjectBehaviorMissionController
  -> target_selected / behavior_start / behavior_complete
  -> Px4BridgeCommandSink
  -> PX4 / AirSim
  -> Dedalus annotated artifacts + sidecar JSON
```

This is not yet non-zero follow/circle motion. The object behavior still emits safe zero/hold velocity during ExecuteMission. Follow/circle/approach velocity math comes next.

---

## 1. Create the local AirSim ghost object-behavior config

Create this file locally:

```bash
cat > config/core_stack_object_behavior_airsim_ghost.yaml <<'EOF'
frame_source: airsim
ego_provider: frame_hint
detector: scripted
camera_stabilizer: null
tracker: simple_centroid
identity_resolver: appearance_only
projector: flat_ground
ghost_targets_enabled: true
ghost_targets_scenario: person_pair_crossing
world_model: in_memory
frame_annotator: ppm_sequence
annotation_output_path: out/object_behavior_airsim_ghost_annotation
annotation_output_fps: 5
fallback_map_frame_id: map_airsim_mission_0001

bridge_mode: stream_binary_ego
bridge_transport: pipe
bridge_command: python3 simulation/airsim-stream-frames-binary.py --include-ego --rate-hz 5 --mavlink-armed-endpoints udpin:127.0.0.1:14540
source_host: 127.0.0.1
source_rpc_port: 41451
vehicle_name: PX4
vehicle_camera_name: front_center

mission_controller: object_behavior
mission_tick_hz: 10
flight_command_sink: px4_bridge
mission_options.behavior_spec_path: config/behaviors/follow_specific_track.yaml
mission_options.object_behavior_hold_velocity_mps: 0.0
mission_options.object_behavior_completion_after_s: 8.0
mission_options.flight_control_mode: px4
mission_options.flight_prepare_session_command: python3 simulation/airsim-prepare-session.py --host 127.0.0.1 --rpc-port 41451 --vehicle-name PX4 --mavlink-endpoints udpin:127.0.0.1:14550,udpin:127.0.0.1:14540,udpin:127.0.0.1:14600
mission_options.flight_safe_height_m: 16
mission_options.flight_px4_command_bridge: python3 simulation/px4-command-bridge.py --mavlink-endpoints udpin:127.0.0.1:14550 --px4-tmux-target dedalus-sim:px4 --safe-height 8
mission_options.flight_home_policy: initial_ego_pose
mission_options.flight_arm_retry_interval_s: 4.0
mission_options.flight_arm_timeout_s: 30.0
mission_options.flight_arm_dispatch_fallback_s: 2.0
mission_options.flight_takeoff_retry_interval_s: 4.0
mission_options.flight_land_retry_interval_s: 4.0
mission_options.flight_land_timeout_s: 90.0
mission_options.flight_disarm_retry_interval_s: 4.0
mission_options.flight_disarm_timeout_s: 30.0
EOF
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

## 2. Run with AirSim/PX4 already running

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

## 3. Validate the run artifacts

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

## 4. Export MP4 review artifact

```bash
python3 scripts/export-ppm-sequence-to-mp4.py \
  --annotation-dir out/object_behavior_airsim_ghost_annotation \
  --output-mp4 out/object_behavior_airsim_ghost/world_overlay.mp4
```

The MP4 is a human review artifact. Mission events and snapshots remain the validation truth.

---

## 5. What this proves

This run proves:

```text
Live AirSim frames can drive the normal CoreStackRunner path.
Ghost targets can be injected into WorldSnapshot agents during a live AirSim run.
TargetSelector can select the requested lower-confidence ghost target by source_track_id.
ObjectBehaviorMissionController can trigger behavior events from WorldSnapshot state.
Mission lifecycle still reaches GoHome / Land / Disarm / Complete.
Dedalus artifact overlays can show the ghost world-model agents on live AirSim camera frames.
```

---

## 6. What this does not prove yet

This does not yet prove:

```text
non-zero follow velocity math
circle / approach / sequence motion
AirSim/Unreal viewport debug overlay
real camera detector quality
real 3D perception quality
obstacle avoidance
```

Those are subsequent slices.
