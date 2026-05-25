# Camera Pointing Intent

## Purpose

Camera/gimbal pointing is a separate actuator path from vehicle velocity/yaw.

Dedalus treats target-stare as two coordinated but distinct policies:

```text
horizontal target stare -> vehicle yaw intent -> PX4 velocity/yaw command path
vertical target stare   -> camera_pointing_intent -> camera/gimbal sink path
```

This separation is intentional. PX4 vehicle yaw changes the aircraft heading. Camera/gimbal pitch changes the optical axis without changing translational flight behavior.

## Current policy owner

`ObjectBehaviorMissionController` owns the target-stare pitch policy. During `ExecuteMission`, when vertical stare is configured, it emits a `camera_pointing_intent` mission event computed from the selected target and ego pose.

The intent is emitted on the same runtime JSONL stream as other mission events:

```text
dedalus_mission_loop --world-snapshot-stream-port 47770
  -> mission_event camera_pointing_intent
```

## Intent schema

Default lifecycle policy:

```text
Prepare        -> mode=neutral, reset pitch to 0 before arming / ready state
Takeoff        -> mode=neutral, keep pitch at 0 during climb
ExecuteMission -> mode=target, point at selected target
GoHome         -> mode=home, point at home / recovery location
Land           -> mode=landing_area, continue looking at the landing location
Complete       -> mode=neutral, reset pitch to 0 while disarming / after landing
```

This starts every mission from a known optical-axis state, keeps the camera
useful during recovery and landing, then avoids leaving the gimbal tilted down
after mission completion.

Current event fields:

```json
{
  "event": "camera_pointing_intent",
  "camera_pointing_mode": "target",
  "vertical_stare_mode": "gimbal",
  "cameras": ["front_center", "0"],
  "agent_id": "agent_ghost_person_001",
  "source_track_id": "ghost_person_001",
  "identity_id": "identity_ghost_person_001",
  "target_elevation_rad": 0.42,
  "target_elevation_deg": 24.0,
  "pitch_rad": -0.42,
  "pitch_deg": -24.0,
  "pitch_unclamped_rad": -0.42,
  "pitch_min_rad": -1.396263,
  "pitch_max_rad": 0.610865,
  "pitch_sign": -1.0,
  "pitch_offset_rad": 0.0,
  "pitch_clamped": false,
  "range_xy_m": 12.3,
  "delta_z_m": 5.5,
  "pitch_valid": true,
  "display_state": "Mission",
  "display_detail": "camera_pointing"
}
```

The sign and clamp fields are part of the policy/configuration layer. A sink should normally apply `pitch_rad` directly rather than recomputing target geometry.

## AirSim sink

The AirSim sink is:

```text
simulation/airsim/scripts/airsim-camera-pointing-bridge.py
```

It subscribes to `camera_pointing_intent`, then calls AirSim `simSetCameraPose` for every camera in the intent.

Validated AirSim mapping in the current setup:

```text
front_center -> Dedalus capture/perception camera
0            -> AirSim operator FPV/F view camera candidate
```

The bridge can verify acceptance with `simGetCameraInfo` and can save proof frames per camera.

Validation command:

```bash
python3 simulation/airsim/scripts/airsim-camera-pointing-bridge.py \
  --stream-port 47770 \
  --host 127.0.0.1 \
  --rpc-port 41451 \
  --vehicle-name PX4 \
  --cameras front_center \
  --cameras 0 \
  --rate-hz 10 \
  --resend-s 0.25 \
  --verify-pose \
  --capture-dir out/object_behavior_airsim_existing_object_circle/camera_pointing_frames \
  --capture-every-s 1.0 \
  --debug \
  --debug-json out/object_behavior_airsim_existing_object_circle/camera_pointing_latest.json
```

Expected artifacts:

```text
camera_pointing_latest.json:
  cameras: ["front_center", "0"]
  per_camera_verify.front_center.accepted_pitch_deg
  per_camera_verify.0.accepted_pitch_deg

camera_pointing_frames:
  camera_pointing_00042_front_center_-074.95.png
  camera_pointing_00042_0_-074.95.png
```

## MAVLink / real hardware sink

The real-hardware sink is:

```text
tools/px4/mavlink-gimbal-pointing-bridge.py
```

It subscribes to the same `camera_pointing_intent` stream and sends the pitch through MAVLink Gimbal Protocol v2.

Dedalus should talk to the MAVLink Gimbal Manager rather than directly to a Gimbal Device. The manager is the application-facing coordination layer for ownership and control arbitration.

Supported transport modes:

```text
message -> GIMBAL_MANAGER_SET_PITCHYAW
command -> MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW
```

Use `message` mode for streamed setpoints and `command` mode for low-rate command/ACK diagnostics.

Dry-run validation:

```bash
python3 tools/px4/mavlink-gimbal-pointing-bridge.py \
  --stream-port 47770 \
  --dry-run \
  --debug \
  --debug-json out/object_behavior_airsim_existing_object_circle/mavlink_gimbal_latest.json
```

Hardware-oriented command-mode probe:

```bash
python3 tools/px4/mavlink-gimbal-pointing-bridge.py \
  --stream-port 47770 \
  --mavlink-endpoints udpin:127.0.0.1:14550 \
  --mode command \
  --request-info \
  --configure-primary \
  --release-on-exit \
  --debug \
  --debug-json out/object_behavior_airsim_existing_object_circle/mavlink_gimbal_latest.json
```

Hardware-oriented streamed mode:

```bash
python3 tools/px4/mavlink-gimbal-pointing-bridge.py \
  --stream-port 47770 \
  --mavlink-endpoints udpin:127.0.0.1:14550 \
  --mode message \
  --configure-primary \
  --release-on-exit \
  --rate-hz 10 \
  --resend-s 0.25 \
  --debug
```

## Configuration

Example behavior config fields:

```yaml
mission_options.object_behavior_vertical_stare_mode: gimbal
mission_options.object_behavior_camera_pointing_cameras: front_center,0
mission_options.object_behavior_camera_pitch_min_deg: -80
mission_options.object_behavior_camera_pitch_max_deg: 35
mission_options.object_behavior_camera_pitch_sign: -1
mission_options.object_behavior_camera_pitch_offset_deg: 0
mission_options.object_behavior_camera_pointing_prepare_mode: neutral
mission_options.object_behavior_camera_pointing_takeoff_mode: neutral
mission_options.object_behavior_camera_pointing_go_home_mode: home
mission_options.object_behavior_camera_pointing_land_mode: landing_area
mission_options.object_behavior_camera_pointing_complete_mode: neutral
```

Future config fields should make the sink explicit:

```yaml
mission_options.object_behavior_camera_pointing_sink: runtime_stream
mission_options.object_behavior_camera_pointing_hardware_sink: mavlink_gimbal
```

## When to move transport to pure C++

The target-stare policy is already C++.

The remaining Python code is transport adaptation:

```text
AirSim bridge  -> AirSim RPC simSetCameraPose
MAVLink bridge -> pymavlink Gimbal Manager messages/commands
```

Do not move this transport layer into C++ merely because vehicle yaw is already C++. Vehicle yaw is part of the existing flight command path. Camera pitch is a separate actuator path.

Move camera/gimbal transport into C++ when at least one of these is true:

```text
1. CameraPointingIntent becomes a typed field in MissionTickOutput rather than a mission-event JSON payload.
2. A formal CameraPointingSink interface exists with Null, AirSim, and MAVLink-gimbal implementations.
3. We need lower latency or stronger process-lifetime guarantees than the bridge process can provide.
4. We intentionally add a native C++ AirSim RPC dependency or MAVLink gimbal client.
5. Real hardware integration requires native ownership/lifecycle management inside the runtime.
```

Until then, keep the bridges as target-specific sinks. They are isolated, testable, and do not duplicate behavior policy.

## Non-goals

```text
No vehicle yaw rewrite.
No PX4 velocity/offboard rewrite.
No direct Gimbal Device control unless a specific hardware integration requires it.
No overlay-side behavior inference.
No obstacle avoidance.
```
