# AirSim Existing-Object Ghost Validation Runbook

This runbook validates Dedalus ghost detections bound to real objects already present in the AirSim scene. It covers the 2.26E follow/existing-object path and the 2.27A/2.27B circle validation path.

Canonical existing-object configs:

```text
config/ci/core_stack_object_behavior_airsim_existing_object.yaml
config/ci/core_stack_object_behavior_airsim_existing_object_circle.yml
```

Default binding:

```text
source_track_id: ghost_person_001
airsim_object_name: BRPlayer_01_96
class: person
```

`BRPlayer_01_96` was selected from an observed AirSim object listing:

```text
BRPlayer_01_96  position_ned_m ~= [-19.19, 29.99, -2.50]
```

If that object is not visible or not present in a given environment, discover another object and replace the binding in the config.

---

## 1. Discover and confirm the object

With AirSim running:

```bash
python3 simulation/airsim/scripts/airsim-list-objects.py \
  --match-class person \
  --sort distance \
  --format table
```

Directly smoke the pose bridge for the selected object:

```bash
python3 simulation/airsim/scripts/airsim-object-poses.py \
  --object BRPlayer_01_96
```

Expected shape:

```json
{"schema_version":1,"source":"airsim_object_poses","timestamp_ns":...,"objects":[{"name":"BRPlayer_01_96","pose_available":true,"position_ned_m":[...],"orientation_quat_xyzw":[...]}]}
```

If the bridge fails, pick another visible object from the listing and update the relevant core-stack config.

---

## 2. Follow/existing-object smoke run

Clean prior output:

```bash
rm -rf \
  out/object_behavior_airsim_existing_object \
  out/object_behavior_airsim_existing_object_annotation
```

Terminal 1:

```bash
./build-staging/apps/dedalus_mission_loop \
  --config config/ci/core_stack_object_behavior_airsim_existing_object.yaml \
  --output-dir out/object_behavior_airsim_existing_object \
  --max-frames 900 \
  --shutdown-max-frames 400 \
  --world-snapshot-stream-port 47770 \
  --progress
```

Terminal 2:

```bash
python3 simulation/airsim/scripts/airsim-world-overlay.py \
  --stream-port 47770 \
  --follow \
  --rate-hz 5 \
  --duration-s 180 \
  --clear \
  --label \
  --debug \
  --debug-json out/object_behavior_airsim_existing_object/overlay_debug_latest.json
```

Optional terminal 3:

```bash
nc 127.0.0.1 47770 | head -10
```

Expected stream event types:

```text
ghost_detections
world_snapshot
mission_event
```

---

## 3. Circle/orbit validation run

Circle behavior uses:

```text
config/ci/core_stack_object_behavior_airsim_existing_object_circle.yml
config/behaviors/circle_existing_object_person.yaml
```

For a three-orbit validation, make sure the behavior spec contains:

```yaml
behavior:
  type: circle
  radius_m: 10.0
  orbit_count: 3.0

completion:
  after_s: 360
  then: go_home_land
```

and the core-stack config has a long enough override:

```yaml
mission_options.object_behavior_completion_after_s: 360.0
```

Clean prior output:

```bash
rm -rf \
  out/object_behavior_airsim_existing_object_circle \
  out/object_behavior_airsim_existing_object_circle_annotation
```

Terminal 1:

```bash
./build-staging/apps/dedalus_mission_loop \
  --config config/ci/core_stack_object_behavior_airsim_existing_object_circle.yml \
  --output-dir out/object_behavior_airsim_existing_object_circle \
  --max-frames 5400 \
  --shutdown-max-frames 1800 \
  --world-snapshot-stream-port 47770 \
  --safe-height 40 \
  --behavior-duration-s 360 \
  --progress
```

Terminal 2:

```bash
python3 simulation/airsim/scripts/airsim-world-overlay.py \
  --stream-port 47770 \
  --follow \
  --rate-hz 5 \
  --duration-s 240 \
  --clear \
  --label \
  --osd \
  --debug \
  --debug-json out/object_behavior_airsim_existing_object_circle/overlay_debug_latest.json
```

Expected behavior event:

```bash
grep '"behavior_complete"' out/object_behavior_airsim_existing_object_circle/mission_events.jsonl
```

Expected completion reason:

```text
orbit_count_elapsed
```

This proves the run completed because the controller counted the requested orbits, not because the fallback duration expired.

---

## 4. Circle trajectory validator

2.27B adds an artifact-only validator for circle/orbit quality:

```bash
python3 tools/validation/validate-circle-trajectory.py \
  --events out/object_behavior_airsim_existing_object_circle/mission_events.jsonl \
  --min-orbits 3.0 \
  --radius 10.0 \
  --avg-radius-error-max 1.0 \
  --max-radius-error-after-latch 3.0 \
  --expect-complete-reason orbit_count_elapsed \
  --require-terminal-settled \
  --require-lifecycle
```

The validator reads `mission_events.jsonl` only. It does not connect to AirSim, PX4, the runtime stream, or the overlay. It checks durable artifacts for:

```text
target_selected
Mission / circling samples
orbit_mode_latched=true
completed orbit count
behavior_complete reason
radius error statistics
GoHome -> Land -> Complete lifecycle evidence
runtime_stop terminal_settled=true
```

Useful quick greps:

```bash
grep '"display_detail":"circling"' out/object_behavior_airsim_existing_object_circle/mission_events.jsonl | head
grep '"orbit_mode_latched":true' out/object_behavior_airsim_existing_object_circle/mission_events.jsonl | head
grep '"circle_completed_orbits"' out/object_behavior_airsim_existing_object_circle/mission_events.jsonl | tail -40
```

---

## 5. Expected visual result

In the AirSim viewport, the selected object should show:

```text
PLAN or PLAN* marker:
  comes from ghost_detections, directly from AirSim simGetObjectPose(BRPlayer_01_96)

AG marker:
  comes from WorldSnapshot.agents after the same ghost detection is injected into perception/world-model

SEL marker:
  comes from mission_event target_selected matched against the latest world_snapshot agent
```

For the existing-object run, all three should be near the visible AirSim object named in the config.

Important interpretation:

```text
PLAN can appear first.
AG appears after world-model ingestion.
SEL appears after behavior selects the target.
```

Small visual offsets are acceptable because markers are lifted above the object for readability. Large horizontal offsets mean a coordinate or source mismatch.

For circle behavior, the AirSim viewport or follow camera is an operator review aid only. The authoritative circle validation is `mission_events.jsonl` plus `tools/validation/validate-circle-trajectory.py`.

---

## 6. Artifact checks

The existing-object runs should produce:

```text
out/object_behavior_airsim_existing_object*/mission_events.jsonl
out/object_behavior_airsim_existing_object*/snapshot_manifest.txt
out/object_behavior_airsim_existing_object*/snapshot_*.json
out/object_behavior_airsim_existing_object*_annotation/frame_*.ppm
out/object_behavior_airsim_existing_object*_annotation/frame_*.world_overlay.json
out/object_behavior_airsim_existing_object*/overlay_debug_latest.json
```

Quick checks:

```bash
grep 'target_selected' out/object_behavior_airsim_existing_object/mission_events.jsonl
grep 'ghost_person_001' out/object_behavior_airsim_existing_object/mission_events.jsonl
cat out/object_behavior_airsim_existing_object/overlay_debug_latest.json
```

The overlay debug JSON should include a track entry for:

```text
ghost_person_001
```

and a small `delta_plan_minus_world_norm_m` when both PLAN and AG are available.

---

## 7. What this validates

The follow/existing-object run validates:

```text
AirSim object pose bridge can read a selected scene object.
GhostTargetProvider can use airsim_objects as a source.
The same AirSim object pose feeds GhostDetectionsFrame and Observation3D.
WorldSnapshot.agents contains ghost_person_001 from the AirSim object pose.
TargetSelector can select ghost_person_001.
MissionEventPublisher emits target_selected.
Runtime stream carries ghost_detections, world_snapshot, and mission_event.
AirSim overlay renders PLAN / AG / SEL over the visible object.
```

The circle/existing-object run additionally validates:

```text
ObjectBehaviorMissionController executes continuous orbit-capture circle behavior.
Circle uses tangent velocity at current radial angle plus radial correction.
Orbit mode latches after capture.
Completed orbit count advances while circling.
Behavior completes with reason=orbit_count_elapsed for orbit-count runs.
Mission proceeds through GoHome -> Land -> Disarm -> Settled.
```

---

## 8. What this does not validate yet

This does not yet validate:

```text
random/all AirSim object selection modes
moving existing AirSim scene actors
camera-derived real detector quality
custom mesh/proxy actor injection
obstacle avoidance
```

Those are later slices.

---

## 9. Next step after success

After a three-orbit circle run passes `validate-circle-trajectory.py`, 2.27A/B circle behavior should be considered validated for the static existing-object path.

Likely next behavior slice:

```text
Approach behavior, or sequence behavior: approach -> circle -> go_home_land.
```
