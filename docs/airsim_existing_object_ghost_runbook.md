# AirSim Existing-Object Ghost Validation Runbook

This runbook validates the 2.26E existing-object ghost source: a Dedalus ghost detection is bound to a real object already present in the AirSim scene.

The first canonical run config is:

```text
config/core_stack_object_behavior_airsim_existing_object.yaml
```

It binds:

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
python3 simulation/airsim-list-objects.py \
  --match-class person \
  --sort distance \
  --format table
```

Directly smoke the pose bridge for the selected object:

```bash
python3 simulation/airsim-object-poses.py \
  --object BRPlayer_01_96
```

Expected shape:

```json
{"schema_version":1,"source":"airsim_object_poses","timestamp_ns":...,"objects":[{"name":"BRPlayer_01_96","pose_available":true,"position_ned_m":[...],"orientation_quat_xyzw":[...]}]}
```

If the bridge fails, pick another visible object from the listing and update:

```text
config/core_stack_object_behavior_airsim_existing_object.yaml
```

---

## 2. Run mission-loop with runtime stream

Clean prior output:

```bash
rm -rf \
  out/object_behavior_airsim_existing_object \
  out/object_behavior_airsim_existing_object_annotation
```

Terminal 1:

```bash
./build-staging/apps/dedalus_mission_loop \
  --config config/core_stack_object_behavior_airsim_existing_object.yaml \
  --output-dir out/object_behavior_airsim_existing_object \
  --max-frames 900 \
  --shutdown-max-frames 400 \
  --world-snapshot-stream-port 47770 \
  --progress
```

Terminal 2:

```bash
python3 simulation/airsim-world-overlay.py \
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

## 3. Expected visual result

In the AirSim viewport, the selected object should show:

```text
PLAN or PLAN* marker:
  comes from ghost_detections, directly from AirSim simGetObjectPose(BRPlayer_01_96)

AG marker:
  comes from WorldSnapshot.agents after the same ghost detection is injected into perception/world-model

SEL marker:
  comes from mission_event target_selected matched against the latest world_snapshot agent
```

For the first existing-object run, all three should be near the visible AirSim object named in the config.

Important interpretation:

```text
PLAN can appear first.
AG appears after world-model ingestion.
SEL appears after behavior selects the target.
```

Small visual offsets are acceptable because markers are lifted above the object for readability. Large horizontal offsets mean a coordinate or source mismatch.

---

## 4. Artifact checks

The existing-object run should produce:

```text
out/object_behavior_airsim_existing_object/mission_events.jsonl
out/object_behavior_airsim_existing_object/snapshot_manifest.txt
out/object_behavior_airsim_existing_object/snapshot_*.json
out/object_behavior_airsim_existing_object_annotation/frame_*.ppm
out/object_behavior_airsim_existing_object_annotation/frame_*.world_overlay.json
out/object_behavior_airsim_existing_object/overlay_debug_latest.json
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

## 5. What this validates

This run validates:

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

---

## 6. What this does not validate yet

This does not yet validate:

```text
random/all AirSim object selection modes
moving existing AirSim scene actors
circle behavior around the object
camera-derived real detector quality
custom mesh/proxy actor injection
obstacle avoidance
```

Those are later slices.

---

## 7. Next step after success

After PLAN / AG / SEL align on the existing object, move to 2.27:

```text
Circle an existing visible static object.
Prefer a parked car or static prop if available.
Use BRPlayer_* only if no better static object is available.
```
