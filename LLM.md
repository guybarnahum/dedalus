# Dedalus LLM Operating Brief

This file is the active orientation document for a new LLM session. Keep it short, current, and action-oriented. Historical notes and superseded debugging context live in `LLM.back.md`.

Repository:

```text
guybarnahum/dedalus
```

Current handoff state:

```text
2.26E is complete and live-validated by the operator.
2.27A is complete and live-validated by the operator.
2.27B is complete and locally/live validated by build/tests plus the circle trajectory validator.
2.28B is complete: no-shim repo layout migration is validated.
2.28B.1 is complete: generated third-party dependencies are staged under third_party/ and validated from empty-state setup.
2.28C is complete: camera/gimbal target-stare policy, runtime dispatch, hardware/simulation sinks, and one-command AirSim mission workflow are validated.
2.29A-E are complete: multi-stage behavior sequence parsing/runtime, AirSim sequence configs, far-person SEL run, active step observability, canonical per-step yaw/camera mode validation, and mixed-mode yaw/camera validation are complete.
2.30A-B are complete: slow moving far-animal SEL validation and moving-target stress matrix are validated. See docs/selected_entity_slow_moving_animal_validation.md and docs/moving_target_behavior_validation_results.md.

4.1C AirSim depth obstacle detector dataflow is live-validated: AirSim DepthPlanar now reaches FramePacket.depth_frame, CoreStackRunner, WorldSnapshot.obstacle_evidence, and the AirSim OSD as classless `airsim_depth_obstacle_detector` volumetric evidence. See docs/airsim_depth_obstacle_detector_validation.md.

5Q-5U obstacle-memory default path is compact-delta-first and SQLite-backed. Current main is operator-validated after the refactor/CI cleanup stack: tests pass and `simulation/airsim/run_mission.sh` invocation works. DRY_RUN was not attempted and should not be assumed available.

GitHub workflow policy: CI and staging run the fast CTest subset (`-LE 'synthetic|scenario'`) plus core-stack smoke validation. Production keeps full CTest as the release gate plus core-stack smoke validation. See docs/ctest_validation_layers.md.

Active next work: continue mechanical cleanup/refactors only after preserving validated state. Avoid changing runtime semantics unless explicitly scoped.

GitHub status checks may be absent; continue to run local build/tests after code changes.
```

---

## 0. Ground-Truth Patch Policy

Ground-truth patch policy:

```text
Before offering code changes, inspect the current repo files that define the call path, data flow, flags, schemas, tests, and scripts being changed.

Do not guess file structure, option names, parser blocks, test layout, function signatures, enum values, generated artifact paths, or runtime wiring.

Do not assume a wrapper script forwards a flag just because the binary supports it. Verify the wrapper parser and pass-through path.

Do not assume a test failure is stale or caused by rebuild state until checking the source assertion, target path, and build output.

When enhancing runtime or data-flow code, first trace:
  source of data
  owning publisher / accumulator
  serialization boundary
  transport or artifact writer
  consuming test / tool / viewer
  validation command

Prefer small, anchored patches against inspected code. If the local file structure differs from the expected anchor, stop and request/inspect the relevant block instead of emitting increasingly broad regex patches.

Balance architectural purity, implementation efficiency, and development risk. Use C++ when the feature belongs in the runtime ownership boundary; use Python/tools only for diagnostics, offline conversion, or intentionally external workflows.
```

---

## 1. Active Milestones

```text
Milestones 2.20-2.23: implemented / validated.
Milestone 2.24: TargetSelector, ghost validation, reprojection validation, world-model evidence plumbing. Implemented through 2.24G.9 baseline.
Milestone 2.25: ObjectBehaviorMissionController skeleton and bounded follow baseline. Implemented.
Milestone 2.26: AirSim ghost behavior validation, live runtime-event stream, AirSim existing-object binding, follow arrival control, and live overlay / OSD. Complete after 2.26E validation.
Milestone 2.27A: robust circle behavior with continuous orbit-capture control law and orbit-count completion. Complete after 3-orbit AirSim validation.
Milestone 2.27B: validation hardening for circle/orbit behavior. Complete after validator CLI, validator test, runbook update, 34/34 CTest, and live trajectory validator pass.
Milestone 2.28B: no-shim target/tool/dependency layout migration. Complete after empty-state setup and live AirSim validation.
Milestone 2.28B.1: generated dependency staging under third_party/. Complete after PX4 SITL, iceoryx, and Colosseum assets staged under third_party and no stale generated dirs under root/simulation.
Milestone 2.28C: camera/gimbal pointing foundation. Complete after typed C++ CameraPointingCommand, lifecycle camera policy, runtime CameraPointingSink dispatch, native C++ MAVLink gimbal sink, AirSim camera bridge, one-command run_mission.sh, and canonical post-run validation.
Milestone 2.29A: behavior spec sequence schema foundation. Complete: BehaviorSpec carries optional yaw_mode and camera_pointing_mode per behavior node / sequence step.
Milestone 2.29B: sequence runtime execution. Complete: ObjectBehaviorMissionController executes approach -> circle sequence steps and applies per-step yaw/camera overrides during ExecuteMission.
Milestone 2.29C: AirSim sequence config and canonical sequence validation. Complete: sequence config/spec plus validate-mission-artifacts --expect-sequence and run_mission.sh sequence validation flags.
Milestone 2.29D: far static SEL and observability validation. Complete: BRPlayer_36 far-person sequence run validates active step fields and per-step yaw/camera mode assertions.
Milestone 2.29E: mixed-mode sequence validation. Complete: static far-person approach target/target -> circle trajectory/target validated.
Milestone 2.30A: slow moving SEL animal sequence validation. Complete: ghost_far_animal_001 at 0.20 m/s validated.
Milestone 2.30B: moving-target stress matrix. Complete: medium, side-motion, and diagonal far-animal trajectories validated.
Milestone 4.1C.2/4.1C.3: AirSim depth-frame classless obstacle detector core, sidecar acquisition, CoreStackRunner handoff, sampling fix, and OSD visualization are live-validated.
Active next slice: 5H persistent mission obstacle map export; 5I site-map merge; 5J derived score/age calculator; 5K run_mission.sh post-process; 5L runtime preload remains diagnostics-only until validated.
```

---

## 2. Current Runtime Architecture

```text
AirSim live frame + ego sidecar
  -> AirSimFrameSource
  -> FrameHintEgoProvider
  -> CoreStackRunner
       -> optional GhostTargetProvider::frame_at(...)
            -> GhostDetectionsPublisher
        -> SensingCoverageProvider
             -> obstacle_sensing_volumes
        -> optional FramePacket.depth_frame
             -> AirSimDepthObstacleDetector
             -> PerceptionPipelineOutput.obstacle_evidence
             -> classless airsim_depth_obstacle_detector evidence
           -> PerceptionPipelineOutput.observations
  -> InMemoryWorldModel
  -> WorldSnapshotPublisher
       -> LatestWorldSnapshotSubscriber
       -> ArtifactSnapshotWriter
       -> optional RuntimeEventStreamServer
  -> LatestWorldSnapshot
  -> MissionRuntime async loop
       -> MissionEventPublisher
  -> ObjectBehaviorMissionController
       -> BehaviorSpec sequence runtime
            -> active_behavior() step selection
            -> approach standoff step
            -> robust circle/orbit step
            -> behavior_sequence_step_start / behavior_sequence_step_complete
            -> behavior_tick_sample active step observability
       -> VelocityCommand policy for vehicle translation/yaw
       -> CameraPointingCommand policy for target/home/landing/neutral camera pitch
  -> Px4BridgeCommandSink
       -> persistent tools/px4/px4-command-bridge.py
       -> PX4 shell: arm, takeoff, land, disarm
       -> pymavlink: OFFBOARD velocity setpoints
       -> LOCAL_POSITION_NED feedback climb to safe height
  -> CameraPointingSink
       -> NullCameraPointingSink by default
       -> MavlinkGimbalPointingSink for real PX4/MAVLink gimbals
       -> runtime event projection for AirSim camera bridge compatibility
  -> PX4 / AirSim
```

Runtime stream camera side-channel for AirSim:

```text
dedalus_mission_loop --world-snapshot-stream-port 47770
  -> mission_event camera_pointing_intent
  -> simulation/airsim/scripts/airsim-camera-pointing-bridge.py
  -> AirSim simSetCameraPose(front_center, pitch)
  -> AirSim simSetCameraPose(0, pitch)
```

Runtime-event stream:

```text
dedalus_mission_loop --world-snapshot-stream-port 47770
  -> one TCP JSONL stream with:
       ghost_detections
       world_snapshot
       mission_event
```

Repo layout boundary:

```text
simulation/airsim/
  Dedalus-owned AirSim target runtime scripts, settings, logs, and AirSim-specific validation.

simulation/airsim/run.sh
  Starts AirSim / PX4 SITL runtime.

simulation/airsim/run_mission.sh
  Starts mission-loop + AirSim camera bridge + overlay + post-run validation in tmux.
  It now has a friendly AirSim RPC preflight so running it before run.sh exits with useful instructions.
  It supports sequence validation flags:
    --expect-sequence
    --expect-sequence-steps approach,circle
    --expect-sequence-step-modes approach:target:target,circle:target:target
    --validation-complete-reason sequence_complete

simulation/airsim/stop.sh
  Normal way to stop AirSim / PX4 SITL runtime.

cleanup.sh
  Root cleanup/reset helper for rebuild/reset state. Do not present cleanup.sh as the normal simulation stop command.

third_party/
  Generated external dependency checkouts/downloads:
    PX4-Autopilot/
    iceoryx_build/
    colosseum_environments/

tools/px4/
  Dedalus PX4/MAVLink protocol tools.

tools/mission/
  Canonical mission artifact validators and summaries.

tools/validation/
  Behavior/trajectory-specific validators.

config/behaviors/
  Behavior specs, trajectories, and ghost fixture assets.
```

Naming and artifact convention:

```text
Use architectural capability names, not planning labels or arbitrary placeholders.

Prefer names that encode the stable subsystem, runtime boundary, contract, scenario, or artifact role:
  out/object_behavior_airsim_existing_object_circle
  out/mission_loop_snapshots
  tools/mission/validate-obstacle-sensing-evidence-snapshots.py
  tools/mission/validate-mission-artifacts.py
  simulation/airsim/run_mission.sh
  obstacle_sensing_volumes
  obstacle_evidence
  runtime_event_stream
  world_snapshot

Avoid names based on planning labels or temporary session language:
  track4
  milestone_XXX
  phase_YYY
  latest_run
  mission_YYYY
  foo.json
  temp.json
  ad-hoc simulation/artifacts/mission_* unless that is the actual architectural path produced by the repo

When referring to output directories, use the concrete `--output-dir` value or the named directory printed by `dedalus_mission_loop` / `run_mission.sh`.
```

Core boundary:

```text
WorldSnapshot is autonomy state.
PerceptionPipelineOutput is evidence.
Ghost detections enter through the same Observation3D path as real detections.
Artifacts are evidence/debug outputs, not IPC.
Overlay is a subscriber/renderer only.
```

---

## 3. Completed Capabilities Through 2.29D

AirSim existing-object binding:

```text
config/core_stack_object_behavior_airsim_existing_object.yaml
  -> binds ghost_person_001 to BRPlayer_01_96 by default

config/core_stack_object_behavior_airsim_existing_object_circle.yml
  -> circle validation config using explicit existing-object binding

config/core_stack_object_behavior_airsim_existing_object_sequence.yml
  -> approach -> circle sequence config for BRPlayer_01_96 / ghost_person_001

config/core_stack_object_behavior_airsim_existing_object_sequence_far_person.yml
  -> approach -> circle sequence config for BRPlayer_36 / ghost_far_person_001

config/behaviors/circle_existing_object_person.yaml
  -> circle behavior spec for the validated existing-object path

config/behaviors/sequence_approach_circle_existing_object.yaml
  -> sequence behavior spec for ghost_person_001

config/behaviors/sequence_approach_circle_far_person.yaml
  -> sequence behavior spec for ghost_far_person_001 / BRPlayer_36

simulation/airsim/scripts/airsim-list-objects.py
  -> lists selectable scene objects by class/distance; used to pick BRPlayer_36 as a farther static person target

simulation/airsim/scripts/airsim-object-poses.py
  -> calls AirSim simGetObjectPose(object_name)
  -> returns compact JSON object poses

GhostTargetProvider(AirSimGhostObjectSourceConfig)
  -> converts selected AirSim object poses to GhostDetectionState
  -> emits GhostDetectionsFrame + Observation3D list
```

Follow behavior:

```text
ObjectBehaviorMissionController consumes WorldSnapshot agents through TargetSelector.
Follow uses target-relative observation geometry.
Follow arrival command is:
  command_velocity = target_velocity + closing_velocity
Static targets converge to zero relative velocity.
Moving targets converge toward matched target velocity.
Follow no longer relies on latching yaw to hide target-location drift.
```

Circle behavior:

```text
Circle is implemented as a continuous orbit-capture control law, not follow-with-sideways-offset and not a brittle waypoint script.

Command law:
  desired_velocity =
      target_velocity
    + tangent_velocity_at_current_radial_angle
    + radial_correction_velocity
    + altitude_correction_velocity

Behavior is robust to imperfect insertion geometry:
  - starts outside radius -> radial correction inward while tangential motion is active
  - starts inside radius -> radial correction outward while tangential motion is active
  - starts near radius but at arbitrary bearing -> enters orbit from current radial angle
  - overshoots -> radial correction recovers on subsequent ticks
  - once orbit mode is reached -> orbit_mode_latched remains true until completion/reset

For known static AirSim existing-object bindings, object_behavior_zero_target_velocity may be enabled so the controller does not velocity-match synthetic/static-object velocity noise. This zeroes target_velocity only; tangent velocity remains active.

Moving-target validation through 2.30B shows the orbit law is not direction-specific across +X, +Y, and diagonal target-center motion. Medium, side-motion, and diagonal far-animal trajectories all completed sequence behavior with stable orbit radius and correct target velocity propagation. See `docs/moving_target_behavior_validation_results.md`.
```

Sequence behavior:

```text
behavior.type: sequence is implemented in ObjectBehaviorMissionController.
The controller tracks sequence_step_index_ and active_behavior().
Approach steps complete when standoff is reached:
  range <= stop_distance_m + position_tolerance_m
Circle steps complete by orbit_count or duration_s.
Intermediate step completion emits:
  behavior_sequence_step_complete
  behavior_sequence_step_start for next step
Final step completion emits:
  behavior_complete reason=sequence_complete

Sequence step observability is present in behavior_tick_sample:
  active_behavior
  active_yaw_mode
  active_camera_pointing_mode
  sequence_step_index

## Persistent obstacle memory checkpoint

```text
5A-5G mission-local obstacle mapping path:
  AirSim DepthPlanar / future obstacle providers
    -> ObstacleEvidence
    -> MissionLocalObstacleMap
    -> LocalFlightMapSnapshot ego crop
    -> TrajectorySafetyEvaluator read-only diagnostics
    -> WorldSnapshot mission_local_obstacle_map diagnostics
    -> offline mission-local viewer
```

The next work is persistent obstacle memory:

```text
5H export mission_obstacle_map.json from snapshots
5I merge mission maps into maps/<site_id>/site_obstacle_map.json
5J compute age/freshness/active score as derived fields
5K add run_mission.sh post-process hooks
5L preload prior site map into mission-local accumulator as diagnostics
5M optional streaming map deltas
5N planner/control use only after explicit validation
```

Decay policy:

- Store absolute timestamps with explicit `time_unit: unix_ns`.
- Store raw evidence primitives: first/last seen, last confirmed occupied, last observed free, last in sensing frustum, positive/negative counts, source stats.
- Do not blindly decay/delete obstacles because a site has not been visited.
- Normalize cell age against whole-site staleness:
  `relative_gap_seconds = max(0, cell_age_seconds - site_staleness_seconds)`.
- Strong decay should come from contradiction or revisits without reconfirmation, not calendar time alone.
- Persisted maps are site-local, not necessarily geodetic/global, until a real site anchor is available.
```


## Current validated obstacle-memory pipeline — 5Q through 5U

The default AirSim obstacle-memory path is now compact-delta-first and SQLite-backed.

Runtime:
- AirSim depth obstacle evidence feeds the mission-local obstacle map.
- Runtime writes compact `mission_obstacle_map_deltas.jsonl`.
- Full `mission_obstacle_map_full.json` is debug/export-only by default and is enabled with `--write-full-obstacle-map-artifact`.

Post-mission:
- `mission_obstacle_map_deltas.jsonl` is imported into `mission_obstacle_map_deltas.sqlite`.
- The delta SQLite DB is compacted into `maps/<site_id>/site_obstacle_map.sqlite`.
- `out/<run>/obstacle_memory_manifest.json` records the selected merge path, site/mission ids, artifact paths, existence, and sizes.

Validation:
- `tools/avoidance/validate_obstacle_memory_manifest.py` validates the manifest schema, ids, selected site-map format, expected merge path, and artifact existence/size consistency.
- When `--merge-obstacle-map` is enabled, validation waits up to `OBSTACLE_MEMORY_MANIFEST_WAIT_SECONDS` seconds for the manifest.
- Default wait is 360 seconds when merging is enabled, or can be overridden with `DEDALUS_OBSTACLE_MEMORY_MANIFEST_WAIT_SECONDS=<seconds>`.
- Manifest validation covers `sqlite`, `json`, `both`, and `sqlite-full-json` formats through integration coverage.

Validated default path:
```text
mission_obstacle_map_deltas.jsonl
  -> mission_obstacle_map_deltas.sqlite
  -> maps/<site_id>/site_obstacle_map.sqlite
  -> out/<run>/obstacle_memory_manifest.json
```

Important commands:
```bash
simulation/airsim/run_mission.sh \
  --output-dir out/<run> \
  --merge-obstacle-map \
  --obstacle-map-site-id <site_id> \
  --obstacle-map-site-frame-id airsim_world \
  --obstacle-map-mission-id <mission_id>
```

Debug full JSON:
```bash
simulation/airsim/run_mission.sh \
  --output-dir out/<run> \
  --merge-obstacle-map \
  --write-full-obstacle-map-artifact \
  --obstacle-map-site-id <site_id> \
  --obstacle-map-site-frame-id airsim_world \
  --obstacle-map-mission-id <mission_id>
```

Manifest validator:
```bash
python3 tools/avoidance/validate_obstacle_memory_manifest.py \
  out/<run>/obstacle_memory_manifest.json \
  --site-id <site_id> \
  --site-frame-id airsim_world \
  --mission-id <mission_id> \
  --site-map-format sqlite
```
