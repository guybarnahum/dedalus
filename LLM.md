# Dedalus LLM Operating Brief

This file is the active orientation document for a new LLM session. Keep it short, current, and action-oriented.

Historical notes, superseded debugging context, and older milestone details live in `LLM.back.md`.

Repository:

```text
guybarnahum/dedalus
```

Current code baseline for this handoff:

```text
main at / after commit ac5493139a196e293a4e8bcec889d3e9c5605f8c
```

Active milestone state:

```text
Milestone 2.20 — Mission robustness, observability, and cleanup
Status: closed / validated.

Milestone 2.21 — Mission artifact validation and replay-grade diagnostics
Status: implemented on main; local validation still expected after checkout.

Next active milestone:
Milestone 2.22 — Scenario/campaign harness.
```

Patch policy:

```text
Default: apply changes directly to main.
Do not create branches or PRs unless the user explicitly asks for a branch or PR.
Do not leave completed work sitting on a feature branch.

Prefer GitHub connector file updates directly on main when available.
If connector patching fails or is ambiguous, provide an exact manual patch.

For normal patches:

  cat > /tmp/change.patch <<'PATCH'
  diff --git ...
  PATCH
  git apply /tmp/change.patch

For full-file replacements:

  cat > /tmp/update_file.sh <<'SH'
  #!/usr/bin/env bash
  set -euo pipefail
  cat > path/to/file <<'EOF'
  ... complete file content ...
  EOF
  SH
  bash /tmp/update_file.sh
```

Current working result:

```text
- `simulation/test-flight.py --trajectory trajectories/circle_figure8.json` works well.
- `dedalus_mission_loop` flies through the mission path using `flight_command_sink: px4_bridge`.
- Back-to-back mission-loop runs work without restarting AirSim.
- Build succeeds.
- CTest expected result after 2.21: 19/19 passing.
- Live mission reaches safe height through pymavlink OFFBOARD control, executes the trajectory, goes home, lands, and disarms through PX4 shell lifecycle commands.
- Ctrl-C / SIGTERM requests graceful mission finish on first interrupt.
- `mission_events.jsonl` is the source artifact for mission debugging and final summaries.
- `simulation/validate-mission-artifacts.py` validates live-run artifact directories.
```

Core rule:

```text
Synchronous command dispatch success is not vehicle-state truth.
Mission transitions into flight execution must be driven by world-model telemetry.
```

Milestone 3 direction:

```text
Milestone 3.0 is now defined as object-conditioned flight behavior:
  detect / track a class instance such as person or car
    -> select target
    -> follow, circle, approach, or sequence behavior
    -> emit bounded velocity vectors through PX4 SITL/AirSim
    -> return / land / disarm
    -> validate artifacts.
```

Post-Milestone 3 direction:

```text
After M3, behaviors should become obstacle-aware through a tactical occupancy / avoidance layer that modifies desired velocity vectors before they reach the flight sink. The sink still receives bounded velocity/yaw intent only.
```

---

## 1. Current Architecture

Dedalus is a live/simulated drone autonomy stack:

```text
sensors / simulation
  -> perception
  -> world model
  -> behavior / mission controller
  -> flight-command sink
  -> AirSim / PX4
```

Current live mission pipeline:

```text
AirSim live frame + ego sidecar
  -> AirSimFrameSource
  -> FrameHintEgoProvider
  -> CoreStackRunner
  -> InMemoryWorldModel
  -> LatestWorldSnapshot
  -> MissionRuntime async loop
  -> TrajectoryMissionController
  -> Px4BridgeCommandSink
  -> persistent simulation/px4-command-bridge.py
       - PX4 shell: arm, takeoff, land, disarm
       - pymavlink: OFFBOARD mode + SET_POSITION_TARGET_LOCAL_NED velocity
       - LOCAL_POSITION_NED feedback climb to safe height
  -> PX4 / AirSim
```

Milestone 3 target architecture:

```text
AirSim live frame + ego sidecar
  -> AirSimFrameSource
  -> perception / detector / tracker
  -> WorldSnapshot agents with class, confidence, local position, velocity
  -> TargetSelector
  -> BehaviorRuntime / ObjectBehaviorMissionController
  -> desired velocity vector
  -> optional future TacticalAvoidancePlanner
  -> Px4BridgeCommandSink
  -> PX4 / AirSim
```

Main current app:

```text
apps/dedalus_mission_loop.cpp
```

The old name `dedalus_replay_mission` is obsolete. Do not use it.

Snapshot artifacts are debug outputs showing what the world model / mission handoff saw. They are not necessarily replay inputs.

---

## 2. Control Path That Works

The working control split deliberately mirrors `simulation/test-flight.py`.

```text
Prepare:
  deterministic AirSim/PX4 session prep
  confirm AirSim connection, GPS validity, API control, and MAVLink reachability

Arm:
  PX4 shell: commander arm

Takeoff:
  PX4 shell: commander takeoff
  bridge waits for takeoff settle

First velocity / climb:
  lazy pymavlink connection after shell takeoff
  prime OFFBOARD velocity stream
  set PX4 OFFBOARD mode
  climb to safe height using LOCAL_POSITION_NED feedback

Trajectory / behavior velocity:
  pymavlink SET_POSITION_TARGET_LOCAL_NED velocity setpoints

Landing:
  PX4 shell: commander land
  wait for ego telemetry / landed height

Complete:
  PX4 shell: commander disarm
```

Key files:

```text
simulation/test-flight.py                    known-good standalone reference
simulation/px4-command-bridge.py             persistent bridge used by mission runtime
src/behavior/px4_bridge_command_sink.cpp      C++ JSONL process-backed command sink
src/behavior/trajectory_mission_controller.cpp
src/behavior/mission_runtime.cpp
apps/dedalus_mission_loop.cpp
config/core_stack_trajectory_mission_placeholder.yaml
```

Do not reintroduce the hand-written native C++ MAVLink encoder as the default live path. `src/behavior/px4_mavlink_command_sink.cpp` may remain as experimental/deprecated code, but the live mission should use `flight_command_sink: px4_bridge`.

---

## 3. Python Helpers vs Native C++ Decision

AirSim itself has a native C++ client API. Python helpers are not used because C++ cannot talk to AirSim.

The current boundary decision is:

```text
- Keep the mission state machine, world model, runtime, behavior controllers, and provider interfaces in C++.
- Keep PX4/MAVLink/OFFBOARD mission control in the Python `px4-command-bridge.py` for now because it uses the same `pymavlink` behavior proven by `simulation/test-flight.py`.
- Do not rewrite the working MAVLink control path in C++ while the mission is still being stabilized.
```

Important distinction:

```text
AirSim RPC control != PX4 control
```

AirSim C++ can eventually replace Python helpers for simulator-side concerns:

```text
- frame streaming
- ego/state reads
- session prep / API control
```

But PX4 trajectory and behavior velocity control currently depends on the validated `pymavlink` path:

```text
- MAVLink heartbeat and target routing
- PX4 mode mapping and COMMAND_ACK handling
- OFFBOARD priming timing
- LOCAL_POSITION_NED feedback climb
- SET_POSITION_TARGET_LOCAL_NED velocity setpoints
```

Recommended migration order:

```text
1. Stabilize current `px4_bridge` mission path.
2. Build object-conditioned behavior on top of the existing velocity-command path.
3. Migrate AirSim frame/ego/session helpers to native C++ if needed.
4. Only later consider a native C++ PX4/MAVLink backend using a real tested MAVLink library, not ad-hoc packet encoding.
```

Bottom line:

```text
Python is not required for AirSim access. It is currently required for the stable PX4/MAVLink mission-control path because `pymavlink` + `test-flight.py` is the proven implementation.
```

---

## 4. Separation of Concerns

Keep these layers separate.

### 4.1 Command intent

Command intent records what the mission runtime requested or dispatched.

Examples:

```text
flight_control.arm_state = arm_requested
  The Arm command dispatch path returned OK.

flight_control.arm_state = arm_failed
  The Arm command dispatch path failed.

flight_control.arm_state = armed_confirmed
  Ego telemetry later confirmed the drone is armed.
```

### 4.2 Telemetry truth

Telemetry truth comes from the vehicle/sim state represented in `WorldSnapshot.ego`.

Examples:

```text
ego.armed_valid && ego.armed
ego.armed_valid && !ego.armed
ego.height_valid && ego.height_m >= safe_height
ego.flight_status
```

### 4.3 Mission lifecycle

The mission state machine owns mission phase:

```text
Prepare
Takeoff
ExecuteMission
GoHome
Land
Complete
Abort
```

Mission lifecycle may look at command intent and telemetry truth, but it must transition to flight execution only from telemetry truth.

Repeat runs exposed a stale armed-telemetry case even though shell arm/takeoff remained healthy. The live config therefore enables:

```yaml
mission_options.flight_arm_dispatch_fallback_s: 2.0
```

This fallback may advance from `Prepare` to `Takeoff` after successful Arm dispatch and a short settle interval when armed telemetry is stale. It does **not** move to `ExecuteMission`; `ExecuteMission` remains gated by ego height reaching safe height.

### 4.4 Behavior intent

Behavior decides what the drone wants to do:

```text
follow target
circle target
approach target
hold/search/go_home/land
```

Behavior should emit desired velocity/yaw intent, not low-level motor/attitude commands.

### 4.5 Future avoidance layer

Post-M3, avoidance modifies behavior intent into a safe command:

```text
BehaviorController
  -> desired velocity vector
  -> TacticalAvoidancePlanner
  -> safe velocity vector
  -> FlightCommandSink
```

Do not make the sink understand obstacles. Do not bury behavior or avoidance logic inside `px4-command-bridge.py`.

---

## 5. Milestone Journey

| Stage | Name | Status | Notes |
|---|---|---:|---|
| 2.1–2.18 | Frame/source/provider/bridge foundation | Done | Synthetic, recorded, AirSim, binary bridge, timing, annotation |
| 2.19A–K | Mission/ego/control foundation | Done | Ego state, mission config, runtime, command intent, armed telemetry |
| 2.19L | PX4 bridge mission parity | Done | Mission path follows `test-flight.py` control sequence |
| 2.19M | Quiet/verbose logs | Done enough | CLI verbosity exists; default is high-level output plus final summary |
| 2.20A | Mission docs/current state | Done | `docs/mission_pipeline_current_state.md` |
| 2.20B | Mission event artifacts | Done | `mission_events.jsonl` from `MissionRuntime` |
| 2.20C | Final summary from events | Done | `dedalus_mission_loop` summarizes `mission_events.jsonl` |
| 2.20D | Repeatable-run hardening | Done / validated | Back-to-back mission runs work without restarting AirSim |
| 2.20E | Closeout tooling/checkpoint | Done | Event summary helper + repeat smoke wrapper + updated docs |
| 2.21 | Mission artifact validator | Implemented | `simulation/validate-mission-artifacts.py` + CTest smoke |

---

## 6. Roadmap to Milestone 3

The path from 2.20 to 3.0 is no longer about making the drone fly. It is about making the drone fly **based on detected/tracked objects**.

```text
2.21 Mission artifact validator
  Validate mission_events + snapshots as a formal live-run artifact directory. Implemented on main.

2.22 Scenario/campaign harness
  Run repeatable mission scenarios/campaigns and preserve metadata.

2.23 Behavior spec parser foundation
  Parse a small declarative behavior language from YAML/JSON.

2.24 Target selector from WorldSnapshot agents
  Select class or instance targets from tracked agents.

2.25 ObjectBehaviorMissionController
  Add a mission controller that owns object-conditioned behavior lifecycle.

2.26 Follow behavior
  Maintain a relative 3D offset from a selected class instance.

2.27 Circle behavior
  Orbit a selected static or slow class instance such as a car/person.

2.28 Approach + behavior sequence
  Approach until standoff/relative condition, then execute follow/circle/action.

2.29 M3 demo hardening
  Validate object-conditioned flight artifacts, repeatability, fallback behavior, and docs.

3.0 Object-conditioned flight behavior demo
  Detect/select class instance -> follow/circle/approach -> return/land/disarm.
```

Milestone 3.0 success criteria:

```text
1. Drone takes off and reaches safe height.
2. WorldSnapshot contains a valid detected/tracked target class instance.
3. TargetSelector selects a class or instance, e.g. person or car.
4. BehaviorRuntime starts follow, circle, approach, or sequence behavior.
5. Controller emits bounded velocity vectors through the existing PX4 bridge path.
6. Drone returns home, lands, and disarms.
7. mission_events + snapshots prove target selection, behavior execution, and completion.
```

---

## 7. Behavior Language v1

The behavior language should be small and declarative.

Core concepts:

```text
Target selector
Behavior type
Reference frame
Desired relative geometry
Constraints
Completion condition
Fallback behavior
```

Example follow behavior:

```yaml
mission:
  name: follow_person_demo

target:
  selector:
    class: person
    confidence_min: 0.55
    policy: highest_confidence

behavior:
  type: follow
  target_frame: target_heading_frame
  relative_offset_m:
    x: -8.0
    y: 0.0
    z: 4.0
  max_speed_mps: 2.0
  position_tolerance_m: 1.5
  lost_target_timeout_s: 5.0

completion:
  after_s: 30
  then: go_home_land
```

Example circle behavior:

```yaml
mission:
  name: circle_car_demo

target:
  selector:
    class: car
    confidence_min: 0.6
    policy: nearest

behavior:
  type: circle
  radius_m: 10.0
  altitude_offset_m: 5.0
  angular_speed_deg_s: 12.0
  direction: clockwise
  center:
    target: selected_target
  max_speed_mps: 3.0
```

Example approach-then-circle sequence:

```yaml
behavior:
  type: sequence
  steps:
    - type: approach
      stop_distance_m: 8.0
      altitude_offset_m: 4.0
    - type: circle
      radius_m: 10.0
      duration_s: 20.0
    - type: go_home_land
```

Initial behavior types:

```text
hold
search
follow
approach
circle
go_home
land
sequence
```

Reference frames to support over time:

```text
target_heading_frame
  offset relative to target direction of travel

drone_heading_frame
  offset relative to current drone heading

world_local_frame
  offset fixed in takeoff-local / map coordinates

camera_frame
  offset based on drone POV/image bearing
```

For Milestone 3, prefer `target_heading_frame` and `world_local_frame` first.

---

## 8. Post-Milestone 3 Spatial Autonomy Roadmap

After M3, the same behaviors should become obstacle-aware while still emitting velocity vectors into PX4 SITL/AirSim.

```text
4.0 Local tactical occupancy map
  Build a drone-relative real-time occupancy map from vision/ego motion.

5.0 Reactive obstacle avoidance planner
  Modify desired behavior velocity into safe velocity using tactical occupancy.

6.0 Persistent traverse map / flight memory
  Remember known-safe corridors, blocked regions, risk/cost surfaces, and preferred lanes across flights.

7.0 Cached route solutions
  Reuse successful routes when live sensing and map priors agree; invalidate them when live sensing contradicts memory.

8.0 Tactical map + drone POV visualization
  Visualize both takeoff-origin/drone-relative map and camera POV overlays.

9.0 Integrated spatial autonomy demo
  Avoid static/dynamic obstacles while following/circling/approaching an object-conditioned target and updating traverse memory.

10.0 Multi-flight site memory
  Maintain site-local memory across missions, including route priors, hazards, repeated obstacle history, and mission outcomes.
```

Important design split:

```text
Tactical occupancy map:
  real-time, short-horizon, safety-critical

Persistent traverse map:
  slower, historical, advisory

Route cache:
  proposed solution memory, invalidated by live sensing

Behavior:
  decides intent

Avoidance planner:
  modifies intent into safe motion

Flight sink:
  sends bounded velocity/yaw intent to PX4/SITL
```

---

## 9. Mission Artifacts and Tools

Mission loop output directory contains:

```text
snapshot_XXXX.json
snapshot_manifest.txt
mission_events.jsonl
```

`mission_events.jsonl` is the compact structured timeline and should be the first artifact inspected for mission behavior.

Summary helper:

```bash
python3 simulation/mission-events-summary.py out/airsim_mission_snapshots/mission_events.jsonl
python3 simulation/mission-events-summary.py out/airsim_mission_snapshots/mission_events.jsonl --expect-complete
```

Formal live-run artifact validator:

```bash
python3 simulation/validate-mission-artifacts.py \
  out/airsim_mission_snapshots \
  --expect-complete \
  --safe-height-m 16 \
  --landed-height-m 1
```

Future M3 behavior-artifact validation:

```bash
python3 simulation/validate-mission-artifacts.py \
  out/object_behavior_mission \
  --expect-complete \
  --expect-behavior \
  --safe-height-m 16 \
  --landed-height-m 1
```

Repeat-run smoke helper:

```bash
RUNS=3 simulation/repeat-mission-smoke.sh
```

`repeat-mission-smoke.sh` assumes AirSim/PX4 is already running. It runs `dedalus_mission_loop` repeatedly and validates each produced `mission_events.jsonl` with `--expect-complete`.

If the scripts are not executable after GitHub checkout, run:

```bash
chmod +x simulation/repeat-mission-smoke.sh simulation/mission-events-summary.py simulation/validate-mission-artifacts.py
```

---

## 10. Commands

### 10.1 Build/test

```bash
cd ~/dedalus
source venv/bin/activate

cmake --build build-staging -j$(nproc)
ctest --test-dir build-staging --output-on-failure
```

Expected current result:

```text
100% tests passed, 0 tests failed out of 19
```

### 10.2 Standalone known-good test flight

```bash
cd ~/dedalus
source venv/bin/activate

python ./simulation/test-flight.py --trajectory trajectories/circle_figure8.json
```

### 10.3 Start AirSim

```bash
cd ~/dedalus/simulation
./stop.sh
./run.sh AirSimNH --airsim-camera-width 640 --airsim-camera-height 360
```

### 10.4 Run live mission loop

Quiet/default:

```bash
cd ~/dedalus
source venv/bin/activate

./build-staging/apps/dedalus_mission_loop \
  --config config/core_stack_trajectory_mission_placeholder.yaml \
  --output-dir out/airsim_mission_snapshots \
  --max-frames 900 \
  --shutdown-max-frames 400 \
  --progress 2>&1 | tee out/airsim_mission_debug.log
```

Full debug:

```bash
./build-staging/apps/dedalus_mission_loop \
  --config config/core_stack_trajectory_mission_placeholder.yaml \
  --output-dir out/airsim_mission_snapshots \
  --max-frames 900 \
  --shutdown-max-frames 400 \
  --progress \
  --verbose 2>&1 | tee out/airsim_mission_debug.log
```

Repeat validation:

```bash
RUNS=3 simulation/repeat-mission-smoke.sh
```

Verbosity contract:

```text
default: high-level mission state transitions + final summary
-v: lifecycle/prep details
-vv: command summaries + sampled world snapshots
-vvv / --verbose: full detailed tick/sink/bridge tracing
```

---

## 11. Ctrl-C / Shutdown Behavior

`dedalus_mission_loop` handles interrupts:

```text
First Ctrl-C / SIGTERM:
  requests graceful mission finish through MissionRuntime.

Second Ctrl-C:
  stops the local main loop after local cleanup paths run.
```

The Python PX4 bridge handles a JSONL `shutdown` command from the C++ sink. On shutdown, if MAVLink was active, it sends a short zero-velocity settle stream, closes the MAVLink socket, and resets internal OFFBOARD/safe-height state before process exit.

Abort is terminal/diagnostic and does not emit velocity commands.

---

## 12. Known Traps

```text
- Do not use dedalus_replay_mission. It was renamed to dedalus_mission_loop.
- Do not treat snapshot artifacts as proof of replay input.
- Do not treat command helper OK as vehicle-state truth.
- Do not hide arming inside velocity commands.
- Do not collapse flight_control.arm_state and ego.armed.
- Do not move to ExecuteMission until Takeoff is confirmed by ego height.
- Do not make the native C++ MAVLink sink the default live path; use px4_bridge.
- Do not debug the working mission by rewriting MAVLink packet encoding in C++.
- Do not let the telemetry sidecar and command bridge fight over the same MAVLink endpoint.
- Do not replace `px4-command-bridge.py` with native C++ until the mission is stable and a real tested MAVLink C++ backend is planned.
- Do not refactor `test-flight.py` / `px4-command-bridge.py` unless repeat-run smoke remains stable.
- Do not put obstacle avoidance inside the flight sink.
- Do not let route memory override fresh tactical sensing.
- Do not let Milestone 3 balloon into full obstacle avoidance; M3 is object-conditioned behavior. Avoidance starts post-M3.
- Do not create branches or PRs unless the user explicitly asks for them.
```

---

## 13. Recommended Next Stage

Recommended next active stage:

```text
Milestone 2.22 — Scenario/campaign harness
```

Suggested first tasks:

```text
1. Wrap repeatable mission scenarios/campaigns around the validated run artifact directory contract.
2. Preserve per-run metadata beside mission_events.jsonl and snapshots.
3. Use validate-mission-artifacts.py as the post-run gate for live mission artifacts.
4. Keep mission event validation separate from frame replay semantics.
5. Preserve the object-conditioned behavior validation extension points for M3.
```

Expected later M3 event types:

```json
{"event":"target_selected","class":"car","track_id":"agent_3","confidence":0.86}
{"event":"behavior_start","behavior":"approach","target":"agent_3"}
{"event":"behavior_complete","behavior":"approach","reason":"standoff_reached"}
{"event":"behavior_start","behavior":"circle","target":"agent_3"}
{"event":"behavior_complete","behavior":"circle","reason":"duration_elapsed"}
```

---

## 14. Handoff Prompt Format

Every new worker handoff should include:

```text
1. Repo and current commit.
2. Active milestone and exact stage.
3. Current architecture summary.
4. Current observed behavior / logs.
5. Current diagnosis.
6. Immediate next tasks in order.
7. Explicit non-goals / traps.
8. Build/test/run commands.
9. Expected success signal.
```

---

## 15. Pointers

Detailed / historical context:

```text
LLM.back.md
docs/core_stack_current_state.md
docs/bridge_transport_plugins.md
docs/binary_frame_bridge_protocol.md
docs/perception_stabilization_annotation.md
docs/mission_pipeline_current_state.md
docs/object_conditioned_behavior_plan.md
WHITEPAPER.md
HANDOFF.md
```