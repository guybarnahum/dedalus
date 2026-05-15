# Behavior / Flight Pipeline Plan

This document records the next runtime direction after the Milestone 2.18 AirSim bridge profiling work.

## Validated Milestone 2.18E bridge baseline

The async frame-prefetch path is now measured with explicit wall-clock accounting:

```text
total_us              = measured CoreStackRunner::run_once() wall-clock time
accounted_total_us    = sum of non-attribution stages
accounting_delta_us   = total_us - accounted_total_us
```

`frame_source.detail.*` timings are background frame-fetch attribution only. They explain where the prefetched frame spent time, but they are excluded from `accounted_total_us` so the profiler does not double-count background work as serial main-thread latency.

Validated 640x360 `frame_ego` capacity result, 600 frames:

```text
actual_resolution_counts: 640x360:600
bridge/main wait p95:     30.220 ms  (GREEN, ~33.1 FPS capacity)
measured runner p95:      31.129 ms  (~32.1 FPS capacity)
accounted runner p95:     30.343 ms
accounting delta abs p95:  0.989 ms

background_fetch_detail:
  read_header p95:        30.519 ms
  read_payload p95:        0.856 ms

bridge-internal timing:
  sim_get_images_ms p95:  29.933 ms
  stdout_write_ms p95:     0.870 ms
```

Conclusion:

```text
At 640x360, the live frame_ego AirSim path is inside the 30 FPS p95 budget.
The remaining dominant cost is AirSim simGetImages / render readback / RPC.
C++ parsing, frame construction, and payload transfer are not the bottleneck.
```

The async prefetch architecture is still important because real perception will be more expensive than the current scripted placeholder. As real perception grows, the runtime can overlap perception/world-model work for frame N with sensor/AirSim acquisition for frame N+1.

## Next runtime direction

The next major integration goal is to close the loop from perception and world-model state into an asynchronous mission controller:

```text
FrameSource
  -> PerceptionPipeline
  -> WorldModel
  -> async MissionController loop
  -> FlightCommandSink
  -> AirSim / PX4 SITL velocity control
```

The behavior/mission system is not a synchronous post-world-model function. It is one runtime loop that watches the changing world model, maintains mission lifecycle state, and emits bounded kinematic intents when the current state requires action.

Initial command shape:

```text
velocity vector + yaw/yaw-rate intent
```

PX4 / the flight controller remains responsible for stabilization, estimator fusion, arming, motor control, low-level failsafes, and flight safety.

## Ego state in the world model

The world model must contain the drone ego state as first-class state, not only external detections. Mission controllers need ego state to make takeoff, landing, go-home, and failsafe decisions.

Minimum ego fields for the first flight loop:

```text
- ego pose / position in local coordinates
- height / altitude above takeoff or local frame
- velocity
- attitude if available
- home / initial pose policy state
- freshness / confidence
- armed / airborne / landed status when available
```

External objects should be representable relative to the drone. Landmarks may also carry map locations so the drone can place itself relative to stable observed features. For the first trajectory mission, ego height and home/initial pose are the key world-model inputs.

## Behavior pipeline semantics

The mission controller should be modeled as a lifecycle state machine attached to the single async behavior loop.

It should scan the latest world-model snapshot or effective world view at its own tick rate, not necessarily once per camera frame. It may respond to changes in:

```text
- ego state and arming/readiness
- mission phase
- dynamic agents
- tactical exclusion zones
- flight corridors
- home/return state
- timeout/failure conditions
- operator/configured mission intent
```

The first lifecycle state set should be intentionally simple:

```text
Idle
  -> Arm / Prepare
  -> Takeoff
  -> ExecuteMission
  -> GoHome
  -> Land
  -> Complete

Any active state
  -> Abort / Failsafe
```

The mission controller owns mission intent and lifecycle transitions. The flight command sink only executes already-bounded commands; it should not decide mission phase.

## Config-selected mission controller

There is one in-process async mission runtime loop, so runtime selection does not need to be exposed in config. Config should only select the mission controller, its tick rate, the flight command sink, and controller-specific options.

Proposed initial shape:

```yaml
mission_controller: trajectory_mission
mission_tick_hz: 10
flight_command_sink: airsim_velocity

mission_options:
  flight_control_mode: px4
  flight_safe_height_m: 8
  flight_trajectory_path: simulation/trajectories/circle_figure8.json
  flight_home_policy: initial_ego_pose
```

Naming intent:

```text
mission_controller
  Selects the concrete mission lifecycle state machine.

mission_tick_hz
  Controls the fixed-rate async behavior loop.

flight_command_sink
  Selects where bounded velocity commands are sent.

mission_options
  Passed to the selected mission_controller. These options configure mission logic;
  they do not change how the async runtime loop is scheduled or executed.
```

The mission controller name should be the extension point for future mission logic, for example:

```text
trajectory_mission
patrol_mission
inspect_agent_mission
follow_agent_mission
return_home_mission
search_pattern_mission
```

## Initial placeholder mission controller

The first implementation should be a `trajectory_mission` controller that mirrors the current `simulation/test-flight.py` behavior.

Purpose:

```text
- exercise the full perception -> world_model -> mission_controller -> flight-control loop
- keep mission behavior config-driven
- preserve the test-flight trajectory format and operational knowledge
- validate async behavior ticks separately from camera-frame ticks
- validate ego state in the world model for takeoff, go-home, and land
- avoid coupling early behavior work to incomplete object/landmark semantics
```

The placeholder should accept the latest `WorldSnapshot` or `EffectiveWorldView` input. It can initially ignore most non-ego world-model content, but it should use ego height and home/initial pose when possible.

Trajectory mission lifecycle:

```text
Idle
  Wait for runtime start and required providers.

Arm / Prepare
  Ensure the selected flight command sink is ready.
  For SITL, mirror the proven test-flight.py arming/takeoff assumptions.

Takeoff
  Climb to the configured safe height or wait for ego altitude to satisfy the takeoff condition.

ExecuteMission
  Play the configured trajectory JSON and emit velocity commands at mission_tick_hz.

GoHome
  Return toward the initial/home ego pose or configured home policy.
  The first placeholder may use a bounded simple velocity toward home.

Land
  Command landing through the flight sink or emit a controlled descent placeholder.

Complete
  Stop sending mission velocity commands.

Abort / Failsafe
  Emit hold/stop or hand off to safety logic.
```

## Future mission controllers

After the trajectory placeholder is working, additional mission controllers can consume world-model state:

```text
hold_position_mission
orbit_agent_mission
follow_agent_mission
inspect_agent_mission
avoid_exclusion_zones_mission
return_to_home_mission
search_pattern_mission
intercept_or_shadow_mission
```

These should remain behind stable mission interfaces and be selected by config.

## Suggested C++ contracts

Sketch only; names can change during implementation.

```cpp
struct VelocityCommand {
    TimePoint timestamp;
    Vec3 velocity_local_mps;
    double yaw_rate_radps{0.0};
    double yaw_rad{0.0};
    bool yaw_rate_valid{true};
    bool yaw_valid{false};
};

enum class MissionLifecycleState {
    Idle,
    Prepare,
    Takeoff,
    ExecuteMission,
    GoHome,
    Land,
    Complete,
    Abort
};

struct MissionTickInput {
    TimePoint now;
    WorldSnapshot snapshot;
};

struct MissionTickOutput {
    MissionLifecycleState state;
    std::optional<VelocityCommand> command;
    std::string status;
};

class MissionController {
public:
    virtual ~MissionController() = default;
    virtual MissionTickOutput tick(const MissionTickInput& input) = 0;
};

class FlightCommandSink {
public:
    virtual ~FlightCommandSink() = default;
    virtual void send(const VelocityCommand& command) = 0;
};
```

The async mission runtime is an implementation detail, not a config-selected provider:

```text
MissionRuntime
  owns one async loop
  ticks the selected MissionController at mission_tick_hz
  reads latest WorldSnapshot
  forwards optional VelocityCommand to FlightCommandSink
```

The trajectory placeholder can be implemented as:

```text
TrajectoryMissionController
  reads test-flight.py-compatible trajectory JSON from mission_options.flight_trajectory_path
  advances by monotonic time or mission elapsed time
  watches latest WorldSnapshot for ego/home/altitude readiness
  outputs VelocityCommand during ExecuteMission
```

The first AirSim/PX4 sink can be implemented as:

```text
AirSimVelocityCommandSink
  sends bounded velocity commands into AirSim/PX4 SITL
  uses the same safe operational assumptions as simulation/test-flight.py
```

## Runtime threading model

The mission runtime should be asynchronous relative to the frame pipeline:

```text
Perception/core-stack loop:
  frame -> perception -> world_model.update -> latest snapshot published

Mission loop:
  at mission_tick_hz:
    read latest snapshot
    mission_controller.tick(snapshot)
    if command: flight_command_sink.send(command)
```

The first implementation can use an in-process latest-snapshot handoff. Later versions can move to a proper bus or shared-memory channel.

Avoid blocking frame ingestion on mission decisions. Avoid blocking mission ticks on camera frame timing unless explicitly configured for debugging.

## Safety and priority rule

The mission controller should never bypass command arbitration. Long-term command priority remains:

```text
Hardware Kill Switch
  > Human RC Override
  > Safety Constraint Layer
  > Mission Controller Intent
```

For the placeholder trajectory controller, keep outputs bounded and explicit. Do not add aggressive autonomy, intercept logic, or target-following behavior until the command path, safety layer, and world-model semantics are validated.

## Recommended milestone split

```text
2.19A — Mission/flight contracts and config loader keys
  Add VelocityCommand, MissionController, MissionRuntime, and FlightCommandSink contracts.
  Add config keys: mission_controller, mission_tick_hz, mission_options, flight_command_sink.

2.19B — Ego state in WorldModel
  Ensure latest drone ego state is represented in the world model with enough altitude/home status
  for takeoff, go-home, and landing decisions.

2.19C — TrajectoryMissionController
  Parse the existing trajectory JSON format and emit velocity commands from ExecuteMission.
  Accept latest WorldSnapshot input and use ego/home state where needed.

2.19D — AirSimVelocityCommandSink placeholder
  Send velocity commands to AirSim/PX4 SITL using the proven test-flight semantics.

2.19E — Async runtime wiring
  Add a mission loop that scans latest world-model snapshots at mission_tick_hz.
  Keep mission control asynchronous from frame ingestion and perception.

2.19F — Integration profile
  Run: live frame -> perception -> world model -> async trajectory mission -> AirSim/PX4 velocity control.
  Keep the old test-flight.py harness as an operational/debug baseline.
```
