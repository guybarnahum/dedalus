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
  -> async Behavior / Mission State Machine
  -> FlightCommandSink
  -> AirSim / PX4 SITL velocity control
```

The behavior pipeline is not just a synchronous post-world-model function. It is an async loop that watches the changing world model, maintains mission lifecycle state, and emits bounded kinematic intents when the current state requires action.

Initial command shape:

```text
velocity vector + yaw/yaw-rate intent
```

PX4 / the flight controller remains responsible for stabilization, estimator fusion, arming, motor control, low-level failsafes, and flight safety.

## Behavior pipeline semantics

The behavior pipeline should be modeled as a mission lifecycle state machine attached to the runtime by config.

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

The state machine owns mission intent and lifecycle transitions. The flight command sink only executes already-bounded commands; it should not decide mission phase.

## Config-selected mission controller

The config should declare which mission state machine to instantiate and attach to the system.

Proposed initial shape:

```yaml
behavior_pipeline: mission_state_machine
behavior_state_machine: trajectory_mission
behavior_tick_hz: 10
behavior_trajectory_path: simulation/trajectories/circle_figure8.json

flight_command_sink: airsim_velocity
flight_control_mode: px4
flight_safe_height_m: 8
flight_home_policy: initial_ego_pose
```

Provider naming intent:

```text
behavior_pipeline: mission_state_machine
  Creates the async behavior loop wrapper.

behavior_state_machine: trajectory_mission
  Creates the concrete mission lifecycle state machine.

flight_command_sink: airsim_velocity
  Sends bounded velocity commands into AirSim/PX4 SITL.
```

The state machine name should be the extension point for future mission controllers, for example:

```text
trajectory_mission
patrol_mission
inspect_agent_mission
follow_agent_mission
return_home_mission
search_pattern_mission
```

## Initial placeholder behavior provider

The first implementation should be a `trajectory_mission` state machine that mirrors the current `simulation/test-flight.py` behavior.

Purpose:

```text
- exercise the full perception -> world_model -> behavior -> flight-control loop
- keep behavior config-driven
- preserve the test-flight trajectory format and operational knowledge
- validate async behavior ticks separately from camera-frame ticks
- avoid coupling early behavior work to incomplete world-model semantics
```

The placeholder should still accept the latest `WorldSnapshot` or `EffectiveWorldView` input so the interface shape is future-ready, but the trajectory mission can initially ignore most world-model content.

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
  Play the configured trajectory JSON and emit velocity commands at behavior_tick_hz.

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

## Future behavior providers

After the trajectory placeholder is working, additional mission state machines can consume world-model state:

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

These should remain behind stable behavior interfaces and be selected by config.

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

struct BehaviorTickInput {
    TimePoint now;
    WorldSnapshot snapshot;
};

struct BehaviorTickOutput {
    MissionLifecycleState state;
    std::optional<VelocityCommand> command;
    std::string status;
};

class MissionStateMachine {
public:
    virtual ~MissionStateMachine() = default;
    virtual BehaviorTickOutput tick(const BehaviorTickInput& input) = 0;
};

class BehaviorRuntime {
public:
    virtual ~BehaviorRuntime() = default;
    virtual void start() = 0;
    virtual void stop() = 0;
};

class FlightCommandSink {
public:
    virtual ~FlightCommandSink() = default;
    virtual void send(const VelocityCommand& command) = 0;
};
```

The trajectory placeholder can be implemented as:

```text
TrajectoryMissionStateMachine
  reads test-flight.py-compatible trajectory JSON
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

The behavior runtime should be asynchronous relative to the frame pipeline:

```text
Perception/core-stack loop:
  frame -> perception -> world_model.update -> latest snapshot published

Behavior loop:
  at behavior_tick_hz:
    read latest snapshot
    state_machine.tick(snapshot)
    if command: flight_command_sink.send(command)
```

The first implementation can use an in-process latest-snapshot handoff. Later versions can move to a proper bus or shared-memory channel.

Avoid blocking frame ingestion on behavior decisions. Avoid blocking behavior ticks on camera frame timing unless explicitly configured.

## Safety and priority rule

The behavior pipeline should never bypass command arbitration. Long-term command priority remains:

```text
Hardware Kill Switch
  > Human RC Override
  > Safety Constraint Layer
  > Behavior / AI Planner Intent
```

For the placeholder trajectory provider, keep outputs bounded and explicit. Do not add aggressive autonomy, intercept logic, or target-following behavior until the command path, safety layer, and world-model semantics are validated.

## Recommended milestone split

```text
2.19A — Behavior/flight contracts and config loader keys
  Add VelocityCommand, MissionStateMachine, BehaviorRuntime, and FlightCommandSink contracts.
  Add config keys and provider registry placeholders.

2.19B — TrajectoryMissionStateMachine
  Parse the existing trajectory JSON format and emit velocity commands from ExecuteMission.
  Accept latest WorldSnapshot input and use ego/home state only where needed.

2.19C — AirSimVelocityCommandSink placeholder
  Send velocity commands to AirSim/PX4 SITL using the proven test-flight semantics.

2.19D — Async runtime wiring
  Add a behavior loop that scans latest world-model snapshots at behavior_tick_hz.
  Keep behavior asynchronous from frame ingestion and perception.

2.19E — Integration profile
  Run: live frame -> perception -> world model -> async trajectory mission -> AirSim/PX4 velocity control.
  Keep the old test-flight.py harness as an operational/debug baseline.
```
