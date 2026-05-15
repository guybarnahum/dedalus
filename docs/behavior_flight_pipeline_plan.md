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

The next major integration goal is to close the loop:

```text
FrameSource
  -> PerceptionPipeline
  -> WorldModel
  -> BehaviorPipeline
  -> FlightCommandSink
  -> AirSim / PX4 SITL velocity control
```

The behavior pipeline should consume the world model and generate bounded kinematic intents, not direct motor or attitude commands.

Initial command shape:

```text
velocity vector + yaw/yaw-rate intent
```

PX4 / the flight controller remains responsible for stabilization, estimator fusion, arming, motor control, low-level failsafes, and flight safety.

## Initial placeholder behavior provider

The first implementation should be a placeholder behavior provider that ignores the world model and plays a trajectory file, mirroring the current `simulation/test-flight.py` behavior.

Purpose:

```text
- exercise the full perception -> world_model -> behavior -> flight-control loop
- keep behavior config-driven
- preserve the test-flight trajectory format and operational knowledge
- avoid coupling early behavior work to incomplete world-model semantics
```

Initial config shape:

```yaml
behavior_pipeline: trajectory
behavior_trajectory_path: simulation/trajectories/circle_figure8.json
behavior_rate_hz: 10

flight_command_sink: airsim_velocity
flight_control_mode: px4
flight_safe_height_m: 8
```

The placeholder should run even if the world model is trivial. It should accept a `WorldSnapshot` or `EffectiveWorldView` input so the interface shape is future-ready, but the trajectory implementation can ignore that input.

## Future behavior providers

After the trajectory placeholder is working, additional behavior providers can consume world-model state:

```text
hold_position
orbit_agent
follow_agent
inspect_agent
avoid_exclusion_zones
return_to_home
search_pattern
intercept_or_shadow
```

These should remain behind a stable `BehaviorPipeline` interface.

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

class BehaviorPipeline {
public:
    virtual ~BehaviorPipeline() = default;
    virtual std::optional<VelocityCommand> update(
        const WorldSnapshot& snapshot) = 0;
};

class FlightCommandSink {
public:
    virtual ~FlightCommandSink() = default;
    virtual void send(const VelocityCommand& command) = 0;
};
```

The trajectory placeholder can be implemented as:

```text
TrajectoryBehaviorPipeline
  reads test-flight.py-compatible trajectory JSON
  advances by monotonic time or frame timestamp
  outputs VelocityCommand
```

The first AirSim/PX4 sink can be implemented as:

```text
AirSimVelocityCommandSink
  sends bounded velocity commands into AirSim/PX4 SITL
  uses the same safe operational assumptions as simulation/test-flight.py
```

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
  Add BehaviorPipeline, VelocityCommand, FlightCommandSink contracts.
  Add config keys and provider registry placeholders.

2.19B — TrajectoryBehaviorPipeline
  Parse the existing trajectory JSON format and emit velocity commands.
  Ignore the world model but accept WorldSnapshot input.

2.19C — AirSimVelocityCommandSink placeholder
  Send velocity commands to AirSim/PX4 SITL using the proven test-flight semantics.

2.19D — Runtime wiring
  Extend CoreStackRunner or add a flight-capable runner so each world-model update can feed behavior and command output.

2.19E — Integration profile
  Run: live frame -> perception -> world model -> trajectory behavior -> AirSim/PX4 velocity control.
  Keep the old test-flight.py harness as an operational/debug baseline.
```
