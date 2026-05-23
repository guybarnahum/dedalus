# Flight Behavior Control Laws

This note records a design principle for object-conditioned flight behaviors in Dedalus.

Object-conditioned behaviors must be implemented as control laws over relative state, not as brittle waypoint scripts. A behavior should remain well-defined across a wide range of initial and final conditions, including imperfect position, velocity, attitude, target motion, command latency, simulator drift, estimator noise, and overshoot. The controller should continuously recompute its desired velocity from the current ego state and selected target state so it can recover toward the intended behavior instead of requiring the vehicle to pass through a single ideal insertion point.

This principle applies especially to circle/orbit behavior.

## Circle / Orbit Control Principle

Circle behavior should not be implemented as follow-with-sideways-offset or as a hard transition through one exact entry coordinate. It should be formulated as a continuous orbit-capture and orbit-maintenance law.

The desired command should be composed from explicit velocity terms:

```text
desired_velocity =
    target_velocity
  + tangent_velocity_at_current_radial_angle
  + radial_correction_velocity
  + altitude_correction_velocity
```

Where:

```text
target_velocity:
  The selected target's local velocity. For known static AirSim existing-object bindings, this may be configured to zero so behavior does not velocity-match synthetic tracker noise.

tangent_velocity_at_current_radial_angle:
  The orbit velocity around the selected target, computed from the current target-to-ego radial vector and requested angular speed/direction. This term should remain active during insertion and maintenance; it should not be suppressed merely because the aircraft has not reached a perfect entry point.

radial_correction_velocity:
  A bounded correction along the current radial direction. If the aircraft is too far outside the requested radius, the correction points inward. If the aircraft is too close, the correction points outward. If the aircraft overshoots and begins drifting away, the correction increases on subsequent ticks up to its clamp.

altitude_correction_velocity:
  A bounded vertical correction or safe-height policy output. It should remain separate from horizontal orbit geometry.
```

The controller should tolerate imperfect insertion geometry:

```text
- If the aircraft starts outside the desired radius, it should fly inward while already carrying tangential orbit speed.
- If the aircraft starts inside the desired radius, it should fly outward while already carrying tangential orbit speed.
- If the aircraft starts near the desired radius but at the wrong bearing, it should enter orbit mode from the current angle rather than forcing a fixed 3 o'clock insertion point.
- If the aircraft overshoots, it should continue applying radial correction while preserving tangent motion.
- Once orbit mode is reached, the controller should latch orbit mode and avoid falling back to a brittle entry law due to small radius or position oscillations.
```

A named entry point such as the 3 o'clock point may be used as an operator-friendly approach bias or debug reference, but it must not be the only condition that allows orbit mode. Live flight should be robust to position and velocity errors; the desired behavior is a circular trajectory around the target, not a waypoint hit.

## Validation Expectations

Circle validation should prove the control-law terms and phase behavior, not merely mission lifecycle success.

Mission events should expose enough debug fields to verify:

```text
circle_phase
orbit_mode_latched
orbit_radius_m
actual_radius_m
radius_error_m
radial_correction_mps
tangent_velocity_mps
target_velocity_mps
desired_velocity_mps
orbit_angle_rad
circle_completed_orbits
raw_vx/raw_vy/vx/vy
```

A successful live validation should show:

```text
- target selection remains stable
- target velocity is appropriate for the target type, including zero for static existing-object bindings when configured
- tangent velocity remains active during insertion and circling
- radial correction changes sign correctly when inside versus outside the desired radius
- circle_phase transitions from arriving to circling without requiring an exact fixed entry coordinate
- orbit_mode_latched becomes true and stays true until behavior completion, target loss, or mission reset
- circle_completed_orbits increases while circling
- the mission still completes through GoHome -> Land -> Disarm -> Settled
```

## Boundary

These behavior laws produce bounded kinematic intent only. They do not implement obstacle avoidance. Post-M3 obstacle avoidance belongs after the behavior controller and before the flight command sink:

```text
ObjectBehaviorMissionController
  -> desired velocity vector
  -> TacticalAvoidancePlanner
  -> safe velocity vector
  -> Px4BridgeCommandSink
```

The flight sink remains a transport boundary and should not contain behavior semantics or avoidance policy.
