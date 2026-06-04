# Milestone 2.31A — Per-step smooth altitude profiles

Milestone 2.31A adds optional per-step drone altitude profiles to behavior specs.

The feature is explicitly opt-in. Existing behaviors that omit `altitude_profile` keep the previous `altitude_offset_m` behavior.

Schema:

```yaml
altitude_profile:
  start_height_m: 28.0
  end_height_m: 16.0
  duration_s: 10.0
  easing: smoothstep
```

Short hold form:

```yaml
altitude_profile:
  hold_height_m: 16.0
```

Semantics:

```text
start_height_m -> end_height_m is an absolute drone height profile in meters.
duration_s controls the interpolation window. If omitted, the behavior duration is used, otherwise a default transition duration is used.
easing supports smoothstep and linear.
The profile resets at each sequence step start.
Altitude motion is independent from XY approach/orbit geometry, yaw policy, and camera pointing policy.
```

Compatibility:

```text
If altitude_profile is missing:
  preserve existing altitude_offset_m behavior.

If altitude_profile is present:
  compute desired_height_m from the step-local profile and command bounded vertical velocity toward that height.
```

Important runtime note:

```text
The existing object_behavior_altitude_policy=safe_height_floor still applies after behavior velocity generation.
For a lower circle, choose an end_height_m at or above the configured safe_height_m, or lower the safe height intentionally for that validation run.
```

Initial example behavior spec:

```text
config/behaviors/sequence_approach_circle_far_animal_slow_altitude_profile.yaml
```

Validation observability appears in behavior_tick_sample / behavior_debug:

```text
altitude_profile_active
altitude_profile_easing
altitude_profile_t
desired_height_m
current_height_m
height_error_m
```
