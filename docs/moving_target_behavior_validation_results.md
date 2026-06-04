# Milestone 2.30B validation summary

Milestone 2.30B is validated.

Common setup:

```text
source_track_id: ghost_far_animal_001
class: animal
behavior spec: config/behaviors/sequence_approach_circle_far_animal_slow_mixed_mode.yaml
approach: yaw=target camera=target
circle: yaw=trajectory camera=target
```

Validated runs:

```text
medium speed: PASS, target_velocity_mps=0.75, completed_orbits=0.992, avg_abs_radius_error=0.301, max_radius_error_after_latch=2.125
side motion:  PASS, target_velocity_mps=0.50, completed_orbits=0.991, avg_abs_radius_error=0.280, max_radius_error_after_latch=2.166
diagonal:     PASS, target_velocity_mps=0.494975, completed_orbits=0.984, avg_abs_radius_error=0.313, max_radius_error_after_latch=2.381
```

All runs reached:

```text
final_state: Complete
state_path: Idle -> Prepare -> Takeoff -> ExecuteMission -> GoHome -> Land -> Complete
sequence_started_steps: approach,circle
sequence_completed_steps: approach
sequence_step_modes:
  approach: yaw=target camera=target
  circle: yaw=trajectory camera=target
behavior_complete reason: sequence_complete
runtime_stop terminal_settled: True
```

Conclusion: 2.30B shows the moving-center orbit behavior is not direction-specific across +X, +Y, and diagonal target motion.
