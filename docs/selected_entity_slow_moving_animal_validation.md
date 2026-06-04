# Milestone 2.30A — Slow moving SEL animal sequence validation

Milestone 2.30A validates the first moving selected entity path after the static mixed-mode sequence proof.

The validation target is a deterministic far animal ghost:

```text
source_track_id: ghost_far_animal_001
class: animal
initial_position_local_m: [58.0, -22.0, 0.0]
trajectory: +X at 0.20 m/s
```

Primary config:

```text
config/core_stack_object_behavior_airsim_far_animal_slow_mixed_mode.yml
```

Primary behavior spec:

```text
config/behaviors/sequence_approach_circle_far_animal_slow_mixed_mode.yaml
```

The sequence remains the mixed-mode proof shape:

```text
approach: yaw=target     camera=target
circle:   yaw=trajectory camera=target
```

Moving-target requirements:

```text
mission_options.object_behavior_zero_target_velocity: false
Observation3D.velocity_local must propagate into AgentState.velocity_local
target_velocity_mps should be approximately 0.20 in behavior events
```

Primary AirSim validation:

```bash
cd ~/dedalus/simulation/airsim

./run_mission.sh \
  --config ../../config/core_stack_object_behavior_airsim_far_animal_slow_mixed_mode.yml \
  --output-dir ../../out/object_behavior_airsim_far_animal_slow_mixed_mode \
  --expect-sequence \
  --expect-sequence-steps approach,circle \
  --expect-sequence-step-modes approach:target:target,circle:trajectory:target \
  --validation-complete-reason sequence_complete \
  --attach
```

Expected evidence:

```text
validation: PASS
final_state: Complete
state_path: Idle -> Prepare -> Takeoff -> ExecuteMission -> GoHome -> Land -> Complete
sequence_started_steps: approach,circle
sequence_completed_steps: approach
sequence_step_modes:
  approach: yaw=target camera=target
  circle: yaw=trajectory camera=target
behavior_complete reason: sequence_complete
target_velocity_mps ~= 0.20
runtime_stop terminal_settled: True
```

Static far-person mixed-mode regression:

```bash
cd ~/dedalus/simulation/airsim

./run_mission.sh \
  --config ../../config/core_stack_object_behavior_airsim_existing_object_sequence_far_person_mixed_mode.yml \
  --output-dir ../../out/object_behavior_airsim_existing_object_sequence_far_person_mixed_mode_regression \
  --expect-sequence \
  --expect-sequence-steps approach,circle \
  --expect-sequence-step-modes approach:target:target,circle:trajectory:target \
  --validation-complete-reason sequence_complete \
  --attach
```

Next slice after 2.30A:

```text
Milestone 2.30B — moving-target stress matrix.

Test deterministic ghost trajectories before native AirSim moving actors:
- slow animal: 0.20 m/s
- medium animal: 0.75 m/s
- crossing target: lateral motion across the drone orbit
- diagonal target: vx + vy
- start-inside-radius case
- start-outside-radius case
```
