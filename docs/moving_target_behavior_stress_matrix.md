# Milestone 2.30B — Moving-target stress matrix

Milestone 2.30B expands the validated slow moving animal case into a deterministic moving-target stress matrix before using native moving AirSim actors.

All cases keep the same selected entity:

```text
source_track_id: ghost_far_animal_001
class: animal
behavior spec: config/behaviors/sequence_approach_circle_far_animal_slow_mixed_mode.yaml
sequence:
  approach: yaw=target     camera=target
  circle:   yaw=trajectory camera=target
```

The matrix:

```text
slow      config/ci/core_stack_object_behavior_airsim_far_animal_slow_mixed_mode.yaml
medium    config/ci/core_stack_object_behavior_airsim_far_animal_medium_mixed_mode.yaml
lateral   config/ci/core_stack_object_behavior_airsim_far_animal_lateral_mixed_mode.yaml
diagonal  config/ci/core_stack_object_behavior_airsim_far_animal_diagonal_mixed_mode.yaml
```

Validation command pattern:

```bash
cd ~/dedalus/simulation/airsim

./run_mission.sh \
  --config ../../config/<CONFIG>.yaml \
  --output-dir ../../out/<OUTPUT> \
  --expect-sequence \
  --expect-sequence-steps approach,circle \
  --expect-sequence-step-modes approach:target:target,circle:trajectory:target \
  --validation-complete-reason sequence_complete \
  --attach
```

Expected evidence for every matrix case:

```text
validation: PASS
final_state: Complete
state_path: Idle -> Prepare -> Takeoff -> ExecuteMission -> GoHome -> Land -> Complete
sequence_started_steps: approach,circle
sequence_step_modes:
  approach: yaw=target camera=target
  circle: yaw=trajectory camera=target
behavior_complete reason: sequence_complete
runtime_stop terminal_settled: True
```

Expected target velocity evidence:

```text
slow:      target_velocity_mps ~= 0.20
medium:    target_velocity_mps ~= 0.75
lateral:   target_velocity_mps ~= 0.50
diagonal:  target_velocity_mps ~= 0.495
```

Suggested run order:

```bash
./run_mission.sh \
  --config ../../config/ci/core_stack_object_behavior_airsim_far_animal_medium_mixed_mode.yaml \
  --output-dir ../../out/object_behavior_airsim_far_animal_medium_mixed_mode \
  --expect-sequence \
  --expect-sequence-steps approach,circle \
  --expect-sequence-step-modes approach:target:target,circle:trajectory:target \
  --validation-complete-reason sequence_complete \
  --attach

./run_mission.sh \
  --config ../../config/ci/core_stack_object_behavior_airsim_far_animal_lateral_mixed_mode.yaml \
  --output-dir ../../out/object_behavior_airsim_far_animal_lateral_mixed_mode \
  --expect-sequence \
  --expect-sequence-steps approach,circle \
  --expect-sequence-step-modes approach:target:target,circle:trajectory:target \
  --validation-complete-reason sequence_complete \
  --attach

./run_mission.sh \
  --config ../../config/ci/core_stack_object_behavior_airsim_far_animal_diagonal_mixed_mode.yaml \
  --output-dir ../../out/object_behavior_airsim_far_animal_diagonal_mixed_mode \
  --expect-sequence \
  --expect-sequence-steps approach,circle \
  --expect-sequence-step-modes approach:target:target,circle:trajectory:target \
  --validation-complete-reason sequence_complete \
  --attach
```
