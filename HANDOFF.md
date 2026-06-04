# Dedalus Handoff Template

This file is a template for generating LLM handoff prompts.

To generate a current handoff, read `LLM.md` and the current repo state, then fill in the template below.

**Usage trigger:** `use HANDOFF.md to generate a handoff prompt from our current state`

---

## How to Generate a Handoff

1. Read `LLM.md` first. Treat it as the active operating brief.
2. Read `docs/runtime_dataflow.md` for the current source -> publisher -> server -> subscriber -> sink architecture.
3. Read `docs/object_conditioned_behavior_plan.md` before object-conditioned behavior work.
4. Read `docs/airsim_existing_object_ghost_runbook.md` before AirSim existing-object validation.
5. Read `docs/mission_scenario_runner.md` for the scenario/campaign harness workflow.
6. Read `docs/world_model_reprojection_validation_plan.md` before reprojection, annotation, or world-model evidence work.
7. Read `WHITEPAPER.md` when architectural rationale is needed.
8. For classless geometric occupancy / avoidance / volume detection work, read these together before coding:
   - `docs/classless_geometric_occupancy_and_avoidance_plan.md`
   - `docs/reflexive_obstacle_avoidance_architecture.md`
   - `docs/geometric_volume_detection_and_spatial_mapping_plan.md`
9. Read `docs/selected_entity_slow_moving_animal_validation.md` and `docs/moving_target_behavior_validation_results.md` when continuing moving-target/object-conditioned behavior work.
10. Read `LLM.back.md` only for historical context when needed; `LLM.md` is authoritative.
11. Use architectural names in handoffs, docs, commands, and validators. Do not invent arbitrary placeholder file or directory names. Use concrete subsystem/scenario names such as `out/object_behavior_airsim_existing_object_circle`, `out/mission_loop_snapshots`, `tools/mission/validate-track4-snapshots.py`, or the exact `--output-dir` printed by the run.
12. Run `git log --oneline -1` to get the current commit SHA.
13. Substitute all `<PLACEHOLDER>` values below with current state.
14. Emit the filled-in prompt as plain text — no surrounding explanation.

---

## Naming and Artifact Convention

```text
Use architectural names, not arbitrary placeholder names.

Prefer names that encode the subsystem, scenario, artifact role, or runtime boundary:
  out/object_behavior_airsim_existing_object_circle
  out/mission_loop_snapshots
  tools/mission/validate-track4-snapshots.py
  tools/mission/validate-mission-artifacts.py
  simulation/airsim/run_mission.sh

Avoid invented generic names such as mission_YYYY, latest_run, foo.json, temp.json, or ad-hoc simulation/artifacts/mission_* unless that is the actual architectural path produced by the repo.

When referring to run artifacts, use the concrete `--output-dir` value or the named output directory printed by `dedalus_mission_loop` / `simulation/airsim/run_mission.sh`.
```

---

## Template

```text
You are continuing work on the Dedalus repo.

Repository:
  guybarnahum/dedalus

Current commit:
  <CURRENT_COMMIT_SHA>

First read:
  LLM.md

Then read:
  docs/runtime_dataflow.md
  docs/object_conditioned_behavior_plan.md
  docs/airsim_existing_object_ghost_runbook.md
  docs/mission_scenario_runner.md
  docs/world_model_reprojection_validation_plan.md

For classless geometric occupancy / avoidance / volume detection work, also read:
  docs/classless_geometric_occupancy_and_avoidance_plan.md
  docs/reflexive_obstacle_avoidance_architecture.md
  docs/geometric_volume_detection_and_spatial_mapping_plan.md

Historical context, only if needed:
  LLM.back.md

Active milestone:
  <ACTIVE_MILESTONE_AND_STAGE>

Current architecture:
  <DATA_FLOW_DIAGRAM — copy from LLM.md Current Runtime Architecture or update if pipeline changed>

Current observed behavior:
  <MOST_RECENT_NOTABLE_LOGS_OR_RUNTIME_OUTPUT>

Current diagnosis:
  <WHAT_IS_VERIFIED, WHAT_IS_STILL_UNVALIDATED, AND WHAT_IS LIKELY MISSING OR BROKEN>

Immediate tasks:
  <NUMBERED_TASK_LIST — copy from LLM.md Active Next Work or update to reflect current state>

Naming convention:
  Use architectural names, not arbitrary placeholder names, for files, directories, artifacts, validators, and commands.
  Use concrete subsystem/scenario names and actual `--output-dir` values printed by the repo.
  Do not refer to non-existent generic paths such as simulation/artifacts/mission_* unless that is the actual path produced by the command being discussed.

Do not:
  - Do not use YOLO/DETR/classifier outputs as a prerequisite for Track 4 obstacle avoidance. Track 4 uses classless geometric/arbitrary-volume evidence.
  - Do not judge a visual/volume detector against global AirSim GT objects outside the configured sensing volume.
  - Do not make AirSim GT global-oracle output mean the same thing as visual detector capability; add/use GT visual-emulation clipping when comparing sources.
  - Do not put obstacle avoidance, map-building policy, or detector semantics inside a flight command sink.
  - Do not duplicate occupancy logic for GT and visual sources; normalize them into the same obstacle evidence / occupancy contract.
  - Do not make `rough_flight_map_builder.cpp` a second perception pipeline; it should consume normalized reflexive occupancy / obstacle evidence.
  - Do not use arbitrary placeholder artifact names when an architectural path or exact `--output-dir` exists.
  <OTHER_DO_NOT_LIST — copy from LLM.md Known Traps and add any session-specific traps>

Patch policy:
  Apply changes directly to main.
  Do not create branches or PRs unless explicitly requested.
  If GitHub connector patching fails, is ambiguous, is blocked, or would require a risky broad rewrite, stop using the connector for that code change.
  When generating manual patches for the user, always provide one unified git diff suitable for git apply.
  Generate an exact manual patch and ask the user to apply it locally.
  Do not keep retrying increasingly complex connector paths after a connector failure.

Validation:
  Always give build/test commands after code patches:

    cmake --build build-staging -j$(nproc)
    ctest --test-dir build-staging --output-on-failure

  For current behavior/runtime changes, also include:

    python3 -m py_compile simulation/airsim/scripts/airsim-world-overlay.py

    ctest --test-dir build-staging --output-on-failure -R \
      'mission_runtime|object_behavior_mission_controller|object_behavior_mission_smoke|core_stack_config_loader|behavior_spec|target_selector|world_snapshot_stream_server|mission_artifact_validator'

  For live AirSim Track 4 validation, prefer `simulation/airsim/run_mission.sh`:

    DEDALUS_AIRSIM_GT_NEARBY_RADIUS_M=80 \
    DEDALUS_AIRSIM_GT_MAX_OBJECTS_PER_FRAME=128 \
    DEDALUS_AIRSIM_GT_STATIC_REFRESH_EVERY_N_FRAMES=10 \
    ./simulation/airsim/run_mission.sh \
      --attach \
      --overlay-debug \
      --source-frame-rate-hz 0 \
      --pipeline-timing \
      --frame-producer-timing \
      --scene-id AirSimNH \
      --refresh-scene-inventory \
      --behavior-duration-s 60 \
      --max-frames 1200 \
      --validation-min-orbits 0.20 \
      --validation-complete-reason duration_elapsed \
      --validation-min-occupied-cells 48

  For Track 4 snapshot evidence validation, use the architectural validator and concrete output directory:

    python3 tools/mission/validate-track4-snapshots.py <OUTPUT_DIR>

  Example output directory:

    out/object_behavior_airsim_existing_object_circle

Expected success:
  <SPECIFIC_LOG_STATE_OR_TEST_RESULT_THAT SIGNALS TASK COMPLETE>
```

---
