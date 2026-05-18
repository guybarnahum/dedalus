# Mission Scenario and Campaign Runners

This document captures the current Milestone 2.22 scenario/campaign harness.

The goal is to make mission runs archive-grade by wrapping `dedalus_mission_loop`, preserving console output, validating run artifact directories, and writing structured per-run and campaign metadata.

## Current status

Milestone 2.22 now provides:

```text
- single scenario runner
- mission artifact validation for Complete and Abort final states
- synthetic lifecycle scenario
- synthetic expected-Abort scenario
- campaign runner
- JSON campaign specification files
- dry-run planning mode
- Markdown campaign report
- live AirSim campaign preset
- progress-aware wrapper output
- Ctrl-C semantics for graceful active-mission shutdown and campaign stop
```

## Single scenario runner

A scenario run invokes one `dedalus_mission_loop`, writes artifacts, then validates them.

Synthetic Complete lifecycle example:

```bash
python3 simulation/run-mission-scenario.py \
  --name synthetic_lifecycle \
  --run-id run_0001 \
  --config config/core_stack_synthetic_mission_ci.yaml \
  --app ./build-staging/apps/dedalus_mission_loop \
  --output-root out/mission_scenarios \
  --max-frames 220 \
  --shutdown-max-frames 50 \
  --safe-height-m 2 \
  --landed-height-m 1 \
  --expect-final-state Complete \
  --overwrite
```

Synthetic expected-Abort lifecycle example:

```bash
python3 simulation/run-mission-scenario.py \
  --name synthetic_abort_land_timeout \
  --run-id run_0001 \
  --config config/core_stack_synthetic_mission_abort_ci.yaml \
  --app ./build-staging/apps/dedalus_mission_loop \
  --output-root out/mission_scenarios \
  --max-frames 220 \
  --shutdown-max-frames 50 \
  --safe-height-m 2 \
  --landed-height-m 1 \
  --expect-final-state Abort \
  --overwrite
```

The resulting directory is:

```text
out/mission_scenarios/<scenario>/<run_id>/
  console.log
  metadata.json
  validator_result.txt
  mission_events.jsonl
  snapshot_manifest.txt
  snapshot_XXXX.json
```

`console.log` captures the mission process output. `metadata.json` records the scenario identity, commands, return codes, validation settings, timestamps, and artifact filenames. `validator_result.txt` captures the post-run artifact validator output.

## Artifact validator

The scenario runner delegates validation to:

```bash
python3 simulation/validate-mission-artifacts.py <run-dir> \
  --expect-final-state Complete \
  --safe-height-m <safe-height> \
  --landed-height-m <landed-height>
```

`--expect-complete` remains shorthand for `--expect-final-state Complete`.

Expected-Abort validation uses:

```bash
python3 simulation/validate-mission-artifacts.py <run-dir> \
  --expect-final-state Abort \
  --safe-height-m <safe-height> \
  --landed-height-m <landed-height>
```

For future M3 object-conditioned behavior scenarios, add:

```bash
--expect-behavior
```

## Campaign runner

A campaign expands one or more scenarios into repeated scenario runs. The campaign runner delegates each real run to `simulation/run-mission-scenario.py`; this keeps the per-run artifact contract centralized.

CLI-driven single-scenario campaign example:

```bash
python3 simulation/run-mission-campaign.py \
  --campaign synthetic_single \
  --campaign-id campaign_0001 \
  --scenario synthetic_lifecycle \
  --repeats 3 \
  --config config/core_stack_synthetic_mission_ci.yaml \
  --app ./build-staging/apps/dedalus_mission_loop \
  --output-root out/mission_campaigns \
  --max-frames 220 \
  --shutdown-max-frames 50 \
  --safe-height-m 2 \
  --landed-height-m 1 \
  --expect-final-state Complete \
  --overwrite
```

Campaign spec example:

```bash
python3 simulation/run-mission-campaign.py \
  --campaign-file config/mission_campaigns/synthetic_ci.json \
  --campaign-id campaign_0001 \
  --app ./build-staging/apps/dedalus_mission_loop \
  --output-root out/mission_campaigns \
  --overwrite
```

The resulting directory is:

```text
out/mission_campaigns/<campaign>/<campaign_id>/
  campaign_summary.json
  campaign_summary.txt
  campaign_report.md
  runs/
    <scenario>/
      run_0001/
        console.log
        metadata.json
        validator_result.txt
        mission_events.jsonl
        snapshot_manifest.txt
        snapshot_XXXX.json
```

## Campaign specification files

Synthetic mixed CI campaign:

```text
config/mission_campaigns/synthetic_ci.json
```

This includes:

```text
synthetic_lifecycle x2              expected Complete
synthetic_abort_land_timeout x1     expected Abort
```

Live AirSim campaign preset:

```text
config/mission_campaigns/airsim_live_smoke.json
```

This includes:

```text
trajectory_px4_bridge_live x3       expected Complete
config: config/core_stack_trajectory_mission_placeholder.yaml
safe_height_m: 16
landed_height_m: 1
```

## Dry-run planning

Use dry-run to validate campaign parsing, repeat expansion, expected final states, output layout, planned command lines, and report generation without launching `dedalus_mission_loop`.

```bash
python3 simulation/run-mission-campaign.py \
  --campaign-file config/mission_campaigns/airsim_live_smoke.json \
  --campaign-id dry_run_0001 \
  --output-root out/mission_campaigns \
  --dry-run \
  --overwrite
```

Dry-run writes:

```text
campaign_summary.json
campaign_summary.txt
campaign_report.md
```

The status is `planned`, and run records include the planned scenario command.

## Live AirSim campaign

Assuming AirSim/PX4 is already running:

```bash
python3 simulation/run-mission-campaign.py \
  --campaign-file config/mission_campaigns/airsim_live_smoke.json \
  --campaign-id live_0001 \
  --output-root out/mission_campaigns \
  --progress \
  --overwrite
```

Expected successful lifecycle for each run:

```text
Prepare -> Takeoff -> ExecuteMission -> GoHome -> Land -> Complete
```

The live campaign uses the current working PX4 bridge path. Do not switch it to the native experimental C++ MAVLink sink.

## Ctrl-C behavior

The wrappers intentionally preserve the mission loop's safety behavior:

```text
First Ctrl-C during a campaign:
  - campaign runner forwards interrupt to the active scenario
  - scenario runner forwards interrupt to dedalus_mission_loop
  - active mission performs graceful finish through GoHome -> Land -> Disarm
  - campaign stops after the active scenario finishes
  - campaign_summary status=interrupted
  - process exits 130

Second Ctrl-C:
  - force-terminates the active child process
```

The campaign should not start the next repeat after an operator-requested interrupt.

## Reports

Campaign output includes:

```text
campaign_summary.json     machine-readable summary, schema_version=4
campaign_summary.txt      compact terminal summary
campaign_report.md        human-readable Markdown report with per-run artifact links
```

`campaign_report.md` includes:

```text
- campaign status
- scenario table
- run table
- expected final state per run
- metadata / events / validator / console artifact links
```

## CI smoke tests

The CTest smoke tests do not require AirSim.

```bash
ctest --test-dir build-staging --output-on-failure -R 'mission_(scenario|campaign|abort)_'
```

Current intent:

```text
mission_scenario_runner   real synthetic Complete lifecycle
mission_abort_scenario    real synthetic expected-Abort lifecycle
mission_campaign_runner   dry-run planning/reporting only
```

The campaign runner test is intentionally dry-run only. It should not duplicate the slower mission-loop lifecycle coverage already provided by the scenario and abort tests.

Full validation:

```bash
cmake --build build-staging -j$(nproc)
ctest --test-dir build-staging --output-on-failure
```

Expected current suite size after Milestone 2.22:

```text
22 tests
```

## Next slice

Milestone 2.23 should start the behavior-spec parser foundation for M3 object-conditioned behavior.
