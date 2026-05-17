# Mission Scenario Runner

This document captures Milestone 2.22.1: the first scenario/campaign harness slice.

The goal is to make a single mission run archive-grade by wrapping `dedalus_mission_loop`, preserving console output, validating the run artifact directory, and writing structured metadata.

## Runner

```bash
python3 simulation/run-mission-scenario.py \
  --name trajectory_live \
  --run-id run_0001 \
  --config config/core_stack_trajectory_mission_placeholder.yaml \
  --output-root out/mission_scenarios \
  --max-frames 900 \
  --shutdown-max-frames 400 \
  --safe-height-m 16 \
  --landed-height-m 1 \
  --overwrite
```

The resulting directory is:

```text
out/mission_scenarios/trajectory_live/run_0001/
  console.log
  metadata.json
  validator_result.txt
  mission_events.jsonl
  snapshot_manifest.txt
  snapshot_XXXX.json
```

## Metadata

`metadata.json` records:

```text
- scenario name and run id
- start / finish timestamps
- elapsed time
- status: passed or failed
- config path
- mission command
- validator command
- mission return code
- validator return code
- artifact filenames
```

This file is intended to become the per-run anchor for later campaign summaries.

## Validation

By default the scenario runner invokes:

```bash
python3 simulation/validate-mission-artifacts.py <run-dir> \
  --expect-complete \
  --safe-height-m <safe-height> \
  --landed-height-m <landed-height>
```

For future M3 object-conditioned behavior scenarios, add:

```bash
--expect-behavior
```

## CI smoke test

The CTest smoke test uses the synthetic mission config so it does not require AirSim:

```bash
ctest --test-dir build-staging --output-on-failure -R mission_scenario_runner
```

It proves that the runner creates the expected run directory, captures logs, writes metadata, and gates the result through the mission artifact validator.

## Next slice

Milestone 2.22.2 should add a campaign-level wrapper that runs several scenario definitions/repeats and writes a campaign summary, rather than replacing this single-run primitive.
