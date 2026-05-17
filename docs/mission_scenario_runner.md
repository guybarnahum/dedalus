# Mission Scenario and Campaign Runners

This document captures Milestone 2.22 scenario/campaign harness work.

The goal is to make mission runs archive-grade by wrapping `dedalus_mission_loop`, preserving console output, validating run artifact directories, and writing structured metadata and campaign summaries.

## Single scenario runner

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

## Campaign runner

```bash
python3 simulation/run-mission-campaign.py \
  --campaign synthetic_ci \
  --campaign-id campaign_0001 \
  --scenario synthetic_lifecycle \
  --repeats 3 \
  --config config/core_stack_synthetic_mission_ci.yaml \
  --output-root out/mission_campaigns \
  --max-frames 220 \
  --shutdown-max-frames 50 \
  --safe-height-m 2 \
  --landed-height-m 1 \
  --overwrite
```

The resulting directory is:

```text
out/mission_campaigns/synthetic_ci/campaign_0001/
  campaign_summary.json
  campaign_summary.txt
  runs/
    synthetic_lifecycle/
      run_0001/
        console.log
        metadata.json
        validator_result.txt
        mission_events.jsonl
        snapshot_manifest.txt
        snapshot_XXXX.json
      run_0002/
        ...
```

The campaign runner delegates each run to `simulation/run-mission-scenario.py`. This keeps the per-run artifact contract in one place and makes campaign summary generation a thin aggregation layer.

## Per-run metadata

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

This file is the per-run anchor used by campaign summaries.

## Campaign summary

`campaign_summary.json` and `campaign_summary.txt` record:

```text
- campaign name and id
- scenario name
- repeat count
- passed / failed counts
- per-run status and return codes
- per-run artifact directories
```

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

## CI smoke tests

The CTest smoke tests use the synthetic mission config so they do not require AirSim:

```bash
ctest --test-dir build-staging --output-on-failure -R 'mission_(scenario|campaign)_runner'
```

The scenario test proves that one run creates the expected archive-grade directory and validates mission lifecycle completion.

The campaign test proves that repeated scenario runs produce per-run metadata and campaign-level summaries.

## Next slice

Milestone 2.22.4 should add first-class abort/edge-case scenario validation, including validator support for expected final states beyond `Complete`.
