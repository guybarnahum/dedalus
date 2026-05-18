# CTest Validation Layers

Dedalus uses CTest labels to separate fast contracts, deterministic unit/synthetic tests, scenario harness tests, and AirSim-facing boundaries.

Plain CTest still runs the full suite:

```bash
ctest --test-dir build-staging --output-on-failure
```

Layered runs use `-L`.

---

## Layers

### contracts

Contract and protocol-boundary tests. These validate data formats, config parsing, protocol wrappers, provider boundaries, and behavior/spec contracts.

```bash
ctest --test-dir build-staging --output-on-failure -L contracts
```

Examples:

```text
world_snapshot_json
behavior_spec
target_selector
core_stack_config_loader
provider_composition
ppm_frame_annotation_sink
airsim_provider_boundary
airsim_velocity_command_sink
px4_mavlink_command_sink
```

### unit

Deterministic native C++ unit tests. Most use synthetic in-memory state or mocked providers.

```bash
ctest --test-dir build-staging --output-on-failure -L unit
```

### synthetic

Tests that exercise synthetic or recorded data, generated mission artifacts, or CI-safe mission loops. These do not require a live AirSim/PX4 session.

```bash
ctest --test-dir build-staging --output-on-failure -L synthetic
```

Examples:

```text
perception_world_model_flow
video_only_world_model_flow
recorded_frame_world_model_flow
replay_recording_smoke
replay_artifact_validator
replay_mission_smoke
mission_artifact_validator
mission_scenario_runner
mission_campaign_runner
mission_abort_scenario
```

### scenario

Archive-grade mission scenario and campaign harness tests. These validate mission artifacts, terminal states, and scenario/campaign reporting.

```bash
ctest --test-dir build-staging --output-on-failure -L scenario
```

Current examples:

```text
mission_scenario_runner
mission_campaign_runner
mission_abort_scenario
```

### airsim

AirSim-facing boundary tests that remain CI-safe. These do not require a live simulator unless explicitly documented later.

```bash
ctest --test-dir build-staging --output-on-failure -L airsim
```

Current example:

```text
airsim_provider_boundary
```

### airsim_live

No normal CTest test is labeled `airsim_live` today. Real live AirSim/PX4 validation remains an explicit operator-run smoke/campaign flow outside normal CTest.

Use one of:

```bash
RUNS=3 simulation/repeat-mission-smoke.sh
```

or:

```bash
python3 simulation/run-mission-campaign.py \
  --campaign-file config/mission_campaigns/airsim_live_smoke.json \
  --campaign-id live_<N> \
  --output-root out/mission_campaigns \
  --progress \
  --overwrite
```

---

## Recommended Development Flow

For fast local work:

```bash
cmake --build build-staging -j$(nproc)
ctest --test-dir build-staging --output-on-failure -L contracts
ctest --test-dir build-staging --output-on-failure -L unit
```

Before handing off or committing larger behavior/runtime changes:

```bash
ctest --test-dir build-staging --output-on-failure
```

For mission runtime, scenario, or artifact-validator changes:

```bash
ctest --test-dir build-staging --output-on-failure -L scenario
```

For AirSim/PX4 live-control changes, run the full suite first and then run live smoke manually with AirSim/PX4 already running.
