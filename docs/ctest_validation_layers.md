# CTest Validation Layers

Dedalus uses a first-class test declaration attribute named `LAYER` in `tests/CMakeLists.txt`.

Plain CTest runs tests in the declaration order from `tests/CMakeLists.txt`. The file is intentionally ordered by validation layer:

```text
contracts -> unit -> synthetic -> scenario
```

So the normal command already executes the suite layer-by-layer:

```bash
ctest --test-dir build-staging --output-on-failure
```

Each test is declared through one of the helper functions:

```cmake
dedalus_add_native_test(
    NAME target_selector
    LAYER contracts
    TAGS unit
    SOURCES unit/test_target_selector.cpp
)
```

or:

```cmake
dedalus_add_python_test(
    NAME mission_scenario_runner
    LAYER scenario
    TAGS synthetic integration
    COMMAND_ARGS ...
)
```

The helper derives:

```text
CTest display name:
  target_selector (contracts)

Only CTest LABEL:
  contracts

Custom properties:
  DEDALUS_LAYER=contracts
  DEDALUS_TAGS=unit
```

CTest labels are intentionally layer-only. This keeps the default `Label Time Summary` readable:

```text
contracts = ...
unit      = ...
synthetic = ...
scenario  = ...
```

Layer filters are precise:

```bash
ctest --test-dir build-staging --output-on-failure -L contracts
ctest --test-dir build-staging --output-on-failure -L unit
ctest --test-dir build-staging --output-on-failure -L synthetic
ctest --test-dir build-staging --output-on-failure -L scenario
```

Secondary metadata such as `unit`, `synthetic`, `integration`, or `airsim` can be stored in `DEDALUS_TAGS`, but those tags are not CTest labels.

---

## Layers

### contracts

Contract and protocol-boundary tests. These validate data formats, config parsing, protocol wrappers, provider boundaries, and behavior/spec contracts.

```bash
ctest --test-dir build-staging --output-on-failure -L contracts
```

Examples:

```text
world_snapshot_json (contracts)
behavior_spec (contracts)
target_selector (contracts)
core_stack_config_loader (contracts)
provider_composition (contracts)
ppm_frame_annotation_sink (contracts)
airsim_provider_boundary (contracts)
airsim_velocity_command_sink (contracts)
px4_mavlink_command_sink (contracts)
```

### unit

Deterministic native C++ unit tests. Most use synthetic in-memory state or mocked providers.

```bash
ctest --test-dir build-staging --output-on-failure -L unit
```

Examples:

```text
perception_world_model_flow (unit)
video_only_world_model_flow (unit)
recorded_frame_world_model_flow (unit)
pipeline_profiler (unit)
trajectory_mission_controller (unit)
mission_runtime (unit)
latest_world_snapshot (unit)
```

### synthetic

Tests that exercise synthetic or recorded data, generated mission artifacts, or CI-safe mission loops. These do not require a live AirSim/PX4 session.

```bash
ctest --test-dir build-staging --output-on-failure -L synthetic
```

Examples:

```text
replay_recording_smoke (synthetic)
replay_artifact_validator (synthetic)
ppm_sequence_mp4_export (synthetic)
replay_mission_smoke (synthetic)
mission_artifact_validator (synthetic)
```

### scenario

Archive-grade mission scenario and campaign harness tests. These validate mission artifacts, terminal states, and scenario/campaign reporting.

```bash
ctest --test-dir build-staging --output-on-failure -L scenario
```

Current examples:

```text
mission_scenario_runner (scenario)
mission_campaign_runner (scenario)
mission_abort_scenario (scenario)
```

### AirSim boundary metadata

AirSim-facing boundary tests that remain CI-safe are currently part of the contracts layer.

Current example:

```text
airsim_provider_boundary (contracts)
```

It carries `DEDALUS_TAGS=unit;airsim`, but it is not labeled `airsim` so it does not add a separate CTest label summary row.

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

## Inspecting Layers

List all tests with their display names in execution order:

```bash
ctest --test-dir build-staging -N
```

List tests for a layer:

```bash
ctest --test-dir build-staging -N -L contracts
ctest --test-dir build-staging -N -L unit
ctest --test-dir build-staging -N -L synthetic
ctest --test-dir build-staging -N -L scenario
```

CTest can filter using `LABELS`. `DEDALUS_LAYER` and `DEDALUS_TAGS` are stored for structured inspection tooling, but the standard command-line layer filter remains `-L`.

---

## Recommended Development Flow

For fast local work:

```bash
cmake --build build-staging -j$(nproc)
ctest --test-dir build-staging --output-on-failure -L contracts
ctest --test-dir build-staging --output-on-failure -L unit
```

Before handing off or committing larger behavior/runtime changes, run plain CTest. It already executes in layer order:

```bash
ctest --test-dir build-staging --output-on-failure
```

For mission runtime, scenario, or artifact-validator changes:

```bash
ctest --test-dir build-staging --output-on-failure -L scenario
```

For AirSim/PX4 live-control changes, run the full suite first and then run live smoke manually with AirSim/PX4 already running.
