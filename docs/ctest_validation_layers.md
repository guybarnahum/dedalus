# CTest Validation Layers

Dedalus uses a first-class test declaration attribute named `LAYER` in `tests/CMakeLists.txt`.

Each test is declared through one of the helper functions:

```cmake
dedalus_add_native_test(
    NAME target_selector
    LAYER contracts
    LABELS unit
    SOURCES unit/test_target_selector.cpp
)
```

or:

```cmake
dedalus_add_python_test(
    NAME mission_scenario_runner
    LAYER scenario
    LABELS synthetic integration
    COMMAND_ARGS ...
)
```

The helper derives three things from the layer:

```text
CTest display name:
  target_selector (contracts)

CTest LABELS:
  contracts;unit

CTest custom property:
  DEDALUS_LAYER=contracts
```

Plain CTest still runs the full suite:

```bash
ctest --test-dir build-staging --output-on-failure
```

Layered runs use `-L`:

```bash
ctest --test-dir build-staging --output-on-failure -L contracts
```

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

### synthetic

Tests that exercise synthetic or recorded data, generated mission artifacts, or CI-safe mission loops. These do not require a live AirSim/PX4 session.

```bash
ctest --test-dir build-staging --output-on-failure -L synthetic
```

Examples:

```text
perception_world_model_flow (unit)
video_only_world_model_flow (unit)
recorded_frame_world_model_flow (unit)
replay_recording_smoke (synthetic)
replay_artifact_validator (synthetic)
replay_mission_smoke (synthetic)
mission_artifact_validator (synthetic)
mission_scenario_runner (scenario)
mission_campaign_runner (scenario)
mission_abort_scenario (scenario)
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

### airsim

AirSim-facing boundary tests that remain CI-safe. These do not require a live simulator unless explicitly documented later.

```bash
ctest --test-dir build-staging --output-on-failure -L airsim
```

Current example:

```text
airsim_provider_boundary (contracts)
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

## Inspecting Layers

List all tests with their display names:

```bash
ctest --test-dir build-staging -N
```

List tests for a layer:

```bash
ctest --test-dir build-staging -N -L contracts
ctest --test-dir build-staging -N -L scenario
```

CTest can filter using `LABELS`. The `DEDALUS_LAYER` property is stored for structured inspection tooling, but the standard command-line filter remains `-L`.

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
