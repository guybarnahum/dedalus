# Pipeline Timing

Milestone 2.9 adds a dependency-free timing harness for the C++ core-stack path.

The goal is to measure coarse runtime stage cost before adding performance-oriented transports, media export, or heavier perception providers. The profiler is intentionally simple and writes newline-delimited JSON so it can run in CI-safe paths and AirSim validation runs without external dependencies.

## Config

Timing is off by default.

Enable it with flat core-stack config keys:

```yaml
pipeline_timing_enabled: true
pipeline_timing_output_path: out/profile/pipeline_profile.jsonl
```

The CI-safe example config is:

```text
config/core_stack_profile_ci.yaml
```

## Output format

The profiler writes one JSON object per processed frame:

```json
{"frame_id":"frame_0001","timestamp_ns":123456789,"total_us":120,"stages":{"frame_source.next_frame":4,"ego_provider.estimate":2,"perception_pipeline.process":20,"world_model.update_ego":1,"world_model.ingest":30,"world_model.snapshot":12,"frame_annotator.annotate":3}}
```

The `total_us` value is the sum of recorded stage durations for that frame. It is a per-stage timing summary, not a wall-clock guarantee for the full app loop because app-level snapshot JSON writes are currently outside `CoreStackRunner`.

## Current stages

Current coarse stages are:

```text
frame_source.next_frame
ego_provider.estimate
perception_pipeline.process
world_model.update_ego
world_model.update_appearance   # only emitted when the frame has appearance metadata
world_model.ingest
world_model.snapshot
frame_annotator.annotate
```

## Synthetic validation

```bash
./build-staging/apps/dedalus_replay_recording \
  --config config/core_stack_profile_ci.yaml \
  --output-dir out/profile_snapshots \
  --max-frames 1

cat out/profile/pipeline_profile.jsonl
```

## Recorded-frame validation

Create or edit a recorded-frame config to include:

```yaml
pipeline_timing_enabled: true
pipeline_timing_output_path: out/recorded_ppm_validation/pipeline_profile.jsonl
```

Then run the recorded-frame replay or validation script.

## AirSim validation

For AirSim binary RGB validation, add these keys to the generated runtime config or validation config:

```yaml
pipeline_timing_enabled: true
pipeline_timing_output_path: out/airsim_binary_ppm_validation/pipeline_profile.jsonl
```

Then run:

```bash
SKIP_BUILD=1 SKIP_CTEST=1 ./scripts/validate-airsim-binary-ppm-annotation.sh
```

The timing output can be inspected with:

```bash
python3 - <<'PY'
import json
from pathlib import Path

path = Path('out/airsim_binary_ppm_validation/pipeline_profile.jsonl')
rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
stages = {}
for row in rows:
    for name, value in row['stages'].items():
        stages.setdefault(name, []).append(value)

for name, values in sorted(stages.items()):
    print(f'{name}: count={len(values)} mean_us={sum(values)/len(values):.1f} max_us={max(values)}')
PY
```

## Interpretation

For current Milestone 2 providers, the likely expensive stages are:

```text
frame_source.next_frame       # AirSim bridge / pipe / binary read
frame_annotator.annotate      # PPM overlay + disk write
world_model.snapshot          # snapshot copy cost
```

Snapshot JSON file writing in `dedalus_replay_recording` is not included yet. If JSON serialization or disk output becomes suspicious, add app-level timing as a follow-up rather than overloading the core runner contract.

Do not add OpenCV, FFmpeg, GStreamer, Tracy, perfetto, or platform-specific profilers to the core timing path. Rich profiling can be added later as optional tooling.
