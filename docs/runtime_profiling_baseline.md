# Runtime Profiling Baseline

This is a measurement-only stage. It adds profiling spans and summary tooling so
runtime optimization work can be driven by measured costs instead of guessing.

## Scope

This stage does **not** change mission behavior, obstacle semantics, controller logic,
or AirSim launch defaults.

It measures:

- frame-source wait and binary frame detail timing
- ego-provider timing
- perception pipeline construction and processing
- AirSim ground-truth / ghost-target frame generation
- ghost-detection publication
- merge cost for GT observations
- world-model ego/update/ingest/snapshot timing
- snapshot publisher/subscriber fan-out timing
- frame annotation timing
- artifact snapshot enqueue/write/manifest timing counters
- runtime event stream message counts and serialization/enqueue/publish timing counters
- mission event publish/log/flush timing counters

## Run a profiling mission

Create an effective config with pipeline timing enabled:

```bash
EFF0_CONFIG="/tmp/dedalus_eff0_airsim_profile.yaml"
cp config/ci/core_stack_object_behavior_airsim_existing_object_circle.yaml "$EFF0_CONFIG"
cat >> "$EFF0_CONFIG" <<'EOF'
pipeline_timing_enabled: true
pipeline_timing_output_path: out/profile/eff0_airsim_pipeline_profile.jsonl
EOF
```

Then run the normal AirSim mission flow with the effective config:

```bash
./simulation/airsim/run_mission.sh \
  --attach \
  --overlay-debug \
  --config "$EFF0_CONFIG" \
  --scene-id AirSimNH \
  --refresh-scene-inventory \
  --behavior-duration-s 60 \
  --max-frames 1200 \
  --validation-min-orbits 0.20 \
  --validation-complete-reason duration_elapsed
```

## Summarize the pipeline profile

```bash
python3 tools/mission/summarize-pipeline-profile.py \
  out/profile/eff0_airsim_pipeline_profile.jsonl \
  --top 40
```

Look first at these stages:

- `frame_source.next_frame_wait`
- `frame_source.detail.read_header`
- `frame_source.detail.read_payload`
- `frame_source.detail.read_sidecar`
- `frame_source.detail.parse_sidecar`
- `perception_pipeline.construct`
- `perception_pipeline.process`
- `ghost_targets.frame_at`
- `ghost_detections.publish`
- `perception_output.merge_ghost_observations`
- `world_model.ingest`
- `world_model.snapshot`
- `snapshot_publisher.publish`
- `frame_annotator.annotate`

## Validate the run

```bash
python3 tools/mission/validate-mission-artifacts.py \
  out/object_behavior_airsim_existing_object_circle \
  --expect-complete \
  --expect-behavior \
  --expect-camera-pointing \
  --expect-camera-modes neutral,target,home,landing_area \
  --camera-frames-dir out/object_behavior_airsim_existing_object_circle/camera_pointing_frames \
  --expect-camera-proof-frames \
  --expect-scene-inventory out/airsim_scene_inventory/AirSimNH.objects.json \
  --expect-occupancy \
  --expect-occupancy-source airsim_ground_truth \
  --expect-min-occupied-cells 100 \
  --expect-source-object-prefix gt_tree_ \
  --expect-source-object-prefix gt_wall_ \
  --expect-source-object-prefix gt_fence_ \
  --expect-source-object-prefix gt_cable_ \
  --expect-swept-volume \
  --safe-height-m 16 \
  --landed-height-m 1.0
```

## Interpreting the baseline

The primary bottleneck candidates are:

1. `ghost_targets.frame_at` — expensive if the AirSim object-pose bridge still
   dominates per-frame GT refresh.
2. `snapshot_publisher.publish` — includes synchronous subscriber fan-out such as
   latest snapshot update, artifact enqueue, and runtime stream serialization.
3. `world_model.snapshot` — full snapshot copy/materialization.
4. `frame_annotator.annotate` — PPM/sidecar/overlay artifact path.
5. `frame_source.next_frame_wait` and detail stages — AirSim frame bridge and
   image payload cost.

EFF-1 should only start after this baseline is green and archived.
