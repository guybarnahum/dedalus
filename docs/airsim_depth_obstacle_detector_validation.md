# AirSim Depth Obstacle Detector Validation

This note records the first successful live validation of the classless AirSim depth obstacle detector path.

## Scope

This validation belongs to the 4.x classless obstacle-sensing path. It validates sensor-derived geometric obstacle evidence from AirSim `DepthPlanar`, not AirSim named-object GT.

The validated provider is:

```text
airsim_depth_obstacle_detector
```

The validated output contract is:

```text
WorldSnapshot.obstacle_evidence[]
  source_provider = airsim_depth_obstacle_detector
  source_kind     = depth_provider
```

## Validated dataflow

```text
AirSim DepthPlanar
  -> simulation/airsim/scripts/airsim-stream-frames-binary.py --include-depth
  -> RGB binary frame + JSON sidecar depth grid
  -> AirSimFrameSource FramePacket.depth_frame
  -> CoreStackRunner current obstacle_sensing_volumes
  -> AirSimDepthObstacleDetector
  -> PerceptionPipelineOutput.obstacle_evidence
  -> InMemoryWorldModel
  -> WorldSnapshot.obstacle_evidence
  -> RuntimeEventStreamServer
  -> simulation/airsim/scripts/airsim-world-overlay.py volumetric evidence renderer
```

## Successful live run evidence

Representative mission output directory:

```text
out/object_behavior_airsim_existing_object_circle_depth_probe
```

Representative bridge depth records showed:

```text
include_depth=True
stride=16
size=16x9
valid ~= 118-126 samples per frame
min ~= 0.100m
max ~= 145-155m
```

The mission produced 908 world snapshots. Sensing volume provenance was:

```text
camera_pointing_intent:      906 snapshots
configured_camera_coverage:   2 snapshots
```

The obstacle evidence provider counts were:

```text
airsim_depth_obstacle_detector: 112275
```

The obstacle evidence source-kind counts were:

```text
depth_provider: 112275
```

Depth evidence was present from:

```text
first: snapshot_0001.json
last:  snapshot_0908.json
```

The final snapshots contained only classless depth evidence, for example:

```text
snapshot_0901.json: evidence=51 providers={'airsim_depth_obstacle_detector': 51}
snapshot_0902.json: evidence=53 providers={'airsim_depth_obstacle_detector': 53}
snapshot_0903.json: evidence=47 providers={'airsim_depth_obstacle_detector': 47}
snapshot_0904.json: evidence=46 providers={'airsim_depth_obstacle_detector': 46}
snapshot_0905.json: evidence=49 providers={'airsim_depth_obstacle_detector': 49}
snapshot_0906.json: evidence=51 providers={'airsim_depth_obstacle_detector': 51}
snapshot_0907.json: evidence=53 providers={'airsim_depth_obstacle_detector': 53}
snapshot_0908.json: evidence=51 providers={'airsim_depth_obstacle_detector': 51}
```

The OSD debug log rendered only the depth provider in the sampled tail:

```text
obstacle_evidence render providers={'airsim_depth_obstacle_detector': 96}
...
obstacle_evidence render providers={'airsim_depth_obstacle_detector': 46-53}
```

## Sampling issue and fix

The first depth-sidecar integration double-sampled the frame:

```text
AirSim full depth image
  -> bridge --depth-stride 16
  -> 16x9 sidecar grid
  -> detector pixel_stride 16
  -> only top-left sidecar sample inspected
```

The correct invariant is:

```text
Bridge --depth-stride controls acquisition / transport downsampling.
Detector pixel_stride controls optional detector-side second-pass decimation.
Detector default pixel_stride must be 1 so every received sidecar sample is consumed.
```

Validation after the fix showed the detector producing tens to 96 evidence volumes per rendered frame instead of one stable corner sample.

## Current interpretation

The 4.x classless depth dataflow is validated end-to-end:

```text
AirSim DepthPlanar -> normalized ObstacleEvidence -> WorldSnapshot -> OSD volumetric evidence
```

The validated path no longer depends on AirSim object-GT for obstacle-detector visualization.

## Remaining work

Next bounded work should expose and validate stable detector configuration knobs:

```text
obstacle_sensing.detectors.airsim_depth.enabled
obstacle_sensing.detectors.airsim_depth.depth_stride
obstacle_sensing.detectors.airsim_depth.max_range_m
obstacle_sensing.detectors.airsim_depth.voxel_size_m
obstacle_sensing.detectors.airsim_depth.max_evidence
obstacle_sensing.detectors.airsim_depth.disable_object_gt_fallback
```

The fallback rule for 4.x should be explicit:

```text
When the classless depth detector is enabled for obstacle sensing, empty depth evidence means no observed depth evidence for that frame. Do not silently replace it with AirSim named-object GT boxes.
```

AirSim object-GT remains useful for 3.x object/semantic validation and target behavior, but it should not be used as the 4.x obstacle detector.

## Validation commands

```bash
python3 tools/validation/report-obstacle-detector-dataflow.py "$OUT"

find "$OUT" -type f -name 'snapshot_*.json' | sort | tail -10 | while read -r f; do
  jq -r '
    [
      input_filename,
      ((.obstacle_sensing_volumes // []) | length),
      ((.obstacle_evidence // []) | length),
      ((.obstacle_evidence // []) | map(.source_provider) | unique | join(",")),
      ((.obstacle_evidence // []) | map(select(.source_provider=="airsim_depth_obstacle_detector")) | length)
    ] | @tsv
  ' "$f"
done

grep -RIn 'obstacle_evidence render providers' simulation/airsim/logs | tail -40
```
