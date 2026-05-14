# AirSim Image Extraction Profiling

Milestone 2.17 tracks the difference between simulator bridge latency and production camera latency.

## Current finding

The live AirSim frame-attached ego path is not currently dominated by pipe transfer.

At 1920x1080, measured `frame_ego` runs showed approximately:

```text
sim_get_images_ms p95 ~= 60-75 ms
stdout_write_ms   p95 ~= 5-6 ms
```

At 1280x720, after manually changing the launch-directory `simulation/settings.json`, the measured `frame_ego` path showed approximately:

```text
actual_resolution_counts: 1280x720
sim_get_images_ms p95 ~= 51-54 ms
stdout_write_ms   p95 ~= 2.5-3 ms
```

This suggests the current Milestone 2 AirSim path is dominated by AirSim image extraction / RPC / render readback behavior. Payload size still affects pipe/write cost, but `simGetImages` remains the dominant stage.

## Packaged AirSimNH settings behavior

For this packaged Colosseum/AirSimNH runtime, Scene capture resolution has been observed to follow the launch-directory `simulation/settings.json` file rather than the generated runtime file passed through `-settings=/tmp/...`.

The discriminating test was:

```text
simulation/settings.json: 1280x720
/tmp/dedalus_airsim_settings_<timestamp>.json: 640x360
AirSimNH process args: -settings=/tmp/dedalus_airsim_settings_<timestamp>.json
RPC Scene frames: 1280x720
```

The generated `/tmp` settings file was verified to contain the requested 640x360 `CaptureSettings`, and the running AirSimNH process was verified to include the `-settings=/tmp/...` argument. Despite that, `simGetImages(Scene)` returned 1280x720 frames, matching the launch-directory `simulation/settings.json` instead of the generated `/tmp` settings file.

Treat this as an observed packaged-runtime anomaly, not a general AirSim contract. Colosseum/AirSim source documents and parses `-settings=...`, but the packaged AirSimNH Scene capture path used here does not make that file the effective source for Scene capture dimensions.

Current operational rule for this environment:

```text
To change Scene RPC capture resolution, edit or patch simulation/settings.json,
then restart AirSim/Colosseum.

Do not rely on -settings=/tmp/... for capture-resolution sweeps in this
packaged AirSimNH environment.
```

`simulation/run.sh --airsim-camera-width W --airsim-camera-height H` should therefore either patch `simulation/settings.json` with a backup/restore flow, or be removed/disabled until it is implemented using the launch-directory settings path. A generated `/tmp` settings file is still useful for inspection, but it should not be treated as effective for Scene capture resolution in this environment.

## Real drone interpretation

On a real drone, `simGetImages` does not exist. It is replaced by the physical camera and platform capture path:

```text
sensor exposure
  -> sensor readout
  -> ISP / driver / DMA buffering
  -> host, pinned, unified, or GPU-accessible memory
  -> perception preprocessing / inference
```

That latency must be measured on the target camera path. It should not be inferred directly from AirSim RPC timings.

## Design decision

Do not prioritize `shared_memory` or VRAM handle passing in Milestone 2 unless bridge-internal timing shows `stdout_write_ms` / transfer time is the dominant cost.

For production perception, still plan for GPU-resident frames:

```text
ImageView::bytes remains the dependency-free CPU fallback.
FramePacket should eventually be able to carry an optional source-neutral ImageBufferHandle.
Backend-specific CUDA, EGL, NVMM, GStreamer, OpenCV, camera SDK, or simulator handles should stay behind provider/accessor modules.
```

## Profiling command

Use the one-command profiler:

```bash
./scripts/profile-airsim-bridge-latency.sh \
  --mode frame-ego \
  --frames 300 \
  --width 1280 \
  --height 720 \
  --skip-build \
  --skip-ctest
```

The `--width` and `--height` arguments are metadata for throughput reporting and expectation checks. They do not change AirSim capture resolution.

For this packaged AirSimNH environment, change `simulation/settings.json` and restart the simulator to perform a true resolution sweep. Confirm the effective result by checking the profiler's `actual_resolution_counts` line.
