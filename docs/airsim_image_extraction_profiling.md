# AirSim Image Extraction Profiling

Milestone 2.17 tracks the difference between simulator bridge latency and production camera latency.

## Current finding

The live AirSim frame-attached ego path is not currently dominated by pipe transfer.

At 1280x720, the measured `frame_ego` path showed approximately:

```text
sim_get_images_ms p95 ~= 61 ms
stdout_write_ms   p95 ~= 5-6 ms
```

At 640x360, reducing pixel count by 4x did not materially reduce latency. The measured path still showed approximately:

```text
sim_get_images_ms p95 ~= 57 ms
stdout_write_ms   p95 ~= 5-6 ms
```

This suggests the current Milestone 2 AirSim path is dominated by AirSim image extraction / RPC / render readback behavior, not raw byte volume or the C++ pipe transport.

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

The `--width` and `--height` arguments are metadata for throughput reporting and expectation checks. They do not change AirSim capture resolution. Change AirSim `settings.json` and restart the simulator to perform a true resolution sweep.
