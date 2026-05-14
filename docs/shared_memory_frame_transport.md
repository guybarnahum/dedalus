# Shared-Memory Frame Transport

Milestone 2.18 defines the RAM shared-memory transport direction for the AirSim/core bridge.

This started as a design/protocol slice, then added one immediate copy reduction in the existing pipe/binary path. The current `BridgeTransport` API now supports reading stream payloads directly into `std::vector<std::uint8_t>` for paths that ultimately need an `ImageView::bytes` vector. This removes the large C++ payload copy that previously converted:

```text
pipe read -> std::string payload -> ImageView::bytes copy
```

into:

```text
pipe read -> std::vector<uint8_t> payload -> move into ImageView::bytes
```

A true no-extra-copy RAM shared-memory transport still requires a future frame-view API that can expose a mapped memory region with lifetime semantics.

## Motivation

Recent AirSim bridge profiling shows that frame size matters:

```text
1920x1080 frame_ego:
  sim_get_images_ms p95 ~= 60-75 ms
  stdout_write_ms   p95 ~= 5-6 ms

1280x720 frame_ego:
  sim_get_images_ms p95 ~= 51-54 ms
  stdout_write_ms   p95 ~= 2.5-3 ms

640x360 frame_ego, 600-frame capacity run:
  actual_resolution_counts: 640x360:600
  frame_source.next_frame_wait p95 ~= 35.317 ms
  frame_source.next_frame_wait p99 ~= 69.087 ms
  sim_get_images_ms p95 ~= 35.146 ms
  sim_get_images_ms p99 ~= 68.919 ms
  stdout_write_ms p95 ~= 0.821 ms
  stdout_write_ms p99 ~= 0.986 ms
```

The dominant current cost remains AirSim image extraction / RPC / render readback through `simGetImages`, but the pipe payload write/read cost scales with frame size. A RAM shared-memory transport can reduce IPC copy cost, especially at 720p/1080p, while preserving the provider boundary.

This should not be confused with a future VRAM-resident frame path. The current AirSim Python bridge receives CPU bytes from `simGetImages`; uploading those bytes to GPU just to pass a pointer would add work. GPU-resident frames become useful when the producer can expose GPU-backed buffers directly, such as camera SDK buffers, NVMM, DMABUF, EGLImage, CUDA IPC, or a simulator plugin.

## Current copy-reduction patch

Milestone 2.18 added:

```cpp
virtual std::optional<std::vector<std::uint8_t>> read_stream_byte_vector(
    const std::string& command,
    std::size_t byte_count) = 0;
```

`PipeBridgeTransport::read_stream_byte_vector()` reads persistent bridge bytes directly into a `std::vector<std::uint8_t>`. `AirSimFrameSource::next_stream_binary_frame()` now uses that path for the large RGB payload and moves the resulting vector into `ImageView::bytes`.

The binary header and ego sidecar still use `std::string` because they are tiny compared with image payloads. `stream_jsonl` and `one_shot_ppm` are unchanged because those debug/compat paths require JSON/base64 or PPM parsing anyway.

Remaining unavoidable copies in the current AirSim path:

```text
AirSim / Unreal render target -> simGetImages CPU/Python bytes
Python writes bytes to stdout pipe
OS pipe buffering
C++ fread into ImageView-owned vector
```

The avoided C++ copy was:

```text
std::string payload -> ImageView::bytes
```

## Scope

Milestone 2.18 target:

```text
producer bridge process
  -> shared memory ring of CPU RGB frame slots
  -> C++ frame source reads latest/next complete slot
  -> FramePacket uses a view/copy policy compatible with current ImageView
```

Non-goals for this slice:

```text
- CUDA / VRAM pointer passing
- simulator render-target plugin work
- changing core perception contracts to depend on shared memory
- making shared memory the default CI path
```

## Required API evolution for true shared memory

The existing `read_stream_byte_vector()` path reduces one copy while preserving the current owning `ImageView::bytes` contract. It is not true no-copy shared memory.

A no-extra-copy RAM path needs a frame-oriented borrowed view, for example:

```cpp
struct BridgeFrameView {
    std::uint64_t sequence;
    std::int64_t timestamp_ns;
    std::uint32_t width;
    std::uint32_t height;
    std::uint32_t channels;
    std::uint32_t pixel_format;
    std::span<const std::byte> payload;
    std::span<const std::byte> sidecar;
};

class FrameBridgeTransport {
public:
    virtual std::optional<BridgeFrameView> read_frame(const std::string& command) = 0;
    virtual void release_frame(std::uint64_t sequence) = 0;
};
```

The exact names can change, but the important property is that the returned object is a borrowed view into transport-managed memory with explicit lifetime/release behavior.

## CPU shared-memory ring protocol

A simple shared-memory ring can use fixed-size slots. Each slot contains a small metadata header followed by RGB bytes and optional sidecar bytes.

Recommended slot header:

```text
magic[8]          = DEDSHM1\0
version           uint32
slot_header_size  uint32
slot_size         uint64
sequence          uint64
timestamp_ns      int64
width             uint32
height            uint32
channels          uint32
pixel_format      uint32
payload_size      uint32
sidecar_size      uint32
state             uint32  // empty, writing, complete
reserved          uint32
```

Producer write order:

```text
1. choose next slot
2. mark state = writing
3. write metadata except complete state
4. write payload and sidecar
5. publish sequence
6. mark state = complete with release/acquire memory ordering
```

Consumer read order:

```text
1. find expected or latest complete slot
2. validate magic/version/header/payload sizes
3. expose payload as BridgeFrameView
4. release when frame source is done
```

The first implementation can use a single-producer/single-consumer policy. Dropped frames are acceptable in capacity profiling if the consumer explicitly records skipped sequence numbers.

## Timing expectations

Shared memory can reduce this part:

```text
stdout_write_ms / pipe payload copy
C++ pipe read into owned frame storage
```

It will not remove this part:

```text
AirSim simGetImages render readback / RPC / Python image_data_uint8 creation
```

Expected benefit based on measured pipe write costs:

```text
640x360:   under ~1 ms p95 saved
1280x720:  about ~2.5-3 ms p95 saved
1920x1080: about ~5-6 ms p95 saved
```

The immediate `read_stream_byte_vector()` patch saves only the extra C++ payload copy, not the Python stdout write or OS pipe transfer. The larger shared-memory transport is still useful, but it should be measured against the already-optimized pipe path.

## Implementation plan

Recommended stages:

```text
2.18A — protocol/design documentation
  Define CPU shared-memory frame transport semantics and API requirements.

2.18B — immediate pipe/binary copy reduction
  Add byte-vector bridge read path and move binary RGB payloads into ImageView::bytes.
  Completed.

2.18C — transport API extension for borrowed frame views
  Add a frame-view oriented transport path while keeping existing pipe methods.
  Pipe implementation may adapt by owning a buffer internally and returning a view.

2.18D — Python shared-memory producer prototype
  Add an optional AirSim bridge mode that creates a POSIX shared-memory segment
  and publishes DEDSHM1 slots.

2.18E — C++ shared-memory consumer prototype
  Implement SharedMemoryBridgeTransport behind bridge_transport: shared_memory.
  Keep pipe as default and keep CI dependency-light.

2.18F — profiler comparison
  Extend profile-airsim-bridge-latency.sh to compare pipe vs shared_memory at
  640x360, 1280x720, and 1920x1080.
```

## Provider-boundary rule

Do not put POSIX shared-memory, CUDA, OpenCV, GStreamer, AirSim, or camera-SDK types into `FramePacket`, `ImageView`, or perception/world-model contracts.

Transport-specific handles belong behind transport/provider modules. Future production `FramePacket` may carry a source-neutral optional image buffer handle, but backend-specific interpretation should live in accessors/providers.
