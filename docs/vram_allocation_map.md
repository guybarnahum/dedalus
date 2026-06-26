# VRAM Allocation Map — L4 EC2 (23 GiB GDDR6)

GPU: NVIDIA L4, Ada Lovelace (sm_89), 23,034 MiB GDDR6, 242 TFLOPS FP16.
Memory model: discrete PCIe — no unified memory. All transfers are explicit
`cudaMemcpyAsync`.

---

## Static Allocation Map (steady-state)

| Client | Footprint | Lifecycle | Controlled by |
|---|---|---|---|
| **AirSim** (Unreal renderer) | ~2,800–3,400 MiB | Persistent while AirSim runs | AirSim / Unreal Engine |
| **ONNX Runtime CUDA EP** | ≤ 1,024 MiB | Persistent after first inference | `ONNXDepthEngineConfig::cuda_arena_limit_bytes` |
| **TensorRT inference engine** (DepthAnythingV2-Small FP16) | ~80–150 MiB | Persistent after `TensorRTDepthEngine` ctor | `TensorRTDepthEngine::Impl::d_input + d_output` |
| **CUDA depth kernels** (projection, RANSAC, Sobel) | ~30–80 MiB | Persistent after `CudaDepthDispatcher` ctor (lazy-resize) | `CudaDepthDispatcher::Impl` device buffers |
| **CUDA runtime overhead** (context, libs) | ~200–400 MiB | Persistent once any CUDA API is called | CUDA driver |
| **TRT builder workspace** | ≤ 512 MiB | First-run engine build only — freed after | `TensorRTDepthEngineConfig::builder_workspace_bytes` |

### Steady-state totals

```
AirSim:          ~3,100 MiB  (mid-estimate)
ONNX EP arena:  ≤1,024 MiB  (hard cap)
TRT engine:       ~120 MiB
CUDA kernels:      ~60 MiB
CUDA runtime:     ~300 MiB
────────────────────────────
Total in-use:   ~4,600 MiB
Headroom:      ~18,400 MiB  (of 23,034 MiB)
```

First-run peak (TRT build): add ≤ 512 MiB → still < 5,200 MiB.

---

## Per-component detail

### AirSim / Unreal Engine
AirSim allocates VRAM for the render scene, all camera buffers, and depth
capture surfaces.  Typical idle scene ≈ 2.8 GiB; complex outdoor environment
with multiple cameras ≈ 3.4 GiB.  This is not under Dedalus control.

Check live:
```bash
nvidia-smi --query-compute-apps=pid,process_name,used_memory \
           --format=csv,noheader
```

### ONNX Runtime CUDA EP
Configured in `ONNXDepthEngineConfig`:
```cpp
std::size_t cuda_arena_limit_bytes{1ULL * 1024 * 1024 * 1024};  // 1 GiB
```
The CUDA EP allocator arena is hard-capped via `gpu_mem_limit` in
`SessionOptionsAppendExecutionProvider_CUDA_V2`.  The arena grows lazily up to
the cap; actual usage for DepthAnythingV2-Small is typically 200–400 MiB.

### TensorRT depth engine
- Model weights (FP16): ~80 MiB for DepthAnythingV2-Small.
- Activation buffers (`d_input` + `d_output`): `3 × 518 × 518 × 4B` + `518 × 518 × 4B` ≈ 3.4 MiB.
- Total: ~85 MiB steady-state.
- Builder workspace (first run only): capped to `builder_workspace_bytes` = 512 MiB.

### CUDA depth kernels (`CudaDepthDispatcher`)
Lazy device buffers sized to the input depth map (640 × 480 default):
- `d_depth`:        `640×480 × 4B` ≈ 1.2 MiB
- `d_evidence`:     `max_evidence × sizeof(DeviceObstacleEvidence)` ≈ 1–4 MiB
- `d_gradient`:     same as `d_depth` ≈ 1.2 MiB
- RANSAC `d_planes`: `256 × sizeof(GpuPlane)` ≈ 0.01 MiB
- Pinned host buffer (`h_count`): 4 bytes (system RAM, not VRAM)

Total: < 10 MiB.  Buffers are reallocated via `cudaFree + cudaMalloc` only when
image dimensions change.

---

## How to measure at runtime

### 1. Per-process breakdown (from the shell)
```bash
# Memory used by each CUDA process on the GPU
nvidia-smi --query-compute-apps=pid,process_name,used_memory \
           --format=csv,noheader,nounits

# Total GPU memory used/free (1-second poll)
watch -n1 "nvidia-smi --query-gpu=memory.used,memory.free \
                       --format=csv,noheader"
```

### 2. From inside the Dedalus process (C++)
```cpp
#include <cuda_runtime.h>

size_t free_bytes{0}, total_bytes{0};
cudaMemGetInfo(&free_bytes, &total_bytes);
// free_bytes: available to this process right now
// total_bytes: total physical VRAM on the device
```
Call this before and after each major allocation to measure actual usage.

### 3. NVML (programmatic per-process accounting)
```python
import pynvml
pynvml.nvmlInit()
handle = pynvml.nvmlDeviceGetHandleByIndex(0)
info   = pynvml.nvmlDeviceGetMemoryInfo(handle)
print(f"Used: {info.used // 1024**2} MiB / {info.total // 1024**2} MiB")

# Per-process:
for proc in pynvml.nvmlDeviceGetComputeRunningProcesses(handle):
    print(f"PID {proc.pid}: {proc.usedGpuMemory // 1024**2} MiB")
```

### 4. NSight Systems (timeline)
```bash
nsys profile --trace=cuda,nvtx -o depth_profile \
    ./build-staging/apps/dedalus_mission_loop --config config/...
nsys-ui depth_profile.nsys-rep
```
Shows kernel timeline, memory allocation events, and host↔device transfers.

---

## Reducing VRAM pressure

If AirSim grows beyond 4 GiB (e.g., more cameras or higher resolution):

1. Lower `cuda_arena_limit_bytes` to 512 MiB — adequate for DepthAnythingV2-Small.
2. Switch from ONNX EP to TRT engine only — eliminates ORT arena overhead.
3. Use INT8 precision in TRT — halves activation memory vs FP16.
4. Use DepthAnythingV2-Small (80 MiB) not Base/Large (> 350 MiB).
5. Reduce `cuda_depth_kernels` resolution — `pixel_stride=2` halves buffer sizes.

---

## Capacity planning

| Scenario | VRAM used | Headroom |
|---|---|---|
| AirSim only (no Dedalus CUDA) | ~3.1 GiB | ~19.9 GiB |
| + ONNX CUDA EP (1 GiB cap) | ~4.1 GiB | ~18.9 GiB |
| + TRT engine (FP16) | ~4.3 GiB | ~18.7 GiB |
| + CUDA depth kernels | ~4.4 GiB | ~18.6 GiB |
| + TRT build workspace (first run) | ~4.9 GiB | ~18.1 GiB |
| Two AirSim camera streams | ~5.5 GiB | ~17.5 GiB |
| TRT Large model (350 MiB) | ~5.1 GiB | ~17.9 GiB |

L4 (23 GiB) has substantial headroom. The 512 MiB builder cap and 1 GiB ONNX
arena cap are conservative defaults — increase them if inference throughput
needs to scale.
