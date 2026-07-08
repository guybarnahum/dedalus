#pragma once

// CudaDepthDispatcher — GPU-accelerated depth pipeline for discrete NVIDIA GPUs.
//
// Only present when DEDALUS_CUDA_ENABLED is defined (CMake DEDALUS_CUDA=ON).
// Mac / CPU-only builds see nothing; the CPU path in depth_projection_kernel.cpp
// is used unchanged.
//
// L4 memory model: host depth map → cudaMemcpyAsync → device (PCIe).
// No cudaMallocManaged — discrete GPU, no unified address space.
//
// Thread ownership: single-threaded detector tick; not reentrant.

#ifdef DEDALUS_CUDA_ENABLED

#include <cstdint>
#include "dedalus/sensing/depth_projection_kernel.hpp"

namespace dedalus {

// Pre-allocated GPU resources for the visual depth pipeline.
// Buffers are sized lazily on first use and reused across frames.
class CudaDepthDispatcher {
public:
    CudaDepthDispatcher();
    ~CudaDepthDispatcher();

    CudaDepthDispatcher(const CudaDepthDispatcher&)            = delete;
    CudaDepthDispatcher& operator=(const CudaDepthDispatcher&) = delete;

    // GPU back-projection.
    // Equivalent to project_depth_to_device_evidence() but each pixel runs
    // in parallel.  No GPU-side voxel dedup (L1 map deduplicates at ingest).
    void project(
        const float*             inverse_depth,  // host, H×W row-major
        const ProjectionParams&  params,
        DeviceObstacleEvidence*  host_out,        // pre-allocated, params.max_evidence entries
        std::uint32_t&           count_out);

    // Parallel RANSAC plane hypotheses on GPU + centroid/emit on host.
    void fit_patches(
        const DeviceObstacleEvidence* evidence,
        std::uint32_t                 count,
        const ProjectionParams&       params,
        DeviceObstacleEvidence*       patches_out,
        std::uint32_t&                patches_count_out);

    // GPU Sobel depth gradient + host BFS thin-structure connected components.
    void detect_thin(
        const float*            inverse_depth,  // host, H×W row-major
        const ProjectionParams& params,
        DeviceObstacleEvidence* thin_out,
        std::uint32_t&          thin_count_out);

private:
    struct Impl;   // defined in cuda_depth_kernels.cu — keeps CUDA headers out of here
    Impl* impl_;
};

}  // namespace dedalus

#endif  // DEDALUS_CUDA_ENABLED
