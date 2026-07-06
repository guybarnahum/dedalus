// cuda_depth_kernels.cu — GPU depth pipeline for NVIDIA discrete GPUs (L4, A10, A100).
//
// Three CUDA kernels + one host BFS pass:
//   project_depth_kernel    — per-pixel back-projection + voxel write (atomic counter)
//   ransac_plane_kernel     — parallel RANSAC plane hypothesis evaluation per block
//   sobel_gradient_kernel   — per-pixel depth-space Sobel gradient magnitude
//
// Memory model: cudaMalloc device buffers + cudaMemcpyAsync (pinned count readback).
// Single cudaStream_t per CudaDepthDispatcher for ordered async execution.
// No cudaMallocManaged — discrete PCIe GPU, no unified address space.

#ifdef DEDALUS_CUDA_ENABLED

#include "dedalus/sensing/cuda_depth_kernels.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <queue>
#include <unordered_set>
#include <vector>

#include <cstdio>

#include <cuda_runtime.h>

namespace dedalus {
namespace {

// ============================================================
// GPU plane for RANSAC block results
// ============================================================
struct GpuPlane {
    float nx{0.0F}, ny{0.0F}, nz{1.0F}, d{0.0F};
    unsigned int inliers{0};
};

// ============================================================
// Kernel 1: parallel depth back-projection
// One thread per stride-sampled depth pixel.
// No GPU-side voxel dedup — L1 map handles it at ingest time.
// ============================================================
__global__ void project_depth_kernel(
    const float* __restrict__ d_depth,
    int W, int H, int stride,
    float inv_fx, float inv_fy, float cx, float cy,
    float k1, float k2,
    float scale, float min_depth, float max_depth,
    float ox, float oy, float oz,
    float fwd_x, float fwd_y, float fwd_z,
    float rgt_x, float rgt_y, float rgt_z,
    float up_x,  float up_y,  float up_z,
    float voxel_size,
    DeviceObstacleEvidence* __restrict__ d_out,
    unsigned int* __restrict__ d_count,
    unsigned int max_ev)
{
    const int col = (static_cast<int>(blockIdx.x) * static_cast<int>(blockDim.x)
                     + static_cast<int>(threadIdx.x)) * stride;
    const int row = (static_cast<int>(blockIdx.y) * static_cast<int>(blockDim.y)
                     + static_cast<int>(threadIdx.y)) * stride;
    if (col >= W || row >= H) return;

    const float dr = d_depth[row * W + col];
    if (!isfinite(dr) || dr <= 0.0F) return;
    const float depth_m = scale / dr;
    if (depth_m < min_depth || depth_m > max_depth) return;

    float xn = (static_cast<float>(col) - cx) * inv_fx;
    float yn = (static_cast<float>(row) - cy) * inv_fy;

    // Brown-Conrady radial undistortion (k1, k2 only)
    if (k1 != 0.0F || k2 != 0.0F) {
        const float r2  = xn*xn + yn*yn;
        const float rad = 1.0F + k1*r2 + k2*r2*r2;
        xn /= rad;  yn /= rad;
    }

    const float xc = xn * depth_m;
    const float yc = yn * depth_m;
    const float zc = depth_m;

    const float lx = ox + zc*fwd_x + xc*rgt_x - yc*up_x;
    const float ly = oy + zc*fwd_y + xc*rgt_y - yc*up_y;
    const float lz = oz + zc*fwd_z + xc*rgt_z - yc*up_z;

    // Voxel centre
    const float vcx = (floorf(lx / voxel_size) + 0.5F) * voxel_size;
    const float vcy = (floorf(ly / voxel_size) + 0.5F) * voxel_size;
    const float vcz = (floorf(lz / voxel_size) + 0.5F) * voxel_size;

    const unsigned int idx = atomicAdd(d_count, 1u);
    if (idx >= max_ev) {
        atomicSub(d_count, 1u);
        return;
    }

    DeviceObstacleEvidence& ev = d_out[idx];
    ev.center_x = vcx;  ev.center_y = vcy;  ev.center_z = vcz;
    ev.size_x   = voxel_size;
    ev.size_y   = voxel_size;
    ev.size_z   = voxel_size;
    ev.normal_x = ev.normal_y = ev.normal_z = 0.0F;
    ev.confidence             = 0.75F;
    ev.range_m                = depth_m;
    ev.state                  = 2u;  // Occupied
    ev.shape                  = 3u;  // SurfacePatch — matches CPU project_depth_to_device_evidence
    ev.is_thin_structure_hint = 0u;
    ev.is_surface_hint        = 1u;
}

// ============================================================
// Kernel 2: parallel RANSAC plane hypothesis evaluation.
// Each thread = one random 3-point hypothesis.
// Block-local best tracked in shared memory → written to d_block_best.
// ============================================================
__device__ __forceinline__ unsigned int lcg_next(unsigned int& s) {
    s = s * 1664525u + 1013904223u;
    return s;
}

__global__ void ransac_plane_kernel(
    const float* __restrict__ d_pts,  // count × 3 floats, x/y/z interleaved
    unsigned int count,
    float inlier_threshold,
    unsigned int seed_offset,
    GpuPlane* __restrict__ d_block_best)
{
    __shared__ float        s_nx, s_ny, s_nz, s_pd;
    __shared__ unsigned int s_best_cnt;

    if (threadIdx.x == 0) {
        s_nx = 0.0F;  s_ny = 0.0F;  s_nz = 1.0F;  s_pd = 0.0F;
        s_best_cnt = 0u;
    }
    __syncthreads();

    if (count < 3u) {
        if (threadIdx.x == 0) d_block_best[blockIdx.x] = GpuPlane{};
        return;
    }

    unsigned int seed = (blockIdx.x * blockDim.x + threadIdx.x) ^ seed_offset ^ 0xDEADBEEFu;

    unsigned int ia = lcg_next(seed) % count;
    unsigned int ib = lcg_next(seed) % count;
    while (ib == ia) ib = lcg_next(seed) % count;
    unsigned int ic = lcg_next(seed) % count;
    while (ic == ia || ic == ib) ic = lcg_next(seed) % count;

    const float ax = d_pts[ia*3+0], ay = d_pts[ia*3+1], az = d_pts[ia*3+2];
    const float bx = d_pts[ib*3+0], by = d_pts[ib*3+1], bz = d_pts[ib*3+2];
    const float cx = d_pts[ic*3+0], cy = d_pts[ic*3+1], cz = d_pts[ic*3+2];

    float nx = (by-ay)*(cz-az) - (bz-az)*(cy-ay);
    float ny = (bz-az)*(cx-ax) - (bx-ax)*(cz-az);
    float nz = (bx-ax)*(cy-ay) - (by-ay)*(cx-ax);
    const float len = sqrtf(nx*nx + ny*ny + nz*nz);
    if (len < 1.0e-6F) {
        __syncthreads();
        if (threadIdx.x == 0) d_block_best[blockIdx.x] = GpuPlane{s_nx,s_ny,s_nz,s_pd,s_best_cnt};
        return;
    }
    nx /= len;  ny /= len;  nz /= len;
    const float pd = -(nx*ax + ny*ay + nz*az);

    unsigned int cnt = 0u;
    for (unsigned int i = 0u; i < count; ++i) {
        const float dist = fabsf(nx*d_pts[i*3+0] + ny*d_pts[i*3+1] + nz*d_pts[i*3+2] + pd);
        if (dist < inlier_threshold) ++cnt;
    }

    if (cnt > 0u) {
        const unsigned int old = atomicMax(&s_best_cnt, cnt);
        if (cnt > old) {
            s_nx = nx;  s_ny = ny;  s_nz = nz;  s_pd = pd;
        }
    }
    __syncthreads();

    if (threadIdx.x == 0) {
        GpuPlane p;
        p.nx = s_nx;  p.ny = s_ny;  p.nz = s_nz;  p.d = s_pd;  p.inliers = s_best_cnt;
        d_block_best[blockIdx.x] = p;
    }
}

// ============================================================
// Kernel 3: Sobel depth-space gradient magnitude
// ============================================================
__global__ void sobel_gradient_kernel(
    const float* __restrict__ d_depth,
    int W, int H,
    float scale, float min_depth, float max_depth,
    float* __restrict__ d_grad)
{
    const int u = static_cast<int>(blockIdx.x) * static_cast<int>(blockDim.x)
                + static_cast<int>(threadIdx.x);
    const int v = static_cast<int>(blockIdx.y) * static_cast<int>(blockDim.y)
                + static_cast<int>(threadIdx.y);
    if (u >= W || v >= H) return;

    if (u < 1 || u >= W-1 || v < 1 || v >= H-1) {
        d_grad[v*W + u] = 0.0F;
        return;
    }

    auto dm = [&](int pu, int pv) -> float {
        const float dr = d_depth[pv*W + pu];
        if (!isfinite(dr) || dr <= 0.0F) return max_depth;
        const float d = scale / dr;
        return (d >= min_depth && d <= max_depth) ? d : max_depth;
    };

    const float gx = -dm(u-1,v-1) + dm(u+1,v-1)
                     - 2.0F*dm(u-1,v) + 2.0F*dm(u+1,v)
                     - dm(u-1,v+1) + dm(u+1,v+1);
    const float gy = -dm(u-1,v-1) - 2.0F*dm(u,v-1) - dm(u+1,v-1)
                     + dm(u-1,v+1) + 2.0F*dm(u,v+1) + dm(u+1,v+1);

    d_grad[v*W + u] = sqrtf(gx*gx + gy*gy);
}

}  // anonymous namespace

// ============================================================
// CudaDepthDispatcher::Impl
// ============================================================
struct CudaDepthDispatcher::Impl {
    cudaStream_t stream{nullptr};

    // Device buffers — sized lazily, reused across frames
    float*                  d_depth{nullptr};
    float*                  d_gradient{nullptr};
    DeviceObstacleEvidence* d_evidence{nullptr};
    unsigned int*           d_count{nullptr};
    float*                  d_pts{nullptr};       // count×3 for RANSAC
    GpuPlane*               d_block_best{nullptr};

    // Pinned host buffer for count readback (avoids blocking cudaMemcpy)
    unsigned int* h_count{nullptr};

    std::size_t depth_elems{0};
    std::size_t evidence_elems{0};
    std::size_t pts_elems{0};

    static constexpr unsigned int MAX_RANSAC_BLOCKS  = 256u;
    static constexpr unsigned int RANSAC_THREADS_BLK = 128u;  // iterations per block

    void ensure_depth(std::size_t n) {
        if (n <= depth_elems) return;
        cudaFree(d_depth);    d_depth    = nullptr;
        cudaFree(d_gradient); d_gradient = nullptr;
        cudaMalloc(&d_depth,    n * sizeof(float));
        cudaMalloc(&d_gradient, n * sizeof(float));
        depth_elems = n;
    }

    void ensure_evidence(std::size_t n) {
        if (n <= evidence_elems) return;
        cudaFree(d_evidence); d_evidence = nullptr;
        cudaMalloc(&d_evidence, n * sizeof(DeviceObstacleEvidence));
        evidence_elems = n;
    }

    void ensure_pts(std::size_t count) {
        const std::size_t needed = count * 3u;
        if (needed <= pts_elems) return;
        cudaFree(d_pts); d_pts = nullptr;
        cudaMalloc(&d_pts, needed * sizeof(float));
        pts_elems = needed;
    }
};

// ============================================================
// CudaDepthDispatcher public API
// ============================================================
CudaDepthDispatcher::CudaDepthDispatcher() : impl_(new Impl) {
    // Always log CUDA init — this constructor fires on the first detect() call
    // (static singleton) and cudaStreamCreate triggers full CUDA context init,
    // which can take 5-30 s on first use.  If the mission hangs here, that
    // confirms CUDA context init as the hang point.
    std::fprintf(stderr, "[CudaDepth] init: cudaStreamCreate...\n");
    std::fflush(stderr);
    const cudaError_t sc_err = cudaStreamCreate(&impl_->stream);
    if (sc_err != cudaSuccess) {
        std::fprintf(stderr, "[CudaDepth] cudaStreamCreate FAILED: %s — GPU unavailable, "
                     "CUDA kernels will be skipped\n", cudaGetErrorString(sc_err));
        std::fflush(stderr);
        return;
    }
    cudaMalloc(&impl_->d_count,      sizeof(unsigned int));
    cudaMalloc(&impl_->d_block_best, Impl::MAX_RANSAC_BLOCKS * sizeof(GpuPlane));
    cudaMallocHost(&impl_->h_count,  sizeof(unsigned int));
    std::fprintf(stderr, "[CudaDepth] init: complete\n");
    std::fflush(stderr);
}

CudaDepthDispatcher::~CudaDepthDispatcher() {
    if (!impl_) return;
    cudaStreamSynchronize(impl_->stream);
    cudaStreamDestroy(impl_->stream);
    cudaFree(impl_->d_depth);
    cudaFree(impl_->d_gradient);
    cudaFree(impl_->d_evidence);
    cudaFree(impl_->d_count);
    cudaFree(impl_->d_pts);
    cudaFree(impl_->d_block_best);
    cudaFreeHost(impl_->h_count);
    delete impl_;
    impl_ = nullptr;
}

void CudaDepthDispatcher::project(
    const float*             depth_relative,
    const ProjectionParams&  p,
    DeviceObstacleEvidence*  host_out,
    std::uint32_t&           count_out)
{
    count_out = 0u;
    if (p.fx <= 0.0F || p.fy <= 0.0F || p.scale <= 0.0F
        || p.width <= 0 || p.height <= 0) return;

    const std::size_t npix = static_cast<std::size_t>(p.width) *
                             static_cast<std::size_t>(p.height);
    impl_->ensure_depth(npix);
    impl_->ensure_evidence(static_cast<std::size_t>(p.max_evidence));

    cudaMemcpyAsync(impl_->d_depth, depth_relative, npix * sizeof(float),
                    cudaMemcpyHostToDevice, impl_->stream);

    const unsigned int zero = 0u;
    cudaMemcpyAsync(impl_->d_count, &zero, sizeof(unsigned int),
                    cudaMemcpyHostToDevice, impl_->stream);

    // Grid covers the sampled pixel set: (W/stride) × (H/stride) threads
    const int sw = (p.width  + p.stride - 1) / p.stride;
    const int sh = (p.height + p.stride - 1) / p.stride;
    const dim3 block(16, 16);
    const dim3 grid(static_cast<unsigned int>((sw + 15) / 16),
                    static_cast<unsigned int>((sh + 15) / 16));

    project_depth_kernel<<<grid, block, 0, impl_->stream>>>(
        impl_->d_depth,
        p.width, p.height, p.stride,
        1.0F/p.fx, 1.0F/p.fy, p.cx, p.cy, p.k1, p.k2,
        p.scale, p.min_depth_m, p.max_depth_m,
        p.origin_x, p.origin_y, p.origin_z,
        p.forward_x, p.forward_y, p.forward_z,
        p.right_x,   p.right_y,   p.right_z,
        p.up_x,      p.up_y,      p.up_z,
        p.voxel_size_m,
        impl_->d_evidence, impl_->d_count, p.max_evidence);

    // Async readback count; synchronize before touching host_out
    cudaMemcpyAsync(impl_->h_count, impl_->d_count, sizeof(unsigned int),
                    cudaMemcpyDeviceToHost, impl_->stream);
    cudaStreamSynchronize(impl_->stream);

    const unsigned int n = std::min(*impl_->h_count, p.max_evidence);
    count_out = static_cast<std::uint32_t>(n);
    if (n > 0u) {
        cudaMemcpy(host_out, impl_->d_evidence,
                   n * sizeof(DeviceObstacleEvidence), cudaMemcpyDeviceToHost);
    }

    // Deduplicate by voxel index — matches CPU project_depth_to_device_evidence behaviour.
    // Centers are already snapped: center = (floor(w/vox)+0.5)*vox, so recovering
    // the index via floor(center/vox) is exact.
    if (count_out > 1u) {
        const float inv_vox = 1.0F / p.voxel_size_m;
        std::unordered_set<std::uint64_t> seen;
        seen.reserve(count_out);
        std::uint32_t out = 0u;
        for (std::uint32_t i = 0u; i < count_out; ++i) {
            const int ix = static_cast<int>(std::floor(host_out[i].center_x * inv_vox));
            const int iy = static_cast<int>(std::floor(host_out[i].center_y * inv_vox));
            const int iz = static_cast<int>(std::floor(host_out[i].center_z * inv_vox));
            const std::uint64_t key =
                (static_cast<std::uint64_t>(static_cast<std::uint16_t>(ix)) << 32u)
              | (static_cast<std::uint64_t>(static_cast<std::uint16_t>(iy)) << 16u)
              |  static_cast<std::uint64_t>(static_cast<std::uint16_t>(iz));
            if (seen.insert(key).second) {
                host_out[out++] = host_out[i];
            }
        }
        count_out = out;
    }
}

void CudaDepthDispatcher::fit_patches(
    const DeviceObstacleEvidence* evidence,
    std::uint32_t                 count,
    const ProjectionParams&       params,
    DeviceObstacleEvidence*       patches_out,
    std::uint32_t&                patches_count_out)
{
    patches_count_out = 0u;
    if (count < 3u) return;

    impl_->ensure_pts(count);

    // Build flat xyz array for the RANSAC kernel
    std::vector<float> pts(count * 3u);
    for (std::uint32_t i = 0u; i < count; ++i) {
        pts[i*3+0] = evidence[i].center_x;
        pts[i*3+1] = evidence[i].center_y;
        pts[i*3+2] = evidence[i].center_z;
    }
    cudaMemcpyAsync(impl_->d_pts, pts.data(), pts.size() * sizeof(float),
                    cudaMemcpyHostToDevice, impl_->stream);
    cudaMemsetAsync(impl_->d_block_best, 0,
                    Impl::MAX_RANSAC_BLOCKS * sizeof(GpuPlane), impl_->stream);

    // 256 blocks × 128 threads/block = 32 768 parallel RANSAC hypotheses
    static unsigned int seed_ctr = 0u;
    ransac_plane_kernel<<<Impl::MAX_RANSAC_BLOCKS, Impl::RANSAC_THREADS_BLK,
                          0, impl_->stream>>>(
        impl_->d_pts, count, params.voxel_size_m, seed_ctr++, impl_->d_block_best);

    std::vector<GpuPlane> block_results(Impl::MAX_RANSAC_BLOCKS);
    cudaMemcpyAsync(block_results.data(), impl_->d_block_best,
                    Impl::MAX_RANSAC_BLOCKS * sizeof(GpuPlane),
                    cudaMemcpyDeviceToHost, impl_->stream);
    cudaStreamSynchronize(impl_->stream);

    // Global best across all blocks
    GpuPlane best{};
    for (const auto& b : block_results) {
        if (b.inliers > best.inliers) best = b;
    }

    const unsigned int min_inliers = std::max(3u, count / 10u);
    if (best.inliers < min_inliers) return;

    // Ensure normal faces camera origin
    const float dot = best.nx*params.origin_x + best.ny*params.origin_y
                    + best.nz*params.origin_z + best.d;
    if (dot < 0.0F) { best.nx=-best.nx; best.ny=-best.ny; best.nz=-best.nz; best.d=-best.d; }

    // Centroid of inliers (host side — count is small after projection)
    const float thresh = params.voxel_size_m;
    float cx=0.0F, cy=0.0F, cz=0.0F;
    unsigned int cnt = 0u;
    for (std::uint32_t i = 0u; i < count; ++i) {
        const float dist = std::abs(best.nx*pts[i*3+0] + best.ny*pts[i*3+1]
                                  + best.nz*pts[i*3+2] + best.d);
        if (dist < thresh) {
            cx += pts[i*3+0]; cy += pts[i*3+1]; cz += pts[i*3+2]; ++cnt;
        }
    }
    if (cnt == 0u) return;
    cx /= static_cast<float>(cnt);
    cy /= static_cast<float>(cnt);
    cz /= static_cast<float>(cnt);

    DeviceObstacleEvidence& ev = patches_out[patches_count_out++];
    ev.center_x = cx;  ev.center_y = cy;  ev.center_z = cz;
    ev.size_x = ev.size_y = ev.size_z = params.voxel_size_m;
    ev.normal_x = best.nx;  ev.normal_y = best.ny;  ev.normal_z = best.nz;
    ev.confidence             = static_cast<float>(cnt) / static_cast<float>(count);
    ev.range_m                = std::abs(best.d);
    ev.state                  = 2u;
    ev.shape                  = 3u;  // SurfacePatch
    ev.is_thin_structure_hint = 0u;
    ev.is_surface_hint        = 1u;
}

void CudaDepthDispatcher::detect_thin(
    const float*            depth_relative,
    const ProjectionParams& params,
    DeviceObstacleEvidence* thin_out,
    std::uint32_t&          thin_count_out)
{
    thin_count_out = 0u;
    const int W = params.width;
    const int H = params.height;
    if (W < 5 || H < 5) return;

    const std::size_t npix = static_cast<std::size_t>(W) * static_cast<std::size_t>(H);
    impl_->ensure_depth(npix);

    cudaMemcpyAsync(impl_->d_depth, depth_relative, npix * sizeof(float),
                    cudaMemcpyHostToDevice, impl_->stream);

    const dim3 block(16, 16);
    const dim3 grid(static_cast<unsigned int>((W + 15) / 16),
                    static_cast<unsigned int>((H + 15) / 16));

    sobel_gradient_kernel<<<grid, block, 0, impl_->stream>>>(
        impl_->d_depth, W, H,
        params.scale, params.min_depth_m, params.max_depth_m,
        impl_->d_gradient);

    // Transfer gradient back to host for BFS (~0.1 ms for 640×480 on L4)
    std::vector<float> grad(npix);
    cudaMemcpyAsync(grad.data(), impl_->d_gradient, npix * sizeof(float),
                    cudaMemcpyDeviceToHost, impl_->stream);
    cudaStreamSynchronize(impl_->stream);

    // Host BFS connected components on high-gradient pixels.
    // Gradient threshold: >2 m/px depth jump = likely thin-structure boundary.
    const float grad_threshold = 2.0F;
    const int   min_length     = 4;
    const float min_aspect     = 2.5F;
    const int   max_thin       = 8;

    std::vector<bool> mask(npix, false);
    for (std::size_t i = 0; i < npix; ++i) mask[i] = (grad[i] > grad_threshold);

    std::vector<bool> visited(npix, false);
    const int du4[4] = {1,-1,0,0};
    const int dv4[4] = {0,0,1,-1};

    for (int v0 = 0; v0 < H && static_cast<int>(thin_count_out) < max_thin; ++v0) {
        for (int u0 = 0; u0 < W && static_cast<int>(thin_count_out) < max_thin; ++u0) {
            const std::size_t idx0 = static_cast<std::size_t>(v0) *
                                     static_cast<std::size_t>(W) +
                                     static_cast<std::size_t>(u0);
            if (!mask[idx0] || visited[idx0]) continue;

            std::vector<std::pair<int,int>> comp;
            std::queue<std::pair<int,int>> q;
            q.push({u0, v0});
            visited[idx0] = true;
            while (!q.empty()) {
                auto [u, v] = q.front(); q.pop();
                comp.push_back({u, v});
                for (int k = 0; k < 4; ++k) {
                    const int nu = u+du4[k], nv = v+dv4[k];
                    if (nu < 0 || nu >= W || nv < 0 || nv >= H) continue;
                    const std::size_t ni = static_cast<std::size_t>(nv) *
                                          static_cast<std::size_t>(W) +
                                          static_cast<std::size_t>(nu);
                    if (!mask[ni] || visited[ni]) continue;
                    visited[ni] = true;
                    q.push({nu, nv});
                }
            }
            if (static_cast<int>(comp.size()) < min_length) continue;

            int u_min=W, u_max=0, v_min=H, v_max=0;
            for (auto& [pu, pv] : comp) {
                u_min=std::min(u_min,pu); u_max=std::max(u_max,pu);
                v_min=std::min(v_min,pv); v_max=std::max(v_max,pv);
            }
            const int   bb_w   = u_max - u_min + 1;
            const int   bb_h   = v_max - v_min + 1;
            const float aspect = static_cast<float>(std::max(bb_w, bb_h)) /
                                 static_cast<float>(std::max(1, std::min(bb_w, bb_h)));
            if (aspect < min_aspect) continue;

            int ua, va, ub, vb;
            if (bb_h >= bb_w) { ua=(u_min+u_max)/2; va=v_min; ub=ua; vb=v_max; }
            else               { va=(v_min+v_max)/2; ua=u_min; vb=va; ub=u_max; }

            auto unproj = [&](int u, int v, float& lx, float& ly, float& lz) -> bool {
                const float dr = depth_relative[
                    static_cast<std::size_t>(v)*static_cast<std::size_t>(W) +
                    static_cast<std::size_t>(u)];
                if (!std::isfinite(dr) || dr <= 0.0F) return false;
                const float dm = params.scale / dr;
                if (dm < params.min_depth_m || dm > params.max_depth_m) return false;
                const float xn = (static_cast<float>(u)-params.cx) / params.fx;
                const float yn = (static_cast<float>(v)-params.cy) / params.fy;
                lx = params.origin_x + dm*params.forward_x + xn*dm*params.right_x - yn*dm*params.up_x;
                ly = params.origin_y + dm*params.forward_y + xn*dm*params.right_y - yn*dm*params.up_y;
                lz = params.origin_z + dm*params.forward_z + xn*dm*params.right_z - yn*dm*params.up_z;
                return true;
            };

            float ax,ay,az,bx,by,bz;
            if (!unproj(ua,va,ax,ay,az)) continue;
            if (!unproj(ub,vb,bx,by,bz)) continue;

            const float mx=(ax+bx)*0.5F, my=(ay+by)*0.5F, mz=(az+bz)*0.5F;
            const float hx=(bx-ax)*0.5F, hy=(by-ay)*0.5F, hz=(bz-az)*0.5F;
            const float length = 2.0F * sqrtf(hx*hx + hy*hy + hz*hz);
            if (length < 0.01F) continue;

            const float inv_len = 1.0F / (length * 0.5F);
            DeviceObstacleEvidence& ev = thin_out[thin_count_out++];
            ev.center_x = mx;  ev.center_y = my;  ev.center_z = mz;
            ev.size_x   = hx;  ev.size_y   = hy;  ev.size_z   = hz;
            ev.normal_x = hx*inv_len;
            ev.normal_y = hy*inv_len;
            ev.normal_z = hz*inv_len;
            ev.confidence             = std::min(1.0F, aspect / 10.0F);
            ev.range_m                = length;
            ev.state                  = 3u;  // ThinStructureRisk
            ev.shape                  = 4u;  // LineSegment
            ev.is_thin_structure_hint = 1u;
            ev.is_surface_hint        = 0u;
        }
    }
}

}  // namespace dedalus

#endif  // DEDALUS_CUDA_ENABLED
