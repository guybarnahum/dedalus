#pragma once

// Shared utilities used by ONNXDepthEngine and UniDepthV2DepthEngine.
// Not part of the public depth inference API — include only from engine TUs.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace dedalus {
namespace depth_engine_utils {

// Nearest-neighbour resize of a uint8 H×W×3 RGB image to dst_w × dst_h,
// followed by ImageNet normalisation.  Output: NCHW float32 [1, 3, dst_h, dst_w].
//
// mean/std are channel-wise (R, G, B order).  Standard ImageNet values:
//   mean = {0.485, 0.456, 0.406}   std = {0.229, 0.224, 0.225}
std::vector<float> resize_and_normalise(
    const std::vector<std::uint8_t>& bytes,
    int src_w, int src_h,
    int dst_w, int dst_h,
    float mean_r, float mean_g, float mean_b,
    float std_r,  float std_g,  float std_b);

// Nearest-neighbour downsample of a float src_h × src_w map to dst_h × dst_w.
// For RGB preprocessing only — not for depth maps (use downsample_min_z instead).
std::vector<float> downsample(
    const float* src, int src_w, int src_h,
    int dst_w, int dst_h);

// Min-pool downsample of a metric depth (Z) map: each output pixel gets the
// minimum Z value (= closest obstacle) within its corresponding input bin.
//
// This is the correct aggregation for obstacle detection:
//   - Nearest-neighbour picks 1 of ~N input pixels, missing thin obstacles.
//   - Min-pool guarantees the closest object in each spatial bin is retained,
//     consistent with the p5-percentile VD4a sampler philosophy.
// Input Z values: metric depth in metres (HIGH=FAR, 0 = invalid/unknown).
// Zero values are excluded from the min so they don't mask valid depth.
std::vector<float> downsample_min_z(
    const float* src, int src_w, int src_h,
    int dst_w, int dst_h);

// Write a float32 H×W array as a NumPy 1.0 .npy file.
// No-op if the file cannot be opened.  Logs path to stderr on success.
void dump_npy_f32(const char* path, const float* data, int h, int w);

// Write a uint8 H×W×C array as a NumPy 1.0 .npy file.
// No-op if the file cannot be opened.  Logs path to stderr on success.
void dump_npy_u8(const char* path, const std::uint8_t* data, int h, int w, int c);

// Write a float32 H×W depth map to DEDALUS_DEPTH_DEBUG_DIR/<frame_id>.npy
// when the env var is set and frame_id is non-empty.  No-op otherwise.
void maybe_write_debug_npy(
    const char* debug_dir,
    const std::string& frame_id,
    const float* data, int h, int w);

}  // namespace depth_engine_utils
}  // namespace dedalus
