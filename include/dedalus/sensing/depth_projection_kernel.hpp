#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "dedalus/core/types.hpp"
#include "dedalus/occupancy/occupancy_types.hpp"

namespace dedalus {

// All-POD input to the projection kernel.
//
// Populated fresh each frame from ObstacleSensingVolume + MetricScaleEstimate.
// Passed by value to both the CPU fallback and the CUDA kernel (VD6).
// No pointers, no std::string — safe to copy to device memory directly.
struct ProjectionParams {
    // Pinhole intrinsics (float for kernel throughput)
    float fx{0.0F};
    float fy{0.0F};
    float cx{0.0F};
    float cy{0.0F};

    // Lens distortion (Brown-Conrady k1, k2 used in unproject step)
    float k1{0.0F};
    float k2{0.0F};

    // Frame dimensions and sampling
    int width{0};
    int height{0};
    int stride{4};            // pixel step; default 4 → 1/16 density

    // Depth range filter
    float min_depth_m{0.2F};
    float max_depth_m{80.0F};

    // Metric scale:  depth_m = scale / inverse_depth
    float scale{1.0F};

    // Voxel grid resolution used to deduplicate evidence
    float voxel_size_m{0.5F};

    // Sensing volume origin and axes in local frame
    float origin_x{0.0F};
    float origin_y{0.0F};
    float origin_z{0.0F};

    float forward_x{1.0F};
    float forward_y{0.0F};
    float forward_z{0.0F};

    float right_x{0.0F};
    float right_y{1.0F};
    float right_z{0.0F};

    float up_x{0.0F};
    float up_y{0.0F};
    float up_z{-1.0F};

    // Maximum evidence entries the caller has allocated in the output buffer
    std::uint32_t max_evidence{512U};
};

// GPU-side intermediate evidence struct.
//
// 48-byte POD: no std::string, no vtable, no dynamic allocation.
// Lives in device memory (or cudaMallocManaged on Jetson).
// Inflated to ObstacleEvidence host-side by inflate() which stamps string fields.
//
// Layout (explicit, no padding surprises):
//   [0 ]  center_x, center_y, center_z       12 bytes
//   [12]  size_x, size_y, size_z             12 bytes
//   [24]  normal_x, normal_y, normal_z       12 bytes
//   [36]  confidence                          4 bytes
//   [40]  range_m                             4 bytes
//   [44]  state, shape, flags (4 × uint8)     4 bytes
//                                         = 48 bytes
struct DeviceObstacleEvidence {
    float center_x{0.0F};
    float center_y{0.0F};
    float center_z{0.0F};

    float size_x{0.0F};
    float size_y{0.0F};
    float size_z{0.0F};

    float normal_x{0.0F};
    float normal_y{0.0F};
    float normal_z{0.0F};

    float confidence{0.0F};
    float range_m{0.0F};

    // Encode ObstacleEvidenceState / ObstacleEvidenceShape as raw uint8.
    // inflate() maps these back to the typed enums.
    std::uint8_t state{0U};   // ObstacleEvidenceState::Unknown = 0
    std::uint8_t shape{0U};   // ObstacleEvidenceShape::Voxel   = 0
    std::uint8_t is_thin_structure_hint{0U};
    std::uint8_t is_surface_hint{0U};
};
static_assert(sizeof(DeviceObstacleEvidence) == 48,
              "DeviceObstacleEvidence must be exactly 48 bytes");

// ---------------------------------------------------------------------------
// CPU fallback free functions (implementations in depth_projection_kernel.cpp)
// CUDA equivalents added at VD6 (same signatures, different TU).
// ---------------------------------------------------------------------------

// Back-project every stride-th depth sample into local-frame voxels.
// Writes at most params.max_evidence entries into out[]; actual count in count_out.
void project_depth_to_device_evidence(
    const float*              inverse_depth,  // H × W, row-major
    const ProjectionParams&   params,
    DeviceObstacleEvidence*   out,             // pre-allocated, params.max_evidence entries
    std::uint32_t&            count_out);

// RANSAC plane fitting over the projected point cloud.
// Emits SurfacePatch-shaped DeviceObstacleEvidence with is_surface_hint=1.
// Runs in the same pass as projection — no re-inference.
void fit_surface_patches_device(
    const DeviceObstacleEvidence* evidence,
    std::uint32_t                 count,
    const ProjectionParams&       params,
    DeviceObstacleEvidence*       patches_out,
    std::uint32_t&                patches_count_out);

// Sobel + NMS + connected-components thin-structure detector.
// No OpenCV. Emits LineSegment-shaped DeviceObstacleEvidence with is_thin_structure_hint=1.
void detect_thin_structures_device(
    const float*            inverse_depth,  // H × W, row-major
    const ProjectionParams& params,
    DeviceObstacleEvidence* thin_out,
    std::uint32_t&          thin_count_out);

// Stamp string fields onto DeviceObstacleEvidence[] → ObstacleEvidence[].
// Called host-side only; string values come from compile-time / config constants.
[[nodiscard]] std::vector<ObstacleEvidence> inflate(
    const DeviceObstacleEvidence* evidence,
    std::uint32_t                 count,
    const std::string&            sensor_name,
    const std::string&            source_provider,
    const MapFrameId&             map_frame_id,
    TimePoint                     timestamp);

}  // namespace dedalus
