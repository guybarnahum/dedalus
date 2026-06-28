#include "dedalus/sensing/visual_depth_obstacle_detector.hpp"

#include <cmath>
#include <cstdint>
#include <vector>

#include "dedalus/sensing/depth_projection_kernel.hpp"
#include "dedalus/sensing/visual_depth_frame.hpp"

#ifdef DEDALUS_CUDA_ENABLED
#include "dedalus/sensing/cuda_depth_kernels.hpp"
#endif

namespace dedalus {
namespace {

#ifdef DEDALUS_CUDA_ENABLED
// File-static singleton dispatcher — one CudaDepthDispatcher per process.
// Safe: VisualDepthObstacleDetector is single-threaded (see class comment).
CudaDepthDispatcher& cuda_dispatch() {
    static CudaDepthDispatcher s;
    return s;
}
#endif

// Extract VisualDepthFrame from EgoSensingFrame.
// Copies image bytes — avoids lifetime coupling to the source frame.
VisualDepthFrame make_visual_depth_frame(const EgoSensingFrame& ego_frame) {
    VisualDepthFrame vdf;
    vdf.timestamp  = ego_frame.frame.timestamp;
    vdf.frame_id   = ego_frame.frame.frame_id;
    vdf.camera_id  = ego_frame.frame.camera_id;
    vdf.map_frame_id = ego_frame.ego.map_frame_id;

    vdf.width    = ego_frame.frame.image.width;
    vdf.height   = ego_frame.frame.image.height;
    vdf.channels = ego_frame.frame.image.channels;
    vdf.bytes    = ego_frame.frame.image.bytes;

    vdf.fx = ego_frame.frame.intrinsics.fx;
    vdf.fy = ego_frame.frame.intrinsics.fy;
    vdf.cx = ego_frame.frame.intrinsics.cx;
    vdf.cy = ego_frame.frame.intrinsics.cy;
    vdf.distortion.k1 = ego_frame.frame.intrinsics.distortion_k1;
    vdf.distortion.k2 = ego_frame.frame.intrinsics.distortion_k2;

    return vdf;
}

// Build ProjectionParams from EgoSensingFrame fields + detector config.
ProjectionParams make_projection_params(
    const EgoSensingFrame&                   ego_frame,
    const DepthInferenceResult&              inferred,
    const MetricScaleEstimate&               scale,
    const VisualDepthObstacleDetectorConfig& cfg) {

    const auto& sv = ego_frame.sensing_volume;
    ProjectionParams p;

    // Intrinsics from the source frame
    p.fx = static_cast<float>(ego_frame.frame.intrinsics.fx);
    p.fy = static_cast<float>(ego_frame.frame.intrinsics.fy);
    p.cx = static_cast<float>(ego_frame.frame.intrinsics.cx);
    p.cy = static_cast<float>(ego_frame.frame.intrinsics.cy);
    p.k1 = static_cast<float>(ego_frame.frame.intrinsics.distortion_k1);
    p.k2 = static_cast<float>(ego_frame.frame.intrinsics.distortion_k2);

    // Depth map dimensions (may differ from image if model rescales)
    p.width  = inferred.width;
    p.height = inferred.height;
    p.stride = static_cast<int>(cfg.pixel_stride);

    p.min_depth_m  = cfg.min_depth_m;
    p.max_depth_m  = cfg.max_depth_m;
    p.scale        = scale.scale;
    p.voxel_size_m = cfg.voxel_size_m;
    p.max_evidence = static_cast<std::uint32_t>(cfg.max_evidence);

    // Gimbal-corrected sensing volume axes (encoder reading at frame timestamp)
    p.origin_x  = static_cast<float>(sv.origin_local.x);
    p.origin_y  = static_cast<float>(sv.origin_local.y);
    p.origin_z  = static_cast<float>(sv.origin_local.z);
    p.forward_x = static_cast<float>(sv.forward_axis_local.x);
    p.forward_y = static_cast<float>(sv.forward_axis_local.y);
    p.forward_z = static_cast<float>(sv.forward_axis_local.z);
    p.right_x   = static_cast<float>(sv.right_axis_local.x);
    p.right_y   = static_cast<float>(sv.right_axis_local.y);
    p.right_z   = static_cast<float>(sv.right_axis_local.z);
    p.up_x      = static_cast<float>(sv.up_axis_local.x);
    p.up_y      = static_cast<float>(sv.up_axis_local.y);
    p.up_z      = static_cast<float>(sv.up_axis_local.z);

    return p;
}

}  // namespace

// ---------------------------------------------------------------------------
// VisualDepthObstacleDetector
// ---------------------------------------------------------------------------

VisualDepthObstacleDetector::VisualDepthObstacleDetector(
    std::unique_ptr<DepthEngineInterface> engine,
    MetricScaleEstimate                   scale,
    VisualDepthObstacleDetectorConfig     config)
    : engine_(std::move(engine))
    , scale_(scale)
    , config_(config) {}

std::string VisualDepthObstacleDetector::provider_name() const {
    return "visual_depth_obstacle_detector";
}

std::vector<ObstacleEvidence> VisualDepthObstacleDetector::detect(
    const EgoSensingFrame& ego_frame) {
    return detect_visual_depth_obstacles(ego_frame, *engine_, scale_, config_);
}

// ---------------------------------------------------------------------------
// Free-function implementation (shared by class and test code)
// ---------------------------------------------------------------------------

std::vector<ObstacleEvidence> detect_visual_depth_obstacles(
    const EgoSensingFrame&                   ego_frame,
    DepthEngineInterface&                    engine,
    const MetricScaleEstimate&               scale,
    const VisualDepthObstacleDetectorConfig& cfg) {

    const VisualDepthFrame vdf = make_visual_depth_frame(ego_frame);

    const DepthInferenceResult inferred = engine.infer(vdf);
    if (!inferred.valid || inferred.depth_relative.empty()) {
        return {};
    }

    const ProjectionParams params = make_projection_params(ego_frame, inferred, scale, cfg);

    // Allocate evidence buffer (shared across projection + surface + thin)
    std::vector<DeviceObstacleEvidence> buf(cfg.max_evidence);
    std::uint32_t count = 0U;

#ifdef DEDALUS_CUDA_ENABLED
    cuda_dispatch().project(inferred.depth_relative.data(), params, buf.data(), count);
#else
    project_depth_to_device_evidence(
        inferred.depth_relative.data(), params, buf.data(), count);
#endif

    if (cfg.detect_surface_patches && count > 0U) {
        std::vector<DeviceObstacleEvidence> patches(64U);
        std::uint32_t patch_count = 0U;
#ifdef DEDALUS_CUDA_ENABLED
        cuda_dispatch().fit_patches(buf.data(), count, params, patches.data(), patch_count);
#else
        fit_surface_patches_device(
            buf.data(), count, params, patches.data(), patch_count);
#endif
        for (std::uint32_t i = 0U; i < patch_count && count < cfg.max_evidence; ++i) {
            buf[count++] = patches[i];
        }
    }

    if (cfg.detect_thin_structures) {
        std::vector<DeviceObstacleEvidence> thin(64U);
        std::uint32_t thin_count = 0U;
#ifdef DEDALUS_CUDA_ENABLED
        cuda_dispatch().detect_thin(
            inferred.depth_relative.data(), params, thin.data(), thin_count);
#else
        detect_thin_structures_device(
            inferred.depth_relative.data(), params, thin.data(), thin_count);
#endif
        for (std::uint32_t i = 0U; i < thin_count && count < cfg.max_evidence; ++i) {
            buf[count++] = thin[i];
        }
    }

    auto result = inflate(
        buf.data(),
        count,
        ego_frame.sensing_volume.camera_name,
        "visual_depth_obstacle_detector",
        ego_frame.ego.map_frame_id,
        ego_frame.frame.timestamp);

    // Compute body-frame bearing and elevation from projected local positions.
    // Uses the sensing volume axes (forward/right/up in local frame) — the same
    // geometry the AirSim GT path uses — so L0 sensor observations are populated.
    const auto& sv = ego_frame.sensing_volume;
    for (auto& ev : result) {
        const double dx = ev.center_local.x - sv.origin_local.x;
        const double dy = ev.center_local.y - sv.origin_local.y;
        const double dz = ev.center_local.z - sv.origin_local.z;
        const double fwd   = dx*sv.forward_axis_local.x + dy*sv.forward_axis_local.y + dz*sv.forward_axis_local.z;
        const double right = dx*sv.right_axis_local.x   + dy*sv.right_axis_local.y   + dz*sv.right_axis_local.z;
        const double up    = dx*sv.up_axis_local.x      + dy*sv.up_axis_local.y      + dz*sv.up_axis_local.z;
        ev.bearing_rad   = static_cast<float>(std::atan2(right, fwd));
        ev.elevation_rad = static_cast<float>(std::atan2(up, std::hypot(fwd, right)));
    }

    return result;
}

}  // namespace dedalus
