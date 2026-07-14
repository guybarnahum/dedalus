#include "dedalus/sensing/airsim_emulation_depth_obstacle_detector.hpp"

#include <cmath>
#include <cstdint>
#include <vector>

#include "dedalus/sensing/depth_projection_kernel.hpp"

namespace dedalus {

AirSimEmulationDepthObstacleDetector::AirSimEmulationDepthObstacleDetector(
    AirSimEmulationDepthObstacleDetectorConfig config)
    : config_(std::move(config)) {}

std::string AirSimEmulationDepthObstacleDetector::provider_name() const {
    return "airsim_gt_vd";
}

std::vector<ObstacleEvidence> AirSimEmulationDepthObstacleDetector::detect(
    const EgoSensingFrame& ego_frame) {

    if (!ego_frame.frame.depth_frame.has_value()) return {};

    const auto& df = *ego_frame.frame.depth_frame;
    const auto& sv = ego_frame.sensing_volume;

    // Sensor name filter — AirSim bridge topology: one GT stream per camera.
    if (!df.sensor_name.empty() && !sv.camera_name.empty() &&
        df.sensor_name != sv.camera_name) {
        last_has_result_ = false;
        return {};
    }

    const std::size_t n =
        static_cast<std::size_t>(df.width) * static_cast<std::size_t>(df.height);
    if (n == 0U || df.depth_m.size() < n) {
        last_has_result_ = false;
        return {};
    }

    // ── Build DepthPipelineInput ──────────────────────────────────────────
    DepthPipelineInput input;

    // Convert metric depth → disparity convention.
    input.inverse_depth = metric_to_inverse_depth(
        df.depth_m.data(), n, config_.scale);
    if (input.inverse_depth.empty()) { last_has_result_ = false; return {}; }

    input.width  = df.width;
    input.height = df.height;
    input.scale  = config_.scale;

    // FoV-based intrinsics (sensing volume is the ground-truth source):
    //   cx = (W-1)/2, fx = cx / tan(hfov/2)  — symmetric camera convention.
    // image.width is intentionally unused: AirSim frames may not carry an RGB
    // image, but the depth frame dims and sensing volume FoV are always valid.
    const float hfov = static_cast<float>(sv.horizontal_fov_rad);
    const float vfov = static_cast<float>(sv.vertical_fov_rad);
    input.cx = (static_cast<float>(df.width)  - 1.0F) * 0.5F;
    input.cy = (static_cast<float>(df.height) - 1.0F) * 0.5F;
    input.fx = input.cx / std::tan(hfov * 0.5F);
    input.fy = input.cy / std::tan(vfov * 0.5F);
    input.k1 = static_cast<float>(ego_frame.frame.intrinsics.distortion_k1);
    input.k2 = static_cast<float>(ego_frame.frame.intrinsics.distortion_k2);

    input.source_kind     = OccupancySourceKind::AirSimGroundTruthVisualEmulation;
    input.sensor_name     = df.sensor_name;
    input.source_provider = provider_name();

    last_input_      = input;
    last_has_result_ = true;

    // ── Delegate all downstream processing to the shared pipeline ─────────
    const DepthPipelineConfig cfg{
        static_cast<int>(config_.depth_grid_cols),
        static_cast<int>(config_.depth_grid_rows),
        config_.min_depth_m,
        config_.max_depth_m,
        config_.voxel_size_m,
        config_.max_evidence,
        config_.detect_surface_patches,
        config_.detect_thin_structures,
    };

    return run_depth_pipeline(input, ego_frame, cfg, &last_params_);
}

}  // namespace dedalus
