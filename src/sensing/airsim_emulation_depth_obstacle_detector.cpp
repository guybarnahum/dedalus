#include "dedalus/sensing/airsim_emulation_depth_obstacle_detector.hpp"

#include <cmath>
#include <cstdint>
#include <vector>

#include "dedalus/sensing/depth_projection_kernel.hpp"

namespace dedalus {
namespace {

// Build ProjectionParams from EgoSensingFrame + GT depth frame dimensions.
ProjectionParams make_params(
    const EgoSensingFrame&                            ego_frame,
    const AirSimDepthFrame&                           df,
    const AirSimEmulationDepthObstacleDetectorConfig& cfg) {

    const auto& sv = ego_frame.sensing_volume;
    ProjectionParams p;

    p.fx = static_cast<float>(ego_frame.frame.intrinsics.fx);
    p.fy = static_cast<float>(ego_frame.frame.intrinsics.fy);
    p.cx = static_cast<float>(ego_frame.frame.intrinsics.cx);
    p.cy = static_cast<float>(ego_frame.frame.intrinsics.cy);
    p.k1 = static_cast<float>(ego_frame.frame.intrinsics.distortion_k1);
    p.k2 = static_cast<float>(ego_frame.frame.intrinsics.distortion_k2);

    p.width  = df.width;
    p.height = df.height;
    p.stride = static_cast<int>(cfg.pixel_stride);

    p.min_depth_m  = cfg.min_depth_m;
    p.max_depth_m  = cfg.max_depth_m;
    p.scale        = cfg.scale;
    p.voxel_size_m = cfg.voxel_size_m;
    p.max_evidence = static_cast<std::uint32_t>(cfg.max_evidence);

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

// Convert GT metric depth (metres) to disparity-convention depth_relative.
// depth_m = scale / depth_relative  →  depth_relative = scale / depth_m
// Returns empty if df.depth_m is empty.
std::vector<float> invert_gt_depth(const AirSimDepthFrame& df, float scale) {
    const std::size_t n = static_cast<std::size_t>(df.width) *
                          static_cast<std::size_t>(df.height);
    if (n == 0U || df.depth_m.size() < n) return {};

    static constexpr float kEpsilon = 1e-6F;
    std::vector<float> rel(n);
    for (std::size_t i = 0U; i < n; ++i) {
        const float dm = df.depth_m[i];
        rel[i] = (dm > kEpsilon) ? (scale / dm) : (scale / kEpsilon);
    }
    return rel;
}

}  // namespace

// ---------------------------------------------------------------------------
// AirSimEmulationDepthObstacleDetector
// ---------------------------------------------------------------------------

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

    // Sensor name filter.
    if (!df.sensor_name.empty() && !sv.camera_name.empty() &&
        df.sensor_name != sv.camera_name) {
        return {};
    }

    const std::vector<float> depth_relative = invert_gt_depth(df, config_.scale);
    if (depth_relative.empty()) return {};

    const ProjectionParams params = make_params(ego_frame, df, config_);

    std::vector<DeviceObstacleEvidence> buf(config_.max_evidence);
    std::uint32_t count = 0U;

    project_depth_to_device_evidence(
        depth_relative.data(), params, buf.data(), count);

    // Landable-surface evaluation via RANSAC surface-patch detection.
    if (config_.detect_surface_patches && count > 0U) {
        std::vector<DeviceObstacleEvidence> patches(64U);
        std::uint32_t patch_count = 0U;
        fit_surface_patches_device(
            buf.data(), count, params, patches.data(), patch_count);
        for (std::uint32_t i = 0U; i < patch_count && count < config_.max_evidence; ++i) {
            buf[count++] = patches[i];
        }
    }

    // Thin-obstacle evaluation via local depth-contrast detection.
    if (config_.detect_thin_structures) {
        std::vector<DeviceObstacleEvidence> thin(64U);
        std::uint32_t thin_count = 0U;
        detect_thin_structures_device(
            depth_relative.data(), params, thin.data(), thin_count);
        for (std::uint32_t i = 0U; i < thin_count && count < config_.max_evidence; ++i) {
            buf[count++] = thin[i];
        }
    }

    auto evidence = inflate(
        buf.data(),
        count,
        df.sensor_name,
        provider_name(),
        ego_frame.ego.map_frame_id,
        ego_frame.frame.timestamp);

    // Override source_kind: inflate() stamps VisualObstacleDetector; this path
    // is AirSim GT geometry flowing through the VD kernels for calibration.
    for (auto& ev : evidence) {
        ev.source_kind = OccupancySourceKind::AirSimGroundTruthVisualEmulation;
    }

    return evidence;
}

}  // namespace dedalus
