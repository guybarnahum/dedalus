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

    // Compute intrinsics directly from the sensing volume FoV + depth frame
    // dimensions.  This is the ground-truth path: the sensing volume always
    // carries the correct angular coverage (validated by
    // validate_obstacle_sensing_cameras()), and the depth frame dims come from
    // the bridge sidecar.  No dependency on ego_frame.frame.image.width/height,
    // which may be 0 in code paths that don't attach a full RGB image.
    //
    // For the front-center AirSim camera at 640×360 with FOV_Degrees=84 and
    // an N×M grid of 40×22, this gives: fx = fy ≈ 22.2, cx=20, cy=11.
    const float hfov = static_cast<float>(sv.horizontal_fov_rad);
    const float vfov = static_cast<float>(sv.vertical_fov_rad);
    p.fx = (static_cast<float>(df.width)  * 0.5F) / std::tan(hfov * 0.5F);
    p.fy = (static_cast<float>(df.height) * 0.5F) / std::tan(vfov * 0.5F);
    p.cx = static_cast<float>(df.width)  * 0.5F;
    p.cy = static_cast<float>(df.height) * 0.5F;
    p.k1 = static_cast<float>(ego_frame.frame.intrinsics.distortion_k1);
    p.k2 = static_cast<float>(ego_frame.frame.intrinsics.distortion_k2);

    p.width     = df.width;
    p.height    = df.height;
    p.grid_cols = static_cast<int>(cfg.depth_grid_cols);
    p.grid_rows = static_cast<int>(cfg.depth_grid_rows);

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

// Convert GT metric depth (metres) to disparity-convention inverse_depth.
// depth_m = scale / inverse_depth  →  inverse_depth = scale / depth_m
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

    last_frame_       = df;
    last_frame_valid_ = true;

    const std::vector<float> inverse_depth = invert_gt_depth(df, config_.scale);
    if (inverse_depth.empty()) return {};

    const ProjectionParams params = make_params(ego_frame, df, config_);
    last_params_ = params;

    std::vector<DeviceObstacleEvidence> buf(config_.max_evidence);
    std::uint32_t count = 0U;

    project_depth_to_device_evidence(
        inverse_depth.data(), params, buf.data(), count);

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
            inverse_depth.data(), params, thin.data(), thin_count);
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

    // Compute body-frame bearing and elevation from projected local positions.
    // inflate() leaves bearing_rad/elevation_rad at 0; fill them here so that
    // collect_l0_sensor_observations() can populate the sensor cone scope inset.
    for (auto& ev : evidence) {
        const double dx    = ev.center_local.x - sv.origin_local.x;
        const double dy    = ev.center_local.y - sv.origin_local.y;
        const double dz    = ev.center_local.z - sv.origin_local.z;
        const double fwd   = dx*sv.forward_axis_local.x + dy*sv.forward_axis_local.y + dz*sv.forward_axis_local.z;
        const double right = dx*sv.right_axis_local.x   + dy*sv.right_axis_local.y   + dz*sv.right_axis_local.z;
        const double up    = dx*sv.up_axis_local.x      + dy*sv.up_axis_local.y      + dz*sv.up_axis_local.z;
        ev.bearing_rad   = static_cast<float>(std::atan2(right, fwd));
        ev.elevation_rad = static_cast<float>(std::atan2(up, std::hypot(fwd, right)));
        ev.source_kind   = OccupancySourceKind::AirSimGroundTruthVisualEmulation;
    }

    return evidence;
}

}  // namespace dedalus
