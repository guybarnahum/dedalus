#include "dedalus/sensing/depth_projection_kernel.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_set>
#include <vector>

namespace dedalus {
namespace {

// Pack voxel index triple into a single uint64 for deduplication.
// Supports ±1 048 576 indices in each dimension (±524 km at 0.5 m resolution).
std::uint64_t pack_voxel(std::int32_t ix, std::int32_t iy, std::int32_t iz) {
    const std::uint64_t ux = static_cast<std::uint64_t>(ix + 1048576) & 0x1FFFFFU;
    const std::uint64_t uy = static_cast<std::uint64_t>(iy + 1048576) & 0x1FFFFFU;
    const std::uint64_t uz = static_cast<std::uint64_t>(iz + 1048576) & 0x1FFFFFU;
    return (ux << 42U) | (uy << 21U) | uz;
}

// Undistort a normalised image coordinate using Brown-Conrady k1, k2.
// One Newton step is sufficient for small distortion values.
void undistort_k1k2(float xn, float yn, float k1, float k2, float& xd, float& yd) {
    const float r2     = xn * xn + yn * yn;
    const float radial = 1.0F + k1 * r2 + k2 * r2 * r2;
    xd = xn / radial;
    yd = yn / radial;
}

}  // namespace

// ---------------------------------------------------------------------------
// CPU projection kernel
// ---------------------------------------------------------------------------

void project_depth_to_device_evidence(
    const float*             depth_relative,
    const ProjectionParams&  params,
    DeviceObstacleEvidence*  out,
    std::uint32_t&           count_out) {

    count_out = 0;

    if (params.fx <= 0.0F || params.fy <= 0.0F || params.scale <= 0.0F) {
        return;
    }

    const float inv_fx = 1.0F / params.fx;
    const float inv_fy = 1.0F / params.fy;

    std::unordered_set<std::uint64_t> seen;
    seen.reserve(static_cast<std::size_t>(params.max_evidence));

    for (int v = 0; v < params.height; v += params.stride) {
        for (int u = 0; u < params.width; u += params.stride) {
            if (count_out >= params.max_evidence) { goto done; }

            const float dr = depth_relative[
                static_cast<std::size_t>(v) * static_cast<std::size_t>(params.width) +
                static_cast<std::size_t>(u)];

            if (!std::isfinite(dr) || dr <= 0.0F) { continue; }

            const float depth_m = params.scale / dr;
            if (depth_m < params.min_depth_m || depth_m > params.max_depth_m) { continue; }

            // Normalised image coords + radial undistortion
            const float xn = (static_cast<float>(u) - params.cx) * inv_fx;
            const float yn = (static_cast<float>(v) - params.cy) * inv_fy;
            float xd = xn;
            float yd = yn;
            if (params.k1 != 0.0F || params.k2 != 0.0F) {
                undistort_k1k2(xn, yn, params.k1, params.k2, xd, yd);
            }

            // Camera-space point (Z forward, X right, Y down)
            const float xc = xd * depth_m;
            const float yc = yd * depth_m;
            const float zc = depth_m;

            // Transform to local frame via sensing-volume axes:
            //   local = origin + zc * forward + xc * right - yc * up
            // (camera Y is down; up_axis_local is the "up" direction in local frame)
            const float lx = params.origin_x
                           + zc * params.forward_x
                           + xc * params.right_x
                           - yc * params.up_x;
            const float ly = params.origin_y
                           + zc * params.forward_y
                           + xc * params.right_y
                           - yc * params.up_y;
            const float lz = params.origin_z
                           + zc * params.forward_z
                           + xc * params.right_z
                           - yc * params.up_z;

            // Snap to voxel grid and deduplicate
            const auto ix = static_cast<std::int32_t>(std::floor(lx / params.voxel_size_m));
            const auto iy = static_cast<std::int32_t>(std::floor(ly / params.voxel_size_m));
            const auto iz = static_cast<std::int32_t>(std::floor(lz / params.voxel_size_m));

            if (!seen.insert(pack_voxel(ix, iy, iz)).second) { continue; }

            // Voxel centre in local frame
            const float half = 0.5F * params.voxel_size_m;
            const float cx   = (static_cast<float>(ix) + 0.5F) * params.voxel_size_m;
            const float cy_v = (static_cast<float>(iy) + 0.5F) * params.voxel_size_m;
            const float cz   = (static_cast<float>(iz) + 0.5F) * params.voxel_size_m;

            DeviceObstacleEvidence& ev = out[count_out++];
            ev.center_x = cx;
            ev.center_y = cy_v;
            ev.center_z = cz;
            ev.size_x   = half * 2.0F;
            ev.size_y   = half * 2.0F;
            ev.size_z   = half * 2.0F;
            ev.normal_x = 0.0F;
            ev.normal_y = 0.0F;
            ev.normal_z = 0.0F;
            ev.confidence             = 0.75F;
            ev.range_m                = depth_m;
            ev.state                  = 2U;  // ObstacleEvidenceState::Occupied
            ev.shape                  = 0U;  // ObstacleEvidenceShape::Voxel
            ev.is_thin_structure_hint = 0U;
            ev.is_surface_hint        = 0U;
        }
    }

done:;
}

// ---------------------------------------------------------------------------
// Surface patch detection — CPU stub (real RANSAC implemented at VD3)
// ---------------------------------------------------------------------------

void fit_surface_patches_device(
    const DeviceObstacleEvidence* /*evidence*/,
    std::uint32_t                 /*count*/,
    const ProjectionParams&       /*params*/,
    DeviceObstacleEvidence*       /*patches_out*/,
    std::uint32_t&                patches_count_out) {
    patches_count_out = 0;
}

// ---------------------------------------------------------------------------
// Thin structure detection — CPU stub (Sobel+NMS+CC implemented at VD3)
// ---------------------------------------------------------------------------

void detect_thin_structures_device(
    const float*            /*depth_relative*/,
    const ProjectionParams& /*params*/,
    DeviceObstacleEvidence* /*thin_out*/,
    std::uint32_t&          thin_count_out) {
    thin_count_out = 0;
}

// ---------------------------------------------------------------------------
// inflate: stamp string fields onto DeviceObstacleEvidence[] → ObstacleEvidence[]
// ---------------------------------------------------------------------------

std::vector<ObstacleEvidence> inflate(
    const DeviceObstacleEvidence* evidence,
    std::uint32_t                 count,
    const std::string&            sensor_name,
    const std::string&            source_provider,
    const MapFrameId&             map_frame_id,
    TimePoint                     timestamp) {

    std::vector<ObstacleEvidence> result;
    result.reserve(count);

    for (std::uint32_t i = 0; i < count; ++i) {
        const DeviceObstacleEvidence& src = evidence[i];
        ObstacleEvidence ev;

        ev.timestamp       = timestamp;
        ev.sensor_name     = sensor_name;
        ev.source_provider = source_provider;
        ev.source_kind     = OccupancySourceKind::VisualObstacleDetector;
        ev.map_frame_id    = map_frame_id;

        ev.state = static_cast<ObstacleEvidenceState>(src.state);
        ev.shape = static_cast<ObstacleEvidenceShape>(src.shape);

        ev.center_local = Vec3{
            static_cast<double>(src.center_x),
            static_cast<double>(src.center_y),
            static_cast<double>(src.center_z)};
        ev.size_m = Vec3{
            static_cast<double>(src.size_x),
            static_cast<double>(src.size_y),
            static_cast<double>(src.size_z)};

        const float nsq = src.normal_x * src.normal_x
                        + src.normal_y * src.normal_y
                        + src.normal_z * src.normal_z;
        ev.has_surface_normal = nsq > 0.5F;
        if (ev.has_surface_normal) {
            ev.surface_normal_local = Vec3{
                static_cast<double>(src.normal_x),
                static_cast<double>(src.normal_y),
                static_cast<double>(src.normal_z)};
            ev.normal_confidence = src.confidence;
        }

        ev.confidence              = static_cast<float>(src.confidence);
        ev.occupancy_probability   = src.confidence;
        ev.range_m                 = src.range_m;
        ev.is_thin_structure_hint  = src.is_thin_structure_hint != 0U;
        ev.is_surface_hint         = src.is_surface_hint != 0U;
        ev.inside_sensing_volume   = true;

        result.push_back(std::move(ev));
    }

    return result;
}

}  // namespace dedalus
