#pragma once

#include <cstddef>
#include <vector>

#include "dedalus/occupancy/occupancy_types.hpp"

namespace dedalus {

struct AirSimDepthFrame {
    TimePoint timestamp;
    FrameId source_frame_id;
    bool has_source_frame{false};
    std::string sensor_name{"front_center"};
    MapFrameId map_frame_id{"map_unknown"};

    int width{0};
    int height{0};
    std::vector<float> depth_m;

    // Optional surface normals sampled on the same grid as depth_m.
    //
    // These come from AirSim's rendered SurfaceNormals image target and are kept
    // as optional debug/provenance data. Published obstacle evidence normals are
    // derived from neighboring back-projected depth samples in local coordinates
    // by default, avoiding AirSim/Unreal normal-frame ambiguity.
    bool has_surface_normals{false};
    std::vector<float> surface_normal_camera_xyz;
};

struct AirSimDepthObstacleDetectorConfig {
    // Detector-side stride over the already-acquired depth frame.
    //
    // AirSim live transport may downsample the full DepthPlanar image before it
    // enters CoreStackRunner. The detector default must consume every sample it
    // receives; upstream acquisition controls transport density. Raise this
    // only for an intentional second decimation pass.
    std::size_t pixel_stride{1U};
    float min_depth_m{0.2F};
    float max_depth_m{80.0F};
    float voxel_size_m{0.75F};
    float confidence{0.75F};
    std::size_t max_evidence{512U};
    float normal_confidence{0.85F};

    // Prefer local-frame normals derived from neighboring back-projected depth
    // samples. This keeps the normal in the same frame as center_local by
    // construction and avoids depending on AirSim/Unreal SurfaceNormals channel,
    // sign, or coordinate-frame conventions.
    bool derive_surface_normals_from_depth{true};

    // Diagnostic-only fallback. Disabled by default because AirSim
    // SurfaceNormals are a rendered image product and their coordinate frame is
    // not part of the detector contract. Enable only for explicit comparison
    // experiments.
    bool use_airsim_surface_normals{false};

    // Display/planning-facing evidence coalescing.
    //
    // The AirSim bridge samples camera depth/normals on an image grid. Without
    // coalescing, large continuous surfaces such as ground during landing emit
    // one SurfacePatch per sampled pixel. When enabled, the detector aggregates
    // neighboring samples into local-space buckets of voxel_size_m and emits one
    // averaged SurfacePatch per bucket without changing the upstream sidecar.
    bool coalesce_surface_patches{false};
};

class AirSimDepthObstacleDetector {
public:
    explicit AirSimDepthObstacleDetector(AirSimDepthObstacleDetectorConfig config = {});

    [[nodiscard]] std::vector<ObstacleEvidence> detect(
        const AirSimDepthFrame& frame,
        const ObstacleSensingVolume& sensing_volume) const;

private:
    AirSimDepthObstacleDetectorConfig config_;
};

[[nodiscard]] std::vector<ObstacleEvidence> detect_airsim_depth_obstacles(
    const AirSimDepthFrame& frame,
    const ObstacleSensingVolume& sensing_volume,
    const AirSimDepthObstacleDetectorConfig& config = {});

}  // namespace dedalus
