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
    std::vector<float> depth_m;};

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
