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
    // Normals are camera-frame unit vectors using the same axis convention as
    // the detector's back-projection: +x=forward, +y=right, +z=up. The detector
    // transforms them into the sensing-volume local/map frame before publishing
    // ObstacleEvidence.
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
