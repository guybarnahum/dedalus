#include "dedalus/sensing/airsim_depth_obstacle_detector.hpp"

namespace dedalus {

AirSimDepthObstacleDetector::AirSimDepthObstacleDetector(AirSimDepthObstacleDetectorConfig config)
    : config_(config) {}

std::vector<ObstacleEvidence> AirSimDepthObstacleDetector::detect(
    const AirSimDepthFrame& frame,
    const ObstacleSensingVolume& sensing_volume) const {
    return detect_airsim_depth_obstacles(frame, sensing_volume, config_);
}

}  // namespace dedalus
