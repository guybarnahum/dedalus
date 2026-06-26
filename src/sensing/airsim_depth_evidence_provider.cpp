#include "dedalus/sensing/airsim_depth_evidence_provider.hpp"

namespace dedalus {

AirSimDepthEvidenceProvider::AirSimDepthEvidenceProvider(
    AirSimDepthObstacleDetectorConfig config)
    : detector_(std::move(config)) {}

std::string AirSimDepthEvidenceProvider::provider_name() const {
    return "airsim_depth_gt";
}

std::vector<ObstacleEvidence> AirSimDepthEvidenceProvider::detect(
    const EgoSensingFrame& ego_frame) {
    if (!ego_frame.frame.depth_frame.has_value()) return {};

    const auto& df = *ego_frame.frame.depth_frame;
    const auto& sv = ego_frame.sensing_volume;

    // Sensor name filter — preserves the matching semantics previously
    // handled inline in CoreStackRunner::run_once().
    if (!df.sensor_name.empty() && !sv.camera_name.empty() &&
        df.sensor_name != sv.camera_name) {
        return {};
    }

    return detector_.detect(df, to_obstacle_sensing_volume(sv));
}

}  // namespace dedalus
