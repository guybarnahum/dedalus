#pragma once

#include "dedalus/sensing/airsim_depth_obstacle_detector.hpp"
#include "dedalus/sensing/obstacle_evidence_provider.hpp"

namespace dedalus {

// Adapts AirSimDepthObstacleDetector to the ObstacleEvidenceProvider interface.
//
// Extracts the AirSim GT depth frame from EgoSensingFrame::frame.depth_frame.
// Returns empty if no depth frame is present or sensor names don't match.
// The sensor-name matching that was previously in CoreStackRunner::run_once()
// lives here so that slot-selection logic stays in one place.
class AirSimDepthEvidenceProvider final : public ObstacleEvidenceProvider {
public:
    explicit AirSimDepthEvidenceProvider(
        AirSimDepthObstacleDetectorConfig config = {});

    [[nodiscard]] std::string provider_name() const override;

    [[nodiscard]] std::vector<ObstacleEvidence> detect(
        const EgoSensingFrame& frame) override;

private:
    AirSimDepthObstacleDetector detector_;
};

}  // namespace dedalus
