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

    // Returns the AirSimDepthFrame used by the most recent detect() call.
    // nullptr if detect() has never been called or no depth frame was present.
    // Pointer is valid until the next detect() call.
    [[nodiscard]] const AirSimDepthFrame* last_depth_frame() const {
        return last_frame_valid_ ? &last_frame_ : nullptr;
    }

private:
    AirSimDepthObstacleDetector detector_;
    AirSimDepthFrame            last_frame_;
    bool                        last_frame_valid_{false};
};

}  // namespace dedalus
