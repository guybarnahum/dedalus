#pragma once

#include <string>
#include <vector>

#include "dedalus/core/types.hpp"
#include "dedalus/occupancy/occupancy_types.hpp"
#include "dedalus/perception/perception_pipeline.hpp"
#include "dedalus/sensors/frame_source.hpp"

namespace dedalus {

// EgoSensingFrame is the provider-neutral obstacle-sensing input contract.
//
// It intentionally reuses existing contracts:
//   - FramePacket: RGB/depth/intrinsics/extrinsics/ego hint.
//   - EgoState: current local_T_body/map_frame_id.
//   - ObstacleSensingVolume: current camera/sensing volume in the active local frame.
//
// Providers must emit ObstacleEvidence only. They must not write world-model
// state, local flight maps, mission-local maps, or visualization artifacts.
struct EgoSensingFrame {
    FramePacket frame;
    EgoState ego;
    ObstacleSensingVolume sensing_volume;

    bool has_depth_frame{false};
    bool has_camera_extrinsics{false};
};

class ObstacleEvidenceProvider {
public:
    virtual ~ObstacleEvidenceProvider() = default;

    virtual std::string provider_name() const = 0;

    virtual std::vector<ObstacleEvidence> detect(const EgoSensingFrame& frame) = 0;
};

}  // namespace dedalus
