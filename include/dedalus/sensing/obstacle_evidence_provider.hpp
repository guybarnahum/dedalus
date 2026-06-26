#pragma once

#include <string>
#include <vector>

#include "dedalus/occupancy/occupancy_types.hpp"
#include "dedalus/sensing/sensing_coverage.hpp"

namespace dedalus {

// ObstacleEvidenceProvider: interface for obstacle depth providers.
//
// detect() receives an EgoSensingFrame (frame + ego + CameraSensingVolume)
// and returns ObstacleEvidence.  Providers that need ObstacleSensingVolume
// may call to_obstacle_sensing_volume() from sensing_coverage.hpp.
//
// EgoSensingFrame is defined in sensing_coverage.hpp — do not redefine it.
//
// Providers must emit ObstacleEvidence only.  They must not write world-model
// state, local flight maps, mission-local maps, or visualization artifacts.
class ObstacleEvidenceProvider {
public:
    virtual ~ObstacleEvidenceProvider() = default;

    [[nodiscard]] virtual std::string provider_name() const = 0;

    [[nodiscard]] virtual std::vector<ObstacleEvidence> detect(
        const EgoSensingFrame& frame) = 0;
};

}  // namespace dedalus
