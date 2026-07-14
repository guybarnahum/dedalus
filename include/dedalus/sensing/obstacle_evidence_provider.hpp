#pragma once

#include <string>
#include <vector>

#include "dedalus/occupancy/occupancy_types.hpp"
#include "dedalus/sensing/depth_projection_kernel.hpp"
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
//
// Depth providers additionally implement the introspection accessors below
// to enable the debug annotator and SSE stamp block without typed casts.
// Non-depth providers use the default no-op implementations.
class ObstacleEvidenceProvider {
public:
    virtual ~ObstacleEvidenceProvider() = default;

    [[nodiscard]] virtual std::string provider_name() const = 0;

    [[nodiscard]] virtual std::vector<ObstacleEvidence> detect(
        const EgoSensingFrame& frame) = 0;

    // ── Debug annotator / SSE introspection ────────────────────────────────
    // Valid after each detect() call on depth providers; nullptr / zero / empty
    // on non-depth providers or before the first detect() call.
    [[nodiscard]] virtual const float*     last_inverse_depth() const { return nullptr; }
    [[nodiscard]] virtual int              last_depth_width()   const { return 0; }
    [[nodiscard]] virtual int              last_depth_height()  const { return 0; }
    [[nodiscard]] virtual ProjectionParams last_params()        const { return {}; }
    // Camera pitch from ego sensing volume (NED: positive = looking downward).
    // Depth providers with a pitch estimate override this; others return 0.
    [[nodiscard]] virtual float            last_pitch_deg()     const { return 0.0f; }
};

}  // namespace dedalus
