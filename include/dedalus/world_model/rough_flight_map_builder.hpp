#pragma once

#include <vector>

#include "dedalus/perception/types.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

namespace dedalus {

struct RoughFlightMapUpdate {
    std::vector<StaticStructure> static_structures;
    std::vector<FlightCorridor> flight_corridors;
    std::vector<Landmark> landmarks;
};

class RoughFlightMapBuilder {
public:
    RoughFlightMapUpdate build(
        const std::vector<Observation3D>& observations,
        const EgoState& ego,
        TimePoint now,
        MapFrameId map_frame_id) const;
};

}  // namespace dedalus
