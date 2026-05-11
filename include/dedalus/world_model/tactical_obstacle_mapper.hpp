#pragma once

#include <vector>

#include "dedalus/perception/types.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

namespace dedalus {

class TacticalObstacleMapper {
public:
    virtual ~TacticalObstacleMapper() = default;
    virtual std::vector<ExclusionZone> map(
        const std::vector<Observation3D>& observations,
        TimePoint now,
        MapFrameId map_frame_id) = 0;
};

class ConeExclusionMapper final : public TacticalObstacleMapper {
public:
    std::vector<ExclusionZone> map(
        const std::vector<Observation3D>& observations,
        TimePoint now,
        MapFrameId map_frame_id) override;
};

}  // namespace dedalus
