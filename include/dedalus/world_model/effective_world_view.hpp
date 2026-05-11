#pragma once

#include <string>
#include <vector>

#include "dedalus/world_model/world_snapshot.hpp"

namespace dedalus {

struct MapConflict {
    std::string conflict_id;
    std::string reason;
    float confidence{0.0F};
};

struct WorldMemorySnapshot {
    std::vector<MapFrame> map_frames;
    std::vector<StaticStructure> static_structures;
    std::vector<Landmark> landmarks;
    float confidence{0.0F};
};

struct EffectiveWorldView {
    WorldSnapshot actual;
    WorldMemorySnapshot memory;
    std::vector<MapConflict> conflicts;
    std::vector<UncertainRegion> uncertain_regions;
};

}  // namespace dedalus
