#pragma once

#include "dedalus/perception/perception_pipeline.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

namespace dedalus {

class InMemoryWorldModel {
public:
    explicit InMemoryWorldModel(MapFrameId map_frame_id = MapFrameId{"map_local_0001"});

    void update_ego(const EgoState& ego);
    void update_appearance(const AppearanceCondition& appearance_condition);
    void ingest(const PerceptionPipelineOutput& perception_output);

    WorldSnapshot snapshot() const;

private:
    WorldSnapshot snapshot_;
};

}  // namespace dedalus
