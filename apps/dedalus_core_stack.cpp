#include <iostream>

#include "dedalus/perception/perception_pipeline.hpp"
#include "dedalus/sensors/frame_source.hpp"
#include "dedalus/world_model/in_memory_world_model.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

int main() {
    dedalus::SyntheticFrameSource frame_source;
    const auto frame = frame_source.next_frame();
    if (!frame.has_value()) {
        return 1;
    }

    dedalus::EgoState ego;
    if (frame->ego_hint.has_value()) {
        ego = *frame->ego_hint;
    } else {
        ego.timestamp = frame->timestamp;
        ego.map_frame_id = dedalus::MapFrameId{"map_local_0001"};
    }

    dedalus::ScriptedDetector detector;
    dedalus::SimpleCentroidTracker tracker;
    dedalus::AppearanceOnlyIdentityResolver identity_resolver;
    dedalus::FlatGroundProjector projector;

    dedalus::PerceptionPipeline pipeline(detector, tracker, identity_resolver, projector);
    const auto perception_output = pipeline.process(*frame, ego);

    dedalus::InMemoryWorldModel world_model(ego.map_frame_id);
    world_model.update_ego(ego);
    if (frame->appearance_condition.has_value()) {
        world_model.update_appearance(*frame->appearance_condition);
    }
    world_model.ingest(perception_output);

    std::cout << dedalus::to_json(world_model.snapshot());
    return 0;
}
