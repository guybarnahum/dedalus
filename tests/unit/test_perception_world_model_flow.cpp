#include <iostream>

#include "dedalus/perception/perception_pipeline.hpp"
#include "dedalus/sensors/frame_source.hpp"
#include "dedalus/world_model/in_memory_world_model.hpp"

int main() {
    dedalus::SyntheticFrameSource frame_source;
    const auto frame = frame_source.next_frame();
    if (!frame.has_value()) {
        std::cerr << "synthetic frame source produced no frame\n";
        return 1;
    }

    if (!frame->ego_hint.has_value()) {
        std::cerr << "synthetic frame has no ego hint\n";
        return 1;
    }

    const dedalus::EgoState ego = *frame->ego_hint;

    dedalus::ScriptedDetector detector;
    dedalus::SimpleCentroidTracker tracker;
    dedalus::AppearanceOnlyIdentityResolver identity_resolver;
    dedalus::FlatGroundProjector projector;
    dedalus::PerceptionPipeline pipeline(detector, tracker, identity_resolver, projector);

    const auto output = pipeline.process(*frame, ego);
    if (output.detections.size() != 1U || output.tracks.size() != 1U ||
        output.identities.size() != 1U || output.observations.size() != 1U) {
        std::cerr << "unexpected pipeline output cardinality\n";
        return 1;
    }

    dedalus::InMemoryWorldModel world_model(ego.map_frame_id);
    world_model.update_ego(ego);
    if (frame->appearance_condition.has_value()) {
        world_model.update_appearance(*frame->appearance_condition);
    }
    world_model.ingest(output);

    const auto snapshot = world_model.snapshot();
    if (snapshot.agents.size() != 1U) {
        std::cerr << "world model did not emit expected agent\n";
        return 1;
    }

    if (snapshot.tactical_exclusion_zones.size() != 1U) {
        std::cerr << "world model did not emit expected tactical exclusion zone\n";
        return 1;
    }

    if (snapshot.tactical_exclusion_zones.front().reason != "dynamic_observation_cone") {
        std::cerr << "unexpected tactical exclusion zone reason\n";
        return 1;
    }

    const auto view = world_model.effective_view();
    if (view.actual.agents.size() != 1U || view.uncertain_regions.empty()) {
        std::cerr << "effective view missing actual agent or uncertainty state\n";
        return 1;
    }

    return 0;
}
