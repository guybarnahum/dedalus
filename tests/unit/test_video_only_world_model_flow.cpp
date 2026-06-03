#include <iostream>

#include "dedalus/perception/perception_pipeline.hpp"
#include "dedalus/sensors/ego_state_provider.hpp"
#include "dedalus/sensors/replay_frame_source.hpp"
#include "dedalus/world_model/in_memory_world_model.hpp"

int main() {
    dedalus::VideoOnlyFrameSource frame_source{1U};
    const auto frame = frame_source.next_frame();
    if (!frame.has_value()) {
        std::cerr << "video-only frame source produced no frame\n";
        return 1;
    }

    if (frame->ego_hint.has_value()) {
        std::cerr << "video-only frame unexpectedly carried telemetry\n";
        return 1;
    }

    dedalus::NoTelemetryEgoProvider ego_provider{dedalus::MapFrameId{"map_video_only_0001"}};
    const auto ego_estimate = ego_provider.estimate(*frame);
    if (!ego_estimate.ego.has_value() || ego_estimate.telemetry_available || ego_estimate.confidence >= 0.5F) {
        std::cerr << "no-telemetry ego provider did not report degraded ego state\n";
        return 1;
    }

    const dedalus::EgoState ego = *ego_estimate.ego;

    auto detector         = std::make_shared<dedalus::ScriptedDetector>();
    auto stabilizer        = std::make_shared<dedalus::NullCameraStabilizer>();
    auto tracker           = std::make_shared<dedalus::SimpleCentroidTracker>();
    auto identity_resolver = std::make_shared<dedalus::AppearanceOnlyIdentityResolver>();
    auto projector         = std::make_shared<dedalus::FlatGroundProjector>();
    dedalus::PerceptionPipeline pipeline(detector, stabilizer, tracker, identity_resolver, projector);

    const auto output = pipeline.process(*frame, ego);
    if (output.observations.size() != 1U) {
        std::cerr << "video-only pipeline did not produce expected observation\n";
        return 1;
    }

    dedalus::InMemoryWorldModel world_model(ego.map_frame_id);
    world_model.update_ego(ego);
    if (frame->appearance_condition.has_value()) {
        world_model.update_appearance(*frame->appearance_condition);
    }
    world_model.ingest(output);

    const auto snapshot = world_model.snapshot();
    if (snapshot.active_map_frame_id.value != "map_video_only_0001") {
        std::cerr << "world model did not preserve video-only relative map frame\n";
        return 1;
    }

    if (snapshot.agents.empty() || snapshot.tactical_exclusion_zones.empty() || snapshot.flight_corridors.empty()) {
        std::cerr << "video-only flow failed to emit world-model artifacts\n";
        return 1;
    }

    if (snapshot.appearance_condition.confidence > 0.3F) {
        std::cerr << "video-only appearance condition should remain low confidence\n";
        return 1;
    }

    return 0;
}
