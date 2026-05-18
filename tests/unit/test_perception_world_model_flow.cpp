#include <cmath>
#include <iostream>

#include "dedalus/perception/perception_pipeline.hpp"
#include "dedalus/sensors/frame_source.hpp"
#include "dedalus/world_model/in_memory_world_model.hpp"

namespace {

bool near(double lhs, double rhs) {
    return std::abs(lhs - rhs) < 1.0e-9;
}

}  // namespace

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
    dedalus::NullCameraStabilizer stabilizer;
    dedalus::SimpleCentroidTracker tracker;
    dedalus::AppearanceOnlyIdentityResolver identity_resolver;
    dedalus::FlatGroundProjector projector;
    dedalus::PerceptionPipeline pipeline(detector, stabilizer, tracker, identity_resolver, projector);

    auto output = pipeline.process(*frame, ego);
    if (output.detections.size() != 1U || output.stabilized_frame.detections.size() != 1U ||
        output.tracks.size() != 1U || output.identities.size() != 1U || output.observations.size() != 1U) {
        std::cerr << "unexpected pipeline output cardinality\n";
        return 1;
    }

    if (output.stabilized_frame.transform_available) {
        std::cerr << "null stabilizer unexpectedly reported a transform\n";
        return 1;
    }

    const auto first_track_id = output.observations.front().track_id;
    auto second_observation = output.observations.front();
    second_observation.track_id = dedalus::TrackId{"track_extra_0002"};
    second_observation.position_local = dedalus::Vec3{9.0, 2.0, -1.0};
    second_observation.confidence = 0.77F;
    output.observations.push_back(second_observation);

    dedalus::InMemoryWorldModel world_model(ego.map_frame_id);
    world_model.update_ego(ego);
    if (frame->appearance_condition.has_value()) {
        world_model.update_appearance(*frame->appearance_condition);
    }
    world_model.ingest(output);

    const auto snapshot = world_model.snapshot();
    if (snapshot.ego.map_frame_id.value != ego.map_frame_id.value) {
        std::cerr << "world model ego map frame did not match input ego\n";
        return 1;
    }
    if (!snapshot.ego.height_valid || !near(snapshot.ego.height_m, -ego.local_T_body.position.z)) {
        std::cerr << "world model did not derive valid ego height from local pose\n";
        return 1;
    }
    if (snapshot.ego.flight_status != dedalus::EgoFlightStatus::Airborne) {
        std::cerr << "world model did not derive airborne ego status\n";
        return 1;
    }
    if (!snapshot.ego.home_T_body.has_value() || !snapshot.ego.home_timestamp.has_value()) {
        std::cerr << "world model did not preserve ego home pose/timestamp\n";
        return 1;
    }
    if (snapshot.ego.confidence <= 0.0F) {
        std::cerr << "world model did not expose ego confidence\n";
        return 1;
    }

    if (snapshot.agents.size() != 2U) {
        std::cerr << "world model did not emit expected agents\n";
        return 1;
    }

    if (snapshot.agents[0].source_track_id.value != first_track_id.value) {
        std::cerr << "first agent did not preserve source track id\n";
        return 1;
    }
    if (snapshot.agents[0].agent_id.value != "agent_" + first_track_id.value) {
        std::cerr << "first agent id was not derived from source track id\n";
        return 1;
    }
    if (snapshot.agents[0].identity_id.value != "identity_" + first_track_id.value) {
        std::cerr << "first identity id was not derived from source track id\n";
        return 1;
    }
    if (snapshot.agents[1].source_track_id.value != "track_extra_0002") {
        std::cerr << "second agent did not preserve source track id\n";
        return 1;
    }
    if (snapshot.agents[1].agent_id.value != "agent_track_extra_0002") {
        std::cerr << "second agent id was not derived from source track id\n";
        return 1;
    }
    if (snapshot.agents[0].agent_id.value == snapshot.agents[1].agent_id.value) {
        std::cerr << "agents from different tracks must not share an agent id\n";
        return 1;
    }

    if (snapshot.tactical_exclusion_zones.size() != 2U) {
        std::cerr << "world model did not emit expected tactical exclusion zones\n";
        return 1;
    }

    if (snapshot.tactical_exclusion_zones.front().reason != "dynamic_observation_cone") {
        std::cerr << "unexpected tactical exclusion zone reason\n";
        return 1;
    }

    if (snapshot.static_structures.empty() || snapshot.static_structures.front().type != "building") {
        std::cerr << "world model did not emit expected static structure artifact\n";
        return 1;
    }

    if (snapshot.flight_corridors.size() != 1U || snapshot.flight_corridors.front().corridor_id.value.empty()) {
        std::cerr << "world model did not emit expected flight corridor\n";
        return 1;
    }

    if (snapshot.landmarks.empty() || snapshot.landmarks.front().type != "building_corner") {
        std::cerr << "world model did not emit expected landmark artifact\n";
        return 1;
    }

    const auto view = world_model.effective_view();
    if (view.actual.agents.size() != 2U || view.uncertain_regions.empty()) {
        std::cerr << "effective view missing actual agents or uncertainty state\n";
        return 1;
    }

    if (view.actual.flight_corridors.empty() || view.actual.landmarks.empty()) {
        std::cerr << "effective view missing rough flight-map artifacts\n";
        return 1;
    }

    return 0;
}
