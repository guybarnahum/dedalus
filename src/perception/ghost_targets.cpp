#include "dedalus/perception/ghost_targets.hpp"

namespace dedalus {
namespace {

constexpr double kNsPerSecond = 1000000000.0;

double elapsed_seconds(TimePoint timestamp, TimePoint scenario_start) {
    const auto delta_ns = timestamp.timestamp_ns - scenario_start.timestamp_ns;
    if (delta_ns <= 0) {
        return 0.0;
    }
    return static_cast<double>(delta_ns) / kNsPerSecond;
}

}  // namespace

GhostTargetProvider::GhostTargetProvider(GhostScenario scenario)
    : scenario_(std::move(scenario)) {}

std::vector<Observation3D> GhostTargetProvider::observations_at(
    TimePoint timestamp,
    MapFrameId map_frame_id,
    TimePoint scenario_start) const {
    const double elapsed_s = elapsed_seconds(timestamp, scenario_start);
    const auto states = scenario_.evaluate(elapsed_s);
    std::vector<Observation3D> observations;
    observations.reserve(states.size());

    for (const auto& state : states) {
        Observation3D observation;
        observation.track_id = state.source_track_id;
        observation.timestamp = timestamp;
        observation.position_local = state.position_local_m;
        observation.position_body = state.position_local_m;
        observation.map_frame_id = map_frame_id;
        observation.class_label = state.class_label;
        observation.faction = FactionLabel::Unknown;
        observation.confidence = static_cast<float>(state.confidence);
        observations.push_back(observation);
    }

    return observations;
}

PerceptionPipelineOutput GhostTargetProvider::output_at(
    TimePoint timestamp,
    MapFrameId map_frame_id,
    TimePoint scenario_start) const {
    PerceptionPipelineOutput output;
    output.observations = observations_at(timestamp, map_frame_id, scenario_start);
    return output;
}

}  // namespace dedalus
