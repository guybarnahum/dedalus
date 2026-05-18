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

Vec3 linear_position(const GhostTargetTrajectory& trajectory, double elapsed_s) {
    return Vec3{
        trajectory.start_local_m.x + trajectory.velocity_local_mps.x * elapsed_s,
        trajectory.start_local_m.y + trajectory.velocity_local_mps.y * elapsed_s,
        trajectory.start_local_m.z + trajectory.velocity_local_mps.z * elapsed_s,
    };
}

}  // namespace

GhostTargetProvider::GhostTargetProvider(std::vector<GhostTargetSpec> targets)
    : targets_(std::move(targets)) {}

std::vector<Observation3D> GhostTargetProvider::observations_at(
    TimePoint timestamp,
    MapFrameId map_frame_id,
    TimePoint scenario_start) const {
    const double elapsed_s = elapsed_seconds(timestamp, scenario_start);
    std::vector<Observation3D> observations;
    observations.reserve(targets_.size());

    for (const auto& target : targets_) {
        Observation3D observation;
        observation.track_id = target.track_id;
        observation.timestamp = timestamp;
        observation.position_local = linear_position(target.trajectory, elapsed_s);
        observation.position_body = observation.position_local;
        observation.map_frame_id = map_frame_id;
        observation.class_label = target.class_label;
        observation.faction = target.faction;
        observation.confidence = target.confidence;
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
