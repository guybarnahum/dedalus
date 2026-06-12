#include "dedalus/simulation/ghost_scenario.hpp"

#include <algorithm>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace dedalus {
namespace {

Vec3 add(Vec3 lhs, Vec3 rhs) {
    return Vec3{lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

Vec3 scale(Vec3 value, double factor) {
    return Vec3{value.x * factor, value.y * factor, value.z * factor};
}

struct TrajectoryEval {
    Vec3 position_delta;
    Vec3 velocity;
};

TrajectoryEval integrate_trajectory(const VelocityTrajectory& trajectory, double scenario_elapsed_s) {
    if (trajectory.empty() || scenario_elapsed_s <= 0.0) {
        return TrajectoryEval{};
    }

    double remaining = scenario_elapsed_s;
    Vec3 position_delta;
    Vec3 velocity;
    for (std::size_t segment_index = 0; segment_index < trajectory.size(); ++segment_index) {
        const auto& segment = trajectory.segment(segment_index);
        const double segment_duration = std::max(0.0, segment.duration_s);
        const double active_duration = std::min(remaining, segment_duration);
        if (active_duration > 0.0) {
            constexpr double kIntegrationStepS = 0.05;
            double integrated = 0.0;
            while (integrated < active_duration) {
                const double dt = std::min(kIntegrationStepS, active_duration - integrated);
                const double midpoint = integrated + 0.5 * dt;
                const auto v = trajectory.velocity_at(segment_index, midpoint);
                position_delta = add(position_delta, scale(v, dt));
                integrated += dt;
            }
            velocity = trajectory.velocity_at(segment_index, active_duration);
        }
        remaining -= active_duration;
        if (remaining <= 0.0) {
            return TrajectoryEval{position_delta, velocity};
        }
    }

    if (trajectory.size() > 0U) {
        const auto last_index = trajectory.size() - 1U;
        const auto& last = trajectory.segment(last_index);
        velocity = trajectory.velocity_at(last_index, last.duration_s);
    }
    return TrajectoryEval{position_delta, velocity};
}

}  // namespace

std::string to_string(ClassLabel label) {
    switch (label) {
        case ClassLabel::Person: return "person";
        case ClassLabel::Drone: return "drone";
        case ClassLabel::Car: return "car";
        case ClassLabel::Boat: return "boat";
        case ClassLabel::Animal: return "animal";
        case ClassLabel::House: return "house";
        case ClassLabel::Building: return "building";
        case ClassLabel::Tree: return "tree";
        case ClassLabel::Road: return "road";
        case ClassLabel::River: return "river";
        case ClassLabel::Terrain: return "terrain";
        case ClassLabel::Pole: return "pole";
        case ClassLabel::Wall: return "wall";
        case ClassLabel::Fence: return "fence";
        case ClassLabel::Cable: return "cable";
        case ClassLabel::Obstacle: return "obstacle";
        case ClassLabel::Unknown:
        default: return "unknown";
    }
}

GhostScenario::GhostScenario(std::string name, MapFrameId map_frame_id, std::vector<GhostDetectionSpec> detections)
    : name_(std::move(name)), map_frame_id_(std::move(map_frame_id)), detections_(std::move(detections)) {}

const std::string& GhostScenario::name() const {
    return name_;
}

const MapFrameId& GhostScenario::map_frame_id() const {
    return map_frame_id_;
}

const std::vector<GhostDetectionSpec>& GhostScenario::detections() const {
    return detections_;
}

bool GhostScenario::empty() const {
    return detections_.empty();
}

std::vector<GhostDetectionState> GhostScenario::evaluate(double scenario_elapsed_s) const {
    std::vector<GhostDetectionState> output;
    output.reserve(detections_.size());
    for (const auto& detection : detections_) {
        GhostDetectionState state;
        state.source_track_id = detection.source_track_id;
        state.class_label = detection.class_label;
        state.confidence = detection.confidence;
        const auto trajectory_eval = integrate_trajectory(detection.trajectory, scenario_elapsed_s);
        state.position_local_m = add(detection.initial_position_local_m, trajectory_eval.position_delta);
        state.velocity_local_mps = trajectory_eval.velocity;
        state.size_m = detection.size_m;
        output.push_back(state);
    }
    return output;
}

}  // namespace dedalus
