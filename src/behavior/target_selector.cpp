#include "dedalus/behavior/target_selector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace dedalus {
namespace {

constexpr double kNsPerSecond = 1000000000.0;

ClassLabel parse_class_label(const std::string& value) {
    if (value == "person") {
        return ClassLabel::Person;
    }
    if (value == "drone") {
        return ClassLabel::Drone;
    }
    if (value == "car" || value == "vehicle") {
        return ClassLabel::Car;
    }
 if (value == "boat") {
        return ClassLabel::Boat;
    }
    if (value == "animal") {
        return ClassLabel::Animal;
    }
    if (value == "house") {
        return ClassLabel::House;
    }
    if (value == "building") {
        return ClassLabel::Building;
    }
    if (value == "tree") {
        return ClassLabel::Tree;
    }
    if (value == "road") {
        return ClassLabel::Road;
    }
    if (value == "river") {
        return ClassLabel::River;
    }
    if (value == "terrain") {
        return ClassLabel::Terrain;
    }
    return ClassLabel::Unknown;
}

bool is_selectable_lifecycle(AgentLifecycle lifecycle) {
    return lifecycle == AgentLifecycle::New || lifecycle == AgentLifecycle::Active;
}

bool finite_vec3(const Vec3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

double distance_sq(const Vec3& lhs, const Vec3& rhs) {
    const double dx = lhs.x - rhs.x;
    const double dy = lhs.y - rhs.y;
    const double dz = lhs.z - rhs.z;
    return dx * dx + dy * dy + dz * dz;
}

double age_seconds(TimePoint now, TimePoint last_seen) {
    const auto delta_ns = now.timestamp_ns - last_seen.timestamp_ns;
    if (delta_ns <= 0) {
        return 0.0;
    }
    return static_cast<double>(delta_ns) / kNsPerSecond;
}

TargetSelection make_selection(const WorldSnapshot& snapshot, const AgentState& agent, std::string reason) {
    TargetSelection selection;
    selection.selected = true;
    selection.status = TargetSelectionStatus::Selected;
    selection.agent_id = agent.agent_id;
    selection.source_track_id = agent.source_track_id;
    selection.identity_id = agent.identity_id;
    selection.class_label = agent.class_label;
    selection.confidence = agent.confidence;
    selection.position_local = agent.position_local;
    selection.velocity_local = agent.velocity_local;
    selection.map_frame_id = agent.map_frame_id;
    selection.target_age_s = age_seconds(snapshot.timestamp, agent.last_seen);
    selection.reason = std::move(reason);
    return selection;
}

TargetSelection make_empty(TargetSelectionStatus status, std::string reason) {
    TargetSelection selection;
    selection.selected = false;
    selection.status = status;
    selection.reason = std::move(reason);
    return selection;
}

bool matches_previous(const AgentState& agent, const TargetSelection& previous) {
    if (!previous.agent_id.value.empty() && agent.agent_id.value == previous.agent_id.value) {
        return true;
    }
    if (!previous.source_track_id.value.empty() &&
        agent.source_track_id.value == previous.source_track_id.value) {
        return true;
    }
    return false;
}

bool matches_spec(const AgentState& agent, const TargetSelectorSpec& spec, ClassLabel requested_class) {
    if (requested_class == ClassLabel::Unknown || agent.class_label != requested_class) {
        return false;
    }
    if (!is_selectable_lifecycle(agent.lifecycle)) {
        return false;
    }
    if (agent.confidence < static_cast<float>(spec.confidence_min)) {
        return false;
    }
    if (!finite_vec3(agent.position_local)) {
        return false;
    }
    if (!spec.track_id.empty() && agent.source_track_id.value != spec.track_id) {
        return false;
    }
    if (!spec.agent_id.empty() && agent.agent_id.value != spec.agent_id) {
        return false;
    }
    return true;
}

}  // namespace

std::string to_string(TargetSelectionStatus status) {
    switch (status) {
        case TargetSelectionStatus::Selected:
            return "selected";
        case TargetSelectionStatus::Reacquiring:
            return "reacquiring";
        case TargetSelectionStatus::Lost:
            return "lost";
        case TargetSelectionStatus::NoCandidates:
            return "no_candidates";
        case TargetSelectionStatus::InvalidSpec:
            return "invalid_spec";
    }
    return "invalid_spec";
}

TargetSelection TargetSelector::select(
    const WorldSnapshot& snapshot,
    const TargetSelectorSpec& spec,
    const std::optional<TargetSelection>& previous) const {
    const ClassLabel requested_class = parse_class_label(spec.class_label);
    if (spec.class_label.empty() || requested_class == ClassLabel::Unknown) {
        return make_empty(TargetSelectionStatus::InvalidSpec, "invalid_or_missing_target_class");
    }
    if (spec.confidence_min < 0.0 || spec.confidence_min > 1.0) {
        return make_empty(TargetSelectionStatus::InvalidSpec, "confidence_min_out_of_range");
    }
    if (spec.reacquire_timeout_s < 0.0) {
        return make_empty(TargetSelectionStatus::InvalidSpec, "negative_reacquire_timeout");
    }

    std::vector<const AgentState*> candidates;
    candidates.reserve(snapshot.agents.size());
    for (const auto& agent : snapshot.agents) {
        if (matches_spec(agent, spec, requested_class)) {
            candidates.push_back(&agent);
        }
    }

    if (previous.has_value() && spec.policy == TargetSelectionPolicy::PersistentTrack) {
        const auto previous_match = std::find_if(
            candidates.begin(),
            candidates.end(),
            [&](const AgentState* agent) { return matches_previous(*agent, *previous); });
        if (previous_match != candidates.end()) {
            return make_selection(snapshot, **previous_match, "previous_target_still_valid");
        }
        if (previous->selected && previous->target_age_s <= spec.reacquire_timeout_s) {
            auto selection = make_empty(TargetSelectionStatus::Reacquiring, "previous_target_temporarily_missing");
            selection.agent_id = previous->agent_id;
            selection.source_track_id = previous->source_track_id;
            selection.identity_id = previous->identity_id;
            selection.class_label = previous->class_label;
            selection.map_frame_id = previous->map_frame_id;
            selection.target_age_s = previous->target_age_s;
            return selection;
        }
    }

    if (candidates.empty()) {
        if (previous.has_value() && previous->selected) {
            auto selection = make_empty(TargetSelectionStatus::Lost, "previous_target_lost");
            selection.agent_id = previous->agent_id;
            selection.source_track_id = previous->source_track_id;
            selection.identity_id = previous->identity_id;
            selection.class_label = previous->class_label;
            selection.map_frame_id = previous->map_frame_id;
            selection.target_age_s = previous->target_age_s;
            return selection;
        }
        return make_empty(TargetSelectionStatus::NoCandidates, "no_matching_agents");
    }

    const AgentState* best = nullptr;
    switch (spec.policy) {
        case TargetSelectionPolicy::Nearest: {
            const Vec3 ego_position = snapshot.ego.local_T_body.position;
            double best_distance_sq = std::numeric_limits<double>::infinity();
            for (const AgentState* candidate : candidates) {
                const double d2 = distance_sq(candidate->position_local, ego_position);
                if (best == nullptr || d2 < best_distance_sq ||
                    (d2 == best_distance_sq && candidate->confidence > best->confidence)) {
                    best = candidate;
                    best_distance_sq = d2;
                }
            }
            return make_selection(snapshot, *best, "nearest_candidate");
        }
        case TargetSelectionPolicy::HighestConfidence:
        case TargetSelectionPolicy::PersistentTrack:
        default: {
            for (const AgentState* candidate : candidates) {
                if (best == nullptr || candidate->confidence > best->confidence) {
                    best = candidate;
                }
            }
            return make_selection(snapshot, *best, "highest_confidence_candidate");
        }
    }
}

}  // namespace dedalus
