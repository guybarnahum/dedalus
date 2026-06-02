#include "dedalus/behavior/target_selector.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

dedalus::AgentState make_agent(
    std::string track_id,
    dedalus::ClassLabel class_label,
    float confidence,
    dedalus::Vec3 position,
    dedalus::AgentLifecycle lifecycle = dedalus::AgentLifecycle::Active) {
    dedalus::AgentState agent;
    agent.source_track_id = dedalus::TrackId{track_id};
    agent.agent_id = dedalus::AgentId{"agent_" + track_id};
    agent.identity_id = dedalus::IdentityId{"identity_" + track_id};
    agent.class_label = class_label;
    agent.confidence = confidence;
    agent.position_local = position;
    agent.velocity_local = dedalus::Vec3{0.2, 0.0, 0.0};
    agent.map_frame_id = dedalus::MapFrameId{"map_local_0001"};
    agent.lifecycle = lifecycle;
    agent.last_seen = dedalus::TimePoint{9000000000};
    return agent;
}

dedalus::WorldSnapshot make_snapshot() {
    dedalus::WorldSnapshot snapshot;
    snapshot.timestamp = dedalus::TimePoint{10000000000};
    snapshot.active_map_frame_id = dedalus::MapFrameId{"map_local_0001"};
    snapshot.ego.map_frame_id = snapshot.active_map_frame_id;
    snapshot.ego.local_T_body.position = dedalus::Vec3{0.0, 0.0, 0.0};
    snapshot.agents.push_back(make_agent(
        "ghost_person_001",
        dedalus::ClassLabel::Person,
        0.82F,
        dedalus::Vec3{12.0, -4.0, 0.0}));
    snapshot.agents.push_back(make_agent(
        "ghost_person_002",
        dedalus::ClassLabel::Person,
        0.91F,
        dedalus::Vec3{8.0, 4.0, 0.0}));
    snapshot.agents.push_back(make_agent(
        "ghost_car_001",
        dedalus::ClassLabel::Car,
        0.95F,
        dedalus::Vec3{3.0, 0.0, 0.0}));
    return snapshot;
}

dedalus::TargetSelectorSpec person_spec() {
    dedalus::TargetSelectorSpec spec;
    spec.class_label = dedalus::ClassLabel::Person;
    spec.confidence_min = 0.5;
    spec.policy = dedalus::TargetSelectionPolicy::HighestConfidence;
    spec.reacquire_timeout_s = 5.0;
    return spec;
}

void selects_highest_confidence_matching_class() {
    const auto snapshot = make_snapshot();
    auto spec = person_spec();
    spec.policy = dedalus::TargetSelectionPolicy::HighestConfidence;

    const auto selection = dedalus::TargetSelector{}.select(snapshot, spec);

    require(selection.selected, "highest-confidence selector should select");
    require(selection.status == dedalus::TargetSelectionStatus::Selected, "selection status should be selected");
    require(selection.source_track_id.value == "ghost_person_002", "highest-confidence person should win");
    require(selection.agent_id.value == "agent_ghost_person_002", "agent id should be copied");
    require(selection.identity_id.value == "identity_ghost_person_002", "identity id should be copied");
    require(selection.class_label == dedalus::ClassLabel::Person, "class label should be copied");
    require(selection.reason == "highest_confidence_candidate", "reason should identify confidence policy");
}

void explicit_track_id_selects_specific_track() {
    const auto snapshot = make_snapshot();
    auto spec = person_spec();
    spec.track_id = "ghost_person_001";
    spec.policy = dedalus::TargetSelectionPolicy::PersistentTrack;

    const auto selection = dedalus::TargetSelector{}.select(snapshot, spec);

    require(selection.selected, "track selector should select");
    require(selection.source_track_id.value == "ghost_person_001", "explicit track id should win");
    require(selection.agent_id.value == "agent_ghost_person_001", "track-selected agent should be copied");
}

void explicit_agent_id_selects_specific_agent() {
    const auto snapshot = make_snapshot();
    auto spec = person_spec();
    spec.agent_id = "agent_ghost_person_001";

    const auto selection = dedalus::TargetSelector{}.select(snapshot, spec);

    require(selection.selected, "agent selector should select");
    require(selection.agent_id.value == "agent_ghost_person_001", "explicit agent id should win");
    require(selection.source_track_id.value == "ghost_person_001", "agent-selected track should be copied");
}

void nearest_selects_closest_matching_class() {
    const auto snapshot = make_snapshot();
    auto spec = person_spec();
    spec.policy = dedalus::TargetSelectionPolicy::Nearest;

    const auto selection = dedalus::TargetSelector{}.select(snapshot, spec);

    require(selection.selected, "nearest selector should select");
    require(selection.source_track_id.value == "ghost_person_002", "nearest person should win");
    require(selection.reason == "nearest_candidate", "reason should identify nearest policy");
}

void persistent_track_keeps_previous_target() {
    const auto snapshot = make_snapshot();
    auto initial_spec = person_spec();
    initial_spec.track_id = "ghost_person_001";
    initial_spec.policy = dedalus::TargetSelectionPolicy::PersistentTrack;

    const auto previous = dedalus::TargetSelector{}.select(snapshot, initial_spec);
    require(previous.selected, "initial specific target should select");

    auto persistent_spec = person_spec();
    persistent_spec.policy = dedalus::TargetSelectionPolicy::PersistentTrack;
    const auto selection = dedalus::TargetSelector{}.select(snapshot, persistent_spec, previous);

    require(selection.selected, "persistent selector should keep previous target");
    require(selection.source_track_id.value == "ghost_person_001", "previous target should beat higher-confidence neighbor");
    require(selection.reason == "previous_target_still_valid", "reason should identify persistence");
}

void ignores_wrong_class_low_confidence_and_retired_agents() {
    auto snapshot = make_snapshot();
    snapshot.agents.push_back(make_agent(
        "ghost_person_low",
        dedalus::ClassLabel::Person,
        0.1F,
        dedalus::Vec3{1.0, 0.0, 0.0}));
    snapshot.agents.push_back(make_agent(
        "ghost_person_retired",
        dedalus::ClassLabel::Person,
        0.99F,
        dedalus::Vec3{0.5, 0.0, 0.0},
        dedalus::AgentLifecycle::Retired));

    auto spec = person_spec();
    spec.policy = dedalus::TargetSelectionPolicy::Nearest;
    spec.confidence_min = 0.5;

    const auto selection = dedalus::TargetSelector{}.select(snapshot, spec);

    require(selection.selected, "selector should select valid person");
    require(selection.source_track_id.value == "ghost_person_002", "invalid nearer people and wrong-class car should be ignored");
}

void no_candidates_and_invalid_spec_are_reported() {
    const auto snapshot = make_snapshot();

    auto no_match_spec = person_spec();
    no_match_spec.track_id = "missing_track";
    const auto no_match = dedalus::TargetSelector{}.select(snapshot, no_match_spec);
    require(!no_match.selected, "missing track should not select");
    require(no_match.status == dedalus::TargetSelectionStatus::NoCandidates, "missing track should report no candidates");
    require(no_match.reason == "no_matching_agents", "missing track reason should be useful");

    dedalus::TargetSelectorSpec invalid_spec;
    // class_label defaults to ClassLabel::Unknown, which triggers InvalidSpec
    const auto invalid = dedalus::TargetSelector{}.select(snapshot, invalid_spec);
    require(!invalid.selected, "invalid spec should not select");
    require(invalid.status == dedalus::TargetSelectionStatus::InvalidSpec, "invalid spec status should be invalid_spec");
}

void reacquire_and_lost_are_reported_for_missing_previous_target() {
    auto snapshot = make_snapshot();
    auto spec = person_spec();
    spec.track_id = "ghost_person_001";
    spec.policy = dedalus::TargetSelectionPolicy::PersistentTrack;

    const auto previous = dedalus::TargetSelector{}.select(snapshot, spec);
    require(previous.selected, "previous target should select before disappearance");

    snapshot.agents.clear();

    auto recent_previous = previous;
    recent_previous.target_age_s = 2.0;
    const auto reacquiring = dedalus::TargetSelector{}.select(snapshot, spec, recent_previous);
    require(!reacquiring.selected, "missing recent previous target should not be selected");
    require(reacquiring.status == dedalus::TargetSelectionStatus::Reacquiring, "recent missing previous target should reacquire");
    require(reacquiring.source_track_id.value == "ghost_person_001", "reacquire status should preserve target handle");

    auto old_previous = previous;
    old_previous.target_age_s = 7.0;
    const auto lost = dedalus::TargetSelector{}.select(snapshot, spec, old_previous);
    require(!lost.selected, "old missing previous target should not be selected");
    require(lost.status == dedalus::TargetSelectionStatus::Lost, "old missing previous target should be lost");
    require(lost.source_track_id.value == "ghost_person_001", "lost status should preserve target handle");
}

}  // namespace

int main() {
    try {
        selects_highest_confidence_matching_class();
        explicit_track_id_selects_specific_track();
        explicit_agent_id_selects_specific_agent();
        nearest_selects_closest_matching_class();
        persistent_track_keeps_previous_target();
        ignores_wrong_class_low_confidence_and_retired_agents();
        no_candidates_and_invalid_spec_are_reported();
        reacquire_and_lost_are_reported_for_missing_previous_target();
    } catch (const std::exception& exc) {
        std::cerr << "test_target_selector failed: " << exc.what() << '\n';
        return 1;
    }
    return 0;
}
