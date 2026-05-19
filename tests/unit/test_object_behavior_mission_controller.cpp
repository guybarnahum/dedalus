#include "dedalus/behavior/object_behavior_mission_controller.hpp"

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
    dedalus::Vec3 position) {
    dedalus::AgentState agent;
    agent.source_track_id = dedalus::TrackId{track_id};
    agent.agent_id = dedalus::AgentId{"agent_" + track_id};
    agent.identity_id = dedalus::IdentityId{"identity_" + track_id};
    agent.class_label = class_label;
    agent.confidence = confidence;
    agent.position_local = position;
    agent.velocity_local = dedalus::Vec3{0.2, 0.0, 0.0};
    agent.map_frame_id = dedalus::MapFrameId{"map_local_0001"};
    agent.lifecycle = dedalus::AgentLifecycle::Active;
    agent.last_seen = dedalus::TimePoint{0};
    return agent;
}

dedalus::WorldSnapshot make_snapshot(std::int64_t timestamp_ns) {
    dedalus::WorldSnapshot snapshot;
    snapshot.timestamp = dedalus::TimePoint{timestamp_ns};
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
    return snapshot;
}

dedalus::ObjectBehaviorMissionConfig make_config() {
    dedalus::ObjectBehaviorMissionConfig config;
    config.behavior_spec.mission_name = "object_behavior_test";
    config.behavior_spec.target.class_label = "person";
    config.behavior_spec.target.track_id = "ghost_person_001";
    config.behavior_spec.target.confidence_min = 0.55;
    config.behavior_spec.target.policy = dedalus::TargetSelectionPolicy::PersistentTrack;
    config.behavior_spec.behavior.type = dedalus::BehaviorType::Follow;
    config.behavior_spec.completion.after_s = 0.2;
    return config;
}

void emits_target_and_behavior_events_and_holds_zero_velocity() {
    dedalus::ObjectBehaviorMissionController controller{make_config()};

    dedalus::MissionTickInput input;
    input.now = dedalus::TimePoint{0};
    input.snapshot = make_snapshot(0);

    const auto first = controller.tick(input);
    require(first.state == dedalus::MissionLifecycleState::ExecuteMission, "first tick should execute mission");
    require(first.command.has_value(), "first tick should emit hold velocity command");
    require(first.command->kind == dedalus::FlightCommandKind::Velocity, "hold command should be velocity");
    require(first.command->velocity_local_mps.x == 0.0, "hold vx should be zero");
    require(first.command->velocity_local_mps.y == 0.0, "hold vy should be zero");
    require(first.command->velocity_local_mps.z == 0.0, "hold vz should be zero");
    require(first.events.size() == 2U, "first tick should emit target_selected and behavior_start");
    require(first.events[0].find("\"event\":\"target_selected\"") != std::string::npos, "missing target_selected event");
    require(first.events[0].find("\"source_track_id\":\"ghost_person_001\"") != std::string::npos, "selected target should be ghost_person_001");
    require(first.events[0].find("ghost_person_002") == std::string::npos, "must not select higher confidence neighbor");
    require(first.events[1].find("\"event\":\"behavior_start\"") != std::string::npos, "missing behavior_start event");

    input.now = dedalus::TimePoint{100000000};
    input.snapshot = make_snapshot(100000000);
    const auto second = controller.tick(input);
    require(second.state == dedalus::MissionLifecycleState::ExecuteMission, "second tick should keep executing");
    require(second.command.has_value(), "second tick should keep holding");
    require(second.events.empty(), "second tick should not re-emit start events");

    input.now = dedalus::TimePoint{300000000};
    input.snapshot = make_snapshot(300000000);
    const auto third = controller.tick(input);
    require(third.state == dedalus::MissionLifecycleState::GoHome, "completion should transition to GoHome");
    require(!third.command.has_value(), "completion tick should not emit a velocity command");
    require(third.events.size() == 1U, "completion tick should emit behavior_complete");
    require(third.events[0].find("\"event\":\"behavior_complete\"") != std::string::npos, "missing behavior_complete event");
    require(third.events[0].find("\"source_track_id\":\"ghost_person_001\"") != std::string::npos, "complete event should carry selected target");
}

void finish_requested_completes_behavior() {
    dedalus::ObjectBehaviorMissionController controller{make_config()};

    dedalus::MissionTickInput input;
    input.now = dedalus::TimePoint{0};
    input.snapshot = make_snapshot(0);
    (void)controller.tick(input);

    input.now = dedalus::TimePoint{100000000};
    input.snapshot = make_snapshot(100000000);
    input.finish_requested = true;
    const auto finish = controller.tick(input);
    require(finish.state == dedalus::MissionLifecycleState::GoHome, "finish should transition to GoHome");
    require(finish.events.size() == 1U, "finish should emit behavior_complete");
    require(finish.events[0].find("finish_requested") != std::string::npos, "finish event should explain reason");
}

}  // namespace

int main() {
    try {
        emits_target_and_behavior_events_and_holds_zero_velocity();
        finish_requested_completes_behavior();
    } catch (const std::exception& exc) {
        std::cerr << "test_object_behavior_mission_controller failed: " << exc.what() << '\n';
        return 1;
    }
    return 0;
}
