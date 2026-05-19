#include "dedalus/behavior/object_behavior_mission_controller.hpp"

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

void require_near(double actual, double expected, double tolerance, const std::string& message) {
    if (std::abs(actual - expected) > tolerance) {
        throw std::runtime_error(message + ": actual=" + std::to_string(actual) + " expected=" + std::to_string(expected));
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

dedalus::WorldSnapshot make_snapshot(
    std::int64_t timestamp_ns,
    double height_m,
    bool armed) {
    dedalus::WorldSnapshot snapshot;
    snapshot.timestamp = dedalus::TimePoint{timestamp_ns};
    snapshot.active_map_frame_id = dedalus::MapFrameId{"map_local_0001"};
    snapshot.ego.map_frame_id = snapshot.active_map_frame_id;
    snapshot.ego.local_T_body.position = dedalus::Vec3{0.0, 0.0, -height_m};
    snapshot.ego.height_m = height_m;
    snapshot.ego.height_valid = true;
    snapshot.ego.armed = armed;
    snapshot.ego.armed_valid = true;
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
    config.behavior_spec.behavior.target_frame = dedalus::ReferenceFrame::TargetHeadingFrame;
    config.behavior_spec.behavior.relative_offset_m = dedalus::BehaviorVector3{-8.0, 0.0, 4.0};
    config.behavior_spec.behavior.max_speed_mps = 2.0;
    config.behavior_spec.behavior.max_vertical_speed_mps = 1.0;
    config.behavior_spec.behavior.position_tolerance_m = 1.5;
    config.behavior_spec.completion.after_s = 0.2;
    config.safe_height_m = 2.0;
    config.arm_timeout_s = 5.0;
    config.disarm_timeout_s = 5.0;
    return config;
}

void lifecycle_gates_before_behavior_and_emits_events() {
    dedalus::ObjectBehaviorMissionController controller{make_config()};
    dedalus::MissionTickInput input;
    input.now = dedalus::TimePoint{0};
    input.snapshot = make_snapshot(0, 0.0, false);

    const auto prepare = controller.tick(input);
    require(prepare.state == dedalus::MissionLifecycleState::Prepare, "first tick should enter Prepare");
    require(prepare.command.has_value(), "Prepare should request arm");
    require(prepare.command->kind == dedalus::FlightCommandKind::Arm, "Prepare command should be Arm");

    input.now = dedalus::TimePoint{100000000};
    input.snapshot = make_snapshot(100000000, 0.0, true);
    const auto takeoff = controller.tick(input);
    require(takeoff.state == dedalus::MissionLifecycleState::Takeoff, "armed ego should enter Takeoff");
    require(!takeoff.events.size(), "Takeoff transition should not emit behavior events yet");

    input.now = dedalus::TimePoint{200000000};
    input.snapshot = make_snapshot(200000000, 2.0, true);
    const auto execute_gate = controller.tick(input);
    require(execute_gate.state == dedalus::MissionLifecycleState::ExecuteMission, "safe height should enter ExecuteMission");
    require(execute_gate.status == "takeoff_complete", "safe-height gate status should be takeoff_complete");
    require(execute_gate.events.empty(), "safe-height gate tick should not select until next ExecuteMission tick");

    input.now = dedalus::TimePoint{300000000};
    input.snapshot = make_snapshot(300000000, 2.0, true);
    const auto behavior = controller.tick(input);
    require(behavior.state == dedalus::MissionLifecycleState::ExecuteMission, "behavior tick should remain ExecuteMission");
    require(behavior.command.has_value(), "behavior tick should emit follow velocity command");
    require(behavior.command->kind == dedalus::FlightCommandKind::Velocity, "follow command should be velocity");
    require_near(behavior.command->velocity_local_mps.x, 2.0, 1.0e-9, "follow vx should be bounded by max_speed");
    require_near(behavior.command->velocity_local_mps.y, -4.0 / 3.0, 1.0e-6, "follow vy should move toward standoff point");
    require_near(behavior.command->velocity_local_mps.z, -1.0, 1.0e-9, "follow vz should be bounded by max_vertical_speed");
    require(behavior.events.size() == 3U, "behavior tick should emit target_selected, behavior_start, behavior_tick_sample");
    require(behavior.events[0].find("\"event\":\"target_selected\"") != std::string::npos, "missing target_selected event");
    require(behavior.events[0].find("\"source_track_id\":\"ghost_person_001\"") != std::string::npos, "selected target should be ghost_person_001");
    require(behavior.events[0].find("ghost_person_002") == std::string::npos, "must not select higher confidence neighbor");
    require(behavior.events[1].find("\"event\":\"behavior_start\"") != std::string::npos, "missing behavior_start event");
    require(behavior.events[2].find("\"event\":\"behavior_tick_sample\"") != std::string::npos, "missing behavior_tick_sample event");

    input.now = dedalus::TimePoint{600000000};
    input.snapshot = make_snapshot(600000000, 2.0, true);
    const auto complete_behavior = controller.tick(input);
    require(complete_behavior.state == dedalus::MissionLifecycleState::GoHome, "completion should transition to GoHome");
    require(complete_behavior.events.size() == 1U, "completion tick should emit behavior_complete");
    require(complete_behavior.events[0].find("\"event\":\"behavior_complete\"") != std::string::npos, "missing behavior_complete event");
    require(complete_behavior.events[0].find("\"source_track_id\":\"ghost_person_001\"") != std::string::npos, "complete event should carry selected target");
}

void landing_and_disarm_reach_complete_status() {
    dedalus::ObjectBehaviorMissionController controller{make_config()};
    dedalus::MissionTickInput input;
    input.now = dedalus::TimePoint{0};
    input.snapshot = make_snapshot(0, 0.0, false);
    (void)controller.tick(input);
    input.now = dedalus::TimePoint{100000000};
    input.snapshot = make_snapshot(100000000, 0.0, true);
    (void)controller.tick(input);
    input.now = dedalus::TimePoint{200000000};
    input.snapshot = make_snapshot(200000000, 2.0, true);
    (void)controller.tick(input);
    input.now = dedalus::TimePoint{300000000};
    input.snapshot = make_snapshot(300000000, 2.0, true);
    (void)controller.tick(input);
    input.now = dedalus::TimePoint{600000000};
    input.snapshot = make_snapshot(600000000, 2.0, true);
    (void)controller.tick(input);

    input.now = dedalus::TimePoint{700000000};
    input.snapshot = make_snapshot(700000000, 2.0, true);
    const auto go_home = controller.tick(input);
    require(go_home.state == dedalus::MissionLifecycleState::Land, "at home should transition to Land");

    input.now = dedalus::TimePoint{800000000};
    input.snapshot = make_snapshot(800000000, 2.0, true);
    const auto land = controller.tick(input);
    require(land.state == dedalus::MissionLifecycleState::Land, "above ground should stay in Land");
    require(land.command.has_value() && land.command->kind == dedalus::FlightCommandKind::Land, "Land should dispatch Land command");

    input.now = dedalus::TimePoint{900000000};
    input.snapshot = make_snapshot(900000000, 0.0, true);
    const auto landed = controller.tick(input);
    require(landed.state == dedalus::MissionLifecycleState::Complete, "landed height should enter Complete");

    input.now = dedalus::TimePoint{1000000000};
    input.snapshot = make_snapshot(1000000000, 0.0, true);
    const auto disarm = controller.tick(input);
    require(disarm.state == dedalus::MissionLifecycleState::Complete, "disarm request should remain Complete");
    require(disarm.command.has_value() && disarm.command->kind == dedalus::FlightCommandKind::Disarm, "Complete should dispatch Disarm command");

    input.now = dedalus::TimePoint{1100000000};
    input.snapshot = make_snapshot(1100000000, 0.0, false);
    const auto complete = controller.tick(input);
    require(complete.state == dedalus::MissionLifecycleState::Complete, "disarmed ego should stay Complete");
    require(complete.status == "complete", "disarmed Complete should report terminal complete status");
}

void finish_requested_completes_behavior() {
    dedalus::ObjectBehaviorMissionController controller{make_config()};
    dedalus::MissionTickInput input;
    input.now = dedalus::TimePoint{0};
    input.snapshot = make_snapshot(0, 0.0, false);
    (void)controller.tick(input);
    input.now = dedalus::TimePoint{100000000};
    input.snapshot = make_snapshot(100000000, 0.0, true);
    (void)controller.tick(input);
    input.now = dedalus::TimePoint{200000000};
    input.snapshot = make_snapshot(200000000, 2.0, true);
    (void)controller.tick(input);
    input.now = dedalus::TimePoint{300000000};
    input.snapshot = make_snapshot(300000000, 2.0, true);
    (void)controller.tick(input);

    input.now = dedalus::TimePoint{400000000};
    input.snapshot = make_snapshot(400000000, 2.0, true);
    input.finish_requested = true;
    const auto finish = controller.tick(input);
    require(finish.state == dedalus::MissionLifecycleState::GoHome, "finish should transition to GoHome");
    require(finish.events.size() == 1U, "finish should emit behavior_complete");
    require(finish.events[0].find("finish_requested") != std::string::npos, "finish event should explain reason");
}

}  // namespace

int main() {
    try {
        lifecycle_gates_before_behavior_and_emits_events();
        landing_and_disarm_reach_complete_status();
        finish_requested_completes_behavior();
    } catch (const std::exception& exc) {
        std::cerr << "test_object_behavior_mission_controller failed: " << exc.what() << '\n';
        return 1;
    }
    return 0;
}
