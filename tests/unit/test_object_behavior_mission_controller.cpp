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
    dedalus::Vec3 position,
    dedalus::Vec3 velocity = dedalus::Vec3{0.2, 0.0, 0.0}) {
    dedalus::AgentState agent;
    agent.source_track_id = dedalus::TrackId{track_id};
    agent.agent_id = dedalus::AgentId{"agent_" + track_id};
    agent.identity_id = dedalus::IdentityId{"identity_" + track_id};
    agent.class_label = class_label;
    agent.confidence = confidence;
    agent.position_local = position;
    agent.velocity_local = velocity;
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

dedalus::ObjectBehaviorMissionConfig make_circle_config() {
    auto config = make_config();
    config.behavior_spec.target.class_label = "car";
    config.behavior_spec.target.track_id = "ghost_car_001";
    config.behavior_spec.behavior.type = dedalus::BehaviorType::Circle;
    config.behavior_spec.behavior.radius_m = 10.0;
    config.behavior_spec.behavior.altitude_offset_m = 2.0;
    config.behavior_spec.behavior.angular_speed_deg_s = 10.0;
    config.behavior_spec.behavior.direction = dedalus::CircleDirection::Clockwise;
    config.behavior_spec.behavior.max_speed_mps = 20.0;
    config.behavior_spec.behavior.max_vertical_speed_mps = 5.0;
    config.behavior_spec.behavior.position_tolerance_m = 1.0;
    config.behavior_spec.completion.after_s = 30.0;
    return config;
}

dedalus::WorldSnapshot make_circle_snapshot(
    std::int64_t timestamp_ns,
    dedalus::Vec3 ego_position,
    dedalus::Vec3 target_position,
    dedalus::Vec3 target_velocity = dedalus::Vec3{0.0, 0.0, 0.0}) {
    auto snapshot = make_snapshot(timestamp_ns, -ego_position.z, true);
    snapshot.ego.local_T_body.position = ego_position;
    snapshot.ego.height_m = -ego_position.z;
    snapshot.agents.clear();
    snapshot.agents.push_back(make_agent(
        "ghost_car_001",
        dedalus::ClassLabel::Car,
        0.91F,
        target_position,
        target_velocity));
    return snapshot;
}

dedalus::MissionTickOutput first_execute_tick(
    dedalus::ObjectBehaviorMissionController& controller,
    dedalus::WorldSnapshot snapshot) {
    dedalus::MissionTickInput input;
    input.now = dedalus::TimePoint{0};
    input.snapshot = make_circle_snapshot(0, dedalus::Vec3{0.0, 0.0, 0.0}, dedalus::Vec3{0.0, 0.0, 0.0});
    input.snapshot.ego.armed = false;
    (void)controller.tick(input);
    input.now = dedalus::TimePoint{100000000};
    input.snapshot.ego.armed = true;
    (void)controller.tick(input);
    input.now = dedalus::TimePoint{200000000};
    input.snapshot.ego.local_T_body.position.z = -2.0;
    input.snapshot.ego.height_m = 2.0;
    (void)controller.tick(input);
    input.now = dedalus::TimePoint{300000000};
    input.snapshot = snapshot;
    input.snapshot.timestamp = input.now;
    return controller.tick(input);
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
    // New arrival controller: velocity = clamp(target_velocity + closing_velocity, max_speed).
    // error=(4,-4), error_xy=4*sqrt(2), slow-zone kp=0.35 -> closing_speed=1.4*sqrt(2).
    // closing=(1.4,-1.4), pre_clamp=(0.2+1.4,-1.4)=(1.6,-1.4), norm=sqrt(4.52)>2 -> scaled to max_speed.
    const double expected_vx = 3.2 / std::sqrt(4.52);
    const double expected_vy = -2.8 / std::sqrt(4.52);
    require_near(behavior.command->velocity_local_mps.x, expected_vx, 1.0e-6, "follow vx should be vector-clamped by max_speed");
    require_near(behavior.command->velocity_local_mps.y, expected_vy, 1.0e-6, "follow vy should be vector-clamped by max_speed");
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

void circle_static_target_points_to_three_oclock_entry() {
    auto config = make_circle_config();
    dedalus::ObjectBehaviorMissionController controller{config};
    const auto behavior = first_execute_tick(
        controller,
        make_circle_snapshot(
            300000000,
            dedalus::Vec3{-10.0, 0.0, -2.0},
            dedalus::Vec3{0.0, 0.0, 0.0}));
    require(behavior.command.has_value(), "circle arriving should emit velocity");
    require(behavior.command->velocity_local_mps.x > 9.0, "circle entry should command toward +X 3 o'clock entry");
    require(behavior.command->velocity_local_mps.y < 0.0, "clockwise orbit insertion should include negative-Y tangent");
    require(behavior.status == "object_behavior_arriving", "far entry should report arriving");
    require(behavior.events.back().find("\"display_detail\":\"arriving\"") != std::string::npos, "arriving tick should publish display detail");
    require(behavior.events.back().find("\"circle_phase\":\"arriving\"") != std::string::npos, "arriving tick should publish circle phase");
}

void circle_moving_target_command_includes_target_velocity() {
    auto config = make_circle_config();
    dedalus::ObjectBehaviorMissionController controller{config};
    const auto behavior = first_execute_tick(
        controller,
        make_circle_snapshot(
            300000000,
            dedalus::Vec3{10.0, 0.0, -2.0},
            dedalus::Vec3{0.0, 0.0, 0.0},
            dedalus::Vec3{1.0, 0.0, 0.0}));
    require(behavior.command.has_value(), "circle moving target should emit velocity");
    require_near(behavior.command->velocity_local_mps.x, 1.0, 1.0e-6, "circling vx should include target velocity");
    require_near(behavior.command->velocity_local_mps.y, -10.0 * 10.0 * 3.14159265358979323846 / 180.0, 1.0e-6, "circling vy should include clockwise tangent velocity");
    require(behavior.status == "object_behavior_circling", "at entry should report circling");
}

void circle_tangent_velocity_has_correct_direction() {
    auto clockwise = make_circle_config();
    dedalus::ObjectBehaviorMissionController clockwise_controller{clockwise};
    const auto clockwise_tick = first_execute_tick(
        clockwise_controller,
        make_circle_snapshot(
            300000000,
            dedalus::Vec3{10.0, 0.0, -2.0},
            dedalus::Vec3{0.0, 0.0, 0.0}));
    require(clockwise_tick.command->velocity_local_mps.y < 0.0, "clockwise tangent at 3 o'clock should be negative Y");

    auto counter = make_circle_config();
    counter.behavior_spec.behavior.direction = dedalus::CircleDirection::CounterClockwise;
    dedalus::ObjectBehaviorMissionController counter_controller{counter};
    const auto counter_tick = first_execute_tick(
        counter_controller,
        make_circle_snapshot(
            300000000,
            dedalus::Vec3{10.0, 0.0, -2.0},
            dedalus::Vec3{0.0, 0.0, 0.0}));
    require(counter_tick.command->velocity_local_mps.y > 0.0, "counter-clockwise tangent at 3 o'clock should be positive Y");
}

void circle_radial_correction_pushes_inward_and_outward() {
    auto config = make_circle_config();
    config.behavior_spec.behavior.position_tolerance_m = 5.0;

    dedalus::ObjectBehaviorMissionController inside_controller{config};
    const auto inside = first_execute_tick(
        inside_controller,
        make_circle_snapshot(
            300000000,
            dedalus::Vec3{8.0, 0.0, -2.0},
            dedalus::Vec3{0.0, 0.0, 0.0}));
    require(inside.command->velocity_local_mps.x > 0.0, "inside orbit should push outward");
    require(inside.events.back().find("\"radius_error_m\":-2.000000") != std::string::npos, "inside tick should expose negative radius error");

    dedalus::ObjectBehaviorMissionController outside_controller{config};
    const auto outside = first_execute_tick(
        outside_controller,
        make_circle_snapshot(
            300000000,
            dedalus::Vec3{12.0, 0.0, -2.0},
            dedalus::Vec3{0.0, 0.0, 0.0}));
    require(outside.command->velocity_local_mps.x < 0.0, "outside orbit should push inward");
    require(outside.events.back().find("\"radius_error_m\":2.000000") != std::string::npos, "outside tick should expose positive radius error");
}

void circle_command_speed_is_clamped() {
    auto config = make_circle_config();
    config.behavior_spec.behavior.max_speed_mps = 1.0;
    dedalus::ObjectBehaviorMissionController controller{config};
    const auto behavior = first_execute_tick(
        controller,
        make_circle_snapshot(
            300000000,
            dedalus::Vec3{-100.0, 0.0, -2.0},
            dedalus::Vec3{0.0, 0.0, 0.0}));
    require(behavior.command.has_value(), "clamped circle tick should emit velocity");
    const double speed_xy = std::sqrt(
        behavior.command->velocity_local_mps.x * behavior.command->velocity_local_mps.x +
        behavior.command->velocity_local_mps.y * behavior.command->velocity_local_mps.y);
    require(speed_xy <= 1.0 + 1.0e-9, "circle command should clamp horizontal speed");
}

void circle_display_detail_transitions_arriving_to_circling() {
    auto config = make_circle_config();
    dedalus::ObjectBehaviorMissionController controller{config};
    (void)first_execute_tick(
        controller,
        make_circle_snapshot(
            300000000,
            dedalus::Vec3{-10.0, 0.0, -2.0},
            dedalus::Vec3{0.0, 0.0, 0.0}));

    dedalus::MissionTickInput input;
    input.now = dedalus::TimePoint{400000000};
    input.snapshot = make_circle_snapshot(
        400000000,
        dedalus::Vec3{10.0, 0.0, -2.0},
        dedalus::Vec3{0.0, 0.0, 0.0});
    const auto circling = controller.tick(input);
    require(circling.status == "object_behavior_circling", "entry point should transition to circling");
    require(!circling.events.empty(), "circle phase transition should emit a display event");
    require(circling.events.back().find("\"display_detail\":\"circling\"") != std::string::npos, "circling tick should publish display detail");
    require(circling.events.back().find("\"circle_phase\":\"circling\"") != std::string::npos, "circling tick should publish circle phase");
}

}  // namespace

int main() {
    try {
        lifecycle_gates_before_behavior_and_emits_events();
        landing_and_disarm_reach_complete_status();
        finish_requested_completes_behavior();
        circle_static_target_points_to_three_oclock_entry();
        circle_moving_target_command_includes_target_velocity();
        circle_tangent_velocity_has_correct_direction();
        circle_radial_correction_pushes_inward_and_outward();
        circle_command_speed_is_clamped();
        circle_display_detail_transitions_arriving_to_circling();
    } catch (const std::exception& exc) {
        std::cerr << "test_object_behavior_mission_controller failed: " << exc.what() << '\n';
        return 1;
    }
    return 0;
}
