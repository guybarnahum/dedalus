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

void approach_moving_target_velocity_matches_standoff_center() {
    auto config = make_config();
    config.behavior_spec.behavior.type = dedalus::BehaviorType::Approach;
    config.behavior_spec.behavior.stop_distance_m = 12.0;
    config.behavior_spec.behavior.max_speed_mps = 3.0;
    config.behavior_spec.behavior.max_vertical_speed_mps = 1.0;
    config.behavior_spec.completion.after_s = 30.0;

    dedalus::ObjectBehaviorMissionController controller{config};
    auto snapshot = make_snapshot(300000000, 2.0, true);
    snapshot.agents.clear();
    snapshot.agents.push_back(make_agent(
        "ghost_person_001",
        dedalus::ClassLabel::Person,
        0.91F,
        dedalus::Vec3{20.0, 0.0, 0.0},
        dedalus::Vec3{0.2, 0.0, 0.0}));

    const auto behavior = first_execute_tick(controller, snapshot);
    require(behavior.command.has_value(), "moving-target approach should emit velocity");
    require(
        behavior.command->velocity_local_mps.x > 0.2,
        "approach command should include target velocity plus closing velocity");
    require(
        behavior.events.back().find("\"target_velocity_mps\":0.200000") != std::string::npos,
        "approach debug event should expose moving target velocity");
}

void circle_static_target_uses_continuous_orbit_capture() {
    auto config = make_circle_config();
    dedalus::ObjectBehaviorMissionController controller{config};
    const auto behavior = first_execute_tick(
        controller,
        make_circle_snapshot(
            300000000,
            dedalus::Vec3{-17.0, 0.0, -2.0},
            dedalus::Vec3{0.0, 0.0, 0.0}));
    require(behavior.command.has_value(), "circle arriving should emit velocity");
    require(behavior.command->velocity_local_mps.x > 0.0, "outside-left circle capture should push outward/right toward orbit radius");
    require(behavior.command->velocity_local_mps.y > 0.0, "outside-left clockwise circle capture should include current-angle tangent");
    require(behavior.events.back().find("\"radial_correction_mps\":-2.000000") != std::string::npos,
            "outside orbit capture should expose inward radial correction");
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

void circle_moving_target_preserves_relative_orbit_authority_under_speed_limit() {
    auto config = make_circle_config();
    config.behavior_spec.behavior.max_speed_mps = 2.0;
    config.behavior_spec.behavior.radius_m = 10.0;
    config.behavior_spec.behavior.angular_speed_deg_s = 8.0;
    config.behavior_spec.behavior.position_tolerance_m = 5.0;

    dedalus::ObjectBehaviorMissionController controller{config};
    const auto behavior = first_execute_tick(
        controller,
        make_circle_snapshot(
            300000000,
            dedalus::Vec3{13.0, 0.0, -2.0},
            dedalus::Vec3{0.0, 0.0, 0.0},
            dedalus::Vec3{0.2, 0.0, 0.0}));

    require(behavior.command.has_value(), "moving-target circle should emit velocity");
    require(
        behavior.command->velocity_local_mps.y < -0.5,
        "moving-target circle should preserve tangent orbit authority under speed limit");
    require(
        behavior.events.back().find("\"target_velocity_mps\":0.200000") != std::string::npos,
        "circle event should expose true moving target velocity");
}

void altitude_profile_is_opt_in_and_bounds_vertical_speed() {
    auto config = make_circle_config();
    config.behavior_spec.behavior.max_vertical_speed_mps = 0.75;
    config.behavior_spec.behavior.altitude_profile.enabled = true;
    config.behavior_spec.behavior.altitude_profile.start_height_m = 22.0;
    config.behavior_spec.behavior.altitude_profile.end_height_m = 14.0;
    config.behavior_spec.behavior.altitude_profile.duration_s = 10.0;
    config.behavior_spec.behavior.altitude_profile.easing = "smoothstep";

    dedalus::ObjectBehaviorMissionController controller{config};
    const auto behavior = first_execute_tick(
        controller,
        make_circle_snapshot(
            300000000,
            dedalus::Vec3{10.0, 0.0, -20.0},
            dedalus::Vec3{0.0, 0.0, 0.0}));

    require(behavior.command.has_value(), "altitude profile circle should emit velocity");
    require_near(
        behavior.command->velocity_local_mps.z,
        -0.75,
        1.0e-9,
        "altitude profile should command bounded climb when current height is below start profile height");
    require(
        behavior.events.back().find("\"altitude_profile_active\":true") != std::string::npos,
        "altitude profile should be observable in behavior tick");
}

void missing_altitude_profile_keeps_legacy_altitude_offset() {
    auto config = make_circle_config();
    config.behavior_spec.behavior.altitude_offset_m = 2.0;
    config.behavior_spec.behavior.max_vertical_speed_mps = 5.0;

    dedalus::ObjectBehaviorMissionController controller{config};
    const auto behavior = first_execute_tick(
        controller,
        make_circle_snapshot(
            300000000,
            dedalus::Vec3{10.0, 0.0, -2.0},
            dedalus::Vec3{0.0, 0.0, 0.0}));

    require(behavior.command.has_value(), "legacy altitude-offset circle should emit velocity");
    require_near(
        behavior.command->velocity_local_mps.z,
        0.0,
        1.0e-9,
        "missing altitude_profile should preserve legacy altitude_offset behavior");
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
            dedalus::Vec3{-17.0, 0.0, -2.0},
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

void circle_target_yaw_points_at_selected_target_not_velocity() {
    auto config = make_circle_config();
    config.yaw_mode = dedalus::ObjectBehaviorYawMode::Target;
    dedalus::ObjectBehaviorMissionController controller{config};

    const auto behavior = first_execute_tick(
        controller,
        make_circle_snapshot(
            300000000,
            dedalus::Vec3{10.0, 0.0, -2.0},
            dedalus::Vec3{0.0, 0.0, 0.0}));

    require(behavior.command.has_value(), "target-yaw circle should emit command");
    require(behavior.command->yaw_valid, "target-yaw circle should emit yaw");
    require_near(
        behavior.command->yaw_rad,
        3.14159265358979323846,
        1.0e-6,
        "ego at +X should yaw toward target at pi radians");
    require(behavior.command->yaw_source == "target", "yaw source should be target");
    require(
        behavior.command->velocity_local_mps.y < 0.0,
        "clockwise circle should still fly tangent while yawing at target");
}

void vertical_stare_gimbal_emits_camera_pointing_intent() {
    auto config = make_circle_config();
    config.vertical_stare_mode = dedalus::ObjectBehaviorVerticalStareMode::Gimbal;
    config.camera_pointing_cameras = {"front_center", "0"};
    config.camera_pitch_min_rad = -80.0 * 3.14159265358979323846 / 180.0;
    config.camera_pitch_max_rad =  80.0 * 3.14159265358979323846 / 180.0;
    dedalus::ObjectBehaviorMissionController controller{config};

    const auto first = first_execute_tick(
        controller,
        make_circle_snapshot(
            300000000,
            dedalus::Vec3{10.0, 0.0, -2.0},
            dedalus::Vec3{0.0, 0.0, 0.0}));

    bool found_intent = false;
    require(first.camera_pointing.has_value(), "gimbal vertical stare should populate typed camera_pointing command");
    require(
        first.camera_pointing->cameras.size() == 2 &&
            first.camera_pointing->cameras[0] == "front_center" &&
            first.camera_pointing->cameras[1] == "0",
        "typed camera_pointing command should carry configured cameras");
    require(
        first.camera_pointing->pitch_valid,
        "typed camera_pointing command should mark pitch valid");
    require(
        first.camera_pointing->source_track_id == "ghost_car_001",
        "typed camera_pointing command should carry source_track_id");
    require(
        first.camera_pointing->agent_id == "agent_ghost_car_001",
        "typed camera_pointing command should carry agent_id");
    require(
        first.camera_pointing->pitch_rad < 0.0,
        "target below ego should request negative typed camera pitch (sign=-1, target elevation positive)");
    for (const auto& event : first.events) {
        if (event.find("\"event\":\"camera_pointing_intent\"") != std::string::npos) {
            require(
                event.find("\"cameras\":[\"front_center\",\"0\"]") != std::string::npos,
                "camera_pointing_intent should include cameras array");
            require(
                event.find("\"pitch_valid\":true") != std::string::npos,
                "camera_pointing_intent should have pitch_valid:true");
            require(
                event.find("\"pitch_deg\":") != std::string::npos,
                "camera_pointing_intent should include pitch_deg");
            found_intent = true;
        }
    }
    require(found_intent, "gimbal vertical stare should emit camera_pointing_intent");

    dedalus::MissionTickInput input;
    input.now = dedalus::TimePoint{400000000};
    input.snapshot = make_circle_snapshot(
        400000000,
        dedalus::Vec3{10.0, 0.0, -2.0},
        dedalus::Vec3{0.0, 0.0, -12.0});
    const auto second = controller.tick(input);

    require(second.camera_pointing.has_value(), "second tick should populate typed camera_pointing command");
    require(
        std::abs(second.camera_pointing->pitch_rad - (45.0 * 3.14159265358979323846 / 180.0)) < 1e-6,
        "below-target geometry should request positive typed AirSim pitch with sign=-1");
    require(
        second.camera_pointing->pitch_clamped == false,
        "45 degree pitch should not clamp with +/-80 degree limits");

    bool found_second_intent = false;
    for (const auto& event : second.events) {
        if (event.find("\"event\":\"camera_pointing_intent\"") != std::string::npos) {
            require(
                event.find("\"pitch_deg\":45.000000") != std::string::npos,
                "second tick camera_pointing_intent pitch_deg should be 45.000000");
            found_second_intent = true;
        }
    }
    require(found_second_intent, "second tick should also emit camera_pointing_intent");
}

void camera_pointing_follows_lifecycle_recovery_and_resets_neutral() {
    auto config = make_circle_config();
    config.vertical_stare_mode = dedalus::ObjectBehaviorVerticalStareMode::Gimbal;
    config.camera_pointing_cameras = {"front_center"};
    config.camera_pitch_min_rad = -80.0 * 3.14159265358979323846 / 180.0;
    config.camera_pitch_max_rad =  80.0 * 3.14159265358979323846 / 180.0;
    config.camera_pointing_prepare_mode = "neutral";
    config.camera_pointing_takeoff_mode = "neutral";
    config.camera_pointing_go_home_mode = "home";
    config.camera_pointing_land_mode = "landing_area";
    config.camera_pointing_complete_mode = "neutral";
    dedalus::ObjectBehaviorMissionController controller{config};

    dedalus::MissionTickInput input;
    // Tick 1: unarmed at origin — home_pose_ set to {0,0,0}, enters Prepare
    input.now = dedalus::TimePoint{0};
    input.snapshot = make_circle_snapshot(0, dedalus::Vec3{0.0, 0.0, 0.0}, dedalus::Vec3{10.0, 0.0, 0.0});
    input.snapshot.ego.armed = false;
    input.snapshot.ego.height_m = 0.0;
    auto prepare = controller.tick(input);
    require(prepare.camera_pointing.has_value(), "Prepare should emit neutral camera reset");
    require(prepare.camera_pointing->mode == "neutral", "Prepare camera pointing should be neutral");
    require(std::abs(prepare.camera_pointing->pitch_rad) < 1e-9, "Prepare neutral reset should use pitch 0");

    // Tick 2: armed → Takeoff
    input.now = dedalus::TimePoint{100000000};
    input.snapshot.ego.armed = true;
    auto takeoff_entry = controller.tick(input);
    require(takeoff_entry.camera_pointing.has_value(), "Prepare-to-Takeoff tick should emit neutral camera reset");
    require(takeoff_entry.camera_pointing->mode == "neutral", "Prepare-to-Takeoff camera pointing should be neutral");
    require(std::abs(takeoff_entry.camera_pointing->pitch_rad) < 1e-9, "Prepare-to-Takeoff neutral reset should use pitch 0");

    // Tick 3: at safe height → ExecuteMission
    input.now = dedalus::TimePoint{200000000};
    input.snapshot = make_circle_snapshot(200000000, dedalus::Vec3{10.0, 0.0, -2.5}, dedalus::Vec3{10.0, 0.0, 0.0});
    auto execute_entry = controller.tick(input);
    require(execute_entry.camera_pointing.has_value(), "Takeoff-to-ExecuteMission tick should emit neutral camera reset");
    require(execute_entry.camera_pointing->mode == "neutral", "Takeoff camera pointing should be neutral");
    require(std::abs(execute_entry.camera_pointing->pitch_rad) < 1e-9, "Takeoff neutral reset should use pitch 0");

    // Tick 4: ExecuteMission with finish_requested=true → camera mode=target, transitions to GoHome
    input.now = dedalus::TimePoint{300000000};
    input.snapshot.timestamp = input.now;
    input.finish_requested = true;
    const auto execute_output = controller.tick(input);
    input.finish_requested = false;
    require(execute_output.camera_pointing.has_value(), "ExecuteMission should emit camera_pointing");
    require(execute_output.camera_pointing->mode == "target",
        "ExecuteMission camera_pointing mode should be target");

    // Tick 5: GoHome — ego at {10,0,-2.5}, home at {0,0,0}, far away → stays GoHome
    input.now = dedalus::TimePoint{400000000};
    input.snapshot.timestamp = input.now;
    const auto go_home_output = controller.tick(input);
    require(go_home_output.camera_pointing.has_value(), "GoHome should emit camera_pointing");
    require(go_home_output.camera_pointing->mode == "home",
        "GoHome camera_pointing mode should be home");

    // Tick 6: GoHome — ego arrives at home XY {0,0,-2.5} → transitions to Land
    input.now = dedalus::TimePoint{500000000};
    input.snapshot = make_circle_snapshot(500000000, dedalus::Vec3{0.0, 0.0, -2.5}, dedalus::Vec3{10.0, 0.0, 0.0});
    const auto go_home_arrived = controller.tick(input);
    require(go_home_arrived.camera_pointing.has_value(), "GoHome arrival tick should emit camera_pointing");
    require(go_home_arrived.camera_pointing->mode == "home",
        "GoHome arrival camera_pointing mode should be home");

    // Tick 7: Land — height above kLandHeightM → camera mode=landing_area, stays in Land
    input.now = dedalus::TimePoint{600000000};
    input.snapshot.timestamp = input.now;
    const auto land_output = controller.tick(input);
    require(land_output.camera_pointing.has_value(), "Land should emit camera_pointing");
    require(land_output.camera_pointing->mode == "landing_area",
        "Land camera_pointing mode should be landing_area");

    // Tick 8: Land — height=0.1 ≤ kLandHeightM → camera mode=landing_area, transitions to Complete
    input.now = dedalus::TimePoint{700000000};
    input.snapshot = make_circle_snapshot(700000000, dedalus::Vec3{0.0, 0.0, -0.1}, dedalus::Vec3{10.0, 0.0, 0.0});
    (void)controller.tick(input);

    // Tick 9: Complete → camera mode=neutral, pitch_rad=0
    input.now = dedalus::TimePoint{800000000};
    input.snapshot.timestamp = input.now;
    const auto complete_output = controller.tick(input);
    require(complete_output.camera_pointing.has_value(), "Complete should emit camera_pointing");
    require(complete_output.camera_pointing->mode == "neutral",
        "Complete camera_pointing mode should be neutral");
    require_near(complete_output.camera_pointing->pitch_rad, 0.0, 1e-9,
        "Complete camera_pointing pitch_rad should be 0");
}

void sequence_steps_execute_with_per_step_yaw_and_camera_modes() {
    auto config = make_circle_config();
    config.yaw_mode = dedalus::ObjectBehaviorYawMode::Trajectory;
    config.vertical_stare_mode = dedalus::ObjectBehaviorVerticalStareMode::Gimbal;
    config.camera_pointing_cameras = {"front_center", "0"};
    config.camera_pitch_min_rad = -80.0 * 3.14159265358979323846 / 180.0;
    config.camera_pitch_max_rad = 80.0 * 3.14159265358979323846 / 180.0;
    config.behavior_spec.behavior.type = dedalus::BehaviorType::Sequence;
    config.behavior_spec.behavior.steps.clear();

    dedalus::BehaviorSpec approach;
    approach.type = dedalus::BehaviorType::Approach;
    approach.stop_distance_m = 8.0;
    approach.position_tolerance_m = 0.5;
    approach.max_speed_mps = 2.0;
    approach.altitude_offset_m = 2.0;
    approach.yaw_mode = "target";
    approach.camera_pointing_mode = "target";

    dedalus::BehaviorSpec circle;
    circle.type = dedalus::BehaviorType::Circle;
    circle.radius_m = 10.0;
    circle.altitude_offset_m = 2.0;
    circle.angular_speed_deg_s = 10.0;
    circle.max_speed_mps = 3.0;
    circle.orbit_count = 0.01;
    circle.yaw_mode = "trajectory";
    circle.camera_pointing_mode = "neutral";

    config.behavior_spec.behavior.steps.push_back(approach);
    config.behavior_spec.behavior.steps.push_back(circle);

    dedalus::ObjectBehaviorMissionController controller{config};

    auto approach_tick = first_execute_tick(
        controller,
        make_circle_snapshot(
            300000000,
            dedalus::Vec3{20.0, 0.0, -2.0},
            dedalus::Vec3{0.0, 0.0, 0.0}));
    require(approach_tick.command.has_value(), "sequence approach should emit command");
    require(approach_tick.command->yaw_source == "target", "approach step yaw_mode=target should yaw to target");
    require(approach_tick.camera_pointing.has_value(), "approach step should emit camera pointing");
    require(approach_tick.camera_pointing->mode == "target", "approach step camera mode should be target");

    dedalus::MissionTickInput input;
    input.now = dedalus::TimePoint{400000000};
    input.snapshot = make_circle_snapshot(
        400000000,
        dedalus::Vec3{8.1, 0.0, -2.0},
        dedalus::Vec3{0.0, 0.0, 0.0});
    const auto transition = controller.tick(input);
    require(transition.status == "object_behavior_sequence_step_complete", "approach standoff should advance sequence");

    input.now = dedalus::TimePoint{500000000};
    input.snapshot = make_circle_snapshot(
        500000000,
        dedalus::Vec3{10.0, 0.0, -2.0},
        dedalus::Vec3{0.0, 0.0, 0.0});
    const auto circle_tick = controller.tick(input);
    require(circle_tick.command.has_value(), "sequence circle should emit command");
    require(circle_tick.command->yaw_source == "trajectory", "circle step yaw_mode=trajectory should yaw from motion");
    require(circle_tick.camera_pointing.has_value(), "circle step should emit camera pointing");
    require(circle_tick.camera_pointing->mode == "neutral", "circle step camera mode override should be neutral");
}

}  // namespace

int main() {
    try {
        lifecycle_gates_before_behavior_and_emits_events();
        landing_and_disarm_reach_complete_status();
        finish_requested_completes_behavior();
        approach_moving_target_velocity_matches_standoff_center();
        circle_static_target_uses_continuous_orbit_capture();
        circle_moving_target_command_includes_target_velocity();
        circle_moving_target_preserves_relative_orbit_authority_under_speed_limit();
        altitude_profile_is_opt_in_and_bounds_vertical_speed();
        missing_altitude_profile_keeps_legacy_altitude_offset();
        circle_tangent_velocity_has_correct_direction();
        circle_radial_correction_pushes_inward_and_outward();
        circle_command_speed_is_clamped();
        circle_display_detail_transitions_arriving_to_circling();
        circle_target_yaw_points_at_selected_target_not_velocity();
        vertical_stare_gimbal_emits_camera_pointing_intent();
        camera_pointing_follows_lifecycle_recovery_and_resets_neutral();
        sequence_steps_execute_with_per_step_yaw_and_camera_modes();
    } catch (const std::exception& exc) {
        std::cerr << "test_object_behavior_mission_controller failed: " << exc.what() << '\n';
        return 1;
    }
    return 0;
}
