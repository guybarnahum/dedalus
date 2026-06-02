#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>

#include "dedalus/behavior/trajectory_mission_controller.hpp"

namespace {

dedalus::WorldSnapshot snapshot_at_height(double height_m, bool armed, bool armed_valid = true) {
    dedalus::WorldSnapshot snapshot;
    snapshot.timestamp = dedalus::TimePoint{0};
    snapshot.active_map_frame_id = dedalus::MapFrameId{"map_test"};
    snapshot.ego.timestamp = snapshot.timestamp;
    snapshot.ego.map_frame_id = snapshot.active_map_frame_id;
    snapshot.ego.local_T_body.position = dedalus::Vec3{0.0, 0.0, -height_m};
    snapshot.ego.height_m = height_m;
    snapshot.ego.height_valid = true;
    snapshot.ego.armed = armed;
    snapshot.ego.armed_valid = armed_valid;
    snapshot.ego.flight_status = height_m > 0.25 ? dedalus::EgoFlightStatus::Airborne : dedalus::EgoFlightStatus::Landed;
    snapshot.ego.confidence = 0.85F;
    return snapshot;
}

dedalus::MissionTickInput input_at(
    double seconds,
    double height_m,
    bool armed,
    bool armed_valid = true,
    bool finish_requested = false) {
    auto snapshot = snapshot_at_height(height_m, armed, armed_valid);
    snapshot.timestamp = dedalus::TimePoint{static_cast<dedalus::Nanoseconds>(seconds * 1'000'000'000.0)};
    snapshot.ego.timestamp = snapshot.timestamp;
    return dedalus::MissionTickInput{snapshot.timestamp, snapshot, std::nullopt, finish_requested};
}

bool require_state(
    const dedalus::MissionTickOutput& output,
    dedalus::MissionLifecycleState expected,
    const char* label) {
    if (output.state != expected) {
        std::cerr << "unexpected state for " << label << "\n";
        return false;
    }
    return true;
}

bool require_command_kind(
    const dedalus::MissionTickOutput& output,
    dedalus::FlightCommandKind expected,
    const char* label) {
    if (!output.command.has_value()) {
        std::cerr << "missing command for " << label << "\n";
        return false;
    }
    if (output.command->kind != expected) {
        std::cerr << "unexpected command kind for " << label << "\n";
        return false;
    }
    return true;
}

bool require_near(double actual, double expected, double tolerance, const char* label) {
    if (std::abs(actual - expected) > tolerance) {
        std::cerr << "unexpected value for " << label << ": got " << actual
                  << ", expected " << expected << "\n";
        return false;
    }
    return true;
}

}  // namespace

int main() {
    dedalus::TrajectoryMissionConfig config;
    config.safe_height_m = 2.0;
    config.takeoff_velocity_mps = 1.0;
    config.go_home_velocity_mps = 1.0;
    config.land_velocity_mps = 0.5;
    config.arm_retry_interval_s = 1.0;
    config.arm_timeout_s = 5.0;
    config.takeoff_retry_interval_s = 1.0;
    config.land_retry_interval_s = 1.0;
    config.land_timeout_s = 10.0;
    config.disarm_retry_interval_s = 1.0;
    config.disarm_timeout_s = 5.0;

    dedalus::TrajectorySegment hold;
    hold.type = "hold";
    hold.label = "test_hold";
    hold.duration_s = 1.0;
    hold.vx_mps = 0.0;
    hold.vy_mps = 0.0;
    hold.vz_mps = 0.0;
    config.trajectory = dedalus::VelocityTrajectory{{hold}};

    dedalus::TrajectoryMissionController controller{config};

    auto output = controller.tick(input_at(0.0, 0.0, false));
    if (!require_state(output, dedalus::MissionLifecycleState::Prepare, "initial arm tick") ||
        !require_command_kind(output, dedalus::FlightCommandKind::Arm, "initial arm tick")) {
        return 1;
    }

    output = controller.tick(input_at(0.5, 0.0, false));
    if (!require_state(output, dedalus::MissionLifecycleState::Prepare, "waiting before arm retry interval")) {
        return 1;
    }
    if (output.command.has_value()) {
        std::cerr << "controller should not re-emit arm before retry interval\n";
        return 1;
    }

    output = controller.tick(input_at(1.1, 0.0, false));
    if (!require_state(output, dedalus::MissionLifecycleState::Prepare, "arm retry") ||
        !require_command_kind(output, dedalus::FlightCommandKind::Arm, "arm retry")) {
        return 1;
    }

    output = controller.tick(input_at(1.2, 0.0, true));
    if (!require_state(output, dedalus::MissionLifecycleState::Takeoff, "armed telemetry transition")) {
        return 1;
    }

    output = controller.tick(input_at(1.3, 0.0, true));
    if (!require_state(output, dedalus::MissionLifecycleState::Takeoff, "takeoff request") ||
        !require_command_kind(output, dedalus::FlightCommandKind::Takeoff, "takeoff request")) {
        return 1;
    }

    output = controller.tick(input_at(1.8, 0.0, true));
    if (!require_state(output, dedalus::MissionLifecycleState::Takeoff, "waiting for takeoff climb")) {
        return 1;
    }
    if (output.command.has_value()) {
        std::cerr << "controller should not re-emit takeoff before retry interval\n";
        return 1;
    }

    output = controller.tick(input_at(1.9, 0.6, true));
    if (!require_state(output, dedalus::MissionLifecycleState::Takeoff, "takeoff velocity assist") ||
        !require_command_kind(output, dedalus::FlightCommandKind::Velocity, "takeoff velocity assist")) {
        return 1;
    }
    if (output.command->velocity_local_mps.z >= 0.0) {
        std::cerr << "takeoff command should climb in NED coordinates with negative z velocity\n";
        return 1;
    }

    output = controller.tick(input_at(2.0, 2.1, true));
    if (!require_state(output, dedalus::MissionLifecycleState::ExecuteMission, "safe height reached")) {
        return 1;
    }

    output = controller.tick(input_at(2.1, 2.1, true));
    if (!require_state(output, dedalus::MissionLifecycleState::ExecuteMission, "execute trajectory") ||
        !require_command_kind(output, dedalus::FlightCommandKind::Velocity, "execute trajectory")) {
        return 1;
    }

    output = controller.tick(input_at(3.3, 2.1, true));
    if (!require_state(output, dedalus::MissionLifecycleState::GoHome, "trajectory complete")) {
        return 1;
    }

    output = controller.tick(input_at(3.4, 2.1, true));
    if (!require_state(output, dedalus::MissionLifecycleState::Land, "home reached")) {
        return 1;
    }

    output = controller.tick(input_at(3.5, 2.1, true));
    if (!require_state(output, dedalus::MissionLifecycleState::Land, "landing") ||
        !require_command_kind(output, dedalus::FlightCommandKind::Land, "landing")) {
        return 1;
    }
    if (output.status != "landing_command_sent") {
        std::cerr << "controller should label first land command as landing_command_sent\n";
        return 1;
    }

    output = controller.tick(input_at(3.6, 2.1, true));
    if (!require_state(output, dedalus::MissionLifecycleState::Land, "waiting for landed telemetry")) {
        return 1;
    }
    if (output.command.has_value()) {
        std::cerr << "controller should not re-emit land while normal landing is in progress\n";
        return 1;
    }

    output = controller.tick(input_at(3.7, 0.0, true));
    if (!require_state(output, dedalus::MissionLifecycleState::Complete, "landed")) {
        return 1;
    }

    output = controller.tick(input_at(3.8, 0.0, true));
    if (!require_state(output, dedalus::MissionLifecycleState::Complete, "disarm request") ||
        !require_command_kind(output, dedalus::FlightCommandKind::Disarm, "disarm request")) {
        return 1;
    }

    output = controller.tick(input_at(4.3, 0.0, true));
    if (!require_state(output, dedalus::MissionLifecycleState::Complete, "waiting before disarm retry interval")) {
        return 1;
    }
    if (output.command.has_value()) {
        std::cerr << "controller should not re-emit disarm before retry interval\n";
        return 1;
    }

    output = controller.tick(input_at(4.9, 0.0, true));
    if (!require_state(output, dedalus::MissionLifecycleState::Complete, "disarm retry") ||
        !require_command_kind(output, dedalus::FlightCommandKind::Disarm, "disarm retry")) {
        return 1;
    }

    output = controller.tick(input_at(5.0, 0.0, false));
    if (!require_state(output, dedalus::MissionLifecycleState::Complete, "disarm confirmed complete")) {
        return 1;
    }
    if (output.status != "complete") {
        std::cerr << "controller should report complete after disarmed telemetry\n";
        return 1;
    }

    dedalus::TrajectoryMissionController land_timeout_controller{config};
    output = land_timeout_controller.tick(input_at(0.0, 0.0, false));
    output = land_timeout_controller.tick(input_at(0.1, 0.0, true));
    output = land_timeout_controller.tick(input_at(0.2, 2.1, true));
    output = land_timeout_controller.tick(input_at(1.3, 2.1, true));
    output = land_timeout_controller.tick(input_at(1.4, 2.1, true));
    output = land_timeout_controller.tick(input_at(1.5, 2.1, true));
    output = land_timeout_controller.tick(input_at(12.0, 2.1, true));
    if (!require_state(output, dedalus::MissionLifecycleState::Abort, "land timeout terminal abort")) {
        return 1;
    }
    if (output.status != "abort") {
        std::cerr << "controller should terminal-abort when landing recovery cannot complete\n";
        return 1;
    }

    const auto trajectory_path = std::filesystem::temp_directory_path() / "dedalus_test_keyframes_trajectory.json";
    {
        std::ofstream trajectory_file{trajectory_path};
        trajectory_file << R"JSON({
  "name": "unit_keyframes",
  "rate_hz": 10,
  "segments": [
    {
      "type": "velocity_keyframes",
      "label": "interpolate",
      "keyframes": [
        { "t": 0, "vx_mps": 0.0, "vy_mps": 0.0, "vz_mps": 0.0 },
        { "t": 6, "vx_mps": 1.0, "vy_mps": -2.0, "vz_mps": 0.5 }
      ]
    }
  ]
})JSON";
    }

    dedalus::MissionOptions options;
    options.safe_height_m = 2.0;
    options.trajectory_path = trajectory_path.string();
    auto loaded_config = dedalus::load_trajectory_mission_config(options);
    if (loaded_config.trajectory.size() != 1U || loaded_config.trajectory.segment(0).keyframes.size() != 2U) {
        std::cerr << "loaded trajectory did not preserve keyframes\n";
        return 1;
    }
    if (!require_near(loaded_config.trajectory.segment(0).duration_s, 6.0, 1e-6, "keyframe duration")) {
        return 1;
    }
    const auto direct_velocity = loaded_config.trajectory.velocity_at(0U, 3.0);
    if (!require_near(direct_velocity.x, 0.5, 1e-6, "direct interpolated vx") ||
        !require_near(direct_velocity.y, -1.0, 1e-6, "direct interpolated vy") ||
        !require_near(direct_velocity.z, 0.25, 1e-6, "direct interpolated vz")) {
        return 1;
    }

    dedalus::TrajectoryMissionController loaded_controller{loaded_config};
    output = loaded_controller.tick(input_at(0.0, 0.0, false));
    output = loaded_controller.tick(input_at(0.1, 0.0, true));
    output = loaded_controller.tick(input_at(0.2, 2.1, true));
    if (!require_state(output, dedalus::MissionLifecycleState::ExecuteMission, "loaded safe height")) {
        return 1;
    }
    output = loaded_controller.tick(input_at(3.2, 2.1, true));
    if (!require_state(output, dedalus::MissionLifecycleState::ExecuteMission, "loaded keyframe execute") ||
        !require_command_kind(output, dedalus::FlightCommandKind::Velocity, "loaded keyframe execute")) {
        return 1;
    }
    if (!require_near(output.command->velocity_local_mps.x, 0.5, 1e-6, "interpolated vx") ||
        !require_near(output.command->velocity_local_mps.y, -1.0, 1e-6, "interpolated vy") ||
        !require_near(output.command->velocity_local_mps.z, 0.25, 1e-6, "interpolated vz")) {
        return 1;
    }
    output = loaded_controller.tick(input_at(6.4, 2.1, true));
    if (!require_state(output, dedalus::MissionLifecycleState::GoHome, "loaded keyframe complete")) {
        return 1;
    }
    std::filesystem::remove(trajectory_path);

    dedalus::TrajectoryMissionController finish_controller{config};
    output = finish_controller.tick(input_at(0.0, 0.0, false));
    if (!require_state(output, dedalus::MissionLifecycleState::Prepare, "finish initial arm") ||
        !require_command_kind(output, dedalus::FlightCommandKind::Arm, "finish initial arm")) {
        return 1;
    }
    output = finish_controller.tick(input_at(0.1, 0.0, true));
    if (!require_state(output, dedalus::MissionLifecycleState::Takeoff, "finish armed transition")) {
        return 1;
    }
    output = finish_controller.tick(input_at(0.2, 2.1, true));
    if (!require_state(output, dedalus::MissionLifecycleState::ExecuteMission, "finish safe height")) {
        return 1;
    }
    output = finish_controller.tick(input_at(0.3, 2.1, true, true, true));
    if (!require_state(output, dedalus::MissionLifecycleState::GoHome, "finish requested go home")) {
        return 1;
    }
    output = finish_controller.tick(input_at(0.4, 2.1, true, true, true));
    if (!require_state(output, dedalus::MissionLifecycleState::Land, "finish requested home reached")) {
        return 1;
    }
    output = finish_controller.tick(input_at(0.5, 2.1, true, true, true));
    if (!require_state(output, dedalus::MissionLifecycleState::Land, "finish requested landing") ||
        !require_command_kind(output, dedalus::FlightCommandKind::Land, "finish requested landing")) {
        return 1;
    }
    output = finish_controller.tick(input_at(0.6, 0.0, true, true, true));
    if (!require_state(output, dedalus::MissionLifecycleState::Complete, "finish requested landed")) {
        return 1;
    }
    output = finish_controller.tick(input_at(0.7, 0.0, true, true, true));
    if (!require_state(output, dedalus::MissionLifecycleState::Complete, "finish requested disarm") ||
        !require_command_kind(output, dedalus::FlightCommandKind::Disarm, "finish requested disarm")) {
        return 1;
    }
    output = finish_controller.tick(input_at(0.8, 0.0, false, true, true));
    if (!require_state(output, dedalus::MissionLifecycleState::Complete, "finish requested complete")) {
        return 1;
    }

    dedalus::TrajectoryMissionController timeout_controller{config};
    output = timeout_controller.tick(input_at(0.0, 0.0, false));
    if (!require_state(output, dedalus::MissionLifecycleState::Prepare, "timeout controller initial arm") ||
        !require_command_kind(output, dedalus::FlightCommandKind::Arm, "timeout controller initial arm")) {
        return 1;
    }
    output = timeout_controller.tick(input_at(6.0, 0.0, false));
    if (!require_state(output, dedalus::MissionLifecycleState::Abort, "arm timeout abort")) {
        return 1;
    }

    return 0;
}
