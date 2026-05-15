#include <iostream>

#include "dedalus/behavior/trajectory_mission_controller.hpp"

namespace {

dedalus::WorldSnapshot snapshot_at_height(double height_m) {
    dedalus::WorldSnapshot snapshot;
    snapshot.timestamp = dedalus::TimePoint{0};
    snapshot.active_map_frame_id = dedalus::MapFrameId{"map_test"};
    snapshot.ego.timestamp = snapshot.timestamp;
    snapshot.ego.map_frame_id = snapshot.active_map_frame_id;
    snapshot.ego.local_T_body.position = dedalus::Vec3{0.0, 0.0, -height_m};
    snapshot.ego.height_m = height_m;
    snapshot.ego.height_valid = true;
    snapshot.ego.flight_status = height_m > 0.25 ? dedalus::EgoFlightStatus::Airborne : dedalus::EgoFlightStatus::Landed;
    snapshot.ego.confidence = 0.85F;
    return snapshot;
}

dedalus::MissionTickInput input_at(double seconds, double height_m) {
    auto snapshot = snapshot_at_height(height_m);
    snapshot.timestamp = dedalus::TimePoint{static_cast<dedalus::Nanoseconds>(seconds * 1'000'000'000.0)};
    snapshot.ego.timestamp = snapshot.timestamp;
    return dedalus::MissionTickInput{snapshot.timestamp, snapshot};
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

}  // namespace

int main() {
    dedalus::TrajectoryMissionConfig config;
    config.safe_height_m = 2.0;
    config.takeoff_velocity_mps = 1.0;
    config.go_home_velocity_mps = 1.0;
    config.land_velocity_mps = 0.5;

    dedalus::TrajectorySegment hold;
    hold.type = "hold";
    hold.label = "test_hold";
    hold.duration_s = 1.0;
    hold.vx_mps = 0.0;
    hold.vy_mps = 0.0;
    hold.vz_mps = 0.0;
    config.segments.push_back(hold);

    dedalus::TrajectoryMissionController controller{config};

    auto output = controller.tick(input_at(0.0, 0.0));
    if (!require_state(output, dedalus::MissionLifecycleState::Prepare, "initial arm tick") ||
        !require_command_kind(output, dedalus::FlightCommandKind::Arm, "initial arm tick")) {
        return 1;
    }

    output = controller.tick(input_at(0.1, 0.0));
    if (!require_state(output, dedalus::MissionLifecycleState::Takeoff, "armed transition")) {
        return 1;
    }

    output = controller.tick(input_at(0.2, 0.0));
    if (!require_state(output, dedalus::MissionLifecycleState::Takeoff, "takeoff low height") ||
        !require_command_kind(output, dedalus::FlightCommandKind::Velocity, "takeoff low height")) {
        return 1;
    }
    if (output.command->velocity_local_mps.z >= 0.0) {
        std::cerr << "takeoff command should climb in NED coordinates with negative z velocity\n";
        return 1;
    }

    output = controller.tick(input_at(0.3, 2.1));
    if (!require_state(output, dedalus::MissionLifecycleState::ExecuteMission, "safe height reached")) {
        return 1;
    }

    output = controller.tick(input_at(0.4, 2.1));
    if (!require_state(output, dedalus::MissionLifecycleState::ExecuteMission, "execute trajectory") ||
        !require_command_kind(output, dedalus::FlightCommandKind::Velocity, "execute trajectory")) {
        return 1;
    }

    output = controller.tick(input_at(1.6, 2.1));
    if (!require_state(output, dedalus::MissionLifecycleState::GoHome, "trajectory complete")) {
        return 1;
    }

    output = controller.tick(input_at(1.7, 2.1));
    if (!require_state(output, dedalus::MissionLifecycleState::Land, "home reached")) {
        return 1;
    }

    output = controller.tick(input_at(1.8, 2.1));
    if (!require_state(output, dedalus::MissionLifecycleState::Land, "landing") ||
        !require_command_kind(output, dedalus::FlightCommandKind::Velocity, "landing")) {
        return 1;
    }
    if (output.command->velocity_local_mps.z <= 0.0) {
        std::cerr << "land command should descend in NED coordinates with positive z velocity\n";
        return 1;
    }

    output = controller.tick(input_at(1.9, 0.0));
    if (!require_state(output, dedalus::MissionLifecycleState::Complete, "landed")) {
        return 1;
    }

    output = controller.tick(input_at(2.0, 0.0));
    if (!require_state(output, dedalus::MissionLifecycleState::Complete, "disarm after landed") ||
        !require_command_kind(output, dedalus::FlightCommandKind::Disarm, "disarm after landed")) {
        return 1;
    }

    return 0;
}
