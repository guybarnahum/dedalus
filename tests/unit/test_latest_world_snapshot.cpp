#include <iostream>
#include <string>

#include "dedalus/behavior/latest_world_snapshot.hpp"

namespace {

bool expect_arm_state(
    const dedalus::LatestWorldSnapshot& latest_snapshot,
    dedalus::FlightControlArmState expected,
    const std::string& context) {
    const auto snapshot = latest_snapshot.latest();
    if (!snapshot) {
        std::cerr << context << ": expected a latest snapshot\n";
        return false;
    }
    if (snapshot->flight_control.arm_state != expected) {
        std::cerr << context << ": unexpected arm_state\n";
        return false;
    }
    return true;
}

dedalus::WorldSnapshot ego_snapshot(
    std::int64_t timestamp_ns,
    bool armed_valid,
    bool armed) {
    dedalus::WorldSnapshot snapshot;
    snapshot.timestamp = dedalus::TimePoint{timestamp_ns};
    snapshot.ego.timestamp = dedalus::TimePoint{timestamp_ns};
    snapshot.ego.armed_valid = armed_valid;
    snapshot.ego.armed = armed;
    return snapshot;
}

}  // namespace

int main() {
    dedalus::LatestWorldSnapshot latest_snapshot;

    latest_snapshot.mark_command_dispatched(
        dedalus::FlightCommandKind::Arm,
        dedalus::TimePoint{100},
        "arm_dispatch_ok");
    if (!expect_arm_state(
            latest_snapshot,
            dedalus::FlightControlArmState::ArmRequested,
            "mark Arm dispatched")) {
        return 1;
    }

    latest_snapshot.publish(ego_snapshot(200, false, false));
    if (!expect_arm_state(
            latest_snapshot,
            dedalus::FlightControlArmState::ArmRequested,
            "fresh ego without armed validity preserves ArmRequested")) {
        return 1;
    }

    latest_snapshot.publish(ego_snapshot(300, true, true));
    if (!expect_arm_state(
            latest_snapshot,
            dedalus::FlightControlArmState::ArmedConfirmed,
            "fresh ego armed confirms ArmRequested")) {
        return 1;
    }

    latest_snapshot.mark_command_dispatched(
        dedalus::FlightCommandKind::Disarm,
        dedalus::TimePoint{400},
        "disarm_dispatch_ok");
    if (!expect_arm_state(
            latest_snapshot,
            dedalus::FlightControlArmState::DisarmRequested,
            "mark Disarm dispatched")) {
        return 1;
    }

    latest_snapshot.publish(ego_snapshot(500, true, false));
    if (!expect_arm_state(
            latest_snapshot,
            dedalus::FlightControlArmState::DisarmedConfirmed,
            "fresh ego disarmed confirms DisarmRequested")) {
        return 1;
    }

    return 0;
}
