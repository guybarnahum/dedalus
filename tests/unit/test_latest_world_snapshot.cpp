#include <iostream>
#include <string>

#include "dedalus/behavior/flight_control_state_tracker.hpp"

namespace {

bool expect_arm_state(
    const dedalus::FlightControlStateTracker& tracker,
    dedalus::FlightControlArmState expected,
    const std::string& context) {
    if (tracker.state().arm_state != expected) {
        std::cerr << context << ": unexpected arm_state\n";
        return false;
    }
    return true;
}

bool expect_snapshot_arm_state(
    const dedalus::WorldSnapshot& snapshot,
    dedalus::FlightControlArmState expected,
    const std::string& context) {
    if (snapshot.flight_control.arm_state != expected) {
        std::cerr << context << ": unexpected snapshot arm_state\n";
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
    dedalus::FlightControlStateTracker tracker;

    tracker.on_command_dispatched(
        dedalus::FlightCommandKind::Arm,
        dedalus::TimePoint{100},
        "arm_dispatch_ok");
    if (!expect_arm_state(tracker, dedalus::FlightControlArmState::ArmRequested,
                          "on_command_dispatched Arm sets ArmRequested")) {
        return 1;
    }

    auto s1 = ego_snapshot(200, false, false);
    tracker.apply_to_snapshot(s1);
    if (!expect_snapshot_arm_state(s1, dedalus::FlightControlArmState::ArmRequested,
                                   "ego without armed_valid preserves ArmRequested")) {
        return 1;
    }
    if (!expect_arm_state(tracker, dedalus::FlightControlArmState::ArmRequested,
                          "tracker state unchanged when ego not valid")) {
        return 1;
    }

    auto s2 = ego_snapshot(300, true, true);
    tracker.apply_to_snapshot(s2);
    if (!expect_snapshot_arm_state(s2, dedalus::FlightControlArmState::ArmedConfirmed,
                                   "ego armed confirms ArmRequested -> ArmedConfirmed")) {
        return 1;
    }
    if (!expect_arm_state(tracker, dedalus::FlightControlArmState::ArmedConfirmed,
                          "tracker state updated to ArmedConfirmed")) {
        return 1;
    }

    tracker.on_command_dispatched(
        dedalus::FlightCommandKind::Disarm,
        dedalus::TimePoint{400},
        "disarm_dispatch_ok");
    if (!expect_arm_state(tracker, dedalus::FlightControlArmState::DisarmRequested,
                          "on_command_dispatched Disarm sets DisarmRequested")) {
        return 1;
    }

    auto s3 = ego_snapshot(500, true, false);
    tracker.apply_to_snapshot(s3);
    if (!expect_snapshot_arm_state(s3, dedalus::FlightControlArmState::DisarmedConfirmed,
                                   "ego disarmed confirms DisarmRequested -> DisarmedConfirmed")) {
        return 1;
    }
    if (!expect_arm_state(tracker, dedalus::FlightControlArmState::DisarmedConfirmed,
                          "tracker state updated to DisarmedConfirmed")) {
        return 1;
    }

    return 0;
}

