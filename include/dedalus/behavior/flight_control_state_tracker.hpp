#pragma once

#include "dedalus/behavior/mission_controller.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

namespace dedalus {

// Tracks FlightControlState transitions driven by command dispatch outcomes
// and ego confirmation. Owned and mutated exclusively by MissionRuntime on its
// tick thread; LatestWorldSnapshot remains a pure SPSC snapshot holder.
class FlightControlStateTracker {
public:
    void on_command_dispatched(
        FlightCommandKind kind,
        TimePoint timestamp,
        const std::string& status) {
        state_.status = status;
        switch (kind) {
            case FlightCommandKind::Arm:
                state_.arm_state = FlightControlArmState::ArmRequested;
                state_.last_arm_request_at = timestamp;
                break;
            case FlightCommandKind::Disarm:
                state_.arm_state = FlightControlArmState::DisarmRequested;
                state_.last_disarm_request_at = timestamp;
                break;
            default:
                break;
        }
    }

    void on_command_failed(
        FlightCommandKind kind,
        TimePoint /*timestamp*/,
        const std::string& status) {
        state_.status = status;
        switch (kind) {
            case FlightCommandKind::Arm:
                state_.arm_state = FlightControlArmState::ArmFailed;
                break;
            case FlightCommandKind::Disarm:
                state_.arm_state = FlightControlArmState::DisarmFailed;
                break;
            default:
                break;
        }
    }

    // Injects tracked state into snapshot.flight_control, then applies ego
    // confirmation. Updates internal state_ so confirmation persists across
    // ticks when ego data is temporarily unavailable.
    void apply_to_snapshot(WorldSnapshot& snapshot) {
        snapshot.flight_control = state_;
        snapshot.flight_control.updated_at = snapshot.timestamp;
        if (!snapshot.ego.armed_valid) return;
        if (snapshot.ego.armed) {
            state_.arm_state = FlightControlArmState::ArmedConfirmed;
            state_.status = "armed_confirmed_by_ego";
        } else if (state_.arm_state == FlightControlArmState::DisarmRequested ||
                   state_.arm_state == FlightControlArmState::ArmedConfirmed ||
                   state_.arm_state == FlightControlArmState::DisarmedConfirmed ||
                   state_.arm_state == FlightControlArmState::Unknown) {
            state_.arm_state = FlightControlArmState::DisarmedConfirmed;
            state_.status = "disarmed_confirmed_by_ego";
        }
        snapshot.flight_control.arm_state = state_.arm_state;
        snapshot.flight_control.status = state_.status;
    }

    [[nodiscard]] const FlightControlState& state() const { return state_; }

private:
    FlightControlState state_;
};

}  // namespace dedalus
