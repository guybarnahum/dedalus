#pragma once

#include <mutex>
#include <optional>
#include <string>

#include "dedalus/behavior/mission_controller.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

namespace dedalus {

class LatestWorldSnapshot {
public:
    void publish(WorldSnapshot snapshot) {
        std::lock_guard<std::mutex> lock{mutex_};
        if (snapshot_.has_value()) {
            snapshot.flight_control = snapshot_->flight_control;
            snapshot.flight_control.updated_at = snapshot.timestamp;
            apply_ego_confirmation(snapshot);
        }
        snapshot_ = std::move(snapshot);
    }

    void mark_command_dispatched(
        FlightCommandKind kind,
        TimePoint timestamp,
        const std::string& status) {
        std::lock_guard<std::mutex> lock{mutex_};
        if (!snapshot_.has_value()) {
            snapshot_ = WorldSnapshot{};
        }
        snapshot_->timestamp = timestamp;
        snapshot_->flight_control.updated_at = timestamp;
        snapshot_->flight_control.status = status;
        switch (kind) {
            case FlightCommandKind::Arm:
                snapshot_->flight_control.arm_state = FlightControlArmState::ArmRequested;
                snapshot_->flight_control.last_arm_request_at = timestamp;
                break;
            case FlightCommandKind::Disarm:
                snapshot_->flight_control.arm_state = FlightControlArmState::DisarmRequested;
                snapshot_->flight_control.last_disarm_request_at = timestamp;
                break;
            case FlightCommandKind::Velocity:
            default:
                break;
        }
    }

    void mark_command_failed(
        FlightCommandKind kind,
        TimePoint timestamp,
        const std::string& status) {
        std::lock_guard<std::mutex> lock{mutex_};
        if (!snapshot_.has_value()) {
            snapshot_ = WorldSnapshot{};
        }
        snapshot_->timestamp = timestamp;
        snapshot_->flight_control.updated_at = timestamp;
        snapshot_->flight_control.status = status;
        switch (kind) {
            case FlightCommandKind::Arm:
                snapshot_->flight_control.arm_state = FlightControlArmState::ArmFailed;
                break;
            case FlightCommandKind::Disarm:
                snapshot_->flight_control.arm_state = FlightControlArmState::DisarmFailed;
                break;
            case FlightCommandKind::Velocity:
            default:
                break;
        }
    }

    [[nodiscard]] std::optional<WorldSnapshot> latest() const {
        std::lock_guard<std::mutex> lock{mutex_};
        return snapshot_;
    }

private:
    static void apply_ego_confirmation(WorldSnapshot& snapshot) {
        if (!snapshot.ego.armed_valid) {
            return;
        }
        if (snapshot.ego.armed) {
            snapshot.flight_control.arm_state = FlightControlArmState::ArmedConfirmed;
            snapshot.flight_control.status = "armed_confirmed_by_ego";
        } else if (snapshot.flight_control.arm_state == FlightControlArmState::DisarmRequested ||
                   snapshot.flight_control.arm_state == FlightControlArmState::ArmedConfirmed ||
                   snapshot.flight_control.arm_state == FlightControlArmState::DisarmedConfirmed ||
                   snapshot.flight_control.arm_state == FlightControlArmState::Unknown) {
            snapshot.flight_control.arm_state = FlightControlArmState::DisarmedConfirmed;
            snapshot.flight_control.status = "disarmed_confirmed_by_ego";
        }
    }

    mutable std::mutex mutex_;
    std::optional<WorldSnapshot> snapshot_;
};

}  // namespace dedalus
