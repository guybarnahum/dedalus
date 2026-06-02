#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "dedalus/behavior/mission_controller.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

namespace dedalus {

class LatestWorldSnapshot {
public:
    void publish(WorldSnapshot snapshot) {
        std::lock_guard<std::mutex> lock{mutex_};
        if (snapshot_) {
            snapshot.flight_control = snapshot_->flight_control;
            snapshot.flight_control.updated_at = snapshot.timestamp;
            apply_ego_confirmation(snapshot);
        }
        snapshot_ = std::make_shared<const WorldSnapshot>(std::move(snapshot));
    }

    void mark_command_dispatched(
        FlightCommandKind kind,
        TimePoint timestamp,
        const std::string& status) {
        std::lock_guard<std::mutex> lock{mutex_};
        WorldSnapshot s = snapshot_ ? *snapshot_ : WorldSnapshot{};
        s.timestamp = timestamp;
        s.flight_control.updated_at = timestamp;
        s.flight_control.status = status;
        switch (kind) {
            case FlightCommandKind::Arm:
                s.flight_control.arm_state = FlightControlArmState::ArmRequested;
                s.flight_control.last_arm_request_at = timestamp;
                break;
            case FlightCommandKind::Disarm:
                s.flight_control.arm_state = FlightControlArmState::DisarmRequested;
                s.flight_control.last_disarm_request_at = timestamp;
                break;
            case FlightCommandKind::Velocity:
            default:
                break;
        }
        snapshot_ = std::make_shared<const WorldSnapshot>(std::move(s));
    }

    void mark_command_failed(
        FlightCommandKind kind,
        TimePoint timestamp,
        const std::string& status) {
        std::lock_guard<std::mutex> lock{mutex_};
        WorldSnapshot s = snapshot_ ? *snapshot_ : WorldSnapshot{};
        s.timestamp = timestamp;
        s.flight_control.updated_at = timestamp;
        s.flight_control.status = status;
        switch (kind) {
            case FlightCommandKind::Arm:
                s.flight_control.arm_state = FlightControlArmState::ArmFailed;
                break;
            case FlightCommandKind::Disarm:
                s.flight_control.arm_state = FlightControlArmState::DisarmFailed;
                break;
            case FlightCommandKind::Velocity:
            default:
                break;
        }
        snapshot_ = std::make_shared<const WorldSnapshot>(std::move(s));
    }

    [[nodiscard]] std::shared_ptr<const WorldSnapshot> latest() const {
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
    std::shared_ptr<const WorldSnapshot> snapshot_;
};

}  // namespace dedalus
