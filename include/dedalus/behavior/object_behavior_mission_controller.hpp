#pragma once

#include <optional>
#include <string>

#include "dedalus/behavior/behavior_spec.hpp"
#include "dedalus/behavior/mission_controller.hpp"
#include "dedalus/behavior/target_selector.hpp"

namespace dedalus {

struct ObjectBehaviorMissionConfig {
    BehaviorMissionSpec behavior_spec;
    double hold_velocity_mps{0.0};
};

ObjectBehaviorMissionConfig load_object_behavior_mission_config(const MissionOptions& options);

class ObjectBehaviorMissionController final : public MissionController {
public:
    explicit ObjectBehaviorMissionController(ObjectBehaviorMissionConfig config);

    MissionTickOutput tick(const MissionTickInput& input) override;

private:
    [[nodiscard]] VelocityCommand command_from_velocity(TimePoint timestamp, Vec3 velocity_local_mps) const;
    [[nodiscard]] std::string target_event(const TargetSelection& selection) const;
    [[nodiscard]] std::string behavior_event(const std::string& event, const std::string& reason) const;
    [[nodiscard]] bool completion_elapsed(TimePoint now) const;

    ObjectBehaviorMissionConfig config_;
    TargetSelector selector_;
    MissionLifecycleState state_{MissionLifecycleState::Idle};
    TimePoint mission_start_;
    TimePoint behavior_start_;
    bool mission_started_{false};
    bool target_selected_emitted_{false};
    bool behavior_start_emitted_{false};
    bool behavior_complete_emitted_{false};
    std::optional<TargetSelection> previous_selection_;
};

}  // namespace dedalus
