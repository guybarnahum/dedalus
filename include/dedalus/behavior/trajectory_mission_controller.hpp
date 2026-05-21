#pragma once

#include <cstddef>
#include <string>

#include "dedalus/behavior/mission_controller.hpp"
#include "dedalus/behavior/velocity_trajectory.hpp"

namespace dedalus {

struct TrajectoryMissionConfig {
    double safe_height_m{8.0};
    double takeoff_velocity_mps{1.0};
    double go_home_velocity_mps{1.0};
    double land_velocity_mps{0.5};
    double yaw_offset_rad{0.0};
    double arm_retry_interval_s{1.0};
    double arm_timeout_s{10.0};
    double arm_dispatch_fallback_s{0.0};
    double takeoff_retry_interval_s{1.0};
    double land_retry_interval_s{1.0};
    double land_timeout_s{60.0};
    double disarm_retry_interval_s{1.0};
    double disarm_timeout_s{10.0};
    std::string home_policy{"initial_ego_pose"};
    VelocityTrajectory trajectory{VelocityTrajectory::default_hold()};
};

TrajectoryMissionConfig load_trajectory_mission_config(const MissionOptions& options);

class TrajectoryMissionController final : public MissionController {
public:
    explicit TrajectoryMissionController(TrajectoryMissionConfig config);

    MissionTickOutput tick(const MissionTickInput& input) override;

private:
    [[nodiscard]] VelocityCommand command_from_velocity(
        TimePoint timestamp,
        Vec3 velocity_local_mps,
        double yaw_offset_rad = 0.0) const;
    [[nodiscard]] VelocityCommand command_with_kind(
        TimePoint timestamp,
        FlightCommandKind kind) const;
    [[nodiscard]] VelocityCommand trajectory_command(TimePoint timestamp) const;
    [[nodiscard]] bool trajectory_complete() const;
    void advance_segment_if_needed();
    void begin_abort_recovery(TimePoint now, double height_m, const std::string& reason);

    TrajectoryMissionConfig config_;
    MissionLifecycleState state_{MissionLifecycleState::Idle};
    TimePoint mission_start_;
    TimePoint state_start_;
    TimePoint arm_last_command_time_;
    TimePoint takeoff_last_command_time_;
    TimePoint land_last_command_time_;
    TimePoint disarm_last_command_time_;
    bool mission_started_{false};
    bool home_initialized_{false};
    bool arm_command_sent_{false};
    bool takeoff_command_sent_{false};
    bool land_command_sent_{false};
    bool disarm_command_sent_{false};
    bool aborting_{false};
    std::string abort_reason_;
    Pose3 home_pose_;
    std::size_t segment_index_{0U};
    double segment_elapsed_s_{0.0};
    TimePoint last_tick_time_;
};

}  // namespace dedalus
