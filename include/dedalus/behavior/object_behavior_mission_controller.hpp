#pragma once

#include <optional>
#include <string>
#include <vector>
#include <cstddef>

#include "dedalus/behavior/behavior_spec.hpp"
#include "dedalus/behavior/mission_controller.hpp"
#include "dedalus/behavior/target_selector.hpp"

namespace dedalus {

enum class ObjectBehaviorAltitudePolicy {
    TargetRelative,
    SafeHeightFloor,
};

enum class ObjectBehaviorYawMode {
    Trajectory,
    Target,
    Hold,
    None,
};

enum class ObjectBehaviorVerticalStareMode {
    None,
    DebugOnly,
    Gimbal,
};

struct ObjectBehaviorMissionConfig {
    BehaviorMissionSpec behavior_spec;
    double hold_velocity_mps{0.0};
    double safe_height_m{8.0};
    double takeoff_velocity_mps{1.0};
    double go_home_velocity_mps{1.0};
    double yaw_offset_rad{0.0};
    double yaw_min_speed_mps{0.35};
    bool yaw_hold_last_when_unstable{true};
    ObjectBehaviorYawMode yaw_mode{ObjectBehaviorYawMode::Trajectory};
    ObjectBehaviorVerticalStareMode vertical_stare_mode{ObjectBehaviorVerticalStareMode::None};
    bool vertical_stare_warn_if_unavailable{true};
    std::vector<std::string> camera_pointing_cameras;
    double camera_pitch_min_rad{-1.3962634015954636};
    double camera_pitch_max_rad{0.6108652381980153};
    double camera_pitch_sign{-1.0};
    double camera_pitch_offset_rad{0.0};
    std::string camera_pointing_prepare_mode{"neutral"};
    std::string camera_pointing_takeoff_mode{"neutral"};
    std::string camera_pointing_go_home_mode{"home"};
    std::string camera_pointing_land_mode{"landing_area"};
    std::string camera_pointing_complete_mode{"neutral"};
    int debug_every_n_ticks{0};
    int debug_level{1};
    ObjectBehaviorAltitudePolicy altitude_policy{ObjectBehaviorAltitudePolicy::TargetRelative};
    bool follow_observation_geometry_enabled{false};
    bool zero_target_velocity{false};
    double follow_min_standoff_m{8.0};
    double follow_max_elevation_angle_deg{35.0};
    double follow_arrival_slow_radius_m{8.0};
    double follow_arrival_hold_radius_m{2.0};
    double follow_arrival_kp{0.35};
    double arm_retry_interval_s{1.0};
    double arm_timeout_s{10.0};
    double arm_dispatch_fallback_s{0.0};
    double takeoff_retry_interval_s{1.0};
    double land_retry_interval_s{1.0};
    double land_timeout_s{60.0};
    double disarm_retry_interval_s{1.0};
    double disarm_timeout_s{10.0};
    std::string home_policy{"initial_ego_pose"};
};

ObjectBehaviorMissionConfig load_object_behavior_mission_config(const MissionOptions& options);

class ObjectBehaviorMissionController final : public MissionController {
public:
    explicit ObjectBehaviorMissionController(ObjectBehaviorMissionConfig config);

    MissionTickOutput tick(const MissionTickInput& input) override;

private:
    [[nodiscard]] VelocityCommand command_from_velocity(
        TimePoint timestamp,
        Vec3 velocity_local_mps,
        double yaw_offset_rad = 0.0) const;
    [[nodiscard]] VelocityCommand command_from_behavior_velocity(
        TimePoint timestamp,
        Vec3 velocity_local_mps,
        const EgoState& ego,
        const TargetSelection& selection,
        double yaw_offset_rad,
        ObjectBehaviorYawMode yaw_mode) const;
    [[nodiscard]] VelocityCommand command_with_kind(TimePoint timestamp, FlightCommandKind kind) const;
    [[nodiscard]] std::string target_event(const TargetSelection& selection) const;
    [[nodiscard]] std::string behavior_event(const std::string& event, const std::string& reason) const;
    [[nodiscard]] std::string sequence_step_event(const std::string& event, const BehaviorSpec& behavior, std::size_t index, const std::string& reason) const;
    [[nodiscard]] std::optional<CameraPointingCommand> camera_pointing_command(
        TimePoint timestamp,
        const EgoState& ego,
        const TargetSelection& selection,
        const std::string& mode) const;
    [[nodiscard]] std::optional<CameraPointingCommand> camera_pointing_command_for_behavior(
        TimePoint timestamp,
        const EgoState& ego,
        const TargetSelection& selection,
        const BehaviorSpec& behavior) const;
    [[nodiscard]] std::optional<CameraPointingCommand> camera_pointing_command_to_point(
        TimePoint timestamp,
        const EgoState& ego,
        const Vec3& target_position_local,
        const std::string& mode) const;
    [[nodiscard]] std::optional<CameraPointingCommand> neutral_camera_pointing_command(
        TimePoint timestamp,
        const std::string& mode) const;
    [[nodiscard]] std::string camera_pointing_intent_event(const CameraPointingCommand& command) const;
    void emit_camera_pointing(MissionTickOutput& output, const CameraPointingCommand& command) const;
    [[nodiscard]] Vec3 go_home_velocity(const EgoState& ego) const;
    [[nodiscard]] bool completion_elapsed(TimePoint now) const;
    void begin_abort_recovery(TimePoint now, double height_m, const std::string& reason);
    [[nodiscard]] const BehaviorSpec& active_behavior() const;
    [[nodiscard]] bool sequence_active() const;
    [[nodiscard]] bool active_behavior_is_last_sequence_step() const;
    [[nodiscard]] ObjectBehaviorYawMode yaw_mode_for_behavior(const BehaviorSpec& behavior) const;
    bool update_circle_orbit_progress(const BehaviorSpec& behavior, bool circling, double orbit_angle_rad);
    void reset_sequence_step(TimePoint now);
    void reset_behavior_run(TimePoint now);

    ObjectBehaviorMissionConfig config_;
    TargetSelector selector_;
    MissionLifecycleState state_{MissionLifecycleState::Idle};
    TimePoint mission_start_;
    TimePoint state_start_;
    TimePoint last_tick_time_;
    TimePoint behavior_start_;
    TimePoint arm_last_command_time_;
    TimePoint takeoff_last_command_time_;
    TimePoint land_last_command_time_;
    TimePoint disarm_last_command_time_;
    Pose3 home_pose_;
    bool mission_started_{false};
    bool home_initialized_{false};
    bool target_selected_emitted_{false};
    bool behavior_start_emitted_{false};
    bool behavior_complete_emitted_{false};
    bool behavior_tick_sample_emitted_{false};
    bool arm_command_sent_{false};
    bool takeoff_command_sent_{false};
    bool land_command_sent_{false};
    bool disarm_command_sent_{false};
    bool aborting_{false};
    std::string abort_reason_;
    std::string last_behavior_display_detail_;
    bool circle_in_orbit_mode_{false};
    bool circle_orbit_tracking_{false};
    double circle_previous_angle_rad_{0.0};
    double circle_completed_orbits_{0.0};
    std::optional<TargetSelection> previous_selection_;
    mutable bool last_stable_yaw_valid_{false};
    mutable double last_stable_yaw_rad_{0.0};
    int execute_tick_count_{0};
    std::size_t sequence_step_index_{0U};
    TimePoint sequence_step_start_;
    bool sequence_step_started_{false};
};

}  // namespace dedalus
