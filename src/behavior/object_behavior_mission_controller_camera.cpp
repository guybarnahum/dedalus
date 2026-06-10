#include "dedalus/behavior/object_behavior_mission_controller.hpp"

#include "dedalus/behavior/behavior_osd.hpp"
#include "dedalus/core/json_utils.hpp"

#include <cmath>
#include <string>
#include <vector>

namespace dedalus {
namespace {

constexpr double kPi = 3.14159265358979323846;

double rad_to_deg(double rad) {
    return rad * 180.0 / kPi;
}

std::string json_string_array(const std::vector<std::string>& values) {
    std::string out = "[";
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i > 0) out += ",";
        out += q(values[i]);
    }
    out += "]";
    return out;
}

}  // namespace

std::optional<CameraPointingCommand> ObjectBehaviorMissionController::camera_pointing_command(
    TimePoint timestamp,
    const EgoState& ego,
    const TargetSelection& selection,
    const std::string& mode) const {
    const std::string effective_mode = mode.empty() ? "target" : mode;
    if (effective_mode == "neutral" || effective_mode == "reset") {
        return neutral_camera_pointing_command(timestamp, effective_mode);
    }
    if ((effective_mode == "home" || effective_mode == "landing_area") && home_initialized_) {
        return camera_pointing_command_to_point(timestamp, ego, home_pose_.position, effective_mode);
    }
    auto command = camera_pointing_command_to_point(
        timestamp,
        ego,
        selection.position_local,
        effective_mode);
    if (!command) {
        return std::nullopt;
    }
    command->source_track_id = selection.source_track_id.value;
    command->agent_id = selection.agent_id.value;
    command->identity_id = selection.identity_id.value;
    return command;
}

std::optional<CameraPointingCommand> ObjectBehaviorMissionController::camera_pointing_command_for_behavior(
    TimePoint timestamp,
    const EgoState& ego,
    const TargetSelection& selection,
    const BehaviorSpec& behavior) const {
    const std::string mode = behavior.camera_pointing_mode.empty()
        ? "target"
        : behavior.camera_pointing_mode;
    return camera_pointing_command(timestamp, ego, selection, mode);
}

std::optional<CameraPointingCommand> ObjectBehaviorMissionController::camera_pointing_command_to_point(
    TimePoint timestamp,
    const EgoState& ego,
    const Vec3& target_position_local,
    const std::string& mode) const {
    if (config_.vertical_stare_mode == ObjectBehaviorVerticalStareMode::None || mode == "none" || mode == "disabled") {
        return std::nullopt;
    }
    if (mode == "neutral" || mode == "reset") {
        return neutral_camera_pointing_command(timestamp, mode);
    }

    CameraPointingCommand command;
    command.timestamp = timestamp;
    command.cameras = config_.camera_pointing_cameras;
    if (command.cameras.empty()) {
        command.cameras.push_back("front_center");
    }
    command.mode = mode;

    const double dx = target_position_local.x - ego.local_T_body.position.x;
    const double dy = target_position_local.y - ego.local_T_body.position.y;
    const double dz = target_position_local.z - ego.local_T_body.position.z;
    const double raw_range_xy = std::hypot(dx, dy);
    const double range_xy = std::max(raw_range_xy, 1e-6);
    const double elevation_rad = std::atan2(dz, range_xy);
    const double bearing_rad = raw_range_xy > 1e-6 ? std::atan2(dy, dx) : ego.local_T_body.rotation_rpy.z;
    const double unclamped_pitch_rad = config_.camera_pitch_sign * elevation_rad + config_.camera_pitch_offset_rad;
    const double pitch_rad = std::max(config_.camera_pitch_min_rad, std::min(config_.camera_pitch_max_rad, unclamped_pitch_rad));

    command.pitch_rad = pitch_rad;
    command.pitch_unclamped_rad = unclamped_pitch_rad;
    command.pitch_min_rad = config_.camera_pitch_min_rad;
    command.pitch_max_rad = config_.camera_pitch_max_rad;
    command.target_elevation_rad = elevation_rad;
    command.target_bearing_rad = bearing_rad;
    command.yaw_rad = bearing_rad;
    command.yaw_valid = true;
    command.range_xy_m = range_xy;
    command.delta_z_m = dz;
    command.pitch_sign = config_.camera_pitch_sign;
    command.pitch_offset_rad = config_.camera_pitch_offset_rad;
    command.pitch_valid = true;
    command.pitch_clamped = std::abs(pitch_rad - unclamped_pitch_rad) > 1e-9;
    return command;
}

std::optional<CameraPointingCommand> ObjectBehaviorMissionController::neutral_camera_pointing_command(
    TimePoint timestamp,
    const std::string& mode) const {
    if (config_.vertical_stare_mode == ObjectBehaviorVerticalStareMode::None || mode == "none" || mode == "disabled") {
        return std::nullopt;
    }

    CameraPointingCommand command;
    command.timestamp = timestamp;
    command.cameras = config_.camera_pointing_cameras;
    if (command.cameras.empty()) {
        command.cameras.push_back("front_center");
    }
    command.mode = mode;
    command.pitch_rad = 0.0;
    command.pitch_unclamped_rad = 0.0;
    command.pitch_min_rad = config_.camera_pitch_min_rad;
    command.pitch_max_rad = config_.camera_pitch_max_rad;
    command.target_elevation_rad = 0.0;
    command.target_bearing_rad = 0.0;
    command.yaw_rad = 0.0;
    command.yaw_valid = true;
    command.range_xy_m = 0.0;
    command.delta_z_m = 0.0;
    command.pitch_sign = config_.camera_pitch_sign;
    command.pitch_offset_rad = config_.camera_pitch_offset_rad;
    command.pitch_valid = true;
    command.pitch_clamped = false;
    return command;
}

ControllerEvent ObjectBehaviorMissionController::camera_pointing_intent_event(
    const CameraPointingCommand& command) const {
    return {ControllerEventKind::CameraPointingIntent,
        ",\"camera_pointing_mode\":" + q(command.mode) +
        ",\"vertical_stare_mode\":" + q(config_.vertical_stare_mode == ObjectBehaviorVerticalStareMode::Gimbal ? "gimbal" : "debug_only") +
        ",\"cameras\":" + json_string_array(command.cameras) +
        ",\"pitch_valid\":" + std::string(command.pitch_valid ? "true" : "false") +
        ",\"pitch_rad\":" + std::to_string(command.pitch_rad) +
        ",\"pitch_deg\":" + std::to_string(rad_to_deg(command.pitch_rad)) +
        ",\"pitch_unclamped_rad\":" + std::to_string(command.pitch_unclamped_rad) +
        ",\"pitch_unclamped_deg\":" + std::to_string(rad_to_deg(command.pitch_unclamped_rad)) +
        ",\"pitch_min_rad\":" + std::to_string(command.pitch_min_rad) +
        ",\"pitch_max_rad\":" + std::to_string(command.pitch_max_rad) +
        ",\"pitch_sign\":" + std::to_string(command.pitch_sign) +
        ",\"pitch_offset_rad\":" + std::to_string(command.pitch_offset_rad) +
        ",\"pitch_clamped\":" + std::string(command.pitch_clamped ? "true" : "false") +
        ",\"yaw_valid\":" + std::string(command.yaw_valid ? "true" : "false") +
        ",\"yaw_rad\":" + std::to_string(command.yaw_rad) +
        ",\"yaw_deg\":" + std::to_string(rad_to_deg(command.yaw_rad)) +
        ",\"target_elevation_rad\":" + std::to_string(command.target_elevation_rad) +
        ",\"target_elevation_deg\":" + std::to_string(rad_to_deg(command.target_elevation_rad)) +
        ",\"target_bearing_rad\":" + std::to_string(command.target_bearing_rad) +
        ",\"target_bearing_deg\":" + std::to_string(rad_to_deg(command.target_bearing_rad)) +
        ",\"range_xy_m\":" + std::to_string(command.range_xy_m) +
        ",\"delta_z_m\":" + std::to_string(command.delta_z_m) +
        ",\"agent_id\":" + q(command.agent_id) +
        ",\"source_track_id\":" + q(command.source_track_id) +
        ",\"identity_id\":" + q(command.identity_id) +
        behavior_display_fields("arriving")};
}

void ObjectBehaviorMissionController::emit_camera_pointing(
    MissionTickOutput& output,
    const CameraPointingCommand& command) const {
    output.camera_pointing = command;
    output.events.push_back(camera_pointing_intent_event(command));
}

}  // namespace dedalus
