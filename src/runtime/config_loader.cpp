#include "dedalus/runtime/config_loader.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace dedalus {
namespace {

constexpr double kPi = 3.14159265358979323846;

std::string trim(std::string value) {
    const auto not_space = [](unsigned char ch) { return !std::isspace(ch); };
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
    value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
    return value;
}

std::string strip_quotes(std::string value) {
    if (value.size() >= 2U) {
        const char first = value.front();
        const char last = value.back();
        if ((first == '"' && last == '"') || (first == '\'' && last == '\'')) {
            return value.substr(1U, value.size() - 2U);
        }
    }
    return value;
}

bool parse_bool(const std::string& value) {
    if (value == "true" || value == "1" || value == "yes" || value == "on") return true;
    if (value == "false" || value == "0" || value == "no" || value == "off") return false;
    throw std::invalid_argument("invalid boolean config value: " + value);
}

std::uint8_t parse_uint8(const std::string& value, const char* context) {
    const int raw = std::stoi(value);
    if (raw < 0 || raw > 255)
        throw std::out_of_range(std::string("value out of uint8 range for ") + context + ": " + value);
    return static_cast<std::uint8_t>(raw);
}

std::uint32_t parse_uint32(const std::string& value, const char* context) {
    const unsigned long raw = std::stoul(value);
    if (raw > std::numeric_limits<std::uint32_t>::max())
        throw std::out_of_range(std::string("value out of uint32 range for ") + context + ": " + value);
    return static_cast<std::uint32_t>(raw);
}

double deg_to_rad(double degrees) {
    return degrees * kPi / 180.0;
}

Vec3 parse_vec3(const std::string& value) {
    std::string normalized;
    normalized.reserve(value.size());
    for (const char ch : value) normalized.push_back((ch == '[' || ch == ']' || ch == ',') ? ' ' : ch);
    std::istringstream input{normalized};
    Vec3 result;
    if (!(input >> result.x >> result.y >> result.z)) throw std::invalid_argument("invalid vec3 config value: " + value);
    std::string extra;
    if (input >> extra) throw std::invalid_argument("invalid vec3 config value with extra data: " + value);
    return result;
}

std::pair<std::size_t, std::string> parse_indexed_field(const std::string& key, const std::string& prefix, const std::string& description) {
    const auto remainder = key.substr(prefix.size());
    const auto dot_pos = remainder.find('.');
    if (dot_pos == std::string::npos || dot_pos == 0U || dot_pos + 1U >= remainder.size()) throw std::invalid_argument("invalid " + description + " key: " + key);
    const auto index_text = remainder.substr(0U, dot_pos);
    const auto field = remainder.substr(dot_pos + 1U);
    std::size_t parsed_chars = 0U;
    const auto index = std::stoul(index_text, &parsed_chars);
    if (parsed_chars != index_text.size()) throw std::invalid_argument("invalid " + description + " index: " + key);
    if (index >= 1024U) throw std::invalid_argument("unreasonable " + description + " index: " + key);
    return {index, field};
}

bool parse_obstacle_sensing_camera_key(MissionOptions& opts, const std::string& key, const std::string& value) {
    const std::string prefix = "obstacle_sensing.cameras.";
    if (key.rfind(prefix, 0U) != 0U) return false;
    const auto [index, field] = parse_indexed_field(key, prefix, "obstacle sensing camera");
    if (opts.obstacle_sensing_cameras.size() <= index) opts.obstacle_sensing_cameras.resize(index + 1U);
    auto& camera = opts.obstacle_sensing_cameras[index];
    if (field == "name" || field == "camera_name") {
        camera.camera_name = value;
        camera.camera_id = CameraId{value};
    } else if (field == "camera_id") {
        camera.camera_id = CameraId{value};
        if (camera.camera_name.empty() || camera.camera_name == "front_center") camera.camera_name = value;
    } else if (field == "role") {
        camera.role = value;
    } else if (field == "horizontal_fov_deg") {
        camera.horizontal_fov_rad = deg_to_rad(std::stod(value));
    } else if (field == "vertical_fov_deg") {
        camera.vertical_fov_rad = deg_to_rad(std::stod(value));
    } else if (field == "horizontal_fov_rad") {
        camera.horizontal_fov_rad = std::stod(value);
    } else if (field == "vertical_fov_rad") {
        camera.vertical_fov_rad = std::stod(value);
    } else if (field == "near_range_m") {
        camera.near_range_m = std::stod(value);
    } else if (field == "far_range_m") {
        camera.far_range_m = std::stod(value);
    } else if (field == "min_reliable_range_m") {
        camera.min_reliable_range_m = std::stod(value);
    } else if (field == "max_reliable_range_m") {
        camera.max_reliable_range_m = std::stod(value);
    } else if (field == "body_T_camera_xyz_m") {
        camera.body_T_camera_xyz_m = parse_vec3(value);
    } else if (field == "body_T_camera_rpy_deg") {
        const auto degrees = parse_vec3(value);
        camera.body_T_camera_rpy_rad = Vec3{deg_to_rad(degrees.x), deg_to_rad(degrees.y), deg_to_rad(degrees.z)};
    } else if (field == "body_T_camera_rpy_rad") {
        camera.body_T_camera_rpy_rad = parse_vec3(value);
    } else if (field == "pointing_source") {
        camera.pointing_source = value;
    } else {
        throw std::invalid_argument("unknown obstacle sensing camera field: " + key);
    }
    return true;
}

// clang-format off
void apply_mission_option(MissionOptions& opts, const std::string& key, const std::string& value) {
    if (parse_obstacle_sensing_camera_key(opts, key, value)) return;
    if (key == "behavior_spec_path")                           { opts.behavior_spec_path = value; return; }
    if (key == "object_behavior_follow_observation_geometry_enabled") { opts.follow_observation_geometry_enabled = parse_bool(value); return; }
    if (key == "object_behavior_zero_target_velocity")         { opts.zero_target_velocity = parse_bool(value); return; }
    if (key == "object_behavior_follow_min_standoff_m")        { opts.follow_min_standoff_m = std::stod(value); return; }
    if (key == "object_behavior_follow_max_elevation_angle_deg") { opts.follow_max_elevation_angle_deg = std::stod(value); return; }
    if (key == "object_behavior_follow_arrival_slow_radius_m") { opts.follow_arrival_slow_radius_m = std::stod(value); return; }
    if (key == "object_behavior_follow_arrival_hold_radius_m") { opts.follow_arrival_hold_radius_m = std::stod(value); return; }
    if (key == "object_behavior_follow_arrival_kp")            { opts.follow_arrival_kp = std::stod(value); return; }
    if (key == "object_behavior_completion_after_s")           { opts.completion_after_s = std::stod(value); return; }
    if (key == "object_behavior_hold_velocity_mps")            { opts.hold_velocity_mps = std::stod(value); return; }
    if (key == "object_behavior_yaw_mode")                     { opts.yaw_mode = value; return; }
    if (key == "object_behavior_yaw_min_speed_mps")            { opts.yaw_min_speed_mps = std::stod(value); return; }
    if (key == "object_behavior_yaw_hold_last_when_unstable")  { opts.yaw_hold_last_when_unstable = parse_bool(value); return; }
    if (key == "object_behavior_yaw_offset_rad")               { opts.object_behavior_yaw_offset_rad = std::stod(value); return; }
    if (key == "object_behavior_vertical_stare_mode")          { opts.vertical_stare_mode = value; return; }
    if (key == "object_behavior_vertical_stare_warn_if_unavailable") { opts.vertical_stare_warn_if_unavailable = parse_bool(value); return; }
    if (key == "object_behavior_debug_every_n_ticks")          { opts.debug_every_n_ticks = std::stoi(value); return; }
    if (key == "object_behavior_debug_level")                  { opts.debug_level = std::stoi(value); return; }
    if (key == "object_behavior_altitude_policy")              { opts.altitude_policy = value; return; }
    if (key == "object_behavior_min_height_m")                 { opts.behavior_min_height_m = std::stod(value); return; }
    if (key == "object_behavior_camera_pointing_cameras")      { opts.camera_pointing_cameras = value; return; }
    if (key == "object_behavior_camera_pitch_min_deg")         { opts.camera_pitch_min_deg = std::stod(value); return; }
    if (key == "object_behavior_camera_pitch_max_deg")         { opts.camera_pitch_max_deg = std::stod(value); return; }
    if (key == "object_behavior_camera_pitch_sign")            { opts.camera_pitch_sign = std::stod(value); return; }
    if (key == "object_behavior_camera_pitch_offset_deg")      { opts.camera_pitch_offset_deg = std::stod(value); return; }
    if (key == "object_behavior_camera_pointing_prepare_mode") { opts.camera_pointing_prepare_mode = value; return; }
    if (key == "object_behavior_camera_pointing_takeoff_mode") { opts.camera_pointing_takeoff_mode = value; return; }
    if (key == "object_behavior_camera_pointing_go_home_mode") { opts.camera_pointing_go_home_mode = value; return; }
    if (key == "object_behavior_camera_pointing_land_mode")    { opts.camera_pointing_land_mode = value; return; }
    if (key == "object_behavior_camera_pointing_complete_mode"){ opts.camera_pointing_complete_mode = value; return; }
    if (key == "object_behavior_camera_pointing_sink")         { opts.camera_pointing_sink = value; return; }
    if (key == "object_behavior_camera_pointing_mavlink_endpoints") { opts.camera_pointing_mavlink_endpoints = value; return; }
    if (key == "object_behavior_camera_pointing_mavlink_source_system_id")    { opts.camera_pointing_mavlink_source_system_id    = parse_uint8(value, key.c_str()); return; }
    if (key == "object_behavior_camera_pointing_mavlink_source_component_id") { opts.camera_pointing_mavlink_source_component_id = parse_uint8(value, key.c_str()); return; }
    if (key == "object_behavior_camera_pointing_mavlink_target_system_id")    { opts.camera_pointing_mavlink_target_system_id    = parse_uint8(value, key.c_str()); return; }
    if (key == "object_behavior_camera_pointing_mavlink_target_component_id") { opts.camera_pointing_mavlink_target_component_id = parse_uint8(value, key.c_str()); return; }
    if (key == "object_behavior_camera_pointing_mavlink_gimbal_device_id")    { opts.camera_pointing_mavlink_gimbal_device_id    = parse_uint8(value, key.c_str()); return; }
    if (key == "object_behavior_camera_pointing_mavlink_flags")               { opts.camera_pointing_mavlink_flags               = parse_uint32(value, key.c_str()); return; }
    if (key == "object_behavior_camera_pointing_deadband_rad") { opts.camera_pointing_deadband_rad = std::stod(value); return; }
    if (key == "object_behavior_camera_pointing_resend_s")     { opts.camera_pointing_resend_s = std::stod(value); return; }
    if (key == "flight_safe_height_m")             { opts.safe_height_m = std::stod(value); return; }
    if (key == "flight_takeoff_height_m")          { opts.takeoff_height_m = std::stod(value); return; }
    if (key == "flight_takeoff_velocity_mps")      { opts.takeoff_velocity_mps = std::stod(value); return; }
    if (key == "flight_go_home_velocity_mps")      { opts.go_home_velocity_mps = std::stod(value); return; }
    if (key == "flight_land_velocity_mps")         { opts.land_velocity_mps = std::stod(value); return; }
    if (key == "flight_arm_retry_interval_s")      { opts.arm_retry_interval_s = std::stod(value); return; }
    if (key == "flight_arm_timeout_s")             { opts.arm_timeout_s = std::stod(value); return; }
    if (key == "flight_arm_dispatch_fallback_s")   { opts.arm_dispatch_fallback_s = std::stod(value); return; }
    if (key == "flight_takeoff_retry_interval_s")  { opts.takeoff_retry_interval_s = std::stod(value); return; }
    if (key == "flight_land_retry_interval_s")     { opts.land_retry_interval_s = std::stod(value); return; }
    if (key == "flight_land_timeout_s")            { opts.land_timeout_s = std::stod(value); return; }
    if (key == "flight_disarm_retry_interval_s")   { opts.disarm_retry_interval_s = std::stod(value); return; }
    if (key == "flight_disarm_timeout_s")          { opts.disarm_timeout_s = std::stod(value); return; }
    if (key == "flight_home_policy")               { opts.home_policy = value; return; }
    if (key == "flight_yaw_offset_rad")            { opts.yaw_offset_rad = std::stod(value); return; }
    if (key == "flight_max_velocity_mps")          { opts.max_velocity_mps = std::stod(value); return; }
    if (key == "flight_prepare_session_command")   { opts.prepare_session_command = value; return; }
    if (key == "flight_trajectory_path")           { opts.trajectory_path = value; return; }
    if (key == "flight_control_mode")              { opts.flight_control_mode = value; return; }
    if (key == "flight_px4_command_bridge")        { opts.px4_command_bridge = value; return; }
    if (key == "flight_velocity_command_bridge")   { opts.velocity_command_bridge = value; return; }
    if (key == "flight_mavlink_command_endpoints") { opts.mavlink_command_endpoints = value; return; }
    if (key == "flight_px4_tmux_target")           { opts.px4_tmux_target = value; return; }
    if (key == "flight_use_px4_shell_lifecycle")   { opts.use_px4_shell_lifecycle = parse_bool(value); return; }
    if (key == "flight_mavlink_target_system_id")    { opts.mavlink_target_system_id    = parse_uint8(value, key.c_str()); return; }
    if (key == "flight_mavlink_target_component_id") { opts.mavlink_target_component_id = parse_uint8(value, key.c_str()); return; }
    if (key == "flight_mavlink_source_system_id")    { opts.mavlink_source_system_id    = parse_uint8(value, key.c_str()); return; }
    if (key == "flight_mavlink_source_component_id") { opts.mavlink_source_component_id = parse_uint8(value, key.c_str()); return; }
    if (key == "flight_mavlink_set_offboard_on_velocity") { opts.set_offboard_on_velocity = parse_bool(value); return; }
    std::cerr << "WARNING: unrecognized mission_options key: " << key << " (check for typos; value will be ignored)\n";
}
// clang-format on

std::string mission_option_key_from(const std::string& key) {
    constexpr auto prefix = "mission_options.";
    return key.substr(std::string{prefix}.size());
}

bool parse_airsim_object_binding_key(CoreStackProviderConfig& config, const std::string& key, const std::string& value) {
    const std::string prefix = "ghost_targets_airsim.objects.";
    if (key.rfind(prefix, 0U) != 0U) return false;
    const auto [index, field] = parse_indexed_field(key, prefix, "AirSim ghost object binding");
    if (config.ghost_targets_airsim_objects.size() <= index) config.ghost_targets_airsim_objects.resize(index + 1U);
    auto& binding = config.ghost_targets_airsim_objects[index];
    if (field == "source_track_id") binding.source_track_id = TrackId{value};
    else if (field == "airsim_object_name") binding.airsim_object_name = value;
    else if (field == "class") binding.class_label = class_label_from_string(value);
    else if (field == "confidence") binding.confidence = std::stod(value);
    else if (field == "size_m") binding.size_m = parse_vec3(value);
    else throw std::invalid_argument("unknown AirSim ghost object binding field: " + key);
    return true;
}

bool parse_airsim_pattern_binding_key(CoreStackProviderConfig& config, const std::string& key, const std::string& value) {
    const std::string prefix = "ghost_targets_airsim.patterns.";
    if (key.rfind(prefix, 0U) != 0U) return false;
    const auto [index, field] = parse_indexed_field(key, prefix, "AirSim ghost object pattern binding");
    if (config.ghost_targets_airsim_patterns.size() <= index) config.ghost_targets_airsim_patterns.resize(index + 1U);
    auto& binding = config.ghost_targets_airsim_patterns[index];
    if (field == "source_track_prefix") binding.source_track_prefix = value;
    else if (field == "airsim_object_pattern") binding.airsim_object_pattern = value;
    else if (field == "class") binding.class_label = class_label_from_string(value);
    else if (field == "confidence") binding.confidence = std::stod(value);
    else if (field == "size_m") binding.size_m = parse_vec3(value);
    else if (field == "max_matches") binding.max_matches = std::stoi(value);
    else throw std::invalid_argument("unknown AirSim ghost object pattern binding field: " + key);
    return true;
}

void apply_config_value(CoreStackProviderConfig& config, const std::string& key, const std::string& value) {
    if (parse_airsim_object_binding_key(config, key, value)) return;
    if (parse_airsim_pattern_binding_key(config, key, value)) return;
    if (key == "frame_source") config.frame_source = value;
    else if (key == "ego_provider") config.ego_provider = value;
    else if (key == "detector") config.detector = value;
    else if (key == "camera_stabilizer") config.camera_stabilizer = value;
    else if (key == "tracker") config.tracker = value;
    else if (key == "identity_resolver") config.identity_resolver = value;
    else if (key == "projector") config.projector = value;
    else if (key == "ghost_targets_enabled") config.ghost_targets_enabled = parse_bool(value);
    else if (key == "ghost_targets_source") config.ghost_targets_source = value;
    else if (key == "ghost_targets_scenario") config.ghost_targets_scenario = value;
    else if (key == "ghost_targets_scenario_path") config.ghost_targets_scenario_path = value;
    else if (key == "ghost_targets_airsim_scene_inventory_path") config.ghost_targets_airsim_scene_inventory_path = value;
    else if (key == "ghost_targets_airsim_object_pose_stream_rate_hz") config.ghost_targets_airsim_object_pose_stream_rate_hz = std::stod(value);
    else if (key == "world_model") config.world_model = value;
    else if (key == "occupancy_source") config.occupancy_source = value;
    else if (key == "frame_annotator") config.frame_annotator = value;
    else if (key == "annotation_output_path") config.annotation_output_path = value;
    else if (key == "annotation_output_fps") config.annotation_output_fps = std::stod(value);
    else if (key == "pipeline_timing_enabled") config.pipeline_timing_enabled = parse_bool(value);
    else if (key == "pipeline_timing_output_path") config.pipeline_timing_output_path = value;
    else if (key == "recorded_manifest_path") config.recorded_manifest_path = value;
    else if (key == "source_host") config.source_host = value;
    else if (key == "source_rpc_port") config.source_rpc_port = std::stoi(value);
    else if (key == "vehicle_name") config.vehicle_name = value;
    else if (key == "vehicle_camera_name") config.vehicle_camera_name = value;
    else if (key == "bridge_transport") config.bridge_transport = value;
    else if (key == "bridge_command") config.bridge_command = value;
    else if (key == "bridge_mode") config.bridge_mode = value;
    else if (key == "ego_bridge_command") config.ego_bridge_command = value;
    else if (key == "fallback_map_frame_id") config.fallback_map_frame_id = MapFrameId{value};
    else if (key == "mission_controller") config.mission_controller = value;
    else if (key == "mission_tick_hz") config.mission_tick_hz = std::stod(value);
    else if (key == "flight_command_sink") config.flight_command_sink = value;
    else if (key.rfind("mission_options.", 0U) == 0U) {
        const auto option_key = mission_option_key_from(key);
        if (option_key.empty()) throw std::invalid_argument("empty mission_options key");
        apply_mission_option(config.mission_options, option_key, value);
    } else {
        throw std::invalid_argument("unknown core-stack config key: " + key);
    }
}

void validate_airsim_object_bindings(const CoreStackProviderConfig& config) {
    if (config.ghost_targets_source != "airsim_objects") return;
    if (config.ghost_targets_airsim_objects.empty() && config.ghost_targets_airsim_patterns.empty()) throw std::invalid_argument("ghost_targets_source=airsim_objects requires exact object or pattern bindings");
    for (std::size_t index = 0; index < config.ghost_targets_airsim_objects.size(); ++index) {
        const auto& binding = config.ghost_targets_airsim_objects[index];
        const auto prefix = "ghost_targets_airsim.objects." + std::to_string(index);
        if (binding.source_track_id.value.empty()) throw std::invalid_argument(prefix + ".source_track_id is required");
        if (binding.airsim_object_name.empty()) throw std::invalid_argument(prefix + ".airsim_object_name is required");
        if (binding.class_label == ClassLabel::Unknown) throw std::invalid_argument(prefix + ".class is required");
        if (binding.confidence < 0.0 || binding.confidence > 1.0) throw std::invalid_argument(prefix + ".confidence must be in [0, 1]");
        if (binding.size_m.x <= 0.0 || binding.size_m.y <= 0.0 || binding.size_m.z <= 0.0) throw std::invalid_argument(prefix + ".size_m components must be positive");
    }
    for (std::size_t index = 0; index < config.ghost_targets_airsim_patterns.size(); ++index) {
        const auto& binding = config.ghost_targets_airsim_patterns[index];
        const auto prefix = "ghost_targets_airsim.patterns." + std::to_string(index);
        if (binding.source_track_prefix.empty()) throw std::invalid_argument(prefix + ".source_track_prefix is required");
        if (binding.airsim_object_pattern.empty()) throw std::invalid_argument(prefix + ".airsim_object_pattern is required");
        if (binding.class_label == ClassLabel::Unknown) throw std::invalid_argument(prefix + ".class is required");
        if (binding.confidence < 0.0 || binding.confidence > 1.0) throw std::invalid_argument(prefix + ".confidence must be in [0, 1]");
        if (binding.max_matches < 0) throw std::invalid_argument(prefix + ".max_matches must be >= 0");
        if (binding.size_m.x <= 0.0 || binding.size_m.y <= 0.0 || binding.size_m.z <= 0.0) throw std::invalid_argument(prefix + ".size_m components must be positive");
    }
}

void validate_obstacle_sensing_cameras(const MissionOptions& options) {
    for (std::size_t index = 0; index < options.obstacle_sensing_cameras.size(); ++index) {
        const auto& camera = options.obstacle_sensing_cameras[index];
        const auto prefix = "mission_options.obstacle_sensing.cameras." + std::to_string(index);
        if (camera.camera_name.empty() || camera.camera_id.value.empty()) throw std::invalid_argument(prefix + ".name is required");
        if (camera.role.empty()) throw std::invalid_argument(prefix + ".role is required");
        if (camera.horizontal_fov_rad <= 0.0 || camera.horizontal_fov_rad >= kPi) throw std::invalid_argument(prefix + ".horizontal_fov must be in (0, 180deg)");
        if (camera.vertical_fov_rad <= 0.0 || camera.vertical_fov_rad >= kPi) throw std::invalid_argument(prefix + ".vertical_fov must be in (0, 180deg)");
        if (camera.near_range_m < 0.0) throw std::invalid_argument(prefix + ".near_range_m must be >= 0");
        if (camera.far_range_m <= camera.near_range_m) throw std::invalid_argument(prefix + ".far_range_m must be > near_range_m");
        if (camera.min_reliable_range_m < 0.0) throw std::invalid_argument(prefix + ".min_reliable_range_m must be >= 0");
        if (camera.max_reliable_range_m < camera.min_reliable_range_m) throw std::invalid_argument(prefix + ".max_reliable_range_m must be >= min_reliable_range_m");
    }
}

void validate_config(const CoreStackProviderConfig& config) {
    if (config.ghost_targets_source != "trajectory_scenario" && config.ghost_targets_source != "airsim_objects") throw std::invalid_argument("unknown ghost_targets_source: " + config.ghost_targets_source);
    if (config.ghost_targets_source != "airsim_objects" && (!config.ghost_targets_airsim_objects.empty() || !config.ghost_targets_airsim_patterns.empty())) throw std::invalid_argument("ghost_targets_airsim bindings require ghost_targets_source=airsim_objects");
    if (config.occupancy_source != "synthetic_fixture" && config.occupancy_source != "airsim_ground_truth") throw std::invalid_argument("unknown occupancy_source: " + config.occupancy_source);
    if (config.occupancy_source == "airsim_ground_truth" && config.ghost_targets_source != "airsim_objects") throw std::invalid_argument("occupancy_source=airsim_ground_truth requires ghost_targets_source=airsim_objects");
    validate_airsim_object_bindings(config);
    validate_obstacle_sensing_cameras(config.mission_options);
}

}  // namespace

void validate_provider_names(const CoreStackProviderConfig& config, const ProviderRegistry& registry) {
    const auto check = [](const std::string& field, const std::string& value, const std::vector<std::string>& valid) {
        if (std::find(valid.begin(), valid.end(), value) == valid.end()) throw std::invalid_argument("unknown " + field + ": " + value);
    };
    check("frame_source",        config.frame_source,        registry.frame_sources());
    check("ego_provider",        config.ego_provider,        registry.ego_providers());
    check("detector",            config.detector,            registry.detectors());
    check("camera_stabilizer",   config.camera_stabilizer,   registry.camera_stabilizers());
    check("tracker",             config.tracker,             registry.trackers());
    check("identity_resolver",   config.identity_resolver,   registry.identity_resolvers());
    check("projector",           config.projector,           registry.projectors());
    check("world_model",         config.world_model,         registry.world_models());
    check("frame_annotator",     config.frame_annotator,     registry.frame_annotators());
    check("mission_controller",  config.mission_controller,  registry.mission_controllers());
    check("flight_command_sink", config.flight_command_sink, registry.flight_command_sinks());
}

CoreStackProviderConfig load_core_stack_config(const std::string& path) {
    std::ifstream input{path};
    if (!input) throw std::runtime_error("failed to open core-stack config: " + path);
    CoreStackProviderConfig config;
    std::string line;
    int line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        const auto comment_pos = line.find('#');
        if (comment_pos != std::string::npos) line = line.substr(0U, comment_pos);
        line = trim(line);
        if (line.empty()) continue;
        const auto separator_pos = line.find(':');
        if (separator_pos == std::string::npos) throw std::runtime_error("invalid core-stack config line " + std::to_string(line_number) + ": missing ':'");
        const std::string key = trim(line.substr(0U, separator_pos));
        const std::string value = strip_quotes(trim(line.substr(separator_pos + 1U)));
        if (key.empty() || value.empty()) throw std::runtime_error("invalid core-stack config line " + std::to_string(line_number) + ": empty key or value");
        try { apply_config_value(config, key, value); }
        catch (const std::exception& ex) { throw std::runtime_error("invalid core-stack config line " + std::to_string(line_number) + ": " + ex.what()); }
    }
    validate_config(config);
    return config;
}

}  // namespace dedalus
