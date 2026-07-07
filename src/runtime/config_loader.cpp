#include "dedalus/runtime/config_loader.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// Platform access to the process environment array.
#ifdef __APPLE__
#  include <crt_externs.h>   // _NSGetEnviron()
#else
   extern char** environ;    // POSIX — declared at file scope, not in any namespace
#endif

#include "dedalus/sensing/airsim_depth_evidence_provider.hpp"
#include "dedalus/sensing/airsim_emulation_depth_obstacle_detector.hpp"
#ifdef DEDALUS_ONNX_DEPTH_ENABLED
#include "dedalus/sensing/metric_scale_estimate.hpp"
#include "dedalus/sensing/onnx_depth_engine.hpp"
#include "dedalus/sensing/visual_depth_obstacle_detector.hpp"
#endif

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

// Return env var value if set and non-empty, otherwise return the config value.
std::string env_str_or(const std::string& config_value, const char* env_var) {
    const char* env = std::getenv(env_var);
    if (env != nullptr && *env != '\0') return std::string{env};
    return config_value;
}

// Warn about any DEDALUS_ env var not in the authoritative list.
// Catches typos (e.g. DEDALUS_PIPELINE_EGO instead of DEDALUS_EGO_PROVIDER) that
// would otherwise be silently ignored. Source of truth: grep getenv + env_str_or
// across the entire src/ tree.
void warn_unknown_dedalus_vars() {
    // Every DEDALUS_ env var read anywhere in the runtime binary.
    // Keep this list in sync with grepping `getenv("DEDALUS_` and `env_str_or(` in src/.
    static const char* const kKnown[] = {
        // config_loader.cpp — provider slot overrides
        "DEDALUS_DEPTH",
        "DEDALUS_DEPTH_EVAL",
        "DEDALUS_EGO_PROVIDER",
        "DEDALUS_EGO_PROVIDER_EVAL",
        "DEDALUS_DETECTOR_EVAL",
        "DEDALUS_CAMERA_STABILIZER_EVAL",
        "DEDALUS_TRACKER_EVAL",
        "DEDALUS_IDENTITY_RESOLVER_EVAL",
        "DEDALUS_PROJECTOR_EVAL",
        // apps/dedalus_mission_loop.cpp + apps/dedalus_viewer.cpp
        "DEDALUS_SITE_ID",
        "DEDALUS_L2_NO_PERSIST",
        // debug instrumentation — set to 1 to trace ego position at every layer
        "DEDALUS_DEBUG_EGO",
        // src/sensing/onnx_depth_engine.cpp
        "DEDALUS_DEPTH_DEBUG_DIR",
        // src/runtime/provider_registry.cpp
        "DEDALUS_AIRSIM_SCENE_INVENTORY",
        "DEDALUS_AIRSIM_GT_NEARBY_RADIUS_M",
        "DEDALUS_AIRSIM_GT_MAX_OBJECTS_PER_FRAME",
        "DEDALUS_AIRSIM_GT_STATIC_REFRESH_EVERY_N_FRAMES",
        // src/avoidance/mission_traversability_map_artifact_writer_env.cpp
        "DEDALUS_MISSION_TRAVERSABILITY_MAP_ARTIFACT",
        "DEDALUS_MISSION_TRAVERSABILITY_MAP_PATH",
        "DEDALUS_MISSION_TRAVERSABILITY_MAP_SITE_ID",
        "DEDALUS_MISSION_TRAVERSABILITY_MAP_SITE_FRAME_ID",
        "DEDALUS_MISSION_TRAVERSABILITY_MAP_MISSION_ID",
        "DEDALUS_MISSION_TRAVERSABILITY_MAP_WRITE_EVERY_UPDATES",
        "DEDALUS_MISSION_TRAVERSABILITY_MAP_MAX_CELLS",
        // src/avoidance/mission_obstacle_map_artifact_writer_env.cpp
        "DEDALUS_MISSION_OBSTACLE_MAP_ARTIFACT",
        "DEDALUS_MISSION_OBSTACLE_MAP_PATH",
        "DEDALUS_MISSION_OBSTACLE_MAP_SITE_ID",
        "DEDALUS_MISSION_OBSTACLE_MAP_SITE_FRAME_ID",
        "DEDALUS_MISSION_OBSTACLE_MAP_MISSION_ID",
        "DEDALUS_MISSION_OBSTACLE_MAP_WRITE_EVERY_UPDATES",
        // src/avoidance/mission_obstacle_map_delta_writer.cpp
        "DEDALUS_MISSION_OBSTACLE_MAP_DELTAS",
        "DEDALUS_MISSION_OBSTACLE_MAP_DELTAS_PATH",
        "DEDALUS_MISSION_OBSTACLE_MAP_DELTAS_WRITE_EVERY_UPDATES",
        nullptr
    };
#ifdef __APPLE__
    char** env_list = *_NSGetEnviron();  // macOS: environ not in any std header
#else
    char** env_list = environ;           // Linux/POSIX
#endif
    if (!env_list) return;
    for (char** e = env_list; *e; ++e) {
        const std::string entry{*e};
        const auto eq_pos = entry.find('=');
        if (eq_pos == std::string::npos) continue;
        const std::string var = entry.substr(0U, eq_pos);
        if (var.size() < 8U || var.substr(0U, 8U) != "DEDALUS_") continue;
        bool known = false;
        for (const char* const* k = kKnown; *k; ++k) {
            if (var == *k) { known = true; break; }
        }
        if (!known) {
            std::cerr << "WARN: unknown DEDALUS_ env var '" << var
                      << "' — typo? It will be silently ignored.\n"
                      << "      Known vars: see LLM.md §Runtime Env Var Reference\n";
        }
    }
}

// Build a depth ObstacleEvidenceProvider by string name.
// Returns nullptr if name is empty (slot inactive).
std::unique_ptr<ObstacleEvidenceProvider> make_depth_provider(
    const std::string& name,
    const AirSimDepthObstacleDetectorConfig& airsim_gt_config,
    const VisualONNXDepthConfig& visual_onnx_config) {
    if (name.empty()) return nullptr;
    if (name == "airsim_gt_detector")
        return std::make_unique<AirSimDepthEvidenceProvider>(airsim_gt_config);
    if (name == "airsim_gt_vd")
        return std::make_unique<AirSimEmulationDepthObstacleDetector>();
    if (name == "visual_onnx") {
#ifdef DEDALUS_ONNX_DEPTH_ENABLED
        if (visual_onnx_config.model_path.empty())
            throw std::invalid_argument("visual_onnx: visual_onnx.model_path is required");
        ONNXDepthEngineConfig engine_cfg;
        engine_cfg.model_path           = visual_onnx_config.model_path;
        engine_cfg.input_name           = visual_onnx_config.input_name;
        engine_cfg.output_name          = visual_onnx_config.output_name;
        engine_cfg.model_input_width    = visual_onnx_config.model_input_width;
        engine_cfg.model_input_height   = visual_onnx_config.model_input_height;
        engine_cfg.use_cuda             = visual_onnx_config.use_cuda;
        engine_cfg.cuda_device_id       = visual_onnx_config.cuda_device_id;
        engine_cfg.cuda_arena_limit_bytes = visual_onnx_config.cuda_arena_limit_bytes;
        engine_cfg.use_coreml           = visual_onnx_config.use_coreml;
        engine_cfg.metric_depth         = visual_onnx_config.metric_depth;
        VisualDepthObstacleDetectorConfig detector_cfg;
        detector_cfg.pixel_stride           = visual_onnx_config.pixel_stride;
        detector_cfg.min_depth_m            = visual_onnx_config.min_depth_m;
        detector_cfg.max_depth_m            = visual_onnx_config.max_depth_m;
        detector_cfg.voxel_size_m           = visual_onnx_config.voxel_size_m;
        detector_cfg.confidence             = visual_onnx_config.confidence;
        detector_cfg.max_evidence           = visual_onnx_config.max_evidence;
        detector_cfg.detect_surface_patches = visual_onnx_config.detect_surface_patches;
        detector_cfg.detect_thin_structures = visual_onnx_config.detect_thin_structures;
        detector_cfg.debug_depth_mp4        = visual_onnx_config.debug_depth_mp4;
        MetricScaleEstimate scale;
        scale.scale      = visual_onnx_config.scale;
        scale.confidence = 1.0F;  // fixed at startup (VD7 will couple to VIO)
        return std::make_unique<VisualDepthObstacleDetector>(
            std::make_unique<ONNXDepthEngine>(std::move(engine_cfg)),
            scale, detector_cfg);
#else
        (void)visual_onnx_config;
        throw std::invalid_argument(
            "visual_onnx: build requires -DDEDALUS_ENABLE_ONNX_DEPTH=ON");
#endif
    }
    throw std::invalid_argument("unknown depth provider: " + name);
}

// Populate depth_slot_a/b in runner from provider string names, applying env
// var overrides (DEDALUS_DEPTH, DEDALUS_DEPTH_EVAL).
void build_depth_slots(CoreStackConfig& config) {
    const std::string depth_name =
        env_str_or(config.providers.depth, "DEDALUS_DEPTH");
    const std::string depth_eval_name =
        env_str_or(config.providers.depth_eval, "DEDALUS_DEPTH_EVAL");

    config.runner.depth_slot_a =
        make_depth_provider(depth_name, config.runner.airsim_depth_obstacle_detector,
                            config.runner.visual_onnx_depth);
    config.runner.depth_slot_b =
        make_depth_provider(depth_eval_name, config.runner.airsim_depth_obstacle_detector,
                            config.runner.visual_onnx_depth);
}

// Apply env var overrides for primary and eval provider names.
void apply_eval_env_overrides(CoreStackProviderConfig& config) {
    config.ego_provider           = env_str_or(config.ego_provider,           "DEDALUS_EGO_PROVIDER");
    config.ego_provider_eval      = env_str_or(config.ego_provider_eval,      "DEDALUS_EGO_PROVIDER_EVAL");
    config.detector_eval          = env_str_or(config.detector_eval,          "DEDALUS_DETECTOR_EVAL");
    config.camera_stabilizer_eval = env_str_or(config.camera_stabilizer_eval, "DEDALUS_CAMERA_STABILIZER_EVAL");
    config.tracker_eval           = env_str_or(config.tracker_eval,           "DEDALUS_TRACKER_EVAL");
    config.identity_resolver_eval = env_str_or(config.identity_resolver_eval, "DEDALUS_IDENTITY_RESOLVER_EVAL");
    config.projector_eval         = env_str_or(config.projector_eval,         "DEDALUS_PROJECTOR_EVAL");
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


bool parse_visual_onnx_key(
    CoreStackRunnerConfig& config,
    const std::string& key,
    const std::string& value) {
    const std::string prefix = "visual_onnx.";
    if (key.rfind(prefix, 0U) != 0U) return false;
    const auto field = key.substr(prefix.size());
    auto& cfg = config.visual_onnx_depth;
    if (field == "model_path")                { cfg.model_path = value; }
    else if (field == "input_name")           { cfg.input_name = value; }
    else if (field == "output_name")          { cfg.output_name = value; }
    else if (field == "model_input_width")    { cfg.model_input_width  = std::stoi(value); }
    else if (field == "model_input_height")   { cfg.model_input_height = std::stoi(value); }
    else if (field == "scale")                { cfg.scale = std::stof(value); }
    else if (field == "use_cuda")             { cfg.use_cuda = parse_bool(value); }
    else if (field == "cuda_device_id")       { cfg.cuda_device_id = std::stoi(value); }
    else if (field == "cuda_arena_limit_bytes") {
        cfg.cuda_arena_limit_bytes = static_cast<std::size_t>(std::stoull(value));
    }
    else if (field == "use_coreml")           { cfg.use_coreml = parse_bool(value); }
    else if (field == "metric_depth")         { cfg.metric_depth = parse_bool(value); }
    else if (field == "pixel_stride")         { cfg.pixel_stride = static_cast<std::size_t>(std::stoul(value)); }
    else if (field == "min_depth_m")          { cfg.min_depth_m = std::stof(value); }
    else if (field == "max_depth_m")          { cfg.max_depth_m = std::stof(value); }
    else if (field == "voxel_size_m")         { cfg.voxel_size_m = std::stof(value); }
    else if (field == "confidence")           { cfg.confidence = std::stof(value); }
    else if (field == "max_evidence")         { cfg.max_evidence = static_cast<std::size_t>(std::stoul(value)); }
    else if (field == "detect_surface_patches") { cfg.detect_surface_patches = parse_bool(value); }
    else if (field == "detect_thin_structures") { cfg.detect_thin_structures = parse_bool(value); }
    else if (field == "debug_depth_mp4")        { cfg.debug_depth_mp4 = value; }
    else { throw std::invalid_argument("unknown visual_onnx field: " + key); }
    return true;
}

bool parse_airsim_depth_obstacle_detector_key(
    CoreStackRunnerConfig& config,
    const std::string& key,
    const std::string& value) {
    const std::string prefix = "airsim_depth_obstacle_detector.";
    if (key.rfind(prefix, 0U) != 0U) return false;

    const auto field = key.substr(prefix.size());
    auto& detector = config.airsim_depth_obstacle_detector;

    if (field == "pixel_stride") {
        detector.pixel_stride = static_cast<std::size_t>(std::stoul(value));
    } else if (field == "min_depth_m") {
        detector.min_depth_m = std::stof(value);
    } else if (field == "max_depth_m") {
        detector.max_depth_m = std::stof(value);
    } else if (field == "voxel_size_m") {
        detector.voxel_size_m = std::stof(value);
    } else if (field == "confidence") {
        detector.confidence = std::stof(value);
    } else if (field == "max_evidence") {
        detector.max_evidence = static_cast<std::size_t>(std::stoul(value));
    } else if (field == "normal_confidence") {
        detector.normal_confidence = std::stof(value);
    } else if (field == "derive_surface_normals_from_depth") {
        detector.derive_surface_normals_from_depth = parse_bool(value);
    } else {
        throw std::invalid_argument("unknown AirSim depth obstacle detector field: " + key);
    }

    return true;
}

void apply_config_value(CoreStackConfig& config, const std::string& key, const std::string& value) {
    if (parse_visual_onnx_key(config.runner, key, value)) return;
    if (parse_airsim_depth_obstacle_detector_key(config.runner, key, value)) return;
    if (parse_airsim_object_binding_key(config.providers, key, value)) return;
    if (parse_airsim_pattern_binding_key(config.providers, key, value)) return;
    if (key == "frame_source") config.providers.frame_source = value;
    else if (key == "ego_provider") config.providers.ego_provider = value;
    else if (key == "ego_provider_eval") config.providers.ego_provider_eval = value;
    else if (key == "depth") config.providers.depth = value;
    else if (key == "depth_eval") config.providers.depth_eval = value;
    else if (key == "detector") config.providers.detector = value;
    else if (key == "detector_eval") config.providers.detector_eval = value;
    else if (key == "camera_stabilizer") config.providers.camera_stabilizer = value;
    else if (key == "camera_stabilizer_eval") config.providers.camera_stabilizer_eval = value;
    else if (key == "tracker") config.providers.tracker = value;
    else if (key == "tracker_eval") config.providers.tracker_eval = value;
    else if (key == "identity_resolver") config.providers.identity_resolver = value;
    else if (key == "identity_resolver_eval") config.providers.identity_resolver_eval = value;
    else if (key == "projector") config.providers.projector = value;
    else if (key == "projector_eval") config.providers.projector_eval = value;
    else if (key == "projector_require_depth") config.providers.projector_require_depth = parse_bool(value);
    else if (key == "ghost_targets_enabled") config.providers.ghost_targets_enabled = parse_bool(value);
    else if (key == "ghost_targets_source") config.providers.ghost_targets_source = value;
    else if (key == "ghost_targets_scenario") config.providers.ghost_targets_scenario = value;
    else if (key == "ghost_targets_scenario_path") config.providers.ghost_targets_scenario_path = value;
    else if (key == "ghost_targets_airsim_scene_inventory_path") config.providers.ghost_targets_airsim_scene_inventory_path = value;
    else if (key == "ghost_targets_airsim_object_pose_stream_rate_hz") config.providers.ghost_targets_airsim_object_pose_stream_rate_hz = std::stod(value);
    else if (key == "world_model") config.providers.world_model = value;
    else if (key == "occupancy_source") config.providers.occupancy_source = value;
    else if (key == "frame_annotator") config.providers.frame_annotator = value;
    else if (key == "annotation_output_path") config.providers.annotation_output_path = value;
    else if (key == "annotation_output_fps") config.providers.annotation_output_fps = std::stod(value);
    else if (key == "pipeline_timing_enabled") config.providers.pipeline_timing_enabled = parse_bool(value);
    else if (key == "pipeline_timing_output_path") config.providers.pipeline_timing_output_path = value;
    else if (key == "recorded_manifest_path") config.providers.recorded_manifest_path = value;
    else if (key == "source_host") config.providers.source_host = value;
    else if (key == "source_rpc_port") config.providers.source_rpc_port = std::stoi(value);
    else if (key == "vehicle_name") config.providers.vehicle_name = value;
    else if (key == "vehicle_camera_name") config.providers.vehicle_camera_name = value;
    else if (key == "bridge_transport") config.providers.bridge_transport = value;
    else if (key == "bridge_command") config.providers.bridge_command = value;
    else if (key == "bridge_mode") config.providers.bridge_mode = value;
    else if (key == "ego_bridge_command") config.providers.ego_bridge_command = value;
    else if (key == "fallback_map_frame_id") config.providers.fallback_map_frame_id = MapFrameId{value};
    else if (key == "mission_controller") config.providers.mission_controller = value;
    else if (key == "mission_tick_hz") config.providers.mission_tick_hz = std::stod(value);
    else if (key == "flight_command_sink") config.providers.flight_command_sink = value;
    else if (key.rfind("mission_options.", 0U) == 0U) {
        const auto option_key = mission_option_key_from(key);
        if (option_key.empty()) throw std::invalid_argument("empty mission_options key");
        apply_mission_option(config.providers.mission_options, option_key, value);
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

void validate_depth_provider_name(const std::string& name, const char* field) {
    if (name.empty()) return;
    if (name != "airsim_gt_detector" && name != "airsim_gt_vd" && name != "visual_onnx")
        throw std::invalid_argument(std::string(field) + ": unknown depth provider: " + name);
}

void validate_config(const CoreStackProviderConfig& config) {
    if (config.ghost_targets_source != "trajectory_scenario" && config.ghost_targets_source != "airsim_objects") throw std::invalid_argument("unknown ghost_targets_source: " + config.ghost_targets_source);
    if (config.ghost_targets_source != "airsim_objects" && (!config.ghost_targets_airsim_objects.empty() || !config.ghost_targets_airsim_patterns.empty())) throw std::invalid_argument("ghost_targets_airsim bindings require ghost_targets_source=airsim_objects");
    if (config.occupancy_source != "synthetic_fixture" && config.occupancy_source != "airsim_ground_truth") throw std::invalid_argument("unknown occupancy_source: " + config.occupancy_source);
    if (config.occupancy_source == "airsim_ground_truth" && config.ghost_targets_source != "airsim_objects") throw std::invalid_argument("occupancy_source=airsim_ground_truth requires ghost_targets_source=airsim_objects");
    validate_airsim_object_bindings(config);
    validate_obstacle_sensing_cameras(config.mission_options);

    // Depth provider names (empty = inactive).
    validate_depth_provider_name(config.depth,      "depth");
    validate_depth_provider_name(config.depth_eval,  "depth_eval");

}

}  // namespace


CoreStackProviderConfig load_core_stack_config(const std::string& path) {
    auto config = load_core_stack_app_config(path);
    return std::move(config.providers);
}


void validate_provider_names(const CoreStackProviderConfig& config, const ProviderRegistry& registry) {
    const auto check = [](const std::string& field, const std::string& value, const std::vector<std::string>& valid) {
        if (std::find(valid.begin(), valid.end(), value) == valid.end()) throw std::invalid_argument("unknown " + field + ": " + value);
    };
    // Optional eval slot validation: empty = inactive (allowed); non-empty must be a known name.
    const auto check_opt = [](const std::string& field, const std::string& value, const std::vector<std::string>& valid) {
        if (value.empty()) return;
        if (std::find(valid.begin(), valid.end(), value) == valid.end()) throw std::invalid_argument("unknown " + field + ": " + value);
    };
    check("frame_source",        config.frame_source,        registry.frame_sources());
    check("ego_provider",        config.ego_provider,        registry.ego_providers());
    check_opt("ego_provider_eval", config.ego_provider_eval, registry.ego_providers());
    check("detector",            config.detector,            registry.detectors());
    check("camera_stabilizer",   config.camera_stabilizer,   registry.camera_stabilizers());
    check("tracker",             config.tracker,             registry.trackers());
    check("identity_resolver",   config.identity_resolver,   registry.identity_resolvers());
    check("projector",           config.projector,           registry.projectors());
    check("world_model",         config.world_model,         registry.world_models());
    check("frame_annotator",     config.frame_annotator,     registry.frame_annotators());
    check("mission_controller",  config.mission_controller,  registry.mission_controllers());
    check("flight_command_sink", config.flight_command_sink, registry.flight_command_sinks());
    // Eval slots (optional).
    check_opt("detector_eval",          config.detector_eval,          registry.detectors());
    check_opt("camera_stabilizer_eval", config.camera_stabilizer_eval, registry.camera_stabilizers());
    check_opt("tracker_eval",           config.tracker_eval,           registry.trackers());
    check_opt("identity_resolver_eval", config.identity_resolver_eval, registry.identity_resolvers());
    check_opt("projector_eval",         config.projector_eval,         registry.projectors());
}

// Load one file's key-value pairs into config, recursively expanding include: directives.
// loading_stack tracks the current include chain for circular-include detection.
// base_dir: directory used to resolve relative include paths (captured once at top-level load,
//           so resolution is stable regardless of CWD changes at runtime).
void load_file_into_config(CoreStackConfig& config,
                            const std::filesystem::path& file_path,
                            std::set<std::string>& loading_stack,
                            const std::filesystem::path& base_dir) {
    const auto canonical = std::filesystem::weakly_canonical(file_path).string();
    if (loading_stack.count(canonical)) {
        throw std::runtime_error("circular include detected involving: " + canonical);
    }
    loading_stack.insert(canonical);

    std::cerr << "config: loading " << file_path.string() << "\n";
    std::ifstream input{file_path};
    if (!input) {
        throw std::runtime_error(
            "❌ config load failed: cannot open '" + file_path.string() + "'\n"
            "   absolute path tried: " + std::filesystem::absolute(file_path).string() + "\n"
            "   base_dir for includes: " + base_dir.string() + "\n"
            "   Include paths must be relative to repo root (e.g. 'config/world-env/airsim.yaml').");
    }

    // Two-pass: collect all (key, value) pairs first so that include: lines are
    // processed before this file's own keys (include = base, own keys = override).
    std::vector<std::pair<std::string, std::string>> include_lines;
    std::vector<std::pair<std::string, std::string>> own_lines;

    std::string line;
    int line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        const auto comment_pos = line.find('#');
        if (comment_pos != std::string::npos) line = line.substr(0U, comment_pos);
        line = trim(line);
        if (line.empty()) continue;
        const auto separator_pos = line.find(':');
        if (separator_pos == std::string::npos)
            throw std::runtime_error("invalid config line " + std::to_string(line_number) +
                                     " in " + file_path.string() + ": missing ':'");
        const std::string key = trim(line.substr(0U, separator_pos));
        const std::string value = strip_quotes(trim(line.substr(separator_pos + 1U)));
        if (key.empty() || value.empty())
            throw std::runtime_error("invalid config line " + std::to_string(line_number) +
                                     " in " + file_path.string() + ": empty key or value");
        if (key == "include") {
            include_lines.emplace_back(key, value);
        } else {
            own_lines.emplace_back(key, value);
        }
    }

    // Apply included files first (base configs, earlier = lower priority).
    // Include paths are resolved relative to base_dir (repo root), captured once
    // at the top-level load call — not relative to the including file and not
    // sensitive to runtime CWD changes.
    for (const auto& [k, inc_path] : include_lines) {
        const auto resolved = std::filesystem::path{inc_path}.is_absolute()
            ? std::filesystem::path{inc_path}
            : base_dir / inc_path;
        load_file_into_config(config, resolved, loading_stack, base_dir);
    }

    // Apply this file's own keys — these override anything set by includes.
    for (const auto& [key, value] : own_lines) {
        try { apply_config_value(config, key, value); }
        catch (const std::exception& ex) {
            throw std::runtime_error("invalid config key '" + key + "' in " +
                                     file_path.string() + ": " + ex.what());
        }
    }

    loading_stack.erase(canonical);
}

CoreStackConfig load_core_stack_app_config(const std::string& path) {
    warn_unknown_dedalus_vars();
    CoreStackConfig config;
    std::set<std::string> loading_stack;
    const auto base_dir = std::filesystem::current_path();
    load_file_into_config(config, std::filesystem::path{path}, loading_stack, base_dir);

    // Apply env var overrides for depth and perception eval provider names
    // before validation so errors reference the final effective names.
    config.providers.depth      = env_str_or(config.providers.depth,      "DEDALUS_DEPTH");
    config.providers.depth_eval = env_str_or(config.providers.depth_eval, "DEDALUS_DEPTH_EVAL");
    apply_eval_env_overrides(config.providers);

    validate_config(config.providers);

    // Resolve string names → concrete provider instances.
    build_depth_slots(config);
    ProviderRegistry{}.populate_runner_eval_slots(config.providers, config.runner);

    return config;
}

CoreStackConfig load_core_stack_app_config(const std::vector<std::string>& paths) {
    if (paths.empty()) throw std::invalid_argument("load_core_stack_app_config: no config paths provided");
    if (paths.size() == 1U) return load_core_stack_app_config(paths[0]);

    // Merge multiple files left-to-right: each file (and its include: chain) is
    // applied on top of the previous, so later files take precedence.
    warn_unknown_dedalus_vars();
    CoreStackConfig config;
    std::set<std::string> loading_stack;
    const auto base_dir = std::filesystem::current_path();
    for (const auto& path : paths) {
        load_file_into_config(config, std::filesystem::path{path}, loading_stack, base_dir);
    }

    config.providers.depth      = env_str_or(config.providers.depth,      "DEDALUS_DEPTH");
    config.providers.depth_eval = env_str_or(config.providers.depth_eval, "DEDALUS_DEPTH_EVAL");
    apply_eval_env_overrides(config.providers);

    validate_config(config.providers);

    build_depth_slots(config);
    ProviderRegistry{}.populate_runner_eval_slots(config.providers, config.runner);

    return config;
}

}  // namespace dedalus
