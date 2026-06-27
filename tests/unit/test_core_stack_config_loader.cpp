#include <algorithm>
#include <cmath>
#include <iostream>

#include "dedalus/runtime/config_loader.hpp"
#include "dedalus/runtime/core_stack_runner.hpp"
#include "dedalus/runtime/provider_registry.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

bool near(double lhs, double rhs, double tolerance = 1.0e-6) {
    return std::abs(lhs - rhs) <= tolerance;
}

bool contains(const std::vector<std::string>& values, const std::string& expected) {
    return std::find(values.begin(), values.end(), expected) != values.end();
}

}  // namespace

int main() {
    const auto config = dedalus::load_core_stack_config("config/ci/core_stack_ci.yaml");

    if (config.frame_source != "synthetic" || config.ego_provider != "frame_hint" ||
        config.detector != "scripted" || config.tracker != "simple_centroid" ||
        config.identity_resolver != "appearance_only" || config.projector != "flat_ground" ||
        config.world_model != "in_memory") {
        std::cerr << "core_stack_ci provider config did not parse expected provider names\n";
        return 1;
    }

    if (config.pipeline_timing_enabled) {
        std::cerr << "core_stack_ci should keep pipeline timing disabled by default\n";
        return 1;
    }

    if (config.ghost_targets_enabled) {
        std::cerr << "core_stack_ci should keep ghost targets disabled by default\n";
        return 1;
    }

    if (config.ghost_targets_source != "trajectory_scenario") {
        std::cerr << "core_stack_ci should default ghost_targets_source to trajectory_scenario\n";
        return 1;
    }

    if (config.fallback_map_frame_id.value != "map_local_0001") {
        std::cerr << "core_stack_ci config did not parse expected fallback map frame\n";
        return 1;
    }

    const auto profile_config = dedalus::load_core_stack_config("config/ci/core_stack_profile_ci.yaml");
    if (!profile_config.pipeline_timing_enabled) {
        std::cerr << "core_stack_profile_ci did not enable pipeline timing\n";
        return 1;
    }
    if (profile_config.pipeline_timing_output_path != "out/profile/pipeline_profile.jsonl") {
        std::cerr << "core_stack_profile_ci did not parse expected timing output path\n";
        return 1;
    }

    const auto sensing_config = dedalus::load_core_stack_config("config/ci/core_stack_sensing_coverage_ci.yaml");
    if (sensing_config.mission_options.obstacle_sensing_cameras.size() != 2U) {
        std::cerr << "sensing coverage config did not parse two obstacle sensing cameras\n";
        return 1;
    }
    const auto& front_camera = sensing_config.mission_options.obstacle_sensing_cameras[0];
    if (front_camera.camera_id.value != "front_center" || front_camera.camera_name != "front_center" ||
        front_camera.role != "visual_obstacle_detector" ||
        !near(front_camera.horizontal_fov_rad, kPi / 2.0) ||
        !near(front_camera.vertical_fov_rad, kPi / 3.0) ||
        !near(front_camera.near_range_m, 0.5) || !near(front_camera.far_range_m, 80.0) ||
        !near(front_camera.min_reliable_range_m, 1.0) || !near(front_camera.max_reliable_range_m, 60.0) ||
        !near(front_camera.body_T_camera_xyz_m.x, 0.1) || !near(front_camera.body_T_camera_xyz_m.z, -0.05) ||
        front_camera.pointing_source != "camera_pointing_intent") {
        std::cerr << "front obstacle sensing camera fields did not parse as expected\n";
        return 1;
    }
    const auto& downward_camera = sensing_config.mission_options.obstacle_sensing_cameras[1];
    if (downward_camera.camera_id.value != "downward" || downward_camera.camera_name != "downward" ||
        downward_camera.role != "landing_area_detector" ||
        !near(downward_camera.horizontal_fov_rad, 1.2217304763960306) ||
        !near(downward_camera.vertical_fov_rad, 0.8726646259971648) ||
        !near(downward_camera.body_T_camera_rpy_rad.y, kPi / 2.0) ||
        downward_camera.pointing_source != "configured_fixed") {
        std::cerr << "downward obstacle sensing camera fields did not parse as expected\n";
        return 1;
    }

    const auto ghost_config = dedalus::load_core_stack_config("config/ci/core_stack_ghost_targets_ci.yaml");
    if (!ghost_config.ghost_targets_enabled) {
        std::cerr << "ghost target config did not enable ghost targets\n";
        return 1;
    }
    if (ghost_config.ghost_targets_source != "trajectory_scenario") {
        std::cerr << "ghost target config should use trajectory_scenario source\n";
        return 1;
    }
    if (ghost_config.ghost_targets_scenario != "person_pair_crossing") {
        std::cerr << "ghost target config did not parse scenario\n";
        return 1;
    }
    if (ghost_config.frame_annotator != "ppm_sequence" ||
        ghost_config.annotation_output_path != "out/ghost_targets_annotation") {
        std::cerr << "ghost target config did not parse annotation settings\n";
        return 1;
    }

    const auto mission_config = dedalus::load_core_stack_config("config/ci/core_stack_trajectory_mission_placeholder.yaml");
    if (mission_config.mission_controller != "trajectory_mission") {
        std::cerr << "mission placeholder did not parse mission_controller\n";
        return 1;
    }
    if (!near(mission_config.mission_tick_hz, 10.0)) {
        std::cerr << "mission placeholder did not parse mission_tick_hz\n";
        return 1;
    }
    if (mission_config.flight_command_sink != "px4_bridge") {
        std::cerr << "mission placeholder did not parse flight_command_sink\n";
        return 1;
    }
    if (mission_config.mission_options.flight_control_mode != "px4" ||
        mission_config.mission_options.safe_height_m != 16.0 ||
        mission_config.mission_options.trajectory_path !=
            "config/behaviors/trajectories/circle_figure8.json" ||
        mission_config.mission_options.home_policy != "initial_ego_pose" ||
        mission_config.mission_options.px4_command_bridge.find(
            "tools/px4/px4-command-bridge.py") == std::string::npos ||
        mission_config.mission_options.prepare_session_command.find(
            "simulation/airsim/scripts/airsim-prepare-session.py") == std::string::npos) {
        std::cerr << "mission placeholder did not parse expected mission_options.* values\n";
        return 1;
    }

    const auto object_behavior_config = dedalus::load_core_stack_config("config/ci/core_stack_object_behavior_mission.yaml");
    if (object_behavior_config.mission_controller != "object_behavior") {
        std::cerr << "object behavior config did not parse mission_controller\n";
        return 1;
    }
    if (object_behavior_config.flight_command_sink != "disabled") {
        std::cerr << "object behavior config should use disabled sink for synthetic skeleton validation\n";
        return 1;
    }
    if (!object_behavior_config.ghost_targets_enabled ||
        object_behavior_config.ghost_targets_source != "trajectory_scenario" ||
        object_behavior_config.ghost_targets_scenario != "person_pair_crossing") {
        std::cerr << "object behavior config should enable ghost target trajectory scenario\n";
        return 1;
    }
    if (object_behavior_config.mission_options.behavior_spec_path !=
        "config/behaviors/follow_specific_track.yaml") {
        std::cerr << "object behavior config did not use canonical config/behaviors spec path\n";
        return 1;
    }

    const auto airsim_object_config = dedalus::load_core_stack_config(
        "config/ci/core_stack_object_behavior_airsim_existing_object_example.yaml");
    if (!airsim_object_config.ghost_targets_enabled ||
        airsim_object_config.ghost_targets_source != "airsim_objects") {
        std::cerr << "AirSim existing-object config did not enable airsim_objects ghost source\n";
        return 1;
    }
    if (airsim_object_config.ghost_targets_airsim_objects.size() != 1U) {
        std::cerr << "AirSim existing-object config did not parse exactly one object binding\n";
        return 1;
    }
    const auto& binding = airsim_object_config.ghost_targets_airsim_objects.front();
    if (binding.source_track_id.value != "ghost_person_001" ||
        binding.airsim_object_name != "BRPlayer_01_96" ||
        binding.class_label != dedalus::ClassLabel::Person ||
        !near(binding.confidence, 0.82) ||
        !near(binding.size_m.x, 0.6) || !near(binding.size_m.y, 0.6) || !near(binding.size_m.z, 1.8)) {
        std::cerr << "AirSim existing-object binding fields did not parse as expected\n";
        return 1;
    }

    dedalus::ProviderRegistry registry;
    dedalus::CoreStackRunner runner{registry.create(config)};
    if (!runner.run_once()) {
        std::cerr << "config-composed runner failed\n";
        return 1;
    }

    const auto snapshot = runner.snapshot();
    if (snapshot.active_map_frame_id.value != "map_local_0001" || snapshot.agents.empty()) {
        std::cerr << "config-composed snapshot missing expected state\n";
        return 1;
    }

    dedalus::CoreStackRunner sensing_runner{registry.create(sensing_config)};
    if (!sensing_runner.run_once()) {
        std::cerr << "sensing coverage config-composed runner failed\n";
        return 1;
    }
    const auto sensing_snapshot = sensing_runner.snapshot();
    if (sensing_snapshot.obstacle_sensing_volumes.size() != 1U) {
        std::cerr << "sensing coverage runner should publish the frame camera sensing volume\n";
        return 1;
    }
    const auto& published_volume = sensing_snapshot.obstacle_sensing_volumes.front();
    const auto expected_origin = dedalus::Vec3{
        sensing_snapshot.ego.local_T_body.position.x + front_camera.body_T_camera_xyz_m.x,
        sensing_snapshot.ego.local_T_body.position.y + front_camera.body_T_camera_xyz_m.y,
        sensing_snapshot.ego.local_T_body.position.z + front_camera.body_T_camera_xyz_m.z};
    if (published_volume.sensor_name != "front_center" || !published_volume.has_source_frame ||
        published_volume.source_frame_id.value.empty() || published_volume.map_frame_id.value != "map_local_0001" ||
        !near(published_volume.origin_local.x, expected_origin.x) ||
        !near(published_volume.origin_local.y, expected_origin.y) ||
        !near(published_volume.origin_local.z, expected_origin.z) ||
        !near(published_volume.horizontal_fov_rad, static_cast<float>(kPi / 2.0)) ||
        !near(published_volume.vertical_fov_rad, static_cast<float>(kPi / 3.0))) {
        std::cerr << "sensing coverage runner did not publish configured front camera volume\n";
        return 1;
    }

    dedalus::CoreStackRunner ghost_runner{registry.create(ghost_config)};
    if (!ghost_runner.run_once()) {
        std::cerr << "ghost config-composed runner failed\n";
        return 1;
    }
    const auto ghost_snapshot = ghost_runner.snapshot();
    bool saw_ghost_person_001 = false;
    bool saw_ghost_person_002 = false;
    for (const auto& agent : ghost_snapshot.agents) {
        if (agent.source_track_id.value == "ghost_person_001") {
            saw_ghost_person_001 = true;
        }
        if (agent.source_track_id.value == "ghost_person_002") {
            saw_ghost_person_002 = true;
        }
    }
    if (!saw_ghost_person_001 || !saw_ghost_person_002) {
        std::cerr << "ghost config-composed snapshot missing expected ghost agents\n";
        return 1;
    }

    const auto airsim_object_providers = registry.create(airsim_object_config);
    if (!airsim_object_providers.ghost_targets) {
        std::cerr << "AirSim existing-object provider did not compose a ghost target provider\n";
        return 1;
    }

    const auto mission_controllers = registry.mission_controllers();
    if (!contains(mission_controllers, "trajectory_mission") || !contains(mission_controllers, "object_behavior")) {
        std::cerr << "mission controller registry missing expected entries\n";
        return 1;
    }

    const auto flight_sinks = registry.flight_command_sinks();
    if (flight_sinks.empty()) {
        std::cerr << "flight command sink registry list is empty\n";
        return 1;
    }
    if (!contains(flight_sinks, "px4_bridge") || !contains(flight_sinks, "px4_mavlink")) {
        std::cerr << "flight command sink registry missing px4_bridge or px4_mavlink\n";
        return 1;
    }

    // validate_provider_names: valid config must pass without throwing
    try {
        dedalus::validate_provider_names(config, registry);
    } catch (const std::exception& ex) {
        std::cerr << "validate_provider_names rejected valid config: " << ex.what() << "\n";
        return 1;
    }

    // validate_provider_names: unknown frame_source must be rejected at load time
    {
        dedalus::CoreStackProviderConfig bad_config = config;
        bad_config.frame_source = "nonexistent_provider";
        try {
            dedalus::validate_provider_names(bad_config, registry);
            std::cerr << "validate_provider_names should reject unknown frame_source\n";
            return 1;
        } catch (const std::invalid_argument&) {
            // expected
        }
    }

    // validate_provider_names: unknown flight_command_sink must be rejected at load time
    {
        dedalus::CoreStackProviderConfig bad_config = config;
        bad_config.flight_command_sink = "unknown_sink";
        try {
            dedalus::validate_provider_names(bad_config, registry);
            std::cerr << "validate_provider_names should reject unknown flight_command_sink\n";
            return 1;
        } catch (const std::invalid_argument&) {
            // expected
        }
    }

    return 0;
}
