#include <algorithm>
#include <cmath>
#include <iostream>

#include "dedalus/runtime/config_loader.hpp"
#include "dedalus/runtime/core_stack_runner.hpp"
#include "dedalus/runtime/provider_registry.hpp"

namespace {

bool near(double lhs, double rhs) {
    return std::abs(lhs - rhs) < 1.0e-9;
}

bool contains(const std::vector<std::string>& values, const std::string& expected) {
    return std::find(values.begin(), values.end(), expected) != values.end();
}

}  // namespace

int main() {
    const auto config = dedalus::load_core_stack_config("config/core_stack_ci.yaml");

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

    const auto profile_config = dedalus::load_core_stack_config("config/core_stack_profile_ci.yaml");
    if (!profile_config.pipeline_timing_enabled) {
        std::cerr << "core_stack_profile_ci did not enable pipeline timing\n";
        return 1;
    }
    if (profile_config.pipeline_timing_output_path != "out/profile/pipeline_profile.jsonl") {
        std::cerr << "core_stack_profile_ci did not parse expected timing output path\n";
        return 1;
    }

    const auto ghost_config = dedalus::load_core_stack_config("config/core_stack_ghost_targets_ci.yaml");
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

    const auto mission_config = dedalus::load_core_stack_config("config/core_stack_trajectory_mission_placeholder.yaml");
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

    const auto object_behavior_config = dedalus::load_core_stack_config("config/core_stack_object_behavior_mission.yaml");
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
        "config/core_stack_object_behavior_airsim_existing_object_example.yaml");
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
