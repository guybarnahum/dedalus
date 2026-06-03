#pragma once

#include <memory>
#include <string>
#include <vector>

#include "dedalus/behavior/mission_controller.hpp"
#include "dedalus/perception/ghost_targets.hpp"
#include "dedalus/perception/perception_pipeline.hpp"
#include "dedalus/sensors/ego_state_provider.hpp"
#include "dedalus/sensors/frame_source.hpp"
#include "dedalus/visualization/frame_annotator.hpp"
#include "dedalus/world_model/in_memory_world_model.hpp"

namespace dedalus {

struct CoreStackProviderConfig {
    std::string frame_source{"synthetic"};
    std::string ego_provider{"frame_hint"};
    std::string detector{"scripted"};
    std::string camera_stabilizer{"null"};
    std::string tracker{"simple_centroid"};
    std::string identity_resolver{"appearance_only"};
    std::string projector{"flat_ground"};
    bool ghost_targets_enabled{false};
    std::string ghost_targets_source{"trajectory_scenario"};
    std::string ghost_targets_scenario{"person_pair_crossing"};
    std::string ghost_targets_scenario_path;
    std::vector<AirSimGhostObjectBinding> ghost_targets_airsim_objects;
    std::vector<AirSimGhostObjectPatternBinding> ghost_targets_airsim_patterns;
    std::string ghost_targets_airsim_scene_inventory_path;
    std::string world_model{"in_memory"};
    std::string occupancy_source{"synthetic_fixture"};
    std::string frame_annotator{"null"};
    std::string annotation_output_path{"out/annotated.mp4"};
    double annotation_output_fps{5.0};
    bool pipeline_timing_enabled{false};
    std::string pipeline_timing_output_path{"out/profile/pipeline_profile.jsonl"};
    std::string recorded_manifest_path;
    std::string source_host{"127.0.0.1"};
    int source_rpc_port{41451};
    std::string vehicle_name{"PX4"};
    std::string vehicle_camera_name{"front_center"};
    std::string bridge_transport{"pipe"};
    std::string bridge_command{"python3 simulation/airsim/scripts/airsim-capture-frame.py"};
    std::string bridge_mode{"one_shot_ppm"};
    std::string ego_bridge_command{"python3 simulation/airsim/scripts/airsim-capture-ego.py"};
    MapFrameId fallback_map_frame_id{"map_local_0001"};

    std::string mission_controller{"disabled"};
    double mission_tick_hz{10.0};
    std::string flight_command_sink{"disabled"};
    MissionOptions mission_options;
};

struct CoreStackProviders {
    std::unique_ptr<FrameSource> frame_source;
    std::unique_ptr<EgoStateProvider> ego_provider;
    std::shared_ptr<Detector> detector;
    std::shared_ptr<CameraStabilizer> camera_stabilizer;
    std::shared_ptr<Tracker> tracker;
    std::shared_ptr<IdentityResolver> identity_resolver;
    std::shared_ptr<Projector3D> projector;
    std::unique_ptr<GhostTargetProvider> ghost_targets;
    std::unique_ptr<InMemoryWorldModel> world_model;
    std::unique_ptr<FrameAnnotationSink> frame_annotator;
};

class ProviderRegistry {
public:
    [[nodiscard]] CoreStackProviders create(const CoreStackProviderConfig& config) const;

    [[nodiscard]] std::vector<std::string> frame_sources() const;
    [[nodiscard]] std::vector<std::string> ego_providers() const;
    [[nodiscard]] std::vector<std::string> detectors() const;
    [[nodiscard]] std::vector<std::string> camera_stabilizers() const;
    [[nodiscard]] std::vector<std::string> trackers() const;
    [[nodiscard]] std::vector<std::string> identity_resolvers() const;
    [[nodiscard]] std::vector<std::string> projectors() const;
    [[nodiscard]] std::vector<std::string> world_models() const;
    [[nodiscard]] std::vector<std::string> frame_annotators() const;
    [[nodiscard]] std::vector<std::string> mission_controllers() const;
    [[nodiscard]] std::vector<std::string> flight_command_sinks() const;
};

}  // namespace dedalus
