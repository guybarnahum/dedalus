#include "dedalus/runtime/provider_registry.hpp"

#include <functional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "dedalus/sensors/recorded_frame_source.hpp"
#include "dedalus/sensors/replay_frame_source.hpp"
#include "dedalus/simulation/airsim_providers.hpp"

namespace dedalus {
namespace {

std::invalid_argument unknown_provider(const std::string& category, const std::string& name) {
    return std::invalid_argument("unknown " + category + " provider: " + name);
}

AirSimProviderConfig airsim_config_from(const CoreStackProviderConfig& config) {
    AirSimProviderConfig airsim;
    airsim.host = config.source_host;
    airsim.rpc_port = config.source_rpc_port;
    airsim.vehicle_name = config.vehicle_name;
    airsim.camera_name = config.vehicle_camera_name;
    airsim.transport = config.bridge_transport;
    airsim.bridge_command = config.bridge_command;
    airsim.bridge_mode = config.bridge_mode;
    airsim.ego_bridge_command = config.ego_bridge_command;
    airsim.map_frame_id = config.fallback_map_frame_id;
    return airsim;
}

std::string ghost_scenario_path_from(const CoreStackProviderConfig& config) {
    if (!config.ghost_targets_scenario_path.empty()) {
        return config.ghost_targets_scenario_path;
    }
    if (config.ghost_targets_scenario == "person_pair_crossing") {
        return "config/behaviors/ghost_detections/person_pair_crossing.json";
    }
    throw std::invalid_argument("unknown ghost_targets_scenario: " + config.ghost_targets_scenario);
}

std::unique_ptr<GhostTargetProvider> make_ghost_target_provider(const CoreStackProviderConfig& config) {
    if (config.ghost_targets_source == "trajectory_scenario") {
        return std::make_unique<GhostTargetProvider>(
            GhostScenario::load_from_file(ghost_scenario_path_from(config)));
    }
    if (config.ghost_targets_source == "airsim_objects") {
        return std::make_unique<GhostTargetProvider>(
            AirSimGhostObjectSourceConfig{
                .host = config.source_host,
                .rpc_port = config.source_rpc_port,
                .bridge_command = "python3 simulation/airsim/scripts/airsim-object-poses.py",
                .bridge_transport = config.bridge_transport,
                .objects = config.ghost_targets_airsim_objects});
    }
    throw std::invalid_argument("unknown ghost_targets_source: " + config.ghost_targets_source);
}

// Table-driven provider resolution. Each entry maps a key string to a factory
// lambda. resolve() walks the table and throws unknown_provider on no match.
template <typename T>
struct FactoryEntry {
    std::string_view key;
    std::function<std::unique_ptr<T>()> factory;
};

template <typename T>
std::unique_ptr<T> resolve(
    const std::string& category,
    const std::string& name,
    std::initializer_list<FactoryEntry<T>> entries) {
    for (const auto& entry : entries) {
        if (name == entry.key) return entry.factory();
    }
    throw unknown_provider(category, name);
}

}  // namespace

CoreStackProviders ProviderRegistry::create(const CoreStackProviderConfig& config) const {
    CoreStackProviders providers;
    const auto airsim = airsim_config_from(config);

    providers.frame_source = resolve<FrameSource>("frame_source", config.frame_source, {
        {"synthetic",         [&]() { return std::make_unique<SyntheticFrameSource>(); }},
        {"synthetic_mission", [&]() { return std::make_unique<SyntheticMissionFrameSource>(); }},
        {"video_only",        [&]() { return std::make_unique<VideoOnlyFrameSource>(1U); }},
        {"recorded_frames",   [&]() -> std::unique_ptr<FrameSource> {
            if (config.recorded_manifest_path.empty()) {
                throw std::invalid_argument("recorded_frames provider requires recorded_manifest_path");
            }
            return std::make_unique<RecordedFrameSource>(config.recorded_manifest_path);
        }},
        {"airsim",            [&]() { return std::make_unique<AirSimFrameSource>(airsim); }},
    });

    providers.ego_provider = resolve<EgoStateProvider>("ego_provider", config.ego_provider, {
        {"frame_hint",   [&]() { return std::make_unique<FrameHintEgoProvider>(config.fallback_map_frame_id); }},
        {"no_telemetry", [&]() { return std::make_unique<NoTelemetryEgoProvider>(config.fallback_map_frame_id); }},
        {"airsim",       [&]() { return std::make_unique<AirSimEgoStateProvider>(airsim); }},
    });

    providers.detector = resolve<Detector>("detector", config.detector, {
        {"scripted",            [&]() { return std::make_unique<ScriptedDetector>(); }},
        {"airsim_ground_truth", [&]() { return std::make_unique<AirSimGroundTruthDetector>(airsim); }},
    });

    providers.camera_stabilizer = resolve<CameraStabilizer>("camera_stabilizer", config.camera_stabilizer, {
        {"null", [&]() { return std::make_unique<NullCameraStabilizer>(); }},
    });

    providers.tracker = resolve<Tracker>("tracker", config.tracker, {
        {"simple_centroid", [&]() { return std::make_unique<SimpleCentroidTracker>(); }},
    });

    providers.identity_resolver = resolve<IdentityResolver>("identity_resolver", config.identity_resolver, {
        {"appearance_only", [&]() { return std::make_unique<AppearanceOnlyIdentityResolver>(); }},
    });

    providers.projector = resolve<Projector3D>("projector", config.projector, {
        {"flat_ground",  [&]() { return std::make_unique<FlatGroundProjector>(); }},
        {"airsim_depth", [&]() { return std::make_unique<AirSimDepthProjector>(airsim); }},
    });

    if (config.ghost_targets_enabled) {
        providers.ghost_targets = make_ghost_target_provider(config);
    }

    providers.world_model = resolve<InMemoryWorldModel>("world_model", config.world_model, {
        {"in_memory", [&]() { return std::make_unique<InMemoryWorldModel>(config.fallback_map_frame_id); }},
    });

    providers.frame_annotator = resolve<FrameAnnotationSink>("frame_annotator", config.frame_annotator, {
        {"null",         [&]() { return std::make_unique<NullFrameAnnotationSink>(); }},
        {"ppm_sequence", [&]() { return std::make_unique<PpmFrameAnnotationSink>(
                                     config.annotation_output_path,
                                     config.annotation_output_fps); }},
        {"mp4",          [&]() { return std::make_unique<Mp4FrameAnnotationSink>(
                                     config.annotation_output_path,
                                     config.annotation_output_fps); }},
    });

    return providers;
}

std::vector<std::string> ProviderRegistry::frame_sources() const {
    return {"synthetic", "synthetic_mission", "video_only", "recorded_frames", "airsim"};
}

std::vector<std::string> ProviderRegistry::ego_providers() const {
    return {"frame_hint", "no_telemetry", "airsim"};
}

std::vector<std::string> ProviderRegistry::detectors() const {
    return {"scripted", "airsim_ground_truth"};
}

std::vector<std::string> ProviderRegistry::camera_stabilizers() const {
    return {"null"};
}

std::vector<std::string> ProviderRegistry::trackers() const {
    return {"simple_centroid"};
}

std::vector<std::string> ProviderRegistry::identity_resolvers() const {
    return {"appearance_only"};
}

std::vector<std::string> ProviderRegistry::projectors() const {
    return {"flat_ground", "airsim_depth"};
}

std::vector<std::string> ProviderRegistry::world_models() const {
    return {"in_memory"};
}

std::vector<std::string> ProviderRegistry::frame_annotators() const {
    return {"null", "ppm_sequence", "mp4"};
}

std::vector<std::string> ProviderRegistry::mission_controllers() const {
    return {"disabled", "trajectory_mission", "object_behavior"};
}

std::vector<std::string> ProviderRegistry::flight_command_sinks() const {
    return {"disabled", "airsim_velocity"};
}

}  // namespace dedalus
