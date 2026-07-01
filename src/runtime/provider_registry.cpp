#include "dedalus/runtime/provider_registry.hpp"

#include "dedalus/runtime/core_stack_runner.hpp"

#include <cstdlib>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "dedalus/sensors/recorded_frame_source.hpp"
#include "dedalus/sensors/replay_frame_source.hpp"
#include "dedalus/sensors/visual_ego_state_provider.hpp"
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

OccupancySourceKind occupancy_source_from(const CoreStackProviderConfig& config) {
    if (config.occupancy_source == "synthetic_fixture") {
        return OccupancySourceKind::SyntheticFixture;
    }
    if (config.occupancy_source == "airsim_ground_truth") {
        return OccupancySourceKind::AirSimGroundTruth;
    }
    throw std::invalid_argument("unknown occupancy_source: " + config.occupancy_source);
}

std::string scene_inventory_path_from(const CoreStackProviderConfig& config) {
    const char* env = std::getenv("DEDALUS_AIRSIM_SCENE_INVENTORY");
    if (env != nullptr && *env != '\0') {
        return std::string{env};
    }
    return config.ghost_targets_airsim_scene_inventory_path;
}

double env_double_or(const char* name, double fallback) {
    const char* env = std::getenv(name);
    if (env == nullptr || *env == '\0') return fallback;
    return std::stod(env);
}

int env_int_or(const char* name, int fallback) {
    const char* env = std::getenv(name);
    if (env == nullptr || *env == '\0') return fallback;
    return std::stoi(env);
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
        if (config.ghost_targets_airsim_objects.empty() && config.ghost_targets_airsim_patterns.empty()) {
            throw std::invalid_argument(
                "ghost_targets_source: airsim_objects requires exact object or pattern bindings");
        }
        return std::make_unique<GhostTargetProvider>(
            AirSimGhostObjectSourceConfig{
                .host = config.source_host,
                .rpc_port = config.source_rpc_port,
                .bridge_command = "python3 simulation/airsim/scripts/airsim-object-poses.py",
                .bridge_transport = config.bridge_transport,
                .scene_inventory_path = scene_inventory_path_from(config),
                .stream_rate_hz = config.ghost_targets_airsim_object_pose_stream_rate_hz,
                .nearby_radius_m = env_double_or("DEDALUS_AIRSIM_GT_NEARBY_RADIUS_M", 80.0),
                .max_objects_per_frame = env_int_or("DEDALUS_AIRSIM_GT_MAX_OBJECTS_PER_FRAME", 128),
                .static_refresh_every_n_frames = env_int_or("DEDALUS_AIRSIM_GT_STATIC_REFRESH_EVERY_N_FRAMES", 10),
                .objects = config.ghost_targets_airsim_objects,
                .patterns = config.ghost_targets_airsim_patterns});
    }
    throw std::invalid_argument("unknown ghost_targets_source: " + config.ghost_targets_source);
}

template <typename T>
struct FactoryEntry {
    std::string_view key;
    std::function<std::unique_ptr<T>()> factory;
};

template <typename T>
struct SharedFactoryEntry {
    std::string_view key;
    std::function<std::shared_ptr<T>()> factory;
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

template <typename T>
std::shared_ptr<T> resolve_shared(
    const std::string& category,
    const std::string& name,
    std::initializer_list<SharedFactoryEntry<T>> entries) {
    for (const auto& entry : entries) {
        if (name == entry.key) return entry.factory();
    }
    throw unknown_provider(category, name);
}

}  // namespace

CoreStackProviders ProviderRegistry::create(const CoreStackProviderConfig& config) const {
    CoreStackProviders providers;
    const auto airsim = airsim_config_from(config);
    providers.obstacle_sensing_cameras = config.mission_options.obstacle_sensing_cameras;

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
        {"frame_hint",      [&]() { return std::make_unique<FrameHintEgoProvider>(config.fallback_map_frame_id); }},
        {"no_telemetry",    [&]() { return std::make_unique<NoTelemetryEgoProvider>(config.fallback_map_frame_id); }},
        {"airsim",          [&]() { return std::make_unique<AirSimEgoStateProvider>(airsim); }},
        {"visual_odometry", [&]() {
            VisualOdometryConfig vo;
            vo.map_frame_id = config.fallback_map_frame_id;
            return std::make_unique<VisualEgoStateProvider>(vo);
        }},
    });

    providers.detector = resolve_shared<Detector>("detector", config.detector, {
        {"null",                [&]() { return std::make_shared<NullDetector>(); }},
        {"scripted",            [&]() { return std::make_shared<ScriptedDetector>(); }},
        {"airsim_ground_truth", [&]() {
            AirSimProviderConfig det_cfg = airsim;
            for (const auto& obj : config.ghost_targets_airsim_objects) {
                AirSimDetectorObjectBinding b;
                b.airsim_object_name = obj.airsim_object_name;
                b.class_label        = obj.class_label;
                b.confidence         = obj.confidence;
                b.size_m             = obj.size_m;
                det_cfg.detector_objects.push_back(std::move(b));
            }
            return std::make_shared<AirSimGroundTruthDetector>(std::move(det_cfg));
        }},
    });

    providers.camera_stabilizer = resolve_shared<CameraStabilizer>("camera_stabilizer", config.camera_stabilizer, {
        {"null", [&]() { return std::make_shared<NullCameraStabilizer>(); }},
    });

    providers.tracker = resolve_shared<Tracker>("tracker", config.tracker, {
        {"simple_centroid", [&]() { return std::make_shared<SimpleCentroidTracker>(); }},
    });

    providers.identity_resolver = resolve_shared<IdentityResolver>("identity_resolver", config.identity_resolver, {
        {"appearance_only", [&]() { return std::make_shared<AppearanceOnlyIdentityResolver>(); }},
    });

    providers.projector = resolve_shared<Projector3D>("projector", config.projector, {
        {"flat_ground",  [&]() { return std::make_shared<FlatGroundProjector>(); }},
        {"airsim_depth", [&]() { return std::make_shared<AirSimDepthProjector>(airsim); }},
    });

    if (config.ghost_targets_enabled) {
        providers.ghost_targets = make_ghost_target_provider(config);
    }

    providers.world_model = resolve<InMemoryWorldModel>("world_model", config.world_model, {
        {"in_memory", [&]() { return std::make_unique<InMemoryWorldModel>(InMemoryWorldModelConfig{
                              .map_frame_id = config.fallback_map_frame_id,
                              .occupancy_source_kind = occupancy_source_from(config)}); }},
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

// Resolve optional eval (slot B) providers from config string names.
// Empty name → leave the slot null (inactive).
// Called by load_core_stack_app_config() after parsing.
void ProviderRegistry::populate_runner_eval_slots(
    const CoreStackProviderConfig& config,
    CoreStackRunnerConfig& runner) const {

    const auto resolve_opt_shared_detector = [&](const std::string& name)
        -> std::shared_ptr<Detector> {
        if (name.empty()) return nullptr;
        return resolve_shared<Detector>("detector_eval", name, {
            {"scripted",            [&]() { return std::make_shared<ScriptedDetector>(); }},
            {"airsim_ground_truth", [&]() { return std::make_shared<AirSimGroundTruthDetector>(airsim_config_from(config)); }},
        });
    };

    const auto resolve_opt_shared_stabilizer = [&](const std::string& name)
        -> std::shared_ptr<CameraStabilizer> {
        if (name.empty()) return nullptr;
        return resolve_shared<CameraStabilizer>("camera_stabilizer_eval", name, {
            {"null", [&]() { return std::make_shared<NullCameraStabilizer>(); }},
        });
    };

    const auto resolve_opt_shared_tracker = [&](const std::string& name)
        -> std::shared_ptr<Tracker> {
        if (name.empty()) return nullptr;
        return resolve_shared<Tracker>("tracker_eval", name, {
            {"simple_centroid", [&]() { return std::make_shared<SimpleCentroidTracker>(); }},
        });
    };

    const auto resolve_opt_shared_identity = [&](const std::string& name)
        -> std::shared_ptr<IdentityResolver> {
        if (name.empty()) return nullptr;
        return resolve_shared<IdentityResolver>("identity_resolver_eval", name, {
            {"appearance_only", [&]() { return std::make_shared<AppearanceOnlyIdentityResolver>(); }},
        });
    };

    const auto resolve_opt_shared_projector = [&](const std::string& name)
        -> std::shared_ptr<Projector3D> {
        if (name.empty()) return nullptr;
        return resolve_shared<Projector3D>("projector_eval", name, {
            {"flat_ground",  [&]() { return std::make_shared<FlatGroundProjector>(); }},
            {"airsim_depth", [&]() { return std::make_shared<AirSimDepthProjector>(airsim_config_from(config)); }},
        });
    };

    const auto resolve_opt_shared_ego = [&](const std::string& name)
        -> std::shared_ptr<EgoStateProvider> {
        if (name.empty()) return nullptr;
        return resolve_shared<EgoStateProvider>("ego_provider_eval", name, {
            {"frame_hint",   [&]() { return std::make_shared<FrameHintEgoProvider>(config.fallback_map_frame_id); }},
            {"no_telemetry", [&]() { return std::make_shared<NoTelemetryEgoProvider>(config.fallback_map_frame_id); }},
            {"airsim",       [&]() { return std::make_shared<AirSimEgoStateProvider>(airsim_config_from(config)); }},
        });
    };

    runner.ego_provider_reference     = resolve_opt_shared_ego(config.ego_provider_eval);
    runner.detector_reference         = resolve_opt_shared_detector(config.detector_eval);
    runner.stabilizer_reference       = resolve_opt_shared_stabilizer(config.camera_stabilizer_eval);
    runner.tracker_reference          = resolve_opt_shared_tracker(config.tracker_eval);
    runner.identity_resolver_reference = resolve_opt_shared_identity(config.identity_resolver_eval);
    runner.projector_reference        = resolve_opt_shared_projector(config.projector_eval);
}

std::vector<std::string> ProviderRegistry::frame_sources() const { return {"synthetic", "synthetic_mission", "video_only", "recorded_frames", "airsim"}; }
std::vector<std::string> ProviderRegistry::ego_providers() const { return {"frame_hint", "no_telemetry", "airsim", "visual_odometry"}; }
std::vector<std::string> ProviderRegistry::detectors() const { return {"null", "scripted", "airsim_ground_truth"}; }
std::vector<std::string> ProviderRegistry::camera_stabilizers() const { return {"null"}; }
std::vector<std::string> ProviderRegistry::trackers() const { return {"simple_centroid"}; }
std::vector<std::string> ProviderRegistry::identity_resolvers() const { return {"appearance_only"}; }
std::vector<std::string> ProviderRegistry::projectors() const { return {"flat_ground", "airsim_depth"}; }
std::vector<std::string> ProviderRegistry::world_models() const { return {"in_memory"}; }
std::vector<std::string> ProviderRegistry::frame_annotators() const { return {"null", "ppm_sequence", "mp4"}; }
std::vector<std::string> ProviderRegistry::mission_controllers() const { return {"disabled", "trajectory_mission", "object_behavior"}; }
std::vector<std::string> ProviderRegistry::flight_command_sinks() const { return {"disabled", "airsim_velocity", "px4_bridge", "px4_mavlink"}; }

}  // namespace dedalus
