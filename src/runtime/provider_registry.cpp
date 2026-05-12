#include "dedalus/runtime/provider_registry.hpp"

#include <stdexcept>
#include <string>
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

}  // namespace

CoreStackProviders ProviderRegistry::create(const CoreStackProviderConfig& config) const {
    CoreStackProviders providers;
    const auto airsim_config = airsim_config_from(config);

    if (config.frame_source == "synthetic") {
        providers.frame_source = std::make_unique<SyntheticFrameSource>();
    } else if (config.frame_source == "video_only") {
        providers.frame_source = std::make_unique<VideoOnlyFrameSource>(1U);
    } else if (config.frame_source == "recorded_frames") {
        if (config.recorded_manifest_path.empty()) {
            throw std::invalid_argument("recorded_frames provider requires recorded_manifest_path");
        }
        providers.frame_source = std::make_unique<RecordedFrameSource>(config.recorded_manifest_path);
    } else if (config.frame_source == "airsim") {
        providers.frame_source = std::make_unique<AirSimFrameSource>(airsim_config);
    } else {
        throw unknown_provider("frame_source", config.frame_source);
    }

    if (config.ego_provider == "frame_hint") {
        providers.ego_provider = std::make_unique<FrameHintEgoProvider>(config.fallback_map_frame_id);
    } else if (config.ego_provider == "no_telemetry") {
        providers.ego_provider = std::make_unique<NoTelemetryEgoProvider>(config.fallback_map_frame_id);
    } else if (config.ego_provider == "airsim") {
        providers.ego_provider = std::make_unique<AirSimEgoStateProvider>(airsim_config);
    } else {
        throw unknown_provider("ego_provider", config.ego_provider);
    }

    if (config.detector == "scripted") {
        providers.detector = std::make_unique<ScriptedDetector>();
    } else if (config.detector == "airsim_ground_truth") {
        providers.detector = std::make_unique<AirSimGroundTruthDetector>(airsim_config);
    } else {
        throw unknown_provider("detector", config.detector);
    }

    if (config.camera_stabilizer == "null") {
        providers.camera_stabilizer = std::make_unique<NullCameraStabilizer>();
    } else {
        throw unknown_provider("camera_stabilizer", config.camera_stabilizer);
    }

    if (config.tracker == "simple_centroid") {
        providers.tracker = std::make_unique<SimpleCentroidTracker>();
    } else {
        throw unknown_provider("tracker", config.tracker);
    }

    if (config.identity_resolver == "appearance_only") {
        providers.identity_resolver = std::make_unique<AppearanceOnlyIdentityResolver>();
    } else {
        throw unknown_provider("identity_resolver", config.identity_resolver);
    }

    if (config.projector == "flat_ground") {
        providers.projector = std::make_unique<FlatGroundProjector>();
    } else if (config.projector == "airsim_depth") {
        providers.projector = std::make_unique<AirSimDepthProjector>(airsim_config);
    } else {
        throw unknown_provider("projector", config.projector);
    }

    if (config.world_model == "in_memory") {
        providers.world_model = std::make_unique<InMemoryWorldModel>(config.fallback_map_frame_id);
    } else {
        throw unknown_provider("world_model", config.world_model);
    }

    if (config.frame_annotator == "null") {
        providers.frame_annotator = std::make_unique<NullFrameAnnotationSink>();
    } else if (config.frame_annotator == "ppm_sequence") {
        providers.frame_annotator = std::make_unique<PpmFrameAnnotationSink>(
            config.annotation_output_path,
            config.annotation_output_fps);
    } else if (config.frame_annotator == "mp4") {
        providers.frame_annotator = std::make_unique<Mp4FrameAnnotationSink>(
            config.annotation_output_path,
            config.annotation_output_fps);
    } else {
        throw unknown_provider("frame_annotator", config.frame_annotator);
    }

    return providers;
}

std::vector<std::string> ProviderRegistry::frame_sources() const {
    return {"synthetic", "video_only", "recorded_frames", "airsim"};
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

}  // namespace dedalus
