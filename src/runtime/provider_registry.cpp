#include "dedalus/runtime/provider_registry.hpp"

#include <stdexcept>
#include <string>
#include <utility>

#include "dedalus/sensors/replay_frame_source.hpp"

namespace dedalus {
namespace {

std::invalid_argument unknown_provider(const std::string& category, const std::string& name) {
    return std::invalid_argument("unknown " + category + " provider: " + name);
}

}  // namespace

CoreStackProviders ProviderRegistry::create(const CoreStackProviderConfig& config) const {
    CoreStackProviders providers;

    if (config.frame_source == "synthetic") {
        providers.frame_source = std::make_unique<SyntheticFrameSource>();
    } else if (config.frame_source == "video_only") {
        providers.frame_source = std::make_unique<VideoOnlyFrameSource>(1U);
    } else {
        throw unknown_provider("frame_source", config.frame_source);
    }

    if (config.ego_provider == "frame_hint") {
        providers.ego_provider = std::make_unique<FrameHintEgoProvider>(config.fallback_map_frame_id);
    } else if (config.ego_provider == "no_telemetry") {
        providers.ego_provider = std::make_unique<NoTelemetryEgoProvider>(config.fallback_map_frame_id);
    } else {
        throw unknown_provider("ego_provider", config.ego_provider);
    }

    if (config.detector == "scripted") {
        providers.detector = std::make_unique<ScriptedDetector>();
    } else {
        throw unknown_provider("detector", config.detector);
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
    } else {
        throw unknown_provider("projector", config.projector);
    }

    if (config.world_model == "in_memory") {
        providers.world_model = std::make_unique<InMemoryWorldModel>(config.fallback_map_frame_id);
    } else {
        throw unknown_provider("world_model", config.world_model);
    }

    return providers;
}

std::vector<std::string> ProviderRegistry::frame_sources() const {
    return {"synthetic", "video_only"};
}

std::vector<std::string> ProviderRegistry::ego_providers() const {
    return {"frame_hint", "no_telemetry"};
}

std::vector<std::string> ProviderRegistry::detectors() const {
    return {"scripted"};
}

std::vector<std::string> ProviderRegistry::trackers() const {
    return {"simple_centroid"};
}

std::vector<std::string> ProviderRegistry::identity_resolvers() const {
    return {"appearance_only"};
}

std::vector<std::string> ProviderRegistry::projectors() const {
    return {"flat_ground"};
}

std::vector<std::string> ProviderRegistry::world_models() const {
    return {"in_memory"};
}

}  // namespace dedalus
