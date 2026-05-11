#pragma once

#include <memory>
#include <string>
#include <vector>

#include "dedalus/perception/perception_pipeline.hpp"
#include "dedalus/sensors/ego_state_provider.hpp"
#include "dedalus/sensors/frame_source.hpp"
#include "dedalus/world_model/in_memory_world_model.hpp"

namespace dedalus {

struct CoreStackProviderConfig {
    std::string frame_source{"synthetic"};
    std::string ego_provider{"frame_hint"};
    std::string detector{"scripted"};
    std::string tracker{"simple_centroid"};
    std::string identity_resolver{"appearance_only"};
    std::string projector{"flat_ground"};
    std::string world_model{"in_memory"};
    MapFrameId fallback_map_frame_id{"map_local_0001"};
};

struct CoreStackProviders {
    std::unique_ptr<FrameSource> frame_source;
    std::unique_ptr<EgoStateProvider> ego_provider;
    std::unique_ptr<Detector> detector;
    std::unique_ptr<Tracker> tracker;
    std::unique_ptr<IdentityResolver> identity_resolver;
    std::unique_ptr<Projector3D> projector;
    std::unique_ptr<InMemoryWorldModel> world_model;
};

class ProviderRegistry {
public:
    [[nodiscard]] CoreStackProviders create(const CoreStackProviderConfig& config) const;

    [[nodiscard]] std::vector<std::string> frame_sources() const;
    [[nodiscard]] std::vector<std::string> ego_providers() const;
    [[nodiscard]] std::vector<std::string> detectors() const;
    [[nodiscard]] std::vector<std::string> trackers() const;
    [[nodiscard]] std::vector<std::string> identity_resolvers() const;
    [[nodiscard]] std::vector<std::string> projectors() const;
    [[nodiscard]] std::vector<std::string> world_models() const;
};

}  // namespace dedalus
