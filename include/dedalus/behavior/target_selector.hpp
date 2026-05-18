#pragma once

#include <optional>
#include <string>

#include "dedalus/behavior/behavior_spec.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

namespace dedalus {

enum class TargetSelectionStatus {
    Selected,
    Reacquiring,
    Lost,
    NoCandidates,
    InvalidSpec,
};

struct TargetSelection {
    bool selected{false};
    TargetSelectionStatus status{TargetSelectionStatus::NoCandidates};

    AgentId agent_id;
    TrackId source_track_id;
    IdentityId identity_id;

    ClassLabel class_label{ClassLabel::Unknown};
    float confidence{0.0F};

    Vec3 position_local;
    Vec3 velocity_local;
    MapFrameId map_frame_id;

    double target_age_s{0.0};
    std::string reason;
};

std::string to_string(TargetSelectionStatus status);

class TargetSelector {
public:
    TargetSelection select(
        const WorldSnapshot& snapshot,
        const TargetSelectorSpec& spec,
        const std::optional<TargetSelection>& previous = std::nullopt) const;
};

}  // namespace dedalus
