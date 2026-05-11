#pragma once

#include <optional>

#include "dedalus/core/types.hpp"
#include "dedalus/sensors/frame_source.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

namespace dedalus {

struct EgoStateEstimate {
    std::optional<EgoState> ego;
    bool telemetry_available{false};
    float confidence{0.0F};
};

class EgoStateProvider {
public:
    virtual ~EgoStateProvider() = default;
    virtual EgoStateEstimate estimate(const FramePacket& frame) = 0;
};

class FrameHintEgoProvider final : public EgoStateProvider {
public:
    explicit FrameHintEgoProvider(MapFrameId fallback_map_frame_id = MapFrameId{"map_local_0001"});

    EgoStateEstimate estimate(const FramePacket& frame) override;

private:
    MapFrameId fallback_map_frame_id_;
};

class NoTelemetryEgoProvider final : public EgoStateProvider {
public:
    explicit NoTelemetryEgoProvider(MapFrameId fallback_map_frame_id = MapFrameId{"map_video_only_0001"});

    EgoStateEstimate estimate(const FramePacket& frame) override;

private:
    MapFrameId fallback_map_frame_id_;
};

}  // namespace dedalus
