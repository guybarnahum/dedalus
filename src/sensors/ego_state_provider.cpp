#include "dedalus/sensors/ego_state_provider.hpp"

#include <utility>

namespace dedalus {

FrameHintEgoProvider::FrameHintEgoProvider(MapFrameId fallback_map_frame_id)
    : fallback_map_frame_id_(std::move(fallback_map_frame_id)) {}

EgoStateEstimate FrameHintEgoProvider::estimate(const FramePacket& frame) {
    if (frame.ego_hint.has_value()) {
        EgoStateEstimate estimate;
        estimate.ego = *frame.ego_hint;
        estimate.telemetry_available = true;
        estimate.confidence = 0.95F;
        return estimate;
    }

    EgoState ego;
    ego.timestamp = frame.timestamp;
    ego.map_frame_id = fallback_map_frame_id_;

    EgoStateEstimate estimate;
    estimate.ego = ego;
    estimate.telemetry_available = false;
    estimate.confidence = 0.2F;
    return estimate;
}

NoTelemetryEgoProvider::NoTelemetryEgoProvider(MapFrameId fallback_map_frame_id)
    : fallback_map_frame_id_(std::move(fallback_map_frame_id)) {}

EgoStateEstimate NoTelemetryEgoProvider::estimate(const FramePacket& frame) {
    EgoState ego;
    ego.timestamp = frame.timestamp;
    ego.map_frame_id = fallback_map_frame_id_;

    EgoStateEstimate estimate;
    estimate.ego = ego;
    estimate.telemetry_available = false;
    estimate.confidence = 0.1F;
    return estimate;
}

}  // namespace dedalus
