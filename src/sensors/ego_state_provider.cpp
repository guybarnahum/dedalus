#include "dedalus/sensors/ego_state_provider.hpp"

namespace dedalus {

NoTelemetryEgoProvider::NoTelemetryEgoProvider(MapFrameId fallback_map_frame_id)
    : fallback_map_frame_id_(std::move(fallback_map_frame_id)) {}

EgoStateEstimate NoTelemetryEgoProvider::estimate(TimePoint timestamp) {
    EgoState ego;
    ego.timestamp = timestamp;
    ego.map_frame_id = fallback_map_frame_id_;

    EgoStateEstimate estimate;
    estimate.ego = ego;
    estimate.telemetry_available = false;
    estimate.confidence = 0.1F;
    return estimate;
}

}  // namespace dedalus
