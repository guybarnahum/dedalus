#pragma once

#include <optional>
#include <string>

#include "dedalus/core/types.hpp"
#include "dedalus/sensing/airsim_depth_obstacle_detector.hpp"
#include "dedalus/sensors/frame_source.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

namespace dedalus {

EgoState parse_ego_json(const std::string& json, const MapFrameId& map_frame_id, TimePoint frame_timestamp);

// depth_width / depth_height come from JSON sidecar; float data from the
// binary depth chunk (already decoded to float32 by the caller).
std::optional<AirSimDepthFrame> parse_depth_frame_from_binary(
    const std::string& json,
    std::vector<float> depth_m,
    const FramePacket& frame,
    const MapFrameId& map_frame_id);

}  // namespace dedalus
