#pragma once

#include <optional>
#include <string>

#include "dedalus/core/types.hpp"
#include "dedalus/sensing/airsim_depth_obstacle_detector.hpp"
#include "dedalus/sensors/frame_source.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

namespace dedalus {

EgoState parse_ego_json(const std::string& json, const MapFrameId& map_frame_id, TimePoint frame_timestamp);

std::optional<AirSimDepthFrame> parse_depth_frame_optional(
    const std::string& json,
    const FramePacket& frame,
    const MapFrameId& map_frame_id);

}  // namespace dedalus
