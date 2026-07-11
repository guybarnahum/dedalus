#pragma once

#include <string>
#include <vector>

#include "dedalus/occupancy/occupancy_types.hpp"

namespace dedalus {

struct AirSimDepthFrame {
    TimePoint timestamp;
    FrameId source_frame_id;
    bool has_source_frame{false};
    std::string sensor_name{"front_center"};
    MapFrameId map_frame_id{"map_unknown"};

    int width{0};
    int height{0};
    std::vector<float> depth_m;};

}  // namespace dedalus
