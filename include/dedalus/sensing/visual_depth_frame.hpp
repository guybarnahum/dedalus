#pragma once

#include <cstdint>
#include <vector>

#include "dedalus/core/types.hpp"

namespace dedalus {

// Full Brown-Conrady distortion model used in the unproject step of the
// depth projection kernel. Not used as a preprocessing step before inference —
// the depth network infers on raw (fisheye) frames; distortion is absorbed into
// network weights.
struct LensDistortion {
    double k1{0.0};
    double k2{0.0};
    double k3{0.0};
    double k4{0.0};
    double k5{0.0};
    double k6{0.0};
    double p1{0.0};  // tangential
    double p2{0.0};  // tangential
};

// Raw RGB frame delivered to the depth engine.
//
// Intentionally independent of AirSim types. Constructed by
// VisualDepthObstacleDetector from EgoSensingFrame::frame.image.
struct VisualDepthFrame {
    TimePoint timestamp;
    FrameId frame_id;
    CameraId camera_id;
    MapFrameId map_frame_id{"map_unknown"};

    int width{0};
    int height{0};
    int channels{3};                  // RGB
    std::vector<std::uint8_t> bytes;  // row-major, H × W × channels

    // Pinhole intrinsics — populated from ObstacleSensingVolume / CameraConfig.
    double fx{0.0};
    double fy{0.0};
    double cx{0.0};
    double cy{0.0};

    // Lens distortion — used only in the unproject step, not before inference.
    LensDistortion distortion;
};

}  // namespace dedalus
