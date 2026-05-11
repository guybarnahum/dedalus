#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "dedalus/core/types.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

namespace dedalus {

struct ImageView {
    int width{0};
    int height{0};
    int channels{0};
    std::vector<std::uint8_t> bytes;
};

struct CameraIntrinsics {
    double fx{0.0};
    double fy{0.0};
    double cx{0.0};
    double cy{0.0};
    double distortion_k1{0.0};
    double distortion_k2{0.0};
};

struct FramePacket {
    FrameId frame_id;
    TimePoint timestamp;
    CameraId camera_id;
    ImageView image;
    CameraIntrinsics intrinsics;

    std::optional<Pose3> camera_T_world;
    std::optional<Pose3> camera_T_body;
    std::optional<EgoState> ego_hint;
    std::optional<AppearanceCondition> appearance_condition;
};

class FrameSource {
public:
    virtual ~FrameSource() = default;
    virtual std::optional<FramePacket> next_frame() = 0;
};

class SyntheticFrameSource final : public FrameSource {
public:
    std::optional<FramePacket> next_frame() override;

private:
    bool emitted_{false};
};

}  // namespace dedalus
