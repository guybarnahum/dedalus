#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "dedalus/core/types.hpp"
#include "dedalus/sensing/airsim_depth_obstacle_detector.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

namespace dedalus {

struct ImageView {
    int width{0};
    int height{0};
    int channels{0};
    std::vector<std::uint8_t> bytes;
};

struct FrameSourceTiming {
    std::string name;
    std::int64_t duration_us{0};
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
    std::optional<AirSimDepthFrame> depth_frame;
    std::optional<Pose3> camera_T_body;
    std::optional<EgoState> ego_hint;
    std::optional<AppearanceCondition> appearance_condition;
    std::vector<FrameSourceTiming> source_timings;
};

class FrameSource {
public:
    virtual ~FrameSource() = default;
    virtual std::optional<FramePacket> next_frame() = 0;
    virtual void request_stop() {}
};

class AsyncPrefetchFrameSource final : public FrameSource {
public:
    explicit AsyncPrefetchFrameSource(std::unique_ptr<FrameSource> inner);
    ~AsyncPrefetchFrameSource() override;

    AsyncPrefetchFrameSource(const AsyncPrefetchFrameSource&) = delete;
    AsyncPrefetchFrameSource& operator=(const AsyncPrefetchFrameSource&) = delete;

    std::optional<FramePacket> next_frame() override;
    void request_stop() override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

class SyntheticFrameSource final : public FrameSource {
public:
    std::optional<FramePacket> next_frame() override;

private:
    bool emitted_{false};
};

class SyntheticMissionFrameSource final : public FrameSource {
public:
    SyntheticMissionFrameSource() = default;
    std::optional<FramePacket> next_frame() override;
    void request_stop() override;

private:
    int frame_index_{0};
    bool stop_requested_{false};
};

}  // namespace dedalus
