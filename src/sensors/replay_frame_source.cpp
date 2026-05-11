#include "dedalus/sensors/replay_frame_source.hpp"

#include <string>

namespace dedalus {

ReplayFrameSource::ReplayFrameSource(std::vector<FramePacket> frames)
    : frames_(std::move(frames)) {}

std::optional<FramePacket> ReplayFrameSource::next_frame() {
    if (next_index_ >= frames_.size()) {
        return std::nullopt;
    }

    return frames_.at(next_index_++);
}

std::size_t ReplayFrameSource::remaining() const {
    return frames_.size() - next_index_;
}

VideoOnlyFrameSource::VideoOnlyFrameSource(std::size_t frame_count)
    : frame_count_(frame_count) {}

std::optional<FramePacket> VideoOnlyFrameSource::next_frame() {
    if (emitted_ >= frame_count_) {
        return std::nullopt;
    }

    const auto frame_number = emitted_ + 1U;

    FramePacket frame;
    frame.frame_id = FrameId{"video_frame_" + std::string(4 - std::to_string(frame_number).size(), '0') +
                             std::to_string(frame_number)};
    frame.timestamp = TimePoint{static_cast<Nanoseconds>(emitted_) * 33'333'333};
    frame.camera_id = CameraId{"video_only_front_center"};
    frame.image.width = 640;
    frame.image.height = 480;
    frame.image.channels = 3;
    frame.image.bytes.resize(
        static_cast<std::size_t>(frame.image.width * frame.image.height * frame.image.channels),
        0U);
    frame.intrinsics.fx = 420.0;
    frame.intrinsics.fy = 420.0;
    frame.intrinsics.cx = 320.0;
    frame.intrinsics.cy = 240.0;

    AppearanceCondition appearance;
    appearance.timestamp = frame.timestamp;
    appearance.lighting_mode = LightingMode::Unknown;
    appearance.weather_mode = WeatherMode::Unknown;
    appearance.sensor_mode = SensorMode::Rgb;
    appearance.confidence = 0.25F;
    frame.appearance_condition = appearance;

    ++emitted_;
    return frame;
}

}  // namespace dedalus
