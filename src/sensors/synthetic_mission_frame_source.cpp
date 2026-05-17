#include "dedalus/sensors/frame_source.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <thread>

namespace dedalus {
namespace {

constexpr int kTotalFrames = 260;
constexpr double kFramePeriodS = 0.1;
constexpr int kImageWidth = 160;
constexpr int kImageHeight = 120;

FramePacket make_base_frame(int frame_index) {
    FramePacket frame;
    const int display_index = frame_index + 1;
    frame.frame_id = FrameId{"synthetic_mission_" + std::to_string(display_index)};
    frame.timestamp = TimePoint{static_cast<Nanoseconds>(display_index * kFramePeriodS * 1'000'000'000.0)};
    frame.camera_id = CameraId{"synthetic_mission_front_center"};
    frame.image.width = kImageWidth;
    frame.image.height = kImageHeight;
    frame.image.channels = 3;
    frame.image.bytes.resize(
        static_cast<std::size_t>(frame.image.width * frame.image.height * frame.image.channels),
        static_cast<std::uint8_t>(frame_index % 255));
    frame.intrinsics.fx = 120.0;
    frame.intrinsics.fy = 120.0;
    frame.intrinsics.cx = 80.0;
    frame.intrinsics.cy = 60.0;
    return frame;
}

double scripted_height_m(int frame_index) {
    if (frame_index < 5) {
        return 0.0;
    }
    if (frame_index < 25) {
        const double u = static_cast<double>(frame_index - 5) / 20.0;
        return 2.2 * std::clamp(u, 0.0, 1.0);
    }
    if (frame_index < 95) {
        return 2.2;
    }
    if (frame_index < 135) {
        const double u = static_cast<double>(frame_index - 95) / 40.0;
        return 2.2 * (1.0 - std::clamp(u, 0.0, 1.0));
    }
    return 0.0;
}

Vec3 scripted_position(double height_m, int frame_index) {
    Vec3 position{0.0, 0.0, -height_m};
    if (frame_index >= 25 && frame_index < 75) {
        position.x = 0.02 * static_cast<double>(frame_index - 25);
    } else if (frame_index >= 75 && frame_index < 95) {
        position.x = std::max(0.0, 1.0 - 0.05 * static_cast<double>(frame_index - 75));
    }
    return position;
}

EgoFlightStatus flight_status_for(double height_m) {
    return height_m > 0.25 ? EgoFlightStatus::Airborne : EgoFlightStatus::Landed;
}

}  // namespace

std::optional<FramePacket> SyntheticMissionFrameSource::next_frame() {
    if (stop_requested_ || frame_index_ >= kTotalFrames) {
        return std::nullopt;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds{100});

    FramePacket frame = make_base_frame(frame_index_);
    const double height_m = scripted_height_m(frame_index_);

    EgoState ego;
    ego.timestamp = frame.timestamp;
    ego.map_frame_id = MapFrameId{"map_mission_ci_0001"};
    ego.height_m = height_m;
    ego.height_valid = true;
    ego.armed = frame_index_ >= 2 && frame_index_ < 150;
    ego.armed_valid = true;
    ego.flight_status = flight_status_for(height_m);
    ego.local_T_body.position = scripted_position(height_m, frame_index_);
    ego.velocity_local = Vec3{0.0, 0.0, 0.0};
    ego.confidence = 0.95F;
    frame.ego_hint = ego;

    AppearanceCondition appearance;
    appearance.timestamp = frame.timestamp;
    appearance.lighting_mode = LightingMode::Day;
    appearance.weather_mode = WeatherMode::Clear;
    appearance.sensor_mode = SensorMode::Rgb;
    appearance.confidence = 0.95F;
    frame.appearance_condition = appearance;

    ++frame_index_;
    return frame;
}

void SyntheticMissionFrameSource::request_stop() {
    stop_requested_ = true;
}

}  // namespace dedalus
