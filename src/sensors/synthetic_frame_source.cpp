#include "dedalus/sensors/frame_source.hpp"

namespace dedalus {

std::optional<FramePacket> SyntheticFrameSource::next_frame() {
    if (emitted_) {
        return std::nullopt;
    }

    emitted_ = true;

    FramePacket frame;
    frame.frame_id = FrameId{"frame_0001"};
    frame.timestamp = TimePoint{123456789};
    frame.camera_id = CameraId{"front_center"};
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

    EgoState ego;
    ego.timestamp = frame.timestamp;
    ego.map_frame_id = MapFrameId{"map_local_0001"};
    ego.local_T_body.position = Vec3{0.0, 0.0, -12.0};
    ego.velocity_local = Vec3{1.2, 0.0, 0.0};
    frame.ego_hint = ego;

    AppearanceCondition appearance;
    appearance.timestamp = frame.timestamp;
    appearance.lighting_mode = LightingMode::Day;
    appearance.weather_mode = WeatherMode::Clear;
    appearance.sensor_mode = SensorMode::Rgb;
    appearance.confidence = 0.75F;
    frame.appearance_condition = appearance;

    return frame;
}

}  // namespace dedalus
