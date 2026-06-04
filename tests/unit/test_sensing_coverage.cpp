#include <cmath>
#include <iostream>
#include <vector>

#include "dedalus/sensing/sensing_coverage.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTolerance = 1.0e-6;

bool near(double lhs, double rhs, double tolerance = kTolerance) {
    return std::abs(lhs - rhs) <= tolerance;
}

bool near_vec(const dedalus::Vec3& lhs, const dedalus::Vec3& rhs, double tolerance = kTolerance) {
    return near(lhs.x, rhs.x, tolerance) && near(lhs.y, rhs.y, tolerance) && near(lhs.z, rhs.z, tolerance);
}

dedalus::FramePacket frame(std::string camera_name) {
    dedalus::FramePacket packet;
    packet.frame_id = dedalus::FrameId{"frame_0001"};
    packet.timestamp = dedalus::TimePoint{1000000000};
    packet.camera_id = dedalus::CameraId{std::move(camera_name)};
    packet.image.width = 640;
    packet.image.height = 480;
    packet.intrinsics.fx = 320.0;
    packet.intrinsics.fy = 320.0;
    packet.intrinsics.cx = 320.0;
    packet.intrinsics.cy = 240.0;
    return packet;
}

dedalus::EgoState ego() {
    dedalus::EgoState state;
    state.timestamp = dedalus::TimePoint{1000000000};
    state.map_frame_id = dedalus::MapFrameId{"map_local_0001"};
    state.local_T_body.position = dedalus::Vec3{0.0, 0.0, -10.0};
    state.local_T_body.rotation_rpy = dedalus::Vec3{0.0, 0.0, 0.0};
    state.height_m = 10.0;
    state.height_valid = true;
    return state;
}

dedalus::CameraSensingConfig config(std::string camera_name) {
    dedalus::CameraSensingConfig cfg;
    cfg.camera_id = dedalus::CameraId{camera_name};
    cfg.camera_name = std::move(camera_name);
    cfg.horizontal_fov_rad = kPi / 2.0;
    cfg.vertical_fov_rad = kPi / 3.0;
    cfg.near_range_m = 0.5;
    cfg.far_range_m = 80.0;
    cfg.min_reliable_range_m = 1.0;
    cfg.max_reliable_range_m = 60.0;
    return cfg;
}

bool validate_neutral_camera_volume() {
    const auto cfg = config("front_center");
    dedalus::SensingCoverageProvider provider{{cfg}};
    const auto volume = provider.volume_for_frame(frame("front_center"), ego(), {});

    if (volume.camera_name != "front_center" || volume.map_frame_id.value != "map_local_0001") {
        std::cerr << "neutral volume did not preserve camera/map identity\n";
        return false;
    }
    if (!near_vec(volume.origin_local, dedalus::Vec3{0.0, 0.0, -10.0})) {
        std::cerr << "neutral volume origin did not match ego/camera origin\n";
        return false;
    }
    if (!near_vec(volume.forward_axis_local, dedalus::Vec3{1.0, 0.0, 0.0})) {
        std::cerr << "neutral volume forward axis should face body +X\n";
        return false;
    }

    const auto forward = dedalus::query_point_in_camera_sensing_volume(volume, dedalus::Vec3{10.0, 0.0, -10.0});
    if (!forward.inside || !near(forward.forward_m, 10.0) || !near(forward.bearing_rad, 0.0)) {
        std::cerr << "forward point should be inside neutral volume\n";
        return false;
    }

    const auto lateral = dedalus::query_point_in_camera_sensing_volume(volume, dedalus::Vec3{10.0, 20.0, -10.0});
    if (lateral.inside) {
        std::cerr << "wide lateral point should be outside neutral volume\n";
        return false;
    }

    const auto behind = dedalus::query_point_in_camera_sensing_volume(volume, dedalus::Vec3{-1.0, 0.0, -10.0});
    if (behind.inside) {
        std::cerr << "behind point should be outside neutral volume\n";
        return false;
    }

    return true;
}

bool validate_downward_pitch() {
    const auto cfg = config("front_center");
    dedalus::CameraPointingState pointing;
    pointing.camera_id = dedalus::CameraId{"front_center"};
    pointing.camera_name = "front_center";
    pointing.timestamp = dedalus::TimePoint{1000000000};
    pointing.pitch_rad = kPi / 4.0;
    pointing.valid = true;
    pointing.measured = false;
    pointing.source = "camera_pointing_intent";

    dedalus::SensingCoverageProvider provider{{cfg}};
    const auto volume = provider.volume_for_frame(frame("front_center"), ego(), {pointing});

    if (!near_vec(volume.forward_axis_local, dedalus::Vec3{std::sqrt(0.5), 0.0, -std::sqrt(0.5)})) {
        std::cerr << "pitched camera forward axis should rotate downward in NED coordinates\n";
        return false;
    }

    const auto forward_level = dedalus::query_point_in_camera_sensing_volume(volume, dedalus::Vec3{10.0, 0.0, -10.0});
    if (forward_level.inside) {
        std::cerr << "forward-level point should leave a 45-degree downward pitched 60-degree vertical FOV\n";
        return false;
    }

    const auto forward_down = dedalus::query_point_in_camera_sensing_volume(volume, dedalus::Vec3{10.0, 0.0, -20.0});
    if (!forward_down.inside) {
        std::cerr << "forward-down point should be inside downward pitched volume\n";
        return false;
    }

    return true;
}

bool validate_mount_yaw_offset() {
    auto cfg = config("right_obstacle_camera");
    cfg.body_T_camera_rpy_rad = dedalus::Vec3{0.0, 0.0, kPi / 2.0};
    dedalus::SensingCoverageProvider provider{{cfg}};
    const auto volume = provider.volume_for_frame(frame("right_obstacle_camera"), ego(), {});

    if (!near_vec(volume.forward_axis_local, dedalus::Vec3{0.0, 1.0, 0.0})) {
        std::cerr << "mount yaw should rotate camera optical axis to body +Y\n";
        return false;
    }

    const auto right_side = dedalus::query_point_in_camera_sensing_volume(volume, dedalus::Vec3{0.0, 10.0, -10.0});
    if (!right_side.inside) {
        std::cerr << "right-side point should be inside yaw-offset camera\n";
        return false;
    }
    const auto forward_body = dedalus::query_point_in_camera_sensing_volume(volume, dedalus::Vec3{10.0, 0.0, -10.0});
    if (forward_body.inside) {
        std::cerr << "body-forward point should be outside yaw-offset camera\n";
        return false;
    }

    return true;
}

bool validate_multiple_cameras_and_export() {
    auto front = config("front_center");
    auto downward = config("downward");
    downward.body_T_camera_rpy_rad = dedalus::Vec3{0.0, kPi / 2.0, 0.0};
    downward.role = "landing_area_detector";

    std::vector<dedalus::FramePacket> frames;
    frames.push_back(frame("front_center"));
    auto downward_frame = frame("downward");
    downward_frame.frame_id = dedalus::FrameId{"frame_0002"};
    frames.push_back(downward_frame);

    dedalus::SensingCoverageProvider provider{{front, downward}};
    const auto coverage = provider.snapshot(frames, ego(), {});
    if (coverage.camera_volumes.size() != 2U) {
        std::cerr << "coverage snapshot should contain one volume per frame camera\n";
        return false;
    }
    if (coverage.camera_volumes[0].camera_name != "front_center" || coverage.camera_volumes[1].camera_name != "downward") {
        std::cerr << "coverage snapshot did not preserve camera order/names\n";
        return false;
    }
    if (coverage.camera_volumes[1].role != "landing_area_detector") {
        std::cerr << "coverage snapshot did not preserve camera role\n";
        return false;
    }

    const auto exported = dedalus::to_obstacle_sensing_volume(coverage.camera_volumes.front());
    if (exported.sensor_name != "front_center" || !exported.has_source_frame || exported.source_frame_id.value != "frame_0001") {
        std::cerr << "exported obstacle sensing volume did not preserve frame/camera metadata\n";
        return false;
    }
    if (!near(exported.far_range_m, 80.0F)) {
        std::cerr << "exported obstacle sensing volume did not preserve range\n";
        return false;
    }

    return true;
}

}  // namespace

int main() {
    if (!validate_neutral_camera_volume()) return 1;
    if (!validate_downward_pitch()) return 1;
    if (!validate_mount_yaw_offset()) return 1;
    if (!validate_multiple_cameras_and_export()) return 1;
    return 0;
}
