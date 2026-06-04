#pragma once

#include <string>
#include <vector>

#include "dedalus/core/types.hpp"
#include "dedalus/sensors/frame_source.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

namespace dedalus {

struct CameraSensingConfig {
    CameraId camera_id{"front_center"};
    std::string camera_name{"front_center"};
    std::string role{"visual_obstacle_detector"};

    double horizontal_fov_rad{1.5707963267948966};
    double vertical_fov_rad{1.0471975511965976};
    double near_range_m{0.5};
    double far_range_m{80.0};
    double min_reliable_range_m{1.0};
    double max_reliable_range_m{60.0};

    Vec3 body_T_camera_xyz_m;
    Vec3 body_T_camera_rpy_rad;

    std::string pointing_source{"camera_pointing_intent"};
};

struct CameraPointingState {
    CameraId camera_id{"front_center"};
    std::string camera_name{"front_center"};
    TimePoint timestamp;

    double pitch_rad{0.0};
    double yaw_rad{0.0};
    double roll_rad{0.0};

    bool valid{false};
    bool measured{false};
    std::string source{"neutral"};
};

struct CameraSensingVolume {
    TimePoint timestamp;
    FrameId frame_id;
    bool has_frame_id{false};

    CameraId camera_id{"front_center"};
    std::string camera_name{"front_center"};
    std::string role{"visual_obstacle_detector"};
    MapFrameId map_frame_id{"map_unknown"};

    Pose3 body_T_camera_mount;
    Pose3 body_T_camera_current;
    Pose3 map_T_camera_current;

    CameraIntrinsics intrinsics;
    double horizontal_fov_rad{0.0};
    double vertical_fov_rad{0.0};
    double near_range_m{0.0};
    double far_range_m{0.0};
    double min_reliable_range_m{0.0};
    double max_reliable_range_m{0.0};

    Vec3 origin_local;
    Vec3 forward_axis_local{1.0, 0.0, 0.0};
    Vec3 right_axis_local{0.0, 1.0, 0.0};
    Vec3 up_axis_local{0.0, 0.0, -1.0};

    CameraPointingState pointing_state;
};

struct SensingCoverageSnapshot {
    TimePoint timestamp;
    MapFrameId map_frame_id{"map_unknown"};
    std::vector<CameraSensingVolume> camera_volumes;
};

struct EgoSensingFrame {
    FramePacket frame;
    EgoState ego;
    CameraSensingVolume sensing_volume;
};

struct SensingVolumeQueryResult {
    bool inside{false};
    double range_m{0.0};
    double bearing_rad{0.0};
    double elevation_rad{0.0};
    double forward_m{0.0};
    double right_m{0.0};
    double up_m{0.0};
};

class SensingCoverageProvider {
public:
    explicit SensingCoverageProvider(std::vector<CameraSensingConfig> camera_configs = {});

    [[nodiscard]] SensingCoverageSnapshot snapshot(
        const std::vector<FramePacket>& frames,
        const EgoState& ego,
        const std::vector<CameraPointingState>& pointing_states) const;

    [[nodiscard]] CameraSensingVolume volume_for_frame(
        const FramePacket& frame,
        const EgoState& ego,
        const std::vector<CameraPointingState>& pointing_states) const;

private:
    std::vector<CameraSensingConfig> camera_configs_;
};

[[nodiscard]] SensingVolumeQueryResult query_point_in_camera_sensing_volume(
    const CameraSensingVolume& volume,
    const Vec3& point_local);

[[nodiscard]] ObstacleSensingVolume to_obstacle_sensing_volume(const CameraSensingVolume& volume);

}  // namespace dedalus
