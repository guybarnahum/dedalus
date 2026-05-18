#pragma once

#include <string>

#include "dedalus/world_model/world_snapshot.hpp"

namespace dedalus {

struct WorldImageCameraIntrinsics {
    int width{0};
    int height{0};
    double fx{0.0};
    double fy{0.0};
    double cx{0.0};
    double cy{0.0};
    double near_plane_m{0.05};
};

struct WorldImageCameraExtrinsics {
    // Body-to-camera transform. The initial M3 convention is:
    //   body/local X: forward
    //   body/local Y: right
    //   body/local Z: down
    //   camera depth axis: +X
    // Viewport projection uses u = fx * (Y/X) + cx and v = fy * (Z/X) + cy.
    Pose3 body_T_camera;
};

struct ProjectedWorldPoint {
    bool visible{false};
    double u_px{0.0};
    double v_px{0.0};
    double depth_m{0.0};
    Vec3 position_body;
    Vec3 position_camera;
    Vec3 position_ego_relative;
    double range_m{0.0};
    double bearing_deg{0.0};
    std::string reason{"invalid_input"};
};

struct WorldToImageProjectionConfig {
    WorldImageCameraIntrinsics intrinsics;
    WorldImageCameraExtrinsics extrinsics;
};

WorldImageCameraIntrinsics pinhole_intrinsics_from_horizontal_fov(
    int width,
    int height,
    double horizontal_fov_deg,
    double near_plane_m = 0.05);

ProjectedWorldPoint project_local_point_to_image(
    const Vec3& point_local,
    const EgoState& ego,
    const WorldToImageProjectionConfig& config);

}  // namespace dedalus
