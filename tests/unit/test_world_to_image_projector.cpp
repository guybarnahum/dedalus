#include "dedalus/visualization/world_to_image_projector.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void require_near(double actual, double expected, double tolerance, const std::string& message) {
    if (std::abs(actual - expected) > tolerance) {
        throw std::runtime_error(message + " actual=" + std::to_string(actual) + " expected=" + std::to_string(expected));
    }
}

dedalus::WorldToImageProjectionConfig default_config() {
    dedalus::WorldToImageProjectionConfig config;
    config.intrinsics = dedalus::pinhole_intrinsics_from_horizontal_fov(640, 360, 90.0);
    config.extrinsics.body_T_camera.position = dedalus::Vec3{0.0, 0.0, 0.0};
    config.extrinsics.body_T_camera.rotation_rpy = dedalus::Vec3{0.0, 0.0, 0.0};
    return config;
}

dedalus::EgoState ego_at(dedalus::Vec3 position, double yaw_rad = 0.0) {
    dedalus::EgoState ego;
    ego.map_frame_id = dedalus::MapFrameId{"map_local_0001"};
    ego.local_T_body.position = position;
    ego.local_T_body.rotation_rpy = dedalus::Vec3{0.0, 0.0, yaw_rad};
    return ego;
}

void projects_center_left_right_and_vertical_motion() {
    const auto config = default_config();
    const auto ego = ego_at(dedalus::Vec3{0.0, 0.0, 0.0});

    const auto center = dedalus::project_local_point_to_image(dedalus::Vec3{10.0, 0.0, 0.0}, ego, config);
    require(center.visible, "point ahead should be visible");
    require(center.reason == "visible", "visible point should report visible reason");
    require_near(center.u_px, config.intrinsics.cx, 1.0e-9, "center u should match cx");
    require_near(center.v_px, config.intrinsics.cy, 1.0e-9, "center v should match cy");
    require_near(center.depth_m, 10.0, 1.0e-9, "center depth should be forward range");
    require_near(center.bearing_deg, 0.0, 1.0e-9, "center bearing should be zero");

    const auto right = dedalus::project_local_point_to_image(dedalus::Vec3{10.0, 2.0, 0.0}, ego, config);
    const auto left = dedalus::project_local_point_to_image(dedalus::Vec3{10.0, -2.0, 0.0}, ego, config);
    require(right.visible && left.visible, "left/right points should be visible");
    require(right.u_px > center.u_px, "positive body/local Y should project to the right");
    require(left.u_px < center.u_px, "negative body/local Y should project to the left");

    const auto down = dedalus::project_local_point_to_image(dedalus::Vec3{10.0, 0.0, 2.0}, ego, config);
    const auto up = dedalus::project_local_point_to_image(dedalus::Vec3{10.0, 0.0, -2.0}, ego, config);
    require(down.visible && up.visible, "up/down points should be visible");
    require(down.v_px > center.v_px, "positive Z/down should project lower in the image");
    require(up.v_px < center.v_px, "negative Z/up should project higher in the image");
}

void rejects_behind_and_outside_points() {
    const auto config = default_config();
    const auto ego = ego_at(dedalus::Vec3{0.0, 0.0, 0.0});

    const auto behind = dedalus::project_local_point_to_image(dedalus::Vec3{-1.0, 0.0, 0.0}, ego, config);
    require(!behind.visible, "point behind camera should not be visible");
    require(behind.reason == "behind_camera", "behind point should report behind_camera");

    const auto outside = dedalus::project_local_point_to_image(dedalus::Vec3{10.0, 20.0, 0.0}, ego, config);
    require(!outside.visible, "point outside horizontal FOV should not be visible");
    require(outside.reason == "outside_image", "outside point should report outside_image");
}

void applies_ego_translation_and_yaw() {
    const auto config = default_config();
    const dedalus::Vec3 fixed_object{10.0, 0.0, 0.0};

    const auto ego_origin = ego_at(dedalus::Vec3{0.0, 0.0, 0.0});
    const auto from_origin = dedalus::project_local_point_to_image(fixed_object, ego_origin, config);
    require(from_origin.visible, "fixed object ahead of origin should be visible");
    require_near(from_origin.u_px, config.intrinsics.cx, 1.0e-9, "origin view should be centered");

    const auto ego_left = ego_at(dedalus::Vec3{0.0, -2.0, 0.0});
    const auto from_left = dedalus::project_local_point_to_image(fixed_object, ego_left, config);
    require(from_left.visible, "fixed object should remain visible after lateral translation");
    require(from_left.position_ego_relative.x == 10.0 && from_left.position_ego_relative.y == 2.0,
            "ego-relative vector should change with ego translation");
    require(from_left.u_px > from_origin.u_px, "object should move right in viewport when ego moves left");

    // With a 90-degree horizontal FOV, 45 degrees is exactly the image edge. Use 30 degrees
    // for the inside-FOV yaw case and reserve 90 degrees for the out-of-view stress case.
    const auto ego_yawed = ego_at(dedalus::Vec3{0.0, 0.0, 0.0}, kPi / 6.0);
    const auto from_yaw = dedalus::project_local_point_to_image(fixed_object, ego_yawed, config);
    require(from_yaw.visible, "fixed object should remain visible under moderate yaw");
    require(from_yaw.u_px < from_origin.u_px, "positive yaw should move forward object left in body/camera view");
}

void stationary_object_moving_drone_stress_case() {
    const auto config = default_config();
    const dedalus::Vec3 stationary_object_local{10.0, 0.0, 0.0};

    const auto frame1 = dedalus::project_local_point_to_image(
        stationary_object_local,
        ego_at(dedalus::Vec3{0.0, 0.0, 0.0}),
        config);
    const auto frame2 = dedalus::project_local_point_to_image(
        stationary_object_local,
        ego_at(dedalus::Vec3{0.0, -3.0, 0.0}),
        config);
    const auto frame3 = dedalus::project_local_point_to_image(
        stationary_object_local,
        ego_at(dedalus::Vec3{0.0, 0.0, 0.0}, kPi / 2.0),
        config);

    require(frame1.visible, "stationary object should be visible in frame 1");
    require(frame2.visible, "stationary object should be visible in frame 2");
    require_near(frame1.position_ego_relative.x, 10.0, 1.0e-9, "frame 1 ego-relative x should be correct");
    require_near(frame2.position_ego_relative.y, 3.0, 1.0e-9, "frame 2 ego-relative y should change");
    require(frame2.u_px > frame1.u_px, "stationary object should move across viewport as drone translates");
    require(!frame3.visible, "stationary object should leave view when drone yaws 90 degrees");
    require(frame3.reason == "behind_camera" || frame3.reason == "outside_image",
            "yawed-away object should report non-visible projection reason");
}

void invalid_intrinsics_are_rejected_without_throwing() {
    auto config = default_config();
    config.intrinsics.fx = 0.0;
    const auto projection = dedalus::project_local_point_to_image(
        dedalus::Vec3{10.0, 0.0, 0.0},
        ego_at(dedalus::Vec3{0.0, 0.0, 0.0}),
        config);
    require(!projection.visible, "invalid intrinsics should not produce a visible point");
    require(projection.reason == "invalid_input", "invalid intrinsics should report invalid_input");
}

}  // namespace

int main() {
    try {
        projects_center_left_right_and_vertical_motion();
        rejects_behind_and_outside_points();
        applies_ego_translation_and_yaw();
        stationary_object_moving_drone_stress_case();
        invalid_intrinsics_are_rejected_without_throwing();
    } catch (const std::exception& exc) {
        std::cerr << "test_world_to_image_projector failed: " << exc.what() << '\n';
        return 1;
    }
    return 0;
}
