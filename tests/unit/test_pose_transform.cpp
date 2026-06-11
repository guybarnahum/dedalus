#include "dedalus/geometry/pose_transform.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

using namespace dedalus;

namespace {

constexpr double kEps = 1.0e-9;
constexpr double kPi = 3.141592653589793238462643383279502884;

void assert_close(const double actual, const double expected) {
    assert(std::fabs(actual - expected) < kEps);
}

void assert_vec_close(const Vec3& actual, const Vec3& expected) {
    assert_close(actual.x, expected.x);
    assert_close(actual.y, expected.y);
    assert_close(actual.z, expected.z);
}

void yaw_pose_transforms_body_forward_into_map_y() {
    const auto map_T_body = make_yaw_pose(Vec3{10.0, 20.0, 2.0}, kPi / 2.0);

    const auto point_map = transform_point(map_T_body, Vec3{2.0, 0.0, 0.0});

    assert_vec_close(point_map, Vec3{10.0, 22.0, 2.0});
}

void inverse_transform_round_trips_point() {
    Pose3 map_T_body;
    map_T_body.position = Vec3{3.0, -4.0, 2.0};
    map_T_body.rotation_rpy = Vec3{0.1, -0.2, 0.3};

    const Vec3 point_body{5.0, 6.0, 7.0};
    const auto point_map = transform_point(map_T_body, point_body);
    const auto round_trip = inverse_transform_point(map_T_body, point_map);

    assert_vec_close(round_trip, point_body);
}

void compose_yaw_poses_matches_sequential_transform() {
    const auto map_T_a = make_yaw_pose(Vec3{1.0, 2.0, 0.0}, kPi / 2.0);
    const auto a_T_b = make_yaw_pose(Vec3{2.0, 0.0, 0.0}, kPi / 2.0);

    const auto map_T_b = compose_pose(map_T_a, a_T_b);

    const Vec3 point_b{1.0, 0.0, 0.0};
    const auto sequential = transform_point(map_T_a, transform_point(a_T_b, point_b));
    const auto composed = transform_point(map_T_b, point_b);

    assert_vec_close(composed, sequential);
    assert_close(yaw_rad(map_T_b), kPi);
}

}  // namespace

int main() {
    yaw_pose_transforms_body_forward_into_map_y();
    inverse_transform_round_trips_point();
    compose_yaw_poses_matches_sequential_transform();

    std::cout << "pose transform tests passed\n";
    return 0;
}
