#include <cmath>
#include <iostream>
#include <vector>

#include "dedalus/runtime/camera_pointing_state_store.hpp"
#include "dedalus/sensing/sensing_coverage.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTolerance = 1.0e-9;

bool near(double lhs, double rhs) {
    return std::abs(lhs - rhs) <= kTolerance;
}

dedalus::CameraSensingConfig front_config() {
    dedalus::CameraSensingConfig config;
    config.camera_id = dedalus::CameraId{"front_center"};
    config.camera_name = "front_center";
    config.horizontal_fov_rad = kPi / 2.0;
    config.vertical_fov_rad = kPi / 3.0;
    return config;
}

dedalus::FramePacket frame() {
    dedalus::FramePacket packet;
    packet.frame_id = dedalus::FrameId{"frame_0001"};
    packet.timestamp = dedalus::TimePoint{2000000000};
    packet.camera_id = dedalus::CameraId{"front_center"};
    return packet;
}

dedalus::EgoState ego() {
    dedalus::EgoState state;
    state.timestamp = dedalus::TimePoint{2000000000};
    state.map_frame_id = dedalus::MapFrameId{"map_local_0001"};
    state.local_T_body.position = dedalus::Vec3{0.0, 0.0, -10.0};
    return state;
}

bool validate_multi_camera_update() {
    dedalus::CameraPointingCommand command;
    command.timestamp = dedalus::TimePoint{1000000000};
    command.cameras = {"front_center", "0"};
    command.pitch_rad = 0.25;
    command.pitch_valid = true;

    dedalus::CameraPointingStateStore store;
    store.apply(command);

    const auto states = store.states();
    if (states.size() != 2U) {
        std::cerr << "state store should create one state per command camera\n";
        return false;
    }
    const auto front = store.state_for_camera_name("front_center");
    const auto numeric = store.state_for_camera(dedalus::CameraId{"0"});
    if (!front.has_value() || !numeric.has_value()) {
        std::cerr << "state store did not expose states by camera name/id\n";
        return false;
    }
    if (!front->valid || front->measured || front->source != "camera_pointing_intent" ||
        !near(front->pitch_rad, 0.25) || front->timestamp.timestamp_ns != 1000000000) {
        std::cerr << "front camera pointing state did not preserve command fields\n";
        return false;
    }
    if (!numeric->valid || !near(numeric->pitch_rad, 0.25)) {
        std::cerr << "numeric camera pointing state did not preserve command fields\n";
        return false;
    }

    return true;
}

bool validate_replacement_and_invalid_state() {
    dedalus::CameraPointingStateStore store;

    dedalus::CameraPointingCommand first;
    first.timestamp = dedalus::TimePoint{1000000000};
    first.cameras = {"front_center"};
    first.pitch_rad = 0.1;
    first.pitch_valid = true;
    store.apply(first);

    dedalus::CameraPointingCommand second;
    second.timestamp = dedalus::TimePoint{1500000000};
    second.cameras = {"front_center"};
    second.pitch_rad = -0.35;
    second.pitch_valid = false;
    store.apply(second);

    const auto states = store.states();
    if (states.size() != 1U) {
        std::cerr << "state store should replace existing camera state instead of duplicating it\n";
        return false;
    }
    const auto front = store.state_for_camera_name("front_center");
    if (!front.has_value() || front->valid || !near(front->pitch_rad, -0.35) ||
        front->timestamp.timestamp_ns != 1500000000) {
        std::cerr << "state store did not replace front camera with latest invalid state\n";
        return false;
    }

    store.clear();
    if (!store.states().empty()) {
        std::cerr << "state store clear did not remove states\n";
        return false;
    }

    return true;
}

bool validate_sensing_coverage_handoff() {
    dedalus::CameraPointingCommand command;
    command.timestamp = dedalus::TimePoint{1000000000};
    command.cameras = {"front_center"};
    command.pitch_rad = kPi / 4.0;
    command.pitch_valid = true;

    dedalus::CameraPointingStateStore store;
    store.apply(command);

    dedalus::SensingCoverageProvider provider{{front_config()}};
    const auto volume = provider.volume_for_frame(frame(), ego(), store.states());
    if (!volume.pointing_state.valid || volume.pointing_state.source != "camera_pointing_intent") {
        std::cerr << "sensing coverage did not receive valid camera pointing state\n";
        return false;
    }
    if (!near(volume.pointing_state.pitch_rad, kPi / 4.0)) {
        std::cerr << "sensing coverage did not receive commanded camera pitch\n";
        return false;
    }
    const auto forward_down = dedalus::query_point_in_camera_sensing_volume(volume, dedalus::Vec3{10.0, 0.0, -20.0});
    if (!forward_down.inside) {
        std::cerr << "sensing coverage volume did not use state-store camera pitch\n";
        return false;
    }

    return true;
}

}  // namespace

int main() {
    if (!validate_multi_camera_update()) return 1;
    if (!validate_replacement_and_invalid_state()) return 1;
    if (!validate_sensing_coverage_handoff()) return 1;
    return 0;
}
