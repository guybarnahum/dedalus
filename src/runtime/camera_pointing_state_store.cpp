#include "dedalus/runtime/camera_pointing_state_store.hpp"

#include <algorithm>
#include <utility>

namespace dedalus {
namespace {

bool camera_name_matches(const CameraPointingState& state, const std::string& camera_name) {
    return state.camera_name == camera_name || state.camera_id.value == camera_name;
}

}  // namespace

CameraPointingState camera_pointing_state_from_command(
    const CameraPointingCommand& command,
    const std::string& camera_name) {
    CameraPointingState state;
    state.camera_id = CameraId{camera_name};
    state.camera_name = camera_name;
    state.timestamp = command.timestamp;
    state.pitch_rad = command.pitch_rad;
    state.yaw_rad = command.yaw_rad;
    state.roll_rad = 0.0;
    state.valid = command.pitch_valid || command.yaw_valid;
    state.measured = false;
    state.source = "camera_pointing_intent";
    return state;
}

void CameraPointingStateStore::apply(const CameraPointingCommand& command) {
    for (const auto& camera_name : command.cameras) {
        if (camera_name.empty()) {
            continue;
        }
        auto state = camera_pointing_state_from_command(command, camera_name);
        auto existing = std::find_if(states_.begin(), states_.end(), [&](const CameraPointingState& candidate) {
            return camera_name_matches(candidate, camera_name);
        });
        if (existing == states_.end()) {
            states_.push_back(std::move(state));
        } else {
            *existing = std::move(state);
        }
    }
}

void CameraPointingStateStore::clear() {
    states_.clear();
}

std::optional<CameraPointingState> CameraPointingStateStore::state_for_camera(const CameraId& camera_id) const {
    return state_for_camera_name(camera_id.value);
}

std::optional<CameraPointingState> CameraPointingStateStore::state_for_camera_name(const std::string& camera_name) const {
    const auto found = std::find_if(states_.begin(), states_.end(), [&](const CameraPointingState& candidate) {
        return camera_name_matches(candidate, camera_name);
    });
    if (found == states_.end()) {
        return std::nullopt;
    }
    return *found;
}

std::vector<CameraPointingState> CameraPointingStateStore::states() const {
    return states_;
}

}  // namespace dedalus
