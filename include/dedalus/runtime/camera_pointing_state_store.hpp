#pragma once

#include <optional>
#include <string>
#include <vector>

#include "dedalus/behavior/mission_controller.hpp"
#include "dedalus/core/types.hpp"
#include "dedalus/sensing/sensing_coverage.hpp"

namespace dedalus {

class CameraPointingStateStore {
public:
    CameraPointingStateStore() = default;

    void apply(const CameraPointingCommand& command);
    void clear();

    [[nodiscard]] std::optional<CameraPointingState> state_for_camera(const CameraId& camera_id) const;
    [[nodiscard]] std::optional<CameraPointingState> state_for_camera_name(const std::string& camera_name) const;
    [[nodiscard]] std::vector<CameraPointingState> states() const;

private:
    std::vector<CameraPointingState> states_;
};

[[nodiscard]] CameraPointingState camera_pointing_state_from_command(
    const CameraPointingCommand& command,
    const std::string& camera_name);

}  // namespace dedalus
