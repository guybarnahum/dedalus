#pragma once

#include <optional>
#include <string>
#include <vector>

#include "dedalus/behavior/velocity_trajectory.hpp"
#include "dedalus/core/types.hpp"

namespace dedalus {

struct GhostDetectionSpec {
    TrackId source_track_id;
    ClassLabel class_label{ClassLabel::Unknown};
    double confidence{0.0};
    Vec3 initial_position_local_m;
    Vec3 size_m;
    std::optional<std::string> trajectory_path;
    VelocityTrajectory trajectory;
};

struct GhostDetectionState {
    TrackId source_track_id;
    ClassLabel class_label{ClassLabel::Unknown};
    double confidence{0.0};
    Vec3 position_local_m;
    Vec3 velocity_local_mps;
    Vec3 size_m;
};

class GhostScenario final {
public:
    GhostScenario() = default;
    GhostScenario(std::string name, MapFrameId map_frame_id, std::vector<GhostDetectionSpec> detections);

    [[nodiscard]] static GhostScenario load_from_file(const std::string& path);
    [[nodiscard]] static GhostScenario parse_json(const std::string& text, const std::string& base_dir = ".");

    [[nodiscard]] const std::string& name() const;
    [[nodiscard]] const MapFrameId& map_frame_id() const;
    [[nodiscard]] const std::vector<GhostDetectionSpec>& detections() const;
    [[nodiscard]] bool empty() const;
    [[nodiscard]] std::vector<GhostDetectionState> evaluate(double scenario_elapsed_s) const;

private:
    std::string name_;
    MapFrameId map_frame_id_;
    std::vector<GhostDetectionSpec> detections_;
};

[[nodiscard]] std::string to_string(ClassLabel label);
[[nodiscard]] ClassLabel class_label_from_string(const std::string& value);

}  // namespace dedalus
