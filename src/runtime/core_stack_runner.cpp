#include "dedalus/runtime/core_stack_runner.hpp"

#include <utility>
#include <vector>

namespace dedalus {
namespace {

}  // namespace

CoreStackRunner::CoreStackRunner(CoreStackProviders providers, CoreStackRunnerConfig config)
    : providers_(std::move(providers)),
      timing_writer_(std::move(config.timing_writer)),
      snapshot_publisher_(std::move(config.snapshot_publisher)),
      ghost_detections_publisher_(std::move(config.ghost_detections_publisher)),
      snapshot_subscriber_handles_(std::move(config.snapshot_subscribers)),
      airsim_depth_obstacle_detector_config_(config.airsim_depth_obstacle_detector),
      sensing_coverage_provider_(providers_.obstacle_sensing_cameras) {
    if (!snapshot_subscriber_handles_.empty()) {
        if (!snapshot_publisher_) {
            snapshot_publisher_ = std::make_shared<WorldSnapshotPublisher>();
        }
        for (const auto& sub : snapshot_subscriber_handles_) {
            snapshot_publisher_->subscribe(sub);
        }
    }
}

CoreStackRunner::~CoreStackRunner() = default;

void CoreStackRunner::update_camera_pointing_states(std::vector<CameraPointingState> states) {
    camera_pointing_states_ = std::move(states);
}


WorldSnapshot CoreStackRunner::snapshot() const {
    return providers_.world_model->snapshot();
}

}  // namespace dedalus
