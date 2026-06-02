#pragma once

#include <memory>
#include <string>
#include <vector>

#include "dedalus/perception/perception_pipeline.hpp"
#include "dedalus/runtime/pubsub.hpp"
#include "dedalus/simulation/bridge_transport.hpp"
#include "dedalus/simulation/ghost_scenario.hpp"

namespace dedalus {

struct AirSimGhostObjectBinding {
    TrackId source_track_id;
    std::string airsim_object_name;
    ClassLabel class_label{ClassLabel::Unknown};
    double confidence{1.0};
    Vec3 size_m{1.0, 1.0, 1.0};
};

struct AirSimGhostObjectPatternBinding {
    std::string source_track_prefix{"gt_obstacle"};
    std::string airsim_object_pattern;
    ClassLabel class_label{ClassLabel::Obstacle};
    double confidence{0.70};
    Vec3 size_m{1.0, 1.0, 1.0};
    int max_matches{64};
};

struct AirSimGhostObjectSourceConfig {
    std::string host{"127.0.0.1"};
    int rpc_port{41451};
    std::string bridge_command{"python3 simulation/airsim/scripts/airsim-object-poses.py"};
    std::string bridge_transport{"pipe"};
    std::vector<AirSimGhostObjectBinding> objects;
    std::vector<AirSimGhostObjectPatternBinding> patterns;
};

struct GhostDetectionsFrame {
    TimePoint timestamp;
    MapFrameId map_frame_id;
    double scenario_elapsed_s{0.0};
    std::vector<GhostDetectionState> detections;
    std::vector<Observation3D> observations;
};

class GhostDetectionsSubscriber : public EventSubscriber<GhostDetectionsFrame> {
public:
    ~GhostDetectionsSubscriber() override = default;

    void on_event(const GhostDetectionsFrame& frame) final {
        on_ghost_detections(frame);
    }

    virtual void on_ghost_detections(const GhostDetectionsFrame& frame) = 0;
};

using GhostDetectionsPublisher = EventPublisher<GhostDetectionsFrame>;

class GhostTargetProvider {
public:
    explicit GhostTargetProvider(GhostScenario scenario);
    explicit GhostTargetProvider(AirSimGhostObjectSourceConfig config);
    ~GhostTargetProvider();

    GhostTargetProvider(const GhostTargetProvider&) = delete;
    GhostTargetProvider& operator=(const GhostTargetProvider&) = delete;
    GhostTargetProvider(GhostTargetProvider&&) noexcept;
    GhostTargetProvider& operator=(GhostTargetProvider&&) noexcept;

    GhostDetectionsFrame frame_at(
        TimePoint timestamp,
        MapFrameId map_frame_id,
        TimePoint scenario_start = TimePoint{0}) const;

    std::vector<Observation3D> observations_at(
        TimePoint timestamp,
        MapFrameId map_frame_id,
        TimePoint scenario_start = TimePoint{0}) const;

    PerceptionPipelineOutput output_at(
        TimePoint timestamp,
        MapFrameId map_frame_id,
        TimePoint scenario_start = TimePoint{0}) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

[[nodiscard]] std::string to_json(const GhostDetectionsFrame& frame);

}  // namespace dedalus
