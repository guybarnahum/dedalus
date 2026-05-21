#pragma once

#include <string>
#include <vector>

#include "dedalus/perception/perception_pipeline.hpp"
#include "dedalus/runtime/pubsub.hpp"
#include "dedalus/simulation/ghost_scenario.hpp"

namespace dedalus {

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
    GhostScenario scenario_;
};

[[nodiscard]] std::string to_json(const GhostDetectionsFrame& frame);

}  // namespace dedalus
