#pragma once

#include <memory>
#include <vector>

#include "dedalus/perception/types.hpp"
#include "dedalus/sensors/frame_source.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

namespace dedalus {

class Detector {
public:
    virtual ~Detector() = default;
    virtual std::vector<Detection2D> detect(const FramePacket& frame) = 0;
};

struct StabilizedFrame {
    FramePacket frame;
    std::vector<Detection2D> detections;
    bool transform_available{false};
    double dx_px{0.0};
    double dy_px{0.0};
    double rotation_rad{0.0};
    double confidence{0.0};
};

class CameraStabilizer {
public:
    virtual ~CameraStabilizer() = default;
    virtual StabilizedFrame stabilize(
        const FramePacket& frame,
        const std::vector<Detection2D>& detections) = 0;
};

class Tracker {
public:
    virtual ~Tracker() = default;
    virtual std::vector<Track2D> update(const std::vector<Detection2D>& detections) = 0;
};

class IdentityResolver {
public:
    virtual ~IdentityResolver() = default;
    virtual std::vector<IdentityHypothesis> resolve(const std::vector<Track2D>& tracks) = 0;
};

class Projector3D {
public:
    virtual ~Projector3D() = default;
    virtual std::vector<Observation3D> project(
        const std::vector<Track2D>& tracks,
        const FramePacket& frame,
        const EgoState& ego) = 0;
};

struct PerceptionPipelineOutput {
    std::vector<Detection2D> detections;
    StabilizedFrame stabilized_frame;
    std::vector<Track2D> tracks;
    std::vector<IdentityHypothesis> identities;
    std::vector<Observation3D> observations;
    std::vector<ObstacleEvidence> obstacle_evidence;
};

class PerceptionPipeline {
public:
    PerceptionPipeline(
        std::shared_ptr<Detector> detector,
        std::shared_ptr<CameraStabilizer> stabilizer,
        std::shared_ptr<Tracker> tracker,
        std::shared_ptr<IdentityResolver> identity_resolver,
        std::shared_ptr<Projector3D> projector);

    PerceptionPipelineOutput process(const FramePacket& frame, const EgoState& ego);

private:
    std::shared_ptr<Detector> detector_;
    std::shared_ptr<CameraStabilizer> stabilizer_;
    std::shared_ptr<Tracker> tracker_;
    std::shared_ptr<IdentityResolver> identity_resolver_;
    std::shared_ptr<Projector3D> projector_;
};

class ScriptedDetector final : public Detector {
public:
    std::vector<Detection2D> detect(const FramePacket& frame) override;
};

class NullDetector final : public Detector {
public:
    std::vector<Detection2D> detect(const FramePacket&) override { return {}; }
};

class NullCameraStabilizer final : public CameraStabilizer {
public:
    StabilizedFrame stabilize(
        const FramePacket& frame,
        const std::vector<Detection2D>& detections) override;
};

class SimpleCentroidTracker final : public Tracker {
public:
    std::vector<Track2D> update(const std::vector<Detection2D>& detections) override;
};

class AppearanceOnlyIdentityResolver final : public IdentityResolver {
public:
    std::vector<IdentityHypothesis> resolve(const std::vector<Track2D>& tracks) override;
};

class FlatGroundProjector final : public Projector3D {
public:
    std::vector<Observation3D> project(
        const std::vector<Track2D>& tracks,
        const FramePacket& frame,
        const EgoState& ego) override;
};

}  // namespace dedalus
