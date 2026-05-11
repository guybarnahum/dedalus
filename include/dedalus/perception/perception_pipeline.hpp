#pragma once

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
    std::vector<Track2D> tracks;
    std::vector<IdentityHypothesis> identities;
    std::vector<Observation3D> observations;
};

class PerceptionPipeline {
public:
    PerceptionPipeline(
        Detector& detector,
        Tracker& tracker,
        IdentityResolver& identity_resolver,
        Projector3D& projector);

    PerceptionPipelineOutput process(const FramePacket& frame, const EgoState& ego);

private:
    Detector& detector_;
    Tracker& tracker_;
    IdentityResolver& identity_resolver_;
    Projector3D& projector_;
};

class ScriptedDetector final : public Detector {
public:
    std::vector<Detection2D> detect(const FramePacket& frame) override;
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
