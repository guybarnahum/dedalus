#pragma once

#include <string>

#include "dedalus/perception/perception_pipeline.hpp"
#include "dedalus/sensors/frame_source.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

namespace dedalus {

struct AnnotationContext {
    FramePacket frame;
    PerceptionPipelineOutput perception;
    WorldSnapshot world_snapshot;
};

class FrameAnnotationSink {
public:
    virtual ~FrameAnnotationSink() = default;

    virtual void annotate(const AnnotationContext& context) = 0;
    virtual void finish() = 0;
};

class NullFrameAnnotationSink final : public FrameAnnotationSink {
public:
    void annotate(const AnnotationContext& context) override;
    void finish() override;
};

class Mp4FrameAnnotationSink final : public FrameAnnotationSink {
public:
    explicit Mp4FrameAnnotationSink(std::string output_path, double output_fps);

    void annotate(const AnnotationContext& context) override;
    void finish() override;

private:
    std::string output_path_;
    double output_fps_{0.0};
};

}  // namespace dedalus
