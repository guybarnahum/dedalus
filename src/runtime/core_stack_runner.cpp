#include "dedalus/runtime/core_stack_runner.hpp"

#include <chrono>
#include <future>
#include <utility>

namespace dedalus {
namespace {

using SteadyClock = std::chrono::steady_clock;

std::int64_t duration_us(const SteadyClock::time_point start) {
    return std::chrono::duration_cast<std::chrono::microseconds>(SteadyClock::now() - start).count();
}

}  // namespace

CoreStackRunner::CoreStackRunner(CoreStackProviders providers)
    : CoreStackRunner(std::move(providers), nullptr) {}

CoreStackRunner::CoreStackRunner(CoreStackProviders providers, std::unique_ptr<PipelineProfiler> timing_writer)
    : providers_(std::move(providers)), timing_writer_(std::move(timing_writer)) {
    start_prefetch();
}

CoreStackRunner::~CoreStackRunner() {
    if (prefetched_frame_.valid()) {
        try {
            (void)prefetched_frame_.get();
        } catch (...) {
            // A one-frame lookahead may still be waiting on the bridge when the
            // runner is destroyed after a bounded --max-frames run. Shutdown-time
            // speculative fetch failures should not abort a run that already
            // produced the requested frames.
        }
    }
}

std::optional<FramePacket> CoreStackRunner::fetch_next_frame() {
    return providers_.frame_source->next_frame();
}

void CoreStackRunner::start_prefetch() {
    prefetched_frame_ = std::async(std::launch::async, [this]() {
        return fetch_next_frame();
    });
}

bool CoreStackRunner::run_once() {
    auto start = SteadyClock::now();
    auto frame = prefetched_frame_.get();
    const auto frame_source_wait_duration_us = duration_us(start);

    if (!frame.has_value()) {
        providers_.frame_annotator->finish();
        return false;
    }

    // Start reading the next frame before processing this one. This overlaps the
    // AirSim/sensor wait with ego/perception/world-model work for the current frame.
    start_prefetch();

    if (timing_writer_) {
        timing_writer_->begin_frame(*frame);
        timing_writer_->record_stage("frame_source.next_frame_wait", frame_source_wait_duration_us);
        for (const auto& source_timing : frame->source_timings) {
            timing_writer_->record_stage(source_timing.name, source_timing.duration_us);
        }
    }

    start = SteadyClock::now();
    const auto ego_estimate = providers_.ego_provider->estimate(*frame);
    if (timing_writer_) {
        timing_writer_->record_stage("ego_provider.estimate", duration_us(start));
    }
    if (!ego_estimate.ego.has_value()) {
        if (timing_writer_) {
            timing_writer_->end_frame();
        }
        return false;
    }

    PerceptionPipeline pipeline(
        *providers_.detector,
        *providers_.camera_stabilizer,
        *providers_.tracker,
        *providers_.identity_resolver,
        *providers_.projector);

    start = SteadyClock::now();
    const auto perception_output = pipeline.process(*frame, *ego_estimate.ego);
    if (timing_writer_) {
        timing_writer_->record_stage("perception_pipeline.process", duration_us(start));
    }

    start = SteadyClock::now();
    providers_.world_model->update_ego(*ego_estimate.ego);
    if (timing_writer_) {
        timing_writer_->record_stage("world_model.update_ego", duration_us(start));
    }

    if (frame->appearance_condition.has_value()) {
        start = SteadyClock::now();
        providers_.world_model->update_appearance(*frame->appearance_condition);
        if (timing_writer_) {
            timing_writer_->record_stage("world_model.update_appearance", duration_us(start));
        }
    }

    start = SteadyClock::now();
    providers_.world_model->ingest(perception_output);
    if (timing_writer_) {
        timing_writer_->record_stage("world_model.ingest", duration_us(start));
    }

    start = SteadyClock::now();
    const auto snapshot_for_annotation = providers_.world_model->snapshot();
    if (timing_writer_) {
        timing_writer_->record_stage("world_model.snapshot", duration_us(start));
    }

    AnnotationContext annotation;
    annotation.frame = perception_output.stabilized_frame.frame;
    annotation.perception = perception_output;
    annotation.world_snapshot = snapshot_for_annotation;

    start = SteadyClock::now();
    providers_.frame_annotator->annotate(annotation);
    if (timing_writer_) {
        timing_writer_->record_stage("frame_annotator.annotate", duration_us(start));
        timing_writer_->end_frame();
    }

    return true;
}

WorldSnapshot CoreStackRunner::snapshot() const {
    return providers_.world_model->snapshot();
}

}  // namespace dedalus
