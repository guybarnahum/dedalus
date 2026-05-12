#include "dedalus/runtime/core_stack_runner.hpp"

namespace dedalus {

CoreStackRunner::CoreStackRunner(CoreStackProviders providers)
    : providers_(std::move(providers)) {}

bool CoreStackRunner::run_once() {
    const auto frame = providers_.frame_source->next_frame();
    if (!frame.has_value()) {
        providers_.frame_annotator->finish();
        return false;
    }

    const auto ego_estimate = providers_.ego_provider->estimate(*frame);
    if (!ego_estimate.ego.has_value()) {
        return false;
    }

    PerceptionPipeline pipeline(
        *providers_.detector,
        *providers_.camera_stabilizer,
        *providers_.tracker,
        *providers_.identity_resolver,
        *providers_.projector);

    const auto perception_output = pipeline.process(*frame, *ego_estimate.ego);

    providers_.world_model->update_ego(*ego_estimate.ego);
    if (frame->appearance_condition.has_value()) {
        providers_.world_model->update_appearance(*frame->appearance_condition);
    }
    providers_.world_model->ingest(perception_output);

    AnnotationContext annotation;
    annotation.frame = perception_output.stabilized_frame.frame;
    annotation.perception = perception_output;
    annotation.world_snapshot = providers_.world_model->snapshot();
    providers_.frame_annotator->annotate(annotation);

    return true;
}

WorldSnapshot CoreStackRunner::snapshot() const {
    return providers_.world_model->snapshot();
}

}  // namespace dedalus
