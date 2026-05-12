#pragma once

#include <memory>

#include "dedalus/runtime/pipeline_profiler.hpp"
#include "dedalus/runtime/provider_registry.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

namespace dedalus {

class CoreStackRunner {
public:
    explicit CoreStackRunner(CoreStackProviders providers);
    CoreStackRunner(CoreStackProviders providers, std::unique_ptr<PipelineProfiler> timing_writer);

    [[nodiscard]] bool run_once();
    [[nodiscard]] WorldSnapshot snapshot() const;

private:
    CoreStackProviders providers_;
    std::unique_ptr<PipelineProfiler> timing_writer_;
};

}  // namespace dedalus
