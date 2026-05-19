#pragma once

#include <future>
#include <memory>
#include <optional>

#include "dedalus/behavior/latest_world_snapshot.hpp"
#include "dedalus/runtime/pipeline_profiler.hpp"
#include "dedalus/runtime/provider_registry.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

namespace dedalus {

class CoreStackRunner {
public:
    explicit CoreStackRunner(CoreStackProviders providers);
    CoreStackRunner(CoreStackProviders providers, std::unique_ptr<PipelineProfiler> timing_writer);
    CoreStackRunner(
        CoreStackProviders providers,
        std::unique_ptr<PipelineProfiler> timing_writer,
        std::shared_ptr<LatestWorldSnapshot> latest_snapshot);
    ~CoreStackRunner();

    CoreStackRunner(const CoreStackRunner&) = delete;
    CoreStackRunner& operator=(const CoreStackRunner&) = delete;
    CoreStackRunner(CoreStackRunner&&) = delete;
    CoreStackRunner& operator=(CoreStackRunner&&) = delete;

    [[nodiscard]] bool run_once();
    [[nodiscard]] WorldSnapshot snapshot() const;

private:
    std::optional<FramePacket> fetch_next_frame();
    void start_prefetch();

    CoreStackProviders providers_;
    std::unique_ptr<PipelineProfiler> timing_writer_;
    std::shared_ptr<LatestWorldSnapshot> latest_snapshot_;
    std::future<std::optional<FramePacket>> prefetched_frame_;
    std::optional<TimePoint> ghost_scenario_start_;
};

}  // namespace dedalus
