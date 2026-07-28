#pragma once

#include <chrono>
#include <cstdint>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "dedalus/sensors/frame_source.hpp"

namespace dedalus {

struct PipelineStageTiming {
    std::string name;
    std::int64_t value{0};
    // Unit of `value`, set by the record_stage() caller — never inferred or
    // guessed downstream. "us" (the default) means value is a real elapsed-time
    // measurement in microseconds; any other unit (e.g. "cells", "mm", "batches")
    // means value is a repurposed non-duration reading. The printer and JSONL
    // writer use this to avoid dividing by 1000 and appending "ms" to a count,
    // which silently misrepresents the value (e.g. a raw count of 254 vs a
    // duration of 254ms look identical in the default stderr output), and to let
    // any future unit pass through without new code here.
    std::string unit{"us"};
};

struct PipelineFrameProfile {
    std::string frame_id;
    std::int64_t timestamp_ns{0};
    std::int64_t measured_total_us{0};
    std::vector<PipelineStageTiming> stages;
};

class PipelineProfiler {
public:
    // frame_budget_us: per-frame latency budget in microseconds, used as the
    // slow-frame diagnostic threshold.  Derive from tick rate:
    //   frame_budget_us = 1'000'000 / mission_tick_hz
    explicit PipelineProfiler(std::filesystem::path output_path,
                              std::int64_t frame_budget_us = 100'000LL);
    // Destructor flushes any in-progress frame so that partial data is
    // committed on normal object destruction (RAII flush guarantee).
    ~PipelineProfiler();

    void begin_frame(const FramePacket& frame);
    // Convenience overload for the common case: a real microsecond duration.
    void record_stage(std::string name, std::int64_t duration_us);
    // unit: caller-supplied unit of `value` (e.g. "us", "cells", "mm", "batches").
    // Anything other than "us" is printed and summarized without unit conversion
    // or being folded into the frame's accounted time budget.
    void record_stage(std::string name, std::int64_t value, std::string unit);
    void set_measured_total(std::int64_t duration_us);
    // Portion of measured_total_us spent blocked waiting on an external,
    // out-of-process frame source (e.g. AirSim rendering + network transfer)
    // rather than in our own pipeline compute. The slow-frame diagnostic
    // gates on (measured_total_us - external_wait_us), not raw wall time, so
    // a frame that's merely waiting on a slow external producer — bounded by
    // that producer's own cadence, not by anything our pipeline controls —
    // doesn't trip the alarm. Defaults to 0 (no adjustment) if never called.
    void set_external_wait_us(std::int64_t duration_us);
    void end_frame();

private:
    std::filesystem::path output_path_;
    std::ofstream output_;
    PipelineFrameProfile current_frame_;
    bool frame_open_{false};
    std::int64_t external_wait_us_{0};

    std::int64_t frame_budget_us_;  // slow-frame diagnostic threshold, derived from tick_hz

    // Rolling perf stats — printed to stderr every kStatsPrintEvery frames.
    // Window covers the last kStatsWindow samples.
    static constexpr std::size_t kStatsWindow{60U};
    static constexpr std::size_t kStatsPrintEvery{30U};
    std::vector<std::int64_t>                          stats_totals_us_;
    std::vector<std::chrono::steady_clock::time_point> stats_times_;
    std::size_t stats_frame_count_{0U};
};

}  // namespace dedalus
