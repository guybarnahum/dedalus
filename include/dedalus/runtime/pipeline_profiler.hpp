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
    std::int64_t duration_us{0};
};

struct PipelineFrameProfile {
    std::string frame_id;
    std::int64_t timestamp_ns{0};
    std::int64_t measured_total_us{0};
    std::vector<PipelineStageTiming> stages;
};

class PipelineProfiler {
public:
    explicit PipelineProfiler(std::filesystem::path output_path);
    // Destructor flushes any in-progress frame so that partial data is
    // committed on normal object destruction (RAII flush guarantee).
    ~PipelineProfiler();

    void begin_frame(const FramePacket& frame);
    void record_stage(std::string name, std::int64_t duration_us);
    void set_measured_total(std::int64_t duration_us);
    void end_frame();

private:
    std::filesystem::path output_path_;
    std::ofstream output_;
    PipelineFrameProfile current_frame_;
    bool frame_open_{false};

    // Rolling perf stats — printed to stderr every kStatsPrintEvery frames.
    // Window covers the last kStatsWindow samples.
    static constexpr std::size_t kStatsWindow{60U};
    static constexpr std::size_t kStatsPrintEvery{30U};
    std::vector<std::int64_t>                          stats_totals_us_;
    std::vector<std::chrono::steady_clock::time_point> stats_times_;
    std::size_t stats_frame_count_{0U};
};

}  // namespace dedalus
