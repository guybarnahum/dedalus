#pragma once

#include <cstdint>
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

    void begin_frame(const FramePacket& frame);
    void record_stage(std::string name, std::int64_t duration_us);
    void set_measured_total(std::int64_t duration_us);
    void end_frame();

private:
    std::filesystem::path output_path_;
    std::ofstream output_;
    PipelineFrameProfile current_frame_;
    bool frame_open_{false};
};

}  // namespace dedalus
