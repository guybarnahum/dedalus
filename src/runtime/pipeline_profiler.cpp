#include "dedalus/runtime/pipeline_profiler.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace dedalus {
namespace {

constexpr std::string_view kFrameSourceDetailPrefix = "frame_source.detail.";

bool is_attribution_only_stage(const std::string& name) {
    return name.rfind(std::string{kFrameSourceDetailPrefix}, 0U) == 0U;
}

std::string escape_json_string(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size());
    for (const char ch : value) {
        switch (ch) {
            case '\\':
                escaped += "\\\\";
                break;
            case '"':
                escaped += "\\\"";
                break;
            case '\n':
                escaped += "\\n";
                break;
            case '\r':
                escaped += "\\r";
                break;
            case '\t':
                escaped += "\\t";
                break;
            default:
                escaped += ch;
                break;
        }
    }
    return escaped;
}

}  // namespace

PipelineProfiler::PipelineProfiler(std::filesystem::path output_path)
    : output_path_(std::move(output_path)) {
    if (output_path_.empty()) {
        throw std::invalid_argument("pipeline profiler requires non-empty profile_output_path");
    }

    if (output_path_.has_parent_path()) {
        std::filesystem::create_directories(output_path_.parent_path());
    }

    output_.open(output_path_, std::ios::out | std::ios::trunc);
    if (!output_) {
        throw std::runtime_error("failed to open pipeline profile output: " + output_path_.string());
    }
}

PipelineProfiler::~PipelineProfiler() {
    if (frame_open_) {
        try {
            end_frame();
        } catch (...) {
            // Swallow write errors on destruction; data is best-effort here.
        }
    }
}

void PipelineProfiler::begin_frame(const FramePacket& frame) {
    if (frame_open_) {
        end_frame();
    }

    current_frame_ = PipelineFrameProfile{};
    current_frame_.frame_id = frame.frame_id.value;
    current_frame_.timestamp_ns = frame.timestamp.timestamp_ns;
    frame_open_ = true;
}

void PipelineProfiler::record_stage(std::string name, const std::int64_t duration_us) {
    if (!frame_open_) {
        return;
    }

    current_frame_.stages.push_back(PipelineStageTiming{std::move(name), duration_us});
}

void PipelineProfiler::set_measured_total(const std::int64_t duration_us) {
    if (!frame_open_) {
        return;
    }

    current_frame_.measured_total_us = duration_us;
}

void PipelineProfiler::end_frame() {
    if (!frame_open_) {
        return;
    }

    std::int64_t accounted_total_us = 0;
    for (const auto& stage : current_frame_.stages) {
        if (!is_attribution_only_stage(stage.name)) {
            accounted_total_us += stage.duration_us;
        }
    }

    const std::int64_t measured_total_us = current_frame_.measured_total_us > 0
        ? current_frame_.measured_total_us
        : accounted_total_us;
    const std::int64_t accounting_delta_us = measured_total_us - accounted_total_us;

    output_ << '{'
            << "\"frame_id\":\"" << escape_json_string(current_frame_.frame_id) << "\",";
    output_ << "\"timestamp_ns\":" << current_frame_.timestamp_ns << ',';
    output_ << "\"total_us\":" << measured_total_us << ',';
    output_ << "\"accounted_total_us\":" << accounted_total_us << ',';
    output_ << "\"accounting_delta_us\":" << accounting_delta_us << ',';
    output_ << "\"stages\":{";

    for (std::size_t i = 0; i < current_frame_.stages.size(); ++i) {
        const auto& stage = current_frame_.stages[i];
        if (i > 0U) {
            output_ << ',';
        }
        output_ << '"' << escape_json_string(stage.name) << "\":" << stage.duration_us;
    }

    output_ << "}}\n";
    output_.flush();
    if (!output_) {
        throw std::runtime_error("failed to write pipeline profile output: " + output_path_.string());
    }

    frame_open_ = false;

    // ── Rolling perf stats ────────────────────────────────────────────────────
    // Keep the last kStatsWindow frame totals in a ring buffer and print
    // FPS + latency percentiles to stderr every kStatsPrintEvery frames.
    const auto now = std::chrono::steady_clock::now();
    const std::size_t idx = stats_frame_count_ % kStatsWindow;
    if (stats_totals_us_.size() < kStatsWindow) {
        stats_totals_us_.push_back(measured_total_us);
        stats_times_.push_back(now);
    } else {
        stats_totals_us_[idx] = measured_total_us;
        stats_times_[idx]     = now;
    }
    ++stats_frame_count_;

    if (stats_frame_count_ % kStatsPrintEvery == 0U) {
        const std::size_t n = stats_totals_us_.size();

        // Percentiles over the window.
        std::vector<std::int64_t> sorted = stats_totals_us_;
        std::sort(sorted.begin(), sorted.end());
        const auto pct = [&](std::size_t p) -> long {
            return static_cast<long>(sorted[p * (n - 1U) / 100U] / 1000LL);
        };

        // FPS: (n-1) inter-frame intervals over the window.
        double fps = 0.0;
        if (n >= 2U) {
            const auto t_oldest = *std::min_element(stats_times_.begin(), stats_times_.end());
            const auto t_newest = *std::max_element(stats_times_.begin(), stats_times_.end());
            const double elapsed_s = std::chrono::duration<double>(t_newest - t_oldest).count();
            if (elapsed_s > 0.0) fps = static_cast<double>(n - 1U) / elapsed_s;
        }

        std::fprintf(stderr,
            "[PipelineStats] n=%-3zu fps=%4.2f"
            "  p50=%ldms p95=%ldms p99=%ldms"
            "  min=%ldms max=%ldms\n",
            n, fps, pct(50), pct(95), pct(99),
            static_cast<long>(sorted.front() / 1000LL),
            static_cast<long>(sorted.back()  / 1000LL));
        std::fflush(stderr);
    }
}

}  // namespace dedalus
