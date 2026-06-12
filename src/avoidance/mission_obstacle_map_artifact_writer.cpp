#include "dedalus/avoidance/mission_obstacle_map_artifact_writer.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

namespace dedalus {
namespace {

constexpr std::uint64_t kUnixNsYear2000 = 946684800000000000ULL;

bool looks_like_unix_ns(const std::uint64_t value) {
    return value >= kUnixNsYear2000;
}

std::uint64_t aligned_unix_ns(
    const std::uint64_t timestamp,
    const std::uint64_t mission_end_timestamp,
    const std::uint64_t mission_end_unix_ns) {
    if (timestamp == 0U) {
        return mission_end_unix_ns;
    }
    if (looks_like_unix_ns(timestamp)) {
        return timestamp;
    }
    if (mission_end_timestamp == 0U) {
        return mission_end_unix_ns;
    }

    const auto delta = mission_end_timestamp > timestamp
        ? mission_end_timestamp - timestamp
        : 0U;
    return mission_end_unix_ns > delta ? mission_end_unix_ns - delta : 0U;
}

double normalized_score(const double raw_score, const double scale) {
    if (scale <= 0.0 || !std::isfinite(raw_score)) {
        return 0.0;
    }
    return std::clamp(raw_score / scale, 0.0, 1.0);
}

double log_odds_from_probability(const double probability) {
    const auto p = std::clamp(probability, 0.001, 0.999);
    return std::log(p / (1.0 - p));
}

std::string escape_json(const std::string& value) {
    std::ostringstream out;
    for (const auto ch : value) {
        switch (ch) {
            case '\\':
                out << "\\\\";
                break;
            case '"':
                out << "\\\"";
                break;
            case '\n':
                out << "\\n";
                break;
            case '\r':
                out << "\\r";
                break;
            case '\t':
                out << "\\t";
                break;
            default:
                if (static_cast<unsigned char>(ch) < 0x20U) {
                    out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                        << static_cast<int>(static_cast<unsigned char>(ch))
                        << std::dec << std::setfill(' ');
                } else {
                    out << ch;
                }
        }
    }
    return out.str();
}

void write_vec3(std::ostringstream& out, const Vec3& value) {
    out << "{\"x\":" << value.x
        << ",\"y\":" << value.y
        << ",\"z\":" << value.z
        << "}";
}

std::string map_frame_value(const MapFrameId& frame_id) {
    return frame_id.value;
}

std::string source_kind_string(const OccupancySourceKind kind) {
    if (kind == OccupancySourceKind::DepthProvider) {
        return "depth_provider";
    }
    return "unknown";
}

std::string status_for(const MissionLocalObstacleCell& cell) {
    if (cell.occupied) {
        return "active";
    }
    if (cell.free) {
        return "free";
    }
    if (cell.observed) {
        return "unknown";
    }
    return "unobserved";
}

double score_scale_for(const MissionLocalObstacleMapSnapshot& snapshot) {
    double scale = 1.0;
    for (const auto& cell : snapshot.cells) {
        scale = std::max(scale, cell.occupied_score);
        scale = std::max(scale, cell.free_score);
    }
    return scale;
}

void atomic_write_text(const std::filesystem::path& path, const std::string& text) {
    std::filesystem::create_directories(path.parent_path());

    const auto tmp = path.string() + ".tmp";
    {
        std::ofstream out(tmp, std::ios::out | std::ios::trunc);
        if (!out) {
            throw std::runtime_error("failed to open temp artifact path: " + tmp);
        }
        out << text;
    }

    std::error_code remove_error;
    std::filesystem::remove(path, remove_error);

    std::error_code rename_error;
    std::filesystem::rename(tmp, path, rename_error);
    if (rename_error) {
        throw std::runtime_error(
            "failed to rename temp artifact " + tmp + " to " + path.string() +
            ": " + rename_error.message());
    }
}

std::string render_artifact(
    const MissionObstacleMapArtifactWriterConfig& config,
    const MissionLocalObstacleMapSnapshot& snapshot,
    const std::uint64_t mission_start_unix_ns,
    const std::uint64_t mission_end_unix_ns) {
    const auto score_scale = score_scale_for(snapshot);
    const auto mission_end_timestamp_ns = snapshot.summary.last_update_timestamp_ns;

    std::ostringstream out;
    out << std::setprecision(17);
    out << "{\n";
    out << "  \"schema\": \"dedalus.mission_obstacle_map.v1\",\n";
    out << "  \"time_unit\": \"unix_ns\",\n";
    out << "  \"site_id\": \"" << escape_json(config.site_id) << "\",\n";
    out << "  \"site_frame_id\": \"" << escape_json(config.site_frame_id) << "\",\n";
    out << "  \"mission_id\": \"" << escape_json(config.mission_id) << "\",\n";
    out << "  \"mission_map_frame_id\": \"" << escape_json(map_frame_value(snapshot.summary.map_frame_id)) << "\",\n";
    out << "  \"site_T_mission\": {\"position\":{\"x\":0,\"y\":0,\"z\":0},\"rotation_rpy\":{\"x\":0,\"y\":0,\"z\":0}},\n";
    out << "  \"mission_start_unix_ns\": " << mission_start_unix_ns << ",\n";
    out << "  \"mission_end_unix_ns\": " << mission_end_unix_ns << ",\n";
    out << "  \"created_at_unix_ns\": " << mission_end_unix_ns << ",\n";
    out << "  \"cell_size_m\": " << snapshot.config.cell_size_m << ",\n";
    out << "  \"vertical_cell_size_m\": " << snapshot.config.vertical_cell_size_m << ",\n";
    out << "  \"score_scale\": " << score_scale << ",\n";
    out << "  \"mission_summary\": {\n";
    out << "    \"observed_cell_count\": " << snapshot.summary.observed_cell_count << ",\n";
    out << "    \"occupied_cell_count\": " << snapshot.summary.occupied_cell_count << ",\n";
    out << "    \"free_cell_count\": " << snapshot.summary.free_cell_count << ",\n";
    out << "    \"update_count\": " << snapshot.summary.update_count << ",\n";
    out << "    \"last_update_timestamp_ns\": " << snapshot.summary.last_update_timestamp_ns << "\n";
    out << "  },\n";
    out << "  \"export_summary\": {\n";
    out << "    \"exported_cell_count\": " << snapshot.cells.size() << ",\n";
    out << "    \"source_cells_are_debug_capped\": false,\n";
    out << "    \"coverage_note\": \"This artifact was written directly from the full runtime MissionLocalObstacleMapSnapshot.\"\n";
    out << "  },\n";
    out << "  \"decay_policy_note\": {\n";
    out << "    \"calendar_age_is_not_erasure\": true,\n";
    out << "    \"recommended_relative_gap_seconds\": \"max(0, cell_age_seconds - site_staleness_seconds)\",\n";
    out << "    \"strong_decay_requires\": \"contradiction or revisits without reconfirmation\"\n";
    out << "  },\n";
    out << "  \"cells\": [\n";

    for (std::size_t i = 0; i < snapshot.cells.size(); ++i) {
        const auto& cell = snapshot.cells[i];

        const auto first_seen = aligned_unix_ns(
            cell.first_observed_timestamp_ns,
            mission_end_timestamp_ns,
            mission_end_unix_ns);
        const auto last_seen = aligned_unix_ns(
            cell.last_observed_timestamp_ns,
            mission_end_timestamp_ns,
            mission_end_unix_ns);

        const auto normalized_occupied = normalized_score(cell.occupied_score, score_scale);
        const auto normalized_free = normalized_score(cell.free_score, score_scale);
        const auto occupied_log_odds = log_odds_from_probability(normalized_occupied);

        out << "    {\n";
        out << "      \"center_mission\": ";
        write_vec3(out, cell.center_map);
        out << ",\n";
        out << "      \"size_m\": ";
        write_vec3(out, cell.size_m);
        out << ",\n";
        out << "      \"observed\": " << (cell.observed ? "true" : "false") << ",\n";
        out << "      \"occupied\": " << (cell.occupied ? "true" : "false") << ",\n";
        out << "      \"free\": " << (cell.free ? "true" : "false") << ",\n";
        out << "      \"occupied_score\": " << cell.occupied_score << ",\n";
        out << "      \"free_score\": " << cell.free_score << ",\n";
        out << "      \"normalized_occupied_score\": " << normalized_occupied << ",\n";
        out << "      \"normalized_free_score\": " << normalized_free << ",\n";
        out << "      \"score_scale\": " << score_scale << ",\n";
        out << "      \"risk_score\": " << cell.risk_score << ",\n";
        out << "      \"confidence\": " << cell.confidence << ",\n";
        out << "      \"occupied_log_odds\": " << occupied_log_odds << ",\n";
        out << "      \"first_seen_unix_ns\": " << first_seen << ",\n";
        out << "      \"last_seen_unix_ns\": " << last_seen << ",\n";
        out << "      \"last_confirmed_occupied_unix_ns\": " << (cell.occupied ? last_seen : 0U) << ",\n";
        out << "      \"last_observed_free_unix_ns\": " << (cell.free ? last_seen : 0U) << ",\n";
        out << "      \"last_in_sensor_frustum_unix_ns\": " << last_seen << ",\n";
        out << "      \"positive_observation_count\": " << (cell.occupied ? 1 : 0) << ",\n";
        out << "      \"negative_observation_count\": " << (cell.free ? 1 : 0) << ",\n";
        out << "      \"mission_observation_count\": 1,\n";
        out << "      \"source\": {\n";
        out << "        \"last_source_kind\": \"" << escape_json(source_kind_string(cell.last_source_kind)) << "\",\n";
        out << "        \"last_source_provider\": \"" << escape_json(cell.last_source_provider) << "\"\n";
        out << "      },\n";
        out << "      \"derived\": {\n";
        out << "        \"persistent_score\": " << normalized_occupied << ",\n";
        out << "        \"freshness_score\": 1,\n";
        out << "        \"active_score\": " << normalized_occupied << ",\n";
        out << "        \"status\": \"" << status_for(cell) << "\"\n";
        out << "      }\n";
        out << "    }";
        if (i + 1U != snapshot.cells.size()) {
            out << ",";
        }
        out << "\n";
    }

    out << "  ]\n";
    out << "}\n";
    return out.str();
}

std::string render_meta(
    const MissionObstacleMapArtifactWriterConfig& config,
    const MissionLocalObstacleMapSnapshot& snapshot,
    const std::uint64_t mission_start_unix_ns,
    const std::uint64_t mission_end_unix_ns) {
    std::ostringstream out;
    out << std::setprecision(17);
    out << "{\n";
    out << "  \"schema\": \"dedalus.mission_obstacle_map.v1.meta\",\n";
    out << "  \"time_unit\": \"unix_ns\",\n";
    out << "  \"site_id\": \"" << escape_json(config.site_id) << "\",\n";
    out << "  \"site_frame_id\": \"" << escape_json(config.site_frame_id) << "\",\n";
    out << "  \"mission_id\": \"" << escape_json(config.mission_id) << "\",\n";
    out << "  \"mission_map_frame_id\": \"" << escape_json(map_frame_value(snapshot.summary.map_frame_id)) << "\",\n";
    out << "  \"mission_start_unix_ns\": " << mission_start_unix_ns << ",\n";
    out << "  \"mission_end_unix_ns\": " << mission_end_unix_ns << ",\n";
    out << "  \"cell_size_m\": " << snapshot.config.cell_size_m << ",\n";
    out << "  \"vertical_cell_size_m\": " << snapshot.config.vertical_cell_size_m << ",\n";
    out << "  \"score_scale\": " << score_scale_for(snapshot) << ",\n";
    out << "  \"observed_cell_count\": " << snapshot.summary.observed_cell_count << ",\n";
    out << "  \"occupied_cell_count\": " << snapshot.summary.occupied_cell_count << ",\n";
    out << "  \"free_cell_count\": " << snapshot.summary.free_cell_count << ",\n";
    out << "  \"exported_cell_count\": " << snapshot.cells.size() << ",\n";
    out << "  \"source_cells_are_debug_capped\": false\n";
    out << "}\n";
    return out.str();
}

}  // namespace

MissionObstacleMapArtifactWriter::MissionObstacleMapArtifactWriter(
    MissionObstacleMapArtifactWriterConfig config)
    : config_(std::move(config)) {}

void MissionObstacleMapArtifactWriter::write_if_due(
    const MissionLocalObstacleMapSnapshot& snapshot) {
    if (!config_.enabled || config_.output_path.empty()) {
        return;
    }

    const auto update_count = snapshot.summary.update_count;
    if (update_count == 0U) {
        return;
    }

    if (last_written_update_count_ != 0U &&
        update_count - last_written_update_count_ < config_.write_every_updates) {
        return;
    }

    const auto mission_end_unix_ns =
        static_cast<std::uint64_t>(snapshot.summary.last_update_timestamp_ns);

    if (!mission_start_unix_ns_.has_value()) {
        mission_start_unix_ns_ = mission_end_unix_ns;
        for (const auto& cell : snapshot.cells) {
            const auto first = cell.first_observed_timestamp_ns;
            if (first != 0U) {
                const auto aligned = aligned_unix_ns(
                    first,
                    snapshot.summary.last_update_timestamp_ns,
                    mission_end_unix_ns);
                mission_start_unix_ns_ = std::min(*mission_start_unix_ns_, aligned);
            }
        }
    }

    atomic_write_text(
        config_.output_path,
        render_artifact(config_, snapshot, *mission_start_unix_ns_, mission_end_unix_ns));

    auto meta_path = config_.output_path;
    meta_path += ".meta.json";
    atomic_write_text(
        meta_path,
        render_meta(config_, snapshot, *mission_start_unix_ns_, mission_end_unix_ns));

    last_written_update_count_ = update_count;
}

}  // namespace dedalus
