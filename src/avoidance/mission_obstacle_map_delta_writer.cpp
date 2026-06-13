#include "dedalus/avoidance/mission_obstacle_map_delta_writer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

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

    const auto delta = mission_end_timestamp > timestamp ? mission_end_timestamp - timestamp : 0U;
    return mission_end_unix_ns > delta ? mission_end_unix_ns - delta : 0U;
}

bool env_enabled(const char* name) {
    const char* value = std::getenv(name);
    if (value == nullptr) {
        return false;
    }
    const std::string v{value};
    return v == "1" || v == "true" || v == "TRUE" || v == "yes" || v == "YES";
}

std::string env_string(const char* name, const std::string& fallback) {
    const char* value = std::getenv(name);
    if (value == nullptr || std::string{value}.empty()) {
        return fallback;
    }
    return std::string{value};
}

std::size_t env_size(const char* name, const std::size_t fallback) {
    const char* value = std::getenv(name);
    if (value == nullptr || std::string{value}.empty()) {
        return fallback;
    }
    try {
        const auto parsed = static_cast<std::size_t>(std::stoull(value));
        return parsed == 0U ? fallback : parsed;
    } catch (...) {
        return fallback;
    }
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

std::string source_kind_string(const OccupancySourceKind kind) {
    if (kind == OccupancySourceKind::DepthProvider) {
        return "depth_provider";
    }
    return "unknown";
}

double score_scale_for(const MissionLocalObstacleMapSnapshot& snapshot) {
    double scale = 1.0;
    for (const auto& cell : snapshot.cells) {
        scale = std::max(scale, cell.occupied_score);
        scale = std::max(scale, cell.free_score);
    }
    return scale;
}

std::uint64_t mission_start_from_snapshot(
    const MissionLocalObstacleMapSnapshot& snapshot,
    const std::uint64_t mission_end_unix_ns) {
    auto mission_start_unix_ns = mission_end_unix_ns;
    for (const auto& cell : snapshot.cells) {
        const auto first = cell.first_observed_timestamp_ns;
        if (first != 0U) {
            const auto aligned = aligned_unix_ns(
                first,
                snapshot.summary.last_update_timestamp_ns,
                mission_end_unix_ns);
            mission_start_unix_ns = std::min(mission_start_unix_ns, aligned);
        }
    }
    return mission_start_unix_ns;
}

std::string render_meta(
    const MissionObstacleMapDeltaWriterConfig& config,
    const MissionLocalObstacleMapSnapshot& snapshot,
    const std::uint64_t mission_start_unix_ns,
    const std::uint64_t mission_end_unix_ns) {
    std::ostringstream out;
    out << std::setprecision(17);
    out << "{\n";
    out << "  \"schema\": \"dedalus.mission_obstacle_map_deltas.v1.meta\",\n";
    out << "  \"time_unit\": \"unix_ns\",\n";
    out << "  \"site_id\": \"" << escape_json(config.site_id) << "\",\n";
    out << "  \"site_frame_id\": \"" << escape_json(config.site_frame_id) << "\",\n";
    out << "  \"mission_id\": \"" << escape_json(config.mission_id) << "\",\n";
    out << "  \"mission_map_frame_id\": \"" << escape_json(snapshot.summary.map_frame_id.value) << "\",\n";
    out << "  \"mission_start_unix_ns\": " << mission_start_unix_ns << ",\n";
    out << "  \"mission_end_unix_ns\": " << mission_end_unix_ns << ",\n";
    out << "  \"cell_size_m\": " << snapshot.config.cell_size_m << ",\n";
    out << "  \"vertical_cell_size_m\": " << snapshot.config.vertical_cell_size_m << ",\n";
    out << "  \"write_every_updates\": " << config.write_every_updates << ",\n";
    out << "  \"format\": \"jsonl\",\n";
    out << "  \"delta_policy\": \"append changed cells whose last observed timestamp advanced since previous batch\",\n";
    out << "  \"hot_path_note\": \"Delta writing is a persistence stream. Runtime map lookup remains in memory.\"\n";
    out << "}\n";
    return out.str();
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

void append_text(const std::filesystem::path& path, const std::string& text) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream out(path, std::ios::out | std::ios::app);
    if (!out) {
        throw std::runtime_error("failed to open delta stream path: " + path.string());
    }
    out << text;
}

std::string render_delta_batch(
    const MissionObstacleMapDeltaWriterConfig& config,
    const MissionLocalObstacleMapSnapshot& snapshot,
    const std::uint64_t mission_start_unix_ns,
    const std::uint64_t previous_snapshot_timestamp_ns) {
    const auto mission_end_unix_ns =
        static_cast<std::uint64_t>(snapshot.summary.last_update_timestamp_ns);
    const auto mission_end_timestamp_ns = snapshot.summary.last_update_timestamp_ns;
    const auto score_scale = score_scale_for(snapshot);

    std::vector<const MissionLocalObstacleCell*> changed_cells;
    changed_cells.reserve(snapshot.cells.size());

    for (const auto& cell : snapshot.cells) {
        if (!cell.observed) {
            continue;
        }
        if (previous_snapshot_timestamp_ns != 0U &&
            cell.last_observed_timestamp_ns <= previous_snapshot_timestamp_ns) {
            continue;
        }
        changed_cells.push_back(&cell);
    }

    std::ostringstream out;
    out << std::setprecision(17);
    out << "{";
    out << "\"schema\":\"dedalus.mission_obstacle_map_delta_batch.v1\",";
    out << "\"time_unit\":\"unix_ns\",";
    out << "\"site_id\":\"" << escape_json(config.site_id) << "\",";
    out << "\"site_frame_id\":\"" << escape_json(config.site_frame_id) << "\",";
    out << "\"mission_id\":\"" << escape_json(config.mission_id) << "\",";
    out << "\"mission_map_frame_id\":\"" << escape_json(snapshot.summary.map_frame_id.value) << "\",";
    out << "\"mission_start_unix_ns\":" << mission_start_unix_ns << ",";
    out << "\"batch_unix_ns\":" << mission_end_unix_ns << ",";
    out << "\"update_count\":" << snapshot.summary.update_count << ",";
    out << "\"previous_snapshot_timestamp_ns\":" << previous_snapshot_timestamp_ns << ",";
    out << "\"score_scale\":" << score_scale << ",";
    out << "\"changed_cell_count\":" << changed_cells.size() << ",";
    out << "\"cells\":[";

    for (std::size_t i = 0; i < changed_cells.size(); ++i) {
        const auto& cell = *changed_cells[i];
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
        const auto free_log_odds = log_odds_from_probability(normalized_free);

        out << "{";
        out << "\"center_mission\":";
        write_vec3(out, cell.center_map);
        out << ",\"size_m\":";
        write_vec3(out, cell.size_m);
        out << ",\"observed\":" << (cell.observed ? "true" : "false");
        out << ",\"occupied\":" << (cell.occupied ? "true" : "false");
        out << ",\"free\":" << (cell.free ? "true" : "false");
        out << ",\"occupied_score\":" << cell.occupied_score;
        out << ",\"free_score\":" << cell.free_score;
        out << ",\"normalized_occupied_score\":" << normalized_occupied;
        out << ",\"normalized_free_score\":" << normalized_free;
        out << ",\"occupied_log_odds\":" << occupied_log_odds;
        out << ",\"free_log_odds\":" << free_log_odds;
        out << ",\"risk_score\":" << cell.risk_score;
        out << ",\"confidence\":" << cell.confidence;
        out << ",\"first_seen_unix_ns\":" << first_seen;
        out << ",\"last_seen_unix_ns\":" << last_seen;
        out << ",\"last_confirmed_occupied_unix_ns\":" << (cell.occupied ? last_seen : 0U);
        out << ",\"last_observed_free_unix_ns\":" << (cell.free ? last_seen : 0U);
        out << ",\"positive_observation_count\":" << (cell.occupied ? 1 : 0);
        out << ",\"negative_observation_count\":" << (cell.free ? 1 : 0);
        out << ",\"mission_observation_count\":1";
        out << ",\"source\":{\"last_source_kind\":\""
            << escape_json(source_kind_string(cell.last_source_kind))
            << "\",\"last_source_provider\":\""
            << escape_json(cell.last_source_provider) << "\"}";
        out << "}";

        if (i + 1U != changed_cells.size()) {
            out << ",";
        }
    }

    out << "]}\n";
    return out.str();
}

}  // namespace

MissionObstacleMapDeltaWriter::MissionObstacleMapDeltaWriter(
    MissionObstacleMapDeltaWriterConfig config)
    : config_(std::move(config)) {}

MissionObstacleMapDeltaWriter MissionObstacleMapDeltaWriter::from_environment() {
    MissionObstacleMapDeltaWriterConfig config;
    config.enabled = env_enabled("DEDALUS_MISSION_OBSTACLE_MAP_DELTAS");
    config.output_path = env_string("DEDALUS_MISSION_OBSTACLE_MAP_DELTAS_PATH", "");
    config.site_id = env_string("DEDALUS_MISSION_OBSTACLE_MAP_SITE_ID", config.site_id);
    config.site_frame_id = env_string("DEDALUS_MISSION_OBSTACLE_MAP_SITE_FRAME_ID", config.site_frame_id);
    config.mission_id = env_string("DEDALUS_MISSION_OBSTACLE_MAP_MISSION_ID", config.mission_id);
    config.write_every_updates =
        env_size("DEDALUS_MISSION_OBSTACLE_MAP_DELTAS_WRITE_EVERY_UPDATES", config.write_every_updates);
    return MissionObstacleMapDeltaWriter{std::move(config)};
}

void MissionObstacleMapDeltaWriter::append_if_due(
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
        mission_start_unix_ns_ = mission_start_from_snapshot(snapshot, mission_end_unix_ns);

        std::error_code remove_error;
        std::filesystem::remove(config_.output_path, remove_error);

        wrote_header_meta_ = true;
    }

    auto meta_path = config_.output_path;
    meta_path += ".meta.json";
    atomic_write_text(
        meta_path,
        render_meta(config_, snapshot, *mission_start_unix_ns_, mission_end_unix_ns));
    wrote_header_meta_ = true;

    append_text(
        config_.output_path,
        render_delta_batch(
            config_,
            snapshot,
            *mission_start_unix_ns_,
            last_written_snapshot_timestamp_ns_));

    last_written_update_count_ = update_count;
    last_written_snapshot_timestamp_ns_ = snapshot.summary.last_update_timestamp_ns;
}

}  // namespace dedalus
