#include "dedalus/avoidance/mission_traversability_map_artifact_writer.hpp"

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

void write_number_or_null(std::ostringstream& out, const double value) {
    if (std::isfinite(value)) {
        out << value;
    } else {
        out << "null";
    }
}

std::string state_string(const TraversabilityCellState state) {
    switch (state) {
        case TraversabilityCellState::Unknown:
            return "unknown";
        case TraversabilityCellState::ObservedFree:
            return "observed_free";
        case TraversabilityCellState::Occupied:
            return "occupied";
        case TraversabilityCellState::Mixed:
            return "mixed";
        case TraversabilityCellState::Stale:
            return "stale";
    }
    return "unknown";
}

void atomic_write_text(const std::filesystem::path& path, const std::string& text) {
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }

    const auto tmp = path.string() + ".tmp";
    {
        std::ofstream out(tmp, std::ios::out | std::ios::trunc);
        if (!out) {
            throw std::runtime_error("failed to open temp traversability artifact path: " + tmp);
        }
        out << text;
    }

    std::error_code remove_error;
    std::filesystem::remove(path, remove_error);

    std::error_code rename_error;
    std::filesystem::rename(tmp, path, rename_error);
    if (rename_error) {
        throw std::runtime_error(
            "failed to rename temp traversability artifact " + tmp + " to " + path.string() +
            ": " + rename_error.message());
    }
}

MissionLocalTraversabilityMapSnapshot capped_snapshot(
    MissionLocalTraversabilityMapSnapshot snapshot,
    const std::size_t max_cells) {
    if (max_cells > 0U && snapshot.cells.size() > max_cells) {
        snapshot.cells.resize(max_cells);
    }
    return snapshot;
}

std::uint64_t first_update_timestamp_ns(const MissionLocalTraversabilityMapSnapshot& snapshot) {
    auto first = snapshot.summary.last_update_timestamp_ns;
    for (const auto& cell : snapshot.cells) {
        if (cell.first_observed_timestamp_ns != 0U) {
            first = first == 0U
                ? cell.first_observed_timestamp_ns
                : std::min(first, cell.first_observed_timestamp_ns);
        }
    }
    return first;
}

std::string render_artifact(
    const MissionTraversabilityMapArtifactWriterConfig& config,
    const MissionLocalTraversabilityMapSnapshot& full_snapshot,
    const MissionLocalTraversabilityMapSnapshot& exported_snapshot,
    const std::uint64_t first_update_ns) {
    std::ostringstream out;
    out << std::setprecision(17);
    out << "{\n";
    out << "  \"schema\": \"dedalus.mission_local_traversability_map.v1\",\n";
    out << "  \"time_unit\": \"timestamp_ns\",\n";
    out << "  \"site_id\": \"" << escape_json(config.site_id) << "\",\n";
    out << "  \"site_frame_id\": \"" << escape_json(config.site_frame_id) << "\",\n";
    out << "  \"mission_id\": \"" << escape_json(config.mission_id) << "\",\n";
    out << "  \"map_frame_id\": \"" << escape_json(full_snapshot.summary.map_frame_id.value) << "\",\n";
    out << "  \"first_update_timestamp_ns\": " << first_update_ns << ",\n";
    out << "  \"last_update_timestamp_ns\": " << full_snapshot.summary.last_update_timestamp_ns << ",\n";
    out << "  \"created_at_timestamp_ns\": " << full_snapshot.summary.last_update_timestamp_ns << ",\n";
    out << "  \"config\": {\n";
    out << "    \"cell_size_m\": " << full_snapshot.config.cell_size_m << ",\n";
    out << "    \"vertical_cell_size_m\": " << full_snapshot.config.vertical_cell_size_m << ",\n";
    out << "    \"required_clearance_m\": " << full_snapshot.config.required_clearance_m << ",\n";
    out << "    \"soft_clearance_m\": " << full_snapshot.config.soft_clearance_m << ",\n";
    out << "    \"clearance_search_radius_m\": " << full_snapshot.config.clearance_search_radius_m << "\n";
    out << "  },\n";
    out << "  \"summary\": {\n";
    out << "    \"update_count\": " << full_snapshot.summary.update_count << ",\n";
    out << "    \"source_obstacle_cell_count\": " << full_snapshot.summary.source_obstacle_cell_count << ",\n";
    out << "    \"accepted_source_cell_count\": " << full_snapshot.summary.accepted_source_cell_count << ",\n";
    out << "    \"cell_count\": " << full_snapshot.summary.cell_count << ",\n";
    out << "    \"occupied_cell_count\": " << full_snapshot.summary.occupied_cell_count << ",\n";
    out << "    \"free_cell_count\": " << full_snapshot.summary.free_cell_count << ",\n";
    out << "    \"mixed_cell_count\": " << full_snapshot.summary.mixed_cell_count << ",\n";
    out << "    \"stale_cell_count\": " << full_snapshot.summary.stale_cell_count << ",\n";
    out << "    \"low_clearance_cell_count\": " << full_snapshot.summary.low_clearance_cell_count << ",\n";
    out << "    \"overhead_risk_cell_count\": " << full_snapshot.summary.overhead_risk_cell_count << ",\n";
    out << "    \"volatile_cell_count\": " << full_snapshot.summary.volatile_cell_count << ",\n";
    out << "    \"minimum_clearance_m\": ";
    write_number_or_null(out, full_snapshot.summary.minimum_clearance_m);
    out << ",\n";
    out << "    \"minimum_vertical_clearance_up_m\": ";
    write_number_or_null(out, full_snapshot.summary.minimum_vertical_clearance_up_m);
    out << "\n";
    out << "  },\n";
    out << "  \"export_summary\": {\n";
    out << "    \"exported_cell_count\": " << exported_snapshot.cells.size() << ",\n";
    out << "    \"source_cell_count\": " << full_snapshot.cells.size() << ",\n";
    out << "    \"source_cells_are_debug_capped\": "
        << (exported_snapshot.cells.size() != full_snapshot.cells.size() ? "true" : "false") << ",\n";
    out << "    \"artifact_role\": \"persistent foundational traversability-map diagnostics; not a command-sink or reflexive safety dependency\"\n";
    out << "  },\n";
    out << "  \"cells\": [\n";

    for (std::size_t i = 0U; i < exported_snapshot.cells.size(); ++i) {
        const auto& cell = exported_snapshot.cells[i];
        out << "    {\n";
        out << "      \"center_map\": ";
        write_vec3(out, cell.center_map);
        out << ",\n";
        out << "      \"size_m\": ";
        write_vec3(out, cell.size_m);
        out << ",\n";
        out << "      \"state\": \"" << state_string(cell.state) << "\",\n";
        out << "      \"occupied_score\": " << cell.occupied_score << ",\n";
        out << "      \"free_score\": " << cell.free_score << ",\n";
        out << "      \"confidence\": " << cell.confidence << ",\n";
        out << "      \"unknown_score\": " << cell.unknown_score << ",\n";
        out << "      \"occupied_hits_capped\": " << cell.occupied_hits_capped << ",\n";
        out << "      \"free_rays_capped\": " << cell.free_rays_capped << ",\n";
        out << "      \"conflict_count_capped\": " << cell.conflict_count_capped << ",\n";
        out << "      \"refresh_count_capped\": " << cell.refresh_count_capped << ",\n";
        out << "      \"first_observed_timestamp_ns\": " << cell.first_observed_timestamp_ns << ",\n";
        out << "      \"last_observed_timestamp_ns\": " << cell.last_observed_timestamp_ns << ",\n";
        out << "      \"age_score\": " << cell.age_score << ",\n";
        out << "      \"stability_score\": " << cell.stability_score << ",\n";
        out << "      \"volatility_score\": " << cell.volatility_score << ",\n";
        out << "      \"nearest_obstacle_distance_m\": ";
        write_number_or_null(out, cell.nearest_obstacle_distance_m);
        out << ",\n";
        out << "      \"clearance_margin_m\": ";
        write_number_or_null(out, cell.clearance_margin_m);
        out << ",\n";
        out << "      \"vertical_clearance_up_m\": ";
        write_number_or_null(out, cell.vertical_clearance_up_m);
        out << ",\n";
        out << "      \"vertical_clearance_down_m\": ";
        write_number_or_null(out, cell.vertical_clearance_down_m);
        out << ",\n";
        out << "      \"costs\": {\n";
        out << "        \"occupied_cost\": " << cell.occupied_cost << ",\n";
        out << "        \"proximity_cost\": " << cell.proximity_cost << ",\n";
        out << "        \"unknown_cost\": " << cell.unknown_cost << ",\n";
        out << "        \"stale_cost\": " << cell.stale_cost << ",\n";
        out << "        \"overhead_cost\": " << cell.overhead_cost << ",\n";
        out << "        \"thin_structure_cost\": " << cell.thin_structure_cost << ",\n";
        out << "        \"total_traversability_cost\": " << cell.total_traversability_cost << "\n";
        out << "      },\n";
        out << "      \"stale\": " << (cell.stale ? "true" : "false") << "\n";
        out << "    }";
        if (i + 1U != exported_snapshot.cells.size()) {
            out << ",";
        }
        out << "\n";
    }

    out << "  ]\n";
    out << "}\n";
    return out.str();
}

std::string render_meta(
    const MissionTraversabilityMapArtifactWriterConfig& config,
    const MissionLocalTraversabilityMapSnapshot& full_snapshot,
    const MissionLocalTraversabilityMapSnapshot& exported_snapshot,
    const std::uint64_t first_update_ns) {
    std::ostringstream out;
    out << std::setprecision(17);
    out << "{\n";
    out << "  \"schema\": \"dedalus.mission_local_traversability_map.v1.meta\",\n";
    out << "  \"time_unit\": \"timestamp_ns\",\n";
    out << "  \"site_id\": \"" << escape_json(config.site_id) << "\",\n";
    out << "  \"site_frame_id\": \"" << escape_json(config.site_frame_id) << "\",\n";
    out << "  \"mission_id\": \"" << escape_json(config.mission_id) << "\",\n";
    out << "  \"map_frame_id\": \"" << escape_json(full_snapshot.summary.map_frame_id.value) << "\",\n";
    out << "  \"first_update_timestamp_ns\": " << first_update_ns << ",\n";
    out << "  \"last_update_timestamp_ns\": " << full_snapshot.summary.last_update_timestamp_ns << ",\n";
    out << "  \"update_count\": " << full_snapshot.summary.update_count << ",\n";
    out << "  \"cell_count\": " << full_snapshot.summary.cell_count << ",\n";
    out << "  \"occupied_cell_count\": " << full_snapshot.summary.occupied_cell_count << ",\n";
    out << "  \"free_cell_count\": " << full_snapshot.summary.free_cell_count << ",\n";
    out << "  \"stale_cell_count\": " << full_snapshot.summary.stale_cell_count << ",\n";
    out << "  \"exported_cell_count\": " << exported_snapshot.cells.size() << ",\n";
    out << "  \"source_cell_count\": " << full_snapshot.cells.size() << ",\n";
    out << "  \"source_cells_are_debug_capped\": "
        << (exported_snapshot.cells.size() != full_snapshot.cells.size() ? "true" : "false") << "\n";
    out << "}\n";
    return out.str();
}

}  // namespace

MissionTraversabilityMapArtifactWriter::MissionTraversabilityMapArtifactWriter(
    MissionTraversabilityMapArtifactWriterConfig config)
    : config_(std::move(config)) {}

void MissionTraversabilityMapArtifactWriter::write_if_due(
    const MissionLocalTraversabilityMapSnapshot& snapshot) {
    write_snapshot(snapshot, false);
}

void MissionTraversabilityMapArtifactWriter::write_final(
    const MissionLocalTraversabilityMapSnapshot& snapshot) {
    write_snapshot(snapshot, true);
}

void MissionTraversabilityMapArtifactWriter::write_snapshot(
    const MissionLocalTraversabilityMapSnapshot& snapshot,
    const bool force) {
    if (!config_.enabled || config_.output_path.empty()) {
        return;
    }

    const auto update_count = snapshot.summary.update_count;
    if (update_count == 0U) {
        return;
    }

    if (!force && last_written_update_count_ != 0U &&
        update_count - last_written_update_count_ < config_.write_every_updates) {
        return;
    }

    if (!first_update_timestamp_ns_.has_value()) {
        first_update_timestamp_ns_ = first_update_timestamp_ns(snapshot);
    }

    const auto exported_snapshot = capped_snapshot(snapshot, config_.max_cells);
    atomic_write_text(
        config_.output_path,
        render_artifact(config_, snapshot, exported_snapshot, *first_update_timestamp_ns_));

    auto meta_path = config_.output_path;
    meta_path += ".meta.json";
    atomic_write_text(
        meta_path,
        render_meta(config_, snapshot, exported_snapshot, *first_update_timestamp_ns_));

    last_written_update_count_ = update_count;
}

}  // namespace dedalus
