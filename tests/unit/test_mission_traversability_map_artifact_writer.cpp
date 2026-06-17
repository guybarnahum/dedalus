#include "dedalus/avoidance/mission_local_traversability_map.hpp"
#include "dedalus/avoidance/mission_traversability_map_artifact_writer.hpp"

#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace dedalus;

namespace {

MapFrameId map_frame(const std::string& value) {
    MapFrameId frame;
    frame.value = value;
    return frame;
}

std::string read_text(const std::filesystem::path& path) {
    std::ifstream in(path);
    assert(in.good());
    std::ostringstream out;
    out << in.rdbuf();
    return out.str();
}

std::filesystem::path unique_temp_dir() {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    auto path = std::filesystem::temp_directory_path();
    path /= "dedalus_traversability_writer_" + std::to_string(stamp);
    std::filesystem::create_directories(path);
    return path;
}

MissionLocalTraversabilityCell occupied_cell(const Vec3& center) {
    MissionLocalTraversabilityCell cell;
    cell.center_map = center;
    cell.size_m = Vec3{1.0, 1.0, 1.0};
    cell.state = TraversabilityCellState::Occupied;
    cell.occupied_score = 1.0;
    cell.confidence = 0.9;
    cell.occupied_hits_capped = 3U;
    cell.first_observed_timestamp_ns = 1000U;
    cell.last_observed_timestamp_ns = 2000U;
    cell.nearest_obstacle_distance_m = 0.0;
    cell.clearance_margin_m = -1.5;
    cell.vertical_clearance_up_m = 0.0;
    cell.vertical_clearance_down_m = 0.0;
    cell.occupied_cost = 1.0;
    cell.proximity_cost = 1.0;
    cell.total_traversability_cost = 1.0;
    return cell;
}

MissionLocalTraversabilityCell unknown_cell(const Vec3& center) {
    MissionLocalTraversabilityCell cell;
    cell.center_map = center;
    cell.size_m = Vec3{1.0, 1.0, 1.0};
    cell.state = TraversabilityCellState::Unknown;
    cell.unknown_score = 1.0;
    cell.unknown_cost = 1.0;
    return cell;
}

MissionLocalTraversabilityMapSnapshot sample_snapshot() {
    MissionLocalTraversabilityMapSnapshot snapshot;
    snapshot.config.cell_size_m = 1.0;
    snapshot.config.vertical_cell_size_m = 1.0;
    snapshot.config.required_clearance_m = 1.5;
    snapshot.config.soft_clearance_m = 3.0;
    snapshot.config.clearance_search_radius_m = 6.0;
    snapshot.summary.map_frame_id = map_frame("mission_local");
    snapshot.summary.update_count = 4U;
    snapshot.summary.last_update_timestamp_ns = 3000U;
    snapshot.summary.source_obstacle_cell_count = 2U;
    snapshot.summary.accepted_source_cell_count = 2U;
    snapshot.summary.cell_count = 2U;
    snapshot.summary.occupied_cell_count = 1U;
    snapshot.summary.low_clearance_cell_count = 1U;
    snapshot.summary.minimum_clearance_m = 0.0;
    snapshot.summary.minimum_vertical_clearance_up_m = 0.0;
    snapshot.cells.push_back(occupied_cell(Vec3{0.5, 0.5, 0.5}));
    snapshot.cells.push_back(unknown_cell(Vec3{10.5, 0.5, 0.5}));
    return snapshot;
}

void writes_full_artifact_and_meta() {
    const auto dir = unique_temp_dir();
    const auto artifact = dir / "mission_traversability_map_full.json";

    MissionTraversabilityMapArtifactWriterConfig config;
    config.enabled = true;
    config.output_path = artifact;
    config.site_id = "test_site";
    config.site_frame_id = "site_local";
    config.mission_id = "test_mission";
    config.write_every_updates = 10U;

    MissionTraversabilityMapArtifactWriter writer(config);
    writer.write_final(sample_snapshot());

    assert(std::filesystem::exists(artifact));
    assert(std::filesystem::exists(std::filesystem::path{artifact.string() + ".meta.json"}));

    const auto text = read_text(artifact);
    assert(text.find("\"schema\": \"dedalus.mission_local_traversability_map.v1\"") != std::string::npos);
    assert(text.find("\"site_id\": \"test_site\"") != std::string::npos);
    assert(text.find("\"map_frame_id\": \"mission_local\"") != std::string::npos);
    assert(text.find("\"artifact_role\": \"persistent foundational traversability-map diagnostics; not a command-sink or reflexive safety dependency\"") != std::string::npos);
    assert(text.find("\"state\": \"occupied\"") != std::string::npos);
    assert(text.find("\"state\": \"unknown\"") != std::string::npos);
    assert(text.find("\"nearest_obstacle_distance_m\": null") != std::string::npos);
    assert(text.find("inf") == std::string::npos);
    assert(text.find("nan") == std::string::npos);

    const auto meta = read_text(std::filesystem::path{artifact.string() + ".meta.json"});
    assert(meta.find("\"schema\": \"dedalus.mission_local_traversability_map.v1.meta\"") != std::string::npos);
    assert(meta.find("\"cell_count\": 2") != std::string::npos);

    std::filesystem::remove_all(dir);
}

void capped_export_preserves_full_summary() {
    const auto dir = unique_temp_dir();
    const auto artifact = dir / "mission_traversability_map_capped.json";

    MissionTraversabilityMapArtifactWriterConfig config;
    config.enabled = true;
    config.output_path = artifact;
    config.max_cells = 1U;

    MissionTraversabilityMapArtifactWriter writer(config);
    writer.write_final(sample_snapshot());

    const auto text = read_text(artifact);
    assert(text.find("\"cell_count\": 2") != std::string::npos);
    assert(text.find("\"exported_cell_count\": 1") != std::string::npos);
    assert(text.find("\"source_cell_count\": 2") != std::string::npos);
    assert(text.find("\"source_cells_are_debug_capped\": true") != std::string::npos);

    std::filesystem::remove_all(dir);
}

void disabled_writer_does_not_create_files() {
    const auto dir = unique_temp_dir();
    const auto artifact = dir / "disabled.json";

    MissionTraversabilityMapArtifactWriterConfig config;
    config.enabled = false;
    config.output_path = artifact;

    MissionTraversabilityMapArtifactWriter writer(config);
    writer.write_final(sample_snapshot());

    assert(!std::filesystem::exists(artifact));
    std::filesystem::remove_all(dir);
}

}  // namespace

int main() {
    writes_full_artifact_and_meta();
    capped_export_preserves_full_summary();
    disabled_writer_does_not_create_files();

    std::cout << "mission traversability map artifact writer tests passed\n";
    return 0;
}
