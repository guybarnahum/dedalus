#include "dedalus/avoidance/mission_traversability_map_artifact_writer.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <string>
#include <utility>

namespace dedalus {
namespace {

bool env_enabled(const char* name) {
    const auto* value = std::getenv(name);
    if (value == nullptr) {
        return false;
    }
    const std::string text{value};
    return text == "1" || text == "true" || text == "TRUE" || text == "yes" || text == "on";
}

std::string env_string(const char* name, std::string fallback = {}) {
    const auto* value = std::getenv(name);
    if (value == nullptr || std::string{value}.empty()) {
        return fallback;
    }
    return std::string{value};
}

std::size_t env_size(const char* name, const std::size_t fallback, const std::size_t minimum = 0U) {
    const auto* value = std::getenv(name);
    if (value == nullptr || std::string{value}.empty()) {
        return fallback;
    }
    try {
        return std::max<std::size_t>(minimum, static_cast<std::size_t>(std::stoull(value)));
    } catch (...) {
        return fallback;
    }
}

std::string sibling_path(const std::string& path, const std::string& filename) {
    const auto slash = path.find_last_of("/");
    if (slash == std::string::npos) {
        return filename;
    }
    return path.substr(0U, slash + 1U) + filename;
}

std::string default_traversability_output_path() {
    const auto obstacle_delta_path = env_string("DEDALUS_MISSION_OBSTACLE_MAP_DELTAS_PATH");
    if (!obstacle_delta_path.empty()) {
        return sibling_path(obstacle_delta_path, "mission_traversability_map_full.json");
    }

    const auto obstacle_full_path = env_string("DEDALUS_MISSION_OBSTACLE_MAP_PATH");
    if (!obstacle_full_path.empty()) {
        return sibling_path(obstacle_full_path, "mission_traversability_map_full.json");
    }

    return "out/mission_traversability_map_full.json";
}

}  // namespace

MissionTraversabilityMapArtifactWriter MissionTraversabilityMapArtifactWriter::from_environment() {
    MissionTraversabilityMapArtifactWriterConfig config;

    // The AirSim mission wrapper already injects the mission obstacle-map
    // environment into the tmux-launched mission loop. Treat that as the
    // canonical signal that mission-local obstacle evidence is being retained,
    // and emit the compact foundational traversability artifact as its planning
    // companion unless explicitly disabled by leaving both obstacle-map artifact
    // streams off.
    const bool explicit_traversability_artifact = env_enabled("DEDALUS_MISSION_TRAVERSABILITY_MAP_ARTIFACT");
    const bool mission_obstacle_artifacts_enabled =
        env_enabled("DEDALUS_MISSION_OBSTACLE_MAP_DELTAS") ||
        env_enabled("DEDALUS_MISSION_OBSTACLE_MAP_ARTIFACT");

    config.enabled = explicit_traversability_artifact || mission_obstacle_artifacts_enabled;
    config.output_path = env_string(
        "DEDALUS_MISSION_TRAVERSABILITY_MAP_PATH",
        default_traversability_output_path());
    config.site_id = env_string(
        "DEDALUS_MISSION_TRAVERSABILITY_MAP_SITE_ID",
        env_string("DEDALUS_MISSION_OBSTACLE_MAP_SITE_ID", "unknown_site"));
    config.site_frame_id = env_string(
        "DEDALUS_MISSION_TRAVERSABILITY_MAP_SITE_FRAME_ID",
        env_string("DEDALUS_MISSION_OBSTACLE_MAP_SITE_FRAME_ID", "site_local"));
    config.mission_id = env_string(
        "DEDALUS_MISSION_TRAVERSABILITY_MAP_MISSION_ID",
        env_string("DEDALUS_MISSION_OBSTACLE_MAP_MISSION_ID", "unknown_mission"));
    config.write_every_updates = env_size(
        "DEDALUS_MISSION_TRAVERSABILITY_MAP_WRITE_EVERY_UPDATES",
        env_size("DEDALUS_MISSION_OBSTACLE_MAP_WRITE_EVERY_UPDATES", 10U, 1U),
        1U);
    config.max_cells = env_size(
        "DEDALUS_MISSION_TRAVERSABILITY_MAP_MAX_CELLS",
        0U,
        0U);
    return MissionTraversabilityMapArtifactWriter{std::move(config)};
}

}  // namespace dedalus
