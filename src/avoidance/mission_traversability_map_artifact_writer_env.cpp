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

}  // namespace

MissionTraversabilityMapArtifactWriter MissionTraversabilityMapArtifactWriter::from_environment() {
    MissionTraversabilityMapArtifactWriterConfig config;
    config.enabled = env_enabled("DEDALUS_MISSION_TRAVERSABILITY_MAP_ARTIFACT");
    config.output_path = env_string(
        "DEDALUS_MISSION_TRAVERSABILITY_MAP_PATH",
        "out/mission_traversability_map_full.json");
    config.site_id = env_string("DEDALUS_MISSION_TRAVERSABILITY_MAP_SITE_ID", "unknown_site");
    config.site_frame_id = env_string(
        "DEDALUS_MISSION_TRAVERSABILITY_MAP_SITE_FRAME_ID",
        "site_local");
    config.mission_id = env_string(
        "DEDALUS_MISSION_TRAVERSABILITY_MAP_MISSION_ID",
        "unknown_mission");
    config.write_every_updates = env_size(
        "DEDALUS_MISSION_TRAVERSABILITY_MAP_WRITE_EVERY_UPDATES",
        10U,
        1U);
    config.max_cells = env_size(
        "DEDALUS_MISSION_TRAVERSABILITY_MAP_MAX_CELLS",
        0U,
        0U);
    return MissionTraversabilityMapArtifactWriter{std::move(config)};
}

}  // namespace dedalus
