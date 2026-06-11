#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

#include "dedalus/avoidance/mission_local_obstacle_map.hpp"

namespace dedalus {

struct MissionObstacleMapArtifactWriterConfig {
    bool enabled{false};
    std::filesystem::path output_path;
    std::string site_id{"unknown_site"};
    std::string site_frame_id{"site_local"};
    std::string mission_id{"unknown_mission"};
    std::size_t write_every_updates{10U};
};

class MissionObstacleMapArtifactWriter {
public:
    explicit MissionObstacleMapArtifactWriter(MissionObstacleMapArtifactWriterConfig config = {});

    static MissionObstacleMapArtifactWriter from_environment();

    void write_if_due(const MissionLocalObstacleMapSnapshot& snapshot);

    [[nodiscard]] bool enabled() const { return config_.enabled; }

private:
    MissionObstacleMapArtifactWriterConfig config_;
    std::optional<std::uint64_t> mission_start_unix_ns_;
    std::uint64_t last_written_update_count_{0U};
};

}  // namespace dedalus
