#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

#include "dedalus/avoidance/mission_local_obstacle_map.hpp"

namespace dedalus {

struct MissionObstacleMapDeltaWriterConfig {
    bool enabled{false};
    std::filesystem::path output_path;
    std::string site_id{"unknown_site"};
    std::string site_frame_id{"site_local"};
    std::string mission_id{"unknown_mission"};
    std::size_t write_every_updates{10U};
};

class MissionObstacleMapDeltaWriter {
public:
    explicit MissionObstacleMapDeltaWriter(MissionObstacleMapDeltaWriterConfig config = {});

    static MissionObstacleMapDeltaWriter from_environment();

    void append_if_due(const MissionLocalObstacleMapSnapshot& snapshot);

    [[nodiscard]] bool enabled() const { return config_.enabled; }

private:
    MissionObstacleMapDeltaWriterConfig config_;
    std::optional<std::uint64_t> mission_start_unix_ns_;
    std::uint64_t last_written_update_count_{0U};
    std::uint64_t last_written_snapshot_timestamp_ns_{0U};
    bool wrote_header_meta_{false};
};

}  // namespace dedalus
