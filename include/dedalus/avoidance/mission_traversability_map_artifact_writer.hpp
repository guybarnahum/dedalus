#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

#include "dedalus/avoidance/mission_local_traversability_map.hpp"

namespace dedalus {

struct MissionTraversabilityMapArtifactWriterConfig {
    bool enabled{false};
    std::filesystem::path output_path;
    std::string site_id{"unknown_site"};
    std::string site_frame_id{"site_local"};
    std::string mission_id{"unknown_mission"};
    std::size_t write_every_updates{10U};
    std::size_t max_cells{0U};
};

class MissionTraversabilityMapArtifactWriter {
public:
    explicit MissionTraversabilityMapArtifactWriter(
        MissionTraversabilityMapArtifactWriterConfig config = {});

    static MissionTraversabilityMapArtifactWriter from_environment();

    void write_if_due(const MissionLocalTraversabilityMapSnapshot& snapshot);
    void write_final(const MissionLocalTraversabilityMapSnapshot& snapshot);

    [[nodiscard]] bool enabled() const { return config_.enabled; }

private:
    void write_snapshot(const MissionLocalTraversabilityMapSnapshot& snapshot, bool force);

    MissionTraversabilityMapArtifactWriterConfig config_;
    std::optional<std::uint64_t> first_update_timestamp_ns_;
    std::uint64_t last_written_update_count_{0U};
};

}  // namespace dedalus
