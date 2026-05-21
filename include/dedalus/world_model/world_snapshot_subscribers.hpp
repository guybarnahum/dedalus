#pragma once

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include "dedalus/behavior/latest_world_snapshot.hpp"
#include "dedalus/world_model/world_snapshot_publisher.hpp"

namespace dedalus {

class LatestWorldSnapshotSubscriber final : public WorldSnapshotSubscriber {
public:
    explicit LatestWorldSnapshotSubscriber(std::shared_ptr<LatestWorldSnapshot> latest_snapshot);

    void on_snapshot(const WorldSnapshot& snapshot) override;

private:
    std::shared_ptr<LatestWorldSnapshot> latest_snapshot_;
};

class ArtifactSnapshotWriter final : public WorldSnapshotSubscriber {
public:
    explicit ArtifactSnapshotWriter(std::filesystem::path output_dir);
    ~ArtifactSnapshotWriter() override;

    ArtifactSnapshotWriter(const ArtifactSnapshotWriter&) = delete;
    ArtifactSnapshotWriter& operator=(const ArtifactSnapshotWriter&) = delete;
    ArtifactSnapshotWriter(ArtifactSnapshotWriter&&) = delete;
    ArtifactSnapshotWriter& operator=(ArtifactSnapshotWriter&&) = delete;

    void on_snapshot(const WorldSnapshot& snapshot) override;

    [[nodiscard]] int frame_count() const;
    [[nodiscard]] const std::filesystem::path& output_dir() const;
    [[nodiscard]] const std::filesystem::path& manifest_path() const;

private:
    static std::string zero_padded(int value, int width);

    std::filesystem::path output_dir_;
    std::filesystem::path manifest_path_;
    std::ofstream manifest_;
    int frame_count_{0};
};

}  // namespace dedalus
