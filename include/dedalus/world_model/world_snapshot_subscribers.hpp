#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

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
    static constexpr int kMaxQueueDepth = 64;

    static std::string zero_padded(int value, int width);
    void writer_loop();

    std::filesystem::path output_dir_;
    std::filesystem::path manifest_path_;
    std::ofstream manifest_;

    std::atomic<int> frame_count_{0};

    std::deque<std::pair<int, std::shared_ptr<const WorldSnapshot>>> write_queue_;
    mutable std::mutex mutex_;
    std::condition_variable queue_cv_;
    std::atomic<bool> running_{false};
    std::thread writer_thread_;
};

}  // namespace dedalus
