#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
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

    void on_snapshot(const std::shared_ptr<const WorldSnapshot>& snapshot) override;

private:
    std::shared_ptr<LatestWorldSnapshot> latest_snapshot_;
};

struct ArtifactSnapshotWriterConfig {
    std::filesystem::path output_dir;
    int max_queue_depth{64};  // frames; oldest frame is dropped and counted when full
};

struct ArtifactSnapshotWriterStats {
    int frame_count{0};
    int dropped_frames{0};
    std::uint64_t enqueue_count{0};
    std::uint64_t write_count{0};
    std::uint64_t enqueue_total_us{0};
    std::uint64_t write_total_us{0};
    std::uint64_t manifest_flush_total_us{0};
};

class ArtifactSnapshotWriter final : public WorldSnapshotSubscriber {
public:
    explicit ArtifactSnapshotWriter(ArtifactSnapshotWriterConfig config);
    ~ArtifactSnapshotWriter() override;

    ArtifactSnapshotWriter(const ArtifactSnapshotWriter&) = delete;
    ArtifactSnapshotWriter& operator=(const ArtifactSnapshotWriter&) = delete;
    ArtifactSnapshotWriter(ArtifactSnapshotWriter&&) = delete;
    ArtifactSnapshotWriter& operator=(ArtifactSnapshotWriter&&) = delete;

    void on_snapshot(const std::shared_ptr<const WorldSnapshot>& snapshot) override;

    [[nodiscard]] int frame_count() const;
    [[nodiscard]] int dropped_frames() const;
    [[nodiscard]] ArtifactSnapshotWriterStats stats() const;
    [[nodiscard]] const std::filesystem::path& output_dir() const;
    [[nodiscard]] const std::filesystem::path& manifest_path() const;

private:
    static std::string zero_padded(int value, int width);
    void writer_loop();

    ArtifactSnapshotWriterConfig config_;
    std::filesystem::path manifest_path_;
    std::ofstream manifest_;

    std::atomic<int> frame_count_{0};
    std::atomic<int> dropped_frames_{0};
    std::atomic<std::uint64_t> enqueue_count_{0};
    std::atomic<std::uint64_t> write_count_{0};
    std::atomic<std::uint64_t> enqueue_total_us_{0};
    std::atomic<std::uint64_t> write_total_us_{0};
    std::atomic<std::uint64_t> manifest_flush_total_us_{0};

    std::deque<std::pair<int, std::shared_ptr<const WorldSnapshot>>> write_queue_;
    mutable std::mutex mutex_;
    std::condition_variable queue_cv_;
    std::atomic<bool> running_{false};
    std::thread writer_thread_;
};

}  // namespace dedalus
