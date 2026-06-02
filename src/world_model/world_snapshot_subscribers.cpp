#include "dedalus/world_model/world_snapshot_subscribers.hpp"

#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace dedalus {

LatestWorldSnapshotSubscriber::LatestWorldSnapshotSubscriber(std::shared_ptr<LatestWorldSnapshot> latest_snapshot)
    : latest_snapshot_(std::move(latest_snapshot)) {
    if (!latest_snapshot_) {
        throw std::invalid_argument("LatestWorldSnapshotSubscriber requires LatestWorldSnapshot");
    }
}

void LatestWorldSnapshotSubscriber::on_snapshot(const WorldSnapshot& snapshot) {
    latest_snapshot_->publish(snapshot);
}

ArtifactSnapshotWriter::ArtifactSnapshotWriter(std::filesystem::path output_dir)
    : output_dir_(std::move(output_dir)),
      manifest_path_(output_dir_ / "snapshot_manifest.txt") {
    std::filesystem::create_directories(output_dir_);
    manifest_.open(manifest_path_, std::ios::out | std::ios::trunc);
    if (!manifest_) {
        throw std::runtime_error("failed to open snapshot manifest: " + manifest_path_.string());
    }
    manifest_ << "# index path timestamp_ns active_map_frame_id\n";
    manifest_.flush();
    running_ = true;
    writer_thread_ = std::thread([this]() { writer_loop(); });
}

ArtifactSnapshotWriter::~ArtifactSnapshotWriter() {
    running_ = false;
    queue_cv_.notify_all();
    if (writer_thread_.joinable()) {
        writer_thread_.join();
    }
    if (manifest_) {
        manifest_.flush();
    }
}

void ArtifactSnapshotWriter::on_snapshot(const WorldSnapshot& snapshot) {
    const int frame_number = ++frame_count_;
    auto item = std::make_shared<const WorldSnapshot>(snapshot);
    {
        std::lock_guard<std::mutex> lock{mutex_};
        if (static_cast<int>(write_queue_.size()) >= kMaxQueueDepth) {
            std::cerr << "ArtifactSnapshotWriter: queue full (" << kMaxQueueDepth
                      << " frames), dropping frame " << write_queue_.front().first << "\n";
            write_queue_.pop_front();
        }
        write_queue_.emplace_back(frame_number, std::move(item));
    }
    queue_cv_.notify_one();
}

int ArtifactSnapshotWriter::frame_count() const {
    return frame_count_.load();
}

const std::filesystem::path& ArtifactSnapshotWriter::output_dir() const {
    return output_dir_;
}

const std::filesystem::path& ArtifactSnapshotWriter::manifest_path() const {
    return manifest_path_;
}

std::string ArtifactSnapshotWriter::zero_padded(int value, int width) {
    std::ostringstream out;
    out << std::setw(width) << std::setfill('0') << value;
    return out.str();
}

void ArtifactSnapshotWriter::writer_loop() {
    while (true) {
        std::pair<int, std::shared_ptr<const WorldSnapshot>> task;
        {
            std::unique_lock<std::mutex> lock{mutex_};
            queue_cv_.wait(lock, [this] {
                return !write_queue_.empty() || !running_;
            });
            if (write_queue_.empty()) {
                break;
            }
            task = std::move(write_queue_.front());
            write_queue_.pop_front();
        }

        const auto frame_number = task.first;
        const auto& snap = task.second;
        const auto snapshot_name = "snapshot_" + zero_padded(frame_number, 4) + ".json";
        const auto snapshot_path = output_dir_ / snapshot_name;

        {
            std::ofstream snapshot_file{snapshot_path};
            if (!snapshot_file) {
                std::cerr << "ArtifactSnapshotWriter: failed to open snapshot output: "
                          << snapshot_path.string() << "\n";
                continue;
            }
            snapshot_file << to_json(*snap);
            // destructor closes and flushes
        }

        manifest_ << frame_number << " " << snapshot_name << " "
                  << snap->timestamp.timestamp_ns << " "
                  << snap->active_map_frame_id.value << "\n";
        manifest_.flush();
    }
}

}  // namespace dedalus
