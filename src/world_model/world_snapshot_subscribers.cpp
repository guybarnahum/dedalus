#include "dedalus/world_model/world_snapshot_subscribers.hpp"

#include <iomanip>
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
}

ArtifactSnapshotWriter::~ArtifactSnapshotWriter() {
    if (manifest_) {
        manifest_.flush();
    }
}

void ArtifactSnapshotWriter::on_snapshot(const WorldSnapshot& snapshot) {
    ++frame_count_;
    const auto snapshot_name = "snapshot_" + zero_padded(frame_count_, 4) + ".json";
    const auto snapshot_path = output_dir_ / snapshot_name;

    {
        std::ofstream snapshot_file{snapshot_path};
        if (!snapshot_file) {
            throw std::runtime_error("failed to open snapshot output: " + snapshot_path.string());
        }
        snapshot_file << to_json(snapshot);
        snapshot_file.flush();
    }

    manifest_ << frame_count_ << " " << snapshot_name << " "
              << snapshot.timestamp.timestamp_ns << " "
              << snapshot.active_map_frame_id.value << "\n";
    manifest_.flush();
}

int ArtifactSnapshotWriter::frame_count() const {
    return frame_count_;
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

}  // namespace dedalus
