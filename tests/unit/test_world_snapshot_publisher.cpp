#include "dedalus/world_model/world_snapshot_publisher.hpp"
#include "dedalus/world_model/world_snapshot_subscribers.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

class RecordingSubscriber final : public dedalus::WorldSnapshotSubscriber {
public:
    void on_snapshot(const std::shared_ptr<const dedalus::WorldSnapshot>& snapshot) override {
        timestamps.push_back(snapshot->timestamp.timestamp_ns);
        agent_counts.push_back(snapshot->agents.size());
    }

    std::vector<std::int64_t> timestamps;
    std::vector<std::size_t> agent_counts;
};

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

std::string read_text(const std::filesystem::path& path) {
    std::ifstream input{path};
    if (!input) {
        throw std::runtime_error("failed to open: " + path.string());
    }
    std::string contents;
    std::string line;
    while (std::getline(input, line)) {
        contents += line;
        contents += '\n';
    }
    return contents;
}

int count_manifest_data_rows(const std::string& manifest) {
    int rows = 0;
    std::size_t cursor = 0;
    while (cursor < manifest.size()) {
        const auto next = manifest.find('\n', cursor);
        const auto line = manifest.substr(cursor, next == std::string::npos ? std::string::npos : next - cursor);
        if (!line.empty() && line[0] != '#') {
            ++rows;
        }
        if (next == std::string::npos) {
            break;
        }
        cursor = next + 1U;
    }
    return rows;
}

dedalus::WorldSnapshot make_snapshot(std::int64_t timestamp_ns, const std::string& track_id) {
    dedalus::WorldSnapshot snapshot;
    snapshot.timestamp = dedalus::TimePoint{timestamp_ns};
    snapshot.active_map_frame_id = dedalus::MapFrameId{"map_test_0001"};
    snapshot.ego.timestamp = snapshot.timestamp;
    snapshot.ego.map_frame_id = snapshot.active_map_frame_id;

    dedalus::AgentState agent;
    agent.agent_id = dedalus::AgentId{"agent_" + track_id};
    agent.source_track_id = dedalus::TrackId{track_id};
    agent.class_label = dedalus::ClassLabel::Person;
    agent.confidence = 0.9F;
    agent.position_local = dedalus::Vec3{1.0, 2.0, 3.0};
    snapshot.agents.push_back(agent);
    return snapshot;
}

void publisher_delivers_to_multiple_subscribers() {
    auto publisher = std::make_shared<dedalus::WorldSnapshotPublisher>();
    auto first = std::make_shared<RecordingSubscriber>();
    auto second = std::make_shared<RecordingSubscriber>();
    publisher->subscribe(first);
    publisher->subscribe(second);

    publisher->publish(std::make_shared<dedalus::WorldSnapshot>(make_snapshot(100, "track_0001")));
    publisher->publish(std::make_shared<dedalus::WorldSnapshot>(make_snapshot(200, "track_0002")));

    require(first->timestamps == std::vector<std::int64_t>{100, 200}, "first subscriber missed snapshots");
    require(second->timestamps == std::vector<std::int64_t>{100, 200}, "second subscriber missed snapshots");
    require(first->agent_counts == std::vector<std::size_t>{1, 1}, "first subscriber got wrong agent counts");
}

void synchronous_publishers_do_not_drop_burst_frames() {
    constexpr int kSnapshotCount = 120;
    auto publisher = std::make_shared<dedalus::WorldSnapshotPublisher>();
    auto first = std::make_shared<RecordingSubscriber>();
    auto second = std::make_shared<RecordingSubscriber>();
    publisher->subscribe(first);
    publisher->subscribe(second);

    for (int index = 1; index <= kSnapshotCount; ++index) {
        publisher->publish(std::make_shared<dedalus::WorldSnapshot>(make_snapshot(index * 1000, "track_" + std::to_string(index))));
    }

    require(static_cast<int>(first->timestamps.size()) == kSnapshotCount, "first subscriber missed burst snapshots");
    require(static_cast<int>(second->timestamps.size()) == kSnapshotCount, "second subscriber missed burst snapshots");
    for (int index = 1; index <= kSnapshotCount; ++index) {
        const auto expected_timestamp = static_cast<std::int64_t>(index * 1000);
        require(first->timestamps[index - 1] == expected_timestamp, "first subscriber saw out-of-order or skipped timestamp");
        require(second->timestamps[index - 1] == expected_timestamp, "second subscriber saw out-of-order or skipped timestamp");
    }
}

void artifact_writer_writes_snapshots_and_manifest() {
    const auto output_dir = std::filesystem::temp_directory_path() / "dedalus_world_snapshot_publisher_test";
    std::filesystem::remove_all(output_dir);

    {
        dedalus::ArtifactSnapshotWriter writer{dedalus::ArtifactSnapshotWriterConfig{.output_dir = output_dir}};
        writer.on_snapshot(std::make_shared<dedalus::WorldSnapshot>(make_snapshot(123, "track_alpha")));
        writer.on_snapshot(std::make_shared<dedalus::WorldSnapshot>(make_snapshot(456, "track_beta")));
        require(writer.frame_count() == 2, "artifact writer frame count should advance");
    }

    const auto manifest_path = output_dir / "snapshot_manifest.txt";
    const auto snapshot_1 = output_dir / "snapshot_0001.json";
    const auto snapshot_2 = output_dir / "snapshot_0002.json";
    require(std::filesystem::exists(manifest_path), "manifest should exist");
    require(std::filesystem::exists(snapshot_1), "snapshot_0001 should exist");
    require(std::filesystem::exists(snapshot_2), "snapshot_0002 should exist");

    const auto manifest = read_text(manifest_path);
    require(manifest.find("# index path timestamp_ns active_map_frame_id") != std::string::npos, "manifest missing header");
    require(manifest.find("1 snapshot_0001.json 123 map_test_0001") != std::string::npos, "manifest missing first row");
    require(manifest.find("2 snapshot_0002.json 456 map_test_0001") != std::string::npos, "manifest missing second row");

    const auto first_snapshot = read_text(snapshot_1);
    require(first_snapshot.find("track_alpha") != std::string::npos, "first snapshot missing agent track");

    std::filesystem::remove_all(output_dir);
}

void artifact_writer_does_not_skip_burst_frames() {
    constexpr int kSnapshotCount = 40;
    const auto output_dir = std::filesystem::temp_directory_path() / "dedalus_world_snapshot_publisher_burst_test";
    std::filesystem::remove_all(output_dir);

    {
        dedalus::ArtifactSnapshotWriter writer{dedalus::ArtifactSnapshotWriterConfig{.output_dir = output_dir}};
        for (int index = 1; index <= kSnapshotCount; ++index) {
            writer.on_snapshot(std::make_shared<dedalus::WorldSnapshot>(make_snapshot(index * 1000, "track_burst_" + std::to_string(index))));
        }
        require(writer.frame_count() == kSnapshotCount, "artifact writer missed burst frame count");
    }

    const auto manifest = read_text(output_dir / "snapshot_manifest.txt");
    require(count_manifest_data_rows(manifest) == kSnapshotCount, "manifest data row count should match published snapshots");
    require(std::filesystem::exists(output_dir / "snapshot_0001.json"), "first burst snapshot missing");
    require(std::filesystem::exists(output_dir / "snapshot_0040.json"), "last burst snapshot missing");
    require(read_text(output_dir / "snapshot_0040.json").find("track_burst_40") != std::string::npos, "last burst snapshot has wrong contents");

    std::filesystem::remove_all(output_dir);
}

void artifact_writer_dropped_frames_counter() {
    const auto output_dir = std::filesystem::temp_directory_path() / "dedalus_world_snapshot_publisher_drop_test";
    std::filesystem::remove_all(output_dir);

    {
        dedalus::ArtifactSnapshotWriterConfig cfg;
        cfg.output_dir = output_dir;
        cfg.max_queue_depth = 4;
        dedalus::ArtifactSnapshotWriter writer{cfg};
        // Push 500 frames in a tight loop. The writer thread performs disk I/O between dequeues
        // and cannot drain frames faster than they are enqueued, so the bounded queue will
        // overflow and dropped_frames must become positive.
        for (int i = 1; i <= 500; ++i) {
            writer.on_snapshot(std::make_shared<dedalus::WorldSnapshot>(make_snapshot(i * 1000, "track_drop_" + std::to_string(i))));
        }
        require(writer.dropped_frames() >= 1, "dropped_frames should be positive after queue overflow");
        require(writer.frame_count() == 500, "frame_count increments regardless of drops");
    }

    std::filesystem::remove_all(output_dir);
}

}  // namespace

int main() {
    try {
        publisher_delivers_to_multiple_subscribers();
        synchronous_publishers_do_not_drop_burst_frames();
        artifact_writer_writes_snapshots_and_manifest();
        artifact_writer_does_not_skip_burst_frames();
        artifact_writer_dropped_frames_counter();
    } catch (const std::exception& exc) {
        std::cerr << "test_world_snapshot_publisher failed: " << exc.what() << '\n';
        return 1;
    }
    return 0;
}
