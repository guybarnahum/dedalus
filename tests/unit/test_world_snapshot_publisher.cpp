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
    void on_snapshot(const dedalus::WorldSnapshot& snapshot) override {
        timestamps.push_back(snapshot.timestamp.timestamp_ns);
        agent_counts.push_back(snapshot.agents.size());
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

dedalus::WorldSnapshot make_snapshot(std::int64_t timestamp_ns, const std::string& track_id) {
    dedalus::WorldSnapshot snapshot;
    snapshot.timestamp = dedalus::TimePoint{timestamp_ns};
    snapshot.active_map_frame_id = dedalus::MapFrameId{"map_test_0001"};
    snapshot.ego.timestamp = snapshot.timestamp;
    snapshot.ego.map_frame_id = snapshot.active_map_frame_id;

    dedalus::AgentTrack agent;
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

    publisher->publish(make_snapshot(100, "track_0001"));
    publisher->publish(make_snapshot(200, "track_0002"));

    require(first->timestamps == std::vector<std::int64_t>{100, 200}, "first subscriber missed snapshots");
    require(second->timestamps == std::vector<std::int64_t>{100, 200}, "second subscriber missed snapshots");
    require(first->agent_counts == std::vector<std::size_t>{1, 1}, "first subscriber got wrong agent counts");
}

void artifact_writer_writes_snapshots_and_manifest() {
    const auto output_dir = std::filesystem::temp_directory_path() / "dedalus_world_snapshot_publisher_test";
    std::filesystem::remove_all(output_dir);

    {
        dedalus::ArtifactSnapshotWriter writer{output_dir};
        writer.on_snapshot(make_snapshot(123, "track_alpha"));
        writer.on_snapshot(make_snapshot(456, "track_beta"));
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

}  // namespace

int main() {
    try {
        publisher_delivers_to_multiple_subscribers();
        artifact_writer_writes_snapshots_and_manifest();
    } catch (const std::exception& exc) {
        std::cerr << "test_world_snapshot_publisher failed: " << exc.what() << '\n';
        return 1;
    }
    return 0;
}
