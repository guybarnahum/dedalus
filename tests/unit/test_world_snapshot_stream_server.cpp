#include "dedalus/runtime/world_snapshot_stream_server.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void close_fd(int fd) {
    if (fd >= 0) {
        ::close(fd);
    }
}

int connect_to(std::uint16_t port) {
    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        throw std::runtime_error("socket failed");
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (::inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr) != 1) {
        close_fd(fd);
        throw std::runtime_error("inet_pton failed");
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{3};
    while (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        if (errno == EINTR) {
            continue;
        }
        if (std::chrono::steady_clock::now() >= deadline) {
            const std::string error = std::strerror(errno);
            close_fd(fd);
            throw std::runtime_error("connect failed: " + error);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{20});
    }
    return fd;
}

std::string read_until_lines(int fd, int line_count) {
    std::string buffer;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{3};
    while (std::chrono::steady_clock::now() < deadline) {
        char chunk[4096];
        const auto received = ::recv(fd, chunk, sizeof(chunk), 0);
        if (received > 0) {
            buffer.append(chunk, static_cast<std::size_t>(received));
            int lines = 0;
            for (const char ch : buffer) {
                if (ch == '\n') {
                    ++lines;
                }
            }
            if (lines >= line_count) {
                return buffer;
            }
            continue;
        }
        if (received == 0) {
            break;
        }
        if (errno == EINTR) {
            continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{20});
    }
    return buffer;
}

dedalus::WorldSnapshot make_snapshot(std::int64_t timestamp_ns, const std::string& track_id) {
    dedalus::WorldSnapshot snapshot;
    snapshot.timestamp = dedalus::TimePoint{timestamp_ns};
    snapshot.active_map_frame_id = dedalus::MapFrameId{"map_stream_test"};
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

dedalus::GhostDetectionsFrame make_ghost_frame() {
    dedalus::GhostDetectionsFrame frame;
    frame.timestamp = dedalus::TimePoint{500};
    frame.map_frame_id = dedalus::MapFrameId{"map_stream_test"};
    frame.scenario_elapsed_s = 1.25;

    dedalus::GhostDetectionState detection;
    detection.source_track_id = dedalus::TrackId{"ghost_person_001"};
    detection.class_label = dedalus::ClassLabel::Person;
    detection.confidence = 0.82;
    detection.position_local_m = dedalus::Vec3{12.0, -4.0, 0.0};
    detection.velocity_local_mps = dedalus::Vec3{0.3, 0.0, 0.0};
    detection.size_m = dedalus::Vec3{0.6, 0.6, 1.8};
    frame.detections.push_back(detection);
    return frame;
}

dedalus::MissionEvent make_target_selected_event() {
    return dedalus::MissionEvent{
        .timestamp = dedalus::TimePoint{750},
        .json = "{\"event\":\"target_selected\",\"tick\":12,\"source_track_id\":\"ghost_person_001\",\"agent_id\":\"agent_ghost_person_001\"}"};
}

void streams_jsonl_runtime_events_to_client() {
    dedalus::RuntimeEventStreamServer server{
        dedalus::RuntimeEventStreamServerConfig{.bind_host = "127.0.0.1", .port = 0}};
    server.start();
    const auto port = server.port();
    require(port > 0, "ephemeral port should be assigned");

    const int fd = connect_to(port);
    std::this_thread::sleep_for(std::chrono::milliseconds{80});

    server.on_ghost_detections(make_ghost_frame());
    server.on_mission_event(make_target_selected_event());
    server.on_snapshot(make_snapshot(1000, "track_a"));
    server.on_snapshot(make_snapshot(2000, "track_b"));

    const auto received = read_until_lines(fd, 4);
    close_fd(fd);
    server.stop();

    require(received.find("\"type\":\"ghost_detections\"") != std::string::npos, "stream missing ghost_detections type");
    require(received.find("\"type\":\"mission_event\"") != std::string::npos, "stream missing mission_event type");
    require(received.find("\"type\":\"world_snapshot\"") != std::string::npos, "stream missing world_snapshot type");
    require(received.find("\"seq\":1") != std::string::npos, "stream missing seq 1");
    require(received.find("\"seq\":2") != std::string::npos, "stream missing seq 2");
    require(received.find("\"seq\":3") != std::string::npos, "stream missing seq 3");
    require(received.find("\"seq\":4") != std::string::npos, "stream missing seq 4");
    require(received.find("ghost_person_001") != std::string::npos, "stream missing ghost/selected track");
    require(received.find("target_selected") != std::string::npos, "stream missing target_selected mission event");
    require(received.find("track_a") != std::string::npos, "stream missing first world track");
    require(received.find("track_b") != std::string::npos, "stream missing second world track");
    require(received.find("map_stream_test") != std::string::npos, "stream missing map frame");

    const auto stats = server.stats();
    require(stats.published_seq == 4, "server published_seq should be 4");
    require(stats.accepted_clients >= 1, "server should accept at least one client");
}

}  // namespace

int main() {
    try {
        streams_jsonl_runtime_events_to_client();
    } catch (const std::exception& exc) {
        std::cerr << "test_world_snapshot_stream_server failed: " << exc.what() << '\n';
        return 1;
    }
    return 0;
}
