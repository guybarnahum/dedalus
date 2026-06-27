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

std::uint16_t reserve_tcp_port() {
    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    require(fd >= 0, "reserve_tcp_port socket failed");

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(0);
    require(::inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr) == 1, "reserve_tcp_port inet_pton failed");
    require(::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0, "reserve_tcp_port bind failed");

    sockaddr_in bound_addr{};
    socklen_t bound_len = sizeof(bound_addr);
    require(::getsockname(fd, reinterpret_cast<sockaddr*>(&bound_addr), &bound_len) == 0, "reserve_tcp_port getsockname failed");

    const auto port = static_cast<std::uint16_t>(ntohs(bound_addr.sin_port));
    close_fd(fd);
    require(port > 0, "reserve_tcp_port returned zero");
    return port;
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

std::string read_available_for_ms(int fd, int milliseconds) {
    std::string buffer;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds{milliseconds};
    while (std::chrono::steady_clock::now() < deadline) {
        char chunk[4096];
        const auto received = ::recv(fd, chunk, sizeof(chunk), MSG_DONTWAIT);
        if (received > 0) {
            buffer.append(chunk, static_cast<std::size_t>(received));
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

void write_request(int fd, const std::string& request) {
    const char* data = request.data();
    std::size_t remaining = request.size();
    while (remaining > 0U) {
        const auto sent = ::send(fd, data, remaining, 0);
        if (sent > 0) {
            data += sent;
            remaining -= static_cast<std::size_t>(sent);
            continue;
        }
        if (errno == EINTR) {
            continue;
        }
        throw std::runtime_error("send request failed");
    }
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

dedalus::MissionObstacleMapDeltaFrame make_delta_frame() {
    return dedalus::MissionObstacleMapDeltaFrame{
        .timestamp_ns = 900,
        .json = "{\"schema\":\"dedalus.mission_obstacle_map_delta_batch.v2\",\"time_unit\":\"unix_ns\",\"site_id\":\"stream_site\",\"site_frame_id\":\"airsim_world\",\"mission_id\":\"stream_mission\",\"mission_map_frame_id\":\"airsim_world\",\"mission_start_unix_ns\":100,\"batch_unix_ns\":900,\"update_count\":4,\"previous_snapshot_timestamp_ns\":500,\"changed_cell_count\":1,\"cells\":[{\"center_mission\":{\"x\":1,\"y\":2,\"z\":3},\"occupied_score\":0.9,\"free_score\":0.1,\"confidence\":0.8,\"first_seen_unix_ns\":100,\"last_seen_unix_ns\":900,\"source_kind\":\"depth_provider\",\"source_provider\":\"airsim_depth_obstacle_detector\"}]}\n"};
}

void streams_jsonl_runtime_events_to_client() {
    dedalus::RuntimeEventStreamServer server{
        dedalus::RuntimeEventStreamServerConfig{.bind_host = "127.0.0.1", .port = 0, .http_bind_host = "127.0.0.1", .http_port = 0}};
    server.start();
    const auto port = server.port();
    const auto http_port = server.http_port();
    require(port > 0, "ephemeral port should be assigned");
    require(http_port > 0, "HTTP ephemeral port should be assigned");

    const int health_fd = connect_to(http_port);
    write_request(health_fd, "GET /healthz HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n\r\n");
    const auto health = read_available_for_ms(health_fd, 500);
    close_fd(health_fd);
    require(health.find("HTTP/1.1 200 OK") != std::string::npos, "healthz missing 200 response");
    require(health.find("\"ok\":true") != std::string::npos, "healthz missing ok=true");

    const int sse_fd = connect_to(http_port);
    write_request(sse_fd, "GET /events HTTP/1.1\r\nHost: localhost\r\nAccept: text/event-stream\r\n\r\n");
    std::this_thread::sleep_for(std::chrono::milliseconds{80});

    const int fd = connect_to(port);
    std::this_thread::sleep_for(std::chrono::milliseconds{80});

    server.on_ghost_detections(make_ghost_frame());
    server.on_mission_event(make_target_selected_event());
    server.on_mission_obstacle_map_delta(make_delta_frame());
    server.on_snapshot(std::make_shared<dedalus::WorldSnapshot>(make_snapshot(1000, "track_a")));
    server.on_snapshot(std::make_shared<dedalus::WorldSnapshot>(make_snapshot(2000, "track_b")));

    const auto received = read_until_lines(fd, 5);
    const auto sse_received = read_available_for_ms(sse_fd, 1000);
    close_fd(fd);
    close_fd(sse_fd);
    server.stop();

    require(received.find("\"type\":\"ghost_detections\"") != std::string::npos, "stream missing ghost_detections type");
    require(received.find("\"type\":\"mission_event\"") != std::string::npos, "stream missing mission_event type");
    require(received.find("\"type\":\"world_snapshot\"") != std::string::npos, "stream missing world_snapshot type");
    require(received.find("\"type\":\"mission_obstacle_map_delta\"") != std::string::npos, "stream missing mission_obstacle_map_delta type");
    require(received.find("\"seq\":1") != std::string::npos, "stream missing seq 1");
    require(received.find("\"seq\":2") != std::string::npos, "stream missing seq 2");
    require(received.find("\"seq\":3") != std::string::npos, "stream missing seq 3");
    require(received.find("\"seq\":4") != std::string::npos, "stream missing seq 4");
    require(received.find("\"seq\":5") != std::string::npos, "stream missing seq 5");
    require(received.find("ghost_person_001") != std::string::npos, "stream missing ghost/selected track");
    require(received.find("target_selected") != std::string::npos, "stream missing target_selected mission event");
    require(received.find("track_a") != std::string::npos, "stream missing first world track");
    require(received.find("track_b") != std::string::npos, "stream missing second world track");
    require(received.find("map_stream_test") != std::string::npos, "stream missing map frame");
    require(received.find("dedalus.mission_obstacle_map_delta_batch.v2") != std::string::npos, "stream missing delta schema");
    require(received.find("airsim_depth_obstacle_detector") != std::string::npos, "stream missing delta source provider");
    require(sse_received.find("HTTP/1.1 200 OK") != std::string::npos, "SSE stream missing 200 response");
    require(sse_received.find("Content-Type: text/event-stream") != std::string::npos, "SSE stream missing content type");
    require(sse_received.find("event: mission_obstacle_map_delta") != std::string::npos, "SSE stream missing mission obstacle delta event name");
    require(sse_received.find("data: {\"type\":\"mission_obstacle_map_delta\"") != std::string::npos, "SSE stream missing mission obstacle delta data");
    require(sse_received.find("dedalus.mission_obstacle_map_delta_batch.v2") != std::string::npos, "SSE stream missing delta schema");

    const auto stats = server.stats();
    require(stats.published_seq == 5, "server published_seq should be 5");
    require(stats.accepted_clients >= 1, "server should accept at least one client");
    require(stats.accepted_sse_clients >= 1, "server should accept at least one SSE client");
}

void bounded_queue_drops_oldest_on_overflow() {
    // Configure a tiny shared queue so we can force a drop without a real client.
    // The server is deliberately NOT started: no writer thread drains the queue,
    // so we can push items faster than they are consumed.
    dedalus::RuntimeEventStreamServerConfig cfg;
    cfg.bind_host = "127.0.0.1";
    cfg.port = 0;
    cfg.max_send_queue_depth = 2;
    dedalus::RuntimeEventStreamServer server{cfg};

    // Three pushes: the third overflows the queue, dropping the oldest.
    server.on_snapshot(std::make_shared<dedalus::WorldSnapshot>(make_snapshot(1000, "track_a")));
    server.on_snapshot(std::make_shared<dedalus::WorldSnapshot>(make_snapshot(2000, "track_b")));
    server.on_snapshot(std::make_shared<dedalus::WorldSnapshot>(make_snapshot(3000, "track_c")));

    const auto s = server.stats();
    require(s.dropped_messages == 1, "one message should be dropped when shared queue overflows");
    require(s.published_seq == 3, "published_seq increments regardless of queue drops");
}

}  // namespace

int main() {
    try {
        streams_jsonl_runtime_events_to_client();
        bounded_queue_drops_oldest_on_overflow();
    } catch (const std::exception& exc) {
        std::cerr << "test_world_snapshot_stream_server failed: " << exc.what() << '\n';
        return 1;
    }
    return 0;
}
