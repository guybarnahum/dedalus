#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <variant>
#include <vector>

#include "dedalus/avoidance/mission_local_traversability_map_publisher.hpp"
#include "dedalus/avoidance/mission_obstacle_map_delta_writer.hpp"
#include "dedalus/behavior/mission_runtime.hpp"
#include "dedalus/perception/ghost_targets.hpp"
#include "dedalus/world_model/world_snapshot_publisher.hpp"

namespace dedalus {

struct RuntimeEventStreamServerConfig {
    std::string bind_host{"127.0.0.1"};
    std::uint16_t port{0};
    // Optional browser-facing HTTP/SSE endpoint. Disabled when http_port == 0.
    std::string http_bind_host{"127.0.0.1"};
    std::uint16_t http_port{0};
    // Optional static-file root for browser diagnostics. Empty disables static serving.
    std::string http_static_root{};
    std::string http_default_file{"mission_local_obstacle_viewer.html"};
    int listen_backlog{8};
    // Shared send queue bound: oldest message is dropped when the queue is full.
    std::size_t max_send_queue_depth{256};
    // Per-client back-buffer bound: client is dropped when its buffer is full.
    std::size_t max_client_pending_depth{16};
};

struct RuntimeEventStreamServerStats {
    std::uint64_t published_seq{0};
    std::size_t connected_clients{0};
    std::uint64_t accepted_clients{0};
    std::uint64_t dropped_clients{0};
    std::size_t sse_clients{0};
    std::uint64_t accepted_sse_clients{0};
    std::uint64_t dropped_sse_clients{0};
    std::uint64_t dropped_messages{0};  // shared queue overflow drops
    std::uint64_t snapshot_messages{0};
    std::uint64_t ghost_detection_messages{0};
    std::uint64_t mission_event_messages{0};
    std::uint64_t mission_obstacle_map_delta_messages{0};
    std::uint64_t traversability_map_snapshot_messages{0};
    std::uint64_t serialize_total_us{0};
    std::uint64_t enqueue_total_us{0};
    std::uint64_t publish_total_us{0};
};

class RuntimeEventStreamServer final : public WorldSnapshotSubscriber, public GhostDetectionsSubscriber, public MissionEventSubscriber, public MissionObstacleMapDeltaSubscriber, public MissionLocalTraversabilityMapSubscriber {
public:
    explicit RuntimeEventStreamServer(RuntimeEventStreamServerConfig config);
    ~RuntimeEventStreamServer() override;

    RuntimeEventStreamServer(const RuntimeEventStreamServer&) = delete;
    RuntimeEventStreamServer& operator=(const RuntimeEventStreamServer&) = delete;
    RuntimeEventStreamServer(RuntimeEventStreamServer&&) = delete;
    RuntimeEventStreamServer& operator=(RuntimeEventStreamServer&&) = delete;

    void start();
    void stop();
    void on_snapshot(const std::shared_ptr<const WorldSnapshot>& snapshot) override;
    void on_ghost_detections(const GhostDetectionsFrame& frame) override;
    void on_mission_event(const MissionEvent& event) override;
    void on_mission_obstacle_map_delta(const MissionObstacleMapDeltaFrame& frame) override;
    void on_traversability_map_snapshot(const MissionLocalTraversabilityMapFrame& frame) override;

    [[nodiscard]] std::uint16_t port() const;
    [[nodiscard]] std::uint16_t http_port() const;
    [[nodiscard]] RuntimeEventStreamServerStats stats() const;

private:
    // A snapshot that has not yet been serialized.  The writer thread picks it
    // up from the queue and calls serialize_snapshot() before publishing.
    struct PendingSnapshot {
        std::uint64_t seq{0};
        std::shared_ptr<const WorldSnapshot> snapshot;
    };
    // Queue items are either pre-serialized strings (all non-snapshot message
    // types) or PendingSnapshots (serialized lazily on the writer thread).
    using QueueItem = std::variant<std::string, PendingSnapshot>;

    void enqueue_line(std::string line);
    void enqueue_snapshot(std::uint64_t seq, std::shared_ptr<const WorldSnapshot> snapshot);
    [[nodiscard]] std::string serialize_snapshot(std::uint64_t seq, const WorldSnapshot& snapshot) const;
    void publish_json_line(const std::string& line);
    void accept_loop();
    void http_accept_loop();
    void writer_loop();
    void close_listen_socket();
    void close_http_listen_socket();
    void close_all_clients();

    RuntimeEventStreamServerConfig config_;
    int listen_fd_{-1};
    int http_listen_fd_{-1};
    std::thread accept_thread_;
    std::thread http_accept_thread_;
    std::thread writer_thread_;
    std::atomic<bool> running_{false};

    mutable std::mutex mutex_;
    std::condition_variable queue_cv_;
    std::deque<QueueItem> send_queue_;
    std::vector<int> client_fds_;
    std::vector<int> sse_client_fds_;
    // Per-client back-buffer: holds lines that could not be sent due to EAGAIN.
    // Owned exclusively by the writer thread; no mutex required.
    std::unordered_map<int, std::deque<std::string>> client_pending_;
    std::unordered_map<int, std::deque<std::string>> sse_client_pending_;
    std::uint64_t published_seq_{0};
    std::uint64_t accepted_clients_{0};
    std::uint64_t dropped_clients_{0};
    std::uint64_t accepted_sse_clients_{0};
    std::uint64_t dropped_sse_clients_{0};
    std::uint64_t dropped_messages_{0};
    std::uint64_t snapshot_messages_{0};
    std::uint64_t ghost_detection_messages_{0};
    std::uint64_t mission_event_messages_{0};
    std::uint64_t mission_obstacle_map_delta_messages_{0};
    std::uint64_t traversability_map_snapshot_messages_{0};
    std::uint64_t serialize_total_us_{0};
    std::uint64_t enqueue_total_us_{0};
    std::uint64_t publish_total_us_{0};
};

using WorldSnapshotStreamServerConfig = RuntimeEventStreamServerConfig;
using WorldSnapshotStreamServerStats = RuntimeEventStreamServerStats;
using WorldSnapshotStreamServer = RuntimeEventStreamServer;

}  // namespace dedalus
