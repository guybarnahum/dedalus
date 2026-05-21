#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "dedalus/behavior/mission_runtime.hpp"
#include "dedalus/perception/ghost_targets.hpp"
#include "dedalus/world_model/world_snapshot_publisher.hpp"

namespace dedalus {

struct RuntimeEventStreamServerConfig {
    std::string bind_host{"127.0.0.1"};
    std::uint16_t port{0};
    int listen_backlog{8};
};

struct RuntimeEventStreamServerStats {
    std::uint64_t published_seq{0};
    std::size_t connected_clients{0};
    std::uint64_t accepted_clients{0};
    std::uint64_t dropped_clients{0};
};

class RuntimeEventStreamServer final : public WorldSnapshotSubscriber, public GhostDetectionsSubscriber, public MissionEventSubscriber {
public:
    explicit RuntimeEventStreamServer(RuntimeEventStreamServerConfig config);
    ~RuntimeEventStreamServer() override;

    RuntimeEventStreamServer(const RuntimeEventStreamServer&) = delete;
    RuntimeEventStreamServer& operator=(const RuntimeEventStreamServer&) = delete;
    RuntimeEventStreamServer(RuntimeEventStreamServer&&) = delete;
    RuntimeEventStreamServer& operator=(RuntimeEventStreamServer&&) = delete;

    void start();
    void stop();
    void on_snapshot(const WorldSnapshot& snapshot) override;
    void on_ghost_detections(const GhostDetectionsFrame& frame) override;
    void on_mission_event(const MissionEvent& event) override;

    [[nodiscard]] std::uint16_t port() const;
    [[nodiscard]] RuntimeEventStreamServerStats stats() const;

private:
    void publish_json_line(const std::string& line);
    void accept_loop();
    void close_listen_socket();
    void close_all_clients();

    RuntimeEventStreamServerConfig config_;
    int listen_fd_{-1};
    std::thread accept_thread_;
    std::atomic<bool> running_{false};

    mutable std::mutex mutex_;
    std::vector<int> client_fds_;
    std::uint64_t published_seq_{0};
    std::uint64_t accepted_clients_{0};
    std::uint64_t dropped_clients_{0};
};

using WorldSnapshotStreamServerConfig = RuntimeEventStreamServerConfig;
using WorldSnapshotStreamServerStats = RuntimeEventStreamServerStats;
using WorldSnapshotStreamServer = RuntimeEventStreamServer;

}  // namespace dedalus
