#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "dedalus/world_model/world_snapshot_publisher.hpp"

namespace dedalus {

struct WorldSnapshotStreamServerConfig {
    std::string bind_host{"127.0.0.1"};
    std::uint16_t port{0};
    int listen_backlog{8};
};

struct WorldSnapshotStreamServerStats {
    std::uint64_t published_seq{0};
    std::size_t connected_clients{0};
    std::uint64_t accepted_clients{0};
    std::uint64_t dropped_clients{0};
};

class WorldSnapshotStreamServer final : public WorldSnapshotSubscriber {
public:
    explicit WorldSnapshotStreamServer(WorldSnapshotStreamServerConfig config);
    ~WorldSnapshotStreamServer() override;

    WorldSnapshotStreamServer(const WorldSnapshotStreamServer&) = delete;
    WorldSnapshotStreamServer& operator=(const WorldSnapshotStreamServer&) = delete;
    WorldSnapshotStreamServer(WorldSnapshotStreamServer&&) = delete;
    WorldSnapshotStreamServer& operator=(WorldSnapshotStreamServer&&) = delete;

    void start();
    void stop();
    void on_snapshot(const WorldSnapshot& snapshot) override;

    [[nodiscard]] std::uint16_t port() const;
    [[nodiscard]] WorldSnapshotStreamServerStats stats() const;

private:
    void accept_loop();
    void close_listen_socket();
    void close_all_clients();

    WorldSnapshotStreamServerConfig config_;
    int listen_fd_{-1};
    std::thread accept_thread_;
    std::atomic<bool> running_{false};

    mutable std::mutex mutex_;
    std::vector<int> client_fds_;
    std::uint64_t published_seq_{0};
    std::uint64_t accepted_clients_{0};
    std::uint64_t dropped_clients_{0};
};

}  // namespace dedalus
