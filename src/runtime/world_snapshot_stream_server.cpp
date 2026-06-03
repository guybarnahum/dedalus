#include "dedalus/runtime/world_snapshot_stream_server.hpp"

#include "dedalus/core/json_utils.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include "dedalus/world_model/world_snapshot.hpp"

namespace dedalus {
namespace {

using SteadyClock = std::chrono::steady_clock;

std::uint64_t elapsed_us(const SteadyClock::time_point start) {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(SteadyClock::now() - start).count());
}

void close_fd(int fd) {
    if (fd >= 0) {
        ::close(fd);
    }
}

void set_nonblocking(int fd) {
    const int flags = ::fcntl(fd, F_GETFL, 0);
    if (flags < 0) {
        throw std::runtime_error("fcntl(F_GETFL) failed: " + std::string(std::strerror(errno)));
    }
    if (::fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        throw std::runtime_error("fcntl(F_SETFL, O_NONBLOCK) failed: " + std::string(std::strerror(errno)));
    }
}

std::string stream_line_for(std::uint64_t seq, const WorldSnapshot& snapshot) {
    std::string line;
    line.reserve(4096U);
    line += "{\"type\":\"world_snapshot\",\"seq\":";
    line += std::to_string(seq);
    line += ",\"timestamp_ns\":";
    line += std::to_string(snapshot.timestamp.timestamp_ns);
    line += ",\"active_map_frame_id\":\"";
    line += json_escape(snapshot.active_map_frame_id.value);
    line += "\",\"snapshot\":";
    line += to_compact_json(to_json(snapshot));
    line += "}\n";
    return line;
}

std::string stream_line_for(std::uint64_t seq, const GhostDetectionsFrame& frame) {
    std::string line;
    line.reserve(2048U);
    line += "{\"type\":\"ghost_detections\",\"seq\":";
    line += std::to_string(seq);
    line += ",\"timestamp_ns\":";
    line += std::to_string(frame.timestamp.timestamp_ns);
    line += ",\"map_frame_id\":\"";
    line += json_escape(frame.map_frame_id.value);
    line += "\",\"ghost_detections\":";
    line += to_compact_json(to_json(frame));
    line += "}\n";
    return line;
}

std::string stream_line_for(std::uint64_t seq, const MissionEvent& event) {
    std::string line;
    line.reserve(event.json.size() + 128U);
    line += "{\"type\":\"mission_event\",\"seq\":";
    line += std::to_string(seq);
    line += ",\"timestamp_ns\":";
    line += std::to_string(event.timestamp.timestamp_ns);
    line += ",\"mission_event\":";
    line += to_compact_json(event.json);
    line += "}\n";
    return line;
}

bool send_all_nonblocking(int fd, const std::string& payload) {
    const char* data = payload.data();
    std::size_t remaining = payload.size();
    while (remaining > 0U) {
        const auto sent = ::send(fd, data, remaining, MSG_NOSIGNAL | MSG_DONTWAIT);
        if (sent > 0) {
            data += sent;
            remaining -= static_cast<std::size_t>(sent);
            continue;
        }
        if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            return false;
        }
        if (sent < 0 && errno == EINTR) {
            continue;
        }
        return false;
    }
    return true;
}

}  // namespace

RuntimeEventStreamServer::RuntimeEventStreamServer(RuntimeEventStreamServerConfig config)
    : config_(std::move(config)) {}

RuntimeEventStreamServer::~RuntimeEventStreamServer() {
    stop();
}

void RuntimeEventStreamServer::start() {
    if (running_.exchange(true)) {
        return;
    }

    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) {
        running_ = false;
        throw std::runtime_error("socket() failed: " + std::string(std::strerror(errno)));
    }

    int reuse = 1;
    (void)::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    set_nonblocking(listen_fd_);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(config_.port);
    if (::inet_pton(AF_INET, config_.bind_host.c_str(), &addr.sin_addr) != 1) {
        close_listen_socket();
        running_ = false;
        throw std::runtime_error("invalid runtime event stream bind host: " + config_.bind_host);
    }

    if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        const auto message = std::string(std::strerror(errno));
        close_listen_socket();
        running_ = false;
        throw std::runtime_error("bind() failed for runtime event stream: " + message);
    }

    if (config_.port == 0) {
        sockaddr_in bound_addr{};
        socklen_t bound_len = sizeof(bound_addr);
        if (::getsockname(listen_fd_, reinterpret_cast<sockaddr*>(&bound_addr), &bound_len) == 0) {
            config_.port = ntohs(bound_addr.sin_port);
        }
    }

    if (::listen(listen_fd_, config_.listen_backlog) < 0) {
        const auto message = std::string(std::strerror(errno));
        close_listen_socket();
        running_ = false;
        throw std::runtime_error("listen() failed for runtime event stream: " + message);
    }

    accept_thread_ = std::thread([this]() { accept_loop(); });
    writer_thread_ = std::thread([this]() { writer_loop(); });
}

void RuntimeEventStreamServer::stop() {
    if (!running_.exchange(false)) {
        return;
    }
    close_listen_socket();
    queue_cv_.notify_all();
    if (accept_thread_.joinable()) {
        accept_thread_.join();
    }
    if (writer_thread_.joinable()) {
        writer_thread_.join();
    }
    close_all_clients();
}

void RuntimeEventStreamServer::on_snapshot(const WorldSnapshot& snapshot) {
    const auto start = SteadyClock::now();
    const std::uint64_t seq = [this] {
        std::lock_guard<std::mutex> lock{mutex_};
        ++snapshot_messages_;
        return ++published_seq_;
    }();
    auto line = stream_line_for(seq, snapshot);
    const auto serialize_duration_us = elapsed_us(start);
    enqueue_line(std::move(line));
    std::lock_guard<std::mutex> lock{mutex_};
    serialize_total_us_ += serialize_duration_us;
}

void RuntimeEventStreamServer::on_ghost_detections(const GhostDetectionsFrame& frame) {
    const auto start = SteadyClock::now();
    const std::uint64_t seq = [this] {
        std::lock_guard<std::mutex> lock{mutex_};
        ++ghost_detection_messages_;
        return ++published_seq_;
    }();
    auto line = stream_line_for(seq, frame);
    const auto serialize_duration_us = elapsed_us(start);
    enqueue_line(std::move(line));
    std::lock_guard<std::mutex> lock{mutex_};
    serialize_total_us_ += serialize_duration_us;
}

void RuntimeEventStreamServer::on_mission_event(const MissionEvent& event) {
    const auto start = SteadyClock::now();
    const std::uint64_t seq = [this] {
        std::lock_guard<std::mutex> lock{mutex_};
        ++mission_event_messages_;
        return ++published_seq_;
    }();
    auto line = stream_line_for(seq, event);
    const auto serialize_duration_us = elapsed_us(start);
    enqueue_line(std::move(line));
    std::lock_guard<std::mutex> lock{mutex_};
    serialize_total_us_ += serialize_duration_us;
}

void RuntimeEventStreamServer::enqueue_line(std::string line) {
    const auto start = SteadyClock::now();
    {
        std::lock_guard<std::mutex> lock{mutex_};
        if (send_queue_.size() >= config_.max_send_queue_depth) {
            send_queue_.pop_front();  // drop oldest; bounds memory under slow consumers
            ++dropped_messages_;
        }
        send_queue_.push_back(std::move(line));
        enqueue_total_us_ += elapsed_us(start);
    }
    queue_cv_.notify_one();
}

void RuntimeEventStreamServer::publish_json_line(const std::string& line) {
    const auto start = SteadyClock::now();
    std::vector<int> clients;
    {
        std::lock_guard<std::mutex> lock{mutex_};
        clients = client_fds_;
    }

    if (clients.empty()) {
        std::lock_guard<std::mutex> lock{mutex_};
        publish_total_us_ += elapsed_us(start);
        return;
    }

    std::vector<int> dead_clients;
    for (const int fd : clients) {
        // Drain any per-client back-buffer before sending the new line.
        // client_pending_ is owned exclusively by the writer thread; no mutex needed.
        auto& pending = client_pending_[fd];
        bool caught_up = true;
        while (!pending.empty()) {
            if (send_all_nonblocking(fd, pending.front())) {
                pending.pop_front();
            } else {
                caught_up = false;
                break;
            }
        }

        if (!caught_up) {
            // Socket buffer still full — add the new line to the per-client queue.
            pending.push_back(line);
            if (pending.size() > config_.max_client_pending_depth) {
                dead_clients.push_back(fd);  // back-buffer overflow: drop client
            }
            continue;
        }

        // Back-buffer is empty — send the new line directly.
        if (!send_all_nonblocking(fd, line)) {
            pending.push_back(line);
            if (pending.size() > config_.max_client_pending_depth) {
                dead_clients.push_back(fd);
            }
        }
    }

    if (!dead_clients.empty()) {
        std::lock_guard<std::mutex> lock{mutex_};
        for (const int fd : dead_clients) {
            auto it = std::find(client_fds_.begin(), client_fds_.end(), fd);
            if (it != client_fds_.end()) {
                close_fd(*it);
                client_fds_.erase(it);
                ++dropped_clients_;
            }
            client_pending_.erase(fd);
        }
    }

    std::lock_guard<std::mutex> lock{mutex_};
    publish_total_us_ += elapsed_us(start);
}

std::uint16_t RuntimeEventStreamServer::port() const {
    return config_.port;
}

RuntimeEventStreamServerStats RuntimeEventStreamServer::stats() const {
    std::lock_guard<std::mutex> lock{mutex_};
    return RuntimeEventStreamServerStats{
        .published_seq = published_seq_,
        .connected_clients = client_fds_.size(),
        .accepted_clients = accepted_clients_,
        .dropped_clients = dropped_clients_,
        .dropped_messages = dropped_messages_,
        .snapshot_messages = snapshot_messages_,
        .ghost_detection_messages = ghost_detection_messages_,
        .mission_event_messages = mission_event_messages_,
        .serialize_total_us = serialize_total_us_,
        .enqueue_total_us = enqueue_total_us_,
        .publish_total_us = publish_total_us_};
}

void RuntimeEventStreamServer::writer_loop() {
    while (true) {
        std::string line;
        {
            std::unique_lock<std::mutex> lock{mutex_};
            queue_cv_.wait(lock, [this] {
                return !send_queue_.empty() || !running_;
            });
            if (send_queue_.empty()) {
                break;
            }
            line = std::move(send_queue_.front());
            send_queue_.pop_front();
        }
        publish_json_line(line);
    }
}

void RuntimeEventStreamServer::accept_loop() {
    while (running_) {
        const int fd = listen_fd_;
        if (fd < 0) {
            break;
        }
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(fd, &read_fds);
        struct timeval tv{};
        tv.tv_usec = 100000;  // 100 ms
        const int ready = ::select(fd + 1, &read_fds, nullptr, nullptr, &tv);
        if (ready < 0) {
            if (errno == EINTR) {
                continue;
            }
            break;
        }
        if (ready == 0) {
            continue;
        }
        sockaddr_in client_addr{};
        socklen_t client_len = sizeof(client_addr);
        const int client_fd = ::accept(fd, reinterpret_cast<sockaddr*>(&client_addr), &client_len);
        if (client_fd < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                continue;
            }
            break;
        }
        set_nonblocking(client_fd);
        std::lock_guard<std::mutex> lock{mutex_};
        client_fds_.push_back(client_fd);
        ++accepted_clients_;
    }
}

void RuntimeEventStreamServer::close_listen_socket() {
    const int fd = listen_fd_;
    listen_fd_ = -1;
    close_fd(fd);
}

void RuntimeEventStreamServer::close_all_clients() {
    std::lock_guard<std::mutex> lock{mutex_};
    for (const int fd : client_fds_) {
        close_fd(fd);
    }
    client_fds_.clear();
    client_pending_.clear();
}

}  // namespace dedalus
