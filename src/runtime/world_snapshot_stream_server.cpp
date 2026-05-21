#include "dedalus/runtime/world_snapshot_stream_server.hpp"

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
#include <sys/socket.h>
#include <unistd.h>

#include "dedalus/world_model/world_snapshot.hpp"

namespace dedalus {
namespace {

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

std::string json_escape(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size() + 8U);
    for (const char ch : value) {
        switch (ch) {
            case '"':
                escaped += "\\\"";
                break;
            case '\\':
                escaped += "\\\\";
                break;
            case '\n':
                escaped += "\\n";
                break;
            case '\r':
                escaped += "\\r";
                break;
            case '\t':
                escaped += "\\t";
                break;
            default:
                escaped.push_back(ch);
                break;
        }
    }
    return escaped;
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
    line += to_json(snapshot);
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
    line += to_json(frame);
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
}

void RuntimeEventStreamServer::stop() {
    if (!running_.exchange(false)) {
        return;
    }
    close_listen_socket();
    if (accept_thread_.joinable()) {
        accept_thread_.join();
    }
    close_all_clients();
}

void RuntimeEventStreamServer::on_snapshot(const WorldSnapshot& snapshot) {
    std::uint64_t seq = 0;
    {
        std::lock_guard<std::mutex> lock{mutex_};
        seq = ++published_seq_;
    }
    publish_json_line(stream_line_for(seq, snapshot));
}

void RuntimeEventStreamServer::on_ghost_detections(const GhostDetectionsFrame& frame) {
    std::uint64_t seq = 0;
    {
        std::lock_guard<std::mutex> lock{mutex_};
        seq = ++published_seq_;
    }
    publish_json_line(stream_line_for(seq, frame));
}

void RuntimeEventStreamServer::publish_json_line(const std::string& line) {
    std::vector<int> clients;
    {
        std::lock_guard<std::mutex> lock{mutex_};
        clients = client_fds_;
    }

    if (clients.empty()) {
        return;
    }

    std::vector<int> dead_clients;
    for (const int fd : clients) {
        if (!send_all_nonblocking(fd, line)) {
            dead_clients.push_back(fd);
        }
    }

    if (dead_clients.empty()) {
        return;
    }

    std::lock_guard<std::mutex> lock{mutex_};
    for (const int fd : dead_clients) {
        auto it = std::find(client_fds_.begin(), client_fds_.end(), fd);
        if (it != client_fds_.end()) {
            close_fd(*it);
            client_fds_.erase(it);
            ++dropped_clients_;
        }
    }
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
        .dropped_clients = dropped_clients_};
}

void RuntimeEventStreamServer::accept_loop() {
    while (running_) {
        sockaddr_in client_addr{};
        socklen_t client_len = sizeof(client_addr);
        const int client_fd = ::accept(listen_fd_, reinterpret_cast<sockaddr*>(&client_addr), &client_len);
        if (client_fd >= 0) {
            try {
                set_nonblocking(client_fd);
                std::lock_guard<std::mutex> lock{mutex_};
                client_fds_.push_back(client_fd);
                ++accepted_clients_;
            } catch (...) {
                close_fd(client_fd);
            }
            continue;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR || errno == EBADF) {
            std::this_thread::sleep_for(std::chrono::milliseconds{20});
            continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{20});
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
}

}  // namespace dedalus
