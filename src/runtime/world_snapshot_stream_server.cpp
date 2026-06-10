#include "dedalus/runtime/world_snapshot_stream_server.hpp"

#include <cerrno>
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
