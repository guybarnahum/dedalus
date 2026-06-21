#include "dedalus/runtime/world_snapshot_stream_server.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
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


int create_listen_socket(const std::string& host, std::uint16_t port, int backlog, const char* label) {
    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        throw std::runtime_error(std::string(label) + " socket() failed: " + std::string(std::strerror(errno)));
    }

    int reuse = 1;
    (void)::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    set_nonblocking(fd);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
        close_fd(fd);
        throw std::runtime_error(std::string("invalid ") + label + " bind host: " + host);
    }

    if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        const auto message = std::string(std::strerror(errno));
        close_fd(fd);
        throw std::runtime_error(std::string("bind() failed for ") + label + ": " + message);
    }

    if (::listen(fd, backlog) < 0) {
        const auto message = std::string(std::strerror(errno));
        close_fd(fd);
        throw std::runtime_error(std::string("listen() failed for ") + label + ": " + message);
    }

    return fd;
}

std::uint16_t bound_port(int fd, std::uint16_t configured_port) {
    if (configured_port != 0 || fd < 0) {
        return configured_port;
    }
    sockaddr_in bound_addr{};
    socklen_t bound_len = sizeof(bound_addr);
    if (::getsockname(fd, reinterpret_cast<sockaddr*>(&bound_addr), &bound_len) == 0) {
        return ntohs(bound_addr.sin_port);
    }
    return configured_port;
}

bool send_blocking_best_effort(int fd, const std::string& payload) {
    const char* data = payload.data();
    std::size_t remaining = payload.size();
    while (remaining > 0U) {
        const auto sent = ::send(fd, data, remaining, MSG_NOSIGNAL);
        if (sent > 0) {
            data += sent;
            remaining -= static_cast<std::size_t>(sent);
            continue;
        }
        if (sent < 0 && errno == EINTR) {
            continue;
        }
        return false;
    }
    return true;
}


std::string http_request_path(const std::string& request) {
    const auto first_space = request.find(' ');
    if (first_space == std::string::npos) {
        return "/";
    }
    const auto second_space = request.find(' ', first_space + 1);
    if (second_space == std::string::npos || second_space <= first_space + 1) {
        return "/";
    }
    return request.substr(first_space + 1, second_space - first_space - 1);
}


std::string content_type_for_path(const std::filesystem::path& path) {
    const auto ext = path.extension().string();
    if (ext == ".html" || ext == ".htm") {
        return "text/html; charset=utf-8";
    }
    if (ext == ".js") {
        return "application/javascript; charset=utf-8";
    }
    if (ext == ".css") {
        return "text/css; charset=utf-8";
    }
    if (ext == ".json") {
        return "application/json; charset=utf-8";
    }
    if (ext == ".txt") {
        return "text/plain; charset=utf-8";
    }
    return "application/octet-stream";
}

bool path_is_within_root(const std::filesystem::path& candidate,
                         const std::filesystem::path& root) {
    auto rel = std::filesystem::relative(candidate, root);
    for (const auto& part : rel) {
        if (part == "..") {
            return false;
        }
    }
    return !rel.empty();
}

std::string http_response(std::string status,
                          std::string content_type,
                          const std::string& body) {
    return "HTTP/1.1 " + status + "\r\n"
           "Content-Type: " + content_type + "\r\n"
           "Cache-Control: no-cache\r\n"
           "Access-Control-Allow-Origin: *\r\n"
           "Content-Length: " + std::to_string(body.size()) + "\r\n"
           "\r\n" +
           body;
}

std::string healthz_body(std::uint16_t tcp_port, std::uint16_t http_port) {
    return std::string("{\"ok\":true,\"runtime_event_tcp_port\":") +
           std::to_string(tcp_port) +
           ",\"runtime_event_http_port\":" +
           std::to_string(http_port) +
           "}\n";
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

    try {
        listen_fd_ =
            create_listen_socket(config_.bind_host, config_.port, config_.listen_backlog, "runtime event stream");
        config_.port = bound_port(listen_fd_, config_.port);
    } catch (...) {
        close_listen_socket();
        running_ = false;
        throw;
    }

    if (config_.http_port != 0) {
        try {
            http_listen_fd_ = create_listen_socket(
                config_.http_bind_host,
                config_.http_port,
                config_.listen_backlog,
                "runtime event HTTP/SSE stream");
            config_.http_port = bound_port(http_listen_fd_, config_.http_port);
        } catch (const std::exception& exc) {
            close_http_listen_socket();
            std::cerr << "dedalus_mission_loop: warning: runtime event HTTP/SSE disabled: "
                      << exc.what() << "\n";
            config_.http_port = 0;
        }
    }

    accept_thread_ = std::thread([this]() { accept_loop(); });
    if (http_listen_fd_ >= 0) {
        http_accept_thread_ = std::thread([this]() { http_accept_loop(); });
    }
    writer_thread_ = std::thread([this]() { writer_loop(); });
}

void RuntimeEventStreamServer::stop() {
    if (!running_.exchange(false)) {
        return;
    }
    close_listen_socket();
    close_http_listen_socket();
    queue_cv_.notify_all();
    if (accept_thread_.joinable()) {
        accept_thread_.join();
    }
    if (http_accept_thread_.joinable()) {
        http_accept_thread_.join();
    }
    if (writer_thread_.joinable()) {
        writer_thread_.join();
    }
    close_all_clients();
}


std::uint16_t RuntimeEventStreamServer::port() const {
    return config_.port;
}

std::uint16_t RuntimeEventStreamServer::http_port() const {
    return config_.http_port;
}

RuntimeEventStreamServerStats RuntimeEventStreamServer::stats() const {
    std::lock_guard<std::mutex> lock{mutex_};
    return RuntimeEventStreamServerStats{
        .published_seq = published_seq_,
        .connected_clients = client_fds_.size(),
        .accepted_clients = accepted_clients_,
        .dropped_clients = dropped_clients_,
        .sse_clients = sse_client_fds_.size(),
        .accepted_sse_clients = accepted_sse_clients_,
        .dropped_sse_clients = dropped_sse_clients_,
        .dropped_messages = dropped_messages_,
        .snapshot_messages = snapshot_messages_,
        .ghost_detection_messages = ghost_detection_messages_,
        .mission_event_messages = mission_event_messages_,
        .mission_obstacle_map_delta_messages = mission_obstacle_map_delta_messages_,
        .traversability_map_snapshot_messages = traversability_map_snapshot_messages_,
        .planning_map_snapshot_messages = planning_map_snapshot_messages_,
        .esdf_snapshot_messages = esdf_snapshot_messages_,
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


void RuntimeEventStreamServer::http_accept_loop() {
    while (running_) {
        const int fd = http_listen_fd_;
        if (fd < 0) {
            break;
        }

        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(fd, &read_fds);
        struct timeval tv{};
        tv.tv_usec = 100000;
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

        char buffer[4096];
        const auto received = ::recv(client_fd, buffer, sizeof(buffer) - 1, 0);
        if (received <= 0) {
            close_fd(client_fd);
            continue;
        }
        buffer[received] = '\0';
        const std::string request{buffer, static_cast<std::size_t>(received)};
        const auto path = http_request_path(request);

        if (path == "/events") {
            const std::string headers =
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: text/event-stream; charset=utf-8\r\n"
                "Cache-Control: no-cache\r\n"
                "Connection: keep-alive\r\n"
                "Access-Control-Allow-Origin: *\r\n"
                "X-Accel-Buffering: no\r\n"
                "\r\n"
                ": connected to Dedalus runtime event stream\r\n\r\n";
            if (!send_blocking_best_effort(client_fd, headers)) {
                close_fd(client_fd);
                continue;
            }
            set_nonblocking(client_fd);
            {
                std::lock_guard<std::mutex> lock{mutex_};
                sse_client_fds_.push_back(client_fd);
                ++accepted_sse_clients_;
                // Reset trav watermark so the next throttled publish is a full
                // traversability_map_snapshot.  The new client has no prior state
                // and needs the complete map before incremental deltas make sense.
                trav_watermark_ns_ = 0U;
            }
            continue;
        }

        if (path == "/healthz") {
            const auto body = healthz_body(config_.port, config_.http_port);
            const std::string response =
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: application/json; charset=utf-8\r\n"
                "Cache-Control: no-cache\r\n"
                "Access-Control-Allow-Origin: *\r\n"
                "Content-Length: " + std::to_string(body.size()) + "\r\n"
                "\r\n" + body;
            (void)send_blocking_best_effort(client_fd, response);
            close_fd(client_fd);
            continue;
        }

        // serve static diagnostic file from configured root, if enabled
        if (!config_.http_static_root.empty()) {
            std::string rel = path;
            const auto query_pos = rel.find('?');
            if (query_pos != std::string::npos) {
                rel = rel.substr(0, query_pos);
            }
            while (!rel.empty() && rel.front() == '/') {
                rel.erase(rel.begin());
            }
            if (rel.empty()) {
                rel = config_.http_default_file;
            }

            const auto root = std::filesystem::weakly_canonical(std::filesystem::path{config_.http_static_root});
            const auto candidate = std::filesystem::weakly_canonical(root / rel);

            if (path_is_within_root(candidate, root) &&
                std::filesystem::exists(candidate) &&
                std::filesystem::is_regular_file(candidate)) {
                std::ifstream input{candidate, std::ios::binary};
                std::string body{
                    std::istreambuf_iterator<char>{input},
                    std::istreambuf_iterator<char>{}};
                const auto response = http_response("200 OK", content_type_for_path(candidate), body);
                (void)send_blocking_best_effort(client_fd, response);
                close_fd(client_fd);
                continue;
            }
        }

        const std::string body = "not found\n";
        const std::string response =
            "HTTP/1.1 404 Not Found\r\n"
            "Content-Type: text/plain; charset=utf-8\r\n"
            "Cache-Control: no-cache\r\n"
            "Access-Control-Allow-Origin: *\r\n"
            "Content-Length: " + std::to_string(body.size()) + "\r\n"
            "\r\n" + body;
        (void)send_blocking_best_effort(client_fd, response);
        close_fd(client_fd);
    }
}

void RuntimeEventStreamServer::close_listen_socket() {
    const int fd = listen_fd_;
    listen_fd_ = -1;
    close_fd(fd);
}

void RuntimeEventStreamServer::close_http_listen_socket() {
    const int fd = http_listen_fd_;
    http_listen_fd_ = -1;
    close_fd(fd);
}

void RuntimeEventStreamServer::close_all_clients() {
    std::lock_guard<std::mutex> lock{mutex_};
    for (const int fd : client_fds_) {
        close_fd(fd);
    }
    for (const int fd : sse_client_fds_) {
        close_fd(fd);
    }
    client_fds_.clear();
    sse_client_fds_.clear();
    client_pending_.clear();
    sse_client_pending_.clear();
}

}  // namespace dedalus
