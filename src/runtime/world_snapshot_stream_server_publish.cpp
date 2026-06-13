#include "dedalus/runtime/world_snapshot_stream_server.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <string>
#include <utility>

#include <sys/socket.h>
#include <unistd.h>

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

}  // namespace dedalus
