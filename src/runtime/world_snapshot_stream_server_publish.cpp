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


std::string event_name_from_json_line(const std::string& line) {
    const std::string needle = "\"type\":\"";
    const auto start = line.find(needle);
    if (start == std::string::npos) {
        return "runtime_event";
    }
    const auto value_start = start + needle.size();
    const auto value_end = line.find('"', value_start);
    if (value_end == std::string::npos || value_end <= value_start) {
        return "runtime_event";
    }
    auto name = line.substr(value_start, value_end - value_start);
    for (char& ch : name) {
        if (ch == '\n' || ch == '\r' || ch == ' ') {
            ch = '_';
        }
    }
    return name.empty() ? std::string{"runtime_event"} : name;
}

std::string trim_json_line(const std::string& line) {
    std::string out = line;
    while (!out.empty() && (out.back() == '\n' || out.back() == '\r')) {
        out.pop_back();
    }
    return out;
}

std::string sse_message_for_json_line(const std::string& line) {
    return std::string{"event: "} + event_name_from_json_line(line) +
           "\n" +
           "data: " + trim_json_line(line) +
           "\n\n";
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

    auto publish_to_clients = [this](
                                  const std::string& payload,
                                  std::vector<int>& client_fds,
                                  std::unordered_map<int, std::deque<std::string>>& pending_by_fd,
                                  std::uint64_t& dropped_counter) {
        std::vector<int> clients = client_fds;
        std::vector<int> dead_clients;

        for (const int fd : clients) {
            auto& pending = pending_by_fd[fd];
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
                pending.push_back(payload);
                if (pending.size() > config_.max_client_pending_depth) {
                    dead_clients.push_back(fd);
                }
                continue;
            }

            if (!send_all_nonblocking(fd, payload)) {
                pending.push_back(payload);
                if (pending.size() > config_.max_client_pending_depth) {
                    dead_clients.push_back(fd);
                }
            }
        }

        for (const int fd : dead_clients) {
            auto it = std::find(client_fds.begin(), client_fds.end(), fd);
            if (it != client_fds.end()) {
                close_fd(*it);
                client_fds.erase(it);
                ++dropped_counter;
            }
            pending_by_fd.erase(fd);
        }
    };

    const auto sse_payload = sse_message_for_json_line(line);

    {
        std::lock_guard<std::mutex> lock{mutex_};
        publish_to_clients(line, client_fds_, client_pending_, dropped_clients_);
        publish_to_clients(sse_payload, sse_client_fds_, sse_client_pending_, dropped_sse_clients_);
        publish_total_us_ += elapsed_us(start);
    }
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
