#include "dedalus/runtime/world_snapshot_stream_server.hpp"

#include "dedalus/core/json_utils.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

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
