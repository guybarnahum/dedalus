#include "dedalus/runtime/world_snapshot_stream_server.hpp"

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

// Send as much of payload as the socket will accept without blocking.
// Returns true if the entire payload was sent.
// On false: *bytes_sent_out contains the number of bytes actually written
// before EAGAIN or error.  The caller must store only the unsent tail
// (payload.substr(*bytes_sent_out)) in the pending queue — re-enqueueing
// the full payload would double-write the already-sent prefix and corrupt
// the SSE stream (root cause of the JSON parse errors on large ESDF events).
bool send_all_nonblocking(int fd, const std::string& payload,
                          std::size_t* bytes_sent_out = nullptr) {
    const char* data = payload.data();
    std::size_t remaining = payload.size();
    std::size_t sent_total = 0U;
    while (remaining > 0U) {
        const auto sent = ::send(fd, data, remaining, MSG_NOSIGNAL | MSG_DONTWAIT);
        if (sent > 0) {
            data       += sent;
            remaining  -= static_cast<std::size_t>(sent);
            sent_total += static_cast<std::size_t>(sent);
            continue;
        }
        if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            if (bytes_sent_out) *bytes_sent_out = sent_total;
            return false;
        }
        if (sent < 0 && errno == EINTR) {
            continue;
        }
        if (bytes_sent_out) *bytes_sent_out = sent_total;
        return false;
    }
    if (bytes_sent_out) *bytes_sent_out = sent_total;
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

void RuntimeEventStreamServer::enqueue_item(QueueItem item) {
    const auto start = SteadyClock::now();
    {
        std::lock_guard<std::mutex> lock{mutex_};
        if (send_queue_.size() >= config_.max_send_queue_depth) {
            send_queue_.pop_front();  // drop oldest; bounds memory under slow consumers
            ++dropped_messages_;
        }
        send_queue_.push_back(std::move(item));
        enqueue_total_us_ += elapsed_us(start);
    }
    queue_cv_.notify_one();
}

void RuntimeEventStreamServer::enqueue_line(std::string line) {
    enqueue_item(std::move(line));
}

void RuntimeEventStreamServer::enqueue_snapshot(
    std::uint64_t seq, std::shared_ptr<const WorldSnapshot> snapshot) {
    enqueue_item(PendingSnapshot{seq, std::move(snapshot)});
}

void RuntimeEventStreamServer::enqueue_traversability_snapshot(
    std::uint64_t seq, std::uint64_t timestamp_ns,
    MissionLocalTraversabilityMapSnapshot snapshot) {
    enqueue_item(PendingTravSnapshot{seq, timestamp_ns, std::move(snapshot)});
}

void RuntimeEventStreamServer::enqueue_planning_snapshot(
    std::uint64_t seq, std::uint64_t timestamp_ns,
    MissionLocalPlanningMapSnapshot snapshot) {
    enqueue_item(PendingPlanningSnapshot{seq, timestamp_ns, std::move(snapshot)});
}

void RuntimeEventStreamServer::enqueue_esdf_snapshot(
    std::uint64_t seq, std::uint64_t timestamp_ns,
    LocalESDFMapSnapshot snapshot) {
    enqueue_item(PendingESDFSnapshot{seq, timestamp_ns, std::move(snapshot)});
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
                std::size_t n_sent = 0U;
                if (send_all_nonblocking(fd, pending.front(), &n_sent)) {
                    pending.pop_front();
                } else {
                    // Trim the already-sent prefix so the retry starts from
                    // the right byte, not from byte 0.  Re-sending from byte 0
                    // would corrupt the SSE stream for large (multi-MB) events.
                    if (n_sent > 0U) {
                        pending.front().erase(0U, n_sent);
                    }
                    caught_up = false;
                    break;
                }
            }

            if (!caught_up) {
                // Socket buffer still full — enqueue the whole new payload.
                pending.push_back(payload);
                if (pending.size() > config_.max_client_pending_depth) {
                    dead_clients.push_back(fd);
                }
                continue;
            }

            {
                std::size_t n_sent = 0U;
                if (!send_all_nonblocking(fd, payload, &n_sent)) {
                    // Enqueue only the unsent tail.
                    pending.push_back(
                        n_sent > 0U ? payload.substr(n_sent) : payload);
                    if (pending.size() > config_.max_client_pending_depth) {
                        dead_clients.push_back(fd);
                    }
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
        QueueItem item;
        {
            std::unique_lock<std::mutex> lock{mutex_};
            queue_cv_.wait(lock, [this] {
                return !send_queue_.empty() || !running_;
            });
            if (send_queue_.empty()) {
                break;
            }
            item = std::move(send_queue_.front());
            send_queue_.pop_front();
        }

        // For PendingSnapshots, PendingTravSnapshots, and PendingPlanningSnapshots,
        // serialize here on the writer thread so that the expensive
        // to_json() / to_compact_stream_json() never runs on the hot path.
        // All other message types are pre-serialized strings.
        bool already_published = false;
        std::string line;
        if (std::holds_alternative<std::string>(item)) {
            line = std::move(std::get<std::string>(item));
        } else if (std::holds_alternative<PendingSnapshot>(item)) {
            auto& pending = std::get<PendingSnapshot>(item);
            const auto t0 = SteadyClock::now();
            line = serialize_snapshot(pending.seq, *pending.snapshot);
            const auto serialize_us = elapsed_us(t0);
            std::lock_guard<std::mutex> lock{mutex_};
            serialize_total_us_ += serialize_us;
        } else if (std::holds_alternative<PendingTravSnapshot>(item)) {
            auto& pending = std::get<PendingTravSnapshot>(item);
            const auto t0 = SteadyClock::now();
            line = serialize_traversability_snapshot(
                pending.seq, pending.timestamp_ns, pending.snapshot);
            const auto serialize_us = elapsed_us(t0);
            std::lock_guard<std::mutex> lock{mutex_};
            serialize_total_us_ += serialize_us;
        } else if (std::holds_alternative<PendingPlanningSnapshot>(item)) {
            auto& pending = std::get<PendingPlanningSnapshot>(item);
            const auto t0 = SteadyClock::now();
            line = serialize_planning_snapshot(
                pending.seq, pending.timestamp_ns, pending.snapshot);
            const auto serialize_us = elapsed_us(t0);
            {
                std::lock_guard<std::mutex> lock{mutex_};
                serialize_total_us_ += serialize_us;
                // Cache for replay to new SSE clients (L2 is always a full snapshot).
                last_planning_sse_ = sse_message_for_json_line(line);
            }
        } else {
            auto& pending = std::get<PendingESDFSnapshot>(item);
            static constexpr std::size_t kChunkCells = 500U;
            const std::size_t total = pending.snapshot.cells.size();

            if (total <= kChunkCells) {
                // Small enough to publish as a single SSE event.
                const auto t0 = SteadyClock::now();
                line = serialize_esdf_snapshot(
                    pending.seq, pending.timestamp_ns, pending.snapshot);
                const auto serialize_us = elapsed_us(t0);
                {
                    std::lock_guard<std::mutex> lock{mutex_};
                    serialize_total_us_ += serialize_us;
                    if (!pending.snapshot.is_delta) {
                        last_esdf_sse_ = sse_message_for_json_line(line);
                    }
                }
                // fall through to the publish_json_line(line) call below
            } else {
                // Large snapshot: split into kChunkCells-cell SSE events so no
                // single event exceeds ~55 KB.  This eliminates the partial-send
                // path for large ESDF maps (900 KB+) where the trailing \n\n event
                // terminator gets stranded in the pending queue; the next event's
                // "event: ..." header is then received by the client inside the
                // data field of the previous event, corrupting the JSON.
                //
                // First chunk carries the original is_delta flag (false = client
                // clears ESDF and starts fresh).  All subsequent chunks are deltas
                // (merge into the just-cleared map).
                //
                // For full snapshots, last_esdf_sse_ is set to the concatenation
                // of all chunk SSE messages so new-client replay sends the full map.
                const bool build_cache = !pending.snapshot.is_delta;
                std::string cache_buf;
                if (build_cache) {
                    cache_buf.reserve(total * 116U);
                }
                std::uint64_t total_serialize_us = 0U;

                for (std::size_t start = 0U; start < total; start += kChunkCells) {
                    LocalESDFMapSnapshot chunk;
                    chunk.config        = pending.snapshot.config;
                    chunk.net_repulsion = pending.snapshot.net_repulsion;
                    chunk.seq           = pending.snapshot.seq;
                    chunk.is_delta      = (start == 0U)
                                          ? pending.snapshot.is_delta
                                          : true;
                    const std::size_t end = std::min(start + kChunkCells, total);
                    chunk.cells.assign(
                        pending.snapshot.cells.cbegin() +
                            static_cast<std::ptrdiff_t>(start),
                        pending.snapshot.cells.cbegin() +
                            static_cast<std::ptrdiff_t>(end));
                    chunk.cell_count = chunk.cells.size();

                    const auto t0 = SteadyClock::now();
                    const auto chunk_line = serialize_esdf_snapshot(
                        pending.seq, pending.timestamp_ns, chunk);
                    total_serialize_us += elapsed_us(t0);

                    if (build_cache) {
                        cache_buf += sse_message_for_json_line(chunk_line);
                    }
                    publish_json_line(chunk_line);
                }

                {
                    std::lock_guard<std::mutex> lock{mutex_};
                    serialize_total_us_ += total_serialize_us;
                    if (build_cache && !cache_buf.empty()) {
                        last_esdf_sse_ = std::move(cache_buf);
                    }
                }
                already_published = true;
            }
        }
        if (!already_published) {
            publish_json_line(line);
        }
    }
}

}  // namespace dedalus
