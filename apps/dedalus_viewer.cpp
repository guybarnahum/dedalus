// dedalus_viewer — standalone live/replay viewer for dedalus mission streams.
//
// Connects to a dedalus RuntimeEventStreamServer TCP JSONL port and relays the
// event stream to browser clients via HTTP/SSE.  Auto-switches between live and
// replay sources: when the live connection is lost the viewer falls back to
// replaying snapshots from an artifact directory; it reconnects to live as soon
// as the runtime is reachable again.
//
// Usage:
//   dedalus_viewer [options]
//
// Options:
//   --host HOST              Runtime TCP host       (default: 127.0.0.1)
//   --port PORT              Runtime TCP port       (default: 47770)
//   --http-port PORT         Viewer HTTP/SSE port   (default: 8090)
//   --replay-dir DIR         Snapshot artifact dir  (default: disabled)
//   --reconnect-s N          Live reconnect interval in seconds (default: 3)
//   --replay-speed F         Replay speed multiplier (default: 1.0)
//   --static-root DIR        Directory to serve static files from (default: disabled)
//   --static-default-file F  Default file for GET / (default: viewer.html)
//   -h / --help
//
// Browser endpoints:
//   GET /events          — SSE stream of all mission events
//   GET /viewer_status   — JSON source status (live/replay/disconnected)
//   GET /healthz         — JSON health check
//   GET /               — Serves --static-default-file (when --static-root is set)
//   GET /<path>          — Serves static file relative to --static-root

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <optional>
#include <unordered_map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <sqlite3.h>

// MSG_NOSIGNAL / MSG_DONTWAIT are Linux extensions; define as 0 on platforms
// that don't have them (SIGPIPE is ignored at startup anyway).
#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif
#ifndef MSG_DONTWAIT
#define MSG_DONTWAIT 0
#endif

namespace {

// ── shared queue ─────────────────────────────────────────────────────────────
//
// One producer (SourceMonitor) writes JSONL lines; one consumer (SseRelay
// writer thread) reads them.  Oldest entries are dropped on overflow.

class SharedQueue {
public:
    explicit SharedQueue(std::size_t max_depth = 512) : max_depth_(max_depth) {}

    void push(std::string line) {
        std::lock_guard<std::mutex> lock{mutex_};
        if (queue_.size() >= max_depth_) {
            queue_.pop_front();
            ++dropped_;
        }
        queue_.push_back(std::move(line));
        cv_.notify_one();
    }

    // Blocks up to 100 ms waiting for a line; returns nullopt when nothing
    // arrives within the window or when running becomes false.
    std::optional<std::string> pop(const std::atomic<bool>& running) {
        std::unique_lock<std::mutex> lock{mutex_};
        cv_.wait_for(lock, std::chrono::milliseconds{100},
            [this, &running] { return !queue_.empty() || !running.load(); });
        if (queue_.empty()) return std::nullopt;
        auto line = std::move(queue_.front());
        queue_.pop_front();
        return line;
    }

    std::uint64_t dropped() const {
        std::lock_guard<std::mutex> lock{mutex_};
        return dropped_;
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<std::string> queue_;
    std::size_t max_depth_;
    std::uint64_t dropped_{0};
};

// ── source status ─────────────────────────────────────────────────────────────

enum class SourceStatus : int { Disconnected = 0, Live = 1, Replay = 2 };

const char* source_status_cstr(SourceStatus s) {
    switch (s) {
        case SourceStatus::Live:   return "live";
        case SourceStatus::Replay: return "replay";
        default:                   return "disconnected";
    }
}

// ── replay reader helpers ─────────────────────────────────────────────────────

struct ReplayFrame {
    int index{0};
    std::string path;
    std::int64_t timestamp_ns{0};
    std::string map_frame_id;
};

// Parses snapshot_manifest.txt from an artifact directory.
// Lines: "# comment" or "<index> <path> <timestamp_ns> <map_frame_id>"
std::vector<ReplayFrame> load_manifest(const std::string& dir) {
    std::vector<ReplayFrame> frames;
    std::ifstream f{dir + "/snapshot_manifest.txt"};
    if (!f) return frames;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss{line};
        ReplayFrame rf;
        std::string rel_path;
        if (ss >> rf.index >> rel_path >> rf.timestamp_ns >> rf.map_frame_id) {
            rf.path = dir + "/" + rel_path;
            frames.push_back(std::move(rf));
        }
    }
    return frames;
}

std::string read_file_contents(const std::string& path) {
    std::ifstream f{path};
    if (!f) return "";
    std::ostringstream buf;
    buf << f.rdbuf();
    return buf.str();
}

// Minimal JSON string escaping (only characters that appear in field values).
std::string json_escape(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (const char c : s) {
        switch (c) {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n";  break;
            case '\r': out += "\\r";  break;
            case '\t': out += "\\t";  break;
            default:   out += c;
        }
    }
    return out;
}

// Wraps a raw snapshot JSON blob in the same JSONL envelope that
// RuntimeEventStreamServer emits for world_snapshot messages.
std::string make_replay_line(std::uint64_t seq, const ReplayFrame& frame,
                             const std::string& snapshot_json) {
    std::string body = snapshot_json;
    // Strip trailing whitespace so the envelope terminates cleanly.
    while (!body.empty() && (body.back() == '\n' || body.back() == '\r' ||
                              body.back() == ' ')) {
        body.pop_back();
    }
    std::string line;
    line.reserve(body.size() + 128U);
    line += "{\"type\":\"world_snapshot\",\"seq\":";
    line += std::to_string(seq);
    line += ",\"timestamp_ns\":";
    line += std::to_string(frame.timestamp_ns);
    line += ",\"active_map_frame_id\":\"";
    line += json_escape(frame.map_frame_id);
    line += "\",\"snapshot\":";
    line += body;
    line += "}\n";
    return line;
}

// ── TCP line client ───────────────────────────────────────────────────────────
//
// Manages a single blocking TCP connection.  connect_now() does a
// non-blocking connect with a 2-second timeout, then switches the socket back
// to blocking mode for reads.  read_line() returns the next \n-terminated line
// or nullopt on EOF/error.

class TcpLineClient {
public:
    TcpLineClient(std::string host, std::uint16_t port)
        : host_(std::move(host)), port_(port) {}

    ~TcpLineClient() { close_fd(); }

    bool connect_now() {
        close_fd();
        fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (fd_ < 0) return false;

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port_);
        if (::inet_pton(AF_INET, host_.c_str(), &addr.sin_addr) != 1) {
            close_fd();
            return false;
        }

        // Non-blocking connect so we can apply a short timeout.
        const int orig_flags = ::fcntl(fd_, F_GETFL, 0);
        ::fcntl(fd_, F_SETFL, orig_flags | O_NONBLOCK);

        const int rc = ::connect(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
        if (rc < 0 && errno != EINPROGRESS) {
            close_fd();
            return false;
        }

        fd_set wfds;
        FD_ZERO(&wfds);
        FD_SET(fd_, &wfds);
        timeval tv{2, 0};  // 2-second connect timeout
        if (::select(fd_ + 1, nullptr, &wfds, nullptr, &tv) != 1) {
            close_fd();
            return false;
        }
        int err = 0;
        socklen_t len = sizeof(err);
        ::getsockopt(fd_, SOL_SOCKET, SO_ERROR, &err, &len);
        if (err != 0) {
            close_fd();
            return false;
        }

        // Restore blocking mode for reads.
        ::fcntl(fd_, F_SETFL, orig_flags & ~O_NONBLOCK);
        buf_.clear();
        return true;
    }

    // Blocks until a complete \n-terminated line is available.
    // Returns nullopt on EOF or socket error.
    std::optional<std::string> read_line() {
        while (true) {
            const auto nl = buf_.find('\n');
            if (nl != std::string::npos) {
                auto line = buf_.substr(0, nl + 1);
                buf_.erase(0, nl + 1);
                return line;
            }
            char chunk[8192];
            const auto n = ::recv(fd_, chunk, sizeof(chunk), 0);
            if (n <= 0) return std::nullopt;
            buf_.append(chunk, static_cast<std::size_t>(n));
        }
    }

    void disconnect() { close_fd(); }

private:
    void close_fd() {
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
        buf_.clear();
    }

    std::string host_;
    std::uint16_t port_;
    int fd_{-1};
    std::string buf_;
};

// ── source monitor ────────────────────────────────────────────────────────────
//
// Runs a dedicated thread that drives the shared queue with JSONL lines.
// Priority order:
//   1. Live — TCP connected to the runtime stream server.
//   2. Replay — reads artifact snapshots when live is unavailable.
//   3. Disconnected — sleeps reconnect_s seconds, then retries live.
//
// After replay completes the monitor retries the live connection immediately.

class SourceMonitor {
public:
    SourceMonitor(std::string live_host, std::uint16_t live_port,
                  std::string replay_dir, double replay_speed,
                  int reconnect_s, SharedQueue& queue)
        : live_host_(std::move(live_host)),
          live_port_(live_port),
          replay_dir_(std::move(replay_dir)),
          replay_speed_(replay_speed > 0.0 ? replay_speed : 1.0),
          reconnect_s_(reconnect_s),
          queue_(queue) {}

    void start() {
        running_ = true;
        thread_ = std::thread([this] { run(); });
    }

    void stop() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
    }

    SourceStatus status() const {
        return static_cast<SourceStatus>(status_.load());
    }
    std::uint64_t seq()       const { return seq_.load(); }
    int replay_frame()        const { return replay_frame_.load(); }
    int replay_total()        const { return replay_total_.load(); }

private:
    void run() {
        TcpLineClient tcp{live_host_, live_port_};

        while (running_) {
            // ── live ──────────────────────────────────────────────────────
            if (tcp.connect_now()) {
                status_ = static_cast<int>(SourceStatus::Live);
                while (running_) {
                    auto line = tcp.read_line();
                    if (!line) break;
                    ++seq_;
                    queue_.push(std::move(*line));
                }
                tcp.disconnect();
            }
            if (!running_) break;
            status_ = static_cast<int>(SourceStatus::Disconnected);

            // ── replay (if configured) ────────────────────────────────────
            if (!replay_dir_.empty()) {
                do_replay();
            } else {
                sleep_chunked(std::chrono::seconds{reconnect_s_});
            }
            // Loop back and try live again.
        }
    }

    void do_replay() {
        const auto frames = load_manifest(replay_dir_);
        if (frames.empty()) {
            sleep_chunked(std::chrono::seconds{reconnect_s_});
            return;
        }

        status_ = static_cast<int>(SourceStatus::Replay);
        replay_total_ = static_cast<int>(frames.size());

        for (std::size_t i = 0; i < frames.size() && running_; ++i) {
            replay_frame_ = static_cast<int>(i + 1);

            const auto& frame = frames[i];
            const auto json = read_file_contents(frame.path);
            if (json.empty()) continue;

            queue_.push(make_replay_line(++seq_, frame, json));

            // Wait the inter-frame interval scaled by replay speed.
            if (i + 1 < frames.size()) {
                const double raw_delta_ns = static_cast<double>(
                    frames[i + 1].timestamp_ns - frames[i].timestamp_ns);
                const auto wait_ns = static_cast<long long>(
                    std::max(0.0, raw_delta_ns / replay_speed_));
                // Cap individual waits at 5 s so the monitor stays responsive.
                const auto wait_ms = std::min(wait_ns / 1'000'000LL, 5'000LL);
                sleep_chunked(std::chrono::milliseconds{wait_ms});
            }
        }

        // Brief pause before the outer loop retries the live connection.
        sleep_chunked(std::chrono::milliseconds{500});
    }

    // Sleeps for the given duration in 50 ms chunks so we remain responsive
    // to shutdown requests.
    template <typename Duration>
    void sleep_chunked(Duration total) {
        const auto deadline = std::chrono::steady_clock::now() + total;
        while (running_ && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds{50});
        }
    }

    std::string live_host_;
    std::uint16_t live_port_;
    std::string replay_dir_;
    double replay_speed_;
    int reconnect_s_;
    SharedQueue& queue_;

    std::atomic<bool> running_{false};
    std::atomic<int> status_{static_cast<int>(SourceStatus::Disconnected)};
    std::atomic<std::uint64_t> seq_{0};
    std::atomic<int> replay_frame_{0};
    std::atomic<int> replay_total_{0};
    std::thread thread_;
};

// ── static file helpers ───────────────────────────────────────────────────────

// Returns a MIME content-type string for a file extension, defaulting to
// application/octet-stream for unknown types.
std::string content_type_for_path(const std::string& path) {
    const auto dot = path.rfind('.');
    if (dot == std::string::npos) return "application/octet-stream";
    const auto ext = path.substr(dot);
    if (ext == ".html" || ext == ".htm") return "text/html; charset=utf-8";
    if (ext == ".js")                    return "application/javascript; charset=utf-8";
    if (ext == ".css")                   return "text/css; charset=utf-8";
    if (ext == ".json")                  return "application/json; charset=utf-8";
    if (ext == ".svg")                   return "image/svg+xml";
    if (ext == ".png")                   return "image/png";
    if (ext == ".ico")                   return "image/x-icon";
    if (ext == ".txt")                   return "text/plain; charset=utf-8";
    return "application/octet-stream";
}

// Returns true iff `canonical` is under `root` (both must be canonical paths).
// Prevents path-traversal attacks (e.g. GET /../secret).
bool path_is_within_root(const std::filesystem::path& root,
                          const std::filesystem::path& canonical) {
    const auto root_str      = root.string();
    const auto canonical_str = canonical.string();
    return canonical_str.size() >= root_str.size() &&
           canonical_str.substr(0, root_str.size()) == root_str &&
           (canonical_str.size() == root_str.size() ||
            canonical_str[root_str.size()] == '/');
}

// ── SSE relay ─────────────────────────────────────────────────────────────────
//
// Minimal HTTP server that:
//   GET /events        — upgrades to an SSE connection and streams events
//   GET /viewer_status — returns JSON describing the current source
//   GET /healthz       — returns {"ok":true}
//   (all other paths)  — 404
//
// SSE client sockets are set to non-blocking; slow clients that can't keep up
// are dropped rather than stalling the writer thread.

// Extracts the "type" field value from a JSONL line produced by the runtime
// stream server, used as the SSE event name.
std::string extract_event_type(const std::string& line) {
    const auto pos = line.find("\"type\":\"");
    if (pos == std::string::npos) return "event";
    const auto start = pos + 8U;
    const auto end   = line.find('"', start);
    if (end == std::string::npos) return "event";
    return line.substr(start, end - start);
}

// Converts a JSONL line to an SSE frame.
std::string make_sse_frame(const std::string& jsonl_line) {
    std::string data = jsonl_line;
    while (!data.empty() && (data.back() == '\n' || data.back() == '\r')) {
        data.pop_back();
    }
    std::string frame;
    frame.reserve(data.size() + 64U);
    frame += "event: ";
    frame += extract_event_type(data);
    frame += "\r\ndata: ";
    frame += data;
    frame += "\r\n\r\n";
    return frame;
}

// ── L2 static cache ───────────────────────────────────────────────────────────
//
// Reads the L2 planning map from a SQLite DB at startup and builds a single
// planning_map_snapshot SSE frame that is pushed to every new SSE client
// immediately on connect — before the live stream starts flowing.
//
// This means the viewer always shows the saved obstacle map even when the
// mission_loop is not running.  When the live runtime connects it will send
// incremental planning_map_delta updates that merge on top of this baseline.

class L2StaticCache {
public:
    // Returns true on success (including when db_path is empty — no-op).
    bool load(const std::string& db_path,
              double cell_m, double vcell_m, double min_score) {
        if (db_path.empty()) return true;

        sqlite3* db = nullptr;
        if (sqlite3_open_v2(db_path.c_str(), &db,
                            SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK) {
            std::fprintf(stderr,
                "[dedalus_viewer] cannot open L2 DB '%s': %s\n",
                db_path.c_str(), sqlite3_errmsg(db));
            sqlite3_close(db);
            return false;
        }

        const char* sql =
            "SELECT xi, yi, zi, score, confidence, count, updated_ns"
            "  FROM cells WHERE score >= ?"
            "  ORDER BY xi, yi, zi;";
        sqlite3_stmt* stmt = nullptr;
        if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK) {
            std::fprintf(stderr,
                "[dedalus_viewer] L2 DB '%s': prepare failed: %s\n",
                db_path.c_str(), sqlite3_errmsg(db));
            sqlite3_close(db);
            return false;
        }
        sqlite3_bind_double(stmt, 1, min_score);

        struct RawCell {
            int xi, yi, zi;
            float score, conf;
            std::uint32_t count;
            std::int64_t  ts;
        };
        std::vector<RawCell> cells;
        while (sqlite3_step(stmt) == SQLITE_ROW) {
            cells.push_back({
                sqlite3_column_int(stmt, 0),
                sqlite3_column_int(stmt, 1),
                sqlite3_column_int(stmt, 2),
                static_cast<float>(sqlite3_column_double(stmt, 3)),
                static_cast<float>(sqlite3_column_double(stmt, 4)),
                static_cast<std::uint32_t>(sqlite3_column_int(stmt, 5)),
                sqlite3_column_int64(stmt, 6),
            });
        }
        sqlite3_finalize(stmt);
        sqlite3_close(db);

        std::fprintf(stderr,
            "[dedalus_viewer] L2 DB '%s': %zu cells loaded\n",
            db_path.c_str(), cells.size());

        if (cells.empty()) return true;

        // Serialize into a single planning_map_snapshot JSONL line — same
        // schema as to_compact_stream_json() / on_planning_map_snapshot().
        std::ostringstream pay;
        pay.precision(6);
        pay << "{\"schema\":\"dedalus.mission_local_planning_map.v1\"";
        pay << ",\"cell_size_m\":"          << cell_m;
        pay << ",\"vertical_cell_size_m\":" << vcell_m;
        pay << ",\"cell_count\":"           << cells.size();
        pay << ",\"total_cells\":"          << cells.size();
        pay << ",\"exported_cells\":"       << cells.size();
        pay << ",\"cells\":[";
        for (std::size_t i = 0; i < cells.size(); ++i) {
            const auto& c = cells[i];
            if (i > 0) pay << ',';
            const double cx = (static_cast<double>(c.xi) + 0.5) * cell_m;
            const double cy = (static_cast<double>(c.yi) + 0.5) * cell_m;
            const double cz = (static_cast<double>(c.zi) + 0.5) * vcell_m;
            pay << "{\"center_map\":[" << cx << ',' << cy << ',' << cz << ']';
            pay << ",\"occupied_score\":"    << c.score;
            pay << ",\"confidence\":"        << c.conf;
            pay << ",\"source_cell_count\":" << c.count;
            if (c.ts > 0) {
                pay << ",\"t\":" << (c.ts / 1'000'000'000LL);
            }
            pay << '}';
        }
        pay << "]}";

        std::string line;
        line.reserve(pay.str().size() + 128U);
        line += "{\"type\":\"planning_map_snapshot\""
                ",\"seq\":0,\"map_seq\":0,\"timestamp_ns\":0"
                ",\"planning_map_snapshot\":";
        line += pay.str();
        line += "}\n";

        burst_.push_back(make_sse_frame(line));
        return true;
    }

    const std::vector<std::string>& burst() const { return burst_; }
    bool empty() const { return burst_.empty(); }

private:
    std::vector<std::string> burst_;  // SSE frames to push on each new connect
};

// Best-effort blocking send.  Returns false if the connection appears broken.
bool send_all(int fd, const char* data, std::size_t size) {
    while (size > 0) {
        const auto n = ::send(fd, data, size, MSG_NOSIGNAL);
        if (n <= 0) {
            if (errno == EINTR) continue;
            return false;
        }
        data += n;
        size -= static_cast<std::size_t>(n);
    }
    return true;
}

bool send_all(int fd, const std::string& s) {
    return send_all(fd, s.data(), s.size());
}

class SseRelay {
public:
    SseRelay(std::string bind_host, std::uint16_t http_port,
             SharedQueue& queue, const SourceMonitor& monitor,
             std::string static_root = {},
             std::string static_default_file = "viewer.html",
             std::string l2_db = {},
             double l2_cell_m = 1.0,
             double l2_vcell_m = 2.0,
             double l2_min_score = 0.5)
        : bind_host_(std::move(bind_host)),
          http_port_(http_port),
          queue_(queue),
          monitor_(monitor),
          static_root_(std::move(static_root)),
          static_default_file_(std::move(static_default_file)),
          l2_db_(std::move(l2_db)),
          l2_cell_m_(l2_cell_m),
          l2_vcell_m_(l2_vcell_m),
          l2_min_score_(l2_min_score) {}

    void start() {
        setup_listen_socket();
        running_ = true;
        accept_thread_ = std::thread([this] { accept_loop(); });
        writer_thread_ = std::thread([this] { writer_loop(); });
    }

    void stop() {
        running_ = false;
        close_listen_socket();
        if (accept_thread_.joinable()) accept_thread_.join();
        {
            std::lock_guard<std::mutex> lock{clients_mutex_};
            for (const int fd : sse_clients_) ::close(fd);
            sse_clients_.clear();
            sse_pending_.clear();
        }
        if (writer_thread_.joinable()) writer_thread_.join();
    }

    std::uint16_t bound_port() const { return bound_port_; }

private:
    void setup_listen_socket() {
        listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (listen_fd_ < 0) throw std::runtime_error("socket() failed");

        const int opt = 1;
        ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port   = htons(http_port_);
        if (::inet_pton(AF_INET, bind_host_.c_str(), &addr.sin_addr) != 1) {
            throw std::runtime_error("inet_pton failed for " + bind_host_);
        }
        if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            throw std::runtime_error("bind() failed on port " + std::to_string(http_port_));
        }
        if (::listen(listen_fd_, 8) < 0) {
            throw std::runtime_error("listen() failed");
        }

        sockaddr_in bound{};
        socklen_t   len = sizeof(bound);
        ::getsockname(listen_fd_, reinterpret_cast<sockaddr*>(&bound), &len);
        bound_port_ = ntohs(bound.sin_port);
    }

    void accept_loop() {
        while (running_) {
            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(listen_fd_, &rfds);
            timeval tv{0, 200'000};  // 200 ms poll interval
            if (::select(listen_fd_ + 1, &rfds, nullptr, nullptr, &tv) <= 0) continue;

            const int client = ::accept(listen_fd_, nullptr, nullptr);
            if (client < 0) continue;

            // Read just enough of the HTTP request to route it.
            char req_buf[2048];
            const auto n = ::recv(client, req_buf, sizeof(req_buf) - 1, 0);
            if (n <= 0) { ::close(client); continue; }
            req_buf[n] = '\0';
            const std::string req{req_buf};

            if (req.find("GET /events") != std::string::npos) {
                handle_sse_upgrade(client);
            } else if (req.find("GET /viewer_status") != std::string::npos) {
                respond_json(client, viewer_status_json());
            } else if (req.find("GET /healthz") != std::string::npos) {
                respond_json(client, "{\"ok\":true}\n");
            } else if (!static_root_.empty()) {
                // Extract the request path from the first line (GET <path> HTTP/1.1).
                std::string req_path;
                const auto sp1 = req.find(' ');
                if (sp1 != std::string::npos) {
                    const auto sp2 = req.find(' ', sp1 + 1);
                    req_path = req.substr(sp1 + 1,
                                          sp2 == std::string::npos ? std::string::npos
                                                                    : sp2 - sp1 - 1);
                }
                // Strip query string.
                const auto qm = req_path.find('?');
                if (qm != std::string::npos) req_path = req_path.substr(0, qm);
                // Map / to the default file.
                if (req_path.empty() || req_path == "/") {
                    req_path = "/" + static_default_file_;
                }
                respond_static_file(client, req_path);
            } else {
                respond_not_found(client);
            }
        }
    }

    void handle_sse_upgrade(int client) {
        std::fprintf(stderr, "[dedalus_viewer] new SSE client (l2_db='%s')\n",
                     l2_db_.c_str());
        const std::string headers =
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: text/event-stream\r\n"
            "Cache-Control: no-cache\r\n"
            "Access-Control-Allow-Origin: *\r\n"
            "Connection: keep-alive\r\n\r\n";
        if (!send_all(client, headers)) {
            ::close(client);
            return;
        }
        // Load the L2 DB fresh on every new connection so the browser always
        // gets the current state — including data written after viewer startup.
        if (!l2_db_.empty()) {
            L2StaticCache cache;
            if (cache.load(l2_db_, l2_cell_m_, l2_vcell_m_, l2_min_score_)) {
                for (const auto& frame : cache.burst()) {
                    if (!send_all(client, frame)) {
                        ::close(client);
                        return;
                    }
                }
            }
        }
        // Switch to non-blocking so slow clients don't stall the writer.
        const int flags = ::fcntl(client, F_GETFL, 0);
        ::fcntl(client, F_SETFL, flags | O_NONBLOCK);

        std::lock_guard<std::mutex> lock{clients_mutex_};
        sse_clients_.push_back(client);
    }

    // Send as many bytes as the kernel will accept without blocking.
    // Returns the number of bytes actually sent (0..size), or -1 on hard error.
    // EAGAIN/EWOULDBLOCK is NOT a hard error — caller gets 0 progress.
    static ssize_t try_send(int fd, const char* data, std::size_t size) {
        const auto n = ::send(fd, data, size, MSG_NOSIGNAL | MSG_DONTWAIT);
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return 0;
        return n;  // positive (partial or full) or -1 (hard error)
    }

    void writer_loop() {
        // Per-client pending: bytes of the previous SSE frame that could not be
        // sent in one call.  They MUST be flushed before the next frame is sent;
        // otherwise the SSE framing is corrupted (the next frame's "event: TYPE"
        // header gets spliced into the previous frame's data field).
        //
        // Without this, ::send() returning a partial count (0 < n < frame.size())
        // would silently discard the unsent tail — including the \r\n\r\n
        // event terminator — causing the corruption seen as
        // "Exponent part is missing a number" in JSON.parse().
        static constexpr std::size_t kMaxPending = 256U * 1024U;  // drop at 256 KB

        while (running_) {
            auto line = queue_.pop(running_);
            if (!line) continue;

            const auto frame = make_sse_frame(*line);
            std::lock_guard<std::mutex> lock{clients_mutex_};
            std::vector<int> dead;

            for (const int fd : sse_clients_) {
                auto& pending = sse_pending_[fd];

                // ── drain previously unsent tail ──────────────────────────
                if (!pending.empty()) {
                    const auto n = try_send(fd, pending.data(), pending.size());
                    if (n < 0) {
                        dead.push_back(fd); continue;  // hard error
                    }
                    if (static_cast<std::size_t>(n) < pending.size()) {
                        pending.erase(0, static_cast<std::size_t>(n));
                        // Can't send the new frame yet; skip it this cycle.
                        // The frame is lost for this client but the stream stays
                        // structurally valid (no corruption).
                        if (pending.size() > kMaxPending) dead.push_back(fd);
                        continue;
                    }
                    pending.clear();
                }

                // ── send new frame ────────────────────────────────────────
                const auto n = try_send(fd, frame.data(), frame.size());
                if (n < 0) {
                    dead.push_back(fd);  // hard error
                } else if (static_cast<std::size_t>(n) < frame.size()) {
                    // Partial send: store the unsent tail so it is sent before
                    // the next frame, preserving SSE event boundaries.
                    pending = frame.substr(static_cast<std::size_t>(n));
                    if (pending.size() > kMaxPending) dead.push_back(fd);
                }
                // n == frame.size(): fully sent, nothing to do.
            }

            for (const int fd : dead) {
                ::close(fd);
                sse_pending_.erase(fd);
                sse_clients_.erase(
                    std::remove(sse_clients_.begin(), sse_clients_.end(), fd),
                    sse_clients_.end());
            }
        }
    }

    void respond_json(int fd, const std::string& body) {
        std::string resp =
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json\r\n"
            "Content-Length: " + std::to_string(body.size()) + "\r\n"
            "Connection: close\r\n\r\n" + body;
        send_all(fd, resp);
        ::close(fd);
    }

    void respond_static_file(int fd, const std::string& url_path) {
        namespace fs = std::filesystem;
        // Resolve the requested path inside static_root_, rejecting traversal.
        try {
            const fs::path root_canon = fs::canonical(static_root_);
            // Strip leading '/' and join.
            const std::string rel = url_path.front() == '/'
                                        ? url_path.substr(1) : url_path;
            const fs::path candidate = root_canon / rel;
            // canonical() throws if the file doesn't exist — treat as 404.
            const fs::path file_canon = fs::canonical(candidate);
            if (!path_is_within_root(root_canon, file_canon)) {
                respond_not_found(fd);
                return;
            }
            const auto body = read_file_contents(file_canon.string());
            if (body.empty() && !fs::is_regular_file(file_canon)) {
                respond_not_found(fd);
                return;
            }
            const auto ct = content_type_for_path(file_canon.string());
            std::string resp =
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: " + ct + "\r\n"
                "Content-Length: " + std::to_string(body.size()) + "\r\n"
                "Cache-Control: no-cache\r\n"
                "Connection: close\r\n\r\n" + body;
            send_all(fd, resp);
            ::close(fd);
        } catch (const std::filesystem::filesystem_error&) {
            respond_not_found(fd);
        }
    }

    void respond_not_found(int fd) {
        const std::string resp =
            "HTTP/1.1 404 Not Found\r\n"
            "Content-Length: 9\r\n"
            "Connection: close\r\n\r\nnot found";
        send_all(fd, resp);
        ::close(fd);
    }

    std::string viewer_status_json() const {
        const auto st  = monitor_.status();
        const auto seq = monitor_.seq();
        std::string j;
        j += "{\"source\":\"";
        j += source_status_cstr(st);
        j += "\",\"seq\":";
        j += std::to_string(seq);
        if (st == SourceStatus::Replay) {
            j += ",\"replay_frame\":";
            j += std::to_string(monitor_.replay_frame());
            j += ",\"replay_total\":";
            j += std::to_string(monitor_.replay_total());
        }
        j += "}\n";
        return j;
    }

    void close_listen_socket() {
        if (listen_fd_ >= 0) { ::close(listen_fd_); listen_fd_ = -1; }
    }

    std::string bind_host_;
    std::uint16_t http_port_{0};
    std::uint16_t bound_port_{0};
    SharedQueue&         queue_;
    const SourceMonitor& monitor_;
    std::string static_root_;
    std::string static_default_file_;
    std::string l2_db_;
    double l2_cell_m_{1.0};
    double l2_vcell_m_{2.0};
    double l2_min_score_{0.5};

    int listen_fd_{-1};
    std::atomic<bool> running_{false};
    std::thread accept_thread_;
    std::thread writer_thread_;

    mutable std::mutex clients_mutex_;
    std::vector<int> sse_clients_;
    std::unordered_map<int, std::string> sse_pending_;  // per-client unsent SSE tail
};

// ── CLI ───────────────────────────────────────────────────────────────────────

struct CliOptions {
    std::string live_host{"127.0.0.1"};
    std::uint16_t live_port{47770};
    std::uint16_t http_port{8090};
    std::string replay_dir;
    int reconnect_s{3};
    double replay_speed{1.0};
    std::string static_root;
    std::string static_default_file{"viewer.html"};
    // L2 planning map DB — served to every new SSE client immediately.
    std::string l2_db;
    double l2_cell_m{1.0};
    double l2_vcell_m{2.0};
    double l2_min_score{0.5};
};

CliOptions parse_args(int argc, char** argv) {
    CliOptions opts;
    for (int i = 1; i < argc; ++i) {
        const std::string arg{argv[i]};
        auto next_arg = [&](const char* flag) -> std::string {
            if (i + 1 >= argc) throw std::invalid_argument(
                std::string{flag} + " requires a value");
            return argv[++i];
        };
        if      (arg == "--host")                { opts.live_host           = next_arg("--host"); }
        else if (arg == "--port")                { opts.live_port           = static_cast<std::uint16_t>(std::stoi(next_arg("--port"))); }
        else if (arg == "--http-port")           { opts.http_port           = static_cast<std::uint16_t>(std::stoi(next_arg("--http-port"))); }
        else if (arg == "--replay-dir")          { opts.replay_dir          = next_arg("--replay-dir"); }
        else if (arg == "--reconnect-s")         { opts.reconnect_s         = std::stoi(next_arg("--reconnect-s")); }
        else if (arg == "--replay-speed")        { opts.replay_speed        = std::stod(next_arg("--replay-speed")); }
        else if (arg == "--static-root")         { opts.static_root         = next_arg("--static-root"); }
        else if (arg == "--static-default-file") { opts.static_default_file = next_arg("--static-default-file"); }
        else if (arg == "--l2-db")               { opts.l2_db               = next_arg("--l2-db"); }
        else if (arg == "--l2-cell-m")           { opts.l2_cell_m           = std::stod(next_arg("--l2-cell-m")); }
        else if (arg == "--l2-vcell-m")          { opts.l2_vcell_m          = std::stod(next_arg("--l2-vcell-m")); }
        else if (arg == "--l2-min-score")        { opts.l2_min_score        = std::stod(next_arg("--l2-min-score")); }
        else if (arg == "-h" || arg == "--help") {
            std::cout <<
                "usage: dedalus_viewer [options]\n"
                "\n"
                "  --host HOST              Runtime TCP host       (default: 127.0.0.1)\n"
                "  --port PORT              Runtime TCP port       (default: 47770)\n"
                "  --http-port PORT         Viewer HTTP/SSE port   (default: 8090)\n"
                "  --replay-dir DIR         Snapshot artifact dir  (default: disabled)\n"
                "  --reconnect-s N          Live reconnect interval in seconds (default: 3)\n"
                "  --replay-speed F         Replay speed multiplier (default: 1.0)\n"
                "  --static-root DIR        Serve static files from DIR (default: disabled)\n"
                "  --static-default-file F  Default file for GET / (default: viewer.html)\n"
                "  --l2-db PATH             L2 SQLite DB to push on every new SSE connect\n"
                "                           (default: maps/$DEDALUS_SITE_ID/l2_map.db if env set)\n"
                "  --l2-cell-m F            L2 XY cell size in metres (default: 1.0)\n"
                "  --l2-vcell-m F           L2 Z  cell size in metres (default: 2.0)\n"
                "  --l2-min-score F         Minimum occupied_score to include (default: 0.5)\n"
                "  -h / --help              Show this help\n"
                "\n"
                "Browser endpoints:\n"
                "  GET /events         — SSE stream of all mission events\n"
                "  GET /viewer_status  — JSON source status\n"
                "  GET /healthz        — JSON health check\n"
                "  GET /               — Serves --static-default-file (when --static-root is set)\n"
                "  GET /<path>         — Serves static file from --static-root (when set)\n";
            std::exit(0);
        } else {
            throw std::invalid_argument("unknown argument: " + arg);
        }
    }
    // Derive L2 DB path from DEDALUS_SITE_ID if --l2-db was not passed explicitly.
    if (opts.l2_db.empty()) {
        if (const char* site_id = std::getenv("DEDALUS_SITE_ID")) {
            opts.l2_db = std::string{"maps/"} + site_id + "/l2_map.db";
        }
    }
    return opts;
}

// ── signal handling ───────────────────────────────────────────────────────────

std::atomic<bool> g_shutdown{false};

void on_signal(int) { g_shutdown = true; }

}  // namespace

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    try {
        const auto opts = parse_args(argc, argv);

        ::signal(SIGINT,  on_signal);
        ::signal(SIGTERM, on_signal);
        ::signal(SIGPIPE, SIG_IGN);  // handle broken SSE clients gracefully

        SharedQueue queue{512};

        SourceMonitor monitor{
            opts.live_host, opts.live_port,
            opts.replay_dir, opts.replay_speed,
            opts.reconnect_s, queue};

        // Bind to all interfaces so browsers on other machines can connect.
        // L2 DB is loaded fresh on every new SSE connection so the viewer
        // always reflects the current DB state (including post-mission flushes).
        SseRelay relay{"0.0.0.0", opts.http_port, queue, monitor,
                       opts.static_root, opts.static_default_file,
                       opts.l2_db, opts.l2_cell_m, opts.l2_vcell_m,
                       opts.l2_min_score};

        relay.start();
        monitor.start();

        const auto http_port = relay.bound_port();
        std::cout
            << "dedalus_viewer started\n"
            << "  live source :  " << opts.live_host << ":" << opts.live_port << "\n"
            << "  browser SSE :  http://0.0.0.0:" << http_port << "/events\n"
            << "  viewer status: http://0.0.0.0:" << http_port << "/viewer_status\n"
            << "  health check:  http://0.0.0.0:" << http_port << "/healthz\n";
        if (!opts.replay_dir.empty()) {
            std::cout << "  replay dir  :  " << opts.replay_dir << "\n";
        }
        if (!opts.l2_db.empty()) {
            std::cout << "  L2 DB       :  " << opts.l2_db << "\n";
        }
        if (!opts.static_root.empty()) {
            std::cout << "  static root :  " << opts.static_root << "\n"
                      << "  viewer URL  :  http://0.0.0.0:" << http_port << "/\n";
        }
        std::cout << std::flush;

        while (!g_shutdown) {
            std::this_thread::sleep_for(std::chrono::milliseconds{100});
        }

        std::cout << "\ndedalus_viewer: shutting down\n" << std::flush;
        monitor.stop();
        relay.stop();
        return 0;

    } catch (const std::exception& ex) {
        std::cerr << "dedalus_viewer: " << ex.what() << "\n";
        return 1;
    }
}
