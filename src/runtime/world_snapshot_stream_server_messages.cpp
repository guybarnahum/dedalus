#include "dedalus/runtime/world_snapshot_stream_server.hpp"

#include "dedalus/avoidance/mission_local_traversability_map_publisher.hpp"
#include "dedalus/core/json_utils.hpp"
#include "dedalus/world_model/world_snapshot.hpp"

#include <chrono>
#include <string>
#include <utility>

namespace dedalus {
namespace {

using SteadyClock = std::chrono::steady_clock;

std::uint64_t elapsed_us(const SteadyClock::time_point start) {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(SteadyClock::now() - start).count());
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

std::string stream_line_for(std::uint64_t seq, const MissionObstacleMapDeltaFrame& frame) {
    std::string payload = frame.json;
    while (!payload.empty() && (payload.back() == '\n' || payload.back() == '\r')) {
        payload.pop_back();
    }

    std::string line;
    line.reserve(payload.size() + 128U);
    line += "{\"type\":\"mission_obstacle_map_delta\",\"seq\":";
    line += std::to_string(seq);
    line += ",\"timestamp_ns\":";
    line += std::to_string(frame.timestamp_ns);
    line += ",\"mission_obstacle_map_delta\":";
    line += payload;
    line += "}\n";
    return line;
}

}  // namespace

void RuntimeEventStreamServer::on_snapshot(const std::shared_ptr<const WorldSnapshot>& snapshot) {
    // Assign a sequence number on the perception thread (preserves ordering),
    // but defer the expensive to_json() / to_compact_json() to the writer thread.
    const std::uint64_t seq = [this] {
        std::lock_guard<std::mutex> lock{mutex_};
        ++snapshot_messages_;
        return ++published_seq_;
    }();
    enqueue_snapshot(seq, snapshot);
}

std::string RuntimeEventStreamServer::serialize_snapshot(
    std::uint64_t seq, const WorldSnapshot& snapshot) const {
    return stream_line_for(seq, snapshot);
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

void RuntimeEventStreamServer::on_mission_obstacle_map_delta(
    const MissionObstacleMapDeltaFrame& frame) {
    const auto start = SteadyClock::now();
    const std::uint64_t seq = [this] {
        std::lock_guard<std::mutex> lock{mutex_};
        ++mission_obstacle_map_delta_messages_;
        return ++published_seq_;
    }();
    auto line = stream_line_for(seq, frame);
    const auto serialize_duration_us = elapsed_us(start);
    enqueue_line(std::move(line));
    std::lock_guard<std::mutex> lock{mutex_};
    serialize_total_us_ += serialize_duration_us;
}

void RuntimeEventStreamServer::on_traversability_map_snapshot(
    const MissionLocalTraversabilityMapFrame& frame) {
    // Read current watermark (calls to this method are serialized by the publisher's
    // mutex, so no concurrent modification is possible, but we lock for correctness
    // because trav_watermark_ns_ is also touched inside the seq-assignment lock below).
    const std::uint64_t watermark = [this] {
        std::lock_guard<std::mutex> lock{mutex_};
        return trav_watermark_ns_;
    }();

    // Filter cells:
    //   First publish  (watermark == 0): include all cells → full snapshot, client clears + rebuilds.
    //   Subsequent     (watermark  > 0): include only cells newer than watermark → delta, client merges.
    const bool is_delta = watermark > 0U;
    MissionLocalTraversabilityMapSnapshot filtered;
    filtered.config  = frame.snapshot.config;
    filtered.summary = frame.snapshot.summary;

    std::uint64_t new_watermark = watermark;
    for (const auto& cell : frame.snapshot.cells) {
        if (!is_delta || cell.last_observed_timestamp_ns > watermark) {
            filtered.cells.push_back(cell);
            if (cell.last_observed_timestamp_ns > new_watermark) {
                new_watermark = cell.last_observed_timestamp_ns;
            }
        }
    }

    // Nothing changed — skip.
    if (is_delta && filtered.cells.empty()) {
        return;
    }

    // Defensive: if every cell had timestamp 0 (shouldn't happen in practice since
    // update_from_mission_obstacle_map always stamps cells), advance watermark to at
    // least 1 so the next call is treated as a delta rather than a second full sync.
    const std::uint64_t effective_watermark =
        (new_watermark == 0U) ? std::uint64_t{1U} : new_watermark;

    // Assign seq, update watermark, enqueue — all under one lock to preserve ordering.
    const std::uint64_t seq = [this, effective_watermark] {
        std::lock_guard<std::mutex> lock{mutex_};
        ++traversability_map_snapshot_messages_;
        trav_watermark_ns_ = effective_watermark;
        return ++published_seq_;
    }();
    enqueue_traversability_snapshot(seq, frame.timestamp_ns, std::move(filtered), is_delta);
}

std::string RuntimeEventStreamServer::serialize_traversability_snapshot(
    std::uint64_t seq,
    std::uint64_t timestamp_ns,
    const MissionLocalTraversabilityMapSnapshot& snapshot,
    bool is_delta) const {
    // No cap in either case:
    //   Delta   — already filtered to changed cells only; count is inherently small.
    //   Snapshot — must include all cells so the watermark covers the full map.
    //              Sending the full map once is acceptable given the 2 s throttle.
    const auto payload = to_compact_stream_json(snapshot, 0U);
    const char* const type = is_delta ? "traversability_map_delta" : "traversability_map_snapshot";
    std::string line;
    line.reserve(payload.size() + 160U);
    line += "{\"type\":\"";
    line += type;
    line += "\",\"seq\":";
    line += std::to_string(seq);
    line += ",\"timestamp_ns\":";
    line += std::to_string(timestamp_ns);
    line += ",\"";
    line += type;
    line += "\":";
    line += payload;
    line += "}\n";
    return line;
}


}  // namespace dedalus
