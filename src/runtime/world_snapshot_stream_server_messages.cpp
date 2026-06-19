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

std::string stream_line_for(std::uint64_t seq, const MissionLocalTraversabilityMapFrame& frame) {
    std::string payload = frame.json;
    while (!payload.empty() && (payload.back() == '\n' || payload.back() == '\r')) {
        payload.pop_back();
    }

    std::string line;
    line.reserve(payload.size() + 128U);
    line += "{\"type\":\"traversability_map_snapshot\",\"seq\":";
    line += std::to_string(seq);
    line += ",\"timestamp_ns\":";
    line += std::to_string(frame.timestamp_ns);
    line += ",\"traversability_map_snapshot\":";
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
    const auto start = SteadyClock::now();
    const std::uint64_t seq = [this] {
        std::lock_guard<std::mutex> lock{mutex_};
        ++traversability_map_snapshot_messages_;
        return ++published_seq_;
    }();
    auto line = stream_line_for(seq, frame);
    const auto serialize_duration_us = elapsed_us(start);
    enqueue_line(std::move(line));
    std::lock_guard<std::mutex> lock{mutex_};
    serialize_total_us_ += serialize_duration_us;
}


}  // namespace dedalus
