#include "dedalus/avoidance/mission_local_traversability_map_publisher.hpp"

#include <algorithm>
#include <sstream>
#include <string>

namespace dedalus {
namespace {

const char* cell_state_name(const TraversabilityCellState state) {
    switch (state) {
        case TraversabilityCellState::ObservedFree:  return "free";
        case TraversabilityCellState::Occupied:      return "occupied";
        case TraversabilityCellState::Mixed:         return "mixed";
        case TraversabilityCellState::Stale:         return "stale";
        case TraversabilityCellState::Unknown:
        default:                                     return "unknown";
    }
}

}  // namespace

// ---- Publisher ---------------------------------------------------------------

void MissionLocalTraversabilityMapPublisher::subscribe(
    std::shared_ptr<MissionLocalTraversabilityMapSubscriber> subscriber) {
    std::lock_guard<std::mutex> lock{mutex_};
    subscribers_.push_back(std::move(subscriber));
}

void MissionLocalTraversabilityMapPublisher::publish(
    const MissionLocalTraversabilityMapFrame& frame) {
    std::lock_guard<std::mutex> lock{mutex_};
    for (auto it = subscribers_.begin(); it != subscribers_.end(); ) {
        if (auto sub = it->lock()) {
            sub->on_traversability_map_snapshot(frame);
            ++it;
        } else {
            it = subscribers_.erase(it);
        }
    }
}

// ---- Compact streaming serializer -------------------------------------------
//
// Produces compact inner JSON embedded as the "traversability_map_snapshot"
// value in the outer stream line.  Schema matches the artifact writer so the
// mission_traversability_map_viewer can consume both.
//
// Field selection is intentionally lean: the viewer needs position, scores,
// state, and clearance flags.  Counts and timestamps are omitted per cell to
// keep per-frame payload small.

std::string to_compact_stream_json(
    const MissionLocalTraversabilityMapSnapshot& snapshot,
    const std::size_t max_cells) {

    const std::size_t total_cells = snapshot.cells.size();
    const std::size_t exported_cells =
        (max_cells > 0U && total_cells > max_cells) ? max_cells : total_cells;

    std::ostringstream out;
    out.precision(6);

    out << "{\"schema\":\"dedalus.mission_local_traversability_map.v1\"";

    // Config fields the viewer uses for grid rendering.
    out << ",\"cell_size_m\":" << snapshot.config.cell_size_m;
    out << ",\"vertical_cell_size_m\":" << snapshot.config.vertical_cell_size_m;

    // Map frame.
    out << ",\"map_frame_id\":\"" << snapshot.summary.map_frame_id.value << "\"";

    // Summary counters.
    out << ",\"cell_count\":"             << snapshot.summary.cell_count;
    out << ",\"occupied_cell_count\":"    << snapshot.summary.occupied_cell_count;
    out << ",\"free_cell_count\":"        << snapshot.summary.free_cell_count;
    out << ",\"mixed_cell_count\":"       << snapshot.summary.mixed_cell_count;
    out << ",\"stale_cell_count\":"       << snapshot.summary.stale_cell_count;
    out << ",\"low_clearance_cell_count\":"  << snapshot.summary.low_clearance_cell_count;
    out << ",\"overhead_risk_cell_count\":"  << snapshot.summary.overhead_risk_cell_count;

    // Bookkeeping so viewer knows the cap was applied.
    out << ",\"total_cells\":"    << total_cells;
    out << ",\"exported_cells\":" << exported_cells;

    // Cell array.
    out << ",\"cells\":[";
    for (std::size_t i = 0U; i < exported_cells; ++i) {
        const auto& cell = snapshot.cells[i];
        if (i > 0U) {
            out << ",";
        }
        out << "{\"center_map\":["
            << cell.center_map.x << ","
            << cell.center_map.y << ","
            << cell.center_map.z << "]";
        out << ",\"log_odds\":"        << cell.log_odds;
        out << ",\"occupied_score\":" << cell.occupied_score;
        out << ",\"free_score\":"     << cell.free_score;
        out << ",\"confidence\":"     << cell.confidence;
        out << ",\"state\":\""        << cell_state_name(cell.state) << "\"";
        out << ",\"stale\":"          << (cell.stale ? "true" : "false");
        out << "}";
    }
    out << "]}";

    return out.str();
}

}  // namespace dedalus
