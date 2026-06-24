#include "dedalus/avoidance/mission_local_planning_map_publisher.hpp"

#include <sstream>
#include <string>

namespace dedalus {

// ---- Publisher ---------------------------------------------------------------

void MissionLocalPlanningMapPublisher::subscribe(
    std::shared_ptr<MissionLocalPlanningMapSubscriber> subscriber) {
    std::lock_guard<std::mutex> lock{mutex_};
    subscribers_.push_back(std::move(subscriber));
}

void MissionLocalPlanningMapPublisher::publish(const MissionLocalPlanningMapFrame& frame) {
    std::lock_guard<std::mutex> lock{mutex_};
    for (auto it = subscribers_.begin(); it != subscribers_.end(); ) {
        if (auto sub = it->lock()) {
            sub->on_planning_map_snapshot(frame);
            ++it;
        } else {
            it = subscribers_.erase(it);
        }
    }
}

// ---- Compact streaming serializer -------------------------------------------
//
// Produces compact inner JSON embedded as the "planning_map_snapshot" value
// in the outer stream line.  Schema: dedalus.mission_local_planning_map.v1
//
// Fields: center_map (world-frame voxel centre), occupied_score,
// confidence, source_cell_count.  No per-cell state string: L2 cells are
// always occupied (cells below min_occupied_score are evicted before
// snapshotting).

std::string to_compact_stream_json(
    const MissionLocalPlanningMapSnapshot& snapshot,
    const std::size_t max_cells) {

    const std::size_t total_cells = snapshot.cells.size();
    const std::size_t exported_cells =
        (max_cells > 0U && total_cells > max_cells) ? max_cells : total_cells;

    std::ostringstream out;
    out.precision(6);

    out << "{\"schema\":\"dedalus.mission_local_planning_map.v1\"";
    out << ",\"cell_size_m\":"          << snapshot.config.cell_size_m;
    out << ",\"vertical_cell_size_m\":" << snapshot.config.vertical_cell_size_m;
    out << ",\"cell_count\":"           << snapshot.cell_count;
    out << ",\"total_cells\":"          << total_cells;
    out << ",\"exported_cells\":"       << exported_cells;

    out << ",\"cells\":[";
    for (std::size_t i = 0U; i < exported_cells; ++i) {
        const auto& cell = snapshot.cells[i];
        if (i > 0U) { out << ","; }
        out << "{\"center_map\":["
            << cell.center_map.x << ","
            << cell.center_map.y << ","
            << cell.center_map.z << "]";
        out << ",\"occupied_score\":"    << cell.occupied_score;
        out << ",\"confidence\":"        << cell.confidence;
        out << ",\"source_cell_count\":" << cell.source_cell_count;
        if (cell.last_updated_ns > 0) {
            // Emit seconds since epoch so the viewer can compute cell age.
            out << ",\"t\":" << (cell.last_updated_ns / 1'000'000'000LL);
        }
        out << "}";
    }
    out << "]}";

    return out.str();
}

}  // namespace dedalus
