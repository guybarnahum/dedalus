// local_esdf_map_publisher.cpp
//
// LocalESDFMapPublisher + to_compact_stream_json() for Stage 6 SSE streaming.

#include "dedalus/avoidance/local_esdf_map_publisher.hpp"

#include <algorithm>
#include <cstdio>
#include <string>

namespace dedalus {

void LocalESDFMapPublisher::subscribe(std::shared_ptr<LocalESDFMapSubscriber> subscriber) {
    std::lock_guard<std::mutex> lock{mutex_};
    subscribers_.push_back(subscriber);
}

void LocalESDFMapPublisher::publish(const LocalESDFMapFrame& frame) {
    std::lock_guard<std::mutex> lock{mutex_};
    // Expire dead subscribers while iterating.
    auto it = subscribers_.begin();
    while (it != subscribers_.end()) {
        auto sub = it->lock();
        if (!sub) {
            it = subscribers_.erase(it);
            continue;
        }
        sub->on_esdf_snapshot(frame);
        ++it;
    }
}

// ─── to_compact_stream_json ──────────────────────────────────────────────────

std::string to_compact_stream_json(const LocalESDFMapSnapshot& snap, std::size_t max_cells) {
    const auto& cfg = snap.config;
    const auto& nr  = snap.net_repulsion;

    const std::size_t n = (max_cells == 0U || snap.cells.size() <= max_cells)
                          ? snap.cells.size()
                          : max_cells;

    // Pre-size: roughly 110 chars per cell (grad + sgrad) + fixed header (~200 chars).
    std::string out;
    out.reserve(200U + n * 110U);

    char buf[256];

    // Header
    std::snprintf(buf, sizeof(buf),
        "{\"cell_size_m\":%.4g,\"vcell_size_m\":%.4g,\"d0_m\":%.4g,\"is_delta\":%s",
        cfg.cell_size_m, cfg.vertical_cell_size_m, cfg.d0_m,
        snap.is_delta ? "true" : "false");
    out += buf;

    // Net repulsion
    std::snprintf(buf, sizeof(buf),
        ",\"net_rep\":{\"x\":%.5g,\"y\":%.5g,\"z\":%.5g}",
        nr.x, nr.y, nr.z);
    out += buf;

    // Cells array
    out += ",\"cells\":[";
    for (std::size_t i = 0U; i < n; ++i) {
        const auto& c = snap.cells[i];
        if (i > 0U) out += ',';
        std::snprintf(buf, sizeof(buf),
            "{\"x\":%.4g,\"y\":%.4g,\"z\":%.4g,\"d\":%.4g"
            ",\"gx\":%.4g,\"gy\":%.4g,\"gz\":%.4g"
            ",\"sgx\":%.4g,\"sgy\":%.4g,\"sgz\":%.4g}",
            c.centre.x, c.centre.y, c.centre.z,
            static_cast<double>(c.d),
            c.grad.x, c.grad.y, c.grad.z,
            c.sgrad.x, c.sgrad.y, c.sgrad.z);
        out += buf;
    }
    out += "]}";
    return out;
}

}  // namespace dedalus
