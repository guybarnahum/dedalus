#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "dedalus/avoidance/local_esdf_map.hpp"

namespace dedalus {

// Raw ESDF frame carried to the stream server.
// The writer thread serializes it via serialize_esdf_snapshot(),
// keeping the expensive to_compact_stream_json() off the hot path.
struct LocalESDFMapFrame {
    std::uint64_t timestamp_ns{0U};
    LocalESDFMapSnapshot snapshot;
};

class LocalESDFMapSubscriber {
public:
    virtual ~LocalESDFMapSubscriber() = default;
    virtual void on_esdf_snapshot(const LocalESDFMapFrame& frame) = 0;
};

class LocalESDFMapPublisher {
public:
    void subscribe(std::shared_ptr<LocalESDFMapSubscriber> subscriber);
    void publish(const LocalESDFMapFrame& frame);

private:
    std::mutex mutex_;
    std::vector<std::weak_ptr<LocalESDFMapSubscriber>> subscribers_;
};

// Serialize an ESDF snapshot to compact inner JSON suitable for embedding in
// a stream line.  max_cells == 0 means no cap.
// JSON shape:
//   {"cell_size_m":1.0,"vcell_size_m":2.0,"d0_m":5.0,"is_delta":false,
//    "net_rep":{"x":0.0,"y":0.0,"z":0.0},
//    "cells":[{"x":0.5,"y":0.5,"z":1.0,"d":2.3,"gx":0.9,"gy":0.1,"gz":0.0},...]}
std::string to_compact_stream_json(
    const LocalESDFMapSnapshot& snapshot,
    std::size_t max_cells = 0U);

}  // namespace dedalus
