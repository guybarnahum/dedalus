#include <iostream>
#include <string>

#include "dedalus/world_model/world_snapshot.hpp"

namespace {

bool contains(const std::string& haystack, const std::string& needle) {
    return haystack.find(needle) != std::string::npos;
}

}  // namespace

int main() {
    dedalus::WorldSnapshot snapshot;
    snapshot.timestamp = dedalus::TimePoint{123456789};
    snapshot.active_map_frame_id = dedalus::MapFrameId{"map_local_0001"};

    dedalus::MapFrame map_frame;
    map_frame.map_frame_id = snapshot.active_map_frame_id;
    map_frame.scale_confidence = 0.5F;
    map_frame.orientation_confidence = 0.5F;
    snapshot.map_frames.push_back(map_frame);

    const std::string json = dedalus::to_json(snapshot);

    if (!contains(json, "\"map_frames\"")) {
        std::cerr << "missing map_frames array\n";
        return 1;
    }

    if (!contains(json, "\"map_frame_id\": \"map_local_0001\"")) {
        std::cerr << "missing serialized map_frame_id\n";
        return 1;
    }

    return 0;
}
