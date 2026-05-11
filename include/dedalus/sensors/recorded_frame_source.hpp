#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "dedalus/sensors/frame_source.hpp"

namespace dedalus {

struct RecordedFrameManifestEntry {
    std::filesystem::path image_path;
    FrameId frame_id;
    TimePoint timestamp;
    CameraId camera_id;
};

class RecordedFrameSource final : public FrameSource {
public:
    explicit RecordedFrameSource(std::filesystem::path manifest_path);

    std::optional<FramePacket> next_frame() override;

private:
    std::vector<RecordedFrameManifestEntry> entries_;
    std::filesystem::path manifest_dir_;
    std::size_t next_index_{0};
};

}  // namespace dedalus
