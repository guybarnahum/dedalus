#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "dedalus/sensors/frame_source.hpp"

namespace dedalus {

class ReplayFrameSource final : public FrameSource {
public:
    explicit ReplayFrameSource(std::vector<FramePacket> frames);

    std::optional<FramePacket> next_frame() override;
    [[nodiscard]] std::size_t remaining() const;

private:
    std::vector<FramePacket> frames_;
    std::size_t next_index_{0};
};

class VideoOnlyFrameSource final : public FrameSource {
public:
    explicit VideoOnlyFrameSource(std::size_t frame_count = 1U);

    std::optional<FramePacket> next_frame() override;

private:
    std::size_t frame_count_{0};
    std::size_t emitted_{0};
};

}  // namespace dedalus
