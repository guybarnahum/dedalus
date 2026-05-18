#pragma once

#include <cstddef>
#include <filesystem>

#include "dedalus/visualization/frame_annotator.hpp"

namespace dedalus {

void write_world_overlay_sidecar(
    const std::filesystem::path& output_dir,
    std::size_t frame_index,
    const AnnotationContext& context,
    double horizontal_fov_deg = 90.0);

}  // namespace dedalus
