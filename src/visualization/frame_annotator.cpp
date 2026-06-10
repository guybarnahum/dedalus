#include "dedalus/visualization/frame_annotator.hpp"

#include <filesystem>
#include <stdexcept>
#include <string>
#include <utility>

namespace dedalus {
void NullFrameAnnotationSink::annotate(const AnnotationContext&) {}

void NullFrameAnnotationSink::finish() {}

PpmFrameAnnotationSink::PpmFrameAnnotationSink(std::string output_dir, double output_fps)
    : output_dir_(std::move(output_dir)), output_fps_(output_fps) {
    if (output_dir_.empty()) {
        throw std::invalid_argument("ppm_sequence frame annotation sink requires annotation_output_path");
    }
    if (output_fps_ <= 0.0) {
        throw std::invalid_argument("ppm_sequence frame annotation sink requires annotation_output_fps > 0");
    }
}

void PpmFrameAnnotationSink::finish() {}

Mp4FrameAnnotationSink::Mp4FrameAnnotationSink(std::string output_path, double output_fps)
    : output_path_(std::move(output_path)), output_fps_(output_fps) {}

void Mp4FrameAnnotationSink::annotate(const AnnotationContext&) {
    throw std::runtime_error(
        "mp4 frame annotation sink is not implemented yet; use frame_annotator: null or ppm_sequence");
}

void Mp4FrameAnnotationSink::finish() {}

}  // namespace dedalus
