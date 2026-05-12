#include "dedalus/visualization/frame_annotator.hpp"

#include <stdexcept>
#include <string>
#include <utility>

namespace dedalus {

void NullFrameAnnotationSink::annotate(const AnnotationContext&) {}

void NullFrameAnnotationSink::finish() {}

Mp4FrameAnnotationSink::Mp4FrameAnnotationSink(std::string output_path, double output_fps)
    : output_path_(std::move(output_path)), output_fps_(output_fps) {}

void Mp4FrameAnnotationSink::annotate(const AnnotationContext&) {
    throw std::runtime_error(
        "mp4 frame annotation sink is not implemented yet; use frame_annotator: null");
}

void Mp4FrameAnnotationSink::finish() {}

}  // namespace dedalus
