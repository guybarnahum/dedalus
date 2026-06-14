#include "dedalus/visualization/frame_annotator.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

#include "dedalus/visualization/world_overlay_sidecar.hpp"

namespace dedalus {

void annotate_frame_overlays(ImageView& image, const AnnotationContext& context);

namespace {

std::string frame_file_name(const std::size_t frame_index) {
    std::ostringstream stream;
    stream << "frame_" << std::setw(6) << std::setfill('0') << frame_index << ".ppm";
    return stream.str();
}

void validate_rgb_image(const ImageView& image) {
    if (image.width <= 0 || image.height <= 0 || image.channels != 3) {
        throw std::runtime_error("PPM frame annotation sink requires non-empty RGB images");
    }
    const auto expected_size = static_cast<std::size_t>(image.width * image.height * image.channels);
    if (image.bytes.size() != expected_size) {
        throw std::runtime_error("PPM frame annotation sink image byte count does not match dimensions");
    }
}

void write_ppm(const std::filesystem::path& path, const ImageView& image) {
    std::ofstream output{path, std::ios::binary};
    if (!output) {
        throw std::runtime_error("failed to open PPM annotation output: " + path.string());
    }
    output << "P6\n" << image.width << ' ' << image.height << "\n255\n";
    output.write(reinterpret_cast<const char*>(image.bytes.data()), static_cast<std::streamsize>(image.bytes.size()));
    if (!output) {
        throw std::runtime_error("failed to write PPM annotation output: " + path.string());
    }
}

void append_manifest_row(
    const std::filesystem::path& manifest_path,
    const std::size_t frame_index,
    const AnnotationContext& context,
    const std::filesystem::path& frame_path,
    const double output_fps) {
    const bool write_header = !std::filesystem::exists(manifest_path);
    std::ofstream manifest{manifest_path, std::ios::app};
    if (!manifest) {
        throw std::runtime_error("failed to open annotation manifest: " + manifest_path.string());
    }
    if (write_header) {
        manifest << "frame_index,frame_id,timestamp_ns,path,output_fps\n";
    }
    manifest << frame_index << ',' << context.frame.frame_id.value << ','
             << context.frame.timestamp.timestamp_ns << ',' << frame_path.string() << ','
             << output_fps << '\n';
}

}  // namespace

void PpmFrameAnnotationSink::annotate(const AnnotationContext& context) {
    validate_rgb_image(context.frame.image);

    std::filesystem::create_directories(output_dir_);
    ImageView annotated = context.frame.image;
    annotate_frame_overlays(annotated, context);

    ++frame_index_;
    const auto frame_path = std::filesystem::path{output_dir_} / frame_file_name(frame_index_);
    write_ppm(frame_path, annotated);
    write_world_overlay_sidecar(std::filesystem::path{output_dir_}, frame_index_, context);
    append_manifest_row(std::filesystem::path{output_dir_} / "manifest.txt", frame_index_, context, frame_path, output_fps_);
}

}  // namespace dedalus
