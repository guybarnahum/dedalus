#include "dedalus/sensing/depth_engine_utils.hpp"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

namespace dedalus {
namespace depth_engine_utils {

std::vector<float> resize_and_normalise(
    const std::vector<std::uint8_t>& bytes,
    int src_w, int src_h,
    int dst_w, int dst_h,
    float mean_r, float mean_g, float mean_b,
    float std_r,  float std_g,  float std_b) {

    std::vector<float> tensor(static_cast<std::size_t>(3 * dst_h * dst_w), 0.0F);

    const float sx = static_cast<float>(src_w) / static_cast<float>(dst_w);
    const float sy = static_cast<float>(src_h) / static_cast<float>(dst_h);

    const float mean[3] = {mean_r, mean_g, mean_b};
    const float stdv[3] = {std_r,  std_g,  std_b};

    for (int dy = 0; dy < dst_h; ++dy) {
        for (int dx = 0; dx < dst_w; ++dx) {
            const int sx_i = std::min(static_cast<int>(dx * sx), src_w - 1);
            const int sy_i = std::min(static_cast<int>(dy * sy), src_h - 1);
            const std::size_t src_idx =
                (static_cast<std::size_t>(sy_i) * static_cast<std::size_t>(src_w) +
                 static_cast<std::size_t>(sx_i)) * 3U;

            for (int c = 0; c < 3; ++c) {
                const float pixel = static_cast<float>(bytes[src_idx + c]) / 255.0F;
                const float norm  = (pixel - mean[c]) / stdv[c];
                const std::size_t dst_idx =
                    static_cast<std::size_t>(c) * static_cast<std::size_t>(dst_h * dst_w) +
                    static_cast<std::size_t>(dy * dst_w + dx);
                tensor[dst_idx] = norm;
            }
        }
    }
    return tensor;
}

std::vector<float> downsample(
    const float* src, int src_w, int src_h,
    int dst_w, int dst_h) {

    std::vector<float> out(static_cast<std::size_t>(dst_h * dst_w), 0.0F);
    const float sx = static_cast<float>(src_w) / static_cast<float>(dst_w);
    const float sy = static_cast<float>(src_h) / static_cast<float>(dst_h);

    for (int dy = 0; dy < dst_h; ++dy) {
        for (int dx = 0; dx < dst_w; ++dx) {
            const int sx_i = std::min(static_cast<int>(dx * sx), src_w - 1);
            const int sy_i = std::min(static_cast<int>(dy * sy), src_h - 1);
            out[static_cast<std::size_t>(dy * dst_w + dx)] =
                src[static_cast<std::size_t>(sy_i * src_w + sx_i)];
        }
    }
    return out;
}

std::vector<float> downsample_min_z(
    const float* src, int src_w, int src_h,
    int dst_w, int dst_h) {

    // Each output pixel covers a bin of input pixels.  Take the minimum valid
    // Z (> 0) within that bin — the closest obstacle wins.
    // Z == 0 means invalid/sky; excluded so it doesn't beat real depth readings.
    static constexpr float kInvalid = 0.0F;
    static constexpr float kSentinel = 1e9F;  // initialiser — replaced by real values

    const float sx = static_cast<float>(src_w) / static_cast<float>(dst_w);
    const float sy = static_cast<float>(src_h) / static_cast<float>(dst_h);

    std::vector<float> out(static_cast<std::size_t>(dst_h * dst_w), kSentinel);

    for (int sy_i = 0; sy_i < src_h; ++sy_i) {
        const int dy = std::min(static_cast<int>(static_cast<float>(sy_i) / sy), dst_h - 1);
        for (int sx_i = 0; sx_i < src_w; ++sx_i) {
            const float z = src[static_cast<std::size_t>(sy_i * src_w + sx_i)];
            if (z <= kInvalid) continue;  // skip invalid pixels
            const int dx = std::min(static_cast<int>(static_cast<float>(sx_i) / sx), dst_w - 1);
            float& cell = out[static_cast<std::size_t>(dy * dst_w + dx)];
            if (z < cell) cell = z;
        }
    }

    // Replace sentinel with 0 (invalid) for any output pixel that had no valid input.
    for (float& v : out) {
        if (v >= kSentinel) v = kInvalid;
    }
    return out;
}

namespace {

// Write NumPy 1.0 file header + raw data.  Used by both dump_npy_* overloads.
void write_npy_header(std::ofstream& f, const std::string& descr,
                      int h, int w, int c) {
    std::string hdr = "{'descr': '" + descr + "', 'fortran_order': False, 'shape': (";
    hdr += std::to_string(h) + ", " + std::to_string(w);
    if (c > 0) hdr += ", " + std::to_string(c);
    hdr += "), }";
    while ((hdr.size() + 1U + 10U) % 64U != 0U) hdr += ' ';
    hdr += '\n';
    const auto hl = static_cast<std::uint16_t>(hdr.size());
    f.write("\x93NUMPY", 6);
    f.put('\x01'); f.put('\x00');
    f.put(static_cast<char>(hl & 0xFFU));
    f.put(static_cast<char>((hl >> 8U) & 0xFFU));
    f.write(hdr.c_str(), static_cast<std::streamsize>(hdr.size()));
}

}  // namespace

void dump_npy_f32(const char* path, const float* data, int h, int w) {
    if (std::ofstream f{path, std::ios::binary}) {
        write_npy_header(f, "<f4", h, w, -1);
        f.write(reinterpret_cast<const char*>(data),
                static_cast<std::streamsize>(h * w) * sizeof(float));
        std::fprintf(stderr, "[depth_engine] frame0 depth → %s\n", path);
    }
}

void dump_npy_u8(const char* path, const std::uint8_t* data, int h, int w, int c) {
    if (std::ofstream f{path, std::ios::binary}) {
        write_npy_header(f, "|u1", h, w, c);
        f.write(reinterpret_cast<const char*>(data),
                static_cast<std::streamsize>(h * w * c));
        std::fprintf(stderr, "[depth_engine] frame0 rgb   → %s\n", path);
    }
}

void maybe_write_debug_npy(
    const char* debug_dir,
    const std::string& frame_id,
    const float* data, int h, int w) {

    if (!debug_dir || frame_id.empty()) return;
    const std::string path = std::string{debug_dir} + "/" + frame_id + ".npy";
    std::string hdr = "{'descr': '<f4', 'fortran_order': False, 'shape': (";
    hdr += std::to_string(h) + ", " + std::to_string(w) + "), }";
    while ((hdr.size() + 1U + 10U) % 64U != 0U) hdr += ' ';
    hdr += '\n';
    const auto hdr_len = static_cast<std::uint16_t>(hdr.size());
    if (std::ofstream npy{path, std::ios::binary}) {
        npy.write("\x93NUMPY", 6);
        npy.put('\x01'); npy.put('\x00');
        npy.put(static_cast<char>(hdr_len & 0xFFU));
        npy.put(static_cast<char>((hdr_len >> 8U) & 0xFFU));
        npy.write(hdr.c_str(), static_cast<std::streamsize>(hdr.size()));
        npy.write(reinterpret_cast<const char*>(data),
                  static_cast<std::streamsize>(h * w) * sizeof(float));
    }
}

}  // namespace depth_engine_utils
}  // namespace dedalus
