#include "dedalus/sensing/depth_debug_annotator.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

namespace dedalus {
namespace {

// ── Minimal 5×5 bitmap font ──────────────────────────────────────────────────
// Each character = 5 bytes, one per row (top→bottom).
// Each byte: bits 4..0 correspond to columns left→right (MSB = leftmost).
// Render: pixel at (col, row) is lit if (rows[row] >> (4 - col)) & 1.
//
// Characters: space, A-Z, 0-9, _, -, :
struct Glyph { uint8_t rows[5]; };

// clang-format off
static constexpr Glyph kGlyphs[128] = {
    /* 0x00-0x1F: control — blank */
    {},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},
    {},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},
    /* 0x20 ' ' */ {},
    /* 0x21 '!' */ {{0x04,0x04,0x04,0x00,0x04}},
    /* 0x22-0x26 */ {},{},{},{},{},
    /* 0x27 */ {},
    /* 0x28-0x2C */ {},{},{},{},{},
    /* 0x2D '-' */ {{0x00,0x00,0x0E,0x00,0x00}},
    /* 0x2E '.' */ {{0x00,0x00,0x00,0x00,0x04}},
    /* 0x2F '/' */ {{0x01,0x02,0x04,0x08,0x10}},
    /* 0x30 '0' */ {{0x0E,0x13,0x15,0x19,0x0E}},
    /* 0x31 '1' */ {{0x04,0x0C,0x04,0x04,0x0E}},
    /* 0x32 '2' */ {{0x0E,0x11,0x06,0x08,0x1F}},
    /* 0x33 '3' */ {{0x0E,0x11,0x06,0x11,0x0E}},
    /* 0x34 '4' */ {{0x02,0x06,0x0A,0x1F,0x02}},
    /* 0x35 '5' */ {{0x1F,0x10,0x1E,0x01,0x1E}},
    /* 0x36 '6' */ {{0x0E,0x10,0x1E,0x11,0x0E}},
    /* 0x37 '7' */ {{0x1F,0x01,0x02,0x04,0x04}},
    /* 0x38 '8' */ {{0x0E,0x11,0x0E,0x11,0x0E}},
    /* 0x39 '9' */ {{0x0E,0x11,0x0F,0x01,0x0E}},
    /* 0x3A ':' */ {{0x00,0x04,0x00,0x04,0x00}},
    /* 0x3B ';' */ {{0x00,0x04,0x00,0x04,0x08}},
    /* 0x3C '<' */ {{0x02,0x04,0x08,0x04,0x02}},
    /* 0x3D '=' */ {{0x00,0x1F,0x00,0x1F,0x00}},
    /* 0x3E '>' */ {{0x08,0x04,0x02,0x04,0x08}},
    /* 0x3F '?' */ {{0x0E,0x11,0x06,0x00,0x04}},
    /* 0x40 '@' */ {{0x0E,0x11,0x17,0x16,0x0F}},
    /* 0x41 'A' */ {{0x0E,0x11,0x1F,0x11,0x11}},
    /* 0x42 'B' */ {{0x1E,0x11,0x1E,0x11,0x1E}},
    /* 0x43 'C' */ {{0x0F,0x10,0x10,0x10,0x0F}},
    /* 0x44 'D' */ {{0x1E,0x11,0x11,0x11,0x1E}},
    /* 0x45 'E' */ {{0x1F,0x10,0x1E,0x10,0x1F}},
    /* 0x46 'F' */ {{0x1F,0x10,0x1E,0x10,0x10}},
    /* 0x47 'G' */ {{0x0F,0x10,0x17,0x11,0x0F}},
    /* 0x48 'H' */ {{0x11,0x11,0x1F,0x11,0x11}},
    /* 0x49 'I' */ {{0x1F,0x04,0x04,0x04,0x1F}},
    /* 0x4A 'J' */ {{0x07,0x01,0x01,0x11,0x0E}},
    /* 0x4B 'K' */ {{0x11,0x12,0x1C,0x12,0x11}},
    /* 0x4C 'L' */ {{0x10,0x10,0x10,0x10,0x1F}},
    /* 0x4D 'M' */ {{0x11,0x1B,0x15,0x11,0x11}},
    /* 0x4E 'N' */ {{0x11,0x19,0x15,0x13,0x11}},
    /* 0x4F 'O' */ {{0x0E,0x11,0x11,0x11,0x0E}},
    /* 0x50 'P' */ {{0x1E,0x11,0x1E,0x10,0x10}},
    /* 0x51 'Q' */ {{0x0E,0x11,0x15,0x12,0x0D}},
    /* 0x52 'R' */ {{0x1E,0x11,0x1E,0x14,0x13}},
    /* 0x53 'S' */ {{0x0F,0x10,0x0E,0x01,0x1E}},
    /* 0x54 'T' */ {{0x1F,0x04,0x04,0x04,0x04}},
    /* 0x55 'U' */ {{0x11,0x11,0x11,0x11,0x0E}},
    /* 0x56 'V' */ {{0x11,0x11,0x0A,0x0A,0x04}},
    /* 0x57 'W' */ {{0x11,0x11,0x15,0x1B,0x11}},
    /* 0x58 'X' */ {{0x11,0x0A,0x04,0x0A,0x11}},
    /* 0x59 'Y' */ {{0x11,0x0A,0x04,0x04,0x04}},
    /* 0x5A 'Z' */ {{0x1F,0x02,0x04,0x08,0x1F}},
    /* 0x5B '[' */ {{0x06,0x04,0x04,0x04,0x06}},
    /* 0x5C '\' */ {{0x10,0x08,0x04,0x02,0x01}},
    /* 0x5D ']' */ {{0x06,0x02,0x02,0x02,0x06}},
    /* 0x5E '^' */ {{0x04,0x0A,0x00,0x00,0x00}},
    /* 0x5F '_' */ {{0x00,0x00,0x00,0x00,0x1F}},
    /* 0x60-0x7F: lowercase → map to upper in render, rest blank */
    {},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},
    {},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},
};
// clang-format on

// ── Color helpers ─────────────────────────────────────────────────────────────

struct Rgb { uint8_t r, g, b; };

// Log-luminance depth → greyscale. white=near, black=far.
uint8_t depth_to_lum(float depth_m, float display_max_m, float log_denom) {
    const float d = std::min(depth_m, display_max_m);
    const float t = 1.0f - std::log1p(d) / log_denom;
    return static_cast<uint8_t>(std::clamp(t * 255.0f, 0.0f, 255.0f));
}

// Color coding for the filter panel (shared between all sources).
Rgb filter_color(float depth_m, bool valid, bool is_evidence,
                 float pitch_down_deg, float min_depth_m, float max_depth_m,
                 uint8_t grey_lum) {
    if (!valid)                  return {30U, 30U, 30U};  // invalid
    if (depth_m > max_depth_m)   return {10U, 10U, 50U};  // too far
    if (depth_m < min_depth_m) {
        if (pitch_down_deg > 15.0f) return {220U,  40U,  0U};  // OOD noise
        if (pitch_down_deg <  5.0f) return {220U, 220U,  0U};  // props/arms expected
        return                           {220U, 130U,  0U};    // ambiguous
    }
    if (is_evidence) return {0U, 220U, 220U};              // evidence = cyan
    return {grey_lum, grey_lum, grey_lum};
}

// Source name → header background color.
Rgb source_color(const std::string& name) {
    if (name.find("visual_onnx") != std::string::npos) return {0U,  90U, 30U};
    if (name.find("airsim_gt")   != std::string::npos) return {80U, 60U, 0U};
    return {50U, 50U, 50U};
}

// ── Label rendering ────────────────────────────────────────────────────────────

static constexpr int kLabelH    = 8;  // header strip height in pixels
static constexpr int kCharW     = 6;  // glyph width (5 pixels + 1 gap)
static constexpr int kCharH     = 5;  // glyph height

// Draw glyph for 'ch' at pixel (px, py) into rgb buffer of stride frame_w.
void draw_char(std::vector<uint8_t>& rgb, int frame_w, int frame_h,
               char ch, int px, int py, Rgb fg) {
    const int idx = static_cast<int>(std::toupper(static_cast<unsigned char>(ch)));
    if (idx < 0 || idx >= 128) return;
    const Glyph& g = kGlyphs[idx];
    for (int row = 0; row < kCharH && (py + row) < frame_h; ++row) {
        for (int col = 0; col < 5 && (px + col) < frame_w; ++col) {
            if ((g.rows[row] >> (4 - col)) & 1) {
                const std::size_t i =
                    static_cast<std::size_t>((py + row) * frame_w + (px + col)) * 3U;
                rgb[i + 0U] = fg.r;
                rgb[i + 1U] = fg.g;
                rgb[i + 2U] = fg.b;
            }
        }
    }
}

// Draw label text starting at (px, py).
void draw_label(std::vector<uint8_t>& rgb, int frame_w, int frame_h,
                const std::string& text, int px, int py) {
    int x = px;
    for (char ch : text) {
        if (x + kCharW >= frame_w) break;
        draw_char(rgb, frame_w, frame_h, ch, x, py, {255U, 255U, 255U});
        x += kCharW;
    }
}

// Fill a horizontal band [y0, y0+h) × [x0, x0+w) with color c.
void fill_band(std::vector<uint8_t>& rgb, int frame_w, int frame_h,
               int x0, int y0, int w, int h, Rgb c) {
    for (int y = y0; y < y0 + h && y < frame_h; ++y) {
        for (int x = x0; x < x0 + w && x < frame_w; ++x) {
            const std::size_t i =
                static_cast<std::size_t>(y * frame_w + x) * 3U;
            rgb[i + 0U] = c.r;
            rgb[i + 1U] = c.g;
            rgb[i + 2U] = c.b;
        }
    }
}

// ── Panel rendering ────────────────────────────────────────────────────────────

// Get metric depth for pixel index i from a panel.
float panel_depth_m(const DepthDebugPanel& p, int idx, float log_denom) {
    (void)log_denom;
    if (p.inverse_depth) {
        const float id = p.inverse_depth[static_cast<std::size_t>(idx)];
        if (!std::isfinite(id) || id <= 1e-6f) return -1.0f;  // invalid
        return p.params.scale / id;
    }
    if (p.depth_m_data) {
        const float dm = p.depth_m_data[static_cast<std::size_t>(idx)];
        if (!std::isfinite(dm) || dm <= 0.0f) return -1.0f;
        return dm;
    }
    return -1.0f;
}

// Render one panel row (raw + filter halves) into the frame buffer.
// row_y0: top pixel row in the full frame for this panel.
// panel_w, panel_h: rendered dimensions (== frame_w/2, frame_h/rows).
// Scales the panel to (panel_w × panel_h) from (p.width × p.height).
void render_panel_row(std::vector<uint8_t>& rgb, int frame_w, int frame_h,
                      const DepthDebugPanel& p, int row_y0,
                      int panel_w, int panel_h,
                      float display_max_m, float log_denom) {

    const bool valid_panel = (p.width > 0) && (p.height > 0) &&
                             (p.inverse_depth != nullptr || p.depth_m_data != nullptr);

    // Header strip (background + label).
    const Rgb hdr_color = source_color(p.source_name);
    fill_band(rgb, frame_w, frame_h, 0, row_y0, frame_w, kLabelH, hdr_color);
    draw_label(rgb, frame_w, frame_h, p.source_name, 2, row_y0 + 1);

    if (!valid_panel) {
        // Grey fill below header.
        fill_band(rgb, frame_w, frame_h, 0, row_y0 + kLabelH,
                  frame_w, panel_h - kLabelH, {60U, 60U, 60U});
        return;
    }

    const int bw = (p.params.grid_cols > 0) ? (p.width  / p.params.grid_cols) : 0;
    const int bh = (p.params.grid_rows > 0) ? (p.height / p.params.grid_rows) : 0;

    for (int ry = kLabelH; ry < panel_h; ++ry) {
        const int src_y = ((ry - kLabelH) * p.height) / std::max(panel_h - kLabelH, 1);
        const int out_y = row_y0 + ry;
        if (out_y >= frame_h) break;

        for (int rx = 0; rx < panel_w; ++rx) {
            const int src_x = (rx * p.width) / std::max(panel_w, 1);
            if (src_x >= p.width || src_y >= p.height) continue;

            const int idx = src_y * p.width + src_x;
            const float dm = panel_depth_m(p, idx, log_denom);
            const bool valid_px = (dm > 0.0f);
            const uint8_t lum = valid_px
                ? depth_to_lum(dm, display_max_m, log_denom)
                : uint8_t{0U};

            // Left half: raw depth greyscale.
            {
                const std::size_t i =
                    static_cast<std::size_t>(out_y * frame_w + rx) * 3U;
                rgb[i + 0U] = lum;
                rgb[i + 1U] = lum;
                rgb[i + 2U] = lum;
            }

            // Right half: filter view.
            {
                // A pixel is counted as evidence if it is the center of a grid cell.
                const bool evidence = valid_px
                    && dm >= p.params.min_depth_m
                    && dm <= p.params.max_depth_m
                    && bw > 0 && bh > 0
                    && (src_x % bw == bw / 2)
                    && (src_y % bh == bh / 2);
                const Rgb fc = filter_color(dm, valid_px, evidence,
                                            p.pitch_down_deg,
                                            p.params.min_depth_m,
                                            p.params.max_depth_m,
                                            lum);
                const std::size_t i =
                    static_cast<std::size_t>(out_y * frame_w + panel_w + rx) * 3U;
                rgb[i + 0U] = fc.r;
                rgb[i + 1U] = fc.g;
                rgb[i + 2U] = fc.b;
            }
        }
    }
}

}  // namespace

// ── DepthDebugAnnotator ────────────────────────────────────────────────────────

DepthDebugAnnotator::DepthDebugAnnotator(DepthDebugAnnotatorConfig config)
    : config_(std::move(config)) {}

DepthDebugAnnotator::~DepthDebugAnnotator() {
    if (pipe_ != nullptr) {
        const int rc = pclose(pipe_);
        if (rc != 0) {
            std::fprintf(stderr, "[DepthAnnotator] ffmpeg exited with code %d\n", rc);
        }
        pipe_ = nullptr;
    }
}

void DepthDebugAnnotator::open_pipe(int frame_w, int frame_h) {
    const std::string cmd =
        "ffmpeg -f rawvideo -pixel_format rgb24"
        " -video_size " + std::to_string(frame_w) + "x" + std::to_string(frame_h) +
        " -framerate 5 -i pipe:0"
        " -vcodec libx264 -crf 23 -pix_fmt yuv420p"
        " -movflags frag_keyframe+empty_moov+default_base_moof"
        " -y " + config_.output_path +
        " 2>/dev/null";
    std::fprintf(stderr, "[DepthAnnotator] opening pipe: %s\n", cmd.c_str());
    pipe_ = popen(cmd.c_str(), "w");
    if (pipe_ == nullptr) {
        std::fprintf(stderr,
            "[DepthAnnotator] ERROR: popen failed (errno=%d: %s) — MP4 disabled\n",
            errno, std::strerror(errno));
    }
}

void DepthDebugAnnotator::annotate(const DepthDebugPanel& primary,
                                   const DepthDebugPanel* eval) {
    if (config_.output_path.empty()) return;

    // Derive frame size from primary panel dimensions.
    const int W = (primary.width  > 0) ? primary.width  : 640;
    const int H = (primary.height > 0) ? primary.height : 360;

    // Layout: 2W wide (raw | filter), H per panel row.
    const int frame_w = 2 * W;
    const int frame_h = eval ? 2 * H : H;

    if (pipe_ == nullptr) {
        open_pipe(frame_w, frame_h);
        if (pipe_ == nullptr) return;
    }

    const float display_max_m = config_.display_max_m;
    const float log_denom     = std::log1p(display_max_m);

    std::vector<uint8_t> frame_rgb(
        static_cast<std::size_t>(frame_w * frame_h) * 3U, 0U);

    render_panel_row(frame_rgb, frame_w, frame_h,
                     primary, /*row_y0=*/0, W, H, display_max_m, log_denom);

    if (eval) {
        render_panel_row(frame_rgb, frame_w, frame_h,
                         *eval, /*row_y0=*/H, W, H, display_max_m, log_denom);
    }

    const std::size_t written =
        std::fwrite(frame_rgb.data(), 1U, frame_rgb.size(), pipe_);
    if (written != frame_rgb.size()) {
        std::fclose(pipe_);
        pipe_ = nullptr;
        std::fprintf(stderr,
            "[DepthAnnotator] pipe broken after %zu/%zu bytes — MP4 disabled\n",
            written, frame_rgb.size());
        return;
    }
    std::fflush(pipe_);
}

}  // namespace dedalus
