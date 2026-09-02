//==============================================================================
// platform/Canvas.cpp
//==============================================================================

#include "platform/Canvas.h"

#include <cstdio>

#include "pe/math/MathUtil.h"

namespace platform {

using pe::Abs;
using pe::Clamp;
using pe::Max;
using pe::Sqrt;

namespace {

//------------------------------------------------------------------------------
// 3x5 点阵字模。每个字符 5 行，每行低 3 位是像素。
//------------------------------------------------------------------------------
struct Glyph {
    char c;
    std::uint8_t rows[5];
};

constexpr Glyph kFont[] = {
    {'0', {0b111, 0b101, 0b101, 0b101, 0b111}},
    {'1', {0b010, 0b110, 0b010, 0b010, 0b111}},
    {'2', {0b111, 0b001, 0b111, 0b100, 0b111}},
    {'3', {0b111, 0b001, 0b111, 0b001, 0b111}},
    {'4', {0b101, 0b101, 0b111, 0b001, 0b001}},
    {'5', {0b111, 0b100, 0b111, 0b001, 0b111}},
    {'6', {0b111, 0b100, 0b111, 0b101, 0b111}},
    {'7', {0b111, 0b001, 0b010, 0b010, 0b010}},
    {'8', {0b111, 0b101, 0b111, 0b101, 0b111}},
    {'9', {0b111, 0b101, 0b111, 0b001, 0b111}},
    {'A', {0b111, 0b101, 0b111, 0b101, 0b101}},
    {'B', {0b110, 0b101, 0b110, 0b101, 0b110}},
    {'C', {0b111, 0b100, 0b100, 0b100, 0b111}},
    {'D', {0b110, 0b101, 0b101, 0b101, 0b110}},
    {'E', {0b111, 0b100, 0b110, 0b100, 0b111}},
    {'F', {0b111, 0b100, 0b110, 0b100, 0b100}},
    {'G', {0b111, 0b100, 0b101, 0b101, 0b111}},
    {'H', {0b101, 0b101, 0b111, 0b101, 0b101}},
    {'I', {0b111, 0b010, 0b010, 0b010, 0b111}},
    {'J', {0b001, 0b001, 0b001, 0b101, 0b111}},
    {'K', {0b101, 0b101, 0b110, 0b101, 0b101}},
    {'L', {0b100, 0b100, 0b100, 0b100, 0b111}},
    {'M', {0b101, 0b111, 0b111, 0b101, 0b101}},
    {'N', {0b101, 0b111, 0b111, 0b111, 0b101}},
    {'O', {0b111, 0b101, 0b101, 0b101, 0b111}},
    {'P', {0b111, 0b101, 0b111, 0b100, 0b100}},
    {'Q', {0b111, 0b101, 0b101, 0b111, 0b001}},
    {'R', {0b111, 0b101, 0b110, 0b101, 0b101}},
    {'S', {0b111, 0b100, 0b111, 0b001, 0b111}},
    {'T', {0b111, 0b010, 0b010, 0b010, 0b010}},
    {'U', {0b101, 0b101, 0b101, 0b101, 0b111}},
    {'V', {0b101, 0b101, 0b101, 0b101, 0b010}},
    {'W', {0b101, 0b101, 0b111, 0b111, 0b101}},
    {'X', {0b101, 0b101, 0b010, 0b101, 0b101}},
    {'Y', {0b101, 0b101, 0b010, 0b010, 0b010}},
    {'Z', {0b111, 0b001, 0b010, 0b100, 0b111}},
    {'/', {0b001, 0b001, 0b010, 0b100, 0b100}},
    {'-', {0b000, 0b000, 0b111, 0b000, 0b000}},
    {'.', {0b000, 0b000, 0b000, 0b000, 0b010}},
    {':', {0b000, 0b010, 0b000, 0b010, 0b000}},
    {'+', {0b000, 0b010, 0b111, 0b010, 0b000}},
    {'\'', {0b010, 0b010, 0b000, 0b000, 0b000}},
};

const Glyph* FindGlyph(char c) noexcept {
    for (const Glyph& g : kFont) {
        if (g.c == c) return &g;
    }
    return nullptr;
}

}  // namespace

//==============================================================================

void Canvas::Rect(int x, int y, int w, int h, std::uint32_t color) {
    for (int j = 0; j < h; ++j) {
        for (int i = 0; i < w; ++i) Set(x + i, y + j, color);
    }
}

void Canvas::Bar(int x, int y, int w, int h, real fraction, std::uint32_t fill,
                 std::uint32_t back) {
    Rect(x, y, w, h, back);
    const int filled = static_cast<int>(Clamp(fraction, real(0), real(1)) * real(w));
    Rect(x, y, filled, h, fill);
}

void Canvas::TaperedBar(real x0, real y0, real w0, real x1, real y1, real w1,
                        std::uint32_t color) {
    const real dx = x1 - x0;
    const real dy = y1 - y0;
    const real length = Max(Sqrt(dx * dx + dy * dy), real(1e-4));

    // 沿"长的那个轴"逐行（或逐列）填，而不是沿线段撒方块：撒方块的边缘会毛，
    // 而枪身、车身这类大块实体，边缘毛了一眼就看出来是拼的。
    if (Abs(dy) >= Abs(dx)) {
        const real stretch = length / Max(Abs(dy), real(1e-4));
        const int step = dy >= real(0) ? 1 : -1;
        const int rows = static_cast<int>(Abs(dy)) + 1;
        for (int i = 0; i < rows; ++i) {
            const real t = rows > 1 ? static_cast<real>(i) / static_cast<real>(rows - 1)
                                    : real(0);
            const real cx = x0 + dx * t;
            const real w = (w0 + (w1 - w0) * t) * stretch;
            const int y = static_cast<int>(y0) + i * step;
            Rect(static_cast<int>(cx - w * real(0.5) + real(0.5)), y,
                 Max(1, static_cast<int>(w + real(0.5))), 1, color);
        }
    } else {
        const real stretch = length / Max(Abs(dx), real(1e-4));
        const int step = dx >= real(0) ? 1 : -1;
        const int cols = static_cast<int>(Abs(dx)) + 1;
        for (int i = 0; i < cols; ++i) {
            const real t = cols > 1 ? static_cast<real>(i) / static_cast<real>(cols - 1)
                                    : real(0);
            const real cy = y0 + dy * t;
            const real w = (w0 + (w1 - w0) * t) * stretch;
            const int x = static_cast<int>(x0) + i * step;
            Rect(x, static_cast<int>(cy - w * real(0.5) + real(0.5)), 1,
                 Max(1, static_cast<int>(w + real(0.5))), color);
        }
    }
}

void Canvas::AddLight(int x, int y, int r, int g, int b) {
    if (x < 0 || x >= m_width || y < 0 || y >= m_height) return;
    const std::uint32_t p = Get(x, y);
    Set(x, y,
        Pack(static_cast<int>((p >> 16) & 0xFF) + r,
             static_cast<int>((p >> 8) & 0xFF) + g,
             static_cast<int>(p & 0xFF) + b));
}

void Canvas::Text(int x, int y, const std::string& text, std::uint32_t color,
                  int scale) {
    int cursor = x;
    for (const char c : text) {
        if (c == ' ') {
            cursor += 4 * scale;
            continue;
        }
        const Glyph* glyph = FindGlyph(c);
        if (glyph != nullptr) {
            for (int row = 0; row < 5; ++row) {
                for (int col = 0; col < 3; ++col) {
                    if ((glyph->rows[row] >> (2 - col)) & 1) {
                        Rect(cursor + col * scale, y + row * scale, scale, scale, color);
                    }
                }
            }
        }
        cursor += 4 * scale;
    }
}

void Canvas::CenteredText(int y, const std::string& text, std::uint32_t color,
                          int scale) {
    Text((m_width - TextWidth(text, scale)) / 2, y, text, color, scale);
}

bool Canvas::SavePpm(const std::string& path) const {
    std::FILE* file = std::fopen(path.c_str(), "wb");
    if (file == nullptr) return false;

    std::fprintf(file, "P6\n%d %d\n255\n", m_width, m_height);
    for (int y = 0; y < m_height; ++y) {
        for (int x = 0; x < m_width; ++x) {
            const std::uint32_t p = m_pixels[static_cast<std::size_t>(y * m_width + x)];
            const unsigned char rgb[3] = {static_cast<unsigned char>((p >> 16) & 0xFF),
                                          static_cast<unsigned char>((p >> 8) & 0xFF),
                                          static_cast<unsigned char>(p & 0xFF)};
            std::fwrite(rgb, 1, 3, file);
        }
    }
    std::fclose(file);
    return true;
}

}  // namespace platform
