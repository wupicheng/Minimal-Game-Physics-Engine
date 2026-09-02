#pragma once
//==============================================================================
// platform/Canvas.h
//
// 一块 32 位像素画布，外加画矩形/斜条/文字的最小工具集。
//
//------------------------------------------------------------------------------
// 为什么它在 platform/ 而不在某个游戏目录里
//------------------------------------------------------------------------------
// 这个仓库现在有两个游戏（`game/` 的 FPS 和 `racing/` 的赛车），它们的渲染
// **算法**完全不同，但"往一块内存里涂颜色、最后交给窗口或者存成图片"这件事
// 是同一件事。第二个游戏出现的那一刻，这段代码就不再属于第一个游戏了。
//
// 它不知道任何游戏概念，也不知道物理引擎的存在 —— 只有像素。
//
//------------------------------------------------------------------------------
// 为什么是 BGRA
//------------------------------------------------------------------------------
// Windows 的 DIB（`BI_RGB` + 32bpp）就是这个字节序，`StretchDIBits` 可以直接
// 吃这块内存，不需要每帧转换。
//==============================================================================

#include <cstdint>
#include <string>
#include <vector>

#include "pe/core/Types.h"

namespace platform {

using pe::real;

class Canvas {
public:
    Canvas(int width, int height)
        : m_width(width), m_height(height),
          m_pixels(static_cast<std::size_t>(width * height), 0u) {}

    int Width() const noexcept { return m_width; }
    int Height() const noexcept { return m_height; }

    const std::uint32_t* Data() const noexcept { return m_pixels.data(); }
    std::uint32_t* Data() noexcept { return m_pixels.data(); }

    static std::uint32_t Pack(int r, int g, int b) noexcept {
        const auto clamp8 = [](int v) { return v < 0 ? 0 : (v > 255 ? 255 : v); };
        return (static_cast<std::uint32_t>(clamp8(r)) << 16) |
               (static_cast<std::uint32_t>(clamp8(g)) << 8) |
               static_cast<std::uint32_t>(clamp8(b));
    }

    void Set(int x, int y, std::uint32_t color) noexcept {
        if (x < 0 || x >= m_width || y < 0 || y >= m_height) return;
        m_pixels[static_cast<std::size_t>(y * m_width + x)] = color;
    }

    std::uint32_t Get(int x, int y) const noexcept {
        if (x < 0 || x >= m_width || y < 0 || y >= m_height) return 0;
        return m_pixels[static_cast<std::size_t>(y * m_width + x)];
    }

    void Fill(std::uint32_t color) {
        for (std::uint32_t& p : m_pixels) p = color;
    }

    //-- 画 ---------------------------------------------------------------------

    void Rect(int x, int y, int w, int h, std::uint32_t color);

    /// 进度条（血条、油门、刹车条都用它）。
    void Bar(int x, int y, int w, int h, real fraction, std::uint32_t fill,
             std::uint32_t back);

    /// 一条会变粗变细的斜条：从 (x0,y0) 宽 w0 画到 (x1,y1) 宽 w1。
    ///
    /// **w 是垂直于轴的宽度，不是画上去那一段的长度。** 一条斜着的宽条，
    /// 被水平线切出来的那一段比它本身的宽要长 —— 长 L/|dy| 倍。漏掉这个换算的
    /// 症状是"越斜的零件画得越细"，一把 35 度的枪会瘦掉五分之一。
    void TaperedBar(real x0, real y0, real w0, real x1, real y1, real w1,
                    std::uint32_t color);

    /// **加法**混合一个发光像素。特效是"额外的光"，不是"另一块颜色" ——
    /// 直接覆盖的话，暗处的火花会显得比亮处的还暗，完全不像在发光。
    void AddLight(int x, int y, int r, int g, int b);

    /// 极简 3x5 点阵字模：数字 + 大写字母 + 几个符号。
    /// 手写这一小张表比引入字体库省事得多，而且不需要任何资源文件。
    /// **只认大写**，小写会被当成缺字跳过。
    void Text(int x, int y, const std::string& text, std::uint32_t color,
              int scale = 1);

    /// 一行居中的字。
    void CenteredText(int y, const std::string& text, std::uint32_t color,
                      int scale = 1);

    /// 一个字符串按当前字号占多宽（像素）。排版时要用。
    static int TextWidth(const std::string& text, int scale = 1) noexcept {
        return static_cast<int>(text.size()) * 4 * scale;
    }

    /// 存成 PPM（P6）。它是最简单的无损位图格式，任何看图软件都认，
    /// 而且**不需要任何图形库就能验证渲染结果** —— 无头环境里这是唯一能
    /// 确认"画出来的东西对不对"的手段。
    bool SavePpm(const std::string& path) const;

private:
    int m_width;
    int m_height;
    std::vector<std::uint32_t> m_pixels;
};

}  // namespace platform
