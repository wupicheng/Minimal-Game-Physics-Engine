#pragma once
//==============================================================================
// game/PixelRenderer.h
//
// **真·像素**渲染器：逐像素光线投射，输出 32 位 RGB 帧缓冲。
//
//------------------------------------------------------------------------------
// 和 ASCII 版的关系
//------------------------------------------------------------------------------
// 渲染**算法完全一样** —— 两者都是"每个像素一次 world.Raycast()"。
// 区别只在于最后一步把命中结果写成什么：
//     ASCII 版  -> 从 "@%#*+=~-:." 里挑一个字符（10 级灰度）
//     这一版    -> 写一个 24 位 RGB 值（1600 万色 + 真正的距离雾）
//
// 也就是说 ASCII 版从来不是"简化版渲染"，它只是把同样的逐像素结果输出到了一个
// 只有 10 级灰度的显示设备上。控制台就是那个设备。
//
//------------------------------------------------------------------------------
// 分辨率为什么这么低
//------------------------------------------------------------------------------
// 实测这台机器上一条 `world.Raycast` 约 **733 ns**（穿过宽相位 DDA + 逐碰撞体
// 精确求交）。于是分辨率直接被射线预算锁死：
//
//     256×144 =  36,864 条 -> 27 ms/帧 -> 37 fps
//     320×180 =  57,600 条 -> 42 ms/帧 -> 24 fps
//     640×360 = 230,400 条 -> 169 ms/帧 -> 6 fps
//
// 所以内部按 256×144 渲染，再整数倍放大贴到窗口上 —— 画面是有颗粒感的复古风，
// 但每一个颗粒都是一次真实的三维射线求交，不是贴图也不是精灵。
//
// 想要更高分辨率有两条路，都记在 UPGRADE_NOTES 里：
//   1. 多线程（逐像素天然可并行）—— 但引擎的查询目前**不是线程安全的**，
//      `PhysicsWorld` 和 `UniformGrid` 里有复用的 mutable 暂存缓冲
//   2. 传统 Wolfenstein 式"每列一根射线" —— 快得多，但要求所有墙等高，
//      我们这张图有台阶、矮掩体、不同高度的箱子，那个假设不成立
//==============================================================================

#include <cstdint>
#include <string>
#include <vector>

#include "game/GameTypes.h"
#include "platform/Canvas.h"

namespace game {

class Game;
struct Camera;
struct Tracer;  ///< 定义在 Game.h —— 特效是游戏状态，不是渲染器的私有数据
struct Impact;

//------------------------------------------------------------------------------
// 帧缓冲
//
// 就是 `platform::Canvas`。它原本是这个文件里的一个类，`racing/` 出现之后
// 搬到了 platform/ —— "往一块内存里涂颜色"这件事，两个游戏做的是同一件事，
// 和 FPS 没有任何关系（见 platform/Canvas.h 顶部）。
//
// 这里留一个别名，是因为"帧缓冲"才是渲染代码里该出现的词；
// 而且原来的名字散在 main.cpp 和截图代码里，没必要为了改名去动它们。
//------------------------------------------------------------------------------
using Framebuffer = platform::Canvas;

//------------------------------------------------------------------------------
// 像素渲染器
//------------------------------------------------------------------------------
class PixelRenderer {
public:
    PixelRenderer(int width, int height)
        : m_frame(width, height),
          m_depth(static_cast<std::size_t>(width * height), real(1e9)) {}

    const Framebuffer& Frame() const noexcept { return m_frame; }
    Framebuffer& Frame() noexcept { return m_frame; }

    /// 逐像素光线投射整个场景，然后叠上世界里的特效（曳光弹、命中火花）。
    void RenderScene(const Game& game, const Camera& camera);

    /// 叠加 HUD：枪、准星、命中标记、血条、弹药、受伤红晕。
    void DrawHud(const Game& game);

    /// 结算横幅（"YOU DIED" / "MISSION COMPLETE"）加一行小字提示。
    ///
    /// 少了它，图形模式下"打输了"和"程序崩了"在玩家眼里是**一模一样**的：
    /// 画面一停、窗口就没了。字符模式一直有这个提示，图形模式当初漏掉了。
    /// hint 为空时不画第二行。
    void DrawBanner(const std::string& text, const std::string& hint,
                    std::uint32_t color);

    std::size_t LastRayCount() const noexcept { return m_rayCount; }

private:
    /// 深度测试：这个像素上，场景有多远。让被墙挡住的火花不穿墙。
    real DepthAt(int x, int y) const;

    void DrawTracer(const Camera& camera, const Tracer& tracer);
    void DrawImpact(const Camera& camera, const Impact& impact);
    void DrawWeapon(const Game& game);
    void DrawHitMarker(const Game& game, int cx, int cy);

    Framebuffer m_frame;
    /// 每个像素上场景的距离（米）。光线投射本来就算出来了，顺手存下来 ——
    /// 有了它，世界里的特效才能正确地被前景遮挡。
    std::vector<real> m_depth;
    std::size_t m_rayCount = 0;
};

}  // namespace game
