#pragma once
//==============================================================================
// game/Renderer.h
//
// 控制台光线投射渲染器。
//
//------------------------------------------------------------------------------
// 它整个就是一层 world.Raycast()
//------------------------------------------------------------------------------
// 这个渲染器没有任何自己的几何数据 —— 没有网格、没有顶点缓冲、没有场景图。
// 每一个屏幕像素就是一次**物理引擎的射线查询**：
//
//     for 每个像素 (x, y):
//         dir = 由相机朝向和像素位置算出的方向
//         world.Raycast({eye, dir}, hit)   <- 引擎干活
//         按 hit.distance 选字符、按 hit.body 是谁选颜色
//
// 这么做有两个好处，而且都不是"取巧"：
//
//   1. **它自动就是正确的 3D**。台阶、不同高度的箱子、比玩家矮的掩体、
//      站在高处的敌人 —— 全都不需要任何特殊处理，因为射线打的是真正的
//      三维几何。经典的 Wolfenstein 式渲染器只投影一根水平射线、
//      假设墙都一样高，那些东西它一个都画不出来。
//
//   2. **画面里看到的就是物理里存在的**。渲染和碰撞用的是同一份数据、
//      同一个查询函数。不存在"看着能过去实际撞墙"这种渲染与物理不一致的 bug ——
//      那是自己写渲染层时最烦人的一类问题。
//
// 代价是每帧要打 W×H 条射线。100×36 = 3600 条，实测在这台机器上约 1.5 ms，
// 完全够用。真接了 GPU 渲染之后这一层就整个扔掉，换成
// `world.GetDebugDrawData()` + 真正的网格。
//
//------------------------------------------------------------------------------
// 为什么不是 Wolfenstein 那种"每列一根射线"
//------------------------------------------------------------------------------
// 那种做法要求"所有墙一样高、地面绝对平"，才能用 1/distance 算出墙的高度。
// 我们的地图里有台阶、有斜坡、有各种高度的箱子，那个假设直接就不成立。
// 逐像素反而更简单：不需要算墙高，射线打到哪儿就是哪儿。
//==============================================================================

#include <string>
#include <vector>

#include "game/GameTypes.h"

namespace game {

class Game;

//------------------------------------------------------------------------------
// 相机
//------------------------------------------------------------------------------
struct Camera {
    Vec3 position = Vec3::Zero();
    real yaw = real(0);    ///< 弧度，0 朝 +X，逆时针为正
    real pitch = real(0);  ///< 弧度，正为抬头
    real fovY = DegToRad(real(60));

    /// 视线方向（单位向量）。
    Vec3 Forward() const noexcept {
        const real cp = Cos(pitch);
        return Vec3(Cos(yaw) * cp, Sin(pitch), Sin(yaw) * cp);
    }
    /// 右手方向（水平面内，不随俯仰变化 —— 免得抬头时准星横着漂）。
    Vec3 Right() const noexcept { return Vec3(-Sin(yaw), real(0), Cos(yaw)); }

    /// "上"必须由 forward × right 叉出来。直接用世界 +Y 的话，
    /// 抬头/低头时画面会产生剪切变形。
    Vec3 Up() const noexcept { return Cross(Right(), Forward()).Normalized(); }
};

//------------------------------------------------------------------------------
// 世界点 -> 屏幕格子
//
// 光线投射是"从屏幕出发问世界"，而特效（曳光弹、命中火花）是**已知世界坐标、
// 要知道画在屏幕哪儿** —— 正好反过来，所以需要一次正向投影。
//
// 两个渲染器共用这一份：ASCII 版和像素版的差别只有 aspectCorrection
// （控制台的字符是竖着长的，见 Renderer.cpp）。投影写两遍的话，
// 迟早会出现"火花在像素版上是对的、在字符版上偏半个屏幕"这种事。
//
// 返回 false 表示这个点在相机背后或者视野外，不该画。
// outDepth 是**相机前方的距离**（米），拿去和光线投射的深度比，
// 就能让被墙挡住的火花不穿墙。
//------------------------------------------------------------------------------
bool ProjectToScreen(const Camera& camera, const Vec3& world, int width, int height,
                     real aspectCorrection, real& outX, real& outY, real& outDepth);

/// 把一条线段裁到相机前方（近平面）。曳光弹经常有一端在身后 ——
/// 不裁的话那一端会被投影到屏幕另一侧，画出一条横穿画面的假线。
/// 返回 false 表示整条线段都在背后。
bool ClipSegmentToCamera(const Camera& camera, Vec3& from, Vec3& to);

//------------------------------------------------------------------------------
// 渲染器
//------------------------------------------------------------------------------
class Renderer {
public:
    Renderer(int width, int height);

    /// 关掉 ANSI 颜色（输出被重定向到文件、或者终端不支持时）。
    void SetColorEnabled(bool enabled) noexcept { m_color = enabled; }
    bool ColorEnabled() const noexcept { return m_color; }

    int Width() const noexcept { return m_width; }
    int Height() const noexcept { return m_height; }

    /// 把整个场景画进内部帧缓冲。
    void RenderScene(const Game& game, const Camera& camera);

    /// 在帧缓冲上叠加 HUD（血量、弹药、准星、提示）。
    void DrawHud(const Game& game);

    /// 在指定行居中写一行字（标题画面、结算画面用）。
    void DrawCenteredText(int row, const std::string& text, int colorCode = 7);

    void Clear();

    /// 把帧缓冲刷到控制台。用一次 fwrite 输出整帧，
    /// 逐字符 putchar 在 Windows 控制台上慢到会闪。
    void Present() const;

    /// 上一帧打了多少条射线、花了多久（诊断用）。
    std::size_t LastRayCount() const noexcept { return m_rayCount; }

private:
    struct Cell {
        char glyph;
        std::uint8_t color;  ///< ANSI 30-37 / 90-97 的低位编码
    };

    void Put(int x, int y, char glyph, std::uint8_t color);
    void PutText(int x, int y, const std::string& text, std::uint8_t color);
    /// 把曳光弹和命中火花叠到场景上（和像素版共用同一份投影）。
    void DrawEffects(const Game& game, const Camera& camera);

    int m_width;
    int m_height;
    bool m_color = true;
    std::vector<Cell> m_cells;
    /// 每个格子上场景的距离（米）。光线投射本来就算出来了，存下来给特效做遮挡。
    std::vector<real> m_depth;
    std::size_t m_rayCount = 0;

    /// 输出缓冲，复用避免每帧分配
    mutable std::string m_out;
};

}  // namespace game
