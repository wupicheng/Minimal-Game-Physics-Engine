#pragma once
//==============================================================================
// racing/RaceRenderer.h
//
// 逐像素光线投射 + 追车相机。
//
//------------------------------------------------------------------------------
// 和 FPS 那个渲染器是同一套算法，但**不是同一份代码**
//------------------------------------------------------------------------------
// 每个像素一次 `world.Raycast()` 这件事是一样的（见 game/Renderer.h 里的长篇
// 说明），共用的部分已经抽到了 platform/Canvas。不共用的是"命中之后画成什么"：
// FPS 按敌人/箱子/墙上色，赛车按路面/护墙/车身上色，而且路面的白线是**算**出来
// 的，不是几何。两个游戏的着色逻辑没有任何可复用的部分，硬合并只会得到一个
// 到处 if (是赛车游戏) 的函数。
//
//------------------------------------------------------------------------------
// 相机在车后面，这件事比听起来重要
//------------------------------------------------------------------------------
// 第一人称开车是开不了的：看不到车身姿态，就判断不出车正在滑还是在抓地。
// 追车相机必须做三件事，缺一个手感就废：
//   1. **平滑跟随**，不是硬贴在车后 —— 硬贴的话过弯时画面会跟着车一起甩
//   2. **看向车的前方**而不是车本身 —— 玩家要看的是即将到达的地方
//   3. **速度越快拉得越远、视野越广** —— 这是速度感的主要来源，
//      比屏幕上的数字管用得多
//==============================================================================

#include <string>
#include <vector>

#include "platform/Canvas.h"
#include "racing/RaceGame.h"

namespace racing {

//------------------------------------------------------------------------------
// 追车相机
//------------------------------------------------------------------------------
struct ChaseCamera {
    Vec3 position = Vec3::Zero();
    Vec3 target = Vec3::Zero();
    real fovY = DegToRad(real(65));

    /// 按车的位姿和速度更新一帧。dt 用来做平滑，所以必须是真实帧间隔。
    void Follow(const Vehicle& car, real dt);

    Vec3 Forward() const noexcept;
    Vec3 Right() const noexcept;
    Vec3 Up() const noexcept;

private:
    bool m_initialised = false;
};

//------------------------------------------------------------------------------
// 渲染器
//------------------------------------------------------------------------------
class RaceRenderer {
public:
    RaceRenderer(int width, int height) : m_canvas(width, height) {}

    const platform::Canvas& Frame() const noexcept { return m_canvas; }
    platform::Canvas& Frame() noexcept { return m_canvas; }

    void RenderScene(const RaceGame& race, const ChaseCamera& camera);
    void DrawHud(const RaceGame& race);
    void DrawBanner(const std::string& title, const std::string& hint,
                    std::uint32_t color);

    std::size_t LastRayCount() const noexcept { return m_rayCount; }

private:
    platform::Canvas m_canvas;
    std::size_t m_rayCount = 0;
};

}  // namespace racing
