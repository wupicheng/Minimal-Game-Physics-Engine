#pragma once
//==============================================================================
// racing/Input.h
//
// 两种驾驶员，输出同一种 `DriveInput`：
//
//   - **键盘**：真人开。用 GetAsyncKeyState 读"按住"状态而不是按键事件 ——
//     油门必须是"踩住就一直加速"。
//   - **机器人**：无人值守时自己把这场比赛跑完。
//
// 机器人存在的理由和 FPS 那边一模一样：它是**冒烟测试**。一场比赛跑完，
// 意味着悬挂、轮胎、摩擦圆、触发器、圈数、复位这一整条链路都真的在工作。
// 一段固定脚本（"前三秒全油门，然后左打死"）跑不完一圈，也就什么都验证不了。
//==============================================================================

#include <vector>

#include "racing/RaceGame.h"

namespace racing {

class IDriver {
public:
    virtual ~IDriver() = default;

    /// 采一帧输入。带着 `RaceGame` 是为了机器人 —— 它要知道下一个弯在哪。
    virtual DriveInput Poll(const RaceGame& race, real dt) = 0;
    virtual bool WantsQuit() const { return false; }
};

//------------------------------------------------------------------------------
// 键盘
//------------------------------------------------------------------------------
class KeyboardDriver final : public IDriver {
public:
    DriveInput Poll(const RaceGame& race, real dt) override;
    bool WantsQuit() const override { return m_quit; }
    /// 玩家按了复位键（手动把车放回赛道）
    bool WantsRespawn() const noexcept { return m_respawn; }

private:
    bool m_quit = false;
    bool m_respawn = false;
};

//------------------------------------------------------------------------------
// 机器人车手
//
// 沿着一条**赛车线**跑：赛道中线上取一圈路点，始终瞄准前方若干米处的那个点。
//
// 为什么是"瞄准前方一段距离"而不是"瞄准最近的路点"：瞄最近点的话，车会一直
// 在中线两侧来回摆（追一个已经在身后的目标）。前视距离随速度增加 ——
// 开得越快，看得越远，这也是真人开车的做法。
//------------------------------------------------------------------------------
class BotDriver final : public IDriver {
public:
    DriveInput Poll(const RaceGame& race, real dt) override;

    /// 这一帧机器人在干什么（打日志用）
    const char* Action() const noexcept { return m_action; }

    /// 这一帧瞄的是赛车线上的哪个点、方向差多少（调试和 HUD 都要）
    Vec3 Target() const noexcept { return m_target; }
    real SteerError() const noexcept { return m_error; }
    real TargetSpeed() const noexcept { return m_targetSpeed; }

private:
    const char* m_action = "";
    Vec3 m_target = Vec3::Zero();
    real m_error = real(0);
    real m_targetSpeed = real(0);
    /// 连续低速了多久。见 Poll 里的"脱困"注释。
    real m_stuckTimer = real(0);
};

//------------------------------------------------------------------------------
// 控制台相关（和 FPS 那边同一套，见 game/Input.h）
//------------------------------------------------------------------------------
bool IsKeyPressed(int virtualKey);
bool HasInteractiveConsole();
bool EnableUtf8Console();

inline constexpr int kKeyRestart = 'R';
inline constexpr int kKeyEscape = 0x1B;

}  // namespace racing
