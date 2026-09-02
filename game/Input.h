#pragma once
//==============================================================================
// game/Input.h
//
// 输入。两种来源，输出同一种 `PlayerIntent`：
//
//   - **键盘**：真人玩。用 GetAsyncKeyState 读**按住**状态，而不是 _getch 读
//     按键事件 —— 移动必须是"按住就一直走"，事件式输入会变成一顿一顿的。
//   - **机器人**：无人值守时自己把这一局打完。
//
// 引擎侧完全不知道输入的存在（架构里 physengine 不含渲染也不含输入），
// 这一层是纯游戏代码。
//
//------------------------------------------------------------------------------
// 为什么机器人是"会瞄准的"而不是一段固定脚本
//------------------------------------------------------------------------------
// 第一版写的是时间轴脚本："第 3 到 6 秒，一边前进一边开火"。它能跑，但**打不死
// 任何人** —— 因为它是瞎打的，枪口指向哪儿纯属运气。跑完的结果永远是
// "还剩 5 个敌人，你阵亡了"，除了证明程序没崩之外什么也没验证。
//
// 换成会瞄准的机器人之后，无人值守跑一遍会真正走完整条玩法链路：
// 发现敌人 -> 转向 -> 开火 -> 命中 -> 击杀 -> 换弹 -> 推进 -> 清场。
// 这样它才既是演示、又是一个有意义的冒烟测试。
//==============================================================================

#include "game/Game.h"

namespace game {

//------------------------------------------------------------------------------
// 结算画面上的按键
//
// 主循环在这时已经停了，没有 PlayerIntent 可采，但还得等玩家决定重开还是退出 ——
// 所以直接问一句某个键按着没有。非 Windows 上永远返回 false。
//------------------------------------------------------------------------------
bool IsKeyPressed(int virtualKey);

/// Win32 的虚拟键码。放在这儿，调用方就不必为了两个常量去 include windows.h。
inline constexpr int kKeyRestart = 'R';
inline constexpr int kKeyEscape = 0x1B;

class IInputSource {
public:
    virtual ~IInputSource() = default;

    /// 采一帧输入。
    ///
    /// 参数里带着 `Game` 是为了机器人 —— 它需要知道敌人在哪儿。
    /// 真人输入完全用不到这个参数，但让两者共用一个接口，
    /// 主循环就不需要知道当前是谁在操作。
    virtual PlayerIntent Poll(const Game& game, real time, real dt) = 0;

    virtual bool WantsQuit() const = 0;
};

//------------------------------------------------------------------------------
// 键盘（Windows 控制台）
//------------------------------------------------------------------------------
class KeyboardInput final : public IInputSource {
public:
    PlayerIntent Poll(const Game& game, real time, real dt) override;
    bool WantsQuit() const override { return m_quit; }

private:
    bool m_quit = false;
};

//------------------------------------------------------------------------------
// 机器人玩家
//
// 行为很朴素，但覆盖了完整的玩法链路：
//   1. 有看得见的敌人 -> 转向它，对准了就开火，太远就靠近、太近就后撤
//   2. 没看见 -> 朝最近的敌人走（撞墙靠 collide-and-slide 蹭过去）
//   3. 弹匣空了 -> 换弹
//   4. 一直做小幅侧移，免得站着挨打
//
// 刻意没有寻路：这张图小、房间之间是直通的门洞，直着走 + 贴墙滑行就够了。
// 真要绕柱子包抄得上导航网格，那是另一个话题。
//------------------------------------------------------------------------------
class BotInput final : public IInputSource {
public:
    explicit BotInput(real timeLimit) : m_timeLimit(timeLimit) {}

    PlayerIntent Poll(const Game& game, real time, real dt) override;
    bool WantsQuit() const override { return m_finished; }

    /// 这一帧机器人在干什么（打日志用）。
    const char* CurrentAction() const noexcept { return m_action; }

private:
    real m_timeLimit;
    bool m_finished = false;
    const char* m_action = "搜索";
};

/// 当前进程是不是接在一个真正的控制台上（不是被重定向到文件/管道）。
/// 决定了默认走键盘还是走机器人。
bool HasInteractiveConsole();

/// 把控制台输出代码页切到 UTF-8。不调的话源码里的中文会按系统传统代码页
/// （简体中文机器上是 GBK）解释，全是乱码。返回是否成功。
bool EnableUtf8Console();

/// 打开 Windows 控制台的 ANSI 转义支持。返回是否成功。
bool EnableAnsiColors();

}  // namespace game
