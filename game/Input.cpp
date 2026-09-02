//==============================================================================
// game/Input.cpp
//==============================================================================

#include "game/Input.h"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

namespace game {

namespace {

/// 把角度绕回 [-pi, pi]。
///
/// 转向必须用这个：玩家朝向 179 度、目标在 -179 度时，两者只差 2 度，
/// 但直接相减会得到 358 度 —— 机器人会绕着整个圈转回去，看着像在发癫。
real WrapAngle(real angle) noexcept {
    while (angle > kPi) angle -= kTwoPi;
    while (angle < -kPi) angle += kTwoPi;
    return angle;
}

}  // namespace

bool IsKeyPressed(int virtualKey) {
#ifdef _WIN32
    return (GetAsyncKeyState(virtualKey) & 0x8000) != 0;
#else
    (void)virtualKey;
    return false;
#endif
}

//==============================================================================
// 键盘
//==============================================================================

PlayerIntent KeyboardInput::Poll(const Game&, real, real) {
    PlayerIntent intent;

#ifdef _WIN32
    //--------------------------------------------------------------------------
    // 用 GetAsyncKeyState 读**按住**状态，不用 _kbhit/_getch。
    //
    // 事件式输入（_getch）拿到的是"刚刚按下了一次"，用它做移动会变成一顿一顿的：
    // 按住 W 时操作系统按键盘重复率发事件（第一次和第二次之间有半秒延迟），
    // 角色会先走一步、停半秒、然后才连续走。移动必须读**状态**。
    //--------------------------------------------------------------------------
    const auto down = [](int vk) { return (GetAsyncKeyState(vk) & 0x8000) != 0; };

    if (down('W')) intent.forward += real(1);
    if (down('S')) intent.forward -= real(1);
    if (down('D')) intent.strafe += real(1);
    if (down('A')) intent.strafe -= real(1);

    // 转向：方向键或者 Q/E
    if (down(VK_RIGHT) || down('E')) intent.turn += real(1);
    if (down(VK_LEFT) || down('Q')) intent.turn -= real(1);
    if (down(VK_UP)) intent.look += real(1);
    if (down(VK_DOWN)) intent.look -= real(1);

    intent.sprint = down(VK_SHIFT);
    intent.jump = down(VK_SPACE);
    // 开火：鼠标左键或者 Ctrl / F
    intent.fire = down(VK_LBUTTON) || down(VK_CONTROL) || down('F');
    intent.reload = down('R');

    if (down(VK_ESCAPE)) m_quit = true;
#endif

    return intent;
}

//==============================================================================
// 机器人
//==============================================================================

PlayerIntent BotInput::Poll(const Game& game, real time, real dt) {
    PlayerIntent intent;

    if (time >= m_timeLimit) {
        m_finished = true;
        return intent;
    }

    //--------------------------------------------------------------------------
    // 弹匣空了先换弹。放在最前面，因为没子弹时别的都不用想了。
    //--------------------------------------------------------------------------
    if (game.Ammo() == 0 && game.Reserve() > 0) {
        intent.reload = true;
        m_action = "换弹";
    }

    //--------------------------------------------------------------------------
    // 选目标：优先看得见的里面最近的；一个都看不见就挑总体最近的去找
    //--------------------------------------------------------------------------
    const std::vector<Game::EnemyView> enemies = game.AliveEnemyViews();
    if (enemies.empty()) {
        m_action = "清场完毕";
        return intent;
    }

    const Game::EnemyView* target = nullptr;
    for (const Game::EnemyView& enemy : enemies) {
        if (!enemy.visible) continue;
        if (target == nullptr || enemy.distance < target->distance) target = &enemy;
    }
    const bool visible = target != nullptr;
    if (!visible) {
        for (const Game::EnemyView& enemy : enemies) {
            if (target == nullptr || enemy.distance < target->distance) target = &enemy;
        }
    }
    if (target == nullptr) return intent;

    //--------------------------------------------------------------------------
    // 转向目标
    //--------------------------------------------------------------------------
    const Vec3 eye = game.EyePosition();
    // 瞄躯干，不瞄头 —— 见 Game::EnemyView::aim
    const Vec3 toTarget = target->aim - eye;

    const real desiredYaw = Atan2(toTarget.z, toTarget.x);
    const real yawError = WrapAngle(desiredYaw - game.PlayerYaw());

    // 把"还差多少角度"换算成"这一帧该按多久转向键"，并夹在 [-1,1]。
    // 除以 (turnSpeed*dt) 是为了让它在接近目标时自动减速，不会来回过冲。
    const real turnStep = game.Config().turnSpeed * dt;
    intent.turn = Clamp(yawError / Max(turnStep, real(1e-4)), real(-1), real(1));

    // 俯仰：瞄准目标的高度
    const real horizontal = Sqrt(toTarget.x * toTarget.x + toTarget.z * toTarget.z);
    const real desiredPitch = Atan2(toTarget.y, Max(horizontal, real(1e-4)));
    const real pitchError = desiredPitch - game.PlayerPitch();
    intent.look = Clamp(pitchError / Max(turnStep * real(0.6), real(1e-4)), real(-1), real(1));

    //--------------------------------------------------------------------------
    // 开火：看得见 + 已经大致对准。
    //
    // 阈值直接由几何算出来：**目标在这个距离上张开多大的角**。
    //     tolerance = atan(躯干半宽 / 距离)
    // 也就是"偏差已经小到子弹落在身体轮廓里了才扣扳机"。
    //
    // 之前这里是一条拍脑袋的经验公式，敌人还是个 0.76 米宽的胶囊时够用；
    // 换成人形（躯干宽 0.37 米）之后它会在偏出身体一米的时候就开火，
    // 结果是机器人满地打空、自己被打死。目标形状变了，判据就得跟着变。
    //--------------------------------------------------------------------------
    const real aimTolerance =
        Clamp(Atan2(kHumanoidTorsoHalfWidth, Max(target->distance, real(0.5))),
              DegToRad(real(0.6)), DegToRad(real(12)));
    // 已经大致对上了？（用来决定要不要停下侧移，见下面的走位）
    const bool settling = visible && Abs(yawError) < aimTolerance * real(4);

    if (visible && game.Ammo() > 0 && !game.Reloading()) {
        if (Abs(yawError) < aimTolerance && Abs(pitchError) < aimTolerance * real(2)) {
            intent.fire = true;
            m_action = "开火";
        } else {
            m_action = "瞄准";
        }
    } else if (!visible) {
        m_action = "搜索";
    }

    //--------------------------------------------------------------------------
    // 走位：太远就推进，太近就后撤，另外一直做小幅侧移
    //--------------------------------------------------------------------------
    const real preferred = real(9);
    if (target->distance > preferred + real(2)) {
        intent.forward = real(1);
        intent.sprint = !visible;  // 没看见目标时跑起来找
    } else if (target->distance < preferred - real(4)) {
        intent.forward = real(-1);
    }

    //--------------------------------------------------------------------------
    // 侧移：正弦摆动。站着不动是最容易被打中的姿势。
    //
    // **但要开枪的时候先停下。** 瞄准是拿上一帧的位置算的，而侧移速度 5.5 m/s
    // 在 1/30 秒里就把自己挪出去 18 厘米 —— 九米外那是 1.2 度，比人形躯干张开的
    // 角度还大。也就是说：一边横着走一边开枪，误差主要是自己造成的。
    // 敌人还是 0.76 米宽的胶囊时这个误差被吃掉了，换成人形之后就不行了。
    //
    // 停下来打也正是真人的打法（CS 里叫急停）。
    //--------------------------------------------------------------------------
    intent.strafe = settling ? real(0) : Sin(time * real(1.7)) * real(0.7);

    return intent;
}

//==============================================================================
// 控制台
//==============================================================================

bool HasInteractiveConsole() {
#ifdef _WIN32
    // 输出被重定向到文件或管道时 GetConsoleMode 会失败 —— 那种情况下
    // 既没人看画面、也没人按键，应该走机器人模式。
    DWORD mode = 0;
    return GetConsoleMode(GetStdHandle(STD_OUTPUT_HANDLE), &mode) != 0;
#else
    return false;
#endif
}

bool EnableUtf8Console() {
#ifdef _WIN32
    //--------------------------------------------------------------------------
    // 源码里的中文字面量是 UTF-8（GCC 默认就按 UTF-8 处理源码和执行字符集），
    // 但 Windows 控制台默认用的是系统的**传统代码页**（简体中文机器上是 GBK/936）。
    // 于是 UTF-8 的字节被按 GBK 解释，中文全成乱码。
    //
    // SetConsoleOutputCP(CP_UTF8) 就是告诉控制台"我输出的是 UTF-8"。
    // 这是 Windows 10 起的标准做法，不需要改源码编码、也不需要转换字符串。
    //--------------------------------------------------------------------------
    return SetConsoleOutputCP(CP_UTF8) != 0;
#else
    return true;
#endif
}

bool EnableAnsiColors() {
#ifdef _WIN32
    const HANDLE handle = GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD mode = 0;
    if (GetConsoleMode(handle, &mode) == 0) return false;

    // Windows 10 之后的控制台要显式打开虚拟终端处理，ANSI 转义才生效。
    // 打不开就退回无色模式 —— 画面还是能看，只是没有颜色。
#ifndef ENABLE_VIRTUAL_TERMINAL_PROCESSING
#define ENABLE_VIRTUAL_TERMINAL_PROCESSING 0x0004
#endif
    return SetConsoleMode(handle, mode | ENABLE_VIRTUAL_TERMINAL_PROCESSING) != 0;
#else
    return true;
#endif
}

}  // namespace game
