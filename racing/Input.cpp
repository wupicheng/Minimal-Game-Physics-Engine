//==============================================================================
// racing/Input.cpp
//==============================================================================

#include "racing/Input.h"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

namespace racing {

namespace {

/// 把角度绕回 [-pi, pi]。转向必须用它：车头朝 179 度、目标在 -179 度时两者
/// 只差 2 度，直接相减会得到 358 度，机器人会往反方向打死方向盘。
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

bool HasInteractiveConsole() {
#ifdef _WIN32
    // 输出被重定向到文件或管道时 GetConsoleMode 会失败 —— 那种情况下既没人
    // 看画面也没人按键，应该走机器人模式
    DWORD mode = 0;
    return GetConsoleMode(GetStdHandle(STD_OUTPUT_HANDLE), &mode) != 0;
#else
    return false;
#endif
}

bool EnableUtf8Console() {
#ifdef _WIN32
    return SetConsoleOutputCP(CP_UTF8) != 0;
#else
    return true;
#endif
}

//==============================================================================
// 键盘
//==============================================================================

DriveInput KeyboardDriver::Poll(const RaceGame&, real) {
    DriveInput input;
    m_respawn = false;

#ifdef _WIN32
    const auto down = [](int vk) { return (GetAsyncKeyState(vk) & 0x8000) != 0; };

    // 油门/刹车：W/S 或者上/下方向键
    if (down('W') || down(VK_UP)) input.throttle = real(1);
    if (down('S') || down(VK_DOWN)) input.brake = real(1);
    // 方向：A/D 或者左/右
    if (down('D') || down(VK_RIGHT)) input.steer += real(1);
    if (down('A') || down(VK_LEFT)) input.steer -= real(1);

    input.handbrake = down(VK_SPACE);
    m_respawn = down(kKeyRestart);
    m_quit = down(kKeyEscape);
#endif
    return input;
}

//==============================================================================
// 机器人车手
//==============================================================================

DriveInput BotDriver::Poll(const RaceGame& race, real dt) {
    DriveInput input;

    const Vehicle& car = race.Car();
    const Vec3 position = car.Position();
    const real speed = car.Speed();

    //--------------------------------------------------------------------------
    // 纯追踪（pure pursuit）：在赛车线上找到离自己最近的那个点，然后沿着线
    // 往前数 lookAhead 米，瞄准那里。
    //
    // 两个都不能省的细节：
    //
    //   - **必须沿着赛车线找目标，不能直接瞄下一个检查点。** 椭圆赛道上，
    //     检查点之间的直线是从内圈护墙里穿过去的 —— 第一版机器人就是这么
    //     一头撞进墙里，然后贴着墙磨完全程的。
    //   - **前视距离随速度变**（约 0.9 秒的行程）。固定前视的车在直道上会
    //     画蛇形：目标太近，方向盘一直在修正一个马上就到的点。
    //--------------------------------------------------------------------------
    const std::vector<Vec3>& line = race.RacingLine();
    if (line.empty()) return input;

    std::size_t nearest = 0;
    real nearestDistance = real(1e9);
    for (std::size_t i = 0; i < line.size(); ++i) {
        const Vec3 delta(line[i].x - position.x, real(0), line[i].z - position.z);
        const real d = delta.LengthSq();
        if (d < nearestDistance) {
            nearestDistance = d;
            nearest = i;
        }
    }

    const real lookAhead = Clamp(speed * real(0.65), real(7), real(18));
    real walked = real(0);
    std::size_t index = nearest;
    while (walked < lookAhead) {
        const std::size_t next = (index + 1) % line.size();
        walked += (line[next] - line[index]).Length();
        index = next;
        if (index == nearest) break;  // 绕了一整圈，赛车线比前视距离还短
    }
    const Vec3 target = line[index];

    Vec3 toTarget(target.x - position.x, real(0), target.z - position.z);
    if (toTarget.LengthSq() < real(0.01)) return input;

    //-- 转向：车头和目标方向的夹角，直接当方向盘 -------------------------------
    const Vec3 forward = car.Forward();
    const real desiredYaw = Atan2(toTarget.x, toTarget.z);
    const real currentYaw = Atan2(forward.x, forward.z);
    const real error = WrapAngle(desiredYaw - currentYaw);

    // 除以最大转角：偏差比一个满舵还大就打死，否则按比例。
    // 再叠一点点速度阻尼，免得高速直道上左右摆
    input.steer = Clamp(error / car.Config().maxSteerAngle, real(-1), real(1));
    m_target = target;
    m_error = error;

    //--------------------------------------------------------------------------
    // 该开多快：扫一遍前方每一个点，取最严的那条限制。
    //
    // 对前方距离 s 处、曲率 κ 的一个点：
    //     那里能过的速度   v_κ  = sqrt(a_横向 / κ)          圆周运动
    //     那么这里最多能开 v    = sqrt(v_κ² + 2·a_刹车·s)   匀减速刚好减到 v_κ
    // 对所有点取最小值，就是"此刻的合法速度"。
    //
    // 走过两个都不对的版本，两个都很有迷惑性：
    //
    //   1. 用"到前方某个点要转多少度"当弯道急缓 —— 弧线的弦永远比弧本身直，
    //      60 米外绕过弯的点方向上可能只偏 20 度，于是 72 km/h 冲进 55 km/h 的弯。
    //   2. 用整个前视窗口的**平均**曲率 —— 半段直道 + 半段弯，平均下来只有弯道
    //      的一半，进弯速度就高了 √2 倍。均值在这里天然是错的，要的是**最坏值**。
    //--------------------------------------------------------------------------
    // 用前后轮里**低**的那个：先滑的是它，弯道速度就由它定
    const real lateralLimit = Min(car.Config().gripLateralFront,
                                  car.Config().gripLateralRear) *
                              real(9.81) * real(0.65);
    constexpr real kBrakeDecel = real(6.5);   ///< 留有余量的可用减速度
    constexpr real kScanDistance = real(70);

    real limit = real(44);
    real scanned = real(0);
    std::size_t scan = nearest;
    while (scanned < kScanDistance) {
        const std::size_t cur = (scan + 1) % line.size();
        const std::size_t next = (cur + 1) % line.size();
        const Vec3 incoming = line[cur] - line[scan];
        const Vec3 outgoing = line[next] - line[cur];
        const real segment = incoming.Length();
        if (segment > real(0.01)) {
            const real turn = Abs(WrapAngle(Atan2(outgoing.x, outgoing.z) -
                                            Atan2(incoming.x, incoming.z)));
            const real curvature = turn / segment;
            if (curvature > real(1e-4)) {
                const real corner = Sqrt(lateralLimit / curvature);
                limit = Min(limit, Sqrt(corner * corner +
                                        real(2) * kBrakeDecel * scanned));
            }
        }
        scanned += segment;
        scan = cur;
        if (scan == nearest) break;
    }

    const real targetSpeed = Clamp(limit, real(12), real(44));
    m_targetSpeed = targetSpeed;

    if (speed < targetSpeed) {
        input.throttle = real(1);
        m_action = "加速";
    } else if (speed > targetSpeed + real(4)) {
        input.brake = Clamp((speed - targetSpeed) / real(10), real(0), real(1));
        m_action = "刹车";
    } else {
        input.throttle = real(0.35);
        m_action = "维持";
    }

    //--------------------------------------------------------------------------
    // 卡住了（比如顶着墙）：倒一下车再重来。
    //
    // 判据必须是"**持续**低速"而不是"这一帧低速"：发车瞬间车速本来就是零，
    // 用瞬时速度判的话机器人一发车就开始倒车，永远起步不了。这个 bug 的表现是
    // 车以 4 km/h 一路蹭到弯里，非常有迷惑性 —— 看着像动力不足，其实是逻辑。
    //--------------------------------------------------------------------------
    if (race.State() == RaceState::Racing && speed < real(1.5)) {
        m_stuckTimer += dt;
    } else {
        m_stuckTimer = real(0);
    }
    if (m_stuckTimer > real(1.5)) {
        input.throttle = real(0);
        input.brake = real(1);
        input.steer = -input.steer;
        m_action = "脱困";
        // 倒够一秒就再试一次
        if (m_stuckTimer > real(2.5)) m_stuckTimer = real(0);
    }

    return input;
}

}  // namespace racing
