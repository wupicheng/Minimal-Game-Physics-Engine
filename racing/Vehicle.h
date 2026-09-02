#pragma once
//==============================================================================
// racing/Vehicle.h
//
// 一台车：**一个刚体 + 四条射线**。
//
//------------------------------------------------------------------------------
// 为什么轮子是射线，而不是四个刚体加铰链
//------------------------------------------------------------------------------
// "物理上更真实"的做法是：四个圆柱刚体，各用一个铰链约束连到车身上，再加弹簧。
// 听起来对，实际上是所有人都放弃过的一条路：
//
//   1. **约束数量爆炸**。四个轮子 = 四个铰链 + 四个弹簧 + 四个转向关节，
//      迭代求解器要在一帧里同时满足它们，参数稍微不对就抖、就软、就散架。
//   2. **接触点太少**。圆柱和地面是线接触，离散求解器每帧只给出一两个接触点，
//      过坎时轮子会卡进地面或者被弹飞。
//   3. **手感调不动**。抓地力、转向不足/过度、漂移，全都只能靠间接地凑约束参数，
//      而不是直接写"这个轮子现在有多少侧向力"。
//
// 所以从 Unity 的 WheelCollider 到 Bullet 的 btRaycastVehicle，主流做法都是
// **射线车**：车身是唯一的刚体，每个轮子每个子步往下打一条射线，拿到接触点
// 之后，把悬挂力和轮胎力**直接加在车身上**。这不是简化版，是行业默认版。
//
// 这么做每一项力都有名字、都能单独调：
//     悬挂  = 弹簧 + 阻尼       -> 车的软硬、侧倾、过坎
//     侧向  = 消除侧滑的冲量     -> 抓地、转向、漂移
//     纵向  = 驱动 / 刹车冲量    -> 加速、刹车、烧胎
//   三者被同一个**摩擦圆**卡住  -> "油门给太猛就抓不住地"是自动出现的，不用写
//
// 引擎这边只需要三件事，而它三件都有：
//     world.Raycast()             找地面
//     body.ApplyForceAtPoint()    悬挂力（持续作用 -> 用力）
//     body.ApplyImpulseAtPoint()  轮胎力（瞬时改速度 -> 用冲量）
//
//------------------------------------------------------------------------------
// 必须按**物理子步**驱动，不能按渲染帧
//------------------------------------------------------------------------------
// 力要被积分，而积分发生在固定步长的子步里。按渲染帧施力的话，一帧跑两个子步时
// 第二个子步没有悬挂力，车会周期性下沉 —— 帧率越低越明显。所以 `RaceGame`
// 自己跑固定步长循环：施力 -> Step(h) -> 施力 -> Step(h)。
//==============================================================================

#include <array>

#include "racing/RacingTypes.h"

namespace racing {

//------------------------------------------------------------------------------
// 驾驶输入（键盘或者机器人填）
//
// 刻意是"想干什么"而不是"按了哪个键"：换手柄、换回放、换 AI 托管都不用改车。
//------------------------------------------------------------------------------
struct DriveInput {
    real throttle = real(0);  ///< [0,1]
    real brake = real(0);     ///< [0,1]，停住之后继续踩就是倒挡
    real steer = real(0);     ///< [-1,1]，正数往右
    bool handbrake = false;
};

//------------------------------------------------------------------------------
// 一个轮子的静态配置
//------------------------------------------------------------------------------
struct WheelConfig {
    Vec3 attach;             ///< 悬挂上端点，车身局部坐标
    bool steered = false;    ///< 跟着方向盘转
    bool driven = true;      ///< 有动力
    bool handbraked = false; ///< 手刹锁它
};

//------------------------------------------------------------------------------
// 车的全部参数
//
// 全是**手感**参数。真实车辆数据（功率曲线、轮胎 Pacejka 公式）在这个尺度上
// 帮不上忙 —— 玩家判断"这车好不好开"靠的是响应，不是数值的出处。
//------------------------------------------------------------------------------
struct VehicleConfig {
    //-- 车身 -------------------------------------------------------------------
    Vec3 halfExtents = Vec3(real(1.00), real(0.35), real(2.0));
    real mass = real(1150);

    //-- 悬挂 -------------------------------------------------------------------
    /// 静止时悬挂多长（上端点到轮心）。它决定车离地多高。
    real suspensionRest = real(0.45);
    /// 最多还能压多少。压到底就是车身直接砸地面了。
    real suspensionTravel = real(0.28);
    /// 弹簧刚度（N/m）。四轮平分 1150 kg = 每轮约 2820 N，取 42000 N/m 时
    /// 静止压缩约 6.7 厘米 —— 大约行程的四分之一，剩下的留给过坎和侧倾。
    real suspensionStiffness = real(42000);
    /// 阻尼（N·s/m）。太小车一直弹，太大过坎像块砖。
    /// 经验起点是临界阻尼的三四成：2*sqrt(k*m/4) ≈ 6900，取 4200。
    real suspensionDamping = real(4200);
    /// 防倾杆。**没有它，重心高于半个轮距的车过弯必翻。**
    /// 它把内外侧悬挂的压缩差转成一对反向的力，直接压住侧倾。
    real antiRollStiffness = real(16000);

    /// 轮子半径。它同时决定车离地多高（悬挂射线的长度里含它）和轮子画多大 ——
    /// 0.31 米（直径 0.62）配 1 米出头的车身，比例接近真车。
    real wheelRadius = real(0.31);

    //-- 轮胎 -------------------------------------------------------------------
    //
    // 侧向摩擦系数。大于 1 物理上完全合法（见 Material.h）：赛车轮胎就是这样。
    //
    // **前轮的抓地力必须比后轮低。** 这不是调参技巧，是车辆动力学里最基本的
    // 一条设定，几乎所有量产车和所有赛车游戏都这么做：
    //
    //   前 < 后  -> 极限时**前轮先滑**，车头推向弯外（转向不足）。松油门、
    //               少打点方向就能救回来，是可控的。
    //   前 = 后  -> 谁先滑全看载荷的随机波动。前一版两个值一样，结果是车在
    //               弯里毫无预兆地**掉头 360 度** —— 打印出来的车头朝向
    //               (0.74,-0.67) -> (-0.97,-0.24) -> (+0.51,+0.86)，转了一整圈。
    //   前 > 后  -> 后轮先滑（转向过度），漂移车才这么设，新手完全开不了。
    //
    // 手刹漂移是**临时**把后轮的抓地力打折（handbrakeGripScale），
    // 也就是临时切到"前 > 后"，松开就回来 —— 想漂的时候才漂。
    real gripLateralFront = real(1.30);
    real gripLateralRear = real(1.75);
    real gripLongitudinal = real(1.7);
    /// 拉手刹时后轮侧向抓地力打几折 —— 漂移就是这一个数。
    real handbrakeGripScale = real(0.28);

    //-- 动力 -------------------------------------------------------------------
    real driveForce = real(11000);   ///< 全部驱动轮合计（N）
    real brakeForce = real(16000);
    real reverseForce = real(5000);
    /// 极速（m/s）。接近它时驱动力线性收到零 —— 比硬夹速度自然得多。
    real maxSpeed = real(58);

    //-- 转向 -------------------------------------------------------------------
    real maxSteerAngle = DegToRad(real(34));
    /// 方向盘打满要多久（秒）。瞬间到位的话高速下一碰方向就甩出去。
    real steerLockTime = real(0.18);
    /// 高速转向衰减：到这个速度时最大转角只剩 highSpeedSteerScale。
    real steerFalloffSpeed = real(32);
    real highSpeedSteerScale = real(0.35);

    //-- 空气动力与阻力 ---------------------------------------------------------
    real dragCoefficient = real(2.6);      ///< F = -k * v * |v|
    real downforceCoefficient = real(7.0); ///< F = -k * v^2，压向地面
    real rollingResistance = real(320);    ///< 松油门自然减速

    /// 腾空时的姿态控制。真实里飞起来是控不了的，但玩家会本能地想在空中摆正
    /// 车头 —— 不给的话每次跳台都以翻车收场。
    real airPitchControl = real(6000);
};

//------------------------------------------------------------------------------
// 一个轮子每个子步算出来的东西（渲染和 HUD 要读）
//------------------------------------------------------------------------------
struct WheelState {
    Vec3 center = Vec3::Zero();       ///< 轮心世界坐标
    Vec3 contactPoint = Vec3::Zero();
    Vec3 contactNormal = Vec3(0, real(1), 0);
    real suspensionLength = real(0);
    real compression = real(0);       ///< 压缩量（米），防倾杆和渲染都要
    real normalForce = real(0);       ///< 这一轮压了多少 N 在地上
    real slip = real(0);              ///< 侧滑程度 [0,1]，打滑提示用
    real spinAngle = real(0);         ///< 轮子转过的角度（纯视觉）
    real steerAngle = real(0);
    bool grounded = false;
};

//------------------------------------------------------------------------------
// 车
//------------------------------------------------------------------------------
class Vehicle {
public:
    static constexpr int kWheelCount = 4;

    Vehicle() = default;

    /// 在世界里造出车身刚体。轮子不是刚体（见文件头），但会额外造四个
    /// **纯装饰**的运动学刚体，好让光线投射渲染器看得见它们。
    void Spawn(PhysicsWorld& world, const VehicleConfig& config, const Vec3& position,
               real yaw);

    /// 推进一个**物理子步**：算悬挂、算轮胎、施力。之后由调用方 world.Step(h)。
    void Update(PhysicsWorld& world, const DriveInput& input, real h);

    /// 把车摆回某个位置（掉出赛道、翻车之后用）。
    void Respawn(PhysicsWorld& world, const Vec3& position, real yaw);

    //-- 查询 -------------------------------------------------------------------
    BodyHandle Body() const noexcept { return m_body; }
    BodyHandle WheelBody(int i) const noexcept { return m_wheelBodies[i]; }
    /// 车壳（纯装饰，物理上车只是一个盒子）
    BodyHandle ShellBody() const noexcept { return m_shellBody; }
    const WheelState& Wheel(int i) const noexcept { return m_wheels[i]; }
    const VehicleConfig& Config() const noexcept { return m_config; }

    Vec3 Position() const noexcept { return m_position; }
    Quat Rotation() const noexcept { return m_rotation; }
    Vec3 Velocity() const noexcept { return m_velocity; }
    Vec3 Forward() const noexcept { return m_rotation.Rotate(Vec3(0, 0, real(1))); }
    Vec3 Up() const noexcept { return m_rotation.Rotate(Vec3(0, real(1), 0)); }
    Vec3 Right() const noexcept { return m_rotation.Rotate(Vec3(real(1), 0, 0)); }

    /// 沿车头方向的速度（m/s）。倒车时是负的。
    real ForwardSpeed() const noexcept { return Dot(m_velocity, Forward()); }
    real Speed() const noexcept { return m_velocity.Length(); }
    real SpeedKmh() const noexcept { return Speed() * real(3.6); }
    real SteerAngle() const noexcept { return m_steerAngle; }

    int GroundedWheels() const noexcept;
    /// 四个轮子里侧滑最厉害的那个 —— HUD 上的"打滑"提示用它。
    real MaxSlip() const noexcept;
    /// 车顶朝下了。翻车判定用。
    bool IsUpsideDown() const noexcept { return Up().y < real(-0.1); }

private:
    void UpdateSuspension(PhysicsWorld& world, RigidBody& body, real h);
    void UpdateAntiRollBars(RigidBody& body);
    void UpdateTireForces(RigidBody& body, const DriveInput& input, real h);
    void UpdateAerodynamics(RigidBody& body, const DriveInput& input, real h);
    /// 把车壳和四个轮子的位姿抄给那些纯装饰刚体
    void SyncVisualBodies(PhysicsWorld& world, real h);

    VehicleConfig m_config;
    std::array<WheelConfig, kWheelCount> m_wheelConfig{};
    std::array<WheelState, kWheelCount> m_wheels{};

    BodyHandle m_body = BodyHandle::Null();
    std::array<BodyHandle, kWheelCount> m_wheelBodies{};
    BodyHandle m_shellBody = BodyHandle::Null();

    // 每个子步开头从刚体抄下来的位姿，省得到处 GetBody()
    Vec3 m_position = Vec3::Zero();
    Quat m_rotation = Quat::Identity();
    Vec3 m_velocity = Vec3::Zero();

    real m_steerAngle = real(0);
};

}  // namespace racing
