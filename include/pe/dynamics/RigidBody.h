#pragma once
//==============================================================================
// pe/dynamics/RigidBody.h
//
// 刚体状态块。**纯 POD**：无虚函数、无构造逻辑、无指针、无所有权。
//
//------------------------------------------------------------------------------
// 为什么坚持 POD
//------------------------------------------------------------------------------
//   1. 整块 memcpy 安全 —— 存档、网络快照、回滚都只是一次内存拷贝
//   2. 能紧密排在 std::vector 里，求解器遍历时缓存友好
//   3. 可以直接拆成 ECS 组件（Transform / Velocity / MassData 三块），
//      拆的时候不需要动任何逻辑代码
//
// 代价是没有"不变量由构造函数保证"这回事：比如 invInertiaWorld 必须由调用方
// 在旋转变化之后手动刷新。这个代价是刻意接受的，刷新点集中在 Integrator 里，
// PhysicsWorld 的 Step() 是唯一的调用者。
//
//------------------------------------------------------------------------------
// 三种刚体类型的差别，全部体现在 invMass / invInertiaWorld 上
//------------------------------------------------------------------------------
//   Static     invMass = 0，不积分，速度恒为 0
//   Kinematic  invMass = 0，不积分速度但积分位置（由游戏逻辑直接设速度）
//   Dynamic    invMass > 0，完整模拟
//
// 注意 Static 和 Kinematic 的 invMass 都是 0 —— 这不是偷懒，而是"无穷大质量"的
// 编码方式：冲量公式 `v += impulse * invMass` 在 invMass = 0 时自动变成
// "速度不变"，于是求解器里一个 `if (isStatic)` 都不需要写。
// 两者的区别只在于**积不积分位置**，那是 Integrator 的事，不是求解器的事。
//==============================================================================

#include <cstdint>

#include "pe/core/Types.h"
#include "pe/dynamics/MassProperties.h"
#include "pe/math/Mat3.h"
#include "pe/math/Quat.h"
#include "pe/math/Transform.h"
#include "pe/math/Vec3.h"

namespace pe {

//------------------------------------------------------------------------------
// 休眠参数
//
// 休眠是达成"50-100 个动态体跑满 60fps"这个目标的主要手段：一局游戏里绝大多数
// 箱子在绝大多数时间是不动的，让它们彻底退出求解器比优化求解器本身有效得多。
//------------------------------------------------------------------------------
struct SleepConfig {
    /// 线速度阈值（米/秒）。低于它才开始累计休眠计时。
    real linearThreshold = real(0.05);

    /// 角速度阈值（弧度/秒）。约 5.7 度/秒。
    real angularThreshold = real(0.1);

    /// 连续低于阈值多久才真的睡着（秒）。
    ///
    /// 不能取 0：物体在弹跳的最高点速度瞬时为零，立刻休眠会让它卡在半空。
    /// 0.5 秒足以跨过任何一次弹跳的顶点。
    real timeToSleep = real(0.5);
};

//------------------------------------------------------------------------------
// 刚体
//------------------------------------------------------------------------------
struct RigidBody {
    //-- 位姿（世界空间）--------------------------------------------------------
    Vec3 position;  ///< 质心的位置。注意是**质心**不是"物体原点"——
                    ///< 当前两者重合（见 MassProperties.h），复合形状时会分开。
    Quat rotation;

    //-- 速度 -------------------------------------------------------------------
    Vec3 linearVelocity;
    Vec3 angularVelocity;  ///< 世界空间，单位 rad/s，方向 = 转轴，长度 = 角速率

    //-- 累积外力（每帧 Step 末尾清零）------------------------------------------
    Vec3 force;
    Vec3 torque;

    //-- 质量属性 ---------------------------------------------------------------
    real invMass;           ///< 0 表示无穷大质量（静态/运动学）
    Mat3 invInertiaLocal;   ///< 局部空间惯量张量的逆（形状决定，不随姿态变）
    Mat3 invInertiaWorld;   ///< 每帧由 R * invInertiaLocal * R^T 刷新的缓存

    //-- 阻尼 -------------------------------------------------------------------
    /// 每秒衰减比例。0 = 不衰减。
    ///
    /// 它不是空气阻力的物理模型，而是**数值稳定手段**：纯粹的无阻尼模拟里，
    /// 求解器的残余误差会慢慢累积成永不停歇的微小抖动，物体永远睡不着。
    /// 一点点阻尼能把这些噪声吃掉。
    real linearDamping;
    real angularDamping;

    //-- 休眠 -------------------------------------------------------------------
    real sleepTimer;  ///< 已经连续"低速"多久了（秒）
    bool isSleeping;

    //-- 分类与过滤 -------------------------------------------------------------
    BodyType type;
    LayerMask layer;
    LayerMask layerMask;

    /// 是否受重力影响。气球、悬浮平台、飞行道具需要关掉它。
    bool useGravity;

    RigidBody() = default;

    //--------------------------------------------------------------------------
    // 构造
    //--------------------------------------------------------------------------

    /// 一个各字段都合法的动态刚体：质量 1、单位惯量、静止在原点。
    /// 之所以不用默认构造函数做这件事，是为了让 RigidBody 保持平凡类型。
    static RigidBody Make(BodyType bodyType = BodyType::Dynamic) noexcept;

    //--------------------------------------------------------------------------
    // 查询
    //--------------------------------------------------------------------------

    bool IsDynamic() const noexcept { return type == BodyType::Dynamic; }

    /// 能不能被冲量推动。静态和运动学都不能。
    bool IsMovableByImpulse() const noexcept { return invMass > real(0); }

    /// 参与模拟（会被积分、会被求解器处理）。
    bool IsActive() const noexcept { return !isSleeping && type != BodyType::Static; }

    real Mass() const noexcept { return invMass > real(0) ? real(1) / invMass : real(0); }

    Transform GetTransform() const noexcept { return Transform(position, rotation); }

    /// 刚体上某一点（世界坐标）的速度。
    ///
    ///     v_point = v + w x r        r 是从质心指向该点的向量
    ///
    /// 这是整个碰撞响应的出发点：求解器要消除的正是两个刚体在**接触点处**的
    /// 相对接近速度，而不是它们质心的速度差。
    Vec3 VelocityAtPoint(const Vec3& worldPoint) const noexcept {
        return linearVelocity + Cross(angularVelocity, worldPoint - position);
    }

    //--------------------------------------------------------------------------
    // 施力 / 施加冲量
    //
    // **力**在下一次速度积分时才生效，会被 dt 缩放，适合持续作用（推力、风）。
    // **冲量**立刻改变速度，与 dt 无关，适合瞬时事件（子弹击中、爆炸、跳跃）。
    // 分不清就问："松手之后它还该继续作用吗？"是 -> 力，否 -> 冲量。
    //--------------------------------------------------------------------------

    void ApplyForce(const Vec3& f) noexcept { force += f; }

    /// 在世界空间某点施力。偏离质心的力会同时产生力矩。
    void ApplyForceAtPoint(const Vec3& f, const Vec3& worldPoint) noexcept {
        force += f;
        torque += Cross(worldPoint - position, f);
    }

    void ApplyTorque(const Vec3& t) noexcept { torque += t; }

    void ApplyImpulse(const Vec3& impulse) noexcept {
        linearVelocity += impulse * invMass;
    }

    /// 在世界空间某点施加冲量。这是求解器唯一用到的施力方式。
    void ApplyImpulseAtPoint(const Vec3& impulse, const Vec3& worldPoint) noexcept {
        linearVelocity += impulse * invMass;
        angularVelocity += invInertiaWorld * Cross(worldPoint - position, impulse);
    }

    void ApplyAngularImpulse(const Vec3& angularImpulse) noexcept {
        angularVelocity += invInertiaWorld * angularImpulse;
    }

    //--------------------------------------------------------------------------
    // 休眠
    //--------------------------------------------------------------------------

    /// 唤醒并把计时器清零。任何"发生了新情况"的地方都该调它：
    /// 新碰撞、施力、被传送、属性被改。漏调的症状是"箱子被打了却不动"。
    void WakeUp() noexcept {
        isSleeping = false;
        sleepTimer = real(0);
    }

    /// 强制休眠：速度清零并标记。清零很重要 —— 留着残余速度的话，
    /// 醒来的瞬间物体会突然窜一下。
    void ForceSleep() noexcept {
        isSleeping = true;
        sleepTimer = real(0);
        linearVelocity = Vec3::Zero();
        angularVelocity = Vec3::Zero();
    }
};

//------------------------------------------------------------------------------
// 创建描述
//
// 用"描述结构 + 工厂函数"而不是一堆构造函数重载：字段一多，重载会爆炸，
// 而且调用点全是没有名字的位置参数。这个结构体让 M8 的
// `PhysicsWorld::CreateBody(desc)` 读起来是自解释的。
//------------------------------------------------------------------------------
struct BodyDesc {
    Vec3 position = Vec3::Zero();
    Quat rotation = Quat::Identity();
    Vec3 linearVelocity = Vec3::Zero();
    Vec3 angularVelocity = Vec3::Zero();

    BodyType type = BodyType::Dynamic;

    /// 质量。<= 0 表示"由形状和密度自动算"（交给 M8 的 World 处理）。
    real mass = real(-1);

    real linearDamping = real(0.02);
    real angularDamping = real(0.05);

    LayerMask layer = layers::kDefault;
    LayerMask layerMask = kLayerAll;

    bool useGravity = true;
    bool startAsleep = false;
};

/// 按描述与质量属性造一个刚体。
/// props.mass <= 0（或 type 不是 Dynamic）时刚体会得到 invMass = 0。
RigidBody MakeRigidBody(const BodyDesc& desc, const MassProperties& props) noexcept;

}  // namespace pe

