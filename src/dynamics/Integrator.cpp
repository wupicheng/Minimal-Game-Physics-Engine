//==============================================================================
// src/dynamics/Integrator.cpp
//
// 半隐式欧拉积分器。原理见 Integrator.h，这里写数值上的细节。
//==============================================================================

#include "pe/dynamics/Integrator.h"

#include <cmath>

namespace pe {

//==============================================================================
// 刚体构造
//==============================================================================

RigidBody RigidBody::Make(BodyType bodyType) noexcept {
    RigidBody b;
    b.position = Vec3::Zero();
    b.rotation = Quat::Identity();
    b.linearVelocity = Vec3::Zero();
    b.angularVelocity = Vec3::Zero();
    b.force = Vec3::Zero();
    b.torque = Vec3::Zero();

    // 静态/运动学的 invMass 为 0 —— 无穷大质量的编码方式，见 RigidBody.h
    const bool dynamic = (bodyType == BodyType::Dynamic);
    b.invMass = dynamic ? real(1) : real(0);
    b.invInertiaLocal =
        dynamic ? Mat3::Identity() : Mat3::Diagonal(Vec3::Zero());
    b.invInertiaWorld = b.invInertiaLocal;

    b.linearDamping = real(0.02);
    b.angularDamping = real(0.05);
    b.sleepTimer = real(0);
    b.isSleeping = false;
    b.type = bodyType;
    b.layer = layers::kDefault;
    b.layerMask = kLayerAll;
    b.useGravity = dynamic;
    return b;
}

RigidBody MakeRigidBody(const BodyDesc& desc, const MassProperties& props) noexcept {
    RigidBody b = RigidBody::Make(desc.type);

    b.position = desc.position;
    b.rotation = desc.rotation.Normalized();
    b.linearVelocity = desc.linearVelocity;
    b.angularVelocity = desc.angularVelocity;
    b.linearDamping = desc.linearDamping;
    b.angularDamping = desc.angularDamping;
    b.layer = desc.layer;
    b.layerMask = desc.layerMask;
    b.useGravity = desc.useGravity && desc.type == BodyType::Dynamic;
    b.isSleeping = desc.startAsleep;

    if (desc.type == BodyType::Dynamic) {
        // desc.mass > 0 时以它为准，否则用形状算出来的质量
        const MassProperties effective =
            (desc.mass > real(0)) ? props.WithMass(desc.mass) : props;

        b.invMass = effective.InverseMass();
        b.invInertiaLocal = (effective.mass > real(0))
                                ? effective.InverseInertia()
                                : Mat3::Diagonal(Vec3::Zero());
    } else {
        b.invMass = real(0);
        b.invInertiaLocal = Mat3::Diagonal(Vec3::Zero());
    }

    UpdateWorldInertia(b);
    return b;
}

//==============================================================================
// 世界空间惯量
//==============================================================================

void UpdateWorldInertia(RigidBody& body) noexcept {
    // I_world^-1 = R * I_local^-1 * R^T
    //
    // 注意这里对**逆矩阵**做同样的变换是合法的：
    //     (R I R^T)^-1 = R^-T I^-1 R^-1 = R I^-1 R^T   （R 正交，R^-1 = R^T）
    // 所以不需要先转换再求逆，直接变换逆矩阵即可 —— 每帧省掉一次 3x3 求逆。
    const Mat3 r = body.rotation.ToMat3();
    body.invInertiaWorld = r * body.invInertiaLocal * r.Transposed();
}

//==============================================================================
// 阶段 A：速度
//==============================================================================

void IntegrateVelocity(RigidBody& body, const Vec3& gravity, real dt) noexcept {
    // 运动学体的速度由游戏逻辑直接设定，引擎不碰。
    // 静态体和睡着的物体也跳过。
    if (body.type != BodyType::Dynamic || body.isSleeping || dt <= real(0)) return;

    //--------------------------------------------------------------------------
    // 线速度
    //
    // 重力是**加速度**不是力，所以直接加，不乘 invMass —— 这一点常被写错成
    // `force += gravity * mass`，虽然数学等价，但那样在 invMass = 0 时会失效，
    // 而且多一次乘除。
    //--------------------------------------------------------------------------
    if (body.useGravity) {
        body.linearVelocity += gravity * dt;
    }
    body.linearVelocity += body.force * (body.invMass * dt);

    //--------------------------------------------------------------------------
    // 角速度：欧拉方程
    //
    //     I_world * dw/dt = tau - w x (I_world * w)
    //
    // 陀螺项 `w x (I*w)` 描述的是"惯量张量随姿态旋转"带来的表观力矩。
    // 少了它，自由旋转刚体的角动量不守恒。
    //
    // 但它在显式积分下是不稳定的：角速度越大，这一项越大（二次增长），
    // 高速自旋时一步就能把角速度顶到发散。工程上的通行做法是给它限幅 ——
    // 限制单步内陀螺项造成的角速度变化不超过当前角速度本身。
    // 这在物理上是"错的"，但发散的模拟比略有误差的模拟错得多。
    //--------------------------------------------------------------------------
    const Vec3 angularAccel = body.invInertiaWorld * body.torque;

    // L = I_world * w。手上只有 I_world^-1，所以要求一次逆。
    // 一次 3x3 求逆约 30 次浮点运算，100 个动态体一帧也就三千次，可以忽略；
    // 真成了热点再在 RigidBody 里加一个 inertiaWorld 缓存即可（代价是多 9 个
    // float，以及多一处需要保持同步的数据）。
    // 惯量退化（零惯量的动态体）时 Inverse() 返回零矩阵，陀螺项自然变成 0 —— 安全。
    const Vec3 angularMomentum = body.invInertiaWorld.Inverse() * body.angularVelocity;
    const Vec3 gyroscopic = Cross(body.angularVelocity, angularMomentum);
    Vec3 gyroDelta = body.invInertiaWorld * (-gyroscopic) * dt;

    // 限幅：陀螺项造成的变化不能超过当前角速度的量级
    const real omegaLen = body.angularVelocity.Length();
    const real gyroLen = gyroDelta.Length();
    if (gyroLen > omegaLen && gyroLen > kEpsilon) {
        gyroDelta *= omegaLen / gyroLen;
    }

    body.angularVelocity += angularAccel * dt + gyroDelta;

    //--------------------------------------------------------------------------
    // 阻尼
    //
    // 用 pow(1 - damping, dt) 而不是 (1 - damping * dt)：
    // 前者是指数衰减的精确解，与步长无关 —— 同样的 damping 在 30fps 和 144fps
    // 下衰减速度一致。后者在大步长时会变成负数，把速度反向。
    //--------------------------------------------------------------------------
    if (body.linearDamping > real(0)) {
        body.linearVelocity *= std::pow(Max(real(0), real(1) - body.linearDamping), dt);
    }
    if (body.angularDamping > real(0)) {
        body.angularVelocity *=
            std::pow(Max(real(0), real(1) - body.angularDamping), dt);
    }
}

//==============================================================================
// 阶段 B：位置
//==============================================================================

void IntegratePosition(RigidBody& body, real dt) noexcept {
    // 静态体不动。运动学体**要**积分 —— 那是它和静态体的唯一区别。
    if (body.type == BodyType::Static || body.isSleeping || dt <= real(0)) return;

    body.position += body.linearVelocity * dt;

    // 四元数积分。Quat::Integrate 内部是一阶近似 q + 0.5*dt*(w⊗q) 再归一化，
    // 精度与稳定性的取舍见 M1 的 UPGRADE_NOTES。
    body.rotation = Quat::Integrate(body.rotation, body.angularVelocity, dt);

    // 姿态变了，世界惯量必须跟着变。漏了这一步的症状很隐蔽：
    // 物体在旋转过程中受到的碰撞响应会用错惯量，转起来"忽轻忽重"。
    UpdateWorldInertia(body);
}

//==============================================================================
// 休眠
//==============================================================================

bool UpdateSleep(RigidBody& body, real dt, const SleepConfig& config) noexcept {
    if (body.type != BodyType::Dynamic || body.isSleeping) return false;

    const bool slowEnough =
        body.linearVelocity.LengthSq() <
            config.linearThreshold * config.linearThreshold &&
        body.angularVelocity.LengthSq() <
            config.angularThreshold * config.angularThreshold;

    if (!slowEnough) {
        // 任何一项超标就重新计时。用"清零"而不是"减一点"，是因为休眠判据要的是
        // **连续**低速 —— 一个反复被撞醒的箱子不该攒够时间睡着。
        body.sleepTimer = real(0);
        return false;
    }

    body.sleepTimer += dt;
    if (body.sleepTimer < config.timeToSleep) return false;

    body.ForceSleep();
    return true;
}

//==============================================================================
// 渲染插值
//==============================================================================

Transform InterpolateTransform(const Transform& previous, const Transform& current,
                               real alpha) noexcept {
    const real t = Clamp(alpha, real(0), real(1));
    return Transform(Lerp(previous.position, current.position, t),
                     Quat::Slerp(previous.rotation, current.rotation, t));
}

}  // namespace pe
