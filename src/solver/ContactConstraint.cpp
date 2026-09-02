//==============================================================================
// src/solver/ContactConstraint.cpp
//
// 接触约束的三段式求解。公式推导见 ContactConstraint.h。
//==============================================================================

#include "pe/solver/ContactConstraint.h"

namespace pe {

namespace {

/// 约束空间的等效质量：在 rA / rB 处沿 dir 施加单位冲量，
/// 接触点处沿 dir 的相对速度会变化多少。
///
///     k = invMassA + invMassB
///       + Dot(dir, Cross(invIA * Cross(rA, dir), rA))
///       + Dot(dir, Cross(invIB * Cross(rB, dir), rB))
///
/// 前两项是平动贡献，后两项是转动贡献。转动那两项的含义：
/// 冲量 J*dir 在 rA 处产生角冲量 rA x (J*dir)，角速度变化 invIA * (rA x dir) * J，
/// 这个角速度让接触点获得 (invIA*(rA x dir)) x rA * J 的速度，
/// 再投影回 dir 上就是它对 k 的贡献。
///
/// 返回的是 k 的**倒数**（0 表示两个物体都是无穷大质量，这个方向解不动）。
real EffectiveMass(const RigidBody& a, const RigidBody& b, const Vec3& rA,
                   const Vec3& rB, const Vec3& dir) noexcept {
    const Vec3 angularA = Cross(a.invInertiaWorld * Cross(rA, dir), rA);
    const Vec3 angularB = Cross(b.invInertiaWorld * Cross(rB, dir), rB);

    const real k = a.invMass + b.invMass + Dot(dir, angularA) + Dot(dir, angularB);
    return k > real(0) ? real(1) / k : real(0);
}

/// 接触点处 B 相对 A 的速度：dv = (vB + wB x rB) - (vA + wA x rA)
inline Vec3 RelativeVelocity(const RigidBody& a, const RigidBody& b, const Vec3& rA,
                             const Vec3& rB) noexcept {
    return (b.linearVelocity + Cross(b.angularVelocity, rB)) -
           (a.linearVelocity + Cross(a.angularVelocity, rA));
}

/// 施加一对大小相等方向相反的冲量。
///
/// 法线由 A 指向 B，所以正的冲量把 B 沿 +normal 推、把 A 沿 -normal 推。
/// invMass = 0 的物体在这里自动不受影响，不需要任何 if 分支（见 RigidBody.h）。
inline void ApplyImpulsePair(RigidBody& a, RigidBody& b, const Vec3& rA, const Vec3& rB,
                             const Vec3& impulse) noexcept {
    a.linearVelocity -= impulse * a.invMass;
    a.angularVelocity -= a.invInertiaWorld * Cross(rA, impulse);
    b.linearVelocity += impulse * b.invMass;
    b.angularVelocity += b.invInertiaWorld * Cross(rB, impulse);
}

}  // namespace

//==============================================================================
// 构造
//==============================================================================

ContactConstraint MakeContactConstraint(RigidBody* bodyA, RigidBody* bodyB,
                                        const Manifold& manifold,
                                        const Material& materialA,
                                        const Material& materialB,
                                        std::uint64_t pairKey) noexcept {
    ContactConstraint c;
    c.bodyA = bodyA;
    c.bodyB = bodyB;
    c.normal = manifold.normal;

    // 切向基由法线现算。用 Erin Catto 那个无分支构造（见 Vec3.h），
    // 它在 n 接近任何一根坐标轴时都不退化 —— 而"角色站在水平地面上"
    // 恰好就是 n = +Y，是最常见的情况，朴素写法必然踩坑。
    BuildOrthonormalBasis(c.normal, c.tangent[0], c.tangent[1]);

    c.friction = CombineFriction(materialA, materialB);
    c.restitution = CombineRestitution(materialA, materialB);
    c.pairKey = pairKey;
    c.pointCount = manifold.pointCount;

    for (std::uint8_t i = 0; i < manifold.pointCount; ++i) {
        ContactConstraintPoint& p = c.points[i];
        const ContactPoint& mp = manifold.points[i];

        p.rA = mp.position - bodyA->position;
        p.rB = mp.position - bodyB->position;
        p.penetration = mp.penetration;
        p.featureId = mp.featureId;

        // 累积冲量的初值。求解器随后会用接触缓存里上一帧的值覆盖它（warm starting）。
        p.normalImpulse = real(0);
        p.tangentImpulse[0] = real(0);
        p.tangentImpulse[1] = real(0);

        p.normalMass = real(0);
        p.tangentMass[0] = real(0);
        p.tangentMass[1] = real(0);
        p.velocityBias = real(0);
    }

    return c;
}

//==============================================================================
// Prepare
//==============================================================================

void ContactConstraint::Prepare(const SolverConfig& config, real dt) noexcept {
    const RigidBody& a = *bodyA;
    const RigidBody& b = *bodyB;

    const real invDt = (dt > real(0)) ? real(1) / dt : real(0);

    for (std::uint8_t i = 0; i < pointCount; ++i) {
        ContactConstraintPoint& p = points[i];

        p.normalMass = EffectiveMass(a, b, p.rA, p.rB, normal);
        p.tangentMass[0] = EffectiveMass(a, b, p.rA, p.rB, tangent[0]);
        p.tangentMass[1] = EffectiveMass(a, b, p.rA, p.rB, tangent[1]);

        //----------------------------------------------------------------------
        // 偏置项 = Baumgarte 位置修正 + 恢复系数
        //
        // 求解的目标是 `Dot(dv, n) + bias >= 0`，所以 bias 取**负值**表示
        // "要求它们以某个速度分离"。
        //----------------------------------------------------------------------
        real bias = real(0);

        // (1) Baumgarte：把超出 slop 的那部分穿透，按 beta 的比例转成分离速度。
        //     上限 maxLinearCorrection 是防"爆开"—— 物体生成时嵌进墙里 1 米深，
        //     没有上限的话会被一脚踹飞。
        const real correctable = p.penetration - config.linearSlop;
        if (correctable > real(0)) {
            bias = -Min(config.baumgarte * correctable * invDt,
                        config.maxLinearCorrection);
        }

        // (2) 恢复：用**求解前**的接近速度算目标分离速度。
        //     阈值判断不可少：静止在地面上的物体每帧都有一丁点接近速度
        //     （来自这一帧的重力积分），没有阈值它会永远微微弹跳、永远睡不着。
        const Vec3 dv = RelativeVelocity(a, b, p.rA, p.rB);
        const real vn = Dot(dv, normal);
        if (vn < -config.restitutionThreshold) {
            bias += restitution * vn;
        }

        p.velocityBias = bias;
    }
}

//==============================================================================
// WarmStart
//==============================================================================

void ContactConstraint::WarmStart() noexcept {
    RigidBody& a = *bodyA;
    RigidBody& b = *bodyB;

    for (std::uint8_t i = 0; i < pointCount; ++i) {
        const ContactConstraintPoint& p = points[i];

        // 把继承来的三个方向的累积冲量一次性施加出去。
        // 这不是"额外的能量"—— 上一帧这些冲量就在维持接触，这一帧接着维持而已。
        const Vec3 impulse = normal * p.normalImpulse +
                             tangent[0] * p.tangentImpulse[0] +
                             tangent[1] * p.tangentImpulse[1];

        ApplyImpulsePair(a, b, p.rA, p.rB, impulse);
    }
}

//==============================================================================
// SolveVelocity
//==============================================================================

void ContactConstraint::SolveVelocity() noexcept {
    RigidBody& a = *bodyA;
    RigidBody& b = *bodyB;

    //--------------------------------------------------------------------------
    // 先摩擦，后法向。
    //
    // 摩擦的上限 mu * normalImpulse 用的是**上一次迭代**的法向冲量，略微滞后。
    // 反过来先解法向的话，法向冲量在本次迭代内变大 -> 摩擦上限跟着变大 ->
    // 摩擦冲量变大 -> 又影响法向，两者互相放大，接触点一多就容易震荡。
    //--------------------------------------------------------------------------
    for (std::uint8_t i = 0; i < pointCount; ++i) {
        ContactConstraintPoint& p = points[i];

        for (int k = 0; k < 2; ++k) {
            const Vec3 dv = RelativeVelocity(a, b, p.rA, p.rB);
            const real vt = Dot(dv, tangent[k]);

            // 目标是让切向相对速度归零
            real lambda = -vt * p.tangentMass[k];

            //------------------------------------------------------------------
            // 库仑锥的**方盒近似**：两个切向各自钳到 ±mu*Pn。
            //
            // TODO(upgrade): 严格的库仑锥要求的是二维切向冲量的**长度**不超过
            //   mu*Pn（一个圆），这里钳的是每个分量（一个正方形），
            //   于是沿对角线方向的摩擦上限被放大到 sqrt(2)*mu。
            //   由于切向基是由法线现算的、方向是任意的，这表现为**摩擦力大小
            //   依赖于切向基的朝向**。
            //   改法：两个切向都解完之后，把 (t0, t1) 当成一个二维向量，
            //   长度超过 mu*Pn 就整体缩放回去。代价是要把两个方向的求解耦合起来。
            //------------------------------------------------------------------
            const real maxFriction = friction * p.normalImpulse;

            // 关键：钳的是**累积**冲量，施加的是差值。
            // 直接把 lambda 钳到范围内是错的 —— 那样就永远无法撤销之前施加过头的
            // 冲量，多个接触点会互相顶死。详见 ContactConstraint.h。
            const real oldImpulse = p.tangentImpulse[k];
            p.tangentImpulse[k] = Clamp(oldImpulse + lambda, -maxFriction, maxFriction);
            lambda = p.tangentImpulse[k] - oldImpulse;

            ApplyImpulsePair(a, b, p.rA, p.rB, tangent[k] * lambda);
        }
    }

    //--------------------------------------------------------------------------
    // 法向
    //--------------------------------------------------------------------------
    for (std::uint8_t i = 0; i < pointCount; ++i) {
        ContactConstraintPoint& p = points[i];

        const Vec3 dv = RelativeVelocity(a, b, p.rA, p.rB);
        const real vn = Dot(dv, normal);

        real lambda = -(vn + p.velocityBias) * p.normalMass;

        // 累积冲量钳到非负：接触只能推开，不能拉住。
        const real oldImpulse = p.normalImpulse;
        p.normalImpulse = Max(oldImpulse + lambda, real(0));
        lambda = p.normalImpulse - oldImpulse;

        ApplyImpulsePair(a, b, p.rA, p.rB, normal * lambda);
    }

    //--------------------------------------------------------------------------
    // 把切向冲量重新收进**这一轮更新后**的库仑锥。
    //
    // 为什么必须补这一步：上面解摩擦时用的是上一轮的法向冲量做上限，
    // 而法向冲量在本轮可能变小 —— 极端情况下变成 0（接触实际上已经分开了）。
    // 不重新收的话，会留下"法向压力为零、却还有摩擦力"的接触点，
    // 那是一个凭空产生的切向力：一摞静止的箱子会被它推得慢慢横向漂移。
    //
    // 关键是**差值照常施加**，不能只改累加器。只改累加器的话，
    // 已经作用到速度上的那部分冲量就无人认领了，账目对不上，
    // warm starting 下一帧继承过去的值也是错的。
    //--------------------------------------------------------------------------
    for (std::uint8_t i = 0; i < pointCount; ++i) {
        ContactConstraintPoint& p = points[i];
        const real maxFriction = friction * p.normalImpulse;

        for (int k = 0; k < 2; ++k) {
            const real clamped =
                Clamp(p.tangentImpulse[k], -maxFriction, maxFriction);
            const real delta = clamped - p.tangentImpulse[k];
            if (delta == real(0)) continue;

            p.tangentImpulse[k] = clamped;
            ApplyImpulsePair(a, b, p.rA, p.rB, tangent[k] * delta);
        }
    }
}

real ContactConstraint::MaxPenetration() const noexcept {
    real m = real(0);
    for (std::uint8_t i = 0; i < pointCount; ++i) {
        m = Max(m, points[i].penetration);
    }
    return m;
}

}  // namespace pe
