#pragma once
//==============================================================================
// pe/math/Quat.h
//
// 单位四元数：引擎中刚体姿态的**唯一**表示。
//
//------------------------------------------------------------------------------
// 为什么用四元数而不是旋转矩阵或欧拉角
//------------------------------------------------------------------------------
// 对比欧拉角：欧拉角有万向锁（gimbal lock），插值不均匀，且"先绕哪个轴"有 12 种
//   约定，接口一旦定错后患无穷。物理里角速度是任意方向的，欧拉角根本不适合积分。
//
// 对比旋转矩阵：矩阵有 9 个数但只有 3 个自由度，冗余度高。逐帧积分之后浮点误差
//   会让它慢慢丧失正交性（行不再互相垂直），表现为物体被逐渐"剪切拉伸"。
//   要修复得做 Gram-Schmidt 正交化，代价不小。四元数只有 4 个数、3 个自由度，
//   纠正误差只需要一次归一化 —— 每帧做一次，成本几乎为零。
//
// 四元数还有一个物理引擎特别看重的性质：角速度积分可以写成一个漂亮的闭式，
// 见下面 Integrate() 的推导。
//
//------------------------------------------------------------------------------
// 约定
//------------------------------------------------------------------------------
//   - q = (x, y, z, w)，其中 w 是**实部**，(x,y,z) 是虚部/向量部。
//     （注意：有些库把 w 放在最前面，交换数据时要当心。）
//   - 表示"绕单位轴 n 转 theta 角"的四元数是：
//         q = ( n * sin(theta/2), cos(theta/2) )
//     半角是四元数的本质：旋转作用在向量上是 q * v * q^-1，用了两次 q，
//     所以每个 q 只需要"转一半"。
//   - 复合遵循和矩阵一致的从右往左：(q1 * q2) 表示先做 q2 再做 q1。
//   - 右手坐标系，正角度是逆时针（从轴的正方向朝原点看）。
//==============================================================================

#include "pe/core/Types.h"
#include "pe/math/Mat3.h"
#include "pe/math/MathUtil.h"
#include "pe/math/Vec3.h"

namespace pe {

struct Quat {
    real x, y, z, w;  ///< w 是实部

    Quat() = default;

    constexpr Quat(real x_, real y_, real z_, real w_) noexcept
        : x(x_), y(y_), z(z_), w(w_) {}

    /// 由向量部和实部构造。
    constexpr Quat(const Vec3& v, real w_) noexcept : x(v.x), y(v.y), z(v.z), w(w_) {}

    static constexpr Quat Identity() noexcept { return Quat(0, 0, 0, 1); }

    /// 向量部。角速度积分和 Rotate 的推导里反复出现。
    constexpr Vec3 Vector() const noexcept { return Vec3(x, y, z); }

    //--------------------------------------------------------------------------
    // 构造
    //--------------------------------------------------------------------------

    /// 绕单位轴 axis 旋转 angleRad 弧度。
    /// axis 会被归一化；零轴退化为单位四元数（不旋转）而不是产生 NaN。
    static Quat FromAxisAngle(const Vec3& axis, real angleRad) noexcept {
        const Vec3 n = axis.Normalized();
        if (n.IsZero()) {
            return Identity();
        }
        const real half = angleRad * real(0.5);
        return Quat(n * Sin(half), Cos(half));
    }

    /// 从旋转矩阵恢复四元数（Shepperd 方法）。
    ///
    /// 原理：把 ToMat3() 的元素代回去解方程。最直接的做法是
    ///     w = sqrt(1 + trace) / 2
    /// 但当 trace 接近 -1（旋转接近 180 度）时，w 接近 0，除以 4w 会放大误差
    /// 甚至除零。Shepperd 的做法是在 w、x、y、z 四个分量里挑**当前最大的那个**
    /// 先算出来（它一定不接近 0），再用它去除其余三个，从而保证数值稳定。
    static Quat FromMat3(const Mat3& m) noexcept {
        const real trace = m.At(0, 0) + m.At(1, 1) + m.At(2, 2);
        if (trace > real(0)) {
            // w 是最大的分量
            const real s = Sqrt(trace + real(1)) * real(2);  // s = 4w
            return Quat((m.At(2, 1) - m.At(1, 2)) / s,
                        (m.At(0, 2) - m.At(2, 0)) / s,
                        (m.At(1, 0) - m.At(0, 1)) / s,
                        real(0.25) * s);
        }
        if (m.At(0, 0) > m.At(1, 1) && m.At(0, 0) > m.At(2, 2)) {
            // x 是最大的分量
            const real s =
                Sqrt(real(1) + m.At(0, 0) - m.At(1, 1) - m.At(2, 2)) * real(2);  // 4x
            return Quat(real(0.25) * s,
                        (m.At(0, 1) + m.At(1, 0)) / s,
                        (m.At(0, 2) + m.At(2, 0)) / s,
                        (m.At(2, 1) - m.At(1, 2)) / s);
        }
        if (m.At(1, 1) > m.At(2, 2)) {
            // y 是最大的分量
            const real s =
                Sqrt(real(1) + m.At(1, 1) - m.At(0, 0) - m.At(2, 2)) * real(2);  // 4y
            return Quat((m.At(0, 1) + m.At(1, 0)) / s,
                        real(0.25) * s,
                        (m.At(1, 2) + m.At(2, 1)) / s,
                        (m.At(0, 2) - m.At(2, 0)) / s);
        }
        // z 是最大的分量
        const real s = Sqrt(real(1) + m.At(2, 2) - m.At(0, 0) - m.At(1, 1)) * real(2);  // 4z
        return Quat((m.At(0, 2) + m.At(2, 0)) / s,
                    (m.At(1, 2) + m.At(2, 1)) / s,
                    real(0.25) * s,
                    (m.At(1, 0) - m.At(0, 1)) / s);
    }

    //--------------------------------------------------------------------------
    // 基本运算
    //--------------------------------------------------------------------------

    constexpr real LengthSq() const noexcept { return x * x + y * y + z * z + w * w; }
    real Length() const noexcept { return Sqrt(LengthSq()); }

    /// 归一化。逐帧积分后必须调用，否则误差累积会让四元数偏离单位球面，
    /// 表现为物体被缓慢缩放。
    Quat Normalized() const noexcept {
        const real lenSq = LengthSq();
        if (lenSq < kNormalizeEpsilonSq) {
            return Identity();  // 退化时回到"不旋转"，绝不返回 NaN
        }
        const real inv = real(1) / Sqrt(lenSq);
        return Quat(x * inv, y * inv, z * inv, w * inv);
    }

    void Normalize() noexcept { *this = Normalized(); }

    /// 共轭：向量部取反。对**单位**四元数来说共轭就等于逆，
    /// 所以引擎里所有求逆的地方都直接用共轭（省一次除法）。
    constexpr Quat Conjugate() const noexcept { return Quat(-x, -y, -z, w); }

    /// 一般逆 = 共轭 / |q|^2。只在不能保证 q 是单位四元数时才用。
    constexpr Quat Inverse() const noexcept {
        const real lenSq = LengthSq();
        if (lenSq < real(1e-12)) {
            return Identity();
        }
        const real inv = real(1) / lenSq;
        return Quat(-x * inv, -y * inv, -z * inv, w * inv);
    }

    constexpr Quat operator-() const noexcept { return Quat(-x, -y, -z, -w); }

    //--------------------------------------------------------------------------
    // 旋转向量
    //--------------------------------------------------------------------------

    /// 用这个四元数旋转向量 v。
    ///
    /// 教科书公式是 v' = q * (v,0) * q^-1，直接展开要做两次四元数乘法（约 32 次
    /// 乘法）。下面用的是等价的展开式（约 18 次乘法）：
    ///
    ///     令 u = q 的向量部，w = q 的实部，则
    ///     v' = v + 2w(u × v) + 2u × (u × v)
    ///
    /// 推导要点：把 q*(v,0)*q^* 展开，利用四元数乘法中虚部乘虚部会同时产生
    /// 点积项和叉积项，实部项最终相消，只剩下上面这三项。
    /// 代码里令 t = 2(u × v)，于是 v' = v + w*t + u × t。
    Vec3 Rotate(const Vec3& v) const noexcept {
        const Vec3 u(x, y, z);
        const Vec3 t = real(2) * Cross(u, v);
        return v + w * t + Cross(u, t);
    }

    /// 用这个四元数的逆旋转向量（世界空间 -> 局部空间）。
    Vec3 RotateInverse(const Vec3& v) const noexcept { return Conjugate().Rotate(v); }

    //--------------------------------------------------------------------------
    // 转成矩阵
    //--------------------------------------------------------------------------

    /// 转成 3x3 旋转矩阵（行优先，配合 v' = M * v）。
    ///
    /// 每一项都来自把 Rotate() 的展开式作用在三个基向量上再整理。
    /// 每帧对每个动态刚体调用一次（用来算 invInertiaWorld），所以值得写成
    /// 先算出所有平方项和交叉项再组装，避免重复乘法。
    Mat3 ToMat3() const noexcept {
        const real xx = x * x, yy = y * y, zz = z * z;
        const real xy = x * y, xz = x * z, yz = y * z;
        const real wx = w * x, wy = w * y, wz = w * z;

        return Mat3(
            Vec3(real(1) - real(2) * (yy + zz), real(2) * (xy - wz), real(2) * (xz + wy)),
            Vec3(real(2) * (xy + wz), real(1) - real(2) * (xx + zz), real(2) * (yz - wx)),
            Vec3(real(2) * (xz - wy), real(2) * (yz + wx), real(1) - real(2) * (xx + yy)));
    }

    //--------------------------------------------------------------------------
    // 轴角分解
    //--------------------------------------------------------------------------

    /// 旋转角度（弧度），范围 [0, pi]。
    real Angle() const noexcept {
        // |w| = cos(theta/2)。取绝对值是因为 q 和 -q 表示同一个旋转，
        // 我们总是返回"较短的那条路"对应的角度。
        return real(2) * SafeAcos(Abs(w));
    }

    /// 旋转轴（单位向量）。旋转角为 0 时轴无意义，返回 +X 作为约定值。
    Vec3 Axis() const noexcept {
        const Vec3 v = Vector();
        const real lenSq = v.LengthSq();
        if (lenSq < kNormalizeEpsilonSq) {
            return Vec3::UnitX();
        }
        // w < 0 时取 -q（同一旋转的另一种表示），保证角度落在 [0, pi]
        const Vec3 axis = v * (real(1) / Sqrt(lenSq));
        return w < real(0) ? -axis : axis;
    }

    //--------------------------------------------------------------------------
    // 角速度积分
    //--------------------------------------------------------------------------

    /// 用世界空间角速度 omega（rad/s）把姿态往前推进 dt。
    ///
    /// 推导：
    ///   四元数的运动学方程是   dq/dt = 0.5 * omega_q * q
    ///   其中 omega_q = (omega, 0) 是把角速度写成纯四元数（实部为 0）。
    ///
    ///   这个 0.5 从哪来？把 q(t+dt) 写成"先有一个小旋转 dq 再乘上 q"：
    ///       q(t+dt) = FromAxisAngle(n, |omega|*dt) * q(t)
    ///   而 FromAxisAngle 里出现的是半角，所以对小角度 theta = |omega|*dt 有
    ///       FromAxisAngle ≈ ( n * (theta/2), 1 )
    ///   那个 /2 就是运动学方程里的 0.5。
    ///
    ///   一阶展开得到本函数用的显式形式：
    ///       q(t+dt) ≈ q + 0.5 * dt * (omega_q * q)，再归一化
    ///
    /// 为什么"再归一化"是必须的：一阶截断让结果稍微离开单位球面（相当于混进了
    /// 一点缩放），不归一化的话误差会指数累积，几秒之内物体就会明显变形。
    ///
    /// 注意 omega 必须是**世界空间**的角速度，因为这里是左乘 q（先转 q，再在
    /// 世界空间叠加小旋转）。RigidBody::angularVelocity 就是按世界空间存的。
    ///
    /// TODO(upgrade): 高速自旋（每步转角接近 pi）时一阶近似误差明显。届时可换成
    ///   指数映射的精确形式 q_new = FromAxisAngle(omega.Normalized(), |omega|*dt) * q，
    ///   代价是一次 sin/cos。当前 FPS 场景的物体自旋速度远达不到这个量级。
    ///
    /// 实现放在本文件末尾：它要用到四元数乘法 operator*，而那是个自由函数，
    /// 必须在结构体定义完成之后才能声明。
    static Quat Integrate(const Quat& q, const Vec3& omega, real dt) noexcept;

    //--------------------------------------------------------------------------
    // 插值
    //--------------------------------------------------------------------------

    /// 球面线性插值：沿四维单位球面上的大圆弧以**匀角速度**从 a 转到 b。
    ///
    /// 为什么不能直接对四个分量做线性插值（nlerp）：nlerp 走的是弦而不是弧，
    /// 归一化之后角速度会在中间变快、两端变慢。做相机或骨骼动画时肉眼可见。
    /// 物理渲染插值（GetInterpolatedTransform）里两帧之间转角很小，nlerp 其实
    /// 够用且更快 —— 但接口统一提供 Slerp，需要性能时再换。
    ///
    /// 两个细节：
    ///   1. q 和 -q 表示同一个旋转。若 Dot(a,b) < 0，说明按原样插值会绕远路
    ///      （走大于 180 度的弧），所以先把 b 取反，保证走短弧。
    ///   2. 两者几乎重合时 sin(theta) 接近 0，公式会除零 —— 退化为线性插值，
    ///      此时两者的差别本来就在浮点噪声级别。
    static Quat Slerp(const Quat& a, const Quat& b, real t) noexcept {
        real cosTheta = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;

        Quat target = b;
        if (cosTheta < real(0)) {
            target = -b;
            cosTheta = -cosTheta;
        }

        if (cosTheta > real(0.9995)) {
            // 几乎同向：线性插值 + 归一化，避免 sin(theta) 除零
            return Quat(Lerp(a.x, target.x, t),
                        Lerp(a.y, target.y, t),
                        Lerp(a.z, target.z, t),
                        Lerp(a.w, target.w, t))
                .Normalized();
        }

        const real theta = SafeAcos(cosTheta);       // 两者夹角
        const real sinTheta = Sin(theta);
        const real wa = Sin((real(1) - t) * theta) / sinTheta;
        const real wb = Sin(t * theta) / sinTheta;

        return Quat(a.x * wa + target.x * wb,
                    a.y * wa + target.y * wb,
                    a.z * wa + target.z * wb,
                    a.w * wa + target.w * wb);
    }
};

//------------------------------------------------------------------------------
// 四元数乘法（Hamilton 积）
//
// (q1 * q2) 的含义是"先施加 q2 的旋转，再施加 q1 的旋转"，和矩阵乘法的
// 从右往左顺序一致。
//
// 展开式来自四元数基的乘法表：i^2 = j^2 = k^2 = ijk = -1。
// 实部由点积贡献（带负号），虚部由叉积和实部与虚部的交叉项贡献：
//     (v1, w1) * (v2, w2) = ( w1*v2 + w2*v1 + v1 × v2 ,  w1*w2 - v1·v2 )
// 注意 v1 × v2 这一项使得四元数乘法**不可交换** —— 这正好对应"旋转的先后
// 顺序不可交换"这个物理事实。
//------------------------------------------------------------------------------
inline constexpr Quat operator*(const Quat& a, const Quat& b) noexcept {
    return Quat(a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z);
}

inline constexpr Quat operator+(const Quat& a, const Quat& b) noexcept {
    return Quat(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
}
inline constexpr Quat operator-(const Quat& a, const Quat& b) noexcept {
    return Quat(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w);
}
inline constexpr Quat operator*(const Quat& q, real s) noexcept {
    return Quat(q.x * s, q.y * s, q.z * s, q.w * s);
}
inline constexpr Quat operator*(real s, const Quat& q) noexcept { return q * s; }

inline constexpr real Dot(const Quat& a, const Quat& b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

inline constexpr bool operator==(const Quat& a, const Quat& b) noexcept {
    return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;
}
inline constexpr bool operator!=(const Quat& a, const Quat& b) noexcept { return !(a == b); }

/// 逐分量比较。注意 q 和 -q 表示同一旋转但这里会判为不等 ——
/// 要比较"旋转是否相同"请用 SameRotation()。
inline constexpr bool NearlyEqual(const Quat& a, const Quat& b,
                                  real eps = kEpsilon) noexcept {
    return NearlyEqual(a.x, b.x, eps) && NearlyEqual(a.y, b.y, eps) &&
           NearlyEqual(a.z, b.z, eps) && NearlyEqual(a.w, b.w, eps);
}

/// 比较两个四元数是否表示同一个旋转（自动处理 q 与 -q 的双重覆盖）。
/// 单位四元数的点积等于 cos(夹角/2)，所以 |dot| 接近 1 就是同一旋转。
inline bool SameRotation(const Quat& a, const Quat& b, real eps = real(1e-5)) noexcept {
    return Abs(Abs(Dot(a, b)) - real(1)) <= eps;
}

//------------------------------------------------------------------------------
// 角速度积分的实现（推导见上面 Quat::Integrate 的声明处）
//
//     q(t+dt) ≈ q + 0.5 * dt * (omega_q * q)，然后归一化
//------------------------------------------------------------------------------
inline Quat Quat::Integrate(const Quat& q, const Vec3& omega, real dt) noexcept {
    // 把角速度写成实部为 0 的纯四元数
    const Quat omegaQ(omega.x, omega.y, omega.z, real(0));
    // 左乘：小旋转叠加在世界空间上（所以 omega 必须是世界空间角速度）
    const Quat dq = omegaQ * q;
    const real halfDt = real(0.5) * dt;
    return Quat(q.x + dq.x * halfDt,
                q.y + dq.y * halfDt,
                q.z + dq.z * halfDt,
                q.w + dq.w * halfDt)
        .Normalized();
}

}  // namespace pe
