#pragma once
//==============================================================================
// pe/collision/GeometryUtil.h
//
// 底层几何工具：点、线段、平面之间的最近点与距离。
//
// 这些函数看起来很朴素，但它们是窄相位的地基：
//   - 球 vs 胶囊  = 点到线段的距离 vs 半径和
//   - 胶囊 vs 胶囊 = 线段到线段的距离 vs 半径和
//   - 球 vs 盒    = 点到盒的最近点（在 AABB::ClosestPoint 里）
//   - 射线 vs 胶囊 = 先用点到线段的距离判断起点是否在内部
//
// 全部是纯函数、无状态、可 constexpr 的头文件实现。
//==============================================================================

#include "pe/core/Types.h"
#include "pe/math/MathUtil.h"
#include "pe/math/Vec3.h"

namespace pe {

//------------------------------------------------------------------------------
// 点 vs 线段
//------------------------------------------------------------------------------

/// 线段 [a, b] 上离 p 最近的点。
/// outT 输出该点的参数（0 表示端点 a，1 表示端点 b），调用方常常需要它
/// 来判断"最近点是落在中间还是端点上"。
///
/// 原理：把 p 投影到直线 ab 上。投影参数
///     t = ((p - a) · (b - a)) / |b - a|^2
/// 分母是 |ab|^2 而不是 |ab|，因为分子里的点积已经带了一个 |ab| 的量纲。
/// 然后把 t 钳制到 [0, 1] —— 钳制这一步就是"线段"和"无限直线"的全部区别，
/// 也正是它让端点处的半球端盖能被正确处理。
inline Vec3 ClosestPointOnSegment(const Vec3& p, const Vec3& a, const Vec3& b,
                                  real& outT) noexcept {
    const Vec3 ab = b - a;
    const real lenSq = ab.LengthSq();

    // 退化成一个点（胶囊的 halfHeight 为 0，即退化成球）
    if (lenSq < kNormalizeEpsilonSq) {
        outT = real(0);
        return a;
    }

    outT = Clamp(Dot(p - a, ab) / lenSq, real(0), real(1));
    return a + ab * outT;
}

/// 同上，不关心参数 t 时的简化版。
inline Vec3 ClosestPointOnSegment(const Vec3& p, const Vec3& a, const Vec3& b) noexcept {
    real t;
    return ClosestPointOnSegment(p, a, b, t);
}

/// 点到线段的距离平方。开方留给调用方 —— 和半径比较时用平方可以省掉 sqrt。
inline real DistanceSqPointSegment(const Vec3& p, const Vec3& a, const Vec3& b) noexcept {
    return (p - ClosestPointOnSegment(p, a, b)).LengthSq();
}

//------------------------------------------------------------------------------
// 线段 vs 线段
//------------------------------------------------------------------------------

/// 两条线段 [p1,q1] 与 [p2,q2] 之间的最近点对。
/// outS / outT 是各自的参数（0 = 起点，1 = 终点），outC1 / outC2 是最近点。
/// 返回两个最近点之间的距离平方。
///
/// 这是胶囊-胶囊窄相位的全部内容：两条胶囊相交，当且仅当它们的中轴线段之间的
/// 距离小于半径和。
///
/// 原理（Ericson《Real-Time Collision Detection》5.1.9）：
/// 先当成两条**无限直线**解二元线性方程组求最近点参数，再把参数钳到 [0,1]。
/// 钳制之后解不再是最优的，所以要**重新求另一条线段上对应的最近点**并再钳一次
/// —— 下面那两个 `if (t < 0)` / `else if (t > 1)` 分支就是干这个的。
/// 少了这一步，"两条错开的线段"会算出偏大的距离，表现为两个胶囊明明贴着却不碰撞。
///
/// 退化情况全部有兜底：两条线段都退化成点、其中一条退化成点、两条平行
/// （denom 为 0，此时任意 s 都行，取 0）。
inline real ClosestPointsSegmentSegment(const Vec3& p1, const Vec3& q1, const Vec3& p2,
                                        const Vec3& q2, real& outS, real& outT,
                                        Vec3& outC1, Vec3& outC2) noexcept {
    const Vec3 d1 = q1 - p1;  // 线段 1 的方向（未归一化）
    const Vec3 d2 = q2 - p2;
    const Vec3 r = p1 - p2;
    const real a = d1.LengthSq();
    const real e = d2.LengthSq();
    const real f = Dot(d2, r);

    real s = real(0);
    real t = real(0);

    if (a <= kNormalizeEpsilonSq && e <= kNormalizeEpsilonSq) {
        // 两条都退化成点
        s = real(0);
        t = real(0);
    } else if (a <= kNormalizeEpsilonSq) {
        // 线段 1 退化成点
        s = real(0);
        t = Clamp(f / e, real(0), real(1));
    } else {
        const real c = Dot(d1, r);
        if (e <= kNormalizeEpsilonSq) {
            // 线段 2 退化成点
            t = real(0);
            s = Clamp(-c / a, real(0), real(1));
        } else {
            const real b = Dot(d1, d2);
            const real denom = a * e - b * b;  // 恒 >= 0（柯西-施瓦茨）

            // denom 为 0 表示两条线段平行。此时最近点不唯一，取 s = 0
            // 这一端，后面的钳制会把它调整到合理位置。
            // 阈值取相对量：denom 的量纲是长度的四次方。
            s = (denom > kEpsilon * a * e)
                    ? Clamp((b * f - c * e) / denom, real(0), real(1))
                    : real(0);

            t = (b * s + f) / e;

            // t 出界之后 s 要重算：这一步才是"线段"与"直线"的区别所在。
            if (t < real(0)) {
                t = real(0);
                s = Clamp(-c / a, real(0), real(1));
            } else if (t > real(1)) {
                t = real(1);
                s = Clamp((b - c) / a, real(0), real(1));
            }
        }
    }

    outS = s;
    outT = t;
    outC1 = p1 + d1 * s;
    outC2 = p2 + d2 * t;
    return (outC1 - outC2).LengthSq();
}

//------------------------------------------------------------------------------
// 线段 vs 轴对齐盒
//------------------------------------------------------------------------------

/// 线段 [a,b] 与以原点为中心、半尺寸为 halfExtents 的轴对齐盒之间的最近点对。
/// 返回距离平方（线段与盒相交时为 0）。
///
/// 这是胶囊-盒窄相位的地基。之所以要单独写而不是"取线段中点再 clamp"：
/// 那种近似在胶囊斜穿盒子角落时会错得很离谱。
///
/// 算法（精确，不迭代）：
/// 令 f(t) = |clamp(a + t*d) - (a + t*d)|^2，即线段上参数 t 处的点到盒的距离平方。
/// 每根轴上的 clamp 只有"低于 -h / 在区间内 / 高于 +h"三种状态，状态只在
/// a_i + t*d_i 恰好等于 ±h_i 时切换，所以整条线段被最多 6 个断点分成最多 7 段。
/// **在每一段内夹取模式是固定的**，于是 f(t) 退化成一个普通的二次函数，
/// 顶点可以直接解出来。逐段求极小再取最好的那个，就得到了全局最优 ——
/// f 是凸函数，所以这个"分段求极小"确实给出全局解。
inline real ClosestPointSegmentAABB(const Vec3& a, const Vec3& b,
                                    const Vec3& halfExtents, real& outT,
                                    Vec3& outSegPoint, Vec3& outBoxPoint) noexcept {
    const Vec3 d = b - a;

    // 收集断点。最多 6 个（每根轴两个），加上线段两端共 8 个。
    real ts[8];
    int count = 0;
    ts[count++] = real(0);
    ts[count++] = real(1);

    for (int i = 0; i < 3; ++i) {
        if (Abs(d[i]) <= kEpsilon) continue;  // 该轴上不动，永远不会切换状态
        const real inv = real(1) / d[i];
        const real t0 = (-halfExtents[i] - a[i]) * inv;
        const real t1 = (halfExtents[i] - a[i]) * inv;
        if (t0 > real(0) && t0 < real(1)) ts[count++] = t0;
        if (t1 > real(0) && t1 < real(1)) ts[count++] = t1;
    }

    // 插入排序：最多 8 个元素，比调 std::sort 划算，也省一个头文件依赖。
    for (int i = 1; i < count; ++i) {
        const real key = ts[i];
        int j = i - 1;
        while (j >= 0 && ts[j] > key) {
            ts[j + 1] = ts[j];
            --j;
        }
        ts[j + 1] = key;
    }

    real bestT = real(0);
    real bestDistSq = real(-1);

    for (int seg = 0; seg + 1 < count; ++seg) {
        const real lo = ts[seg];
        const real hi = ts[seg + 1];
        if (hi - lo <= real(0)) continue;  // 重复断点

        // 用区间中点判定这一段的夹取模式（模式在段内是常数）
        const real mid = (lo + hi) * real(0.5);
        const Vec3 pm = a + d * mid;

        // f(t) = A*t^2 + B*t + C，只有被夹取的那些轴才贡献
        real coefA = real(0);
        real coefB = real(0);
        for (int i = 0; i < 3; ++i) {
            real ci;
            if (pm[i] < -halfExtents[i]) {
                ci = -halfExtents[i];
            } else if (pm[i] > halfExtents[i]) {
                ci = halfExtents[i];
            } else {
                continue;  // 这根轴上没被夹，不贡献距离
            }
            const real e0 = a[i] - ci;
            coefA += d[i] * d[i];
            coefB += real(2) * e0 * d[i];
        }

        // 二次函数顶点 t = -B / 2A；A 为 0 表示这一段里 f 是常数
        const real t = (coefA > kEpsilon) ? Clamp(-coefB / (real(2) * coefA), lo, hi) : lo;

        // 不用 A/B/C 反算距离，直接算 —— 避免把"夹取模式"的假设带进结果
        const Vec3 p = a + d * t;
        const Vec3 q(Clamp(p.x, -halfExtents.x, halfExtents.x),
                     Clamp(p.y, -halfExtents.y, halfExtents.y),
                     Clamp(p.z, -halfExtents.z, halfExtents.z));
        const real distSq = (p - q).LengthSq();

        if (bestDistSq < real(0) || distSq < bestDistSq) {
            bestDistSq = distSq;
            bestT = t;
        }
    }

    outT = bestT;
    outSegPoint = a + d * bestT;
    outBoxPoint = Vec3(Clamp(outSegPoint.x, -halfExtents.x, halfExtents.x),
                       Clamp(outSegPoint.y, -halfExtents.y, halfExtents.y),
                       Clamp(outSegPoint.z, -halfExtents.z, halfExtents.z));
    return bestDistSq < real(0) ? real(0) : bestDistSq;
}

//------------------------------------------------------------------------------
// 三角形 / 平面上的常用量（M4 的面裁剪会用到，先放这里）
//------------------------------------------------------------------------------

/// 点 p 到平面（法线 n、过点 origin）的有符号距离。
/// 正值表示 p 在法线指向的一侧。n 必须是单位向量。
inline real SignedDistanceToPlane(const Vec3& p, const Vec3& planeOrigin,
                                  const Vec3& n) noexcept {
    return Dot(p - planeOrigin, n);
}

/// 把点投影到平面上。
inline Vec3 ProjectPointOnPlane(const Vec3& p, const Vec3& planeOrigin,
                                const Vec3& n) noexcept {
    return p - n * SignedDistanceToPlane(p, planeOrigin, n);
}

}  // namespace pe
