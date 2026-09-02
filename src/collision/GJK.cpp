//==============================================================================
// src/collision/GJK.cpp
//
// GJK 的实现。原理见 GJK.h，这里只写代码层面的取舍。
//
// 结构：
//   1. 支撑函数（每种形状的核）
//   2. 单纯形上的最近点 —— 点 / 线段 / 三角形 / 四面体四种情形
//   3. 主循环
//
// 第 2 部分是全部的复杂度所在，而且它是纯几何、可以脱离 GJK 单独验证的，
// 所以测试里对它有专门的用例。
//==============================================================================

#include "pe/collision/GJK.h"

#include "pe/math/Mat3.h"

namespace pe {

//==============================================================================
// 支撑函数
//==============================================================================

Vec3 ConvexProxy::SupportCore(const Vec3& dir) const noexcept {
    switch (shape.type) {
        case ShapeType::Sphere:
            // 球的核是一个点：不管朝哪个方向，支撑点都是球心
            return transform.position;

        case ShapeType::Capsule: {
            // 胶囊的核是一条线段：取投影更大的那个端点
            const Vec3 axis = transform.TransformDirection(Vec3::UnitY());
            const real h = shape.capsule.halfHeight;
            return transform.position + axis * (Dot(dir, axis) >= real(0) ? h : -h);
        }

        case ShapeType::Box: {
            // 盒子的核就是盒子本身：每根轴独立取符号，得到 8 个角里最"迎着" dir 的那个。
            // 推导见 NarrowPhase.cpp 的 BoxProjectedRadius —— 同一个道理：
            // 各分量可以独立取极值，所以逐轴贪心就是全局最优。
            const Mat3 r = transform.rotation.ToMat3();
            Vec3 p = transform.position;
            for (int i = 0; i < 3; ++i) {
                const Vec3 axis = r.Column(i);
                p += axis * (Dot(dir, axis) >= real(0) ? shape.box.halfExtents[i]
                                                       : -shape.box.halfExtents[i]);
            }
            return p;
        }
    }
    return transform.position;
}

real ConvexProxy::CoreRadius() const noexcept {
    switch (shape.type) {
        case ShapeType::Sphere: return shape.sphere.radius;
        case ShapeType::Capsule: return shape.capsule.radius;
        case ShapeType::Box: return real(0);
    }
    return real(0);
}

namespace {

//==============================================================================
// 单纯形上离原点最近的点
//==============================================================================
//
// 每个函数都返回最近点，并通过 outBary 给出各顶点的重心权重、通过 outMask 标出
// 哪些顶点对这个最近点有贡献（低位对应第 0 个顶点）。
//
// **约简**（丢掉 mask 里没标的顶点）不是可选的优化，而是算法正确性的一部分：
// 不丢的话单纯形会一直是 4 个点，新的支撑点没地方放，迭代就卡死了。
//
// 全部特化成"到原点"的版本（而不是通用的"到点 p"），因为 GJK 只关心原点，
// 特化之后能省掉一堆减法，可读性也更好 —— 少一个到处出现的 p。
//------------------------------------------------------------------------------

/// 线段 [a,b] 上离原点最近的点。
Vec3 ClosestOnSegment(const Vec3& a, const Vec3& b, real outBary[4],
                      int& outMask) noexcept {
    const Vec3 ab = b - a;
    const real denom = ab.LengthSq();

    // t = Dot(-a, ab) / |ab|^2，即把原点投影到直线 ab 上
    const real t = (denom > kNormalizeEpsilonSq) ? (-Dot(a, ab) / denom) : real(0);

    if (t <= real(0)) {
        outBary[0] = real(1);
        outMask = 0b0001;
        return a;
    }
    if (t >= real(1)) {
        outBary[1] = real(1);
        outMask = 0b0010;
        return b;
    }
    outBary[0] = real(1) - t;
    outBary[1] = t;
    outMask = 0b0011;
    return a + ab * t;
}

/// 三角形 [a,b,c] 上离原点最近的点。
///
/// 这是 Ericson《Real-Time Collision Detection》5.1.5 的 Voronoi 区域判定，
/// 特化到 p = 原点。逐个排除：三个顶点区域 -> 三条边区域 -> 面内部。
/// 顺序不能乱，后面的判据依赖前面的已经被排除。
Vec3 ClosestOnTriangle(const Vec3& a, const Vec3& b, const Vec3& c, real outBary[4],
                       int& outMask) noexcept {
    const Vec3 ab = b - a;
    const Vec3 ac = c - a;

    // 顶点 a 的区域
    const real d1 = Dot(ab, -a);
    const real d2 = Dot(ac, -a);
    if (d1 <= real(0) && d2 <= real(0)) {
        outBary[0] = real(1);
        outMask = 0b0001;
        return a;
    }

    // 顶点 b 的区域
    const real d3 = Dot(ab, -b);
    const real d4 = Dot(ac, -b);
    if (d3 >= real(0) && d4 <= d3) {
        outBary[1] = real(1);
        outMask = 0b0010;
        return b;
    }

    // 边 ab 的区域
    const real vc = d1 * d4 - d3 * d2;
    if (vc <= real(0) && d1 >= real(0) && d3 <= real(0)) {
        const real denom = d1 - d3;
        const real v = (Abs(denom) > kEpsilon) ? d1 / denom : real(0);
        outBary[0] = real(1) - v;
        outBary[1] = v;
        outMask = 0b0011;
        return a + ab * v;
    }

    // 顶点 c 的区域
    const real d5 = Dot(ab, -c);
    const real d6 = Dot(ac, -c);
    if (d6 >= real(0) && d5 <= d6) {
        outBary[2] = real(1);
        outMask = 0b0100;
        return c;
    }

    // 边 ac 的区域
    const real vb = d5 * d2 - d1 * d6;
    if (vb <= real(0) && d2 >= real(0) && d6 <= real(0)) {
        const real denom = d2 - d6;
        const real w = (Abs(denom) > kEpsilon) ? d2 / denom : real(0);
        outBary[0] = real(1) - w;
        outBary[2] = w;
        outMask = 0b0101;
        return a + ac * w;
    }

    // 边 bc 的区域
    const real va = d3 * d6 - d5 * d4;
    if (va <= real(0) && (d4 - d3) >= real(0) && (d5 - d6) >= real(0)) {
        const real denom = (d4 - d3) + (d5 - d6);
        const real w = (Abs(denom) > kEpsilon) ? (d4 - d3) / denom : real(0);
        outBary[1] = real(1) - w;
        outBary[2] = w;
        outMask = 0b0110;
        return b + (c - b) * w;
    }

    // 剩下的只能是面内部
    const real denom = va + vb + vc;
    if (Abs(denom) <= kEpsilon) {
        // 退化成一条线（三点共线）。交给线段版本处理，免得除以 0。
        return ClosestOnSegment(a, b, outBary, outMask);
    }
    const real inv = real(1) / denom;
    const real v = vb * inv;
    const real w = vc * inv;
    outBary[0] = real(1) - v - w;
    outBary[1] = v;
    outBary[2] = w;
    outMask = 0b0111;
    return a + ab * v + ac * w;
}

/// 原点是不是在平面 abc 的"远离 d"那一侧。
/// 用第四个顶点 d 来定向平面，避免依赖 a、b、c 的绕向。
bool OriginOutsideFace(const Vec3& a, const Vec3& b, const Vec3& c,
                       const Vec3& d) noexcept {
    const Vec3 n = Cross(b - a, c - a);
    const real signOrigin = Dot(-a, n);  // 原点相对平面的有符号量
    const real signD = Dot(d - a, n);    // 第四个顶点相对平面的有符号量

    // 严格反号才算"在外面"。共面（signOrigin 恰为 0）算在里面，
    // 这样原点落在四面体表面上时会被判成相交 —— 和引擎其它地方"贴面算相交"
    // 的闭区间约定一致（见 AABB.h）。
    return signOrigin * signD < real(0);
}

/// 四面体 [a,b,c,d] 上离原点最近的点。原点在内部时返回原点本身、mask 为全 1。
///
//------------------------------------------------------------------------------
// 扁四面体的陷阱
//------------------------------------------------------------------------------
// 四个顶点近乎共面时，每个面用来定向的"第四个顶点"也几乎落在这个面上，于是
// OriginOutsideFace 里的 signD 趋于 0，四个面就全都判不出"原点在外面"——
// 于是算法会得出"原点在内部"这个结论，也就是**误报相交**。
//
// 这不是罕见的角落情况，而是必然会发生的：
// 球的核是一个点、胶囊的核是一条线段，所以
//     球 ⊖ 球     = 一个点（0 维）
//     球 ⊖ 胶囊   = 一条线段（1 维）
//     胶囊 ⊖ 胶囊 = 一个平行四边形（2 维）
// 这些闵可夫斯基差**根本没有体积**，任何 4 个顶点都必然共面。
// 少了下面这个体积判据，所有球/胶囊之间的查询都会被误判成相交。
//
// 所以先算有向体积、拿三条棱长做无量纲化，扁到一定程度就不走"内部"这条路，
// 而是老老实实在四个面上取最近点。
//------------------------------------------------------------------------------
Vec3 ClosestOnTetrahedron(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d,
                          real outBary[4], int& outMask) noexcept {
    const Vec3* verts[4] = {&a, &b, &c, &d};

    // 四个面，每个面记下它用的是哪三个顶点（下标）和用来定向的第四个顶点
    static const int faces[4][3] = {{0, 1, 2}, {0, 2, 3}, {0, 3, 1}, {1, 3, 2}};
    static const int opposite[4] = {3, 1, 2, 0};

    // 六倍有向体积，用三条棱长归一化成无量纲量再比阈值 ——
    // 直接和绝对值比会让"小而正常"的四面体被误判成扁的。
    const Vec3 ab = b - a;
    const Vec3 ac = c - a;
    const Vec3 ad = d - a;
    const real volume6 = Dot(Cross(ab, ac), ad);
    const real scale = ab.Length() * ac.Length() * ad.Length();
    const bool flat = Abs(volume6) <= real(1e-6) * Max(scale, kEpsilon);

    Vec3 best = Vec3::Zero();
    real bestDistSq = real(-1);
    int bestMask = 0;
    real bestBary[4] = {real(0), real(0), real(0), real(0)};

    for (int f = 0; f < 4; ++f) {
        const int i0 = faces[f][0];
        const int i1 = faces[f][1];
        const int i2 = faces[f][2];

        // 扁四面体上跳过"原点在不在这个面外侧"的判定，四个面全都参与比较。
        if (!flat && !OriginOutsideFace(*verts[i0], *verts[i1], *verts[i2],
                                        *verts[opposite[f]])) {
            continue;
        }

        real bary[4] = {real(0), real(0), real(0), real(0)};
        int mask = 0;
        const Vec3 p =
            ClosestOnTriangle(*verts[i0], *verts[i1], *verts[i2], bary, mask);
        const real distSq = p.LengthSq();

        if (bestDistSq < real(0) || distSq < bestDistSq) {
            bestDistSq = distSq;
            best = p;

            // 三角形版本的 bary/mask 用的是"面内的 0/1/2"编号，
            // 这里要翻译回四面体的顶点编号
            bestMask = 0;
            bestBary[0] = bestBary[1] = bestBary[2] = bestBary[3] = real(0);
            const int idx[3] = {i0, i1, i2};
            for (int k = 0; k < 3; ++k) {
                if (mask & (1 << k)) {
                    bestMask |= (1 << idx[k]);
                    bestBary[idx[k]] = bary[k];
                }
            }
        }
    }

    if (bestDistSq < real(0)) {
        // 四个面都没把原点排在外面 => 原点在四面体内部 => 相交
        // （扁四面体永远走不到这里：上面把所有面都算了一遍，bestDistSq 一定被填过）
        outMask = 0b1111;
        outBary[0] = outBary[1] = outBary[2] = outBary[3] = real(0.25);
        return Vec3::Zero();
    }

    outMask = bestMask;
    for (int i = 0; i < 4; ++i) outBary[i] = bestBary[i];
    return best;
}

}  // namespace

//==============================================================================
// 主循环
//==============================================================================

GjkResult Gjk(const ConvexProxy& a, const ConvexProxy& b) noexcept {
    GjkResult result;
    result.status = GjkStatus::Separated;
    result.distance = real(0);
    result.pointA = Vec3::Zero();
    result.pointB = Vec3::Zero();
    result.simplex.count = 0;
    result.iterations = 0;

    // 闵可夫斯基差上朝 dir 的支撑点，同时记下它来自 A、B 上的哪两个点
    const auto supportDiff = [&](const Vec3& dir, Vec3& outW, Vec3& outPa,
                                 Vec3& outPb) noexcept {
        outPa = a.SupportCore(dir);
        outPb = b.SupportCore(-dir);
        outW = outPa - outPb;
    };

    // 初始方向：两个形状中心的连线。任何方向都能收敛，但这个方向通常一两步就够。
    Vec3 dir = b.transform.position - a.transform.position;
    if (dir.IsZero()) dir = Vec3::UnitX();

    GjkSimplex& sx = result.simplex;
    supportDiff(dir, sx.w[0], sx.pa[0], sx.pb[0]);
    sx.count = 1;

    Vec3 v = sx.w[0];  // 当前单纯形上离原点最近的点
    real bary[4] = {real(1), real(0), real(0), real(0)};

    // |v| 在精确算术下是单调不增的。浮点世界里它可能停滞甚至微微反弹，
    // 那就说明已经到精度极限了 —— 再迭代只会在几个顶点之间来回打转，
    // 直到把迭代次数耗光。记住历史最好值并在不再改善时收手。
    real bestVLenSq = v.LengthSq();

    for (int iter = 0; iter < kGjkMaxIterations; ++iter) {
        result.iterations = iter + 1;

        const real vLenSq = v.LengthSq();

        // 最近点已经是原点：核相交
        if (vLenSq <= kNormalizeEpsilonSq) {
            result.status = GjkStatus::Intersecting;
            result.distance = real(0);
            return result;
        }

        // 朝原点方向（也就是 -v）找新的支撑点
        Vec3 w, pa, pb;
        supportDiff(-v, w, pa, pb);

        //----------------------------------------------------------------------
        // 终止判据。
        //
        // Dot(v, v) - Dot(w, v) 是"新支撑点相对当前最近点，在搜索方向上还能
        // 前进多少"。它 <= 0 意味着整个 D 都在过 v、法线为 v 的支撑平面的
        // 另一侧，于是 v 就是全局最近点，|v| 就是距离。
        //
        // 判据写成相对形式（右边乘了 vLenSq）而非绝对阈值：这样在毫米和百米
        // 两种尺度下的行为一致。绝对阈值在大坐标下会过早退出、在小坐标下
        // 会白白多迭代几轮。
        //----------------------------------------------------------------------
        if (vLenSq - Dot(w, v) <= kGjkTolerance * vLenSq) {
            break;
        }

        // 重复顶点：说明支撑函数已经给不出新东西了，再迭代也是原地打转
        bool duplicate = false;
        for (int i = 0; i < sx.count; ++i) {
            if ((sx.w[i] - w).LengthSq() <= kNormalizeEpsilonSq) {
                duplicate = true;
                break;
            }
        }
        if (duplicate) break;

        // 加入单纯形
        sx.w[sx.count] = w;
        sx.pa[sx.count] = pa;
        sx.pb[sx.count] = pb;
        ++sx.count;

        // 求新的最近点并约简
        real newBary[4] = {real(0), real(0), real(0), real(0)};
        int mask = 0;
        Vec3 newV;

        switch (sx.count) {
            case 1:
                newV = sx.w[0];
                newBary[0] = real(1);
                mask = 0b0001;
                break;
            case 2: newV = ClosestOnSegment(sx.w[0], sx.w[1], newBary, mask); break;
            case 3:
                newV = ClosestOnTriangle(sx.w[0], sx.w[1], sx.w[2], newBary, mask);
                break;
            default:
                newV = ClosestOnTetrahedron(sx.w[0], sx.w[1], sx.w[2], sx.w[3], newBary,
                                            mask);
                break;
        }

        // 四面体把原点包住了 -> 相交。此时**不要**约简单纯形：
        // EPA 需要一个完整的四面体作为初始多面体。
        if (mask == 0b1111 && sx.count == 4) {
            result.status = GjkStatus::Intersecting;
            result.distance = real(0);
            return result;
        }

        // 约简：只留下对最近点有贡献的顶点
        GjkSimplex reduced;
        reduced.count = 0;
        real reducedBary[4] = {real(0), real(0), real(0), real(0)};
        for (int i = 0; i < sx.count; ++i) {
            if ((mask & (1 << i)) == 0) continue;
            reduced.w[reduced.count] = sx.w[i];
            reduced.pa[reduced.count] = sx.pa[i];
            reduced.pb[reduced.count] = sx.pb[i];
            reducedBary[reduced.count] = newBary[i];
            ++reduced.count;
        }

        if (reduced.count == 0) {
            // 不该发生；真发生了就当作相交处理，交给上层的兜底路径
            result.status = GjkStatus::Intersecting;
            result.distance = real(0);
            return result;
        }

        sx = reduced;
        for (int i = 0; i < 4; ++i) bary[i] = reducedBary[i];
        v = newV;

        // 没有实质改善就收手（见上面 bestVLenSq 的说明）。
        // 用相对判据：改善量不到千万分之一就算没动。
        const real newLenSq = v.LengthSq();
        if (newLenSq >= bestVLenSq * (real(1) - kGjkTolerance)) break;
        bestVLenSq = newLenSq;
    }

    if (result.iterations >= kGjkMaxIterations) {
        result.status = GjkStatus::MaxIterations;
    }

    //--------------------------------------------------------------------------
    // 用重心坐标把最近点反算回两个形状上的见证点。
    //
    // 这正是当初在单纯形里同时保存 pa、pb 的原因：闵可夫斯基差上的点没有
    // "位置"的意义，只有把权重作用回原来的支撑点，才能得到真实空间里的坐标。
    //--------------------------------------------------------------------------
    Vec3 pointA = Vec3::Zero();
    Vec3 pointB = Vec3::Zero();
    real weightSum = real(0);
    for (int i = 0; i < sx.count; ++i) {
        pointA += sx.pa[i] * bary[i];
        pointB += sx.pb[i] * bary[i];
        weightSum += bary[i];
    }
    if (weightSum > kEpsilon) {
        const real inv = real(1) / weightSum;
        pointA *= inv;
        pointB *= inv;
    } else {
        pointA = sx.pa[0];
        pointB = sx.pb[0];
    }

    result.pointA = pointA;
    result.pointB = pointB;
    result.distance = v.Length();
    if (result.status != GjkStatus::MaxIterations) {
        result.status = GjkStatus::Separated;
    }
    return result;
}

}  // namespace pe
