//==============================================================================
// src/collision/EPA.cpp
//
// EPA 的实现。原理见 EPA.h。
//
// 全部用**固定大小的数组**，不做任何堆分配：EPA 虽然不在最热的路径上，但它会
// 在碰撞发生的那一帧被调用，而"偶尔一帧多几次 malloc"正是帧时间尖刺的经典来源。
// 容量用满时不崩，而是返回当前最好的那个面 —— 精度差一点点，总好过失败。
//==============================================================================

#include "pe/collision/EPA.h"

namespace pe {

namespace {

//------------------------------------------------------------------------------
// 容量
//
// 每轮迭代加 1 个顶点，凸多面体的面数满足 F = 2V - 4，所以 64 个顶点对应
// 124 个面。迭代上限是 48，正常情况下连一半都用不到。
//------------------------------------------------------------------------------
constexpr int kMaxVertices = 64;
constexpr int kMaxFaces = 128;
constexpr int kMaxHorizonEdges = 64;

struct Face {
    int v[3];      ///< 三个顶点在顶点数组里的下标
    Vec3 normal;   ///< 朝外的单位法线（背离原点）
    real distance; ///< 原点到这个面所在平面的距离（因为法线朝外，恒 >= 0）
    bool alive;
};

struct Polytope {
    Vec3 w[kMaxVertices];   ///< 闵可夫斯基差上的点
    Vec3 pa[kMaxVertices];  ///< 对应 A 的核上的支撑点
    Vec3 pb[kMaxVertices];
    int vertexCount = 0;

    Face faces[kMaxFaces];
    int faceCount = 0;

    int AddVertex(const Vec3& wp, const Vec3& pap, const Vec3& pbp) noexcept {
        if (vertexCount >= kMaxVertices) return -1;
        const int i = vertexCount++;
        w[i] = wp;
        pa[i] = pap;
        pb[i] = pbp;
        return i;
    }

    /// 加一个三角面，并把法线定向成**背离原点**。
    ///
    /// 定向这一步是 EPA 的命门：法线朝里的话，"离原点最近的面"会被算成负距离，
    /// 扩张方向也会反过来，多面体会朝内塌陷而不是向外长。
    /// 这里不依赖顶点的绕向，而是直接用"原点在平面的哪一侧"来定向，
    /// 因为原点在多面体内部这一点是有保证的。
    bool AddFace(int i0, int i1, int i2) noexcept {
        if (faceCount >= kMaxFaces) return false;

        const Vec3 a = w[i0];
        Vec3 n = Cross(w[i1] - a, w[i2] - a);
        const real lenSq = n.LengthSq();

        // 退化三角形（三点共线）：面积为零，法线没有意义，直接丢掉。
        // 留着它会让"最近面"选中一个法线是噪声的面。
        if (lenSq <= real(1e-18)) return false;

        n *= real(1) / Sqrt(lenSq);
        real d = Dot(n, a);
        if (d < real(0)) {
            n = -n;
            d = -d;
        }

        Face& f = faces[faceCount++];
        f.v[0] = i0;
        f.v[1] = i1;
        f.v[2] = i2;
        f.normal = n;
        f.distance = d;
        f.alive = true;
        return true;
    }
};

//------------------------------------------------------------------------------
// 把不足 4 个顶点的单纯形撑成四面体
//
// GJK 判定相交时，单纯形不一定是完整的四面体：原点可能恰好落在一条边或一个面上
// （两个盒子面贴面就是这种情况，非常常见）。EPA 必须从一个有体积的多面体出发，
// 所以要先补点。
//
// 补法：朝一个"当前单纯形张不出来"的方向取支撑点。
//   - 1 个点：依次试 ±X ±Y ±Z 六个方向
//   - 2 个点：在与线段垂直的平面里试若干方向
//   - 3 个点：沿三角形法线的正反两个方向
//------------------------------------------------------------------------------
bool ExpandToTetrahedron(const ConvexProxy& a, const ConvexProxy& b,
                         Polytope& poly) noexcept {
    const auto supportDiff = [&](const Vec3& dir, Vec3& outW, Vec3& outPa,
                                 Vec3& outPb) noexcept {
        outPa = a.SupportCore(dir);
        outPb = b.SupportCore(-dir);
        outW = outPa - outPb;
    };

    /// 朝 dir 取支撑点，若它与已有顶点都不重合就加进去
    const auto tryAdd = [&](const Vec3& dir) noexcept -> bool {
        Vec3 wp, pap, pbp;
        supportDiff(dir, wp, pap, pbp);
        for (int i = 0; i < poly.vertexCount; ++i) {
            if ((poly.w[i] - wp).LengthSq() <= real(1e-12)) return false;
        }
        return poly.AddVertex(wp, pap, pbp) >= 0;
    };

    // 1 -> 2
    if (poly.vertexCount == 1) {
        const Vec3 dirs[6] = {Vec3::UnitX(),  Vec3::UnitY(),  Vec3::UnitZ(),
                              -Vec3::UnitX(), -Vec3::UnitY(), -Vec3::UnitZ()};
        for (int i = 0; i < 6 && poly.vertexCount < 2; ++i) tryAdd(dirs[i]);
        if (poly.vertexCount < 2) return false;
    }

    // 2 -> 3：在垂直于线段的平面里找
    if (poly.vertexCount == 2) {
        const Vec3 axis = poly.w[1] - poly.w[0];
        Vec3 t1, t2;
        const Vec3 unitAxis = axis.Normalized();
        if (unitAxis.IsZero()) return false;
        BuildOrthonormalBasis(unitAxis, t1, t2);

        const Vec3 dirs[4] = {t1, -t1, t2, -t2};
        for (int i = 0; i < 4 && poly.vertexCount < 3; ++i) tryAdd(dirs[i]);
        if (poly.vertexCount < 3) return false;
    }

    // 3 -> 4：沿三角形法线的两侧找
    if (poly.vertexCount == 3) {
        const Vec3 n = Cross(poly.w[1] - poly.w[0], poly.w[2] - poly.w[0]).Normalized();
        if (n.IsZero()) return false;
        if (poly.vertexCount < 4) tryAdd(n);
        if (poly.vertexCount < 4) tryAdd(-n);
        if (poly.vertexCount < 4) return false;
    }

    return poly.vertexCount >= 4;
}

}  // namespace

//==============================================================================
// EPA
//==============================================================================

EpaResult Epa(const ConvexProxy& a, const ConvexProxy& b,
              const GjkSimplex& simplex) noexcept {
    EpaResult result;
    result.valid = false;
    result.normal = Vec3::UnitY();
    result.depth = real(0);
    result.pointA = Vec3::Zero();
    result.pointB = Vec3::Zero();

    Polytope poly;
    for (int i = 0; i < simplex.count && i < 4; ++i) {
        poly.AddVertex(simplex.w[i], simplex.pa[i], simplex.pb[i]);
    }

    if (!ExpandToTetrahedron(a, b, poly)) {
        //----------------------------------------------------------------------
        // 撑不成四面体，说明闵可夫斯基差**根本没有体积**。
        //
        // 这不是异常，而是球/胶囊必然会走到的路径（它们的核是点和线段）：
        //     球 ⊖ 球     = 一个点
        //     球 ⊖ 胶囊   = 一条线段
        //     胶囊 ⊖ 胶囊 = 一个平行四边形
        // 原点落在这些低维集合上，意味着两个**核**恰好接触，核的穿透深度就是 0
        // —— 真正的穿透全部来自半径，由调用方加上。
        //
        // 唯一还需要给出的是方向：必须挑一个**离开这个低维集合**的方向，
        // 沿着它推开才是最省力的。
        //   - 差是一个平行四边形（3 个顶点）-> 取它的法线
        //   - 差是一条线段（2 个顶点）      -> 取任意垂直于它的方向
        //   - 差是一个点（1 个顶点）        -> 任意方向都行
        // 这正是解析解里 AnyPerpendicular / Cross 那几个兜底方向在做的事，
        // 只不过这里是从单纯形的维数反推出来的。
        //----------------------------------------------------------------------
        // 注意**不能**拿顶点个数当维数用：ExpandToTetrahedron 在失败之前可能
        // 已经塞进来几个共线的点（两条共线胶囊就是这样，三个顶点全在一条线上）。
        // 必须从点集本身量出它真正张成了几维。
        Vec3 dir = Vec3::Zero();

        // 最长的一条边定出主轴
        int i0 = 0;
        int i1 = -1;
        real bestLenSq = real(0);
        for (int i = 0; i < poly.vertexCount; ++i) {
            for (int j = i + 1; j < poly.vertexCount; ++j) {
                const real lenSq = (poly.w[i] - poly.w[j]).LengthSq();
                if (lenSq > bestLenSq) {
                    bestLenSq = lenSq;
                    i0 = i;
                    i1 = j;
                }
            }
        }

        if (i1 >= 0 && bestLenSq > kNormalizeEpsilonSq) {
            const Vec3 axis = (poly.w[i1] - poly.w[i0]) * (real(1) / Sqrt(bestLenSq));

            // 离这条轴最远的点：够远就说明点集是二维的（一个平行四边形）
            Vec3 bestPerp = Vec3::Zero();
            real bestPerpLen = real(0);
            for (int k = 0; k < poly.vertexCount; ++k) {
                const Vec3 d = poly.w[k] - poly.w[i0];
                const Vec3 perp = d - axis * Dot(d, axis);
                const real len = perp.Length();
                if (len > bestPerpLen) {
                    bestPerpLen = len;
                    bestPerp = perp;
                }
            }

            // 相对判据：垂直分量相对主轴长度可以忽略，就认为是一维
            if (bestPerpLen > real(1e-4) * Sqrt(bestLenSq)) {
                dir = Cross(axis, bestPerp).Normalized();  // 二维：取平面法线
            } else {
                Vec3 t1, t2;
                BuildOrthonormalBasis(axis, t1, t2);
                dir = t1;  // 一维：取任意垂直于轴线的方向
            }
        }

        if (dir.IsZero()) dir = Vec3::UnitY();  // 零维：任何方向都同样正确

        // 定向：让法线大致由 A 指向 B。这样交换 A、B 时法线会自动取反
        // （两个中心恰好重合时无法判断，那种构型下任何方向都同样正确）。
        const Vec3 centerOffset = b.transform.position - a.transform.position;
        if (Dot(dir, centerOffset) < real(0)) dir = -dir;

        // 见证点取所有支撑点的平均。对"两条共线的胶囊部分重叠"这类情形，
        // 支撑点是两端的极值，平均下来正好落在重叠区间的中间。
        Vec3 avgA = Vec3::Zero();
        Vec3 avgB = Vec3::Zero();
        for (int i = 0; i < poly.vertexCount; ++i) {
            avgA += poly.pa[i];
            avgB += poly.pb[i];
        }
        if (poly.vertexCount > 0) {
            const real inv = real(1) / static_cast<real>(poly.vertexCount);
            avgA *= inv;
            avgB *= inv;
        }

        result.valid = true;
        result.normal = dir;
        result.depth = real(0);
        result.pointA = avgA;
        result.pointB = avgB;
        return result;
    }

    // 初始四面体的四个面
    poly.AddFace(0, 1, 2);
    poly.AddFace(0, 2, 3);
    poly.AddFace(0, 3, 1);
    poly.AddFace(1, 3, 2);
    if (poly.faceCount < 4) return result;

    int bestFace = -1;

    for (int iter = 0; iter < kEpaMaxIterations; ++iter) {
        //----------------------------------------------------------------------
        // 1. 找离原点最近的活面
        //----------------------------------------------------------------------
        bestFace = -1;
        real bestDistance = real(0);
        for (int i = 0; i < poly.faceCount; ++i) {
            if (!poly.faces[i].alive) continue;
            if (bestFace < 0 || poly.faces[i].distance < bestDistance) {
                bestFace = i;
                bestDistance = poly.faces[i].distance;
            }
        }
        if (bestFace < 0) return result;

        //----------------------------------------------------------------------
        // 2. 朝这个面的法线取新的支撑点
        //----------------------------------------------------------------------
        const Vec3 dir = poly.faces[bestFace].normal;
        const Vec3 pap = a.SupportCore(dir);
        const Vec3 pbp = b.SupportCore(-dir);
        const Vec3 wp = pap - pbp;

        //----------------------------------------------------------------------
        // 3. 收敛判定：新点在法线方向上没能比这个面更外面，说明面已经贴在
        //    闵可夫斯基差的边界上了
        //----------------------------------------------------------------------
        if (Dot(wp, dir) - bestDistance < kEpaTolerance) break;

        //----------------------------------------------------------------------
        // 4. 删掉所有"能看见新点"的面，收集地平线
        //
        // 一条边如果只被一个待删的面用到，它就在地平线上；被两个待删的面共用的
        // 边则在被删区域的内部，不属于地平线。所以做法是：把所有待删面的边都丢进
        // 一个列表，出现两次的成对抵消，剩下的就是地平线。
        //----------------------------------------------------------------------
        int horizon[kMaxHorizonEdges][2];
        int horizonCount = 0;
        bool overflow = false;

        for (int i = 0; i < poly.faceCount; ++i) {
            Face& f = poly.faces[i];
            if (!f.alive) continue;

            // 新点在这个面的外侧 => 这个面被"看见"，要删
            if (Dot(f.normal, wp) - f.distance <= real(0)) continue;
            f.alive = false;

            for (int e = 0; e < 3; ++e) {
                const int e0 = f.v[e];
                const int e1 = f.v[(e + 1) % 3];

                // 找反向的同一条边，有就抵消
                bool cancelled = false;
                for (int h = 0; h < horizonCount; ++h) {
                    if (horizon[h][0] == e1 && horizon[h][1] == e0) {
                        horizon[h][0] = horizon[horizonCount - 1][0];
                        horizon[h][1] = horizon[horizonCount - 1][1];
                        --horizonCount;
                        cancelled = true;
                        break;
                    }
                }
                if (cancelled) continue;

                if (horizonCount >= kMaxHorizonEdges) {
                    overflow = true;
                    break;
                }
                horizon[horizonCount][0] = e0;
                horizon[horizonCount][1] = e1;
                ++horizonCount;
            }
            if (overflow) break;
        }

        if (overflow || horizonCount < 3) break;  // 拿当前最好的面收工

        //----------------------------------------------------------------------
        // 5. 把新点和地平线上每条边连成新的面
        //----------------------------------------------------------------------
        const int newIndex = poly.AddVertex(wp, pap, pbp);
        if (newIndex < 0) break;  // 顶点用满了，收工

        // 先把死面压实，给新面腾地方（不做的话 faceCount 会一直涨到上限）
        int write = 0;
        for (int i = 0; i < poly.faceCount; ++i) {
            if (poly.faces[i].alive) poly.faces[write++] = poly.faces[i];
        }
        poly.faceCount = write;

        bool addFailed = false;
        for (int h = 0; h < horizonCount; ++h) {
            if (!poly.AddFace(horizon[h][0], horizon[h][1], newIndex)) {
                // 面数用满或退化三角形。退化的那个可以跳过，用满了就得停。
                if (poly.faceCount >= kMaxFaces) {
                    addFailed = true;
                    break;
                }
            }
        }
        if (addFailed) break;
    }

    if (bestFace < 0 || !poly.faces[bestFace].alive) {
        // 最后一轮把最好的面删掉了却没能补上新的，退而求其次找一个活面
        bestFace = -1;
        real bestDistance = real(0);
        for (int i = 0; i < poly.faceCount; ++i) {
            if (!poly.faces[i].alive) continue;
            if (bestFace < 0 || poly.faces[i].distance < bestDistance) {
                bestFace = i;
                bestDistance = poly.faces[i].distance;
            }
        }
        if (bestFace < 0) return result;
    }

    const Face& f = poly.faces[bestFace];

    //--------------------------------------------------------------------------
    // 把"原点在最近面上的投影"用重心坐标反算回 A、B 上的见证点。
    //
    // 投影点 p = normal * distance。求它在三角形 (w0,w1,w2) 内的重心坐标，
    // 再用同一组权重去加权 pa / pb —— 因为 w = pa - pb 是线性关系，
    // 权重可以原样搬过去。
    //--------------------------------------------------------------------------
    const Vec3 p = f.normal * f.distance;
    const Vec3& w0 = poly.w[f.v[0]];
    const Vec3& w1 = poly.w[f.v[1]];
    const Vec3& w2 = poly.w[f.v[2]];

    const Vec3 e1 = w1 - w0;
    const Vec3 e2 = w2 - w0;
    const Vec3 e0 = p - w0;

    const real d11 = Dot(e1, e1);
    const real d12 = Dot(e1, e2);
    const real d22 = Dot(e2, e2);
    const real d01 = Dot(e0, e1);
    const real d02 = Dot(e0, e2);
    const real denom = d11 * d22 - d12 * d12;

    real l0 = real(1);
    real l1 = real(0);
    real l2 = real(0);
    if (Abs(denom) > real(1e-12)) {
        l1 = (d22 * d01 - d12 * d02) / denom;
        l2 = (d11 * d02 - d12 * d01) / denom;
        l0 = real(1) - l1 - l2;

        // 数值噪声可能把权重顶到区间外一点点，夹回去并重新归一化。
        // 不夹的话见证点会跑到三角形外面，接触点位置就飘了。
        l0 = Clamp(l0, real(0), real(1));
        l1 = Clamp(l1, real(0), real(1));
        l2 = Clamp(l2, real(0), real(1));
        const real sum = l0 + l1 + l2;
        if (sum > kEpsilon) {
            const real inv = real(1) / sum;
            l0 *= inv;
            l1 *= inv;
            l2 *= inv;
        } else {
            l0 = real(1);
            l1 = real(0);
            l2 = real(0);
        }
    }

    result.valid = true;
    result.normal = f.normal;
    result.depth = f.distance;
    result.pointA =
        poly.pa[f.v[0]] * l0 + poly.pa[f.v[1]] * l1 + poly.pa[f.v[2]] * l2;
    result.pointB =
        poly.pb[f.v[0]] * l0 + poly.pb[f.v[1]] * l1 + poly.pb[f.v[2]] * l2;
    return result;
}

}  // namespace pe
