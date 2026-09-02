//==============================================================================
// src/collision/NarrowPhase.cpp
//
// 窄相位的解析闭式解。设计取舍见 NarrowPhase.h，这里写算法推导。
//
// 通读顺序建议：先看 SphereVsSphereCore —— 六个形状对里有三个（球-球、球-胶囊、
// 胶囊-胶囊）在化简之后就是它，剩下三个和盒子有关的才是真正需要动脑的部分。
//==============================================================================

#include "pe/collision/NarrowPhase.h"

#include "pe/collision/EPA.h"
#include "pe/collision/GJK.h"
#include "pe/collision/GeometryUtil.h"
#include "pe/math/Mat3.h"

namespace pe {

namespace {

//==============================================================================
// 通用小工具
//==============================================================================

/// 与 v 正交的任意单位向量。退化情况下（两个形状的中心重合）用来兜一个方向出来，
/// 免得法线变成零向量或者 NaN。
Vec3 AnyPerpendicular(const Vec3& v) noexcept {
    Vec3 t1, t2;
    const Vec3 n = v.Normalized();
    if (n.IsZero()) return Vec3::UnitY();
    BuildOrthonormalBasis(n, t1, t2);
    return t1;
}

//==============================================================================
// 球 vs 球（三个形状对的公共内核）
//==============================================================================
//
// 球-胶囊、胶囊-胶囊在化简之后都是这个函数：
//   - 球 vs 胶囊 = 球 vs "以胶囊轴线上最近点为球心、半径为胶囊半径的球"
//   - 胶囊 vs 胶囊 = 两条轴线的最近点各当成一个球
// 这正是胶囊被称为"扫掠球体"（swept sphere）的原因 —— 它就是一条线段外扩半径，
// 所以只要先把"线段上哪一点参与接触"解出来，剩下的就退化成球-球。
//
// fallbackNormal 用于两个球心重合的退化情况：此时法线方向在数学上是任意的，
// 但绝不能返回零向量（求解器会当成 NaN 传播）。不同的调用者能给出不同质量的
// 兜底方向 —— 比如胶囊会给一个垂直于自己轴线的方向，因为"从侧面推出去"
// 确实是最短的逃逸路径。
//------------------------------------------------------------------------------
bool SphereVsSphereCore(const Vec3& centerA, real radiusA, const Vec3& centerB,
                        real radiusB, const Vec3& fallbackNormal,
                        Manifold& out) noexcept {
    const Vec3 delta = centerB - centerA;
    const real distSq = delta.LengthSq();
    const real radiusSum = radiusA + radiusB;
    const real limit = radiusSum + kSpeculativeMargin;

    if (distSq > limit * limit) {
        out.Clear();
        return false;
    }

    const real dist = Sqrt(distSq);

    // 法线由 A 指向 B（全引擎统一的约定，见 Manifold.h）
    const Vec3 normal =
        (dist > kEpsilon) ? delta * (real(1) / dist) : fallbackNormal;

    // 两个见证点：A 表面上最靠近 B 的点、B 表面上最靠近 A 的点。
    // 它们的间距恰好是 -penetration，所以中点到两者的距离相等。
    const Vec3 witnessA = centerA + normal * radiusA;
    const Vec3 witnessB = centerB - normal * radiusB;

    out.normal = normal;
    out.pointCount = 0;
    out.AddPoint((witnessA + witnessB) * real(0.5), radiusSum - dist,
                 MakeFeatureId(0, 0, 0, 0));
    return true;
}

//==============================================================================
// 点 vs 盒（球-盒、胶囊-盒的公共内核，全部在盒的局部空间里算）
//==============================================================================
//
// 分两种情况，而且两者在边界上是连续的：
//
// (1) 点在盒外。最近点就是把坐标逐轴钳到 [-h, h]。法线沿"点 -> 最近点"，
//     穿透深度 = 半径 - 距离。
//
// (2) 点在盒内（钳制之后没变，距离为 0）。此时"最近点"给不出方向，必须换个问法：
//     从哪个面出去最省事？逐轴算到两个面的距离，取最小的那个面。
//     法线是**指向盒内**的（因为约定是"由 A 指向 B"，A 是球、B 是盒），
//     穿透深度 = 半径 + 到那个面的距离。
//
//     容易搞反的地方：这里必须取"到最近面的距离"而不是"到最远面的距离"。
//     取错的话物体会从它陷得最深的那一侧被弹出去，表现为球穿过墙落到另一边。
//
// 边界上的连续性：点从外面逼近盒面时 dist -> 0，法线 -> 指向盒内，穿透 -> 半径；
// 而 (2) 在 depth = 0 时给出的正是同一组值。所以不会在贴面时抖动。
//------------------------------------------------------------------------------
bool PointVsBoxLocal(const Vec3& pointLocal, real radius, const Vec3& half,
                     Vec3& outNormalLocal, real& outPenetration,
                     Vec3& outBoxPointLocal) noexcept {
    const Vec3 closest(Clamp(pointLocal.x, -half.x, half.x),
                       Clamp(pointLocal.y, -half.y, half.y),
                       Clamp(pointLocal.z, -half.z, half.z));

    const Vec3 delta = closest - pointLocal;  // 由点指向盒
    const real distSq = delta.LengthSq();
    const real limit = radius + kSpeculativeMargin;

    if (distSq > limit * limit) return false;

    if (distSq > kNormalizeEpsilonSq) {
        // 情况 (1)：点在盒外
        const real dist = Sqrt(distSq);
        outNormalLocal = delta * (real(1) / dist);
        outPenetration = radius - dist;
        outBoxPointLocal = closest;
        return true;
    }

    // 情况 (2)：点在盒内，找最近的那个面
    int axis = 0;
    real minDepth = half[0] - Abs(pointLocal[0]);
    for (int i = 1; i < 3; ++i) {
        const real depth = half[i] - Abs(pointLocal[i]);
        if (depth < minDepth) {
            minDepth = depth;
            axis = i;
        }
    }

    const real sign = pointLocal[axis] >= real(0) ? real(1) : real(-1);

    // 面的朝外法线；接触法线（球 -> 盒）与它相反
    outNormalLocal = Vec3::Zero();
    outNormalLocal[axis] = -sign;
    outPenetration = radius + minDepth;

    outBoxPointLocal = pointLocal;
    outBoxPointLocal[axis] = sign * half[axis];
    return true;
}

/// 把 PointVsBoxLocal 的结果装配成世界空间的单点流形。
/// pointLocal 是"球心"（对胶囊来说是轴线上参与接触的那个点）。
void EmitPointBoxManifold(const Transform& boxTransform, const Vec3& pointLocal,
                          real radius, const Vec3& normalLocal, real penetration,
                          const Vec3& boxPointLocal, Manifold& out) noexcept {
    const Vec3 witnessSphereLocal = pointLocal + normalLocal * radius;
    const Vec3 midLocal = (witnessSphereLocal + boxPointLocal) * real(0.5);

    out.normal = boxTransform.TransformDirection(normalLocal);
    out.pointCount = 0;
    out.AddPoint(boxTransform.TransformPoint(midLocal), penetration,
                 MakeFeatureId(0, 0, 0, 0));
}

//==============================================================================
// 盒子的世界空间框架
//==============================================================================

struct BoxFrame {
    Vec3 center;
    Vec3 axis[3];  ///< 三根局部轴在世界空间的单位向量
    Vec3 half;
};

BoxFrame MakeBoxFrame(const Shape& shape, const Transform& transform) noexcept {
    BoxFrame f;
    f.center = transform.position;

    // 旋转矩阵的**列**就是局部轴在世界空间的表示（见 Mat3.h 的 FromColumns 说明）
    const Mat3 r = transform.rotation.ToMat3();
    f.axis[0] = r.Column(0);
    f.axis[1] = r.Column(1);
    f.axis[2] = r.Column(2);

    f.half = shape.box.halfExtents;
    return f;
}

/// 盒子在方向 dir（单位向量）上的投影半径，即把盒子压扁到这根轴上之后的半长。
///
/// 推导：盒子上任意一点是 center + Σ s_k * half[k] * axis[k]（s_k ∈ [-1,1]），
/// 投影到 dir 上是 Dot(center,dir) + Σ s_k * half[k] * Dot(axis[k],dir)。
/// 每一项各自取到最大值时整体最大，而 s_k 可以独立取 ±1，
/// 所以最大偏移就是 Σ half[k] * |Dot(axis[k], dir)|。
real BoxProjectedRadius(const BoxFrame& b, const Vec3& dir) noexcept {
    return b.half[0] * Abs(Dot(dir, b.axis[0])) + b.half[1] * Abs(Dot(dir, b.axis[1])) +
           b.half[2] * Abs(Dot(dir, b.axis[2]));
}

//==============================================================================
// 盒-盒的分离轴测试（SAT）
//==============================================================================
//
// 分离轴定理：两个凸多面体不相交，当且仅当存在一根轴，两者投影到该轴上的区间
// 不重叠。对两个盒子，只需要检查 15 根候选轴就足够：
//     3 根 A 的面法线 + 3 根 B 的面法线 + 3x3 根"A 的边 x B 的边"叉积
// 前 6 根覆盖"面碰面"和"面碰顶点"，后 9 根覆盖"边斜着搭在边上"——
// 最后这种情况用面法线是查不出来的，这也是为什么不能只测 6 根轴。
//
// 相交时，重叠量最小的那根轴给出**最小平移向量（MTV）**：沿它把两个盒子推开
// 那么多，恰好分离。这就是接触法线和穿透深度的来源。
//------------------------------------------------------------------------------

struct SatResult {
    Vec3 faceAxis;                ///< 最优面轴，已定向为"由 A 指向 B"
    real faceOverlap = real(0);   ///< 该轴上的重叠量（负值表示分离）
    int faceIndex = -1;           ///< 0..2 = A 的轴，3..5 = B 的轴

    Vec3 edgeAxis;
    real edgeOverlap = real(0);
    int edgeIndex = -1;  ///< 6 + i*3 + j，i 是 A 的轴、j 是 B 的轴
};

/// 返回 false 表示存在分离轴（间隙大于 margin），此时 result 的内容无意义。
bool SatBoxBox(const BoxFrame& A, const BoxFrame& B, real margin,
               SatResult& result) noexcept {
    const Vec3 t = B.center - A.center;

    // 单根轴的测试。axis 必须是单位向量，否则重叠量的量纲就不是米，
    // 15 根轴之间没法互相比较 —— 这是 SAT 用来求 MTV（而不只是判相交）时
    // 最容易出错的地方：经典的 OBB 相交测试不需要归一化，照抄过来就会错。
    const auto test = [&](const Vec3& axis, real& bestOverlap, int& bestIndex,
                          Vec3& bestAxis, int index) noexcept -> bool {
        const real ra = BoxProjectedRadius(A, axis);
        const real rb = BoxProjectedRadius(B, axis);
        const real separation = Dot(t, axis);
        const real overlap = ra + rb - Abs(separation);

        if (overlap < -margin) return false;  // 找到分离轴，可以立刻收工

        if (bestIndex < 0 || overlap < bestOverlap) {
            bestOverlap = overlap;
            bestIndex = index;
            // 定向成"由 A 指向 B"：这样最终的法线不用再判断符号
            bestAxis = (separation >= real(0)) ? axis : -axis;
        }
        return true;
    };

    for (int i = 0; i < 3; ++i) {
        if (!test(A.axis[i], result.faceOverlap, result.faceIndex, result.faceAxis, i)) {
            return false;
        }
    }
    for (int j = 0; j < 3; ++j) {
        if (!test(B.axis[j], result.faceOverlap, result.faceIndex, result.faceAxis,
                  3 + j)) {
            return false;
        }
    }

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            const Vec3 cross = Cross(A.axis[i], B.axis[j]);
            const real lenSq = cross.LengthSq();

            // 两根轴平行时叉积趋于零向量，归一化会把浮点噪声放大成一个乱七八糟
            // 的方向。这种构型（两个盒子有一对轴平行）本来就由面轴完整覆盖，
            // 直接跳过是安全的 —— 漏掉它反而比用一根噪声轴安全得多。
            if (lenSq < real(1e-6)) continue;

            const Vec3 axis = cross * (real(1) / Sqrt(lenSq));
            if (!test(axis, result.edgeOverlap, result.edgeIndex, result.edgeAxis,
                      6 + i * 3 + j)) {
                return false;
            }
        }
    }

    return true;
}

//==============================================================================
// 面裁剪（Sutherland-Hodgman）
//==============================================================================

/// 用一个半空间裁剪凸多边形，保留 Dot(p - planePoint, planeNormal) <= 0 的一侧。
/// 返回裁剪后的顶点数。out 至少要能放下 count + 1 个点。
///
/// 逐条边走：本端在内就保留；本端与另一端分居两侧就再插入一个交点。
/// 因为多边形是凸的，一条边最多与平面相交一次，所以不需要更复杂的处理。
int ClipPolygonByPlane(const Vec3* in, int count, const Vec3& planePoint,
                       const Vec3& planeNormal, Vec3* out) noexcept {
    int outCount = 0;
    for (int i = 0; i < count; ++i) {
        const Vec3& a = in[i];
        const Vec3& b = in[(i + 1) % count];
        const real da = Dot(a - planePoint, planeNormal);
        const real db = Dot(b - planePoint, planeNormal);

        if (da <= real(0)) out[outCount++] = a;

        // 跨越平面（严格异号）时补一个交点
        if ((da > real(0)) != (db > real(0))) {
            const real denom = da - db;
            const real s = (Abs(denom) > kEpsilon) ? da / denom : real(0);
            out[outCount++] = a + (b - a) * Clamp(s, real(0), real(1));
        }
    }
    return outCount;
}

/// 从候选接触点里挑出最多 4 个塞进流形。
///
/// 为什么不是"取最深的 4 个"：那样挑出来的点很可能全都挤在接触区域的同一条边上，
/// 于是流形退化成一条线，箱子会绕着这条线摇晃。要的是**张成面积最大**的一组点，
/// 它们才能提供阻止翻转的力偶。
///
/// 挑法：最深的点必选（它决定了穿透深度）；再选离它最远的点定出一条基线；
/// 最后在基线两侧各挑一个离基线最远的 —— 有符号面积的正负恰好就是"哪一侧"。
void ReduceAndEmit(const Vec3* points, const real* penetrations,
                   const std::uint32_t* ids, int count, const Vec3& normal,
                   Manifold& out) noexcept {
    out.normal = normal;
    out.pointCount = 0;

    if (count <= 4) {
        for (int i = 0; i < count; ++i) {
            out.AddPoint(points[i], penetrations[i], ids[i]);
        }
        return;
    }

    int chosen[4] = {-1, -1, -1, -1};

    // (1) 最深的点
    int i0 = 0;
    for (int i = 1; i < count; ++i) {
        if (penetrations[i] > penetrations[i0]) i0 = i;
    }
    chosen[0] = i0;

    // (2) 离它最远的点
    int i1 = -1;
    real bestDistSq = real(-1);
    for (int i = 0; i < count; ++i) {
        if (i == i0) continue;
        const real d = (points[i] - points[i0]).LengthSq();
        if (d > bestDistSq) {
            bestDistSq = d;
            i1 = i;
        }
    }
    chosen[1] = i1;

    // (3)(4) 基线两侧各取一个离基线最远的
    int i2 = -1;
    int i3 = -1;
    if (i1 >= 0) {
        const Vec3 baseline = points[i1] - points[i0];
        real bestPos = real(0);
        real bestNeg = real(0);
        for (int i = 0; i < count; ++i) {
            if (i == i0 || i == i1) continue;
            // 有符号面积：正负表示在基线的哪一侧
            const real area = Dot(Cross(baseline, points[i] - points[i0]), normal);
            if (area > bestPos) {
                bestPos = area;
                i2 = i;
            } else if (area < bestNeg) {
                bestNeg = area;
                i3 = i;
            }
        }
    }
    chosen[2] = i2;
    chosen[3] = i3;

    for (int k = 0; k < 4; ++k) {
        const int idx = chosen[k];
        if (idx >= 0) out.AddPoint(points[idx], penetrations[idx], ids[idx]);
    }

    // 极端退化（所有候选点共线）时上面可能凑不满 4 个，用剩下的补齐，
    // 免得白白丢掉本来有效的接触点。
    for (int i = 0; i < count && out.pointCount < 4; ++i) {
        bool used = false;
        for (int k = 0; k < 4; ++k) used = used || (chosen[k] == i);
        if (!used) out.AddPoint(points[i], penetrations[i], ids[i]);
    }
}

}  // namespace

//==============================================================================
// 球 vs 球
//==============================================================================

bool CollideSphereSphere(const Shape& a, const Transform& ta, const Shape& b,
                         const Transform& tb, Manifold& out) noexcept {
    // 两个球心重合时法线在数学上任意，取 +Y（"往上弹开"是最不容易出事的选择）。
    return SphereVsSphereCore(ta.position, a.sphere.radius, tb.position,
                              b.sphere.radius, Vec3::UnitY(), out);
}

//==============================================================================
// 球 vs 胶囊
//==============================================================================
//
// 胶囊是"线段外扩半径"，所以离球心最近的那一点一定在轴线上的最近点处。
// 求出它，问题就退化成球 vs 球。
//==============================================================================

bool CollideSphereCapsule(const Shape& a, const Transform& ta, const Shape& b,
                          const Transform& tb, Manifold& out) noexcept {
    Vec3 p, q;
    GetCapsuleSegment(b, tb, p, q);

    const Vec3 closest = ClosestPointOnSegment(ta.position, p, q);

    // 球心正好落在胶囊轴线上时，最短的逃逸方向是**垂直于轴线**往侧面推，
    // 而不是沿着轴线顶出去（胶囊通常细长，侧面近得多）。
    return SphereVsSphereCore(ta.position, a.sphere.radius, closest,
                              b.capsule.radius, AnyPerpendicular(q - p), out);
}

//==============================================================================
// 球 vs 盒
//==============================================================================

bool CollideSphereBox(const Shape& a, const Transform& ta, const Shape& b,
                      const Transform& tb, Manifold& out) noexcept {
    // 全程在盒的局部空间里算：那里盒子是轴对齐的，钳制就是最近点。
    // 这也是引擎里没有独立 OBB 形状的原因 —— "带旋转的 Box" 在自己的局部空间
    // 永远是 AABB。
    const Vec3 centerLocal = tb.InverseTransformPoint(ta.position);

    Vec3 normalLocal;
    real penetration;
    Vec3 boxPointLocal;
    if (!PointVsBoxLocal(centerLocal, a.sphere.radius, b.box.halfExtents, normalLocal,
                         penetration, boxPointLocal)) {
        out.Clear();
        return false;
    }

    EmitPointBoxManifold(tb, centerLocal, a.sphere.radius, normalLocal, penetration,
                         boxPointLocal, out);
    return true;
}

//==============================================================================
// 胶囊 vs 胶囊
//==============================================================================

bool CollideCapsuleCapsule(const Shape& a, const Transform& ta, const Shape& b,
                           const Transform& tb, Manifold& out) noexcept {
    Vec3 p1, q1, p2, q2;
    GetCapsuleSegment(a, ta, p1, q1);
    GetCapsuleSegment(b, tb, p2, q2);

    real s, t;
    Vec3 c1, c2;
    ClosestPointsSegmentSegment(p1, q1, p2, q2, s, t, c1, c2);

    // 两条轴线相交（最近点重合）时的兜底方向：垂直于两条轴线所张的平面。
    // 两条轴线平行时叉积退化，改用垂直于第一条轴线的任意方向。
    Vec3 fallback = Cross(q1 - p1, q2 - p2);
    fallback = fallback.IsZero() ? AnyPerpendicular(q1 - p1) : fallback.Normalized();

    return SphereVsSphereCore(c1, a.capsule.radius, c2, b.capsule.radius, fallback,
                              out);
}

//==============================================================================
// 胶囊 vs 盒
//==============================================================================
//
// 同样在盒的局部空间里做。把胶囊的中轴线段变换过去之后，问题变成
// "线段与轴对齐盒之间的最近点对"，GeometryUtil 里的 ClosestPointSegmentAABB
// 给的是精确解（分段二次极小，不是迭代）。
//
// 线段真的插进盒子里时（距离为 0）最近点对给不出方向，退回"点在盒内"的那套逻辑。
// 此时要选线段上**陷得最深**的那个点作代表：在盒内，"到表面的距离"是若干个线性
// 函数取最小，是一个凹函数，沿线段的最大值必然在端点或者折点上取到。
// 这里采样了两个端点、中点和最近点参数四个候选 —— 不是严格最优，但深度穿透
// 本来就是"求解器要在几帧内推出去"的异常状态，差一点点无关紧要。
//
//------------------------------------------------------------------------------
// TODO(upgrade): 胶囊躺平时需要两个接触点
//   现在无论什么姿态都只给一个接触点。胶囊**竖着**站在面上（底端盖顶着地面）时
//   这是几何上正确的 —— 球面和平面本来就只接触一个点，所以 FPS 的核心场景
//   （角色站地面、贴墙）不受影响。
//
//   但胶囊**躺平**压在面上时，真实接触区域是一条线段，只报一个点会让物体
//   绕着它摇晃（ragdoll 倒地、横放的圆柱形道具会看出来）。
//   做法：算出法线之后判断 |Dot(轴向, 法线)| 是否接近 0（轴线与接触面平行），
//   是的话把轴线投影到接触面上、裁到面内，两端各生成一个接触点。
//   同样的处理也适用于 CollideCapsuleCapsule 的平行胶囊。
//
// TODO(upgrade): 深度穿透时的精确 MTV
//   下面那四个采样点是权宜之计。要精确的话，"线段上到盒表面距离最大的点"
//   是 6 个线性函数的下包络在区间上求最大值，可以解析地解出来。
//   或者等 M5 的 GJK/EPA —— 深度穿透正是 EPA 的主场。
//==============================================================================

bool CollideCapsuleBox(const Shape& a, const Transform& ta, const Shape& b,
                       const Transform& tb, Manifold& out) noexcept {
    const Vec3 half = b.box.halfExtents;
    const real radius = a.capsule.radius;

    Vec3 pWorld, qWorld;
    GetCapsuleSegment(a, ta, pWorld, qWorld);

    const Vec3 p = tb.InverseTransformPoint(pWorld);
    const Vec3 q = tb.InverseTransformPoint(qWorld);

    real t;
    Vec3 segPoint, boxPoint;
    const real distSq = ClosestPointSegmentAABB(p, q, half, t, segPoint, boxPoint);

    const real limit = radius + kSpeculativeMargin;
    if (distSq > limit * limit) {
        out.Clear();
        return false;
    }

    if (distSq > kNormalizeEpsilonSq) {
        // 线段在盒外：直接用最近点对定出法线
        const real dist = Sqrt(distSq);
        const Vec3 normalLocal = (boxPoint - segPoint) * (real(1) / dist);

        EmitPointBoxManifold(tb, segPoint, radius, normalLocal, radius - dist,
                             boxPoint, out);
        return true;
    }

    // 线段插进盒里：挑一个陷得最深的采样点
    const real candidates[4] = {real(0), real(0.5), real(1), t};
    Vec3 deepest = segPoint;
    real deepestDepth = real(-1);

    for (int i = 0; i < 4; ++i) {
        const Vec3 c = p + (q - p) * candidates[i];
        // 在盒内时，到表面的距离 = 各轴余量的最小值；在盒外则为负
        real depth = half[0] - Abs(c[0]);
        depth = Min(depth, half[1] - Abs(c[1]));
        depth = Min(depth, half[2] - Abs(c[2]));
        if (depth > deepestDepth) {
            deepestDepth = depth;
            deepest = c;
        }
    }

    Vec3 normalLocal;
    real penetration;
    Vec3 boxPointLocal;
    if (!PointVsBoxLocal(deepest, radius, half, normalLocal, penetration,
                         boxPointLocal)) {
        out.Clear();
        return false;
    }

    EmitPointBoxManifold(tb, deepest, radius, normalLocal, penetration, boxPointLocal,
                         out);
    return true;
}

//==============================================================================
// 盒 vs 盒
//==============================================================================

bool CollideBoxBox(const Shape& a, const Transform& ta, const Shape& b,
                   const Transform& tb, Manifold& out) noexcept {
    const BoxFrame A = MakeBoxFrame(a, ta);
    const BoxFrame B = MakeBoxFrame(b, tb);

    SatResult sat;
    if (!SatBoxBox(A, B, kSpeculativeMargin, sat) || sat.faceIndex < 0) {
        out.Clear();
        return false;
    }

    //--------------------------------------------------------------------------
    // 面轴 还是 边轴
    //
    // 数学上应该无条件取重叠量最小的那根轴，但实践中必须**偏向面轴**：
    //   - 边叉积轴的数值质量差得多。两根近乎平行的轴叉出来的方向，
    //     方向误差可以到好几度，据此算出的法线会让物体沿墙面滑走。
    //   - 面接触能裁剪出一整块接触区域（最多 4 个点），边-边接触只能给一个点。
    //     一个点撑不住堆叠。
    // 所以只有当边轴**明显**更浅时才用它。相对 + 绝对双阈值是 Box2D 的做法：
    // 相对阈值处理大物体，绝对阈值处理小物体。
    //--------------------------------------------------------------------------
    constexpr real kEdgeBiasRelative = real(0.95);
    constexpr real kEdgeBiasAbsolute = real(0.005);  // 5 毫米

    const bool useEdge =
        sat.edgeIndex >= 0 &&
        sat.edgeOverlap < sat.faceOverlap * kEdgeBiasRelative - kEdgeBiasAbsolute;

    if (useEdge) {
        //----------------------------------------------------------------------
        // 边-边接触：两条最接近的棱，接触点取它们的最近点对的中点。
        //
        // 哪两条棱？沿法线方向"最靠前"的那两条。定位方法：先取盒子在法线方向上的
        // 支撑顶点，但**不要**沿棱本身的那根轴做取舍（棱平行于该轴，两端一样远，
        // 取哪端都对），于是得到棱的中心，再沿该轴向两边各推半个长度。
        //----------------------------------------------------------------------
        const int ia = (sat.edgeIndex - 6) / 3;  // A 的哪根轴
        const int ib = (sat.edgeIndex - 6) % 3;  // B 的哪根轴
        const Vec3 normal = sat.edgeAxis;        // 已定向为 A -> B

        Vec3 centerA = A.center;
        for (int k = 0; k < 3; ++k) {
            if (k == ia) continue;
            centerA += A.axis[k] * (Dot(normal, A.axis[k]) >= real(0) ? A.half[k]
                                                                     : -A.half[k]);
        }
        Vec3 centerB = B.center;
        for (int k = 0; k < 3; ++k) {
            if (k == ib) continue;
            // B 要朝 -normal 方向取支撑（它在 A 的那一侧）
            centerB += B.axis[k] * (Dot(normal, B.axis[k]) <= real(0) ? B.half[k]
                                                                     : -B.half[k]);
        }

        const Vec3 a0 = centerA - A.axis[ia] * A.half[ia];
        const Vec3 a1 = centerA + A.axis[ia] * A.half[ia];
        const Vec3 b0 = centerB - B.axis[ib] * B.half[ib];
        const Vec3 b1 = centerB + B.axis[ib] * B.half[ib];

        real s, t;
        Vec3 c1, c2;
        ClosestPointsSegmentSegment(a0, a1, b0, b1, s, t, c1, c2);

        out.normal = normal;
        out.pointCount = 0;
        out.AddPoint((c1 + c2) * real(0.5), sat.edgeOverlap,
                     MakeFeatureId(2, static_cast<std::uint8_t>(ia),
                                   static_cast<std::uint8_t>(ib), 0));
        return true;
    }

    //--------------------------------------------------------------------------
    // 面接触：参考面 vs 入射面
    //--------------------------------------------------------------------------
    const bool refIsA = sat.faceIndex < 3;
    const BoxFrame& ref = refIsA ? A : B;
    const BoxFrame& inc = refIsA ? B : A;
    const int refAxis = refIsA ? sat.faceIndex : sat.faceIndex - 3;

    // sat.faceAxis 的定向是"由 A 指向 B"。参考面的外法线要"由 ref 指向 inc"，
    // 所以参考盒是 B 的时候要取反。
    const Vec3 refNormal = refIsA ? sat.faceAxis : -sat.faceAxis;

    const Vec3 refFaceCenter = ref.center + refNormal * ref.half[refAxis];

    // 参考面内的两根轴
    const int refU = (refAxis + 1) % 3;
    const int refV = (refAxis + 2) % 3;

    //--------------------------------------------------------------------------
    // 入射面：inc 上朝向最"迎着"参考法线的那个面，也就是外法线与 refNormal
    // 点积最负的那个面。选错了会裁出一块背面，接触点全落在物体内部。
    //--------------------------------------------------------------------------
    int incAxis = 0;
    real incSign = real(1);
    real minDot = real(2);  // 点积的取值范围是 [-1, 1]，2 一定会被第一次比较刷掉
    for (int k = 0; k < 3; ++k) {
        const real d = Dot(inc.axis[k], refNormal);
        if (d < minDot) {
            minDot = d;
            incAxis = k;
            incSign = real(1);
        }
        if (-d < minDot) {
            minDot = -d;
            incAxis = k;
            incSign = real(-1);
        }
    }

    const Vec3 incFaceNormal = inc.axis[incAxis] * incSign;
    const Vec3 incFaceCenter = inc.center + incFaceNormal * inc.half[incAxis];
    const int incU = (incAxis + 1) % 3;
    const int incV = (incAxis + 2) % 3;

    const Vec3 su = inc.axis[incU] * inc.half[incU];
    const Vec3 sv = inc.axis[incV] * inc.half[incV];

    // 入射面的四个角，按环绕顺序（裁剪算法依赖相邻顶点构成边）
    Vec3 bufA[16];
    Vec3 bufB[16];
    bufA[0] = incFaceCenter - su - sv;
    bufA[1] = incFaceCenter + su - sv;
    bufA[2] = incFaceCenter + su + sv;
    bufA[3] = incFaceCenter - su + sv;
    int count = 4;

    //--------------------------------------------------------------------------
    // 用参考面的四个侧面把入射面裁掉超出去的部分。
    // 注意裁的是**侧面**（4 个），不包括参考面本身 —— 参考面那一侧的判断
    // 留到后面按穿透深度筛，因为我们要保留"还没接触但很近"的推测性接触点。
    //--------------------------------------------------------------------------
    const int sideAxes[2] = {refU, refV};
    Vec3* src = bufA;
    Vec3* dst = bufB;

    for (int s = 0; s < 2; ++s) {
        const int axisIdx = sideAxes[s];
        for (int sign = 0; sign < 2; ++sign) {
            const Vec3 planeNormal =
                ref.axis[axisIdx] * (sign == 0 ? real(1) : real(-1));
            const Vec3 planePoint = refFaceCenter + planeNormal * ref.half[axisIdx];

            count = ClipPolygonByPlane(src, count, planePoint, planeNormal, dst);
            Vec3* tmp = src;
            src = dst;
            dst = tmp;

            if (count == 0) break;
        }
        if (count == 0) break;
    }

    if (count == 0) {
        // 裁光了。理论上 SAT 说相交就不该发生，但浮点世界里"理论上"不算数 ——
        // 退化成单点接触总好过报一个空流形让上层以为没碰上。
        out.normal = sat.faceAxis;
        out.pointCount = 0;
        out.AddPoint((A.center + B.center) * real(0.5), sat.faceOverlap,
                     MakeFeatureId(3, 0, 0, 0));
        return true;
    }

    //--------------------------------------------------------------------------
    // 保留真正压在参考面上（或者近到 kSpeculativeMargin 以内）的点
    //--------------------------------------------------------------------------
    Vec3 points[16];
    real penetrations[16];
    std::uint32_t ids[16];
    int kept = 0;

    const std::uint8_t idRef =
        static_cast<std::uint8_t>((refIsA ? 0 : 1) * 8 + refAxis * 2 +
                                  (Dot(refNormal, ref.axis[refAxis]) >= real(0) ? 0 : 1));
    const std::uint8_t idInc = static_cast<std::uint8_t>(
        incAxis * 2 + (incSign >= real(0) ? 0 : 1));

    for (int i = 0; i < count; ++i) {
        const real separation = Dot(src[i] - refFaceCenter, refNormal);
        if (separation > kSpeculativeMargin) continue;

        // 接触点取中点：src[i] 在入射面（inc 的表面）上，它在参考面上的投影
        // 是另一个见证点，两者相距 separation。
        points[kept] = src[i] - refNormal * (separation * real(0.5));
        penetrations[kept] = -separation;
        ids[kept] = MakeFeatureId(idRef, idInc, static_cast<std::uint8_t>(i), 0);
        ++kept;
    }

    if (kept == 0) {
        out.Clear();
        return false;
    }

    // 法线始终按"由 A 指向 B"输出，与参考盒是谁无关
    ReduceAndEmit(points, penetrations, ids, kept, sat.faceAxis, out);
    return true;
}

//==============================================================================
// 分派
//==============================================================================

bool Collide(const Shape& a, const Transform& ta, const Shape& b, const Transform& tb,
             Manifold& out) noexcept {
    // 规范顺序：形状对按 ShapeType 的枚举值升序排列，只需要实现一半的组合。
    // 顺序反了就交换着调用，再把法线取反 —— 接触点用的是两个见证点的中点，
    // 交换 A、B 不会改变它，所以取反法线就够了。
    const bool swapped = a.type > b.type;
    const Shape& first = swapped ? b : a;
    const Transform& tFirst = swapped ? tb : ta;
    const Shape& second = swapped ? a : b;
    const Transform& tSecond = swapped ? ta : tb;

    bool hit = false;
    switch (first.type) {
        case ShapeType::Sphere:
            switch (second.type) {
                case ShapeType::Sphere:
                    hit = CollideSphereSphere(first, tFirst, second, tSecond, out);
                    break;
                case ShapeType::Capsule:
                    hit = CollideSphereCapsule(first, tFirst, second, tSecond, out);
                    break;
                case ShapeType::Box:
                    hit = CollideSphereBox(first, tFirst, second, tSecond, out);
                    break;
            }
            break;

        case ShapeType::Capsule:
            switch (second.type) {
                case ShapeType::Sphere:
                    break;  // 规范顺序保证到不了这里
                case ShapeType::Capsule:
                    hit = CollideCapsuleCapsule(first, tFirst, second, tSecond, out);
                    break;
                case ShapeType::Box:
                    hit = CollideCapsuleBox(first, tFirst, second, tSecond, out);
                    break;
            }
            break;

        case ShapeType::Box:
            switch (second.type) {
                case ShapeType::Sphere:
                case ShapeType::Capsule:
                    break;  // 同上
                case ShapeType::Box:
                    hit = CollideBoxBox(first, tFirst, second, tSecond, out);
                    break;
            }
            break;
    }

    if (!hit) {
        out.Clear();
        return false;
    }

    if (swapped) out.normal = -out.normal;
    return true;
}

//==============================================================================
// 通用凸形状路径（GJK + EPA）
//==============================================================================
//
// 分两条支路，分界线是"两个**核**相不相交"（核 = 去掉半径外扩之后的形状，
// 见 GJK.h）：
//
//   (1) 核分离。GJK 直接给出核之间的距离和最近点对，接触信息完全由它推出：
//         法线   = 从 A 的最近点指向 B 的最近点
//         穿透   = rA + rB - 核距离
//       球、胶囊之间的碰撞全部走这里 —— 它们的核是点和线段，几乎不可能相交。
//       这一支不需要 EPA，也没有任何迭代精度问题。
//
//   (2) 核相交。GJK 只能回答"相交"，深度交给 EPA：
//         法线   = EPA 最近面的朝外法线
//         穿透   = 核穿透 + rA + rB
//       实际上只有盒-盒重叠、以及胶囊轴线插进盒子里会走到这里。
//
// 两支的输出格式完全一样，都是"两个见证点 + 深度"，和解析解那一套对齐。
//==============================================================================

bool CollideConvex(const Shape& a, const Transform& ta, const Shape& b,
                   const Transform& tb, Manifold& out) noexcept {
    const ConvexProxy pa(a, ta);
    const ConvexProxy pb(b, tb);

    const real radiusA = pa.CoreRadius();
    const real radiusB = pb.CoreRadius();
    const real radiusSum = radiusA + radiusB;

    const GjkResult gjk = Gjk(pa, pb);

    if (gjk.status != GjkStatus::Intersecting) {
        //----------------------------------------------------------------------
        // (1) 核分离
        //----------------------------------------------------------------------
        const real dist = gjk.distance;
        if (dist > radiusSum + kSpeculativeMargin) {
            out.Clear();
            return false;
        }

        const Vec3 delta = gjk.pointB - gjk.pointA;
        const real deltaLen = delta.Length();

        // 核距离为 0 却没被判成相交：数值上的边缘情形，兜一个方向出来
        const Vec3 normal = (deltaLen > kEpsilon)
                                ? delta * (real(1) / deltaLen)
                                : Vec3::UnitY();

        const Vec3 witnessA = gjk.pointA + normal * radiusA;
        const Vec3 witnessB = gjk.pointB - normal * radiusB;

        out.normal = normal;
        out.pointCount = 0;
        out.AddPoint((witnessA + witnessB) * real(0.5), radiusSum - dist,
                     MakeFeatureId(0, 0, 0, 0));
        return true;
    }

    //--------------------------------------------------------------------------
    // (2) 核相交 -> EPA
    //--------------------------------------------------------------------------
    const EpaResult epa = Epa(pa, pb, gjk.simplex);

    if (!epa.valid) {
        // EPA 撑不起初始四面体（两个核完全重合，比如两个同心的球）。
        // 这时没有任何方向信息可言，给一个安全的兜底：往上推开整个半径和。
        out.normal = Vec3::UnitY();
        out.pointCount = 0;
        out.AddPoint((ta.position + tb.position) * real(0.5), radiusSum,
                     MakeFeatureId(0, 0, 0, 0));
        return true;
    }

    const Vec3 witnessA = epa.pointA + epa.normal * radiusA;
    const Vec3 witnessB = epa.pointB - epa.normal * radiusB;

    out.normal = epa.normal;
    out.pointCount = 0;
    out.AddPoint((witnessA + witnessB) * real(0.5), epa.depth + radiusSum,
                 MakeFeatureId(1, 0, 0, 0));
    return true;
}

real ConvexDistance(const Shape& a, const Transform& ta, const Shape& b,
                    const Transform& tb, Vec3& outPointA, Vec3& outPointB) noexcept {
    const ConvexProxy pa(a, ta);
    const ConvexProxy pb(b, tb);

    const GjkResult gjk = Gjk(pa, pb);
    const real radiusA = pa.CoreRadius();
    const real radiusB = pb.CoreRadius();

    if (gjk.status == GjkStatus::Intersecting) {
        outPointA = gjk.pointA;
        outPointB = gjk.pointB;
        return real(0);
    }

    // 核的最近点沿连线各自外扩半径，就得到真实表面上的最近点
    const Vec3 delta = gjk.pointB - gjk.pointA;
    const real deltaLen = delta.Length();
    const Vec3 dir = (deltaLen > kEpsilon) ? delta * (real(1) / deltaLen) : Vec3::UnitY();

    outPointA = gjk.pointA + dir * radiusA;
    outPointB = gjk.pointB - dir * radiusB;

    // 半径和大于核距离时表面已经重叠了，距离取 0（不返回负数）
    return Max(real(0), gjk.distance - radiusA - radiusB);
}

//==============================================================================
// 只判相交
//==============================================================================
//
// 和 Collide 走的是同一批几何量，区别只在于不生成接触点。对盒-盒来说这省掉了
// 整个面裁剪流程（十几次点积加上多边形裁剪），对触发器这种每帧成百上千次的
// 查询很值。
//==============================================================================

bool Overlap(const Shape& a, const Transform& ta, const Shape& b,
             const Transform& tb) noexcept {
    const bool swapped = a.type > b.type;
    const Shape& first = swapped ? b : a;
    const Transform& tFirst = swapped ? tb : ta;
    const Shape& second = swapped ? a : b;
    const Transform& tSecond = swapped ? ta : tb;

    switch (first.type) {
        case ShapeType::Sphere: {
            switch (second.type) {
                case ShapeType::Sphere: {
                    const real r = first.sphere.radius + second.sphere.radius;
                    return DistanceSq(ta.position, tb.position) <= r * r;
                }
                case ShapeType::Capsule: {
                    Vec3 p, q;
                    GetCapsuleSegment(second, tSecond, p, q);
                    const real r = first.sphere.radius + second.capsule.radius;
                    return DistanceSqPointSegment(tFirst.position, p, q) <= r * r;
                }
                case ShapeType::Box: {
                    const Vec3 c = tSecond.InverseTransformPoint(tFirst.position);
                    const Vec3 h = second.box.halfExtents;
                    const Vec3 closest(Clamp(c.x, -h.x, h.x), Clamp(c.y, -h.y, h.y),
                                       Clamp(c.z, -h.z, h.z));
                    const real r = first.sphere.radius;
                    return (closest - c).LengthSq() <= r * r;
                }
            }
            break;
        }

        case ShapeType::Capsule: {
            switch (second.type) {
                case ShapeType::Sphere:
                    break;  // 规范顺序保证到不了这里
                case ShapeType::Capsule: {
                    Vec3 p1, q1, p2, q2;
                    GetCapsuleSegment(first, tFirst, p1, q1);
                    GetCapsuleSegment(second, tSecond, p2, q2);
                    real s, t;
                    Vec3 c1, c2;
                    const real d2 =
                        ClosestPointsSegmentSegment(p1, q1, p2, q2, s, t, c1, c2);
                    const real r = first.capsule.radius + second.capsule.radius;
                    return d2 <= r * r;
                }
                case ShapeType::Box: {
                    Vec3 pw, qw;
                    GetCapsuleSegment(first, tFirst, pw, qw);
                    const Vec3 p = tSecond.InverseTransformPoint(pw);
                    const Vec3 q = tSecond.InverseTransformPoint(qw);
                    real t;
                    Vec3 segPoint, boxPoint;
                    const real d2 = ClosestPointSegmentAABB(
                        p, q, second.box.halfExtents, t, segPoint, boxPoint);
                    const real r = first.capsule.radius;
                    return d2 <= r * r;
                }
            }
            break;
        }

        case ShapeType::Box: {
            switch (second.type) {
                case ShapeType::Sphere:
                case ShapeType::Capsule:
                    break;  // 同上
                case ShapeType::Box: {
                    SatResult sat;
                    return SatBoxBox(MakeBoxFrame(first, tFirst),
                                     MakeBoxFrame(second, tSecond), real(0), sat);
                }
            }
            break;
        }
    }

    return false;
}

}  // namespace pe
