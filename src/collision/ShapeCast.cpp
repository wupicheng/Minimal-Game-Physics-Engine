//==============================================================================
// src/collision/ShapeCast.cpp
//
// 保守推进的实现。原理见 ShapeCast.h。
//==============================================================================

#include "pe/collision/ShapeCast.h"

#include "pe/collision/NarrowPhase.h"

namespace pe {

bool ShapeCast(const Shape& moving, const Transform& start, const Vec3& displacement,
               const Shape& target, const Transform& targetTransform,
               ShapeCastHit& out) noexcept {
    out.fraction = real(0);
    out.point = start.position;
    out.normal = Vec3::Zero();
    out.startPenetrating = false;
    out.depth = real(0);

    //--------------------------------------------------------------------------
    // 起点处就已经接触或重叠：**必须单独处理**，不能交给后面的推进循环。
    //
    // 原因是推进循环靠"两个最近点的连线"来定方向，而两个形状贴在一起时这两点
    // 重合，方向退化成零向量 —— 循环里只能退回用位移方向当法线，那是错的。
    // 症状极具迷惑性：一个**站在地面上**的角色朝前扫掠，会得到一个指向后方的
    // "墙面"法线，于是被自己脚下的地面当成墙挡住，一步也走不动。
    //
    // 这里改用 CollideConvex 取法线：它在"恰好接触"这个区间里给出的是真正的
    // 表面法线（推测性接触，见 NarrowPhase.h），不依赖最近点连线。
    //--------------------------------------------------------------------------
    {
        Manifold m;
        if (CollideConvex(moving, start, target, targetTransform, m)) {
            const real depth = m.MaxPenetration();
            // 流形的法线由 A（moving）指向 B（target）。要"把 moving 推出去"
            // 方向是反过来的 —— 这与 normal 的约定（朝向扫掠形状来的一侧）一致。
            const Vec3 escapeNormal = -m.normal;

            if (depth > kShapeCastTolerance) {
                // 真的嵌进去了。物体生成时卡在墙里、被传送进几何体、
                // 上一帧被求解器推过头，都会走到这里。
                out.fraction = real(0);
                out.startPenetrating = true;
                out.depth = depth;
                out.point = m.points[0].position;
                out.normal = escapeNormal;
                return true;
            }

            if (depth >= -kShapeCastTolerance) {
                //--------------------------------------------------------------
                // 恰好贴着（既没嵌进去，也没离开容差范围）。
                //
                // 这时候撞不撞得上，**取决于往哪儿走**：
                //   - 位移朝表面里去   -> 立刻命中，fraction = 0
                //   - 位移朝外或者平行 -> 不挡路，返回未命中
                //
                // 第二条是角色能沿着地面走路的前提。少了它，任何"贴着表面"的
                // 物体都会被自己脚下那个面永久挡住 —— 而且因为两个凸形状沿直线
                // 运动时相交的时间集合是一个区间，"贴着且正在离开"意味着接触
                // 到此为止，后面不可能再撞上，所以直接返回 false 是安全的。
                //--------------------------------------------------------------
                if (Dot(displacement, escapeNormal) < -kEpsilon) {
                    out.fraction = real(0);
                    out.point = m.points[0].position;
                    out.normal = escapeNormal;
                    return true;
                }
                return false;
            }
            // depth 比 -kShapeCastTolerance 还小，说明只是落在推测性接触的
            // 外围（还有肉眼可见的缝），照常走下面的推进循环。
        }
    }

    const real displacementLength = displacement.Length();
    if (displacementLength <= kEpsilon) {
        // 原地不动，且没有接触 -> 什么都撞不到
        return false;
    }
    const Vec3 direction = displacement * (real(1) / displacementLength);

    //--------------------------------------------------------------------------
    // 保守推进
    //--------------------------------------------------------------------------
    real t = real(0);

    // 最后一次算出来的、方向明确的"从我指向目标"的单位向量。
    // 收敛到接触点时两个最近点重合，方向变得无意义（零向量），
    // 所以要把上一轮那个还算得准的方向留着当法线用。
    Vec3 lastDirection = direction;

    for (int iter = 0; iter < kShapeCastMaxIterations; ++iter) {
        const Transform current(start.position + displacement * t, start.rotation);

        Vec3 pointOnMoving, pointOnTarget;
        const real distance = ConvexDistance(moving, current, target, targetTransform,
                                             pointOnMoving, pointOnTarget);

        const Vec3 delta = pointOnTarget - pointOnMoving;
        const real deltaLength = delta.Length();
        if (deltaLength > kEpsilon) {
            lastDirection = delta * (real(1) / deltaLength);
        }

        //----------------------------------------------------------------------
        // 贴上了
        //----------------------------------------------------------------------
        if (distance <= kShapeCastTolerance) {
            out.fraction = Clamp(t, real(0), real(1));
            out.point = (pointOnMoving + pointOnTarget) * real(0.5);
            // 法线朝向扫掠形状来的一侧，所以是"目标指向我"，即 -lastDirection
            out.normal = -lastDirection;
            out.startPenetrating = false;
            return true;
        }

        //----------------------------------------------------------------------
        // 沿位移方向，每单位 t 能靠近多少。
        //
        // 注意投影的是**整段位移**而不是单位方向：t 的量纲是"整段位移的比例"，
        // 所以 approach 的量纲必须是"每单位 t 走过的靠近量"。
        // 用单位方向的话步长会被放大 |displacement| 倍，直接跳过接触点 —— 漏报。
        //----------------------------------------------------------------------
        const real approach = Dot(displacement, lastDirection);
        if (approach <= kEpsilon) {
            // 越走越远（或者平行擦过），这一整段位移都不可能撞上
            return false;
        }

        // 保守步长：至少这么长的时间里绝不可能相撞
        t += distance / approach;

        if (t > real(1)) {
            // 走完全程也没够着
            return false;
        }
    }

    //--------------------------------------------------------------------------
    // 迭代用尽仍未收敛。
    //
    // 实践中只会在极端掠射（位移几乎与表面平行）时出现。此时 t 已经非常接近
    // 真实的接触点，直接把它当作命中返回 —— 这是**保守**的：宁可略早一点停下，
    // 也不能漏报（漏报意味着穿墙）。
    //--------------------------------------------------------------------------
    out.fraction = Clamp(t, real(0), real(1));
    out.point = start.position + displacement * out.fraction;
    out.normal = -lastDirection;
    out.startPenetrating = false;
    return true;
}

}  // namespace pe
