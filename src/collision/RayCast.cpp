//==============================================================================
// src/collision/RayCast.cpp
//
// 各形状的射线求交闭式解。
//
// 每个函数的注释都写了"方程怎么列的"，因为这类代码一旦写出来就很难从
// 结果反推是哪一步错了 —— 符号错、少考虑一个根、法线方向反，症状都只是
// "子弹偶尔打不中"。
//==============================================================================

#include "pe/collision/RayCast.h"

#include "pe/collision/GeometryUtil.h"

namespace pe {

namespace {

/// 起点在物体内部时的统一返回。见 Ray.h 的约定 2。
inline void FillInsideHit(const Ray& ray, RaycastHit& out) noexcept {
    out.distance = real(0);
    out.point = ray.origin;
    // 法线取 -direction：这样调用方按"法线朝向射线来向"的通用假设去做
    // 反射/推出计算时，行为仍然是合理的（把子弹原路弹回）。
    out.normal = -ray.direction;
}

}  // namespace

//==============================================================================
// 射线 vs 球
//==============================================================================
//
// 把射线代入球面方程：
//     |o + t*d - c|^2 = r^2
// 令 m = o - c，展开（利用 d 是单位向量，所以 d·d = 1）：
//     t^2 + 2*(m·d)*t + (m·m - r^2) = 0
// 这是标准二次方程 t^2 + 2bt + c0 = 0，其中
//     b  = m·d          （射线起点到球心的连线在射线方向上的投影，取负号才是距离）
//     c0 = m·m - r^2    （起点到球心距离的平方，减去半径平方；>0 表示起点在球外）
// 判别式 disc = b^2 - c0，解为 t = -b ± sqrt(disc)。
//
// 两个提前退出的判断很值钱（射线查询是 FPS 里最高频的操作）：
//   - c0 <= 0：起点在球内 —— 直接按"内部命中"约定返回
//   - b > 0 且 c0 > 0：起点在球外，且射线朝**背离**球心的方向 —— 不可能命中
//==============================================================================
bool RaycastSphere(const Vec3& center, real radius, const Ray& ray,
                   RaycastHit& out) noexcept {
    const Vec3 m = ray.origin - center;
    const real b = Dot(m, ray.direction);
    const real c0 = m.LengthSq() - radius * radius;

    if (c0 <= real(0)) {
        FillInsideHit(ray, out);
        return true;
    }
    if (b > real(0)) {
        return false;  // 在球外且背离球心
    }

    const real disc = b * b - c0;
    if (disc < real(0)) {
        return false;  // 无实根，射线从旁边擦过去了
    }

    // 取较小的根（先碰到的那个交点）。
    // 这里 t 必定为正：c0 > 0 意味着 disc < b^2，于是 sqrt(disc) < |b| = -b，
    // 所以 t = -b - sqrt(disc) > 0。不需要再判负。
    const real t = -b - Sqrt(disc);
    if (t > ray.maxDistance) {
        return false;  // 命中点在射程之外
    }

    out.distance = t;
    out.point = ray.PointAt(t);
    // 球面外法线就是从球心指向命中点的方向
    out.normal = (out.point - center).Normalized();
    return true;
}

//==============================================================================
// 射线 vs AABB —— slab 方法
//==============================================================================
//
// 把 AABB 看成三对平行平面（"slab"，板）的交集。射线穿过每一对板，会得到一段
// 参数区间 [t1, t2]。射线与盒子相交，当且仅当三段区间的**交集非空**。
//
// 所以算法就是维护一个逐步收窄的区间 [tmin, tmax]：
//     tmin = max(所有进入时刻)
//     tmax = min(所有离开时刻)
// 一旦 tmin > tmax，说明"最晚的进入"发生在"最早的离开"之后，不可能同时在
// 三对板内，立即返回不中。
//
// tmin 初值取 0（不考虑起点之前的交点），tmax 初值取射程 —— 这样射程限制
// 自然地融进了同一套区间运算里，不需要最后再单独判一次。
//
// 法线：记录下最后一次**收窄 tmin** 的那个轴，命中面就在那个轴上。
// 面朝向由射线在该轴上的方向决定：d > 0 说明从负面进入，法线是 -e_i。
//==============================================================================
bool RaycastAABB(const AABB& box, const Ray& ray, RaycastHit& out) noexcept {
    real tmin = real(0);
    real tmax = ray.maxDistance;
    int hitAxis = -1;  // -1 表示 tmin 从未被收窄，即起点已经在盒内

    for (int i = 0; i < 3; ++i) {
        const real d = ray.direction[i];
        const real o = ray.origin[i];

        if (Abs(d) < kEpsilon) {
            // 射线平行于这对板。此时要么永远在板内（不产生约束），
            // 要么永远在板外（直接不可能命中）。
            // 单独处理是为了避免除以 0 得到 inf，进而在后面产生 0*inf = NaN。
            if (o < box.min[i] || o > box.max[i]) {
                return false;
            }
            continue;
        }

        const real inv = real(1) / d;
        real tNear = (box.min[i] - o) * inv;
        real tFar = (box.max[i] - o) * inv;
        if (tNear > tFar) {
            // 射线朝负方向走时两者会颠倒，换回来
            const real tmp = tNear;
            tNear = tFar;
            tFar = tmp;
        }

        if (tNear > tmin) {
            tmin = tNear;
            hitAxis = i;
        }
        if (tFar < tmax) {
            tmax = tFar;
        }
        if (tmin > tmax) {
            return false;
        }
    }

    if (hitAxis < 0) {
        // 三个轴都没能把 tmin 从 0 推上去 => 起点在盒内
        FillInsideHit(ray, out);
        return true;
    }

    out.distance = tmin;
    out.point = ray.PointAt(tmin);

    Vec3 n = Vec3::Zero();
    n[hitAxis] = ray.direction[hitAxis] > real(0) ? real(-1) : real(1);
    out.normal = n;
    return true;
}

//==============================================================================
// 射线 vs OBB
//==============================================================================
//
// 不需要新算法：把射线变换到盒子的局部空间，盒子在那里就是轴对齐的，
// 直接复用 slab 方法，再把结果变换回世界空间。
//
// 关键点：**距离 t 在两个空间里是同一个值**。因为 Transform 只含旋转和平移，
// 都是保长变换，局部空间里的单位方向向量变换回去还是单位的，走过的路程不变。
// （这正是 Transform 刻意不含缩放的好处之一 —— 有缩放的话这里就要重新换算。）
//==============================================================================
bool RaycastOBB(const Transform& transform, const Vec3& halfExtents, const Ray& ray,
                RaycastHit& out) noexcept {
    Ray local;
    local.origin = transform.InverseTransformPoint(ray.origin);
    local.direction = transform.InverseTransformDirection(ray.direction);
    local.maxDistance = ray.maxDistance;

    RaycastHit localHit;
    if (!RaycastAABB(AABB(-halfExtents, halfExtents), local, localHit)) {
        return false;
    }

    out.distance = localHit.distance;
    out.point = transform.TransformPoint(localHit.point);
    // 法线是方向，只旋转不平移
    out.normal = transform.TransformDirection(localHit.normal);
    return true;
}

//==============================================================================
// 射线 vs 胶囊
//==============================================================================
//
// 胶囊 = 线段外扩 radius，表面由三部分组成：
//     侧面（有限圆柱面） + 上端盖半球 + 下端盖半球
// 分别求交，取参数最小的那个有效解。
//
// 在胶囊的局部空间里算（轴 = +Y），三部分的方程都变得很简单。
//
// --- 侧面 ---
// 无限长圆柱面 x^2 + z^2 = r^2 在 XZ 平面上就是一个圆，所以把射线投影到
// XZ 平面，问题退化成 2D 的"射线 vs 圆"：
//     a = dx^2 + dz^2            （注意 a 可能为 0：射线平行于胶囊轴）
//     b = ox*dx + oz*dz
//     c = ox^2 + oz^2 - r^2
//     a*t^2 + 2b*t + c = 0
// 求出 t 之后，还要检查交点的 y 是否落在 [-halfHeight, +halfHeight] 内 ——
// 落在外面说明射线打在了"无限圆柱的延长部分"上，那里其实是端盖的地盘。
//
// --- 端盖 ---
// 就是两个球。但求出的交点必须在端盖那一侧之外（下端盖要求 y <= -halfHeight）。
// 为什么必须过滤：球面上 y > -halfHeight 的那部分点，代入"点到中轴线段的距离"
// 会得到 sqrt(r^2 - (y+h)^2) < r，也就是说那些点其实在胶囊**内部**，
// 根本不是表面。不过滤的话会返回一个位于实体内部的假命中点。
//==============================================================================
bool RaycastCapsule(const Transform& transform, real radius, real halfHeight,
                    const Ray& ray, RaycastHit& out) noexcept {
    // ---- 变换到胶囊局部空间 ----
    const Vec3 o = transform.InverseTransformPoint(ray.origin);
    const Vec3 d = transform.InverseTransformDirection(ray.direction);

    const Vec3 segA(real(0), -halfHeight, real(0));
    const Vec3 segB(real(0), halfHeight, real(0));

    // ---- 起点在胶囊内部？ ----
    if (DistanceSqPointSegment(o, segA, segB) <= radius * radius) {
        FillInsideHit(ray, out);
        return true;
    }

    real bestT = ray.maxDistance;
    Vec3 bestNormalLocal = Vec3::Zero();
    bool found = false;

    // ---- 侧面（有限圆柱） ----
    {
        const real a = d.x * d.x + d.z * d.z;
        if (a > kEpsilon) {  // a 接近 0 表示射线几乎平行于胶囊轴，只可能打到端盖
            const real b = o.x * d.x + o.z * d.z;
            const real c = o.x * o.x + o.z * o.z - radius * radius;
            const real disc = b * b - a * c;

            if (disc >= real(0)) {
                // 近交点。起点已确定在胶囊外，所以只需要考虑较小的那个根。
                const real t = (-b - Sqrt(disc)) / a;
                if (t >= real(0) && t < bestT) {
                    const real y = o.y + t * d.y;
                    if (y >= -halfHeight && y <= halfHeight) {
                        bestT = t;
                        // 圆柱侧面的法线是径向的，没有 Y 分量
                        const Vec3 p = o + d * t;
                        bestNormalLocal = Vec3(p.x, real(0), p.z).Normalized();
                        found = true;
                    }
                }
            }
        }
    }

    // ---- 两个端盖（球） ----
    // 这里手写而不是调用 RaycastSphere，因为需要在局部空间做、还要额外过滤 y，
    // 而且已经确定起点在外部（可以跳过 RaycastSphere 里的内部判断）。
    const Vec3 caps[2] = {segA, segB};
    for (int i = 0; i < 2; ++i) {
        const Vec3 m = o - caps[i];
        const real b = Dot(m, d);
        const real c = m.LengthSq() - radius * radius;
        const real disc = b * b - c;
        if (disc < real(0)) {
            continue;
        }
        const real t = -b - Sqrt(disc);
        if (t < real(0) || t >= bestT) {
            continue;
        }
        const Vec3 p = o + d * t;
        // 过滤掉落在圆柱段范围内的那半个球面（那部分在胶囊内部，不是表面）
        const bool onCap = (i == 0) ? (p.y <= -halfHeight) : (p.y >= halfHeight);
        if (!onCap) {
            continue;
        }
        bestT = t;
        bestNormalLocal = (p - caps[i]).Normalized();
        found = true;
    }

    if (!found) {
        return false;
    }

    out.distance = bestT;
    out.point = ray.PointAt(bestT);
    out.normal = transform.TransformDirection(bestNormalLocal);
    return true;
}

//==============================================================================
// 分派
//==============================================================================
bool RaycastShape(const Shape& shape, const Transform& transform, const Ray& ray,
                  RaycastHit& out) noexcept {
    switch (shape.type) {
        case ShapeType::Sphere:
            // 球是各向同性的，旋转无关，只用位置
            return RaycastSphere(transform.position, shape.sphere.radius, ray, out);
        case ShapeType::Capsule:
            return RaycastCapsule(transform, shape.capsule.radius,
                                  shape.capsule.halfHeight, ray, out);
        case ShapeType::Box:
            return RaycastOBB(transform, shape.box.halfExtents, ray, out);
    }
    return false;  // 不可达
}

}  // namespace pe
