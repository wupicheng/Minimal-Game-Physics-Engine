//==============================================================================
// src/collision/Shape.cpp
//
// 形状的几何属性：包围半径、体积、局部 AABB、世界 AABB。
//
// 这些 switch 故意**不写 default 分支**。这样将来 M5 加入 ConvexHull 时，
// -Wswitch（-Wall 的一部分）会把每一处遗漏精确地报出来。
// 代价是每个函数末尾要补一个"不可达"的 return，见下。
//==============================================================================

#include "pe/collision/Shape.h"

#include "pe/math/Mat3.h"

namespace pe {

//------------------------------------------------------------------------------
// 参数合法性
//------------------------------------------------------------------------------

bool Shape::IsValid() const noexcept {
    switch (type) {
        case ShapeType::Sphere:
            return sphere.radius > real(0);
        case ShapeType::Capsule:
            // halfHeight 允许为 0（退化成球），但半径必须为正
            return capsule.radius > real(0) && capsule.halfHeight >= real(0);
        case ShapeType::Box:
            return box.halfExtents.x > real(0) && box.halfExtents.y > real(0) &&
                   box.halfExtents.z > real(0);
    }
    return false;  // 不可达；仅为消除"控制流到达非 void 函数末尾"的警告
}

//------------------------------------------------------------------------------
// 包围半径
//------------------------------------------------------------------------------

real Shape::BoundingRadius() const noexcept {
    switch (type) {
        case ShapeType::Sphere:
            return sphere.radius;
        case ShapeType::Capsule:
            // 离中心最远的点是端盖半球的极点：halfHeight + radius
            return capsule.halfHeight + capsule.radius;
        case ShapeType::Box:
            // 离中心最远的是角点，距离就是半尺寸向量的长度
            return box.halfExtents.Length();
    }
    return real(0);  // 不可达
}

//------------------------------------------------------------------------------
// 体积（M6 由它 × 密度得到质量）
//------------------------------------------------------------------------------

real Shape::Volume() const noexcept {
    switch (type) {
        case ShapeType::Sphere: {
            const real r = sphere.radius;
            // V = (4/3) * pi * r^3
            return (real(4) / real(3)) * kPi * r * r * r;
        }
        case ShapeType::Capsule: {
            const real r = capsule.radius;
            const real h = capsule.halfHeight;
            // 胶囊 = 圆柱段 + 两个半球（拼起来正好是一个完整的球）
            //   圆柱：pi * r^2 * (2h)
            //   两个半球：(4/3) * pi * r^3
            const real cylinder = kPi * r * r * (real(2) * h);
            const real spheres = (real(4) / real(3)) * kPi * r * r * r;
            return cylinder + spheres;
        }
        case ShapeType::Box: {
            const Vec3& he = box.halfExtents;
            // 半尺寸的 8 倍
            return real(8) * he.x * he.y * he.z;
        }
    }
    return real(0);  // 不可达
}

//------------------------------------------------------------------------------
// 局部 AABB
//------------------------------------------------------------------------------

AABB Shape::LocalAABB() const noexcept {
    switch (type) {
        case ShapeType::Sphere: {
            const Vec3 r(sphere.radius);
            return AABB(-r, r);
        }
        case ShapeType::Capsule: {
            // 轴沿 Y：Y 方向是 halfHeight + radius，X/Z 方向只有 radius
            const Vec3 he(capsule.radius, capsule.halfHeight + capsule.radius,
                          capsule.radius);
            return AABB(-he, he);
        }
        case ShapeType::Box:
            return AABB(-box.halfExtents, box.halfExtents);
    }
    return AABB(Vec3::Zero(), Vec3::Zero());  // 不可达
}

//------------------------------------------------------------------------------
// 胶囊的世界空间中轴线段
//------------------------------------------------------------------------------

void GetCapsuleSegment(const Shape& shape, const Transform& transform, Vec3& outA,
                       Vec3& outB) noexcept {
    // 局部轴是 +Y，所以两个端点是 (0, ±halfHeight, 0)
    const Vec3 axis = transform.TransformDirection(Vec3::UnitY());
    const Vec3 offset = axis * shape.capsule.halfHeight;
    outA = transform.position - offset;
    outB = transform.position + offset;
}

//------------------------------------------------------------------------------
// 世界 AABB
//------------------------------------------------------------------------------

AABB ComputeWorldAABB(const Shape& shape, const Transform& transform) noexcept {
    switch (shape.type) {
        case ShapeType::Sphere: {
            // 球是各向同性的：旋转不影响它，只需要平移球心再外扩半径。
            const Vec3 r(shape.sphere.radius);
            return AABB(transform.position - r, transform.position + r);
        }

        case ShapeType::Capsule: {
            // 胶囊 = 线段外扩 radius。所以它的 AABB 就是
            // "两个端点的 AABB" 再各方向外扩 radius —— 这是精确且紧致的，
            // 因为球形外扩在轴对齐方向上恰好就是各分量 ±radius。
            Vec3 a, b;
            GetCapsuleSegment(shape, transform, a, b);
            AABB box = AABB::FromMinMax(a, b);
            box.Expand(shape.capsule.radius);
            return box;
        }

        case ShapeType::Box: {
            // 旋转后的盒子的世界 AABB。
            //
            // 推导：盒子的 8 个角点在局部空间是 (±hx, ±hy, ±hz)。
            // 变换到世界后，第 i 个分量是
            //     sum_j R[i][j] * (s_j * h_j)     其中 s_j ∈ {-1, +1}
            // 要让这个分量取到最大，每一项都取正，即
            //     max_i = sum_j |R[i][j]| * h_j
            // 也就是"把旋转矩阵逐元素取绝对值，再乘上半尺寸向量"。
            // 最小值由对称性就是 -max_i。
            //
            // 这就是经典的 abs(R) * halfExtents 技巧，一次矩阵乘法搞定，
            // 比枚举 8 个角点快得多。
            const Mat3 r = transform.rotation.ToMat3();
            const Mat3 absR(AbsPerComponent(r.rows[0]), AbsPerComponent(r.rows[1]),
                            AbsPerComponent(r.rows[2]));
            const Vec3 worldHalfExtents = absR * shape.box.halfExtents;
            return AABB(transform.position - worldHalfExtents,
                        transform.position + worldHalfExtents);
        }
    }
    return AABB(transform.position, transform.position);  // 不可达
}

}  // namespace pe
