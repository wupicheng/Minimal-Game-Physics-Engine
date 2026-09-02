//==============================================================================
// tests/test_collision_shapes.cpp
//
// M2 的形状与 AABB 测试。
//
// 重点测三类：
//   1. AABB 的布尔判定边界（贴面算不算相交 —— 宽相位宁可多报不可漏报）
//   2. 形状参数的语义（胶囊的 halfHeight 到底含不含端盖，这是最易错的一个）
//   3. ComputeWorldAABB 的**紧致性**：旋转后的盒子不能退化成包围球那么大，
//      否则宽相位会产生海量无效候选对
//==============================================================================

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <type_traits>

#include "pe/collision/AABB.h"
#include "pe/collision/GeometryUtil.h"
#include "pe/collision/Ray.h"
#include "pe/collision/Shape.h"
#include "pe/math/Quat.h"
#include "pe/math/Transform.h"

using namespace pe;
using Catch::Approx;

namespace {
constexpr real kTol = real(1e-5);

bool Eq(const Vec3& a, const Vec3& b, real tol = kTol) { return NearlyEqual(a, b, tol); }
}  // namespace

//==============================================================================
// AABB
//==============================================================================

TEST_CASE("AABB 基本量", "[collision][aabb]") {
    const AABB box(Vec3(-1, -2, -3), Vec3(1, 2, 3));

    REQUIRE(Eq(box.Center(), Vec3::Zero()));
    REQUIRE(Eq(box.HalfExtents(), Vec3(1, 2, 3)));
    REQUIRE(Eq(box.Size(), Vec3(2, 4, 6)));
    REQUIRE(box.Volume() == Approx(real(48)));
    // 2 * (2*4 + 4*6 + 6*2) = 2 * 44 = 88
    REQUIRE(box.SurfaceArea() == Approx(real(88)));
    REQUIRE(box.IsValid());
}

TEST_CASE("AABB 由中心与半尺寸构造", "[collision][aabb]") {
    const AABB box = AABB::FromCenterHalfExtents(Vec3(5, 0, 0), Vec3(1, 1, 1));
    REQUIRE(Eq(box.min, Vec3(4, -1, -1)));
    REQUIRE(Eq(box.max, Vec3(6, 1, 1)));
}

TEST_CASE("AABB::Invalid 可用于累积", "[collision][aabb][degenerate]") {
    // 空盒子的 min > max，这是刻意的：任何一次 Include 都会同时修正两端。
    AABB box = AABB::Invalid();
    REQUIRE_FALSE(box.IsValid());

    box.Include(Vec3(1, 1, 1));
    REQUIRE(box.IsValid());
    REQUIRE(Eq(box.min, Vec3(1, 1, 1)));
    REQUIRE(Eq(box.max, Vec3(1, 1, 1)));

    box.Include(Vec3(-2, 3, 0));
    REQUIRE(Eq(box.min, Vec3(-2, 1, 0)));
    REQUIRE(Eq(box.max, Vec3(1, 3, 1)));

    // 关键：累积结果不应该无端地包含原点
    REQUIRE_FALSE(box.Contains(Vec3::Zero()));
}

TEST_CASE("AABB::FromPoints", "[collision][aabb]") {
    const Vec3 pts[] = {Vec3(1, 2, 3), Vec3(-4, 0, 5), Vec3(0, -7, 1)};
    const AABB box = AABB::FromPoints(pts, 3);
    REQUIRE(Eq(box.min, Vec3(-4, -7, 1)));
    REQUIRE(Eq(box.max, Vec3(1, 2, 5)));
}

TEST_CASE("AABB 相交判定用闭区间（贴面算相交）", "[collision][aabb][convention]") {
    // 宽相位宁可多报一对（窄相位会否掉），也不能漏报。所以贴面必须算相交。
    const AABB a(Vec3(0, 0, 0), Vec3(1, 1, 1));
    const AABB touching(Vec3(1, 0, 0), Vec3(2, 1, 1));   // 恰好共面
    const AABB separated(Vec3(real(1.001), 0, 0), Vec3(2, 1, 1));
    const AABB overlapping(Vec3(real(0.5), 0, 0), Vec3(2, 1, 1));

    REQUIRE(a.Overlaps(touching));
    REQUIRE(a.Overlaps(overlapping));
    REQUIRE_FALSE(a.Overlaps(separated));

    // 对称性
    REQUIRE(touching.Overlaps(a));
    REQUIRE_FALSE(separated.Overlaps(a));

    // 自己和自己一定相交
    REQUIRE(a.Overlaps(a));
}

TEST_CASE("AABB 包含判定", "[collision][aabb]") {
    const AABB outer(Vec3(-2, -2, -2), Vec3(2, 2, 2));
    const AABB inner(Vec3(-1, -1, -1), Vec3(1, 1, 1));

    REQUIRE(outer.Contains(inner));
    REQUIRE_FALSE(inner.Contains(outer));
    REQUIRE(outer.Contains(Vec3::Zero()));
    REQUIRE(outer.Contains(Vec3(2, 2, 2)));  // 边界点算包含
    REQUIRE_FALSE(outer.Contains(Vec3(real(2.001), 0, 0)));
}

TEST_CASE("AABB 膨胀与合并", "[collision][aabb]") {
    AABB box(Vec3(0, 0, 0), Vec3(1, 1, 1));
    box.Expand(real(0.5));
    REQUIRE(Eq(box.min, Vec3(real(-0.5), real(-0.5), real(-0.5))));
    REQUIRE(Eq(box.max, Vec3(real(1.5), real(1.5), real(1.5))));

    const AABB a(Vec3(0, 0, 0), Vec3(1, 1, 1));
    const AABB b(Vec3(5, -3, 0), Vec3(6, -2, 1));
    const AABB merged = Merge(a, b);
    REQUIRE(Eq(merged.min, Vec3(0, -3, 0)));
    REQUIRE(Eq(merged.max, Vec3(6, 1, 1)));
    REQUIRE(merged.Contains(a));
    REQUIRE(merged.Contains(b));
}

TEST_CASE("AABB 扫掠覆盖整段位移", "[collision][aabb]") {
    // CCD / 运动感知 fat AABB 的基础。
    const AABB box(Vec3(0, 0, 0), Vec3(1, 1, 1));
    const AABB swept = Sweep(box, Vec3(10, 0, 0));

    REQUIRE(Eq(swept.min, Vec3(0, 0, 0)));
    REQUIRE(Eq(swept.max, Vec3(11, 1, 1)));
    REQUIRE(swept.Contains(box));
    REQUIRE(swept.Contains(AABB(Vec3(10, 0, 0), Vec3(11, 1, 1))));

    // 负方向位移同样要覆盖
    const AABB back = Sweep(box, Vec3(0, -5, 0));
    REQUIRE(Eq(back.min, Vec3(0, -5, 0)));
    REQUIRE(Eq(back.max, Vec3(1, 1, 1)));
}

TEST_CASE("AABB 最近点与距离", "[collision][aabb]") {
    const AABB box(Vec3(-1, -1, -1), Vec3(1, 1, 1));

    // 盒外：各轴分别钳制
    REQUIRE(Eq(box.ClosestPoint(Vec3(5, 0, 0)), Vec3(1, 0, 0)));
    REQUIRE(Eq(box.ClosestPoint(Vec3(5, 5, 5)), Vec3(1, 1, 1)));
    REQUIRE(box.DistanceSq(Vec3(4, 0, 0)) == Approx(real(9)));  // (4-1)^2

    // 盒内：最近点就是自己，距离为 0
    REQUIRE(Eq(box.ClosestPoint(Vec3(real(0.5), 0, 0)), Vec3(real(0.5), 0, 0)));
    REQUIRE(box.DistanceSq(Vec3::Zero()) == Approx(real(0)));

    // 只有落在区间外的轴才贡献距离
    REQUIRE(box.DistanceSq(Vec3(2, 0, 3)) == Approx(real(1) + real(4)));
}

//==============================================================================
// GeometryUtil
//==============================================================================

TEST_CASE("ClosestPointOnSegment 的钳制行为", "[collision][geometry]") {
    const Vec3 a(0, 0, 0);
    const Vec3 b(10, 0, 0);
    real t;

    // 投影落在中间
    REQUIRE(Eq(ClosestPointOnSegment(Vec3(3, 5, 0), a, b, t), Vec3(3, 0, 0)));
    REQUIRE(t == Approx(real(0.3)));

    // 投影落在 a 之前 —— 钳制到端点。这一步正是"线段"区别于"无限直线"的地方，
    // 也正是它让胶囊的半球端盖被正确处理。
    REQUIRE(Eq(ClosestPointOnSegment(Vec3(-5, 5, 0), a, b, t), a));
    REQUIRE(t == Approx(real(0)));

    // 投影落在 b 之后
    REQUIRE(Eq(ClosestPointOnSegment(Vec3(99, 5, 0), a, b, t), b));
    REQUIRE(t == Approx(real(1)));
}

TEST_CASE("ClosestPointOnSegment 退化线段不产生 NaN", "[collision][geometry][degenerate]") {
    // halfHeight 为 0 的胶囊会让线段退化成一个点
    const Vec3 p(0, 0, 0);
    real t;
    const Vec3 c = ClosestPointOnSegment(Vec3(1, 1, 1), p, p, t);
    REQUIRE(c.IsFinite());
    REQUIRE(Eq(c, p));
    REQUIRE(t == Approx(real(0)));
}

TEST_CASE("DistanceSqPointSegment", "[collision][geometry]") {
    const Vec3 a(0, -1, 0);
    const Vec3 b(0, 1, 0);

    REQUIRE(DistanceSqPointSegment(Vec3(3, 0, 0), a, b) == Approx(real(9)));
    REQUIRE(DistanceSqPointSegment(Vec3(0, 0, 0), a, b) == Approx(real(0)));
    // 端点之外：距离要算到端点，不是算到直线
    REQUIRE(DistanceSqPointSegment(Vec3(0, 4, 0), a, b) == Approx(real(9)));
}

//==============================================================================
// Shape
//==============================================================================

TEST_CASE("Shape 工厂与合法性", "[collision][shape]") {
    const Shape sphere = Shape::MakeSphere(real(2));
    REQUIRE(sphere.type == ShapeType::Sphere);
    REQUIRE(sphere.sphere.radius == Approx(real(2)));
    REQUIRE(sphere.IsValid());

    const Shape capsule = Shape::MakeCapsule(real(0.3), real(0.6));
    REQUIRE(capsule.type == ShapeType::Capsule);
    REQUIRE(capsule.IsValid());

    const Shape box = Shape::MakeBox(Vec3(1, 2, 3));
    REQUIRE(box.type == ShapeType::Box);
    REQUIRE(Eq(box.box.halfExtents, Vec3(1, 2, 3)));
    REQUIRE(box.IsValid());

    // 非法参数
    REQUIRE_FALSE(Shape::MakeSphere(real(0)).IsValid());
    REQUIRE_FALSE(Shape::MakeSphere(real(-1)).IsValid());
    REQUIRE_FALSE(Shape::MakeBox(Vec3(1, 0, 1)).IsValid());
}

TEST_CASE("胶囊的 halfHeight 不含端盖", "[collision][shape][convention]") {
    // 这是最容易搞错的一个参数。一个身高 1.8 米、半径 0.3 米的角色，
    // halfHeight 应该是 0.6（= 1.8/2 - 0.3），不是 0.9。
    const Shape c = Shape::MakeCapsuleFromHeight(real(0.3), real(1.8));
    REQUIRE(c.capsule.radius == Approx(real(0.3)));
    REQUIRE(c.capsule.halfHeight == Approx(real(0.6)));

    // 总高确实是 1.8
    REQUIRE(real(2) * (c.capsule.halfHeight + c.capsule.radius) == Approx(real(1.8)));

    // 总高不足 2*radius 时退化成球（halfHeight 被钳到 0），不能出负数
    const Shape degenerate = Shape::MakeCapsuleFromHeight(real(1), real(1));
    REQUIRE(degenerate.capsule.halfHeight == Approx(real(0)));
    REQUIRE(degenerate.IsValid());
}

TEST_CASE("Shape 包围半径", "[collision][shape]") {
    REQUIRE(Shape::MakeSphere(real(2)).BoundingRadius() == Approx(real(2)));
    // 胶囊最远点是端盖极点
    REQUIRE(Shape::MakeCapsule(real(1), real(3)).BoundingRadius() == Approx(real(4)));
    // 盒子最远点是角点
    REQUIRE(Shape::MakeBox(Vec3(3, 4, 0)).BoundingRadius() == Approx(real(5)));
}

TEST_CASE("Shape 体积对得上解析公式", "[collision][shape]") {
    const real r = real(2);
    REQUIRE(Shape::MakeSphere(r).Volume() ==
            Approx((real(4) / real(3)) * kPi * r * r * r));

    // 胶囊 = 圆柱 + 一整个球
    const real h = real(3);
    const real expected = kPi * r * r * (real(2) * h) + (real(4) / real(3)) * kPi * r * r * r;
    REQUIRE(Shape::MakeCapsule(r, h).Volume() == Approx(expected));

    // halfHeight = 0 的胶囊体积应等于同半径的球
    REQUIRE(Shape::MakeCapsule(r, real(0)).Volume() ==
            Approx(Shape::MakeSphere(r).Volume()));

    REQUIRE(Shape::MakeBox(Vec3(1, 2, 3)).Volume() == Approx(real(48)));
}

TEST_CASE("Shape 局部 AABB", "[collision][shape]") {
    REQUIRE(NearlyEqual(Shape::MakeSphere(real(2)).LocalAABB(),
                        AABB(Vec3(-2, -2, -2), Vec3(2, 2, 2)), kTol));

    // 胶囊：Y 方向多出端盖，X/Z 方向只有半径
    REQUIRE(NearlyEqual(Shape::MakeCapsule(real(1), real(3)).LocalAABB(),
                        AABB(Vec3(-1, -4, -1), Vec3(1, 4, 1)), kTol));

    REQUIRE(NearlyEqual(Shape::MakeBox(Vec3(1, 2, 3)).LocalAABB(),
                        AABB(Vec3(-1, -2, -3), Vec3(1, 2, 3)), kTol));
}

//==============================================================================
// ComputeWorldAABB —— 紧致性是这里的重点
//==============================================================================

TEST_CASE("世界 AABB：球只受平移影响", "[collision][shape][aabb]") {
    const Shape s = Shape::MakeSphere(real(2));
    // 球是各向同性的，任意旋转都不应改变结果
    const Transform t(Vec3(10, 0, -5),
                      Quat::FromAxisAngle(Vec3(1, 2, 3).Normalized(), real(1.234)));

    const AABB box = ComputeWorldAABB(s, t);
    REQUIRE(NearlyEqual(box, AABB(Vec3(8, -2, -7), Vec3(12, 2, -3)), real(1e-4)));
}

TEST_CASE("世界 AABB：盒子旋转 90 度等于交换轴", "[collision][shape][aabb]") {
    // 绕 Y 转 90 度：局部 z 轴（半长 3）转到世界 x，局部 x 轴（半长 1）转到世界 -z
    const Shape s = Shape::MakeBox(Vec3(1, 2, 3));
    const Transform t(Vec3::Zero(), Quat::FromAxisAngle(Vec3::UnitY(), kHalfPi));

    const AABB box = ComputeWorldAABB(s, t);
    REQUIRE(NearlyEqual(box, AABB(Vec3(-3, -2, -1), Vec3(3, 2, 1)), real(1e-4)));
}

TEST_CASE("世界 AABB：盒子旋转 45 度的紧致性", "[collision][shape][aabb]") {
    // 边长 2 的立方体绕 Z 转 45 度。
    // 精确解：X/Y 方向的半尺寸都是 |cos45| * 1 + |sin45| * 1 = sqrt(2) ≈ 1.41421，
    // Z 方向不变仍是 1。
    //
    // 这个用例守着"紧致"这条要求：如果哪天有人图省事把实现改成
    // "用 BoundingRadius 画个球形 AABB"，半尺寸会变成 sqrt(3) ≈ 1.732，
    // 这里立刻会挂。宽相位的候选对数量对 AABB 体积极其敏感。
    const Shape s = Shape::MakeCube(real(1));
    const Transform t(Vec3::Zero(), Quat::FromAxisAngle(Vec3::UnitZ(), DegToRad(real(45))));

    const AABB box = ComputeWorldAABB(s, t);
    const real sqrt2 = Sqrt(real(2));

    REQUIRE(box.HalfExtents().x == Approx(sqrt2).margin(real(1e-4)));
    REQUIRE(box.HalfExtents().y == Approx(sqrt2).margin(real(1e-4)));
    REQUIRE(box.HalfExtents().z == Approx(real(1)).margin(real(1e-4)));

    // 明确地比包围球小
    REQUIRE(box.HalfExtents().x < s.BoundingRadius());
}

TEST_CASE("世界 AABB：胶囊躺倒", "[collision][shape][aabb]") {
    // 胶囊默认轴沿 +Y。绕 Z 转 90 度之后轴变成 -X（或 +X，AABB 一样）。
    // 结果应该是 X 方向 halfHeight + radius，Y/Z 方向只有 radius。
    const Shape s = Shape::MakeCapsule(real(0.5), real(2));
    const Transform t(Vec3::Zero(), Quat::FromAxisAngle(Vec3::UnitZ(), kHalfPi));

    const AABB box = ComputeWorldAABB(s, t);
    REQUIRE(box.HalfExtents().x == Approx(real(2.5)).margin(real(1e-4)));
    REQUIRE(box.HalfExtents().y == Approx(real(0.5)).margin(real(1e-4)));
    REQUIRE(box.HalfExtents().z == Approx(real(0.5)).margin(real(1e-4)));
}

TEST_CASE("世界 AABB：未旋转时等于局部 AABB 加平移", "[collision][shape][aabb]") {
    const Vec3 pos(3, -4, 5);
    const Transform t(pos, Quat::Identity());

    const Shape shapes[] = {Shape::MakeSphere(real(1)),
                            Shape::MakeCapsule(real(0.4), real(1.2)),
                            Shape::MakeBox(Vec3(2, 1, real(0.5)))};

    for (const Shape& s : shapes) {
        const AABB local = s.LocalAABB();
        const AABB world = ComputeWorldAABB(s, t);
        REQUIRE(NearlyEqual(world, AABB(local.min + pos, local.max + pos), real(1e-4)));
    }
}

TEST_CASE("GetCapsuleSegment 端点正确", "[collision][shape]") {
    const Shape s = Shape::MakeCapsule(real(0.5), real(2));

    // 未旋转：沿 Y
    Vec3 a, b;
    GetCapsuleSegment(s, Transform(Vec3(1, 1, 1), Quat::Identity()), a, b);
    REQUIRE(Eq(a, Vec3(1, -1, 1), real(1e-5)));
    REQUIRE(Eq(b, Vec3(1, 3, 1), real(1e-5)));

    // 绕 Z 转 90 度：+Y 变成 -X
    GetCapsuleSegment(s, Transform(Vec3::Zero(), Quat::FromAxisAngle(Vec3::UnitZ(), kHalfPi)),
                      a, b);
    REQUIRE(Eq(a, Vec3(2, 0, 0), real(1e-4)));
    REQUIRE(Eq(b, Vec3(-2, 0, 0), real(1e-4)));
}

//==============================================================================
// POD 性质
//==============================================================================

TEST_CASE("碰撞类型保持平凡可复制", "[collision][pod]") {
    // Shape 里用了 union，而 union 的成员必须是平凡默认构造的 ——
    // 这正是 M1 里把 Vec3 的默认构造写成 `= default`（不初始化）的原因。
    STATIC_REQUIRE(std::is_trivially_copyable_v<Shape>);
    STATIC_REQUIRE(std::is_trivially_default_constructible_v<Shape>);
    STATIC_REQUIRE(std::is_trivially_copyable_v<AABB>);
    STATIC_REQUIRE(std::is_trivially_default_constructible_v<AABB>);
    STATIC_REQUIRE(std::is_trivially_copyable_v<Ray>);
    STATIC_REQUIRE(std::is_trivially_copyable_v<RaycastHit>);
    STATIC_REQUIRE(std::is_standard_layout_v<AABB>);
    STATIC_REQUIRE(sizeof(AABB) == 6 * sizeof(real));
}
