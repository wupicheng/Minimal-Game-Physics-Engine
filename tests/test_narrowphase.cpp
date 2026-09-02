//==============================================================================
// tests/test_narrowphase.cpp
//
// M4 窄相位（解析闭式解）的测试。
//
//------------------------------------------------------------------------------
// 窄相位错了会怎样
//------------------------------------------------------------------------------
// 窄相位的输出直接喂给求解器，而求解器会把它的每一个错误放大成可见的行为：
//   - 法线反了       -> 物体互相吸进去，穿模
//   - 穿透深度算大了 -> 物体被弹飞
//   - 接触点太少     -> 箱子放地上会自己转起来然后倒下
//   - 漏报接触       -> 穿墙
// 而且这些症状都是"偶尔出现"的，靠玩游戏很难复现。所以这里的测试分四层：
//
//   A. **约定**：法线是单位向量、由 A 指向 B、穿透深度为正。每个形状对都验。
//   B. **构造用例**：手工摆出穿透深度已知的构型，逐个核对数值。
//   C. **交叉验证**：同一个几何体走两条独立的代码路径，结果必须一致。
//        - halfHeight = 0 的胶囊 == 同半径的球（三处：球-胶囊、胶囊-胶囊、胶囊-盒）
//        - Collide(A,B) 与 Collide(B,A) 的对称性
//        - Collide() 与 Overlap() 的一致性
//   D. **随机不变量**，其中最强的一条是：
//
//        沿法线把 B 推开 penetration + eps 之后，两者必须真的不再重叠。
//
//      这一条同时钉死了法线方向和穿透深度 —— 任何一个错了它都会失败。
//      它还是**独立**的检查：验证用的是 Overlap()，走的不是生成流形的那条路径。
//      另外用"随机撒点找公共点"做无漏报检查，那是一个和窄相位完全无关的判据。
//==============================================================================

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cstdint>

#include "pe/collision/GeometryUtil.h"
#include "pe/collision/NarrowPhase.h"
#include "pe/math/Quat.h"

using namespace pe;
using Catch::Approx;

namespace {

constexpr real kTol = real(1e-4);

bool Eq(const Vec3& a, const Vec3& b, real tol = kTol) { return NearlyEqual(a, b, tol); }

//------------------------------------------------------------------------------
// 确定性随机数（与 test_raycast / test_broadphase 同一套）
//------------------------------------------------------------------------------
struct Rng {
    std::uint32_t state;

    explicit Rng(std::uint32_t seed) : state(seed) {}

    std::uint32_t NextU32() {
        state = state * 1664525u + 1013904223u;
        return state;
    }
    real Unit() { return real(NextU32() >> 8) / real(1 << 24); }
    real Range(real lo, real hi) { return lo + (hi - lo) * Unit(); }
    std::uint32_t Below(std::uint32_t n) { return NextU32() % n; }
    bool Chance(real p) { return Unit() < p; }

    Vec3 InCube(real half) {
        return Vec3(Range(-half, half), Range(-half, half), Range(-half, half));
    }

    Quat Rotation() {
        // 均匀随机四元数（Shoemake 的方法）
        const real u1 = Unit();
        const real u2 = Unit();
        const real u3 = Unit();
        const real s1 = Sqrt(real(1) - u1);
        const real s2 = Sqrt(u1);
        return Quat(s1 * Sin(kTwoPi * u2), s1 * Cos(kTwoPi * u2), s2 * Sin(kTwoPi * u3),
                    s2 * Cos(kTwoPi * u3));
    }

    Shape AnyShape() {
        switch (Below(3)) {
            case 0: return Shape::MakeSphere(Range(real(0.2), real(1)));
            case 1: return Shape::MakeCapsule(Range(real(0.2), real(0.6)),
                                              Range(real(0.1), real(1)));
            default: return Shape::MakeBox(Vec3(Range(real(0.2), real(1)),
                                                Range(real(0.2), real(1)),
                                                Range(real(0.2), real(1))));
        }
    }
};

//------------------------------------------------------------------------------
// 独立的判据：点在不在形状里
//
// 刻意不复用窄相位的任何代码（除了取胶囊线段这个纯粹的坐标换算），
// 这样"撒点找到公共点却报不碰撞"才是一个真正独立的漏报检查。
//------------------------------------------------------------------------------
bool PointInShape(const Shape& s, const Transform& t, const Vec3& p) {
    switch (s.type) {
        case ShapeType::Sphere:
            return (p - t.position).LengthSq() <= s.sphere.radius * s.sphere.radius;
        case ShapeType::Capsule: {
            Vec3 a, b;
            GetCapsuleSegment(s, t, a, b);
            return DistanceSqPointSegment(p, a, b) <=
                   s.capsule.radius * s.capsule.radius;
        }
        case ShapeType::Box: {
            const Vec3 l = t.InverseTransformPoint(p);
            const Vec3 h = s.box.halfExtents;
            return Abs(l.x) <= h.x && Abs(l.y) <= h.y && Abs(l.z) <= h.z;
        }
    }
    return false;
}

Transform At(const Vec3& p) { return Transform(p, Quat::Identity()); }
Transform At(const Vec3& p, const Quat& q) { return Transform(p, q); }

/// 把形状 s 沿 dir 平移 d 之后的位姿
Transform Shifted(const Transform& t, const Vec3& dir, real d) {
    return Transform(t.position + dir * d, t.rotation);
}

//------------------------------------------------------------------------------
// 通用约定检查：任何一个报了接触的流形都必须满足
//------------------------------------------------------------------------------
void CheckManifoldConventions(const Manifold& m) {
    REQUIRE(m.pointCount >= 1);
    REQUIRE(m.pointCount <= 4);
    REQUIRE(NearlyEqual(m.normal.Length(), real(1), real(1e-3)));
    for (std::uint8_t i = 0; i < m.pointCount; ++i) {
        // 允许轻微为负（推测性接触），但不该负得离谱
        REQUIRE(m.points[i].penetration >= -kSpeculativeMargin - kTol);
        REQUIRE(IsFinite(m.points[i].penetration));
        REQUIRE(IsFinite(m.points[i].position.x));
        REQUIRE(IsFinite(m.points[i].position.y));
        REQUIRE(IsFinite(m.points[i].position.z));
    }
}

}  // namespace

//==============================================================================
// A. 球 vs 球
//==============================================================================

TEST_CASE("窄相位 球vs球", "[narrowphase][collision]") {
    const Shape s1 = Shape::MakeSphere(real(1));
    const Shape s2 = Shape::MakeSphere(real(0.5));
    Manifold m;

    SECTION("分离时不报接触") {
        REQUIRE_FALSE(Collide(s1, At(Vec3(0, 0, 0)), s2, At(Vec3(5, 0, 0)), m));
        REQUIRE(m.pointCount == 0);  // 返回 false 时流形必须是干净的
    }

    SECTION("重叠：穿透深度与法线") {
        // 半径和 1.5，圆心距 1.2 -> 重叠 0.3
        REQUIRE(Collide(s1, At(Vec3(0, 0, 0)), s2, At(Vec3(real(1.2), 0, 0)), m));
        CheckManifoldConventions(m);

        REQUIRE(m.pointCount == 1);
        REQUIRE(Eq(m.normal, Vec3(1, 0, 0)));  // 由 A 指向 B
        REQUIRE(m.points[0].penetration == Approx(real(0.3)).margin(kTol));

        // 见证点：A 表面 x=1，B 表面 x=1.2-0.5=0.7，中点 x=0.85
        REQUIRE(Eq(m.points[0].position, Vec3(real(0.85), 0, 0)));
    }

    SECTION("恰好相切：推测性接触，深度为 0") {
        REQUIRE(Collide(s1, At(Vec3(0, 0, 0)), s2, At(Vec3(real(1.5), 0, 0)), m));
        REQUIRE(m.points[0].penetration == Approx(real(0)).margin(kTol));
    }

    SECTION("刚刚超出推测范围就不报了") {
        const real justOutside = real(1.5) + kSpeculativeMargin + real(0.001);
        REQUIRE_FALSE(Collide(s1, At(Vec3(0, 0, 0)), s2, At(Vec3(justOutside, 0, 0)), m));
    }

    SECTION("圆心重合：退化但不产生 NaN") {
        REQUIRE(Collide(s1, At(Vec3(0, 0, 0)), s2, At(Vec3(0, 0, 0)), m));
        CheckManifoldConventions(m);
        REQUIRE(m.points[0].penetration == Approx(real(1.5)).margin(kTol));
    }

    SECTION("交换 A B：法线取反，接触点不变") {
        Manifold m2;
        REQUIRE(Collide(s1, At(Vec3(0, 0, 0)), s2, At(Vec3(real(1.2), 0, 0)), m));
        REQUIRE(Collide(s2, At(Vec3(real(1.2), 0, 0)), s1, At(Vec3(0, 0, 0)), m2));

        REQUIRE(Eq(m2.normal, -m.normal));
        REQUIRE(Eq(m2.points[0].position, m.points[0].position));
        REQUIRE(m2.points[0].penetration == Approx(m.points[0].penetration).margin(kTol));
    }
}

//==============================================================================
// B. 球 vs 胶囊
//==============================================================================

TEST_CASE("窄相位 球vs胶囊", "[narrowphase][collision]") {
    // 沿 +Y 的胶囊：圆柱段半长 1，半径 0.5 -> 总高 3
    const Shape cap = Shape::MakeCapsule(real(0.5), real(1));
    const Shape sph = Shape::MakeSphere(real(0.5));
    Manifold m;

    SECTION("从侧面撞圆柱段") {
        // 球心 (0.8, 0.5, 0)：到轴线距离 0.8，半径和 1.0 -> 重叠 0.2
        REQUIRE(Collide(sph, At(Vec3(real(0.8), real(0.5), 0)), cap, At(Vec3(0, 0, 0)), m));
        CheckManifoldConventions(m);
        REQUIRE(m.points[0].penetration == Approx(real(0.2)).margin(kTol));
        // 法线由球指向胶囊轴线，即 -X
        REQUIRE(Eq(m.normal, Vec3(-1, 0, 0)));
    }

    SECTION("撞上端盖：退化成球vs球") {
        // 轴线端点在 (0,1,0)。球心 (0,1.8,0)：距离 0.8 -> 重叠 0.2
        REQUIRE(Collide(sph, At(Vec3(0, real(1.8), 0)), cap, At(Vec3(0, 0, 0)), m));
        REQUIRE(m.points[0].penetration == Approx(real(0.2)).margin(kTol));
        REQUIRE(Eq(m.normal, Vec3(0, -1, 0)));
    }

    SECTION("halfHeight=0 的胶囊必须等价于同半径的球") {
        // 交叉验证：两条完全独立的代码路径（球-胶囊 vs 球-球）
        const Shape degenerate = Shape::MakeCapsule(real(0.5), real(0));
        const Shape equivalent = Shape::MakeSphere(real(0.5));

        Rng rng(777u);
        for (int i = 0; i < 200; ++i) {
            const Transform ta = At(rng.InCube(real(1.5)));
            const Transform tb = At(rng.InCube(real(1.5)), rng.Rotation());

            Manifold viaCapsule, viaSphere;
            const bool hitCap = Collide(sph, ta, degenerate, tb, viaCapsule);
            const bool hitSph = Collide(sph, ta, equivalent, tb, viaSphere);

            REQUIRE(hitCap == hitSph);
            if (!hitCap) continue;
            REQUIRE(Eq(viaCapsule.normal, viaSphere.normal, real(1e-3)));
            REQUIRE(viaCapsule.points[0].penetration ==
                    Approx(viaSphere.points[0].penetration).margin(kTol));
            REQUIRE(Eq(viaCapsule.points[0].position, viaSphere.points[0].position));
        }
    }

    SECTION("球心正好落在轴线上：法线垂直于轴线") {
        REQUIRE(Collide(sph, At(Vec3(0, real(0.3), 0)), cap, At(Vec3(0, 0, 0)), m));
        CheckManifoldConventions(m);
        // 最短的逃逸方向是侧向，不是沿轴顶出去
        REQUIRE(Abs(Dot(m.normal, Vec3(0, 1, 0))) < real(1e-3));
    }
}

//==============================================================================
// C. 球 vs 盒
//==============================================================================

TEST_CASE("窄相位 球vs盒", "[narrowphase][collision]") {
    const Shape box = Shape::MakeCube(real(1));
    const Shape sph = Shape::MakeSphere(real(0.5));
    Manifold m;

    SECTION("撞面中心") {
        // 球心 (1.3,0,0)：到 +X 面距离 0.3，半径 0.5 -> 重叠 0.2
        REQUIRE(Collide(sph, At(Vec3(real(1.3), 0, 0)), box, At(Vec3(0, 0, 0)), m));
        CheckManifoldConventions(m);
        REQUIRE(m.points[0].penetration == Approx(real(0.2)).margin(kTol));
        REQUIRE(Eq(m.normal, Vec3(-1, 0, 0)));  // 由球指向盒
        // 见证点：球面 x=0.8，盒面 x=1，中点 x=0.9
        REQUIRE(Eq(m.points[0].position, Vec3(real(0.9), 0, 0)));
    }

    SECTION("撞角") {
        // 球心 (1.2,1.2,1.2)，最近点是角 (1,1,1)，距离 sqrt(3)*0.2 ≈ 0.3464
        const real dist = Sqrt(real(3)) * real(0.2);
        REQUIRE(Collide(sph, At(Vec3(real(1.2), real(1.2), real(1.2))), box,
                        At(Vec3(0, 0, 0)), m));
        REQUIRE(m.points[0].penetration == Approx(real(0.5) - dist).margin(kTol));
        REQUIRE(Eq(m.normal, Vec3(-1, -1, -1).Normalized(), real(1e-3)));
    }

    SECTION("球心在盒内：从最近的那个面推出去") {
        // 球心 (0.9, 0.1, 0)：离 +X 面 0.1，离其它面都更远
        REQUIRE(Collide(sph, At(Vec3(real(0.9), real(0.1), 0)), box, At(Vec3(0, 0, 0)), m));
        CheckManifoldConventions(m);
        REQUIRE(m.points[0].penetration == Approx(real(0.5) + real(0.1)).margin(kTol));
        // 法线指向盒内（约定是由 A 指向 B），也就是 -X
        REQUIRE(Eq(m.normal, Vec3(-1, 0, 0)));
    }

    SECTION("球心正好在盒心：不产生 NaN") {
        REQUIRE(Collide(sph, At(Vec3(0, 0, 0)), box, At(Vec3(0, 0, 0)), m));
        CheckManifoldConventions(m);
        REQUIRE(m.points[0].penetration == Approx(real(1.5)).margin(kTol));
    }

    SECTION("绕 Y 转 45 度的盒子") {
        const Quat rot = Quat::FromAxisAngle(Vec3(0, 1, 0), DegToRad(real(45)));
        // 转 45 度之后，盒子在 X 方向的最远点是原来的角，距离 sqrt(2)
        // 球心放在 (sqrt(2)+0.3, 0, 0)：到那个角的距离 0.3，重叠 0.2
        const real corner = Sqrt(real(2));
        REQUIRE(Collide(sph, At(Vec3(corner + real(0.3), 0, 0)), box,
                        At(Vec3(0, 0, 0), rot), m));
        REQUIRE(m.points[0].penetration == Approx(real(0.2)).margin(kTol));
        REQUIRE(Eq(m.normal, Vec3(-1, 0, 0), real(1e-3)));
    }

    SECTION("远处不报接触") {
        REQUIRE_FALSE(Collide(sph, At(Vec3(5, 0, 0)), box, At(Vec3(0, 0, 0)), m));
    }
}

//==============================================================================
// D. 胶囊 vs 胶囊
//==============================================================================

TEST_CASE("窄相位 胶囊vs胶囊", "[narrowphase][collision]") {
    const Shape cap = Shape::MakeCapsule(real(0.5), real(1));  // 沿 +Y
    Manifold m;

    SECTION("两根平行竖直胶囊侧面相碰") {
        // 轴距 0.8，半径和 1.0 -> 重叠 0.2
        REQUIRE(Collide(cap, At(Vec3(0, 0, 0)), cap, At(Vec3(real(0.8), 0, 0)), m));
        CheckManifoldConventions(m);
        REQUIRE(m.points[0].penetration == Approx(real(0.2)).margin(kTol));
        REQUIRE(Eq(m.normal, Vec3(1, 0, 0)));
    }

    SECTION("横着的胶囊压在竖着的胶囊头顶") {
        // B 绕 Z 转 90 度变成沿 X。A 的轴线上端是 (0,1,0)，
        // 把 B 抬到 y=1.8，两条轴线的最近距离就是 0.8 -> 重叠 0.2。
        //
        // 注意 y 不能取 0.8：那样 B 的轴线会**穿过** A 的轴线（A 的轴线覆盖
        // y ∈ [-1,1]），最近距离变成 0，穿透深度是整个半径和。
        const Quat rot = Quat::FromAxisAngle(Vec3(0, 0, 1), DegToRad(real(90)));
        REQUIRE(Collide(cap, At(Vec3(0, 0, 0)), cap, At(Vec3(0, real(1.8), 0), rot), m));
        CheckManifoldConventions(m);
        REQUIRE(m.points[0].penetration == Approx(real(0.2)).margin(kTol));
        REQUIRE(Eq(m.normal, Vec3(0, 1, 0), real(1e-3)));
    }

    SECTION("十字交叉：两条轴线真的相交时穿透深度是整个半径和") {
        const Quat rot = Quat::FromAxisAngle(Vec3(0, 0, 1), DegToRad(real(90)));
        REQUIRE(Collide(cap, At(Vec3(0, 0, 0)), cap, At(Vec3(0, real(0.8), 0), rot), m));
        CheckManifoldConventions(m);
        REQUIRE(m.points[0].penetration == Approx(real(1)).margin(kTol));
    }

    SECTION("端盖对端盖") {
        // A 的上端点 (0,1,0)，B 的下端点在 (0, 2.8-1, 0) = (0,1.8,0)：间距 0.8
        REQUIRE(Collide(cap, At(Vec3(0, 0, 0)), cap, At(Vec3(0, real(2.8), 0)), m));
        REQUIRE(m.points[0].penetration == Approx(real(0.2)).margin(kTol));
        REQUIRE(Eq(m.normal, Vec3(0, 1, 0)));
    }

    SECTION("两根都退化成球时等价于球vs球") {
        const Shape degen = Shape::MakeCapsule(real(0.4), real(0));
        const Shape sph = Shape::MakeSphere(real(0.4));

        Rng rng(31337u);
        for (int i = 0; i < 200; ++i) {
            const Transform ta = At(rng.InCube(real(1)), rng.Rotation());
            const Transform tb = At(rng.InCube(real(1)), rng.Rotation());

            Manifold viaCapsule, viaSphere;
            const bool h1 = Collide(degen, ta, degen, tb, viaCapsule);
            const bool h2 = Collide(sph, ta, sph, tb, viaSphere);
            REQUIRE(h1 == h2);
            if (!h1) continue;
            REQUIRE(viaCapsule.points[0].penetration ==
                    Approx(viaSphere.points[0].penetration).margin(kTol));
            REQUIRE(Eq(viaCapsule.points[0].position, viaSphere.points[0].position));
        }
    }

    SECTION("轴线相交：法线垂直于两条轴线") {
        const Quat rot = Quat::FromAxisAngle(Vec3(0, 0, 1), DegToRad(real(90)));
        REQUIRE(Collide(cap, At(Vec3(0, 0, 0)), cap, At(Vec3(0, 0, 0), rot), m));
        CheckManifoldConventions(m);
        // 两条轴线分别沿 Y 和 X，公垂线是 Z
        REQUIRE(Abs(Abs(m.normal.z) - real(1)) < real(1e-3));
    }
}

//==============================================================================
// E. 胶囊 vs 盒
//==============================================================================

TEST_CASE("窄相位 胶囊vs盒", "[narrowphase][collision]") {
    const Shape cap = Shape::MakeCapsule(real(0.5), real(1));  // 沿 +Y，总高 3
    const Shape box = Shape::MakeBox(Vec3(2, 1, 2));
    Manifold m;

    SECTION("角色站在平台上") {
        // 盒顶面 y=1。胶囊底端盖球心在 y = center-1，表面在 center-1.5。
        // 取 center = 2.4 -> 表面在 0.9，陷进去 0.1
        REQUIRE(Collide(cap, At(Vec3(0, real(2.4), 0)), box, At(Vec3(0, 0, 0)), m));
        CheckManifoldConventions(m);
        REQUIRE(m.points[0].penetration == Approx(real(0.1)).margin(kTol));
        REQUIRE(Eq(m.normal, Vec3(0, -1, 0)));  // 由胶囊指向盒
        // 接触点在见证点的中点：胶囊表面 y=0.9，盒面 y=1 -> y=0.95
        REQUIRE(Eq(m.points[0].position, Vec3(0, real(0.95), 0)));
    }

    SECTION("贴着侧墙") {
        // 盒的 +X 面在 x=2。胶囊中心 (2.4, 0, 0)，到面距离 0.4，半径 0.5 -> 重叠 0.1
        REQUIRE(Collide(cap, At(Vec3(real(2.4), 0, 0)), box, At(Vec3(0, 0, 0)), m));
        REQUIRE(m.points[0].penetration == Approx(real(0.1)).margin(kTol));
        REQUIRE(Eq(m.normal, Vec3(-1, 0, 0)));
    }

    SECTION("halfHeight=0 的胶囊必须等价于同半径的球") {
        const Shape degen = Shape::MakeCapsule(real(0.5), real(0));
        const Shape sph = Shape::MakeSphere(real(0.5));

        Rng rng(555u);
        for (int i = 0; i < 300; ++i) {
            const Transform ta = At(rng.InCube(real(2.5)), rng.Rotation());
            const Transform tb = At(rng.InCube(real(1)), rng.Rotation());

            Manifold viaCapsule, viaSphere;
            const bool h1 = Collide(degen, ta, box, tb, viaCapsule);
            const bool h2 = Collide(sph, ta, box, tb, viaSphere);

            INFO("iteration " << i);
            REQUIRE(h1 == h2);
            if (!h1) continue;
            REQUIRE(Eq(viaCapsule.normal, viaSphere.normal, real(1e-3)));
            REQUIRE(viaCapsule.points[0].penetration ==
                    Approx(viaSphere.points[0].penetration).margin(kTol));
            REQUIRE(Eq(viaCapsule.points[0].position, viaSphere.points[0].position));
        }
    }

    SECTION("斜着搭在盒子角上") {
        const Quat rot = Quat::FromAxisAngle(Vec3(0, 0, 1), DegToRad(real(45)));
        REQUIRE(Collide(cap, At(Vec3(real(2.2), real(1.2), 0), rot), box,
                        At(Vec3(0, 0, 0)), m));
        CheckManifoldConventions(m);
    }

    SECTION("完全穿过盒子也不崩") {
        // 一根很长的胶囊横穿盒子中心
        const Shape longCap = Shape::MakeCapsule(real(0.3), real(5));
        REQUIRE(Collide(longCap, At(Vec3(0, 0, 0)), box, At(Vec3(0, 0, 0)), m));
        CheckManifoldConventions(m);
        REQUIRE(m.points[0].penetration > real(0));
    }
}

//==============================================================================
// F. 盒 vs 盒
//==============================================================================

TEST_CASE("窄相位 盒vs盒", "[narrowphase][collision]") {
    const Shape box = Shape::MakeCube(real(1));
    Manifold m;

    SECTION("面对面：应该给出 4 个等深的接触点") {
        // A 在原点，B 在 (1.8,0,0) -> 沿 X 重叠 0.2
        REQUIRE(Collide(box, At(Vec3(0, 0, 0)), box, At(Vec3(real(1.8), 0, 0)), m));
        CheckManifoldConventions(m);

        // 这是关键：单点接触撑不住堆叠，必须裁出整个接触面
        REQUIRE(m.pointCount == 4);
        REQUIRE(Eq(m.normal, Vec3(1, 0, 0)));
        for (std::uint8_t i = 0; i < m.pointCount; ++i) {
            REQUIRE(m.points[i].penetration == Approx(real(0.2)).margin(kTol));
            // 接触点在两个面的中间：A 面 x=1，B 面 x=0.8
            REQUIRE(m.points[i].position.x == Approx(real(0.9)).margin(kTol));
        }
    }

    SECTION("四个接触点各不相同，张成一块面积") {
        REQUIRE(Collide(box, At(Vec3(0, 0, 0)), box, At(Vec3(real(1.8), 0, 0)), m));
        REQUIRE(m.pointCount == 4);
        for (std::uint8_t i = 0; i < m.pointCount; ++i) {
            for (std::uint8_t j = static_cast<std::uint8_t>(i + 1); j < m.pointCount;
                 ++j) {
                REQUIRE_FALSE(Eq(m.points[i].position, m.points[j].position, real(1e-3)));
            }
        }
    }

    SECTION("上下堆叠") {
        REQUIRE(Collide(box, At(Vec3(0, 0, 0)), box, At(Vec3(0, real(1.9), 0)), m));
        REQUIRE(m.pointCount == 4);
        REQUIRE(Eq(m.normal, Vec3(0, 1, 0)));
        for (std::uint8_t i = 0; i < m.pointCount; ++i) {
            REQUIRE(m.points[i].penetration == Approx(real(0.1)).margin(kTol));
        }
    }

    SECTION("小盒子压在大盒子上：接触面由小盒子决定") {
        const Shape small = Shape::MakeCube(real(0.3));
        REQUIRE(Collide(box, At(Vec3(0, 0, 0)), small, At(Vec3(0, real(1.25), 0)), m));
        REQUIRE(m.pointCount == 4);
        REQUIRE(Eq(m.normal, Vec3(0, 1, 0)));
        for (std::uint8_t i = 0; i < m.pointCount; ++i) {
            REQUIRE(m.points[i].penetration == Approx(real(0.05)).margin(kTol));
            // 裁剪的结果不能超出小盒子的范围
            REQUIRE(Abs(m.points[i].position.x) <= real(0.3) + kTol);
            REQUIRE(Abs(m.points[i].position.z) <= real(0.3) + kTol);
        }
    }

    SECTION("绕 Y 转 45 度：角顶着面") {
        const Quat rot = Quat::FromAxisAngle(Vec3(0, 1, 0), DegToRad(real(45)));
        const real corner = Sqrt(real(2));  // 转过之后的半宽
        // 间距 1 + corner - 0.2 -> 沿 X 重叠 0.2
        REQUIRE(Collide(box, At(Vec3(0, 0, 0)), box,
                        At(Vec3(real(1) + corner - real(0.2), 0, 0), rot), m));
        CheckManifoldConventions(m);
        REQUIRE(Eq(m.normal, Vec3(1, 0, 0), real(1e-3)));
        REQUIRE(m.MaxPenetration() == Approx(real(0.2)).margin(real(1e-3)));
    }

    SECTION("分离时不报接触") {
        REQUIRE_FALSE(Collide(box, At(Vec3(0, 0, 0)), box, At(Vec3(5, 0, 0)), m));
        REQUIRE(m.pointCount == 0);
    }

    SECTION("只在一根轴上错开一点点也算分离") {
        REQUIRE_FALSE(Collide(box, At(Vec3(0, 0, 0)), box,
                              At(Vec3(real(1.5), real(2.5), 0)), m));
    }

    SECTION("交换 A B：法线取反、穿透深度不变") {
        Manifold m2;
        const Quat rot = Quat::FromAxisAngle(Vec3(1, 1, 0).Normalized(),
                                             DegToRad(real(30)));
        REQUIRE(Collide(box, At(Vec3(0, 0, 0)), box, At(Vec3(real(1.6), real(0.4), 0), rot), m));
        REQUIRE(Collide(box, At(Vec3(real(1.6), real(0.4), 0), rot), box, At(Vec3(0, 0, 0)), m2));

        // 盒-盒不保证接触点集合相同（参考面的选取带偏置），
        // 但法线和穿透深度必须是对称的
        REQUIRE(Eq(m2.normal, -m.normal, real(1e-3)));
        REQUIRE(m2.MaxPenetration() == Approx(m.MaxPenetration()).margin(real(1e-3)));
    }
}

//==============================================================================
// G. Overlap 与 Collide 的一致性
//==============================================================================

TEST_CASE("窄相位 Overlap 与 Collide 一致", "[narrowphase][collision]") {
    Rng rng(0xBEEFu);
    int overlapping = 0;

    for (int i = 0; i < 4000; ++i) {
        const Shape sa = rng.AnyShape();
        const Shape sb = rng.AnyShape();
        const Transform ta = At(rng.InCube(real(1.5)), rng.Rotation());
        const Transform tb = At(rng.InCube(real(1.5)), rng.Rotation());

        Manifold m;
        const bool collided = Collide(sa, ta, sb, tb, m);
        const bool overlaps = Overlap(sa, ta, sb, tb);

        INFO("iteration " << i);

        // Overlap 为真 => 一定重叠 => Collide 必须报接触（Collide 的判定还更宽松）
        if (overlaps) {
            REQUIRE(collided);
            ++overlapping;
        }

        // Collide 报出了正的穿透深度 => 确实重叠 => Overlap 必须为真
        if (collided && m.MaxPenetration() > kSpeculativeMargin) {
            REQUIRE(overlaps);
        }
    }

    // 确认样本里真的有大量重叠，不是"两边都恒为假"的假通过
    REQUIRE(overlapping > 500);
}

//==============================================================================
// H. 随机不变量 —— 最强的一层
//==============================================================================

TEST_CASE("窄相位 沿法线推开穿透深度之后必须分离", "[narrowphase][collision]") {
    // 这一条同时钉死了**法线方向**和**穿透深度**：
    //   - 法线反了 -> 推的方向不对，推完只会陷得更深
    //   - 深度算小了 -> 推完还在重叠
    //   - 深度算大了 -> 这条测不出来，由上面的构造用例负责
    // 而且验证用的是 Overlap()，走的不是生成流形的那条代码路径。
    Rng rng(20260901u);
    int checked = 0;

    for (int i = 0; i < 6000; ++i) {
        const Shape sa = rng.AnyShape();
        const Shape sb = rng.AnyShape();
        const Transform ta = At(rng.InCube(real(1.2)), rng.Rotation());
        const Transform tb = At(rng.InCube(real(1.2)), rng.Rotation());

        Manifold m;
        if (!Collide(sa, ta, sb, tb, m)) continue;

        const real depth = m.MaxPenetration();
        if (depth <= real(0)) continue;  // 推测性接触，本来就没重叠

        // 胶囊-盒在"线段整个插进盒子里"时是采样近似（见 NarrowPhase.cpp 的说明），
        // 那种深度穿透下的 MTV 不保证精确，跳过。
        // 判据：穿透深度超过胶囊半径就说明轴线已经进到盒子内部了。
        const bool capsuleBoxDeep =
            ((sa.type == ShapeType::Capsule && sb.type == ShapeType::Box &&
              depth >= sa.capsule.radius) ||
             (sb.type == ShapeType::Capsule && sa.type == ShapeType::Box &&
              depth >= sb.capsule.radius));
        if (capsuleBoxDeep) continue;

        // 沿法线把 B 推开（法线由 A 指向 B），多推一点点容差
        const Transform tbPushed = Shifted(tb, m.normal, depth + real(1e-3));

        INFO("iteration " << i << " depth " << depth << " types "
                          << static_cast<int>(sa.type) << "/"
                          << static_cast<int>(sb.type));
        REQUIRE_FALSE(Overlap(sa, ta, sb, tbPushed));
        ++checked;
    }

    REQUIRE(checked > 1000);
}

TEST_CASE("窄相位 有公共点就必须报接触", "[narrowphase][collision]") {
    // 完全独立的无漏报检查：随便撒点，只要找到一个同时落在两个形状里的点，
    // 那它们就是真的重叠了，窄相位没有任何借口报不碰撞。
    // PointInShape 不依赖窄相位的任何代码。
    Rng rng(24680u);
    int confirmed = 0;

    for (int i = 0; i < 1500; ++i) {
        const Shape sa = rng.AnyShape();
        const Shape sb = rng.AnyShape();
        const Transform ta = At(rng.InCube(real(1)), rng.Rotation());
        const Transform tb = At(rng.InCube(real(1)), rng.Rotation());

        bool foundCommon = false;
        for (int k = 0; k < 60 && !foundCommon; ++k) {
            // 在 A 的包围盒里撒点，这样命中率高一些
            const AABB ba = ComputeWorldAABB(sa, ta);
            const Vec3 p(rng.Range(ba.min.x, ba.max.x), rng.Range(ba.min.y, ba.max.y),
                         rng.Range(ba.min.z, ba.max.z));
            foundCommon = PointInShape(sa, ta, p) && PointInShape(sb, tb, p);
        }

        if (!foundCommon) continue;

        Manifold m;
        INFO("iteration " << i);
        REQUIRE(Collide(sa, ta, sb, tb, m));
        CheckManifoldConventions(m);
        ++confirmed;
    }

    REQUIRE(confirmed > 200);
}

TEST_CASE("窄相位 交换顺序的对称性", "[narrowphase][collision]") {
    Rng rng(112233u);
    int checked = 0;

    for (int i = 0; i < 3000; ++i) {
        const Shape sa = rng.AnyShape();
        const Shape sb = rng.AnyShape();
        const Transform ta = At(rng.InCube(real(1.2)), rng.Rotation());
        const Transform tb = At(rng.InCube(real(1.2)), rng.Rotation());

        Manifold ab, ba;
        const bool h1 = Collide(sa, ta, sb, tb, ab);
        const bool h2 = Collide(sb, tb, sa, ta, ba);

        INFO("iteration " << i);
        REQUIRE(h1 == h2);
        if (!h1) continue;

        REQUIRE(Eq(ab.normal, -ba.normal, real(1e-3)));
        REQUIRE(ab.MaxPenetration() == Approx(ba.MaxPenetration()).margin(real(1e-3)));

        // 盒-盒之外的形状对还要求接触点完全相同（见 NarrowPhase.h 的说明）
        const bool boxBox = sa.type == ShapeType::Box && sb.type == ShapeType::Box;
        if (!boxBox) {
            REQUIRE(ab.pointCount == ba.pointCount);
            for (std::uint8_t k = 0; k < ab.pointCount; ++k) {
                REQUIRE(Eq(ab.points[k].position, ba.points[k].position, real(1e-3)));
            }
        }
        ++checked;
    }

    REQUIRE(checked > 500);
}

//==============================================================================
// I. 退化输入
//==============================================================================

TEST_CASE("窄相位 退化输入不产生 NaN", "[narrowphase][collision][degenerate]") {
    Manifold m;

    SECTION("零半径的球") {
        const Shape zero = Shape::MakeSphere(real(0));
        const Shape box = Shape::MakeCube(real(1));
        REQUIRE(Collide(zero, At(Vec3(0, 0, 0)), box, At(Vec3(0, 0, 0)), m));
        CheckManifoldConventions(m);
    }

    SECTION("退化成薄片的盒子") {
        const Shape flat = Shape::MakeBox(Vec3(1, real(0), 1));
        const Shape box = Shape::MakeCube(real(1));
        REQUIRE(Collide(flat, At(Vec3(0, 0, 0)), box, At(Vec3(0, real(0.5), 0)), m));
        CheckManifoldConventions(m);
    }

    SECTION("两个完全重合的盒子") {
        const Shape box = Shape::MakeCube(real(1));
        REQUIRE(Collide(box, At(Vec3(0, 0, 0)), box, At(Vec3(0, 0, 0)), m));
        CheckManifoldConventions(m);
        REQUIRE(m.MaxPenetration() == Approx(real(2)).margin(kTol));
    }

    SECTION("两个完全重合的胶囊") {
        const Shape cap = Shape::MakeCapsule(real(0.5), real(1));
        REQUIRE(Collide(cap, At(Vec3(0, 0, 0)), cap, At(Vec3(0, 0, 0)), m));
        CheckManifoldConventions(m);
    }
}
