//==============================================================================
// tests/test_raycast.cpp
//
// M2 的射线求交测试。
//
// 射线检测在 FPS 里是最高频、也最容易"偶尔出错"的东西 —— 一条子弹偶尔打不中
// 或者穿墙，玩家会立刻感觉到，但很难复现。所以这里除了逐个形状的定点用例，
// 还做了两类更强的检查：
//
//   A. **交叉验证**：同一个几何体用两条独立的代码路径算，结果必须一致。
//        - halfHeight = 0 的胶囊  ==  同半径的球
//        - 旋转 90 度的 OBB       ==  交换过轴的 AABB
//        - RaycastShape 的分派    ==  直接调用具体函数
//      两条路径同时写错同一个 bug 的概率极低，这比和硬编码数字比对可靠得多。
//
//   B. **不变量的随机测试**：撒几千条随机射线，凡是报命中的，
//      命中点必须真的落在该形状的表面上（|到表面的距离| ≈ 0），
//      法线必须是单位向量且朝着射线来的方向。
//      这类检查能抓到定点用例覆盖不到的角落（掠射、贴边、内部起点）。
//==============================================================================

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cstdint>

#include "pe/collision/GeometryUtil.h"
#include "pe/collision/RayCast.h"
#include "pe/math/Quat.h"

using namespace pe;
using Catch::Approx;

namespace {

constexpr real kTol = real(1e-4);

bool Eq(const Vec3& a, const Vec3& b, real tol = kTol) { return NearlyEqual(a, b, tol); }

/// 确定性的线性同余随机数发生器。
/// 刻意不用 std::mt19937：测试必须每次跑出完全一样的结果，
/// 失败时才能靠种子精确复现。
struct Rng {
    std::uint32_t state;

    explicit Rng(std::uint32_t seed) : state(seed) {}

    std::uint32_t NextU32() {
        state = state * 1664525u + 1013904223u;
        return state;
    }

    /// [0, 1) 区间的均匀分布。取高 24 位（float 的有效位数就是 24 位）。
    real Unit() { return real(NextU32() >> 8) / real(1 << 24); }

    real Range(real lo, real hi) { return lo + (hi - lo) * Unit(); }

    Vec3 InCube(real half) {
        return Vec3(Range(-half, half), Range(-half, half), Range(-half, half));
    }

    /// 球面上的均匀方向（拒绝采样，避免立方体角点方向偏多）。
    Vec3 Direction() {
        for (int i = 0; i < 32; ++i) {
            const Vec3 v = InCube(real(1));
            const real lenSq = v.LengthSq();
            if (lenSq > real(0.01) && lenSq <= real(1)) {
                return v * (real(1) / Sqrt(lenSq));
            }
        }
        return Vec3::UnitX();
    }
};

/// 生成一条**朝目标附近发射**的随机射线。
///
/// 为什么不用"随机起点 + 完全随机方向"：那样撒 1000 条也只有个位数能打中一个
/// 半径 1 米的物体，随机测试等于在空跑。这里让射线从目标周围的球壳上出发、
/// 朝目标附近抖动一个偏移量射过去，命中率能到 30%~60%，
/// 而 spread 稍大于物体尺寸又能保证仍有相当比例的擦边与不中样本 ——
/// 恰好覆盖了掠射、贴边这些最容易出错的角落。
Ray AimedRay(Rng& rng, const Vec3& target, real spread, real originDistance) {
    const Vec3 origin =
        target + rng.Direction() * rng.Range(originDistance * real(0.6), originDistance);
    const Vec3 aim = target + rng.InCube(spread);
    return Ray(origin, aim - origin, real(100));
}

/// 命中结果的通用不变量检查（对所有形状都必须成立）。
void CheckHitInvariants(const Ray& ray, const RaycastHit& hit) {
    INFO("distance = " << hit.distance);

    // 距离落在合法区间内
    REQUIRE(hit.distance >= real(0));
    REQUIRE(hit.distance <= ray.maxDistance + kTol);

    // 命中点必须真的在射线上
    REQUIRE(Eq(hit.point, ray.PointAt(hit.distance), real(1e-3)));

    // 法线是单位向量
    REQUIRE(hit.normal.Length() == Approx(real(1)).margin(real(1e-3)));

    // 法线朝向射线来的一侧。对凸形状这永远成立：
    // 射线从外面打进来，第一个碰到的面必然是背对射线方向的。
    REQUIRE(Dot(hit.normal, ray.direction) < real(1e-3));
}

}  // namespace

//==============================================================================
// 射线 vs 球
//==============================================================================

TEST_CASE("射线 vs 球：正面命中", "[raycast][sphere]") {
    const Vec3 center(10, 0, 0);
    const Ray ray(Vec3::Zero(), Vec3::UnitX());
    RaycastHit hit;

    REQUIRE(RaycastSphere(center, real(2), ray, hit));
    REQUIRE(hit.distance == Approx(real(8)));            // 10 - 2
    REQUIRE(Eq(hit.point, Vec3(8, 0, 0)));
    REQUIRE(Eq(hit.normal, -Vec3::UnitX()));             // 表面朝向射线来的方向
    CheckHitInvariants(ray, hit);
}

TEST_CASE("射线 vs 球：擦过去不算命中", "[raycast][sphere]") {
    const Vec3 center(10, 0, 0);
    RaycastHit hit;

    // 偏移量刚好超过半径
    REQUIRE_FALSE(RaycastSphere(center, real(2),
                                Ray(Vec3(0, real(2.001), 0), Vec3::UnitX()), hit));
    // 刚好在半径内则命中
    REQUIRE(RaycastSphere(center, real(2), Ray(Vec3(0, real(1.999), 0), Vec3::UnitX()),
                          hit));
}

TEST_CASE("射线 vs 球：球在身后不算命中", "[raycast][sphere]") {
    RaycastHit hit;
    // 球在 +X，射线朝 -X
    REQUIRE_FALSE(RaycastSphere(Vec3(10, 0, 0), real(2),
                                Ray(Vec3::Zero(), -Vec3::UnitX()), hit));
}

TEST_CASE("射线 vs 球：起点在内部返回距离 0", "[raycast][sphere][convention]") {
    // 见 Ray.h 的约定 2：玩家贴脸开枪时宁可"立即命中"，也不能让子弹穿出去。
    const Ray ray(Vec3::Zero(), Vec3::UnitX());
    RaycastHit hit;

    REQUIRE(RaycastSphere(Vec3::Zero(), real(5), ray, hit));
    REQUIRE(hit.distance == Approx(real(0)));
    REQUIRE(Eq(hit.point, ray.origin));
    REQUIRE(Eq(hit.normal, -ray.direction));
}

TEST_CASE("射线 vs 球：射程限制", "[raycast][sphere]") {
    const Vec3 center(10, 0, 0);
    RaycastHit hit;

    // 命中点在 t = 8
    REQUIRE_FALSE(RaycastSphere(center, real(2),
                                Ray(Vec3::Zero(), Vec3::UnitX(), real(7.9)), hit));
    REQUIRE(RaycastSphere(center, real(2),
                          Ray(Vec3::Zero(), Vec3::UnitX(), real(8.1)), hit));
}

//==============================================================================
// 射线 vs AABB
//==============================================================================

TEST_CASE("射线 vs AABB：六个面的法线", "[raycast][aabb][convention]") {
    const AABB box(Vec3(-1, -1, -1), Vec3(1, 1, 1));
    RaycastHit hit;

    struct Case {
        Vec3 origin;
        Vec3 dir;
        Vec3 expectedNormal;
    };
    const Case cases[] = {
        {Vec3(-5, 0, 0), Vec3::UnitX(), -Vec3::UnitX()},
        {Vec3(5, 0, 0), -Vec3::UnitX(), Vec3::UnitX()},
        {Vec3(0, -5, 0), Vec3::UnitY(), -Vec3::UnitY()},
        {Vec3(0, 5, 0), -Vec3::UnitY(), Vec3::UnitY()},
        {Vec3(0, 0, -5), Vec3::UnitZ(), -Vec3::UnitZ()},
        {Vec3(0, 0, 5), -Vec3::UnitZ(), Vec3::UnitZ()},
    };

    for (const Case& c : cases) {
        const Ray ray(c.origin, c.dir);
        REQUIRE(RaycastAABB(box, ray, hit));
        REQUIRE(hit.distance == Approx(real(4)));
        REQUIRE(Eq(hit.normal, c.expectedNormal));
        CheckHitInvariants(ray, hit);
    }
}

TEST_CASE("射线 vs AABB：平行于某对面时的处理", "[raycast][aabb][degenerate]") {
    // 射线平行于 Y 方向的板。这一支是专门写来避免 1/0 得 inf、
    // 进而 0*inf 得 NaN 的，必须两种情况都覆盖。
    const AABB box(Vec3(-1, -1, -1), Vec3(1, 1, 1));
    RaycastHit hit;

    // 平行且落在板内 -> 应该命中
    REQUIRE(RaycastAABB(box, Ray(Vec3(-5, real(0.5), 0), Vec3::UnitX()), hit));
    REQUIRE(hit.distance == Approx(real(4)));

    // 平行且落在板外 -> 不可能命中
    REQUIRE_FALSE(RaycastAABB(box, Ray(Vec3(-5, real(1.5), 0), Vec3::UnitX()), hit));
    REQUIRE(hit.point.IsFinite());  // 即使不中也不该产生 NaN 残留
}

TEST_CASE("射线 vs AABB：起点在盒内", "[raycast][aabb][convention]") {
    const AABB box(Vec3(-1, -1, -1), Vec3(1, 1, 1));
    const Ray ray(Vec3::Zero(), Vec3(1, 1, 1).Normalized());
    RaycastHit hit;

    REQUIRE(RaycastAABB(box, ray, hit));
    REQUIRE(hit.distance == Approx(real(0)));
    REQUIRE(Eq(hit.normal, -ray.direction));
}

TEST_CASE("射线 vs AABB：斜射与射程", "[raycast][aabb]") {
    const AABB box(Vec3(0, 0, -1), Vec3(2, 2, 1));
    RaycastHit hit;

    // 从 (-2, 1, 0) 朝 +X 打，在 x = 0 处进入，距离 2
    const Ray ray(Vec3(-2, 1, 0), Vec3::UnitX());
    REQUIRE(RaycastAABB(box, ray, hit));
    REQUIRE(hit.distance == Approx(real(2)));
    REQUIRE(Eq(hit.normal, -Vec3::UnitX()));

    // 射程不够
    REQUIRE_FALSE(RaycastAABB(box, Ray(Vec3(-2, 1, 0), Vec3::UnitX(), real(1.9)), hit));
}

//==============================================================================
// 射线 vs OBB
//==============================================================================

TEST_CASE("OBB 不旋转时与 AABB 完全一致", "[raycast][obb]") {
    // 交叉验证 1：OBB 走的是"变换到局部空间再复用 slab"的路径，
    // 单位旋转下它必须和直接的 AABB 求交给出同样的结果。
    const Vec3 he(1, 2, 3);
    const Vec3 center(4, -1, 2);
    const AABB box = AABB::FromCenterHalfExtents(center, he);
    const Transform t(center, Quat::Identity());

    Rng rng(12345);
    int hitCount = 0;
    for (int i = 0; i < 500; ++i) {
        const Ray ray = AimedRay(rng, center, real(5), real(15));

        RaycastHit a, b;
        const bool hitA = RaycastAABB(box, ray, a);
        const bool hitB = RaycastOBB(t, he, ray, b);

        REQUIRE(hitA == hitB);
        if (hitA) {
            ++hitCount;
            REQUIRE(a.distance == Approx(b.distance).margin(real(1e-3)));
            REQUIRE(Eq(a.point, b.point, real(1e-3)));
            REQUIRE(Eq(a.normal, b.normal, real(1e-3)));
        }
    }
    REQUIRE(hitCount > 50);  // 确认样本里确实有足够多的命中，不是空跑
}

TEST_CASE("OBB 旋转 90 度等价于交换轴的 AABB", "[raycast][obb]") {
    // 交叉验证 2：局部半尺寸 (1,2,3) 的盒子绕 Y 转 90 度之后，
    // 几何上就是半尺寸 (3,2,1) 的轴对齐盒子。两条路径必须给出一样的答案，
    // 包括法线（局部 +Z 面的法线转过去正好是世界 +X）。
    const Vec3 heLocal(1, 2, 3);
    const Vec3 heWorld(3, 2, 1);
    const Transform t(Vec3::Zero(), Quat::FromAxisAngle(Vec3::UnitY(), kHalfPi));
    const AABB equivalent(-heWorld, heWorld);

    Rng rng(777);
    int hitCount = 0;
    for (int i = 0; i < 500; ++i) {
        const Ray ray = AimedRay(rng, Vec3::Zero(), real(5), real(15));

        RaycastHit a, b;
        const bool hitA = RaycastAABB(equivalent, ray, a);
        const bool hitB = RaycastOBB(t, heLocal, ray, b);

        REQUIRE(hitA == hitB);
        if (hitA) {
            ++hitCount;
            REQUIRE(a.distance == Approx(b.distance).margin(real(1e-3)));
            REQUIRE(Eq(a.normal, b.normal, real(1e-3)));
        }
    }
    REQUIRE(hitCount > 50);
}

TEST_CASE("OBB 命中点确实落在盒面上", "[raycast][obb]") {
    const Vec3 he(real(0.5), real(1.5), real(2));
    const Transform t(Vec3(2, 3, -1),
                      Quat::FromAxisAngle(Vec3(real(0.3), real(1), real(-0.5)).Normalized(),
                                          real(0.9)));

    Rng rng(2024);
    int hitCount = 0;
    for (int i = 0; i < 1000; ++i) {
        const Ray ray = AimedRay(rng, t.position, real(3), real(12));
        RaycastHit hit;
        if (!RaycastOBB(t, he, ray, hit)) {
            continue;
        }
        ++hitCount;
        CheckHitInvariants(ray, hit);

        if (hit.distance <= real(0)) {
            continue;  // 起点在内部，命中点就是起点，不在表面上
        }

        // 把命中点变回盒子局部空间，它必须落在某个面上：
        // 至少一个轴的 |坐标| 等于半尺寸，且没有任何轴超出半尺寸。
        const Vec3 local = t.InverseTransformPoint(hit.point);
        real maxOverflow = real(-1);
        for (int axis = 0; axis < 3; ++axis) {
            maxOverflow = Max(maxOverflow, Abs(local[axis]) - he[axis]);
        }
        INFO("local = (" << local.x << ", " << local.y << ", " << local.z << ")");
        REQUIRE(maxOverflow == Approx(real(0)).margin(real(1e-3)));
    }
    REQUIRE(hitCount > 100);
}

//==============================================================================
// 射线 vs 胶囊
//==============================================================================

TEST_CASE("射线 vs 胶囊：打侧面", "[raycast][capsule]") {
    // 竖直胶囊，半径 1，圆柱段半长 2。从 -X 方向水平打向腰部。
    const Transform t = Transform::Identity();
    const Ray ray(Vec3(-10, 0, 0), Vec3::UnitX());
    RaycastHit hit;

    REQUIRE(RaycastCapsule(t, real(1), real(2), ray, hit));
    REQUIRE(hit.distance == Approx(real(9)));       // 10 - 1
    REQUIRE(Eq(hit.point, Vec3(-1, 0, 0)));
    REQUIRE(Eq(hit.normal, -Vec3::UnitX()));        // 侧面法线是径向的，没有 Y 分量
    CheckHitInvariants(ray, hit);
}

TEST_CASE("射线 vs 胶囊：打端盖", "[raycast][capsule]") {
    // 从正上方往下打，应该命中上端盖的极点 y = halfHeight + radius = 3
    const Ray ray(Vec3(0, 10, 0), -Vec3::UnitY());
    RaycastHit hit;

    REQUIRE(RaycastCapsule(Transform::Identity(), real(1), real(2), ray, hit));
    REQUIRE(hit.distance == Approx(real(7)));
    REQUIRE(Eq(hit.point, Vec3(0, 3, 0)));
    REQUIRE(Eq(hit.normal, Vec3::UnitY()));
    CheckHitInvariants(ray, hit);
}

TEST_CASE("射线 vs 胶囊：从圆柱段侧上方擦过端盖", "[raycast][capsule]") {
    // 这条射线在 y = 2.5 的高度水平飞过 —— 已经超过圆柱段（y <= 2），
    // 所以只可能打到上端盖球面，不能被"无限圆柱"误判。
    // 上端盖球心 (0,2,0) 半径 1，在 y=2.5 的截面上半径是 sqrt(1 - 0.25) ≈ 0.866。
    const Ray ray(Vec3(-10, real(2.5), 0), Vec3::UnitX());
    RaycastHit hit;

    REQUIRE(RaycastCapsule(Transform::Identity(), real(1), real(2), ray, hit));
    const real expectedX = -Sqrt(real(1) - real(0.25));
    REQUIRE(hit.point.x == Approx(expectedX).margin(real(1e-3)));
    REQUIRE(hit.point.y == Approx(real(2.5)).margin(real(1e-3)));
    // 法线必须有 +Y 分量（在球盖上，不是在圆柱侧面上）
    REQUIRE(hit.normal.y > real(0.1));
    CheckHitInvariants(ray, hit);
}

TEST_CASE("射线 vs 胶囊：超出端盖高度就打不中", "[raycast][capsule]") {
    RaycastHit hit;
    // y = 3.001 已经超过顶点 y = 3
    REQUIRE_FALSE(RaycastCapsule(Transform::Identity(), real(1), real(2),
                                 Ray(Vec3(-10, real(3.001), 0), Vec3::UnitX()), hit));
    // y = 2.999 还在里面
    REQUIRE(RaycastCapsule(Transform::Identity(), real(1), real(2),
                           Ray(Vec3(-10, real(2.999), 0), Vec3::UnitX()), hit));
}

TEST_CASE("射线 vs 胶囊：起点在内部", "[raycast][capsule][convention]") {
    const Ray ray(Vec3(0, 1, 0), Vec3::UnitX());
    RaycastHit hit;

    REQUIRE(RaycastCapsule(Transform::Identity(), real(1), real(2), ray, hit));
    REQUIRE(hit.distance == Approx(real(0)));
    REQUIRE(Eq(hit.normal, -ray.direction));
}

TEST_CASE("射线 vs 胶囊：沿轴方向射入", "[raycast][capsule][degenerate]") {
    // 射线平行于胶囊轴时，圆柱侧面的二次项系数 a 为 0，
    // 代码必须走"只考虑端盖"的分支而不是除以 0。
    const Ray ray(Vec3(0, 10, 0), -Vec3::UnitY());
    RaycastHit hit;
    REQUIRE(RaycastCapsule(Transform::Identity(), real(1), real(2), ray, hit));
    REQUIRE(hit.point.IsFinite());
    REQUIRE(hit.distance == Approx(real(7)));

    // 平行于轴但偏出半径之外 -> 不中，且不产生 NaN
    REQUIRE_FALSE(RaycastCapsule(Transform::Identity(), real(1), real(2),
                                 Ray(Vec3(real(1.5), 10, 0), -Vec3::UnitY()), hit));
}

TEST_CASE("halfHeight=0 的胶囊等价于球", "[raycast][capsule]") {
    // 交叉验证 3：胶囊退化成球时，两条完全不同的代码路径
    //（胶囊走"圆柱 + 两端盖"，球走二次方程）必须给出一致的结果。
    const real r = real(1.5);
    const Vec3 center(1, -2, 3);
    const Transform t(center, Quat::FromAxisAngle(Vec3(1, 1, 0).Normalized(), real(0.7)));

    Rng rng(31337);
    int hitCount = 0;
    for (int i = 0; i < 1000; ++i) {
        const Ray ray = AimedRay(rng, center, real(3), real(12));

        RaycastHit a, b;
        const bool hitA = RaycastSphere(center, r, ray, a);
        const bool hitB = RaycastCapsule(t, r, real(0), ray, b);

        INFO("i = " << i);
        REQUIRE(hitA == hitB);
        if (hitA) {
            ++hitCount;
            REQUIRE(a.distance == Approx(b.distance).margin(real(1e-3)));
            REQUIRE(Eq(a.point, b.point, real(1e-2)));
            REQUIRE(Eq(a.normal, b.normal, real(1e-2)));
        }
    }
    REQUIRE(hitCount > 100);
}

TEST_CASE("胶囊命中点确实落在表面上", "[raycast][capsule]") {
    // 不变量测试：命中点到中轴线段的距离必须恰好等于半径。
    // 这一条同时覆盖了侧面和两个端盖 —— 胶囊表面的定义本来就是
    // "到线段距离 = r 的点集"，所以这是最本质的验证。
    const real r = real(0.8);
    const real hh = real(1.6);
    const Transform t(Vec3(1, 2, -3),
                      Quat::FromAxisAngle(Vec3(real(0.2), real(0.5), real(1)).Normalized(),
                                          real(1.1)));

    Vec3 segA, segB;
    GetCapsuleSegment(Shape::MakeCapsule(r, hh), t, segA, segB);

    Rng rng(90210);
    int hitCount = 0;
    for (int i = 0; i < 2000; ++i) {
        const Ray ray = AimedRay(rng, t.position, real(3), real(12));
        RaycastHit hit;
        if (!RaycastCapsule(t, r, hh, ray, hit)) {
            continue;
        }
        ++hitCount;
        CheckHitInvariants(ray, hit);

        if (hit.distance <= real(0)) {
            continue;  // 起点在内部
        }

        INFO("i = " << i << " distance = " << hit.distance);
        const real dist = Sqrt(DistanceSqPointSegment(hit.point, segA, segB));
        REQUIRE(dist == Approx(r).margin(real(2e-3)));
    }
    REQUIRE(hitCount > 100);
}

//==============================================================================
// 分派
//==============================================================================

TEST_CASE("RaycastShape 的分派与直接调用一致", "[raycast][dispatch]") {
    const Transform t(Vec3(2, -1, 4),
                      Quat::FromAxisAngle(Vec3(1, 2, 3).Normalized(), real(0.6)));

    const Shape sphere = Shape::MakeSphere(real(1.2));
    const Shape capsule = Shape::MakeCapsule(real(0.5), real(1));
    const Shape box = Shape::MakeBox(Vec3(1, real(0.6), real(1.4)));

    Rng rng(4242);
    int total = 0;
    for (int i = 0; i < 600; ++i) {
        const Ray ray = AimedRay(rng, t.position, real(3), real(10));

        RaycastHit direct, dispatched;

        REQUIRE(RaycastSphere(t.position, sphere.sphere.radius, ray, direct) ==
                RaycastShape(sphere, t, ray, dispatched));

        REQUIRE(RaycastCapsule(t, capsule.capsule.radius, capsule.capsule.halfHeight, ray,
                               direct) == RaycastShape(capsule, t, ray, dispatched));

        if (RaycastOBB(t, box.box.halfExtents, ray, direct)) {
            REQUIRE(RaycastShape(box, t, ray, dispatched));
            REQUIRE(direct.distance == Approx(dispatched.distance).margin(real(1e-4)));
            ++total;
        } else {
            REQUIRE_FALSE(RaycastShape(box, t, ray, dispatched));
        }
    }
    REQUIRE(total > 20);
}

TEST_CASE("球的射线检测不受旋转影响", "[raycast][dispatch]") {
    // 球是各向同性的，RaycastShape 对球刻意只用了 transform.position。
    // 这个用例守着那个优化：换任意旋转，结果必须一模一样。
    const Shape sphere = Shape::MakeSphere(real(2));
    const Ray ray(Vec3(-10, real(0.5), real(0.3)), Vec3::UnitX());

    RaycastHit a, b;
    REQUIRE(RaycastShape(sphere, Transform(Vec3(1, 0, 0), Quat::Identity()), ray, a));
    REQUIRE(RaycastShape(
        sphere,
        Transform(Vec3(1, 0, 0),
                  Quat::FromAxisAngle(Vec3(real(0.3), real(-1), real(0.7)).Normalized(),
                                      real(2.2))),
        ray, b));

    REQUIRE(a.distance == Approx(b.distance));
    REQUIRE(Eq(a.point, b.point));
    REQUIRE(Eq(a.normal, b.normal));
}

//==============================================================================
// Ray 本身
//==============================================================================

TEST_CASE("Ray 构造函数归一化方向", "[raycast][ray][convention]") {
    // direction 必须是单位向量，否则 distance 就不是"米"了。
    const Ray ray(Vec3::Zero(), Vec3(3, 4, 0));
    REQUIRE(ray.direction.Length() == Approx(real(1)));
    REQUIRE(Eq(ray.direction, Vec3(real(0.6), real(0.8), real(0))));
    REQUIRE(Eq(ray.PointAt(real(5)), Vec3(3, 4, 0)));
}

TEST_CASE("Ray 零方向退化为不命中任何东西", "[raycast][ray][degenerate]") {
    const Ray ray(Vec3::Zero(), Vec3::Zero());
    REQUIRE(ray.direction.IsFinite());
    REQUIRE(ray.maxDistance == Approx(real(0)));

    // 射程为 0，除了"起点已在内部"之外不该命中任何东西
    RaycastHit hit;
    REQUIRE_FALSE(RaycastSphere(Vec3(10, 0, 0), real(1), ray, hit));
}

TEST_CASE("Ray::FromTo 的射程恰好是两点距离", "[raycast][ray]") {
    // 视线遮挡判定的标准写法：目标之后的墙不能算作遮挡。
    const Ray ray = Ray::FromTo(Vec3::Zero(), Vec3(0, 0, 10));
    REQUIRE(ray.maxDistance == Approx(real(10)));
    REQUIRE(Eq(ray.direction, Vec3::UnitZ()));

    RaycastHit hit;
    // 障碍物在目标之前 -> 挡住
    REQUIRE(RaycastSphere(Vec3(0, 0, 5), real(1), ray, hit));
    // 障碍物在目标之后 -> 不算挡住
    REQUIRE_FALSE(RaycastSphere(Vec3(0, 0, 15), real(1), ray, hit));
}
