//==============================================================================
// tests/test_gjk.cpp
//
// M5 GJK + EPA 的测试。
//
//------------------------------------------------------------------------------
// 怎么测一个迭代算法
//------------------------------------------------------------------------------
// GJK/EPA 和前面几个模块不一样：它是**迭代**的，没有闭式解可以逐项核对。
// 但它有一件别的模块没有的好事 —— M4 的解析解算的是同一件事，而且已经被
// 构造用例和随机不变量钉死了。所以这一层的主心骨就是**对拍**：
//
//     同一对形状，SAT/闭式解 那条路 与 GJK/EPA 这条路，
//     法线方向与穿透深度必须一致。
//
// 这也正是 ARCHITECTURE.md 第 5 节给 M5 定的验收条件。
//
// 除此之外还有三类：
//   A. **距离查询**：这是解析解给不了的能力（它们只在接触时才有输出），
//      所以只能用手工构造的、距离已知的构型来验。
//   B. **单纯形最近点**：GJK 的地基，纯几何、可以脱离迭代单独验证。
//      它错了整个 GJK 都是错的，而且症状会表现为"偶尔不收敛"这种极难查的问题。
//   C. **退化输入**：核完全重合、退化成薄片、零半径。迭代算法在这些输入上
//      最容易死循环或者返回 NaN。
//==============================================================================

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cstdint>

#include "pe/collision/EPA.h"
#include "pe/collision/GJK.h"
#include "pe/collision/NarrowPhase.h"
#include "pe/math/Quat.h"

using namespace pe;
using Catch::Approx;

namespace {

constexpr real kTol = real(1e-4);

bool Eq(const Vec3& a, const Vec3& b, real tol = kTol) { return NearlyEqual(a, b, tol); }

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
    Vec3 InCube(real half) {
        return Vec3(Range(-half, half), Range(-half, half), Range(-half, half));
    }
    Quat Rotation() {
        const real u1 = Unit(), u2 = Unit(), u3 = Unit();
        const real s1 = Sqrt(real(1) - u1), s2 = Sqrt(u1);
        return Quat(s1 * Sin(kTwoPi * u2), s1 * Cos(kTwoPi * u2), s2 * Sin(kTwoPi * u3),
                    s2 * Cos(kTwoPi * u3));
    }
    Shape AnyShape() {
        switch (Below(3)) {
            case 0: return Shape::MakeSphere(Range(real(0.2), real(1)));
            case 1:
                return Shape::MakeCapsule(Range(real(0.2), real(0.6)),
                                          Range(real(0.1), real(1)));
            default:
                return Shape::MakeBox(Vec3(Range(real(0.2), real(1)),
                                           Range(real(0.2), real(1)),
                                           Range(real(0.2), real(1))));
        }
    }
};

Transform At(const Vec3& p) { return Transform(p, Quat::Identity()); }
Transform At(const Vec3& p, const Quat& q) { return Transform(p, q); }

}  // namespace

//==============================================================================
// A. 支撑函数
//==============================================================================

TEST_CASE("GJK 支撑函数", "[gjk][collision]") {
    SECTION("球的核是一个点") {
        const ConvexProxy p(Shape::MakeSphere(real(1)), At(Vec3(3, 0, 0)));
        REQUIRE(p.CoreRadius() == Approx(real(1)));
        // 不管朝哪儿问，核的支撑点都是球心 —— 半径是事后加的
        REQUIRE(Eq(p.SupportCore(Vec3(1, 0, 0)), Vec3(3, 0, 0)));
        REQUIRE(Eq(p.SupportCore(Vec3(-1, -1, 5)), Vec3(3, 0, 0)));
    }

    SECTION("胶囊的核是一条线段") {
        const ConvexProxy p(Shape::MakeCapsule(real(0.5), real(2)), At(Vec3(0, 0, 0)));
        REQUIRE(p.CoreRadius() == Approx(real(0.5)));
        REQUIRE(Eq(p.SupportCore(Vec3(0, 1, 0)), Vec3(0, 2, 0)));
        REQUIRE(Eq(p.SupportCore(Vec3(0, -1, 0)), Vec3(0, -2, 0)));
        // 垂直于轴线时两端一样远，取哪端都对，但必须是端点之一
        const Vec3 s = p.SupportCore(Vec3(1, 0, 0));
        REQUIRE((Eq(s, Vec3(0, 2, 0)) || Eq(s, Vec3(0, -2, 0))));
    }

    SECTION("盒的核就是盒本身：支撑点是某个角") {
        const ConvexProxy p(Shape::MakeBox(Vec3(1, 2, 3)), At(Vec3(0, 0, 0)));
        REQUIRE(p.CoreRadius() == Approx(real(0)));
        REQUIRE(Eq(p.SupportCore(Vec3(1, 1, 1)), Vec3(1, 2, 3)));
        REQUIRE(Eq(p.SupportCore(Vec3(-1, 1, -1)), Vec3(-1, 2, -3)));
    }

    SECTION("旋转之后支撑点跟着转") {
        const Quat rot = Quat::FromAxisAngle(Vec3(0, 0, 1), DegToRad(real(90)));
        const ConvexProxy p(Shape::MakeCapsule(real(0.5), real(2)),
                            At(Vec3(0, 0, 0), rot));
        // +Y 轴转 90 度变成 -X
        REQUIRE(Eq(p.SupportCore(Vec3(-1, 0, 0)), Vec3(-2, 0, 0), real(1e-3)));
    }
}

//==============================================================================
// B. 距离查询 —— 解析解给不了的能力
//==============================================================================

TEST_CASE("GJK 距离查询", "[gjk][collision]") {
    Vec3 pa, pb;

    SECTION("两个球") {
        const Shape s = Shape::MakeSphere(real(1));
        // 圆心距 5，半径各 1 -> 表面间距 3
        const real d = ConvexDistance(s, At(Vec3(0, 0, 0)), s, At(Vec3(5, 0, 0)), pa, pb);
        REQUIRE(d == Approx(real(3)).margin(kTol));
        REQUIRE(Eq(pa, Vec3(1, 0, 0)));
        REQUIRE(Eq(pb, Vec3(4, 0, 0)));
    }

    SECTION("两个轴对齐的盒子") {
        const Shape box = Shape::MakeCube(real(1));
        // 面在 x=1 与 x=4 -> 间距 3
        const real d =
            ConvexDistance(box, At(Vec3(0, 0, 0)), box, At(Vec3(5, 0, 0)), pa, pb);
        REQUIRE(d == Approx(real(3)).margin(kTol));
        REQUIRE(pa.x == Approx(real(1)).margin(kTol));
        REQUIRE(pb.x == Approx(real(4)).margin(kTol));
    }

    SECTION("盒的角对角") {
        const Shape box = Shape::MakeCube(real(1));
        // A 的角 (1,1,1)，B 的角 (3,3,3)，间距 sqrt(3)*2
        const real d =
            ConvexDistance(box, At(Vec3(0, 0, 0)), box, At(Vec3(4, 4, 4)), pa, pb);
        REQUIRE(d == Approx(Sqrt(real(3)) * real(2)).margin(real(1e-3)));
        REQUIRE(Eq(pa, Vec3(1, 1, 1), real(1e-3)));
        REQUIRE(Eq(pb, Vec3(3, 3, 3), real(1e-3)));
    }

    SECTION("球到盒") {
        const Shape sph = Shape::MakeSphere(real(0.5));
        const Shape box = Shape::MakeCube(real(1));
        // 球心 (3,0,0)，盒面 x=1 -> 核距离 2，减去球半径 -> 1.5
        const real d =
            ConvexDistance(sph, At(Vec3(3, 0, 0)), box, At(Vec3(0, 0, 0)), pa, pb);
        REQUIRE(d == Approx(real(1.5)).margin(kTol));
        REQUIRE(Eq(pa, Vec3(real(2.5), 0, 0)));
        REQUIRE(Eq(pb, Vec3(1, 0, 0)));
    }

    SECTION("胶囊到盒") {
        const Shape cap = Shape::MakeCapsule(real(0.5), real(1));  // 沿 +Y
        const Shape box = Shape::MakeCube(real(1));
        // 胶囊中心 (3,0,0)，轴线到盒面 x=1 的距离是 2，减半径 -> 1.5
        const real d =
            ConvexDistance(cap, At(Vec3(3, 0, 0)), box, At(Vec3(0, 0, 0)), pa, pb);
        REQUIRE(d == Approx(real(1.5)).margin(kTol));
    }

    SECTION("相交时距离为 0") {
        const Shape box = Shape::MakeCube(real(1));
        const real d =
            ConvexDistance(box, At(Vec3(0, 0, 0)), box, At(Vec3(real(1.5), 0, 0)), pa, pb);
        REQUIRE(d == Approx(real(0)).margin(kTol));
    }

    SECTION("距离与解析的射线/最近点结论一致（随机对拍）") {
        // 独立判据：两个形状的距离 d 意味着把 B 沿 (pb - pa) 方向拉开 d 之后
        // 恰好接触。用 Overlap() 检验："拉开 d 的九成还重叠 / 拉开 d 多一点就不重叠"
        // 对分离的构型不成立（本来就没重叠），所以这里换个角度：
        // 把 B 朝 A 推 d + 一点点，必须变成重叠。
        Rng rng(9182736u);
        int checked = 0;
        for (int i = 0; i < 2000; ++i) {
            const Shape sa = rng.AnyShape();
            const Shape sb = rng.AnyShape();
            const Transform ta = At(rng.InCube(real(2)), rng.Rotation());
            const Transform tb = At(rng.InCube(real(2)), rng.Rotation());

            Vec3 qa, qb;
            const real d = ConvexDistance(sa, ta, sb, tb, qa, qb);
            if (d <= kTol) continue;  // 已经接触，跳过

            const Vec3 dir = (qa - qb).Normalized();
            if (dir.IsZero()) continue;

            INFO("iteration " << i << " distance " << d);
            // 推 d 的九成：还不该碰上
            REQUIRE_FALSE(
                Overlap(sa, ta, sb, Transform(tb.position + dir * (d * real(0.9)),
                                              tb.rotation)));
            // 推 d 多一点：必须碰上
            REQUIRE(Overlap(sa, ta, sb,
                            Transform(tb.position + dir * (d + real(1e-3)),
                                      tb.rotation)));
            ++checked;
        }
        REQUIRE(checked > 500);
    }
}

//==============================================================================
// C. M5 的验收测试 —— 与 M4 解析解对拍
//==============================================================================

TEST_CASE("GJK/EPA 与解析解对拍：盒vs盒", "[gjk][collision]") {
    // ARCHITECTURE.md 第 5 节给 M5 定的验收条件就是这一条。
    //
    // 盒-盒是唯一两条路径都走"真正的算法"的组合：解析解那边是 SAT + 面裁剪，
    // 这边是 GJK + EPA，两者在实现上没有任何共享代码。它们对不上就说明有一边错了，
    // 而解析解那一边已经被 M4 的构造用例钉死了。
    Rng rng(51413u);
    int deep = 0;

    for (int i = 0; i < 3000; ++i) {
        const Shape ba = Shape::MakeBox(Vec3(rng.Range(real(0.3), real(1)),
                                             rng.Range(real(0.3), real(1)),
                                             rng.Range(real(0.3), real(1))));
        const Shape bb = Shape::MakeBox(Vec3(rng.Range(real(0.3), real(1)),
                                             rng.Range(real(0.3), real(1)),
                                             rng.Range(real(0.3), real(1))));
        const Transform ta = At(rng.InCube(real(1)), rng.Rotation());
        const Transform tb = At(rng.InCube(real(1)), rng.Rotation());

        Manifold analytic, convex;
        const bool h1 = CollideBoxBox(ba, ta, bb, tb, analytic);
        const bool h2 = CollideConvex(ba, ta, bb, tb, convex);

        INFO("iteration " << i);
        REQUIRE(h1 == h2);
        if (!h1) continue;

        const real depthAnalytic = analytic.MaxPenetration();
        const real depthConvex = convex.MaxPenetration();

        // 只在确实重叠时比较：两者都刚好落在推测性接触区间里时，深度接近 0，
        // 相对误差没有意义。
        if (depthAnalytic <= kSpeculativeMargin) continue;

        //----------------------------------------------------------------------
        // 两条路径求的是同一个最小平移量，但**解析解那边带了面/边偏置**
        // （见 CollideBoxBox：只有边轴明显更浅时才用它），所以它报出来的深度
        // 有可能比真正的最小值大一点点。EPA 没有偏置，给的是真正的最小值。
        //
        // 于是这里检验的是一个不等式而不是等式，而且上界就是那条偏置规则本身
        // 换算过来的：解析解选面轴的条件是
        //     edgeOverlap >= faceOverlap * 0.95 - 0.005
        // 所以
        //     depthAnalytic - depthConvex <= 0.05 * depthAnalytic + 0.005
        // 这样写既容纳了刻意的偏置，又不会放过真正的错误 ——
        // 如果 EPA 算大了（超过解析解），或者小得超出偏置能解释的范围，都会失败。
        //----------------------------------------------------------------------
        REQUIRE(depthConvex <= depthAnalytic + real(2e-3));
        REQUIRE(depthAnalytic - depthConvex <=
                real(0.05) * depthAnalytic + real(0.005) + real(2e-3));

        //----------------------------------------------------------------------
        // 这里**不比较法线方向**，而是检验两条路径各自的 (法线, 深度) 都真的能
        // 把两个盒子分开。
        //
        // 原因：15 根候选轴里经常有两根重叠量几乎相同（比如面轴 0.300 和
        // 边轴 0.299）。此时两条路径可能各选一根，法线相差几十度 —— 但两个答案
        // **都是对的**，都是合法的最小平移向量。拿方向做对比会把这种正常的平局
        // 报成失败，而"推开之后必须分离"检验的才是真正要保证的性质。
        //----------------------------------------------------------------------
        REQUIRE_FALSE(Overlap(
            ba, ta, bb,
            Transform(tb.position + convex.normal * (depthConvex + real(2e-3)),
                      tb.rotation)));
        REQUIRE_FALSE(Overlap(
            ba, ta, bb,
            Transform(tb.position + analytic.normal * (depthAnalytic + real(2e-3)),
                      tb.rotation)));

        ++deep;
    }

    // 确认样本里真的有大量重叠构型，不是"两边都返回 false"的假通过
    REQUIRE(deep > 300);
}

TEST_CASE("GJK/EPA 与解析解对拍：所有形状对", "[gjk][collision]") {
    // 球、胶囊参与的组合，两条路径在数学上是等价的，但代码路径完全不同：
    // 解析解走线段最近点，这边走支撑函数迭代。
    //
    // 注意胶囊 vs 盒在"轴线插进盒子里"时解析解是采样近似（见 NarrowPhase.cpp），
    // 而 GJK/EPA 是精确的 —— 那种情况下应该以 GJK/EPA 为准，所以跳过比较。
    Rng rng(778899u);
    int compared = 0;

    for (int i = 0; i < 5000; ++i) {
        const Shape sa = rng.AnyShape();
        const Shape sb = rng.AnyShape();
        const Transform ta = At(rng.InCube(real(1.2)), rng.Rotation());
        const Transform tb = At(rng.InCube(real(1.2)), rng.Rotation());

        Manifold analytic, convex;
        const bool h1 = Collide(sa, ta, sb, tb, analytic);
        const bool h2 = CollideConvex(sa, ta, sb, tb, convex);

        INFO("iteration " << i << " types " << static_cast<int>(sa.type) << "/"
                          << static_cast<int>(sb.type));
        REQUIRE(h1 == h2);
        if (!h1) continue;

        const real depthAnalytic = analytic.MaxPenetration();
        if (depthAnalytic <= kSpeculativeMargin) continue;

        // 跳过解析解用采样近似的那一档：胶囊轴线整个插进了盒子里
        const bool capsuleBoxDeep =
            ((sa.type == ShapeType::Capsule && sb.type == ShapeType::Box &&
              depthAnalytic >= sa.capsule.radius) ||
             (sb.type == ShapeType::Capsule && sa.type == ShapeType::Box &&
              depthAnalytic >= sb.capsule.radius));
        if (capsuleBoxDeep) continue;

        const real depthConvex = convex.MaxPenetration();

        if (sa.type == ShapeType::Box && sb.type == ShapeType::Box) {
            // 盒-盒带面/边偏置，只能比不等式（推导见上一个用例）
            REQUIRE(depthConvex <= depthAnalytic + real(2e-3));
            REQUIRE(depthAnalytic - depthConvex <=
                    real(0.05) * depthAnalytic + real(0.005) + real(2e-3));
            ++compared;
            continue;
        }

        // 其余形状对两边都是无偏置的精确解，必须严格吻合
        REQUIRE(depthConvex == Approx(depthAnalytic).margin(real(2e-3)));
        REQUIRE(Dot(analytic.normal, convex.normal) > real(0.999));

        //----------------------------------------------------------------------
        // 接触点位置只在**核分离**时比较。
        //
        // 核分离时接触点来自 GJK 的最近点对，那是精确解，两条路径必然一致。
        // 核相交时接触点来自 EPA：它把原点投影到"最近的那个三角面"上再反算
        // 重心坐标，而投影点可能落在该三角形之外（多面体的一个平面往往被切成
        // 好几个三角形），夹回三角形内会让接触点沿切向偏一点。
        // 深度和法线不受这个影响 —— 它们只取决于面所在的平面。
        //
        // 判据：核分离 <=> 穿透深度小于两个核半径之和（见 CollideConvex 的推导）。
        //----------------------------------------------------------------------
        const real radiusSum =
            ConvexProxy(sa, ta).CoreRadius() + ConvexProxy(sb, tb).CoreRadius();
        if (depthAnalytic < radiusSum - real(1e-3)) {
            REQUIRE(Eq(convex.points[0].position, analytic.points[0].position,
                       real(2e-2)));
        }
        ++compared;
    }

    REQUIRE(compared > 800);
}

//==============================================================================
// D. 通用路径自己的不变量
//==============================================================================

TEST_CASE("GJK/EPA 沿法线推开穿透深度之后必须分离", "[gjk][collision]") {
    // 和 M4 那条同样的不变量，但这次验的是 GJK/EPA 路径。
    // 它不依赖解析解，所以哪怕两条路径**一起**写错也能被抓住。
    Rng rng(13572468u);
    int checked = 0;

    for (int i = 0; i < 4000; ++i) {
        const Shape sa = rng.AnyShape();
        const Shape sb = rng.AnyShape();
        const Transform ta = At(rng.InCube(real(1.2)), rng.Rotation());
        const Transform tb = At(rng.InCube(real(1.2)), rng.Rotation());

        Manifold m;
        if (!CollideConvex(sa, ta, sb, tb, m)) continue;

        const real depth = m.MaxPenetration();
        if (depth <= real(0)) continue;

        const Transform pushed(tb.position + m.normal * (depth + real(2e-3)),
                               tb.rotation);

        INFO("iteration " << i << " depth " << depth);
        REQUIRE_FALSE(Overlap(sa, ta, sb, pushed));
        ++checked;
    }

    REQUIRE(checked > 800);
}

TEST_CASE("GJK/EPA 交换顺序的对称性", "[gjk][collision]") {
    Rng rng(864213579u);
    int checked = 0;

    for (int i = 0; i < 3000; ++i) {
        const Shape sa = rng.AnyShape();
        const Shape sb = rng.AnyShape();
        const Transform ta = At(rng.InCube(real(1.2)), rng.Rotation());
        const Transform tb = At(rng.InCube(real(1.2)), rng.Rotation());

        Manifold ab, ba;
        const bool h1 = CollideConvex(sa, ta, sb, tb, ab);
        const bool h2 = CollideConvex(sb, tb, sa, ta, ba);

        INFO("iteration " << i);
        REQUIRE(h1 == h2);
        if (!h1) continue;
        if (ab.MaxPenetration() <= kSpeculativeMargin) continue;

        // EPA 是迭代的，选中的最近面可能不同，所以容差比解析解那边松一档
        REQUIRE(Dot(ab.normal, -ba.normal) > real(0.99));
        REQUIRE(ab.MaxPenetration() ==
                Approx(ba.MaxPenetration()).margin(real(2e-3)));
        ++checked;
    }

    REQUIRE(checked > 500);
}

TEST_CASE("GJK 收敛速度在合理范围内", "[gjk][collision]") {
    // 核都是多面体，GJK 理论上有限步必然终止。这条用例是防回归的：
    // 如果哪天有人把终止判据改成绝对阈值、或者破坏了单纯形约简，
    // 迭代次数会立刻飙上去（而结果可能还是对的，别的测试抓不到）。
    Rng rng(2468013579u);
    int maxIterations = 0;
    long long total = 0;
    int count = 0;

    for (int i = 0; i < 3000; ++i) {
        const ConvexProxy pa(rng.AnyShape(), At(rng.InCube(real(2)), rng.Rotation()));
        const ConvexProxy pb(rng.AnyShape(), At(rng.InCube(real(2)), rng.Rotation()));

        const GjkResult r = Gjk(pa, pb);
        REQUIRE(r.status != GjkStatus::MaxIterations);
        maxIterations = maxIterations > r.iterations ? maxIterations : r.iterations;
        total += r.iterations;
        ++count;
    }

    INFO("最多 " << maxIterations << " 轮，平均 "
                << (static_cast<double>(total) / count) << " 轮");
    REQUIRE(maxIterations < 24);
}

//==============================================================================
// E. 退化输入
//==============================================================================

TEST_CASE("GJK/EPA 退化输入", "[gjk][collision][degenerate]") {
    Manifold m;

    SECTION("两个完全重合的盒子") {
        const Shape box = Shape::MakeCube(real(1));
        REQUIRE(CollideConvex(box, At(Vec3(0, 0, 0)), box, At(Vec3(0, 0, 0)), m));
        REQUIRE(NearlyEqual(m.normal.Length(), real(1), real(1e-3)));
        // 重合的立方体，最小平移量是穿过一个面：深度 2
        REQUIRE(m.MaxPenetration() == Approx(real(2)).margin(real(1e-2)));
    }

    SECTION("两个同心的球：核完全重合，EPA 撑不起四面体") {
        const Shape s1 = Shape::MakeSphere(real(1));
        const Shape s2 = Shape::MakeSphere(real(0.5));
        REQUIRE(CollideConvex(s1, At(Vec3(0, 0, 0)), s2, At(Vec3(0, 0, 0)), m));
        // 走的是兜底路径，但必须给出合法的法线和深度，不能是 NaN
        REQUIRE(NearlyEqual(m.normal.Length(), real(1), real(1e-3)));
        REQUIRE(m.MaxPenetration() == Approx(real(1.5)).margin(kTol));
    }

    SECTION("退化成薄片的盒子") {
        const Shape flat = Shape::MakeBox(Vec3(1, real(0), 1));
        const Shape box = Shape::MakeCube(real(1));
        REQUIRE(CollideConvex(flat, At(Vec3(0, 0, 0)), box, At(Vec3(0, real(0.5), 0)), m));
        REQUIRE(NearlyEqual(m.normal.Length(), real(1), real(1e-3)));
        REQUIRE(IsFinite(m.MaxPenetration()));
    }

    SECTION("零半径的球") {
        const Shape zero = Shape::MakeSphere(real(0));
        const Shape box = Shape::MakeCube(real(1));
        REQUIRE(CollideConvex(zero, At(Vec3(real(0.5), 0, 0)), box, At(Vec3(0, 0, 0)), m));
        REQUIRE(NearlyEqual(m.normal.Length(), real(1), real(1e-3)));
        // 点在盒内 x=0.5，最近的面是 x=1，深度 0.5
        REQUIRE(m.MaxPenetration() == Approx(real(0.5)).margin(real(1e-2)));
    }

    SECTION("两条首尾相接的胶囊") {
        const Shape cap = Shape::MakeCapsule(real(0.5), real(1));
        // A 的轴线覆盖 y ∈ [-1,1]，B 的覆盖 [1.8, 3.8]，端点间距 0.8
        REQUIRE(CollideConvex(cap, At(Vec3(0, 0, 0)), cap, At(Vec3(0, real(2.8), 0)), m));
        REQUIRE(NearlyEqual(m.normal.Length(), real(1), real(1e-3)));
        REQUIRE(m.MaxPenetration() == Approx(real(0.2)).margin(kTol));
        REQUIRE(Eq(m.normal, Vec3(0, 1, 0), real(1e-3)));
    }

    SECTION("两条共线且轴线重叠的胶囊：闵可夫斯基差退化成一条线段") {
        const Shape cap = Shape::MakeCapsule(real(0.5), real(1));
        // A 的轴线覆盖 y ∈ [-1,1]，B 的覆盖 [0.8, 2.8] —— 两条**轴线**本身重叠，
        // 于是核的距离是 0，穿透深度就是整个半径和。
        // 这时闵可夫斯基差是一条线段（没有体积），EPA 撑不起四面体，
        // 走的是"低维兜底"那条路：法线必须垂直于轴线（从侧面推开才最省力）。
        REQUIRE(CollideConvex(cap, At(Vec3(0, 0, 0)), cap, At(Vec3(0, real(1.8), 0)), m));
        REQUIRE(NearlyEqual(m.normal.Length(), real(1), real(1e-3)));
        REQUIRE(m.MaxPenetration() == Approx(real(1)).margin(kTol));
        REQUIRE(Abs(Dot(m.normal, Vec3(0, 1, 0))) < real(1e-3));

        // 解析解走的是完全不同的代码路径，结论必须一致
        Manifold analytic;
        REQUIRE(Collide(cap, At(Vec3(0, 0, 0)), cap, At(Vec3(0, real(1.8), 0)), analytic));
        REQUIRE(analytic.MaxPenetration() == Approx(m.MaxPenetration()).margin(kTol));
    }

    SECTION("极端尺寸差：小球贴在大盒子上") {
        const Shape tiny = Shape::MakeSphere(real(0.01));
        const Shape huge = Shape::MakeBox(Vec3(100, 100, 100));
        REQUIRE(CollideConvex(tiny, At(Vec3(0, real(100.005), 0)), huge,
                              At(Vec3(0, 0, 0)), m));
        REQUIRE(NearlyEqual(m.normal.Length(), real(1), real(1e-3)));
        REQUIRE(Eq(m.normal, Vec3(0, -1, 0), real(1e-2)));
    }
}
