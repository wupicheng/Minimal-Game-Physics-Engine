//==============================================================================
// tests/test_broadphase.cpp
//
// M3 宽相位（UniformGrid）的测试。
//
//------------------------------------------------------------------------------
// 怎么测一个"加速结构"
//------------------------------------------------------------------------------
// 加速结构本身不产生新的语义 —— 它的正确性定义就是"结果和暴力做法完全一样，
// 只是更快"。所以这个文件的核心是**对拍**：同一批数据、同一套过滤规则，
// 一边走网格，一边走 O(n^2) 暴力，两边的结果集必须一模一样。
//
// 对拍能同时抓住两类错误，而定点用例只能抓住第一类：
//   - 漏报（网格少给了一对）—— 空间划分算错了，格子区间没覆盖到
//   - 多报（网格多给了一对）—— 去重失效，或者过滤规则没用上
//
// 关键点是：暴力参考实现**必须复用同一个** BroadPhaseShouldCollide。
// 如果两边各写一份过滤规则，对拍检验的就变成了"两份规则抄得一不一样"，
// 而不是"空间划分对不对"。
//
// 除了对拍，还有三类：
//   A. 结构性质：id 复用、fat AABB 让 Update 变成空操作、超大体走暴力列表
//   B. DDA 的边角：负方向、轴对齐、起点在物体内、射程截断、空网格
//   C. 过滤语义：静态-静态不配对、双方休眠不配对、层掩码要双方都同意
//==============================================================================

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cstdint>
#include <vector>

#include "pe/collision/RayCast.h"
#include "pe/collision/UniformGrid.h"

using namespace pe;

namespace {

//------------------------------------------------------------------------------
// 确定性随机数（和 test_raycast.cpp 用的是同一套，理由见那边：
// 测试必须每次跑出完全一样的结果，失败时才能靠种子精确复现）
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

    /// [0, n) 的整数
    std::uint32_t Below(std::uint32_t n) { return NextU32() % n; }

    bool Chance(real p) { return Unit() < p; }

    Vec3 InCube(real half) {
        return Vec3(Range(-half, half), Range(-half, half), Range(-half, half));
    }

    /// 单位球面上的均匀方向
    Vec3 Direction() {
        for (;;) {
            const Vec3 v = InCube(real(1));
            const real len2 = v.LengthSq();
            if (len2 > real(1e-4) && len2 <= real(1)) return v * (real(1) / Sqrt(len2));
        }
    }
};

//------------------------------------------------------------------------------
// 暴力参考实现
//
// 只做"两两枚举"，过滤一律转交 BroadPhaseShouldCollide。
//------------------------------------------------------------------------------

std::vector<BroadPhasePair> BruteForcePairs(const IBroadPhase& bp,
                                            const std::vector<ProxyId>& ids) {
    std::vector<BroadPhasePair> out;
    for (std::size_t i = 0; i + 1 < ids.size(); ++i) {
        const BroadPhaseProxy* a = bp.GetProxy(ids[i]);
        if (a == nullptr) continue;
        for (std::size_t j = i + 1; j < ids.size(); ++j) {
            const BroadPhaseProxy* b = bp.GetProxy(ids[j]);
            if (b == nullptr) continue;
            if (BroadPhaseShouldCollide(*a, *b)) out.emplace_back(ids[i], ids[j]);
        }
    }
    std::sort(out.begin(), out.end());
    return out;
}

std::vector<ProxyId> BruteForceAABB(const IBroadPhase& bp,
                                    const std::vector<ProxyId>& ids, const AABB& box) {
    std::vector<ProxyId> out;
    for (const ProxyId id : ids) {
        const BroadPhaseProxy* p = bp.GetProxy(id);
        if (p != nullptr && box.Overlaps(p->fatAabb)) out.push_back(id);
    }
    std::sort(out.begin(), out.end());
    return out;
}

std::vector<ProxyId> BruteForceRay(const IBroadPhase& bp,
                                   const std::vector<ProxyId>& ids, const Ray& ray) {
    std::vector<ProxyId> out;
    RaycastHit hit;
    for (const ProxyId id : ids) {
        const BroadPhaseProxy* p = bp.GetProxy(id);
        if (p != nullptr && RaycastAABB(p->fatAabb, ray, hit)) out.push_back(id);
    }
    std::sort(out.begin(), out.end());
    return out;
}

//------------------------------------------------------------------------------
// 取出网格的结果并排序，方便和暴力结果直接比
//------------------------------------------------------------------------------

std::vector<BroadPhasePair> SortedPairs(const IBroadPhase& bp) {
    std::vector<BroadPhasePair> out;
    bp.QueryPairs(out);
    std::sort(out.begin(), out.end());
    return out;
}

std::vector<ProxyId> SortedAABB(const IBroadPhase& bp, const AABB& box) {
    std::vector<ProxyId> out;
    bp.QueryAABB(box, out);
    std::sort(out.begin(), out.end());
    return out;
}

std::vector<ProxyId> SortedRay(const IBroadPhase& bp, const Ray& ray) {
    std::vector<ProxyId> out;
    bp.QueryRay(ray, out);
    std::sort(out.begin(), out.end());
    return out;
}

bool HasDuplicates(const std::vector<BroadPhasePair>& sorted) {
    return std::adjacent_find(sorted.begin(), sorted.end()) != sorted.end();
}

bool HasDuplicates(const std::vector<ProxyId>& sorted) {
    return std::adjacent_find(sorted.begin(), sorted.end()) != sorted.end();
}

/// 造一个中心在 c、半尺寸为 h 的动态 proxy 描述
BroadPhaseProxyDesc MakeDesc(const Vec3& c, const Vec3& h, bool isStatic = false) {
    BroadPhaseProxyDesc d;
    d.aabb = AABB::FromCenterHalfExtents(c, h);
    d.isStatic = isStatic;
    return d;
}

BroadPhaseProxyDesc MakeDesc(const Vec3& c, real h, bool isStatic = false) {
    return MakeDesc(c, Vec3(h, h, h), isStatic);
}

}  // namespace

//==============================================================================
// A. 结构性质
//==============================================================================

TEST_CASE("宽相位 proxy 的注册与注销", "[broadphase][collision]") {
    UniformGrid grid(real(2));

    REQUIRE(grid.ProxyCount() == 0);

    const ProxyId a = grid.Insert(MakeDesc(Vec3(0, 0, 0), real(0.5)));
    const ProxyId b = grid.Insert(MakeDesc(Vec3(10, 0, 0), real(0.5)));

    REQUIRE(grid.ProxyCount() == 2);
    REQUIRE(grid.GetProxy(a) != nullptr);
    REQUIRE(grid.GetProxy(b) != nullptr);

    SECTION("注销之后取不到，数量减一") {
        grid.Remove(a);
        REQUIRE(grid.ProxyCount() == 1);
        REQUIRE(grid.GetProxy(a) == nullptr);
        REQUIRE(grid.GetProxy(b) != nullptr);
    }

    SECTION("重复注销是安全的空操作") {
        grid.Remove(a);
        grid.Remove(a);
        grid.Remove(kInvalidProxyId);
        REQUIRE(grid.ProxyCount() == 1);
    }

    SECTION("id 会被复用") {
        grid.Remove(a);
        const ProxyId c = grid.Insert(MakeDesc(Vec3(-5, 0, 0), real(0.5)));
        REQUIRE(c == a);
        REQUIRE(grid.ProxyCount() == 2);
        // 复用之后拿到的必须是新数据，不能是旧 proxy 的残留
        REQUIRE(grid.GetProxy(c)->fatAabb.Contains(Vec3(-5, 0, 0)));
    }

    SECTION("Clear 之后网格是空的") {
        grid.Clear();
        REQUIRE(grid.ProxyCount() == 0);
        REQUIRE(grid.GetProxy(a) == nullptr);

        std::vector<BroadPhasePair> pairs;
        grid.QueryPairs(pairs);
        REQUIRE(pairs.empty());
    }
}

TEST_CASE("宽相位 fat AABB 让小幅移动不动哈希表", "[broadphase][collision]") {
    // margin 给大一点，好把"膨胀壳"的效果和格子边界区分开
    UniformGrid grid(real(2), real(0.5));
    const ProxyId id = grid.Insert(MakeDesc(Vec3(0, 0, 0), real(0.25)));

    const AABB fat0 = grid.GetProxy(id)->fatAabb;

    SECTION("新 AABB 还在膨胀壳里 -> 返回 false 且什么都不改") {
        const AABB moved = AABB::FromCenterHalfExtents(Vec3(real(0.1), 0, 0), Vec3(real(0.25), real(0.25), real(0.25)));
        REQUIRE(fat0.Contains(moved));
        REQUIRE(grid.Update(id, moved) == false);
        REQUIRE(NearlyEqual(grid.GetProxy(id)->fatAabb, fat0));
    }

    SECTION("移出膨胀壳 -> 返回 true 并重建 fat AABB") {
        const AABB moved = AABB::FromCenterHalfExtents(Vec3(5, 0, 0), Vec3(real(0.25), real(0.25), real(0.25)));
        REQUIRE(grid.Update(id, moved) == true);

        const AABB fat1 = grid.GetProxy(id)->fatAabb;
        REQUIRE(fat1.Contains(moved));
        REQUIRE_FALSE(NearlyEqual(fat1, fat0));

        // 移动之后必须能在新位置被查到、在旧位置查不到
        REQUIRE(SortedAABB(grid, AABB::FromCenterHalfExtents(Vec3(5, 0, 0), Vec3(1, 1, 1))) ==
                std::vector<ProxyId>{id});
        REQUIRE(SortedAABB(grid, AABB::FromCenterHalfExtents(Vec3(0, 0, 0), Vec3(real(0.1), real(0.1), real(0.1)))).empty());
    }

    SECTION("对无效 id 的 Update / SetSleeping 是安全的空操作") {
        REQUIRE(grid.Update(kInvalidProxyId, AABB::FromCenterHalfExtents(Vec3(0, 0, 0), Vec3(1, 1, 1))) == false);
        grid.SetSleeping(kInvalidProxyId, true);
        REQUIRE(grid.ProxyCount() == 1);
    }
}

TEST_CASE("宽相位 跨多个格子的物体只被吐出一次", "[broadphase][collision]") {
    // cellSize 1 米，两个 6 米见方的盒子 —— 它们会同时出现在几十个格子里。
    // 没有去重的话这一对会被重复吐出上百次。
    UniformGrid grid(real(1));

    const ProxyId a = grid.Insert(MakeDesc(Vec3(0, 0, 0), real(3)));
    const ProxyId b = grid.Insert(MakeDesc(Vec3(1, 1, 1), real(3)));

    const std::vector<BroadPhasePair> pairs = SortedPairs(grid);
    REQUIRE(pairs.size() == 1);
    REQUIRE(pairs[0] == BroadPhasePair(a, b));
}

TEST_CASE("宽相位 超大物体走暴力列表但结果不变", "[broadphase][collision]") {
    // cellSize 0.5 米 + 一块 200x200 米的地面 = 16 万个格子，远超阈值，
    // 这个 proxy 必须被判定为超大体。它的可查询性不能因此打折。
    UniformGrid grid(real(0.5));

    BroadPhaseProxyDesc groundDesc;
    groundDesc.aabb = AABB(Vec3(-100, real(-1), -100), Vec3(100, real(0), 100));
    groundDesc.isStatic = true;
    const ProxyId ground = grid.Insert(groundDesc);

    const ProxyId box = grid.Insert(MakeDesc(Vec3(3, real(0.4), -7), real(0.5)));

    const UniformGridStats stats = grid.ComputeStats();
    REQUIRE(stats.oversizedCount == 1);
    REQUIRE(stats.proxyCount == 2);

    SECTION("超大体照样参与配对") {
        const std::vector<BroadPhasePair> pairs = SortedPairs(grid);
        REQUIRE(pairs == std::vector<BroadPhasePair>{BroadPhasePair(ground, box)});
    }

    SECTION("超大体照样能被 AABB 查询命中") {
        // 查询盒要跨过地面（y=0），否则只会碰到箱子 —— 地面只有 y ∈ [-1, 0] 这一层
        const AABB q = AABB::FromCenterHalfExtents(Vec3(3, real(0.4), -7),
                                                   Vec3(real(0.6), real(0.6), real(0.6)));
        const std::vector<ProxyId> hits = SortedAABB(grid, q);
        std::vector<ProxyId> expect{ground, box};
        std::sort(expect.begin(), expect.end());
        REQUIRE(hits == expect);

        // 换一个完全在地面上方的查询盒，就只该碰到箱子
        REQUIRE(SortedAABB(grid, AABB::FromCenterHalfExtents(
                                     Vec3(3, real(0.4), -7),
                                     Vec3(real(0.2), real(0.2), real(0.2)))) ==
                std::vector<ProxyId>{box});
    }

    SECTION("超大体照样能被射线命中") {
        // 从高处朝下打，先穿过箱子再打到地面
        const Ray ray(Vec3(3, 20, -7), Vec3(0, -1, 0), real(100));
        const std::vector<ProxyId> hits = SortedRay(grid, ray);
        std::vector<ProxyId> expect{ground, box};
        std::sort(expect.begin(), expect.end());
        REQUIRE(hits == expect);
    }

    SECTION("超大体注销之后不再出现") {
        grid.Remove(ground);
        REQUIRE(grid.ComputeStats().oversizedCount == 0);
        REQUIRE(SortedPairs(grid).empty());
    }
}

//==============================================================================
// B. 过滤语义
//==============================================================================

TEST_CASE("宽相位 过滤规则", "[broadphase][collision]") {
    UniformGrid grid(real(2));

    SECTION("静态-静态永远不配对") {
        grid.Insert(MakeDesc(Vec3(0, 0, 0), real(1), /*isStatic*/ true));
        grid.Insert(MakeDesc(Vec3(real(0.5), 0, 0), real(1), /*isStatic*/ true));
        REQUIRE(SortedPairs(grid).empty());
    }

    SECTION("双方都休眠不配对，任一方醒着就配对") {
        BroadPhaseProxyDesc d0 = MakeDesc(Vec3(0, 0, 0), real(1));
        BroadPhaseProxyDesc d1 = MakeDesc(Vec3(real(0.5), 0, 0), real(1));
        d0.isSleeping = true;
        d1.isSleeping = true;
        const ProxyId a = grid.Insert(d0);
        const ProxyId b = grid.Insert(d1);

        REQUIRE(SortedPairs(grid).empty());

        grid.SetSleeping(a, false);
        REQUIRE(SortedPairs(grid) == std::vector<BroadPhasePair>{BroadPhasePair(a, b)});
    }

    SECTION("睡着的动态体压在静态地面上也不配对") {
        // 这是休眠最主要的收益场景：一堆睡着的箱子躺在地上，
        // 整个场景的候选对数应该是 0。
        BroadPhaseProxyDesc sleeper = MakeDesc(Vec3(0, 1, 0), real(1));
        sleeper.isSleeping = true;
        grid.Insert(sleeper);
        grid.Insert(MakeDesc(Vec3(0, real(-0.5), 0), real(1), /*isStatic*/ true));
        REQUIRE(SortedPairs(grid).empty());
    }

    SECTION("层掩码必须双方都同意") {
        BroadPhaseProxyDesc d0 = MakeDesc(Vec3(0, 0, 0), real(1));
        BroadPhaseProxyDesc d1 = MakeDesc(Vec3(real(0.5), 0, 0), real(1));
        d0.layer = layers::kPlayer;
        d1.layer = layers::kProjectile;

        // 单向同意：抛射物愿意撞玩家，玩家却把抛射物层屏蔽了 -> 不配对
        d0.layerMask = ~layers::kProjectile;
        d1.layerMask = kLayerAll;
        const ProxyId a = grid.Insert(d0);
        const ProxyId b = grid.Insert(d1);
        REQUIRE(SortedPairs(grid).empty());

        // 双方都同意 -> 配对
        grid.Remove(a);
        d0.layerMask = kLayerAll;
        const ProxyId c = grid.Insert(d0);
        REQUIRE(SortedPairs(grid) == std::vector<BroadPhasePair>{BroadPhasePair(c, b)});
    }

    SECTION("AABB 查询不做层过滤") {
        // 查询发起方是一个凭空来的盒子，它没有自己的层，所以不该被过滤掉任何东西
        BroadPhaseProxyDesc d = MakeDesc(Vec3(0, 0, 0), real(1));
        d.layer = layers::kPlayer;
        d.layerMask = kLayerNone;  // 谁都不想撞
        const ProxyId id = grid.Insert(d);

        REQUIRE(SortedAABB(grid, AABB::FromCenterHalfExtents(Vec3(0, 0, 0), Vec3(1, 1, 1))) ==
                std::vector<ProxyId>{id});
    }
}

//==============================================================================
// C. 格子边界与负坐标
//==============================================================================

TEST_CASE("宽相位 负坐标不会塌进同一个格子", "[broadphase][collision]") {
    // 用截断取整代替 floor 的经典 bug：-0.5 和 +0.5 会被算成同一个格子，
    // 于是原点附近变成一个双倍大的格子。表现是"原点附近的物体互相配对不上"
    // 或者"远处的物体莫名其妙被配对"。
    UniformGrid grid(real(1), /*margin*/ real(0));

    // 两个半径 0.2 的小球，分别在 -0.5 和 +0.5，中间隔着 0.6 米，不该配对
    const ProxyId a = grid.Insert(MakeDesc(Vec3(real(-0.5), 0, 0), real(0.2)));
    const ProxyId b = grid.Insert(MakeDesc(Vec3(real(0.5), 0, 0), real(0.2)));
    REQUIRE(SortedPairs(grid).empty());

    // 各自的位置必须能被精确查到
    REQUIRE(SortedAABB(grid, AABB(Vec3(real(-0.6), real(-0.1), real(-0.1)), Vec3(real(-0.4), real(0.1), real(0.1)))) ==
            std::vector<ProxyId>{a});
    REQUIRE(SortedAABB(grid, AABB(Vec3(real(0.4), real(-0.1), real(-0.1)), Vec3(real(0.6), real(0.1), real(0.1)))) ==
            std::vector<ProxyId>{b});

    // 负半轴上的一对贴在一起的物体必须能配对
    const ProxyId c = grid.Insert(MakeDesc(Vec3(real(-10.4), real(-10.4), real(-10.4)), real(0.3)));
    const ProxyId d = grid.Insert(MakeDesc(Vec3(real(-10.1), real(-10.1), real(-10.1)), real(0.3)));
    const std::vector<BroadPhasePair> pairs = SortedPairs(grid);
    REQUIRE(pairs == std::vector<BroadPhasePair>{BroadPhasePair(c, d)});
}

TEST_CASE("宽相位 贴着格子边界的物体不会漏配", "[broadphase][collision]") {
    // 两个物体恰好各在格子边界的两侧、面贴面。宽相位用闭区间，必须报出来。
    UniformGrid grid(real(1), /*margin*/ real(0));

    const ProxyId a = grid.Insert(MakeDesc(Vec3(real(-0.5), 0, 0), Vec3(real(0.5), real(0.5), real(0.5))));
    const ProxyId b = grid.Insert(MakeDesc(Vec3(real(0.5), 0, 0), Vec3(real(0.5), real(0.5), real(0.5))));

    // a 的 max.x == b 的 min.x == 0，正好贴面
    REQUIRE(SortedPairs(grid) == std::vector<BroadPhasePair>{BroadPhasePair(a, b)});
}

//==============================================================================
// D. DDA 射线遍历
//==============================================================================

TEST_CASE("宽相位 射线遍历的基本方向", "[broadphase][collision][raycast]") {
    UniformGrid grid(real(1), /*margin*/ real(0));

    // 沿 +X 排成一串的小盒子，间隔 2 米
    std::vector<ProxyId> line;
    for (int i = -5; i <= 5; ++i) {
        line.push_back(grid.Insert(
            MakeDesc(Vec3(real(i) * real(2), 0, 0), real(0.4))));
    }

    SECTION("正方向轴对齐射线打中整条线") {
        const Ray ray(Vec3(real(-20), 0, 0), Vec3(1, 0, 0), real(100));
        std::vector<ProxyId> expect = line;
        std::sort(expect.begin(), expect.end());
        REQUIRE(SortedRay(grid, ray) == expect);
    }

    SECTION("负方向轴对齐射线结果相同") {
        const Ray ray(Vec3(real(20), 0, 0), Vec3(-1, 0, 0), real(100));
        std::vector<ProxyId> expect = line;
        std::sort(expect.begin(), expect.end());
        REQUIRE(SortedRay(grid, ray) == expect);
    }

    SECTION("射程截断：只打到前几个") {
        // 从 x = -20 出发，射程 15 -> 只能够到 x = -5 之前的盒子，
        // 即 x = -10、-8、-6 三个（-4 在 x=-4，超出 -20+15=-5 之外）
        const Ray ray(Vec3(real(-20), 0, 0), Vec3(1, 0, 0), real(15));
        const std::vector<ProxyId> hits = SortedRay(grid, ray);
        REQUIRE(hits == BruteForceRay(grid, line, ray));
        REQUIRE(hits.size() == 3);
    }

    SECTION("完全错过：平行但偏移") {
        const Ray ray(Vec3(real(-20), 10, 0), Vec3(1, 0, 0), real(100));
        REQUIRE(SortedRay(grid, ray).empty());
    }

    SECTION("起点在某个盒子内部") {
        const Ray ray(Vec3(0, 0, 0), Vec3(0, 1, 0), real(100));
        // 起点在 index 5（x=0）的盒子里；RaycastAABB 对内部起点返回命中
        REQUIRE(SortedRay(grid, ray) == std::vector<ProxyId>{line[5]});
    }
}

TEST_CASE("宽相位 空网格与退化射线", "[broadphase][collision][raycast]") {
    UniformGrid grid(real(2));

    SECTION("空网格上的查询不崩且返回空") {
        std::vector<ProxyId> out;
        grid.QueryRay(Ray(Vec3(0, 0, 0), Vec3(1, 0, 0), real(10)), out);
        REQUIRE(out.empty());

        grid.QueryAABB(AABB::FromCenterHalfExtents(Vec3(0, 0, 0), Vec3(5, 5, 5)), out);
        REQUIRE(out.empty());
    }

    SECTION("零方向射线（射程被构造函数置零）只命中起点所在的物体") {
        const ProxyId id = grid.Insert(MakeDesc(Vec3(0, 0, 0), real(1)));
        grid.Insert(MakeDesc(Vec3(10, 0, 0), real(1)));

        // Ray 的构造函数会把零方向退化成 +X 且 maxDistance = 0
        const Ray degenerate(Vec3(0, 0, 0), Vec3(0, 0, 0));
        REQUIRE(degenerate.maxDistance == real(0));
        REQUIRE(SortedRay(grid, degenerate) == std::vector<ProxyId>{id});
    }

    SECTION("射线完全在网格包围盒之外") {
        grid.Insert(MakeDesc(Vec3(0, 0, 0), real(1)));
        const Ray ray(Vec3(real(1000), real(1000), real(1000)), Vec3(1, 0, 0), real(10));
        REQUIRE(SortedRay(grid, ray).empty());
    }
}

//==============================================================================
// E. 对拍 —— M3 的验收测试
//==============================================================================

TEST_CASE("宽相位 与暴力 O(n^2) 对拍：1000 个随机物体", "[broadphase][collision]") {
    // ARCHITECTURE.md 第 5 节 M3 的验收条件就是这一条。
    //
    // 场景刻意做得"难看"：尺寸跨一个数量级（0.1 ~ 3 米）、静态动态混杂、
    // 一部分休眠、层掩码随机。物体尺寸的分布越不均匀，均匀网格越容易露馅。
    Rng rng(20260901u);
    UniformGrid grid(real(2));

    constexpr int kCount = 1000;
    std::vector<ProxyId> ids;
    ids.reserve(kCount);

    for (int i = 0; i < kCount; ++i) {
        BroadPhaseProxyDesc d;
        const Vec3 center = rng.InCube(real(30));
        const real h = rng.Range(real(0.1), real(3));
        d.aabb = AABB::FromCenterHalfExtents(center, Vec3(h, h, h));
        d.isStatic = rng.Chance(real(0.6));
        d.isSleeping = !d.isStatic && rng.Chance(real(0.3));
        d.layer = rng.Chance(real(0.25)) ? layers::kPlayer : layers::kDefault;
        d.layerMask = rng.Chance(real(0.15)) ? ~layers::kPlayer : kLayerAll;
        ids.push_back(grid.Insert(d));
    }

    const std::vector<BroadPhasePair> expected = BruteForcePairs(grid, ids);
    const std::vector<BroadPhasePair> actual = SortedPairs(grid);

    INFO("expected " << expected.size() << " pairs, got " << actual.size());
    REQUIRE_FALSE(HasDuplicates(actual));
    REQUIRE(actual == expected);

    // 这个场景应该确实产生了不少候选对，否则"对拍通过"只说明两边都是空的
    REQUIRE(expected.size() > 100);
}

TEST_CASE("宽相位 增删改之后仍与暴力一致", "[broadphase][collision]") {
    // 上一个用例只测了"一次性构建"。真实场景里物体会不停地移动、生灭，
    // 而 UnlinkCells / LinkCells 的配对错误只有在这种搅动下才会暴露：
    // 比如注销时按**新**的格子区间去反注册（而不是登记时用的那个），
    // 会在哈希表里留下悬空的 id —— 一次性构建的用例完全看不出来。
    Rng rng(0xC0FFEEu);
    UniformGrid grid(real(2), real(0.1));

    std::vector<ProxyId> ids;
    for (int i = 0; i < 300; ++i) {
        BroadPhaseProxyDesc d;
        d.aabb = AABB::FromCenterHalfExtents(rng.InCube(real(15)),
                                             Vec3(real(0.5), real(0.5), real(0.5)));
        d.isStatic = rng.Chance(real(0.3));
        ids.push_back(grid.Insert(d));
    }

    for (int frame = 0; frame < 30; ++frame) {
        // 移动一批
        for (int k = 0; k < 60; ++k) {
            const ProxyId id = ids[rng.Below(static_cast<std::uint32_t>(ids.size()))];
            const BroadPhaseProxy* p = grid.GetProxy(id);
            if (p == nullptr) continue;
            const Vec3 c = p->fatAabb.Center() + rng.InCube(real(3));
            grid.Update(id, AABB::FromCenterHalfExtents(
                                c, Vec3(real(0.5), real(0.5), real(0.5))));
        }

        // 删一批
        for (int k = 0; k < 10; ++k) {
            const std::uint32_t slot = rng.Below(static_cast<std::uint32_t>(ids.size()));
            grid.Remove(ids[slot]);
            ids[slot] = ids.back();
            ids.pop_back();
            if (ids.empty()) break;
        }

        // 补一批（会复用刚释放的 id）
        for (int k = 0; k < 10; ++k) {
            BroadPhaseProxyDesc d;
            d.aabb = AABB::FromCenterHalfExtents(rng.InCube(real(15)),
                                                 Vec3(real(0.5), real(0.5), real(0.5)));
            d.isStatic = rng.Chance(real(0.3));
            ids.push_back(grid.Insert(d));
        }

        // 改休眠标志
        for (int k = 0; k < 20; ++k) {
            grid.SetSleeping(ids[rng.Below(static_cast<std::uint32_t>(ids.size()))],
                             rng.Chance(real(0.5)));
        }

        const std::vector<BroadPhasePair> actual = SortedPairs(grid);
        INFO("frame " << frame);
        REQUIRE_FALSE(HasDuplicates(actual));
        REQUIRE(actual == BruteForcePairs(grid, ids));
    }

    // 全部删光之后，网格里不该剩下任何东西
    for (const ProxyId id : ids) grid.Remove(id);
    REQUIRE(grid.ProxyCount() == 0);
    REQUIRE(SortedPairs(grid).empty());

    const UniformGridStats stats = grid.ComputeStats();
    INFO("残留静态格子 " << stats.staticCellCount << "，动态格子 "
                        << stats.dynamicCellCount);
    // 空格子必须被从哈希表里删掉，否则长时间运行后遍历会在空格子上空转
    REQUIRE(stats.staticCellCount == 0);
    REQUIRE(stats.dynamicCellCount == 0);
}

TEST_CASE("宽相位 AABB 查询与暴力对拍", "[broadphase][collision]") {
    Rng rng(4242u);
    UniformGrid grid(real(2));

    std::vector<ProxyId> ids;
    for (int i = 0; i < 500; ++i) {
        const real h = rng.Range(real(0.2), real(2));
        BroadPhaseProxyDesc d;
        d.aabb = AABB::FromCenterHalfExtents(rng.InCube(real(25)), Vec3(h, h, h));
        d.isStatic = rng.Chance(real(0.5));
        ids.push_back(grid.Insert(d));
    }

    for (int i = 0; i < 200; ++i) {
        // 查询盒的尺寸也跨数量级：从比格子小得多，到比整个场景还大
        const real h = rng.Range(real(0.05), real(40));
        const AABB box =
            AABB::FromCenterHalfExtents(rng.InCube(real(30)), Vec3(h, h, h));

        const std::vector<ProxyId> actual = SortedAABB(grid, box);
        INFO("query " << i << " halfExtent " << h);
        REQUIRE_FALSE(HasDuplicates(actual));
        REQUIRE(actual == BruteForceAABB(grid, ids, box));
    }
}

TEST_CASE("宽相位 射线查询与暴力对拍", "[broadphase][collision][raycast]") {
    // DDA 的验收测试。随机射线里包含了轴对齐、掠射、起点在物体内部、
    // 起点在场景之外等各种情况 —— 这些恰好是 DDA 最容易写错的地方。
    Rng rng(987654321u);
    UniformGrid grid(real(2));

    std::vector<ProxyId> ids;
    for (int i = 0; i < 400; ++i) {
        const real h = rng.Range(real(0.2), real(2));
        BroadPhaseProxyDesc d;
        d.aabb = AABB::FromCenterHalfExtents(rng.InCube(real(20)), Vec3(h, h, h));
        d.isStatic = rng.Chance(real(0.5));
        ids.push_back(grid.Insert(d));
    }

    int totalHits = 0;

    for (int i = 0; i < 500; ++i) {
        Vec3 dir = rng.Direction();
        // 每五条里有一条强制轴对齐：方向分量为 0 是 DDA 里要单独处理的分支，
        // 纯随机方向几乎永远碰不到它。
        if (i % 5 == 0) {
            dir = Vec3::Zero();
            dir[static_cast<int>(rng.Below(3))] = rng.Chance(real(0.5)) ? real(1) : real(-1);
        }

        // 起点一半在场景内（会命中"起点在内部"的分支），一半在场景外
        const Vec3 origin = rng.Chance(real(0.5)) ? rng.InCube(real(20))
                                                  : rng.InCube(real(60));
        const Ray ray(origin, dir, rng.Range(real(1), real(120)));

        const std::vector<ProxyId> actual = SortedRay(grid, ray);
        const std::vector<ProxyId> expected = BruteForceRay(grid, ids, ray);

        INFO("ray " << i << " origin (" << origin.x << ", " << origin.y << ", "
                    << origin.z << ") dir (" << ray.direction.x << ", "
                    << ray.direction.y << ", " << ray.direction.z << ") maxDist "
                    << ray.maxDistance);
        REQUIRE_FALSE(HasDuplicates(actual));
        REQUIRE(actual == expected);

        totalHits += static_cast<int>(expected.size());
    }

    // 确认这批射线真的打中了东西，不是"两边都是空集"的假通过。
    // 种子固定，这个数是确定的（当前实现下正好 241），阈值只是防止哪天场景参数
    // 被改得太稀疏、对拍退化成"两边都返回空"还一直显示通过。
    INFO("total hits = " << totalHits);
    REQUIRE(totalHits > 200);
}

TEST_CASE("宽相位 极端 cellSize 下结果不变", "[broadphase][collision]") {
    // 均匀网格唯一的调参旋钮是 cellSize。它只该影响速度，绝不该影响结果。
    // 这里用同一批数据在四种量级的 cellSize 下跑，要求四份结果完全相同。
    Rng rng(13579u);

    struct Item {
        AABB box;
        bool isStatic;
    };
    std::vector<Item> items;
    for (int i = 0; i < 200; ++i) {
        const real h = rng.Range(real(0.2), real(2));
        items.push_back(
            Item{AABB::FromCenterHalfExtents(rng.InCube(real(12)), Vec3(h, h, h)),
                 rng.Chance(real(0.4))});
    }

    const Ray probe(Vec3(-30, real(0.3), real(-7.5)), Vec3(1, real(0.1), real(0.2)),
                    real(80));
    const AABB probeBox =
        AABB::FromCenterHalfExtents(Vec3(2, 1, real(-3)), Vec3(4, 4, 4));

    std::vector<BroadPhasePair> refPairs;
    std::vector<ProxyId> refRay;
    std::vector<ProxyId> refBox;

    // 0.25 米（比大多数物体还小，物体跨很多格子）到 50 米（整个场景挤在一格里）
    const real sizes[] = {real(0.25), real(1), real(5), real(50)};
    for (std::size_t s = 0; s < 4; ++s) {
        UniformGrid grid(sizes[s]);
        std::vector<ProxyId> ids;
        for (const Item& it : items) {
            BroadPhaseProxyDesc d;
            d.aabb = it.box;
            d.isStatic = it.isStatic;
            ids.push_back(grid.Insert(d));
        }

        const std::vector<BroadPhasePair> pairs = SortedPairs(grid);
        const std::vector<ProxyId> rayHits = SortedRay(grid, probe);
        const std::vector<ProxyId> boxHits = SortedAABB(grid, probeBox);

        INFO("cellSize " << sizes[s]);
        REQUIRE(pairs == BruteForcePairs(grid, ids));
        REQUIRE(rayHits == BruteForceRay(grid, ids, probe));
        REQUIRE(boxHits == BruteForceAABB(grid, ids, probeBox));

        if (s == 0) {
            refPairs = pairs;
            refRay = rayHits;
            refBox = boxHits;
        } else {
            // id 的分配顺序在四次里完全一致，所以结果可以逐个对比
            REQUIRE(pairs == refPairs);
            REQUIRE(rayHits == refRay);
            REQUIRE(boxHits == refBox);
        }
    }
}

TEST_CASE("宽相位 坐标爆炸时不会把网格撑爆", "[broadphase][collision][degenerate]") {
    // 求解器发散时物体坐标会变成 1e30 甚至 inf。宽相位不该跟着崩 ——
    // 它是引擎里唯一会拿浮点坐标去开哈希表的地方，一旦把 inf 转成 int32
    // 得到随机值，就会往表里塞天文数字个格子。
    UniformGrid grid(real(2));

    const ProxyId sane = grid.Insert(MakeDesc(Vec3(0, 0, 0), real(1)));

    const real huge = real(1e30);
    const ProxyId blown =
        grid.Insert(MakeDesc(Vec3(huge, huge, huge), Vec3(huge, huge, huge)));

    // 撑爆坐标的那个必然被判成超大体，走暴力列表，不会往哈希表里塞东西
    const UniformGridStats stats = grid.ComputeStats();
    REQUIRE(stats.oversizedCount == 1);
    REQUIRE(stats.staticCellCount + stats.dynamicCellCount < 100);

    // 正常的那个照样查得到
    REQUIRE(grid.GetProxy(sane) != nullptr);
    std::vector<ProxyId> out;
    grid.QueryAABB(AABB::FromCenterHalfExtents(Vec3(0, 0, 0), Vec3(real(0.1), real(0.1), real(0.1))), out);
    REQUIRE(std::find(out.begin(), out.end(), sane) != out.end());

    grid.Remove(blown);
    REQUIRE(grid.ComputeStats().oversizedCount == 0);
}
