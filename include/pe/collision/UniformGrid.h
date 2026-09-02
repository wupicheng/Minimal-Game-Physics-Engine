#pragma once
//==============================================================================
// pe/collision/UniformGrid.h
//
// IBroadPhase 的第一版实现：无限稀疏均匀网格（空间哈希）。
//
//------------------------------------------------------------------------------
// 结构
//------------------------------------------------------------------------------
// 世界被切成边长 cellSize 的立方体格子。格子不预先分配 —— 用
// unordered_map<CellKey, vector<ProxyId>> 存"非空的格子"，所以网格在三个方向上
// 都是无限的，坐标为负也没问题，内存只和实际占用的格子数成正比。
//
// 一个 proxy 的 fat AABB 会覆盖一段连续的格子区间 [minCell, maxCell]，
// 它被登记进这个区间里的**每一个**格子。所以大物体会在多个格子里各留一份 id。
//
//------------------------------------------------------------------------------
// 三张表，不是一张
//------------------------------------------------------------------------------
//   m_staticCells    静态体。构建一次，之后永远不动。
//   m_dynamicCells   会动的体。每帧可能重新登记。
//   m_oversized      "超大"的体，不进任何格子，见下。
//
// 拆成静态/动态两张表的收益：配对时只需要遍历**动态**表的格子，对每个格子再去
// 静态表里查同一个 key。于是"静态-静态"这类对连产生的机会都没有，也不需要事后
// 过滤。FPS 地图里静态体比动态体多一个数量级，这一步基本上把配对开销从
// O(静态数) 降到了 O(动态数)。
//
// m_oversized 是给"跨越太多格子"的物体准备的逃生舱。典型例子是 200x200 米的
// 地面：cellSize=2 米时它要占 100x100 = 10000 个格子，登记一次就要往哈希表里塞
// 一万个条目，而且此后每次射线查询扫到的每个格子都会命中它。把这类物体拿出来
// 单独存成一个线性列表，配对与查询时对它们做暴力 AABB 测试 —— 只要这种物体的
// 数量是个位数，暴力就是最快的做法。
// 判定阈值是 kMaxCellsPerProxy。
//
//------------------------------------------------------------------------------
// 去重：不用哈希集合
//------------------------------------------------------------------------------
// 两个大物体可能同时出现在好几个格子里，逐格配对会重复吐出同一对。常见做法是
// 把 (a,b) 塞进 unordered_set 去重，但那样每对都要付一次哈希 + 可能的堆分配。
//
// 这里用的是"归属格"技巧：对于同时占据格子 K 的两个 proxy，它们的格子区间的
// 交集是 [max(minA,minB), min(maxA,maxB)]，这个交集非空（K 就在里面），而且
// **只依赖这两个 proxy，与当前遍历到哪个格子无关**。于是规定：只有当 K 恰好是
// 交集的最小角时才吐出这一对。每一对因此有且只有一个格子会吐出它。
// O(1)、无分配、无哈希。
//
//------------------------------------------------------------------------------
// TODO(upgrade): 均匀网格 -> BVH / 多级网格
//   均匀网格的死穴是"只有一个 cellSize"：物体尺寸跨一个数量级以上时，格子对小
//   物体太大（每格挤几十个）、对大物体太小（一个物体跨几百格），两头不讨好。
//
//   失效信号看 ComputeStats()：avgCellsPerProxy 长期大于 10，
//   或者 maxProxiesPerCell 是 avgProxiesPerCell 的几十倍，就该换了。
//
//   两条路：
//     - 分层均匀网格：按尺寸分 2~3 档，每档一张网格（cellSize 逐档翻倍），
//       查询时逐档查。改动小，能吃掉大部分问题。
//     - 动态 AABB 树（BVH）：新增 DynamicBVH 实现同一个 IBroadPhase 即可，
//       PhysicsWorld 一行都不用动。AABB::SurfaceArea() 已经为 SAH 建树备好了。
//==============================================================================

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "pe/collision/AABB.h"
#include "pe/collision/BroadPhase.h"
#include "pe/collision/Ray.h"
#include "pe/core/Types.h"

namespace pe {

//------------------------------------------------------------------------------
// 网格的诊断信息
//
// 均匀网格唯一需要调的参数就是 cellSize，而它调得好不好只能靠实测。
// 这个结构就是给调参用的：把它每隔几秒打一行日志，看 maxProxiesPerCell 和
// avgProxiesPerCell 差多少就知道分布是不是过于集中。
//------------------------------------------------------------------------------
struct UniformGridStats {
    std::size_t proxyCount = 0;         ///< 存活的 proxy 总数
    std::size_t staticCellCount = 0;    ///< 静态表里非空格子数
    std::size_t dynamicCellCount = 0;   ///< 动态表里非空格子数
    std::size_t oversizedCount = 0;     ///< 走暴力路径的超大物体数
    std::size_t maxProxiesPerCell = 0;  ///< 最挤的那个格子里有多少个
    real avgProxiesPerCell = real(0);   ///< 平均每个非空格子里有多少个

    /// 一个 proxy 平均登记进了几个格子。远大于 1 说明 cellSize 相对物体太小。
    real avgCellsPerProxy = real(0);
};

//------------------------------------------------------------------------------
// UniformGrid
//------------------------------------------------------------------------------
class UniformGrid final : public IBroadPhase {
public:
    /// 单个 proxy 允许占据的格子数上限。超过就转入暴力列表。
    ///
    /// 取 4096（相当于 16x16x16 的区间）：足够让"一堵墙""一段楼梯"这类正常的
    /// 大静态体留在网格里享受空间划分的好处，又能挡住地面、天空盒这种真正
    /// 覆盖全场的东西。
    static constexpr std::size_t kMaxCellsPerProxy = 4096;

    /// DDA 单次射线遍历的格子数上限，纯粹是防死循环的保险丝。
    /// 正常情况下遍历会被网格总包围盒截断，根本到不了这个数。
    static constexpr std::size_t kMaxRaySteps = 8192;

    /// cellSize 建议取"场景里典型动态物体尺寸的 2 倍"。
    /// 太小 -> 大物体跨越大量格子，登记与遍历都变贵；
    /// 太大 -> 每个格子里物体太多，退化成暴力配对。
    /// margin 就是 fat AABB 的外扩量，含义见 BroadPhase.h。
    explicit UniformGrid(real cellSize = real(2), real margin = kAabbMargin);

    //-- IBroadPhase ------------------------------------------------------------

    ProxyId Insert(const BroadPhaseProxyDesc& desc) override;
    void Remove(ProxyId id) override;
    bool Update(ProxyId id, const AABB& tightAabb,
                const Vec3& predictedDisplacement = Vec3::Zero()) override;
    void SetSleeping(ProxyId id, bool sleeping) override;

    void QueryPairs(std::vector<BroadPhasePair>& out) const override;
    void QueryAABB(const AABB& box, std::vector<ProxyId>& out) const override;
    void QueryRay(const Ray& ray, std::vector<ProxyId>& out) const override;

    const BroadPhaseProxy* GetProxy(ProxyId id) const override;
    std::size_t ProxyCount() const override { return m_aliveCount; }
    void Clear() override;

    //-- UniformGrid 特有 -------------------------------------------------------

    real CellSize() const noexcept { return m_cellSize; }
    real Margin() const noexcept { return m_margin; }

    /// 所有已登记（不含超大体）proxy 的总包围盒。射线遍历先被它裁剪。
    ///
    /// TODO(upgrade): 它是**只增不减**的 —— 有物体飞到很远处再被销毁之后，
    /// 包围盒不会缩回来，射线遍历会在空区域多走一段（只影响速度，不影响结果，
    /// 因为空格子在哈希表里查不到东西）。真要收紧的话，可以在
    /// ComputeStats() 之类的低频路径上重算一次，或者每隔 N 帧重算。
    const AABB& Bounds() const noexcept { return m_bounds; }

    UniformGridStats ComputeStats() const;

private:
    //--------------------------------------------------------------------------
    // 格子坐标
    //--------------------------------------------------------------------------
    struct CellKey {
        std::int32_t x, y, z;

        friend constexpr bool operator==(CellKey a, CellKey b) noexcept {
            return a.x == b.x && a.y == b.y && a.z == b.z;
        }
    };

    struct CellKeyHash {
        std::size_t operator()(CellKey k) const noexcept;
    };

    /// 闭区间的格子范围（min 和 max 两端都算在内）。
    struct CellRange {
        CellKey min, max;

        constexpr bool Contains(CellKey k) const noexcept {
            return k.x >= min.x && k.x <= max.x && k.y >= min.y && k.y <= max.y &&
                   k.z >= min.z && k.z <= max.z;
        }
    };

    /// 内部 proxy：对外视图 + 网格登记状态。
    struct Proxy {
        BroadPhaseProxy data;
        CellRange range;      ///< 登记进了哪段格子区间（oversized 时无意义）
        bool alive = false;
        bool oversized = false;
    };

    using CellMap = std::unordered_map<CellKey, std::vector<ProxyId>, CellKeyHash>;

    //--------------------------------------------------------------------------
    // 内部工具
    //--------------------------------------------------------------------------
    std::int32_t CoordToCell(real v) const noexcept;
    CellKey PointToCell(const Vec3& p) const noexcept;
    CellRange AabbToRange(const AABB& box) const noexcept;
    static std::size_t RangeCellCount(const CellRange& r) noexcept;

    CellMap& MapFor(bool isStatic) noexcept {
        return isStatic ? m_staticCells : m_dynamicCells;
    }
    const CellMap& MapFor(bool isStatic) const noexcept {
        return isStatic ? m_staticCells : m_dynamicCells;
    }

    void LinkCells(ProxyId id);
    void UnlinkCells(ProxyId id);

    /// 供 QueryAABB / QueryRay 做 O(1) 去重的"访问戳"。
    /// 每次查询把计数器加一，proxy 的戳等于当前计数器就说明这次查询已经收过它。
    /// 比每次查询都新建一个 unordered_set 便宜得多。
    void BeginQuery() const;
    bool TryVisit(ProxyId id) const;

    /// 从一个格子里收集通过测试的 proxy（内部带访问戳去重）。
    /// 两个重载分别服务 QueryAABB 与 QueryRay。
    void CollectCellIf(const CellMap& map, CellKey key, const AABB& box,
                       std::vector<ProxyId>& out) const;
    void CollectCellIf(const CellMap& map, CellKey key, const Ray& ray,
                       std::vector<ProxyId>& out) const;

    /// 格子 key 是不是这一对 proxy 的"归属格"——配对去重的核心，
    /// 原理见本文件顶部"去重：不用哈希集合"。
    static bool IsOwnerCell(CellKey key, const CellRange& a,
                            const CellRange& b) noexcept;

    //--------------------------------------------------------------------------
    // 数据
    //--------------------------------------------------------------------------
    real m_cellSize;
    real m_invCellSize;
    real m_margin;

    std::vector<Proxy> m_proxies;   ///< 按 ProxyId 直接下标索引
    std::vector<ProxyId> m_freeIds; ///< 已注销、可复用的 id
    std::size_t m_aliveCount = 0;

    CellMap m_staticCells;
    CellMap m_dynamicCells;
    std::vector<ProxyId> m_oversized;

    AABB m_bounds;  ///< 所有已登记 proxy 的总包围盒（只增不减）

    mutable std::vector<std::uint32_t> m_visitStamp;
    mutable std::uint32_t m_visitCounter = 0;
};

}  // namespace pe
