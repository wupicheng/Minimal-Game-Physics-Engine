//==============================================================================
// src/collision/UniformGrid.cpp
//
// 均匀网格宽相位的实现。结构与设计取舍见 UniformGrid.h 顶部的说明，
// 这里只写"算法本身为什么这样写"。
//==============================================================================

#include "pe/collision/UniformGrid.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "pe/collision/RayCast.h"
#include "pe/math/MathUtil.h"

namespace pe {

namespace {

//------------------------------------------------------------------------------
// 格子坐标的取值范围
//
// 限制在 ±2^20：cellSize = 2 米时对应 ±2000 公里，任何 FPS 地图都远远用不完，
// 而且留出了足够的余量让 (max - min) 这类差值不会在 int32 里溢出。
// 真正的作用是给"坐标算出了 inf / NaN"兜底：物理爆掉时物体坐标会变成 1e30，
// 没有这个夹取的话它会被转成一个随机的 int32，然后把哈希表撑爆。
//------------------------------------------------------------------------------
constexpr std::int32_t kCellCoordMax = 1 << 20;
constexpr std::int32_t kCellCoordMin = -kCellCoordMax;

/// RangeCellCount 在溢出/超限时返回的哨兵值，保证一定大于 kMaxCellsPerProxy。
constexpr std::size_t kCellCountOverflow =
    UniformGrid::kMaxCellsPerProxy + 1;

/// 把射线裁剪到盒子内，得到进入/离开的参数区间 [tEnter, tExit]。
///
/// 和 RaycastAABB 是同一套平板（slab）判定，区别只在于这里两端都要 ——
/// DDA 需要知道"从哪儿开始走、走到哪儿为止"，而 RaycastAABB 只关心第一个交点。
/// 两者判"中不中"的结论必然一致（同样的比较、同样的容差），
/// 这一点是 QueryRay 的结果能和逐个 RaycastAABB 对拍的前提。
bool ClipRayToAABB(const Ray& ray, const AABB& box, real& tEnter,
                   real& tExit) noexcept {
    real t0 = real(0);
    real t1 = ray.maxDistance;

    for (int i = 0; i < 3; ++i) {
        const real d = ray.direction[i];
        const real o = ray.origin[i];

        if (Abs(d) < kEpsilon) {
            // 平行于这对平板：要么整条射线都在板内（不产生约束），
            // 要么整条都在板外（不可能相交）。单独处理是为了避开 0*inf = NaN。
            if (o < box.min[i] || o > box.max[i]) return false;
            continue;
        }

        const real inv = real(1) / d;
        real tn = (box.min[i] - o) * inv;
        real tf = (box.max[i] - o) * inv;
        if (tn > tf) std::swap(tn, tf);

        if (tn > t0) t0 = tn;
        if (tf < t1) t1 = tf;
        if (t0 > t1) return false;
    }

    tEnter = t0;
    tExit = t1;
    return true;
}

}  // namespace

//==============================================================================
// 构造
//==============================================================================

UniformGrid::UniformGrid(real cellSize, real margin)
    : m_cellSize(cellSize > kEpsilon ? cellSize : real(1)),
      m_invCellSize(real(1) / (cellSize > kEpsilon ? cellSize : real(1))),
      m_margin(margin > real(0) ? margin : real(0)),
      m_bounds(AABB::Invalid()) {}

//==============================================================================
// 格子坐标换算
//==============================================================================

//------------------------------------------------------------------------------
// 哈希：三个坐标各乘一个大奇数异或到一起，再走一遍 splitmix64 的雪崩混合。
//
// 不能简单地用 x*73856093 ^ y*19349663 ^ z*83492791 之后直接返回：
// libstdc++ 的 unordered_map 是拿哈希值对**质数**取模分桶的，低位质量差一点
// 影响不大；但 MSVC 的实现是取低若干位，那种朴素哈希在"物体沿坐标轴排成一排"
// 这种最常见的场景下会大量撞桶。多一次混合换来对实现细节不敏感。
//------------------------------------------------------------------------------
std::size_t UniformGrid::CellKeyHash::operator()(CellKey k) const noexcept {
    std::uint64_t h =
        static_cast<std::uint64_t>(static_cast<std::uint32_t>(k.x)) * 0x9E3779B97F4A7C15ull ^
        static_cast<std::uint64_t>(static_cast<std::uint32_t>(k.y)) * 0xC2B2AE3D27D4EB4Full ^
        static_cast<std::uint64_t>(static_cast<std::uint32_t>(k.z)) * 0x165667B19E3779F9ull;

    h ^= h >> 30;
    h *= 0xBF58476D1CE4E5B9ull;
    h ^= h >> 27;
    h *= 0x94D049BB133111EBull;
    h ^= h >> 31;
    return static_cast<std::size_t>(h);
}

//------------------------------------------------------------------------------
// 世界坐标 -> 格子坐标。
//
// 必须用 floor 而不是截断取整：截断在负数一侧是朝零取整的，会让 -0.5 和 +0.5
// 落进同一个格子，于是原点附近的格子变成两倍大 —— 这类"只在负半轴出问题"的
// bug 极难在正向的测试场景里发现。
//------------------------------------------------------------------------------
std::int32_t UniformGrid::CoordToCell(real v) const noexcept {
    const real c = std::floor(v * m_invCellSize);

    // 写成"不满足下界就夹到下界"的形式，NaN 会因为所有比较都为假而落进这里，
    // 从而避免 static_cast<int32_t>(NaN) 这个未定义行为。
    if (!(c >= static_cast<real>(kCellCoordMin))) return kCellCoordMin;
    if (c > static_cast<real>(kCellCoordMax)) return kCellCoordMax;
    return static_cast<std::int32_t>(c);
}

UniformGrid::CellKey UniformGrid::PointToCell(const Vec3& p) const noexcept {
    return CellKey{CoordToCell(p.x), CoordToCell(p.y), CoordToCell(p.z)};
}

UniformGrid::CellRange UniformGrid::AabbToRange(const AABB& box) const noexcept {
    return CellRange{PointToCell(box.min), PointToCell(box.max)};
}

std::size_t UniformGrid::RangeCellCount(const CellRange& r) noexcept {
    // 用 int64 做差再转 uint64：坐标已被夹在 ±2^20 内，差值最大 2^21，
    // 两两相乘不超过 2^42，不会溢出。
    const std::int64_t dx = static_cast<std::int64_t>(r.max.x) - r.min.x + 1;
    const std::int64_t dy = static_cast<std::int64_t>(r.max.y) - r.min.y + 1;
    const std::int64_t dz = static_cast<std::int64_t>(r.max.z) - r.min.z + 1;

    // 区间反向（传进来的 AABB 本身就是非法的）时当成空，交给上层的 AABB 判定去否。
    if (dx <= 0 || dy <= 0 || dz <= 0) return 0;

    const std::uint64_t limit = static_cast<std::uint64_t>(kCellCountOverflow);
    std::uint64_t n = static_cast<std::uint64_t>(dx);
    if (n >= limit) return kCellCountOverflow;
    n *= static_cast<std::uint64_t>(dy);
    if (n >= limit) return kCellCountOverflow;
    n *= static_cast<std::uint64_t>(dz);
    if (n >= limit) return kCellCountOverflow;

    return static_cast<std::size_t>(n);
}

//==============================================================================
// 网格登记 / 注销
//==============================================================================

void UniformGrid::LinkCells(ProxyId id) {
    Proxy& p = m_proxies[id];

    const CellRange r = AabbToRange(p.data.fatAabb);

    if (RangeCellCount(r) > kMaxCellsPerProxy) {
        // 超大体：不进格子，走暴力列表。
        p.oversized = true;
        p.range = r;  // 存着只为诊断，配对时用不到
        m_oversized.push_back(id);
        return;
    }

    p.oversized = false;
    p.range = r;

    CellMap& map = MapFor(p.data.isStatic);
    for (std::int32_t z = r.min.z; z <= r.max.z; ++z) {
        for (std::int32_t y = r.min.y; y <= r.max.y; ++y) {
            for (std::int32_t x = r.min.x; x <= r.max.x; ++x) {
                map[CellKey{x, y, z}].push_back(id);
            }
        }
    }

    // 总包围盒只增不减：见 Bounds() 的说明。
    m_bounds.Include(p.data.fatAabb);
}

void UniformGrid::UnlinkCells(ProxyId id) {
    Proxy& p = m_proxies[id];

    if (p.oversized) {
        // 超大体的数量是个位数，线性找 + 交换删除足够。
        for (std::size_t i = 0; i < m_oversized.size(); ++i) {
            if (m_oversized[i] == id) {
                m_oversized[i] = m_oversized.back();
                m_oversized.pop_back();
                break;
            }
        }
        p.oversized = false;
        return;
    }

    CellMap& map = MapFor(p.data.isStatic);
    const CellRange& r = p.range;

    for (std::int32_t z = r.min.z; z <= r.max.z; ++z) {
        for (std::int32_t y = r.min.y; y <= r.max.y; ++y) {
            for (std::int32_t x = r.min.x; x <= r.max.x; ++x) {
                const auto it = map.find(CellKey{x, y, z});
                if (it == map.end()) continue;

                std::vector<ProxyId>& list = it->second;
                for (std::size_t i = 0; i < list.size(); ++i) {
                    if (list[i] == id) {
                        list[i] = list.back();
                        list.pop_back();
                        break;
                    }
                }
                // 空格子必须从表里删掉，否则物体走过的路径会留下一串空条目，
                // 长时间运行后配对遍历会在大量空格子上空转。
                if (list.empty()) map.erase(it);
            }
        }
    }
}

//==============================================================================
// 生命周期
//==============================================================================

ProxyId UniformGrid::Insert(const BroadPhaseProxyDesc& desc) {
    ProxyId id;
    if (!m_freeIds.empty()) {
        id = m_freeIds.back();
        m_freeIds.pop_back();
    } else {
        id = static_cast<ProxyId>(m_proxies.size());
        m_proxies.emplace_back();
        m_visitStamp.push_back(0u);
    }

    Proxy& p = m_proxies[id];
    p.data.fatAabb = desc.aabb.Expanded(m_margin);
    p.data.userData = desc.userData;
    p.data.layer = desc.layer;
    p.data.layerMask = desc.layerMask;
    p.data.isStatic = desc.isStatic;
    p.data.isSleeping = desc.isSleeping;
    p.alive = true;

    LinkCells(id);
    ++m_aliveCount;
    return id;
}

void UniformGrid::Remove(ProxyId id) {
    if (id >= m_proxies.size() || !m_proxies[id].alive) return;

    UnlinkCells(id);
    m_proxies[id].alive = false;
    m_freeIds.push_back(id);
    --m_aliveCount;
}

bool UniformGrid::Update(ProxyId id, const AABB& tightAabb,
                         const Vec3& predictedDisplacement) {
    if (id >= m_proxies.size() || !m_proxies[id].alive) return false;

    Proxy& p = m_proxies[id];

    // 常态：物体还在膨胀壳里，什么都不用做。这一行就是 fat AABB 的全部意义。
    if (p.data.fatAabb.Contains(tightAabb)) return false;

    // 判定用紧致 AABB（上面那行），构建壳时才加预测量 —— 顺序反了预测就失效，
    // 理由见 BroadPhase.h 里 Update 的说明。
    AABB newFat = tightAabb.Expanded(m_margin);
    if (!predictedDisplacement.IsZero()) {
        newFat = Sweep(newFat, predictedDisplacement);
    }
    const CellRange newRange = AabbToRange(newFat);
    const bool newOversized = RangeCellCount(newRange) > kMaxCellsPerProxy;

    // 次常态：壳跟不上了要重算，但覆盖的格子区间恰好没变（物体在格子内部
    // 移动了一小段）。这时只换 AABB，不动哈希表 —— 省掉一次注销 + 一次登记。
    if (!p.oversized && !newOversized && newRange.min == p.range.min &&
        newRange.max == p.range.max) {
        p.data.fatAabb = newFat;
        m_bounds.Include(newFat);
        return true;
    }

    UnlinkCells(id);
    p.data.fatAabb = newFat;
    LinkCells(id);
    return true;
}

void UniformGrid::SetSleeping(ProxyId id, bool sleeping) {
    if (id >= m_proxies.size() || !m_proxies[id].alive) return;
    m_proxies[id].data.isSleeping = sleeping;
}

const BroadPhaseProxy* UniformGrid::GetProxy(ProxyId id) const {
    if (id >= m_proxies.size() || !m_proxies[id].alive) return nullptr;
    return &m_proxies[id].data;
}

void UniformGrid::Clear() {
    m_staticCells.clear();
    m_dynamicCells.clear();
    m_oversized.clear();
    m_proxies.clear();
    m_freeIds.clear();
    m_visitStamp.clear();
    m_aliveCount = 0;
    m_bounds = AABB::Invalid();
}

//==============================================================================
// 查询：候选对
//==============================================================================

void UniformGrid::QueryPairs(std::vector<BroadPhasePair>& out) const {
    out.clear();

    //--------------------------------------------------------------------------
    // 通道一：网格。
    //
    // 只遍历**动态**表的格子。静态体之间永远不会被配对，因为静态表的格子
    // 根本不会成为遍历的起点 —— 这比"先配对再用规则过滤掉"省下的不是一点半点。
    //--------------------------------------------------------------------------
    for (const auto& entry : m_dynamicCells) {
        const CellKey key = entry.first;
        const std::vector<ProxyId>& dyn = entry.second;

        // 动态 vs 动态
        for (std::size_t i = 0; i + 1 < dyn.size(); ++i) {
            const Proxy& a = m_proxies[dyn[i]];
            for (std::size_t j = i + 1; j < dyn.size(); ++j) {
                const Proxy& b = m_proxies[dyn[j]];
                if (!IsOwnerCell(key, a.range, b.range)) continue;
                if (!BroadPhaseShouldCollide(a.data, b.data)) continue;
                out.emplace_back(dyn[i], dyn[j]);
            }
        }

        // 动态 vs 静态：同一个格子 key 去静态表里查一次
        const auto sIt = m_staticCells.find(key);
        if (sIt == m_staticCells.end()) continue;

        for (const ProxyId did : dyn) {
            const Proxy& a = m_proxies[did];
            for (const ProxyId sid : sIt->second) {
                const Proxy& b = m_proxies[sid];
                if (!IsOwnerCell(key, a.range, b.range)) continue;
                if (!BroadPhaseShouldCollide(a.data, b.data)) continue;
                out.emplace_back(did, sid);
            }
        }
    }

    //--------------------------------------------------------------------------
    // 通道二：超大体。它们不在任何格子里，只能和别人逐个比。
    //
    // 复杂度是 O(超大体数 * 总体数)。之所以能接受：超大体按定义是"覆盖大半张
    // 地图"的东西，一张 FPS 地图里也就地面加几堵外墙，个位数。
    // BroadPhaseShouldCollide 的第一个判断（双方都不醒着）是纯位运算，
    // 静态地面 vs 上千个静态箱子会在那一行就被否掉。
    //--------------------------------------------------------------------------
    for (std::size_t i = 0; i < m_oversized.size(); ++i) {
        const ProxyId oid = m_oversized[i];
        const Proxy& o = m_proxies[oid];

        // 超大 vs 超大：靠 i < j 保证每对只出现一次
        for (std::size_t j = i + 1; j < m_oversized.size(); ++j) {
            const ProxyId otherId = m_oversized[j];
            if (!BroadPhaseShouldCollide(o.data, m_proxies[otherId].data)) continue;
            out.emplace_back(oid, otherId);
        }

        // 超大 vs 网格内的：网格内的那一方不会在别处再吐出这一对，所以不会重
        for (std::size_t pid = 0; pid < m_proxies.size(); ++pid) {
            const Proxy& p = m_proxies[pid];
            if (!p.alive || p.oversized) continue;
            if (!BroadPhaseShouldCollide(o.data, p.data)) continue;
            out.emplace_back(oid, static_cast<ProxyId>(pid));
        }
    }
}

//==============================================================================
// 查询：AABB 重叠
//==============================================================================

void UniformGrid::QueryAABB(const AABB& box, std::vector<ProxyId>& out) const {
    out.clear();
    BeginQuery();

    const CellRange r = AabbToRange(box);

    // 查询盒本身就大到跨越几千个格子时，逐格遍历比直接扫一遍所有 proxy 还贵。
    // 这是和"超大 proxy 走暴力列表"完全对称的取舍，阈值也共用同一个。
    if (RangeCellCount(r) > kMaxCellsPerProxy) {
        for (std::size_t pid = 0; pid < m_proxies.size(); ++pid) {
            const Proxy& p = m_proxies[pid];
            if (!p.alive) continue;
            if (box.Overlaps(p.data.fatAabb)) out.push_back(static_cast<ProxyId>(pid));
        }
        return;
    }

    for (std::int32_t z = r.min.z; z <= r.max.z; ++z) {
        for (std::int32_t y = r.min.y; y <= r.max.y; ++y) {
            for (std::int32_t x = r.min.x; x <= r.max.x; ++x) {
                const CellKey key{x, y, z};
                CollectCellIf(m_staticCells, key, box, out);
                CollectCellIf(m_dynamicCells, key, box, out);
            }
        }
    }

    for (const ProxyId id : m_oversized) {
        if (TryVisit(id) && box.Overlaps(m_proxies[id].data.fatAabb)) out.push_back(id);
    }
}

//==============================================================================
// 查询：射线（3D DDA）
//==============================================================================
//
// 用的是 Amanatides & Woo 的体素遍历（"A Fast Voxel Traversal Algorithm for Ray
// Tracing", 1987）。核心思想：射线穿过网格时，每次只会跨过 x / y / z 三族平面
// 中的一族，所以只要维护"沿射线走到下一个 x 平面 / y 平面 / z 平面各需要多少
// 距离"这三个值，每步取最小的那个推进即可 —— 每步只有三次比较和一次加法，
// 没有除法，也不需要重新计算交点。
//
// 它天然是**由近及远**的，所以上层做精确求交时可以在第一次命中之后提前退出。
//
// 起步之前先把射线裁到网格的总包围盒里：不裁的话，一条射程 1000 米的射线在
// cellSize = 0.25 米的网格里要走 4000 步，其中绝大多数是空格子。
//==============================================================================

void UniformGrid::QueryRay(const Ray& ray, std::vector<ProxyId>& out) const {
    out.clear();
    BeginQuery();

    RaycastHit scratch;  // 只用来接住返回值，宽相位不关心命中点和法线

    // 超大体不在格子里，逐个测。
    for (const ProxyId id : m_oversized) {
        if (TryVisit(id) && RaycastAABB(m_proxies[id].data.fatAabb, ray, scratch)) {
            out.push_back(id);
        }
    }

    if (m_staticCells.empty() && m_dynamicCells.empty()) return;

    real tEnter = real(0);
    real tExit = real(0);
    if (!ClipRayToAABB(ray, m_bounds, tEnter, tExit)) return;

    //--------------------------------------------------------------------------
    // 初始化
    //--------------------------------------------------------------------------
    const Vec3 start = ray.PointAt(tEnter);

    std::int32_t cell[3] = {CoordToCell(start.x), CoordToCell(start.y),
                            CoordToCell(start.z)};
    std::int32_t step[3];
    real tMax[3];    // 沿射线走到下一个该轴平面所需的参数
    real tDelta[3];  // 跨过一整个格子在该轴上要走的参数增量

    const real kBig = std::numeric_limits<real>::max();

    for (int i = 0; i < 3; ++i) {
        const real d = ray.direction[i];

        if (Abs(d) < kEpsilon) {
            // 与该轴的平面平行，永远跨不过去。
            step[i] = 0;
            tMax[i] = kBig;
            tDelta[i] = kBig;
            continue;
        }

        step[i] = d > real(0) ? 1 : -1;

        // 前方那张平面的世界坐标：朝正方向走是当前格子的右边界（cell+1），
        // 朝负方向走是左边界（cell）。
        const std::int32_t boundaryCell = cell[i] + (d > real(0) ? 1 : 0);
        const real boundary = static_cast<real>(boundaryCell) * m_cellSize;

        tMax[i] = tEnter + (boundary - start[i]) / d;
        tDelta[i] = m_cellSize / Abs(d);
    }

    //--------------------------------------------------------------------------
    // 推进
    //
    // 步数上限只是保险丝：正常情况下 tMax > tExit 会先一步结束循环。
    //--------------------------------------------------------------------------
    for (std::size_t steps = 0; steps < kMaxRaySteps; ++steps) {
        const CellKey key{cell[0], cell[1], cell[2]};
        CollectCellIf(m_staticCells, key, ray, out);
        CollectCellIf(m_dynamicCells, key, ray, out);

        int axis = 0;
        if (tMax[1] < tMax[axis]) axis = 1;
        if (tMax[2] < tMax[axis]) axis = 2;

        // 下一次跨面已经在射线终点（或包围盒出口）之外了，遍历结束。
        if (tMax[axis] > tExit) break;

        cell[axis] += step[axis];
        tMax[axis] += tDelta[axis];
    }
}

//==============================================================================
// 查询辅助
//==============================================================================

void UniformGrid::BeginQuery() const {
    // 计数器回绕时把所有戳清零重来。uint32 的周期是 43 亿次查询，
    // 按每帧 100 次算能跑一年多，但"几乎不会发生"不等于"不会发生"。
    if (m_visitCounter == 0xFFFFFFFFu) {
        std::fill(m_visitStamp.begin(), m_visitStamp.end(), 0u);
        m_visitCounter = 0u;
    }
    ++m_visitCounter;
}

bool UniformGrid::TryVisit(ProxyId id) const {
    if (m_visitStamp[id] == m_visitCounter) return false;
    m_visitStamp[id] = m_visitCounter;
    return true;
}

void UniformGrid::CollectCellIf(const CellMap& map, CellKey key, const AABB& box,
                                std::vector<ProxyId>& out) const {
    const auto it = map.find(key);
    if (it == map.end()) return;

    for (const ProxyId id : it->second) {
        // 先打戳再做几何测试：一个 proxy 横跨多个格子时，第二次遇到它就直接跳过，
        // 连 AABB 测试都省了 —— 不管上次的结论是中还是不中，结论都不会变。
        if (!TryVisit(id)) continue;
        if (box.Overlaps(m_proxies[id].data.fatAabb)) out.push_back(id);
    }
}

void UniformGrid::CollectCellIf(const CellMap& map, CellKey key, const Ray& ray,
                                std::vector<ProxyId>& out) const {
    const auto it = map.find(key);
    if (it == map.end()) return;

    RaycastHit scratch;
    for (const ProxyId id : it->second) {
        if (!TryVisit(id)) continue;
        if (RaycastAABB(m_proxies[id].data.fatAabb, ray, scratch)) out.push_back(id);
    }
}

bool UniformGrid::IsOwnerCell(CellKey key, const CellRange& a,
                              const CellRange& b) noexcept {
    return key.x == (a.min.x > b.min.x ? a.min.x : b.min.x) &&
           key.y == (a.min.y > b.min.y ? a.min.y : b.min.y) &&
           key.z == (a.min.z > b.min.z ? a.min.z : b.min.z);
}

//==============================================================================
// 诊断
//==============================================================================

UniformGridStats UniformGrid::ComputeStats() const {
    UniformGridStats s;
    s.proxyCount = m_aliveCount;
    s.staticCellCount = m_staticCells.size();
    s.dynamicCellCount = m_dynamicCells.size();
    s.oversizedCount = m_oversized.size();

    std::size_t entries = 0;
    for (const CellMap* map : {&m_staticCells, &m_dynamicCells}) {
        for (const auto& entry : *map) {
            entries += entry.second.size();
            s.maxProxiesPerCell = std::max(s.maxProxiesPerCell, entry.second.size());
        }
    }

    const std::size_t cells = s.staticCellCount + s.dynamicCellCount;
    if (cells > 0) {
        s.avgProxiesPerCell =
            static_cast<real>(entries) / static_cast<real>(cells);
    }
    // 分母只算进了格子的 proxy：超大体一个格子都不占，算进去会把这个指标稀释掉。
    const std::size_t gridded = m_aliveCount - m_oversized.size();
    if (gridded > 0) {
        s.avgCellsPerProxy =
            static_cast<real>(entries) / static_cast<real>(gridded);
    }
    return s;
}

}  // namespace pe
