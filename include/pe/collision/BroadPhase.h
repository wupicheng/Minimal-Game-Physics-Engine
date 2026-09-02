#pragma once
//==============================================================================
// pe/collision/BroadPhase.h
//
// 宽相位（Broad Phase）的抽象接口与公共数据结构。
//
//------------------------------------------------------------------------------
// 宽相位是干什么的
//------------------------------------------------------------------------------
// n 个碰撞体两两配对是 O(n^2)。2000 个静态体 + 100 个动态体，暴力配对每帧要做
// 约 220 万次 AABB 测试 —— 光这一步就能吃掉整个 16.6 ms 的帧预算。
// 宽相位用空间划分把它降到接近 O(n)：只有"空间上挨得近"的对才会被吐出来。
//
// 宽相位的输出是**候选对**，不是碰撞。它只保证：真正相交的对一定在输出里
// （不能漏报 / no false negative）。多报几对无所谓，窄相位会把它们否掉。
// 这条不对称的要求贯穿整个实现 —— 所有边界判定取闭区间、所有取整朝外扩。
//
//------------------------------------------------------------------------------
// 为什么要有抽象接口
//------------------------------------------------------------------------------
// 第一版实现是均匀网格（UniformGrid），它假设物体尺寸和分布大致均匀 —— 这对
// FPS 地图成立。等到场景变成"几十公里的开阔地 + 几个密集城区"，均匀网格就会
// 退化（格子太大则每格几百个物体，太小则大物体跨越上万个格子），那时要换成
// BVH 或八叉树。换实现时 PhysicsWorld 的代码一行都不用改 —— 这就是这个接口
// 存在的全部理由。
//
//------------------------------------------------------------------------------
// 代理（proxy）与 fat AABB
//------------------------------------------------------------------------------
// 每个碰撞体在宽相位里注册一个 proxy，proxy 缓存它的世界 AABB 并外扩一圈
// margin，称为 "fat AABB"。物体在这层膨胀壳内小幅移动时，proxy 占据的格子集合
// 不变，于是不需要动哈希表 —— 这是宽相位最主要的省钱手段。
//
// 配对判定用的是 **fat AABB**，不是紧致 AABB。这不是偷懒，而是必需的：
// 求解器的 warm starting 要求接触流形跨帧存活，所以候选对必须在两个物体**真正
// 接触之前**就产生，让流形有机会在第一帧接触时就带着上一帧的累积冲量。用紧致
// AABB 配对会让流形在接触的那一帧才诞生，堆叠会明显地"陷一下再弹回来"。
// 代价是两个物体相距 2*margin 以内就会被报成候选对，窄相位多跑几次。
//
//------------------------------------------------------------------------------
// TODO(upgrade): 速度感知的 fat AABB
//   固定的 kAabbMargin = 0.05 米对 FPS 的速度来说太小了：5 m/s 的物体在 1/60 秒
//   里走 0.083 米，直接跨出膨胀壳。实测（2000 静态 + 100 动态）有 76% 的 Update
//   触发了格子重建 —— 也就是说 fat AABB 这个机制目前基本等于没开。
//
//   修法是把 margin 拆成"固定量 + 沿速度方向的扫掠量"：
//       fat = Sweep(tight.Expanded(margin), velocity * dt * k)   // k 取 2~4
//   AABB.h 里的 Sweep() 就是为此准备的。
//
//   注意**不要**把速度塞进这个接口 —— 宽相位按设计不知道速度的存在
//   （ARCHITECTURE.md §2）。正确做法是 M6 有了刚体速度之后，由 PhysicsWorld
//   自己算好扫掠 AABB 再传进 Update()。
//==============================================================================

#include <cstdint>
#include <vector>

#include "pe/collision/AABB.h"
#include "pe/collision/Ray.h"
#include "pe/core/Types.h"

namespace pe {

//------------------------------------------------------------------------------
// ProxyId
//
// 故意用裸的 uint32 而不是带代际号的 Handle：proxy 的生命周期完全由上层
// （PhysicsWorld）掌握，与 Collider 一一对应、同生共死，不存在"用户手里攥着一个
// 过期 proxy id"的场景 —— 用户手里攥的是 ColliderHandle，那一层才需要代际号。
// 宽相位是引擎内部结构，再套一层代际检查是纯粹的开销。
//------------------------------------------------------------------------------
using ProxyId = std::uint32_t;

inline constexpr ProxyId kInvalidProxyId = kInvalidIndex;

//------------------------------------------------------------------------------
// 注册 proxy 时提供的描述
//------------------------------------------------------------------------------
struct BroadPhaseProxyDesc {
    /// 紧致的世界 AABB。宽相位自己负责外扩成 fat AABB，调用方不要预先扩。
    AABB aabb;

    /// 透传给上层的关联数据，宽相位不解释它的含义。
    /// PhysicsWorld 会把 ColliderHandle（index + generation 共 64 位）打包进来，
    /// 这样查询结果能直接还原成句柄，而 collision 层始终不知道句柄的存在。
    std::uint64_t userData = 0;

    /// 碰撞过滤，语义见 Types.h：双方都同意才检测。
    LayerMask layer = layers::kDefault;
    LayerMask layerMask = kLayerAll;

    /// 静态物体（永不移动）。静态体会被放进独立的一张网格，构建一次就不再更新，
    /// 且静态-静态对直接丢弃。FPS 地图里静态体占绝大多数，这个拆分收益很大。
    ///
    /// 注意：注册之后**不能**改这个标志（要改就 Remove 再 Insert）。
    /// 允许中途切换意味着要在两张网格之间搬家，而这个需求在真实场景里几乎不存在
    /// —— 会动的电梯、门从一开始就该注册成非静态。
    bool isStatic = false;

    /// 休眠标志。双方都不"醒着"的对会被丢弃，见 BroadPhaseShouldCollide。
    bool isSleeping = false;
};

//------------------------------------------------------------------------------
// 宽相位里存的 proxy 状态（对外的只读视图）
//------------------------------------------------------------------------------
struct BroadPhaseProxy {
    AABB fatAabb;  ///< 外扩过 margin 的世界 AABB；配对与查询用的都是它
    std::uint64_t userData;
    LayerMask layer;
    LayerMask layerMask;
    bool isStatic;
    bool isSleeping;
};

//------------------------------------------------------------------------------
// 候选对
//
// 约定 a < b。有序化有两个目的：配对时去重只需要比较一次，以及上层的接触流形
// 缓存能用 (a, b) 直接做哈希键，不必考虑两种顺序。
//------------------------------------------------------------------------------
struct BroadPhasePair {
    ProxyId a;
    ProxyId b;

    BroadPhasePair() = default;

    /// 自动把两个 id 排序，调用方不用操心传入顺序。
    constexpr BroadPhasePair(ProxyId i, ProxyId j) noexcept
        : a(i < j ? i : j), b(i < j ? j : i) {}

    friend constexpr bool operator==(BroadPhasePair l, BroadPhasePair r) noexcept {
        return l.a == r.a && l.b == r.b;
    }
    friend constexpr bool operator!=(BroadPhasePair l, BroadPhasePair r) noexcept {
        return !(l == r);
    }
    /// 全序，方便排序后做对拍与二分查找。
    friend constexpr bool operator<(BroadPhasePair l, BroadPhasePair r) noexcept {
        return l.a != r.a ? l.a < r.a : l.b < r.b;
    }
};

//------------------------------------------------------------------------------
// 配对过滤
//
// 放在这里而不是塞进某个实现内部，是因为**所有** IBroadPhase 实现必须用完全同
// 一套过滤规则，否则换实现会静默地改变游戏行为。测试里的暴力参考实现也直接调用
// 它 —— 这样对拍检验的才是"空间划分逻辑对不对"，而不是"过滤规则有没有抄错"。
//------------------------------------------------------------------------------

/// 一个 proxy 是否"醒着"。静态体永远不醒 —— 它不会自己动，所以只涉及静态体和
/// 睡着的物体的接触不可能在这一帧发生变化，整对可以直接丢掉。
inline constexpr bool BroadPhaseIsAwake(const BroadPhaseProxy& p) noexcept {
    return !p.isStatic && !p.isSleeping;
}

/// 两个 proxy 是否应该被吐给窄相位。
///
/// 判断顺序是刻意安排的：先做最便宜、最容易否掉的（标志位与掩码是纯整数运算），
/// 最后才做需要 6 次浮点比较的 AABB 重叠。
inline constexpr bool BroadPhaseShouldCollide(const BroadPhaseProxy& x,
                                              const BroadPhaseProxy& y) noexcept {
    // 双方都不醒着：这一条包含了"静态-静态"这个最常见的情况。
    // FPS 地图里静态体上千个，它挡掉的对数比其它所有规则加起来还多。
    if (!BroadPhaseIsAwake(x) && !BroadPhaseIsAwake(y)) return false;

    // 层过滤：必须双方都同意（理由见 Types.h）。
    if ((x.layer & y.layerMask) == 0u) return false;
    if ((y.layer & x.layerMask) == 0u) return false;

    return x.fatAabb.Overlaps(y.fatAabb);
}

//------------------------------------------------------------------------------
// IBroadPhase
//------------------------------------------------------------------------------
class IBroadPhase {
public:
    virtual ~IBroadPhase() = default;

    //-- 生命周期 ---------------------------------------------------------------

    /// 注册一个 proxy，返回它的 id。id 会在 Remove 之后被复用。
    virtual ProxyId Insert(const BroadPhaseProxyDesc& desc) = 0;

    /// 注销。传入无效或已注销的 id 是安全的空操作。
    virtual void Remove(ProxyId id) = 0;

    /// 物体移动后同步它的紧致世界 AABB。
    ///
    /// 返回值 = "这次调用真的动了内部结构吗"。新 AABB 仍落在 fat AABB 里时返回
    /// false 并且什么都不做 —— 这是常态。上层可以用这个返回值做统计来判断
    /// margin 调得合不合适（返回 true 的比例长期高于三成就说明 margin 太小）。
    ///
    /// `predictedDisplacement` 是**只影响新建的 fat AABB、不影响判定**的预测量：
    /// 重建 fat AABB 时会额外沿它扫掠一段，让物体在接下来的几帧里都待在壳内。
    ///
    /// 这个参数**必须**和 tightAabb 分开传，不能由调用方自己把 AABB 扫掠好再传进来
    /// —— 那样的话下一帧传进来的"已扫掠 AABB"同样朝前伸出一截，永远装不进
    /// 上一帧的壳，于是每帧都要重建，预测完全失效。实测过：两种写法的重建率
    /// 都是 94%，一模一样地没用。判定必须用紧致 AABB，预测只能加在壳上。
    virtual bool Update(ProxyId id, const AABB& tightAabb,
                        const Vec3& predictedDisplacement = Vec3::Zero()) = 0;

    /// 更新休眠标志。休眠是 dynamics 层的概念，宽相位只是拿它做过滤。
    virtual void SetSleeping(ProxyId id, bool sleeping) = 0;

    //-- 查询 -------------------------------------------------------------------

    /// 吐出这一帧所有候选对（已去重、已过滤）。out 会先被清空。
    ///
    /// 保证：输出的对集合与"对所有 proxy 两两调用 BroadPhaseShouldCollide"的
    /// 结果**完全一致**（不多不少）。测试就是按这一条对拍的。
    virtual void QueryPairs(std::vector<BroadPhasePair>& out) const = 0;

    /// 所有 fat AABB 与给定盒子重叠的 proxy。out 会先被清空。
    /// 用途：爆炸伤害范围、触发区域的粗筛、编辑器框选。
    ///
    /// 这里**不做**层过滤 —— 层是"物体之间要不要碰"的规则，而查询的发起方是一个
    /// 凭空来的盒子，它没有自己的层。需要过滤请在拿到 userData 之后由上层自己做。
    virtual void QueryAABB(const AABB& box, std::vector<ProxyId>& out) const = 0;

    /// 所有 fat AABB 被射线击中的 proxy。out 会先被清空。
    ///
    /// 保证：结果与"对所有 proxy 逐个做射线-AABB 测试"完全一致。
    ///
    /// 返回的是**候选**，没有精确命中点：宽相位不知道形状。上层拿到候选后逐个做
    /// 精确求交再取最近的那个。之所以不在这里就地按距离排序：按格子推进的遍历
    /// 天然是由近及远的（见 UniformGrid 的 DDA 说明），上层往往能在第一次精确
    /// 命中之后就提前退出，排序反而是浪费。
    virtual void QueryRay(const Ray& ray, std::vector<ProxyId>& out) const = 0;

    //-- 访问 -------------------------------------------------------------------

    /// 取 proxy 的只读状态；id 无效时返回 nullptr。
    virtual const BroadPhaseProxy* GetProxy(ProxyId id) const = 0;

    /// 存活的 proxy 数量。
    virtual std::size_t ProxyCount() const = 0;

    /// 清空所有 proxy。
    virtual void Clear() = 0;
};

}  // namespace pe
