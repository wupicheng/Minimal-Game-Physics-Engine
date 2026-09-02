#pragma once
//==============================================================================
// pe/solver/SequentialImpulseSolver.h
//
// 顺序冲量求解器（Sequential Impulse），以及它依赖的接触缓存。
//
//------------------------------------------------------------------------------
// "顺序"是什么意思
//------------------------------------------------------------------------------
// 一帧里有几百个接触点，它们互相耦合：解开 A-B 之间的接触会改变 B 的速度，
// 于是 B-C 之间刚解好的接触又被破坏了。严格解法是把所有约束写成一个大的
// 线性互补问题（LCP）一次性解出来 —— 但那是 O(n^3) 的稠密矩阵求解，
// 实时完全做不到。
//
// 顺序冲量是它的 **Gauss-Seidel 迭代版本**：一次只解一个接触点，解完立刻把速度
// 更新写回去，下一个接触点用的就是更新过的值。跑若干轮（默认 8 轮），
// 结果会收敛到 LCP 解的附近。
//
// "立刻写回"这一点很关键 —— 如果攒到一轮结束再统一更新（Jacobi 迭代），
// 收敛速度会慢好几倍。代价是**结果依赖求解顺序**，所以顺序必须是确定的
// （见下面关于确定性的说明）。
//
//------------------------------------------------------------------------------
// 接触缓存与 warm starting
//------------------------------------------------------------------------------
// 8 轮迭代解不出精确答案。对单次碰撞无所谓，但对**堆叠**是致命的：
// 一摞 10 个箱子，最下面那个要撑住上面 9 个的重量，8 轮迭代只够把冲量推到
// 正确值的一部分，于是每帧都下陷一点点 —— 肉眼看就是箱子堆在缓慢地"沉进地板"。
//
// 解法是 warm starting：一摞静止的箱子每帧的受力几乎完全一样，所以上一帧解出的
// 冲量就是这一帧极好的初始猜测。从它出发，几轮迭代就够了。
//
// 要做到这一点，就得**跨帧记住每个接触点的累积冲量**。而接触点每帧都在重新生成，
// 所以需要一个身份标识把新旧接触点对应起来 —— 那就是 `featureId`
// （由窄相位生成，见 Manifold.h）加上 `(colliderA, colliderB)` 这个对键。
//
// 这就是 ContactCache 的全部职责。
//
//------------------------------------------------------------------------------
// 确定性
//------------------------------------------------------------------------------
// Gauss-Seidel 的结果依赖求解顺序，所以约束的加入顺序必须是确定的 ——
// 求解器内部用 `std::vector` 按加入顺序求解，不做任何排序或并行。
// 上层（宽相位）遍历哈希表的顺序在同一次运行里是确定的，但**跨进程不保证**
// （哈希表的迭代顺序可能受内存布局影响）。要做网络确定性同步的话，
// 得在这一层之前对候选对排序 —— 见 UPGRADE_NOTES。
//==============================================================================

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "pe/collision/Manifold.h"
#include "pe/dynamics/Material.h"
#include "pe/dynamics/RigidBody.h"
#include "pe/solver/ContactConstraint.h"
#include "pe/solver/IConstraint.h"

namespace pe {

//------------------------------------------------------------------------------
// 接触缓存
//
// 按 (pairKey, featureId) 存上一帧的累积冲量。
//------------------------------------------------------------------------------
class ContactCache {
public:
    /// 单个接触对缓存下来的东西。最多 4 个点，与 Manifold 对齐。
    struct CachedPoint {
        std::uint32_t featureId;
        real normalImpulse;
        real tangentImpulse[2];
    };

    struct Entry {
        CachedPoint points[4];
        std::uint8_t count;
        /// 这一条上次被用到是哪一帧。用来清理已经消失的接触 —— 少了这一步，
        /// 一局游戏下来哈希表会长满几万条早已不存在的接触。
        std::uint32_t lastFrame;
    };

    /// 帧号 +1。EndFrame 会把上一帧没碰过的条目删掉。
    void BeginFrame() noexcept { ++m_frame; }

    /// 查上一帧同一个接触点的累积冲量。找不到返回 false（新接触，从零开始）。
    bool Lookup(std::uint64_t pairKey, std::uint32_t featureId,
                CachedPoint& out) const noexcept;

    /// 把本帧解出来的累积冲量写回。
    void Store(const ContactConstraint& constraint) noexcept;

    /// 删掉本帧没有被 Store 过的条目（接触已经分开了）。
    void EndFrame() noexcept;

    void Clear() noexcept {
        m_entries.clear();
        m_frame = 0;
    }

    std::size_t Size() const noexcept { return m_entries.size(); }

private:
    std::unordered_map<std::uint64_t, Entry> m_entries;
    std::uint32_t m_frame = 0;
};

//------------------------------------------------------------------------------
// 一次接触的输入
//------------------------------------------------------------------------------
struct ContactInput {
    RigidBody* bodyA;
    RigidBody* bodyB;
    const Manifold* manifold;
    Material materialA;
    Material materialB;

    /// 稳定标识。同一对碰撞体在相邻帧必须给出同一个值，否则 warm starting 失效。
    /// PhysicsWorld 会把两个 ColliderHandle 打包进来。
    std::uint64_t pairKey;
};

//------------------------------------------------------------------------------
// 求解器
//
// 用法（PhysicsWorld 的 Step 内部）：
//     solver.BeginFrame();
//     for (每个接触)  solver.AddContact(...);
//     for (每个关节)  solver.AddConstraint(...);
//     solver.Solve(dt);          // 内部：Prepare -> WarmStart -> N 次迭代
//     solver.EndFrame();         // 回写缓存、清理消失的接触
//------------------------------------------------------------------------------
class SequentialImpulseSolver {
public:
    SequentialImpulseSolver() = default;
    explicit SequentialImpulseSolver(const SolverConfig& config) : m_config(config) {}

    const SolverConfig& Config() const noexcept { return m_config; }
    void SetConfig(const SolverConfig& config) noexcept { m_config = config; }

    /// 清空本帧的约束列表并推进缓存的帧号。
    void BeginFrame() noexcept;

    /// 加一个接触。会立刻从缓存里取出上一帧的累积冲量。
    /// 流形为空（pointCount == 0）时忽略。
    void AddContact(const ContactInput& input);

    /// 加一个走虚接口的约束（关节等）。求解器不持有它的生命周期。
    void AddConstraint(IConstraint* constraint) { m_constraints.push_back(constraint); }

    /// 求解：Prepare -> WarmStart -> velocityIterations 轮迭代。
    /// 直接修改刚体的速度，**不**动位置 —— 位置由 IntegratePosition 负责。
    void Solve(real dt);

    /// 把本帧的累积冲量写回缓存，并清理已经消失的接触。
    void EndFrame() noexcept;

    void Clear() noexcept;

    //-- 诊断 -------------------------------------------------------------------

    std::size_t ContactCount() const noexcept { return m_contacts.size(); }

    /// 本帧所有接触点里最深的穿透。调堆叠时盯这个值：
    /// 它应该稳定在 linearSlop 附近，持续增长就说明求解器没跟上
    /// （迭代次数不够，或者 warm starting 失效了）。
    real MaxPenetration() const noexcept;

    /// 本帧施加的法向冲量总和。可以用来检验受力是否合理
    /// （静止堆叠时它应该约等于 总重量 * dt）。
    real TotalNormalImpulse() const noexcept;

    const std::vector<ContactConstraint>& Contacts() const noexcept { return m_contacts; }
    const ContactCache& Cache() const noexcept { return m_cache; }

private:
    SolverConfig m_config;
    std::vector<ContactConstraint> m_contacts;
    std::vector<IConstraint*> m_constraints;
    ContactCache m_cache;
};

}  // namespace pe
