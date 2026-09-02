//==============================================================================
// src/solver/SequentialImpulseSolver.cpp
//
// 求解循环与接触缓存。原理见 SequentialImpulseSolver.h。
//==============================================================================

#include "pe/solver/SequentialImpulseSolver.h"

namespace pe {

//==============================================================================
// 接触缓存
//==============================================================================

bool ContactCache::Lookup(std::uint64_t pairKey, std::uint32_t featureId,
                          CachedPoint& out) const noexcept {
    const auto it = m_entries.find(pairKey);
    if (it == m_entries.end()) return false;

    // 按 featureId 线性找。最多 4 个点，线性扫描比任何索引结构都快。
    const Entry& entry = it->second;
    for (std::uint8_t i = 0; i < entry.count; ++i) {
        if (entry.points[i].featureId == featureId) {
            out = entry.points[i];
            return true;
        }
    }
    return false;
}

void ContactCache::Store(const ContactConstraint& constraint) noexcept {
    Entry& entry = m_entries[constraint.pairKey];
    entry.count = constraint.pointCount;
    entry.lastFrame = m_frame;

    for (std::uint8_t i = 0; i < constraint.pointCount; ++i) {
        entry.points[i].featureId = constraint.points[i].featureId;
        entry.points[i].normalImpulse = constraint.points[i].normalImpulse;
        entry.points[i].tangentImpulse[0] = constraint.points[i].tangentImpulse[0];
        entry.points[i].tangentImpulse[1] = constraint.points[i].tangentImpulse[1];
    }
}

void ContactCache::EndFrame() noexcept {
    // 删掉本帧没被 Store 过的条目 —— 那些接触已经分开了。
    // 少了这一步，一局游戏下来哈希表会长满几万条早已不存在的接触，
    // 内存和查找都会慢慢劣化。
    for (auto it = m_entries.begin(); it != m_entries.end();) {
        if (it->second.lastFrame != m_frame) {
            it = m_entries.erase(it);
        } else {
            ++it;
        }
    }
}

//==============================================================================
// 求解器
//==============================================================================

void SequentialImpulseSolver::BeginFrame() noexcept {
    m_contacts.clear();
    m_constraints.clear();
    m_cache.BeginFrame();
}

void SequentialImpulseSolver::AddContact(const ContactInput& input) {
    if (input.manifold == nullptr || input.manifold->pointCount == 0) return;
    if (input.bodyA == nullptr || input.bodyB == nullptr) return;

    // 两个都推不动就没必要解 —— 静态对静态、或者两个都在睡。
    // 宽相位那一层已经滤掉了大部分，这里是最后一道。
    if (input.bodyA->invMass <= real(0) && input.bodyB->invMass <= real(0)) return;

    ContactConstraint constraint =
        MakeContactConstraint(input.bodyA, input.bodyB, *input.manifold,
                              input.materialA, input.materialB, input.pairKey);

    //--------------------------------------------------------------------------
    // Warm starting：按 featureId 把上一帧的累积冲量接过来。
    //
    // 对不上的点（新产生的接触、或者接触特征变了）从零开始，这是正确的行为 ——
    // 硬套一个不相干的冲量比从零开始更糟。
    //--------------------------------------------------------------------------
    if (m_config.warmStarting) {
        for (std::uint8_t i = 0; i < constraint.pointCount; ++i) {
            ContactCache::CachedPoint cached;
            if (!m_cache.Lookup(constraint.pairKey, constraint.points[i].featureId,
                                cached)) {
                continue;
            }
            constraint.points[i].normalImpulse = cached.normalImpulse;
            constraint.points[i].tangentImpulse[0] = cached.tangentImpulse[0];
            constraint.points[i].tangentImpulse[1] = cached.tangentImpulse[1];
        }
    }

    m_contacts.push_back(constraint);
}

void SequentialImpulseSolver::Solve(real dt) {
    if (dt <= real(0)) return;

    //--------------------------------------------------------------------------
    // 阶段 1：Prepare —— 算等效质量和偏置。这些量在迭代中不变，只算一次。
    //--------------------------------------------------------------------------
    for (ContactConstraint& c : m_contacts) {
        c.Prepare(m_config, dt);
    }
    for (IConstraint* c : m_constraints) {
        c->Prepare(m_config, dt);
    }

    //--------------------------------------------------------------------------
    // 阶段 2：WarmStart —— 施加继承来的冲量作为初始猜测。
    //
    // 必须在所有 Prepare 之后、所有迭代之前统一做。混在迭代里做的话，
    // 后加入的约束会看到被前面的 warm start 改过的速度，等效质量却是按
    // 改之前算的，两者不一致。
    //--------------------------------------------------------------------------
    if (m_config.warmStarting) {
        for (ContactConstraint& c : m_contacts) {
            c.WarmStart();
        }
        for (IConstraint* c : m_constraints) {
            c->WarmStart();
        }
    }

    //--------------------------------------------------------------------------
    // 阶段 3：迭代。
    //
    // Gauss-Seidel：每个约束解完立刻把速度写回，下一个约束用更新过的值。
    // 顺序固定为加入顺序 —— 结果依赖顺序，所以不能排序也不能并行
    // （见头文件里关于确定性的说明）。
    //--------------------------------------------------------------------------
    for (int iter = 0; iter < m_config.velocityIterations; ++iter) {
        for (ContactConstraint& c : m_contacts) {
            c.SolveVelocity();
        }
        for (IConstraint* c : m_constraints) {
            c->SolveVelocity();
        }
    }
}

void SequentialImpulseSolver::EndFrame() noexcept {
    for (const ContactConstraint& c : m_contacts) {
        m_cache.Store(c);
    }
    m_cache.EndFrame();
}

void SequentialImpulseSolver::Clear() noexcept {
    m_contacts.clear();
    m_constraints.clear();
    m_cache.Clear();
}

//==============================================================================
// 诊断
//==============================================================================

real SequentialImpulseSolver::MaxPenetration() const noexcept {
    real m = real(0);
    for (const ContactConstraint& c : m_contacts) {
        m = Max(m, c.MaxPenetration());
    }
    return m;
}

real SequentialImpulseSolver::TotalNormalImpulse() const noexcept {
    real total = real(0);
    for (const ContactConstraint& c : m_contacts) {
        for (std::uint8_t i = 0; i < c.pointCount; ++i) {
            total += c.points[i].normalImpulse;
        }
    }
    return total;
}

}  // namespace pe
