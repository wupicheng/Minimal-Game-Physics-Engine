#pragma once
//==============================================================================
// pe/core/SlotArray.h
//
// 句柄化对象池：连续存储 + 槽位复用 + 代际号防悬垂。
// PhysicsWorld 用它来持有 RigidBody、Collider 等所有对象的生命周期。
//
// 存储布局（当前版本，稀疏数组）：
//
//     slots:  [ A(gen=1,alive) ][ 空洞(gen=2,dead) ][ C(gen=1,alive) ] ...
//     free:   [ 1 ]                                   <- 可复用的槽位下标
//
// 优点：Add/Remove/Get 全是 O(1)，句柄永久稳定（对象不会因为别人被删而搬家）。
// 缺点：删除多了会留下空洞，遍历要跳过死槽位，缓存利用率下降。
//
// ---- 已知局限 / 升级点（见 UPGRADE_NOTES.md）----
// TODO(upgrade): 空洞率高时可升级为"稠密数组 + 间接表"：
//     sparse[handle.index] -> dense 下标，dense 数组永远紧凑无空洞。
//     代价是删除时要搬动最后一个元素并回填间接表。等到剖析显示遍历
//     成为热点（大量创建销毁的场景，比如满地弹壳）再做。
// TODO(upgrade): ECS 化时，把 T 拆成多个并行数组（SoA），SlotArray 只保留
//     索引管理逻辑，数据由各个组件数组自己持有。当前接口已经隔离了这一点：
//     外部只通过 Handle 访问，不假设 T 的存储方式。
//
// ---- 使用约束（重要）----
// Get() 返回的指针在下一次 Add()/Clear() 后可能失效（底层 vector 会扩容搬家）。
// 绝对不要跨帧、跨调用缓存这个指针。正确姿势是每次用 Handle 重新 Get。
//==============================================================================

#include <cstdint>
#include <utility>
#include <vector>

#include "pe/core/Handle.h"
#include "pe/core/Types.h"

namespace pe {

template <class T, class Tag>
class SlotArray {
public:
    using HandleType = Handle<Tag>;

    //--------------------------------------------------------------------------
    // 增删查
    //--------------------------------------------------------------------------

    /// 就地构造一个对象，返回它的句柄。
    template <class... Args>
    HandleType Emplace(Args&&... args) {
        std::uint32_t idx;
        if (!m_free.empty()) {
            // 复用一个空洞。注意 generation 保持槽位上次删除时递增后的值，
            // 不重置 —— 这正是旧句柄失效的依据。
            idx = m_free.back();
            m_free.pop_back();
            m_slots[idx].value = T(std::forward<Args>(args)...);
            m_slots[idx].alive = true;
        } else {
            idx = static_cast<std::uint32_t>(m_slots.size());
            m_slots.push_back(Slot{T(std::forward<Args>(args)...), /*generation*/ 1u,
                                   /*alive*/ true});
        }
        ++m_aliveCount;
        return HandleType(idx, m_slots[idx].generation);
    }

    /// 拷贝一个已有对象进池子。
    HandleType Add(const T& value) { return Emplace(value); }

    /// 销毁句柄指向的对象。句柄已失效时返回 false（重复删除是安全的）。
    bool Remove(HandleType h) {
        if (!IsValid(h)) {
            return false;
        }
        Slot& slot = m_slots[h.index];
        slot.alive = false;
        // 代际号 +1：所有还拿着旧句柄的人从这一刻起全部失效。
        // 溢出回绕（约 42 亿次复用后）在理论上会让某个远古句柄复活，
        // 实践中达不到，且回绕时跳过 0 以免和空句柄撞上。
        slot.generation = (slot.generation == 0xFFFFFFFFu) ? 1u : slot.generation + 1u;
        slot.value = T{};  // 让 T 的资源尽早释放；对 POD 来说等于清零
        m_free.push_back(h.index);
        --m_aliveCount;
        return true;
    }

    /// 句柄是否指向一个当前存活的对象。
    bool IsValid(HandleType h) const {
        return h.IsValid() && h.index < m_slots.size() && m_slots[h.index].alive &&
               m_slots[h.index].generation == h.generation;
    }

    /// 取对象指针；句柄失效返回 nullptr。
    /// 调用方必须检查返回值 —— 这是引擎里唯一允许"对象可能已经没了"的地方。
    T* Get(HandleType h) { return IsValid(h) ? &m_slots[h.index].value : nullptr; }

    const T* Get(HandleType h) const {
        return IsValid(h) ? &m_slots[h.index].value : nullptr;
    }

    //--------------------------------------------------------------------------
    // 容量与遍历
    //--------------------------------------------------------------------------

    /// 存活对象数量。
    std::size_t Size() const { return m_aliveCount; }
    bool Empty() const { return m_aliveCount == 0; }

    /// 槽位总数（含空洞）。遍历时的上界，也是空洞率的分母。
    std::size_t SlotCount() const { return m_slots.size(); }

    void Clear() {
        m_slots.clear();
        m_free.clear();
        m_aliveCount = 0;
    }

    void Reserve(std::size_t n) { m_slots.reserve(n); }

    /// 遍历所有存活对象，回调签名 fn(HandleType, T&)。
    /// 回调里**不要**调用 Emplace（会让引用失效）；需要延迟创建就先记下来，
    /// 遍历结束后再处理。Remove 是安全的（只改标记，不搬动内存）。
    template <class Fn>
    void ForEach(Fn&& fn) {
        for (std::uint32_t i = 0; i < static_cast<std::uint32_t>(m_slots.size()); ++i) {
            if (m_slots[i].alive) {
                fn(HandleType(i, m_slots[i].generation), m_slots[i].value);
            }
        }
    }

    template <class Fn>
    void ForEach(Fn&& fn) const {
        for (std::uint32_t i = 0; i < static_cast<std::uint32_t>(m_slots.size()); ++i) {
            if (m_slots[i].alive) {
                fn(HandleType(i, m_slots[i].generation), m_slots[i].value);
            }
        }
    }

    /// 按槽位下标直接访问，跳过句柄校验。仅供已经确认存活的内部热循环使用
    /// （例如求解器拿到的是宽相位刚验证过的索引），外部代码请用 Get()。
    T& AtUnchecked(std::uint32_t index) { return m_slots[index].value; }
    const T& AtUnchecked(std::uint32_t index) const { return m_slots[index].value; }
    bool IsAliveAt(std::uint32_t index) const {
        return index < m_slots.size() && m_slots[index].alive;
    }
    /// 由槽位下标反查出完整句柄（内部循环需要把索引还原成句柄时用）。
    HandleType HandleAt(std::uint32_t index) const {
        return HandleType(index, m_slots[index].generation);
    }

private:
    struct Slot {
        T value;
        std::uint32_t generation;  ///< 从 1 起（0 留给空句柄）
        bool alive;
    };

    std::vector<Slot> m_slots;
    std::vector<std::uint32_t> m_free;  ///< 空洞槽位的下标栈（LIFO，缓存局部性稍好）
    std::size_t m_aliveCount = 0;
};

}  // namespace pe
