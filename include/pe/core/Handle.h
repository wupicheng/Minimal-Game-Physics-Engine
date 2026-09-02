#pragma once
//==============================================================================
// pe/core/Handle.h
//
// 带代际号（generation）的类型安全句柄。
//
// 为什么不用裸指针 / shared_ptr 指向刚体：
//   1. 刚体数据存在连续的 std::vector 里（为了缓存友好和将来 ECS 化）。
//      vector 扩容会让所有裸指针失效，这类 bug 极难复现。
//   2. 对象池会复用槽位。如果只用 index 做句柄，销毁 A 之后创建 B 复用了同一槽位，
//      那么残留的旧句柄会"神奇地"指向 B —— 这是最难查的一类逻辑错误。
//      代际号解决它：每次槽位被回收，generation 就 +1，旧句柄的 generation
//      对不上，立刻被识别为失效。
//   3. shared_ptr 每个对象一次堆分配 + 引用计数，对 50~100 个刚体逐帧遍历
//      是纯粹的浪费，而且指针追逐会打散缓存。
//
// 模板参数 Tag 只用于区分类型，从不实例化。这样 BodyHandle 和 ColliderHandle
// 是两个不能互相赋值的类型，编译期就能挡住"把碰撞体句柄传给要刚体句柄的函数"。
//==============================================================================

#include <cstdint>
#include <functional>

#include "pe/core/Types.h"

namespace pe {

template <class Tag>
struct Handle {
    std::uint32_t index;       ///< 在 SlotArray 中的槽位下标
    std::uint32_t generation;  ///< 该槽位被复用的次数；0 表示"从未指向任何对象"

    /// 默认构造得到一个空句柄。写成 = default 保持平凡（trivial），
    /// 这样 Handle 是 POD，可以安全地 memcpy、放进 union、塞进 ECS 组件。
    Handle() = default;

    constexpr Handle(std::uint32_t idx, std::uint32_t gen) noexcept
        : index(idx), generation(gen) {}

    /// 空句柄。注意 generation 为 0：SlotArray 分配出的槽位代际号从 1 起，
    /// 所以空句柄永远不会意外匹配上任何真实对象。
    static constexpr Handle Null() noexcept { return Handle(kInvalidIndex, 0u); }

    /// 只检查"是不是空句柄"。它**不**保证对象还活着 ——
    /// 判断对象是否存活必须问 SlotArray（它才知道当前槽位的代际号）。
    constexpr bool IsValid() const noexcept { return index != kInvalidIndex; }

    friend constexpr bool operator==(Handle a, Handle b) noexcept {
        return a.index == b.index && a.generation == b.generation;
    }
    friend constexpr bool operator!=(Handle a, Handle b) noexcept { return !(a == b); }

    /// 提供全序，方便把句柄放进 std::set / 做有序对去重（宽相位配对时要用）。
    friend constexpr bool operator<(Handle a, Handle b) noexcept {
        return a.index != b.index ? a.index < b.index : a.generation < b.generation;
    }
};

//------------------------------------------------------------------------------
// 引擎里的两种句柄
//
// Tag 类型只用来区分，从不定义、也从不实例化 —— `struct BodyTag` 这样一个
// 不完整类型就够了。好处是 BodyHandle 和 ColliderHandle 成为两个**不能互相赋值**
// 的类型，把"碰撞体句柄传给了要刚体句柄的函数"这类错误挡在编译期。
//------------------------------------------------------------------------------
using BodyHandle = Handle<struct BodyTag>;
using ColliderHandle = Handle<struct ColliderTag>;

/// 把两个碰撞体句柄打包成一个稳定的 64 位键。
///
/// 用途有两处，它们对这个键的要求是一样的：
///   - 求解器的接触缓存（warm starting 要靠它找到上一帧的同一对）
///   - 触发器的重叠集合差分
///
/// **必须先排序再打包**：碰撞对是无序的，(a,b) 和 (b,a) 是同一对，
/// 顺序不同却给出两个不同的键的话，warm starting 会在两帧之间反复失效，
/// 而触发器会同时报出 Enter 和 Exit。
inline std::uint64_t MakePairKey(ColliderHandle a, ColliderHandle b) noexcept {
    // 只用 index 参与打包：generation 已经由 SlotArray 保证同一时刻不会有两个
    // 相同 index 的存活对象，而把 generation 也塞进来反而会让"句柄失效重建"
    // 变成一个新键 —— 那正是我们不想要的（对象没变，键却变了）。
    const std::uint32_t lo = a.index < b.index ? a.index : b.index;
    const std::uint32_t hi = a.index < b.index ? b.index : a.index;
    return (static_cast<std::uint64_t>(hi) << 32) | static_cast<std::uint64_t>(lo);
}

}  // namespace pe

//------------------------------------------------------------------------------
// std::hash 特化：宽相位要把 (colliderA, colliderB) 放进 unordered_set 去重，
// 接触流形缓存也要按句柄对做键，所以句柄必须可哈希。
//------------------------------------------------------------------------------
namespace std {
template <class Tag>
struct hash<pe::Handle<Tag>> {
    std::size_t operator()(pe::Handle<Tag> h) const noexcept {
        // 把两个 32 位字段拼成一个 64 位再混合。index 放低位、generation 放高位，
        // 因为 index 的分布更密集，低位混合质量更好。
        const std::uint64_t key =
            (static_cast<std::uint64_t>(h.generation) << 32) | h.index;
        return std::hash<std::uint64_t>{}(key);
    }
};
}  // namespace std
