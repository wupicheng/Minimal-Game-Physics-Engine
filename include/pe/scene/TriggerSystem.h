#pragma once
//==============================================================================
// pe/scene/TriggerSystem.h
//
// 触发器的 Enter / Stay / Exit 状态机。
//
//------------------------------------------------------------------------------
// 它到底在解决什么问题
//------------------------------------------------------------------------------
// 碰撞检测每帧给出的是一个**快照**："现在这些对重叠着"。但游戏逻辑要的是
// **变化**："谁刚进来、谁刚出去"。捡道具要在进入的那一帧触发一次，
// 而不是每帧都触发；买枪区要在离开时关掉界面。
//
// 从快照得到变化，就是集合差分：
//
//     本帧有、上帧没有  -> Enter
//     两帧都有          -> Stay
//     上帧有、本帧没有  -> Exit
//
// 听起来平凡，但有三个地方容易写错，而且错了都很难查：
//
//   1. **对键必须无序**。(a,b) 和 (b,a) 是同一对。顺序不同却算成两个键的话，
//      同一次重叠会同时报出 Enter 和 Exit，每帧闪烁。
//      `MakePairKey()` 已经在 Handle.h 里把排序做掉了。
//
//   2. **销毁碰撞体必须补发 Exit**。玩家死在买枪区里、道具被捡走的同一帧，
//      物体没了，差分自然不会再报 Exit —— 于是游戏侧的"区域内玩家列表"
//      会永远留着一个幽灵。`RemoveCollider()` 就是干这个的。
//
//   3. **事件不能在差分过程中直接回调**。理由见 Events.h：回调里创建/销毁物体
//      会让正在遍历的容器失效。所以这里只往 EventQueue 里塞。
//
//------------------------------------------------------------------------------
// 用法（PhysicsWorld 的 Step 内部）
//------------------------------------------------------------------------------
//     triggers.BeginFrame();
//     for (每一对重叠的触发器)  triggers.AddOverlap(trigger, other);
//     triggers.EndFrame(eventQueue);      // 差分 -> 事件入队
//==============================================================================

#include <cstdint>
#include <unordered_map>

#include "pe/core/Handle.h"
#include "pe/scene/Events.h"

namespace pe {

class TriggerSystem {
public:
    /// 开始收集本帧的重叠。
    void BeginFrame() noexcept { ++m_frame; }

    /// 报告一对重叠。同一对重复报告是安全的（幂等）。
    ///
    /// trigger 是带 isTrigger 标志的那一个；两个都是触发器时，
    /// 由调用方决定谁当 trigger —— 事件会按调用方给的顺序报出去。
    void AddOverlap(ColliderHandle trigger, ColliderHandle other);

    /// 与上一帧做差分，把 Enter / Stay / Exit 压进事件队列。
    void EndFrame(EventQueue& queue);

    /// 碰撞体被销毁时调用：为它参与的所有重叠补发 Exit。
    ///
    /// 必须在物体真正销毁**之前**调，句柄那时还有意义。
    void RemoveCollider(ColliderHandle collider, EventQueue& queue);

    void Clear() noexcept {
        m_overlaps.clear();
        m_frame = 0;
    }

    /// 当前正在重叠的对数。
    std::size_t OverlapCount() const noexcept { return m_overlaps.size(); }

    /// 这一对现在重叠着吗。游戏侧偶尔需要直接问一句而不是等事件。
    bool IsOverlapping(ColliderHandle a, ColliderHandle b) const noexcept;

private:
    struct Record {
        ColliderHandle trigger;
        ColliderHandle other;
        /// 最后一次被 AddOverlap 报告是哪一帧。
        /// 它等于当前帧号就说明"本帧还在"，否则就是刚离开。
        std::uint32_t lastFrame;
        /// 已经报过 Enter 了吗。用来区分本帧新增（Enter）与持续（Stay）。
        bool entered;
    };

    std::unordered_map<std::uint64_t, Record> m_overlaps;
    std::uint32_t m_frame = 0;
};

}  // namespace pe
