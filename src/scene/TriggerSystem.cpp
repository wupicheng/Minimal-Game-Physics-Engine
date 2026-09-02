//==============================================================================
// src/scene/TriggerSystem.cpp
//
// Enter / Stay / Exit 的差分。设计理由见 TriggerSystem.h。
//==============================================================================

#include "pe/scene/TriggerSystem.h"

namespace pe {

namespace {

//------------------------------------------------------------------------------
// 单帧派发上限的保险丝。
//
// 正常情况下永远碰不到：回调里销毁物体最多补发有限条 Exit，而被销毁的物体
// 不会再被销毁第二次。它防的是"回调里不断创建并立刻销毁物体"这类失控写法 ——
// 宁可这一帧少派几条事件，也好过整个游戏卡死在派发循环里。
//------------------------------------------------------------------------------
constexpr std::size_t kMaxDispatchPerFrame = 1u << 16;

}  // namespace

//==============================================================================
// 事件派发
//==============================================================================

void DispatchEvents(const EventQueue& queue, IPhysicsEventListener* listener) {
    if (listener == nullptr) return;

    //--------------------------------------------------------------------------
    // **必须用下标遍历，而且必须按值取出事件。**
    //
    // Events.h 承诺"回调里增删物体是安全的"，而销毁一个碰撞体会走
    // `TriggerSystem::RemoveCollider()` —— 它会往**同一个队列**里补发 Exit
    // 事件。也就是说：正在被遍历的 vector 会在遍历过程中 push_back。
    //
    // 一旦扩容，range-for 缓存下来的 begin/end 就全部变成野指针，
    // 引用 `e` 也指向已经释放的内存 —— 后果是随机的越界读和崩溃。
    // 这不是理论风险：游戏里"走过弹药箱、回调里销毁它"的那一帧就会闪退。
    //
    // 下标每次重新取容器，扩容不影响；顺带把回调中新产生的事件也一并派发掉，
    // 于是"物品被捡走"和它配套的 Exit 在同一帧就送到了游戏手里。
    //--------------------------------------------------------------------------
    for (std::size_t i = 0; i < queue.TriggerEvents().size(); ++i) {
        if (i >= kMaxDispatchPerFrame) break;  // 保险丝，见下
        const TriggerEvent e = queue.TriggerEvents()[i];  // 拷贝：回调会让引用失效
        switch (e.type) {
            case TriggerEventType::Enter:
                listener->OnTriggerEnter(e.trigger, e.other);
                break;
            case TriggerEventType::Stay:
                listener->OnTriggerStay(e.trigger, e.other);
                break;
            case TriggerEventType::Exit:
                listener->OnTriggerExit(e.trigger, e.other);
                break;
        }
    }

    for (std::size_t i = 0; i < queue.ContactEvents().size(); ++i) {
        if (i >= kMaxDispatchPerFrame) break;
        const ContactEvent e = queue.ContactEvents()[i];
        switch (e.type) {
            case ContactEventType::Begin: listener->OnContactBegin(e); break;
            case ContactEventType::End: listener->OnContactEnd(e); break;
        }
    }
}

//==============================================================================
// 触发器
//==============================================================================

void TriggerSystem::AddOverlap(ColliderHandle trigger, ColliderHandle other) {
    // 无序键：(a,b) 与 (b,a) 必须落到同一条记录上，否则同一次重叠会
    // 同时报出 Enter 和 Exit，每帧闪烁。排序在 MakePairKey 里做掉了。
    const std::uint64_t key = MakePairKey(trigger, other);

    const auto it = m_overlaps.find(key);
    if (it != m_overlaps.end()) {
        // 已经在重叠集合里，只更新"本帧还在"的标记。
        // **不覆盖 trigger/other**：谁是触发器由第一次报告时决定，
        // 中途换来换去会让游戏侧收到的事件参数在两帧之间跳变。
        it->second.lastFrame = m_frame;
        return;
    }

    m_overlaps.emplace(key, Record{trigger, other, m_frame, /*entered*/ false});
}

void TriggerSystem::EndFrame(EventQueue& queue) {
    for (auto it = m_overlaps.begin(); it != m_overlaps.end();) {
        Record& record = it->second;

        if (record.lastFrame != m_frame) {
            //------------------------------------------------------------------
            // 本帧没被报告 -> 离开了。
            //
            // 只有**已经报过 Enter** 的才需要报 Exit。理论上不会出现
            // "进来了但没报过 Enter 就走了"（Enter 在同一次 EndFrame 里就发了），
            // 但这个判断让状态机对"中途 Clear"之类的操作也保持自洽。
            //------------------------------------------------------------------
            if (record.entered) {
                queue.Push(TriggerEvent{TriggerEventType::Exit, record.trigger,
                                        record.other});
            }
            it = m_overlaps.erase(it);
            continue;
        }

        if (!record.entered) {
            queue.Push(
                TriggerEvent{TriggerEventType::Enter, record.trigger, record.other});
            record.entered = true;
        } else {
            queue.Push(
                TriggerEvent{TriggerEventType::Stay, record.trigger, record.other});
        }

        ++it;
    }
}

void TriggerSystem::RemoveCollider(ColliderHandle collider, EventQueue& queue) {
    //--------------------------------------------------------------------------
    // 为它参与的每一对补发 Exit。
    //
    // 少了这一步，"玩家死在买枪区里"或者"道具被捡走"之后，游戏侧维护的
    // "区域内对象列表"会永远留着一个幽灵条目 —— 而且这种泄漏只在特定
    // 死亡/销毁时机下才出现，属于最难复现的那一类 bug。
    //--------------------------------------------------------------------------
    for (auto it = m_overlaps.begin(); it != m_overlaps.end();) {
        const Record& record = it->second;
        if (record.trigger != collider && record.other != collider) {
            ++it;
            continue;
        }

        if (record.entered) {
            queue.Push(
                TriggerEvent{TriggerEventType::Exit, record.trigger, record.other});
        }
        it = m_overlaps.erase(it);
    }
}

bool TriggerSystem::IsOverlapping(ColliderHandle a, ColliderHandle b) const noexcept {
    return m_overlaps.find(MakePairKey(a, b)) != m_overlaps.end();
}

}  // namespace pe
