#pragma once
//==============================================================================
// pe/scene/Events.h
//
// 物理事件与事件队列。
//
//------------------------------------------------------------------------------
// 为什么是队列而不是回调
//------------------------------------------------------------------------------
// 直觉写法是"检测到碰撞就直接调用户的回调"。**这会炸。**
//
// 用户的回调里最常做的事就是"捡到弹药 -> 销毁这个物品"或者"踩到地雷 -> 生成爆炸"。
// 而这些操作会创建/销毁刚体，也就是会改动引擎正在遍历的那些容器 ——
// 迭代器失效，轻则漏掉后面的碰撞，重则崩溃。而且崩溃点会在引擎内部，
// 和真正的原因（用户回调）隔了好几层调用栈，极难定位。
//
// 所以事件一律**先入队**，在 `Step()` 的最末尾、所有遍历都结束之后统一派发。
// 那时用户想怎么增删物体都是安全的。
//
// **派发循环本身也必须扛得住这件事。** 回调里销毁一个碰撞体会让
// `TriggerSystem::RemoveCollider()` 往**正在派发的这个队列**里补发 Exit ——
// 也就是说，被遍历的 vector 会在遍历过程中 push_back。所以
// `DispatchEvents()` 用下标遍历、按值取出事件，绝不能用 range-for
// （见 TriggerSystem.cpp；回归测试在 tests/test_world.cpp）。
//
// 代价是事件比它"发生"的时刻晚一点点被看到，但那个延迟在同一帧之内，
// 游戏逻辑完全感知不到。
//==============================================================================

#include <cstdint>
#include <vector>

#include "pe/core/Handle.h"
#include "pe/core/Types.h"
#include "pe/math/Vec3.h"

namespace pe {

//------------------------------------------------------------------------------
// 触发器事件
//------------------------------------------------------------------------------
enum class TriggerEventType : std::uint8_t {
    /// 这一帧刚进来。
    Enter,
    /// 上一帧在里面，这一帧还在。
    ///
    /// 之所以要有 Stay 而不是让游戏自己记状态：伤害区、治疗区这类持续生效的
    /// 逻辑需要"每帧都被通知一次"，而让每个游戏系统各自维护一份重叠集合，
    /// 既重复又容易和引擎的判定不一致。
    Stay,
    /// 上一帧在里面，这一帧出去了。**碰撞体被销毁时也会补发 Exit** ——
    /// 少了这一条，"玩家死在买枪区里"会让买枪区永远以为他还在。
    Exit,
};

struct TriggerEvent {
    TriggerEventType type;
    ColliderHandle trigger;  ///< 带 isTrigger 标志的那一个
    ColliderHandle other;    ///< 进来的那个
};

//------------------------------------------------------------------------------
// 碰撞事件
//------------------------------------------------------------------------------
enum class ContactEventType : std::uint8_t {
    /// 这一对这一帧刚开始接触。用来播放撞击音效、生成火花。
    Begin,
    /// 这一对这一帧分开了。
    End,
};

struct ContactEvent {
    ContactEventType type;
    ColliderHandle a;
    ColliderHandle b;

    /// 接触点与法线（Begin 时有效，End 时为零）。
    /// 音效音量、粒子朝向都要用它。
    Vec3 point;
    Vec3 normal;

    /// 这一次接触的法向冲量总和（Begin 时有效）。
    ///
    /// 它是"撞得有多狠"的唯一靠谱度量 —— 比相对速度好用，因为它已经把质量算进去了。
    /// 撞击音量、破坏判定、摔落伤害都该按它来。
    real impulse;
};

//------------------------------------------------------------------------------
// 事件队列
//------------------------------------------------------------------------------
class EventQueue {
public:
    void Clear() noexcept {
        m_triggerEvents.clear();
        m_contactEvents.clear();
    }

    void Push(const TriggerEvent& e) { m_triggerEvents.push_back(e); }
    void Push(const ContactEvent& e) { m_contactEvents.push_back(e); }

    const std::vector<TriggerEvent>& TriggerEvents() const noexcept {
        return m_triggerEvents;
    }
    const std::vector<ContactEvent>& ContactEvents() const noexcept {
        return m_contactEvents;
    }

    bool Empty() const noexcept {
        return m_triggerEvents.empty() && m_contactEvents.empty();
    }

private:
    std::vector<TriggerEvent> m_triggerEvents;
    std::vector<ContactEvent> m_contactEvents;
};

//------------------------------------------------------------------------------
// 监听器
//
// 游戏侧实现它，`PhysicsWorld::SetEventListener` 注册。
// 所有回调都在 `Step()` 的最末尾被调用，此时创建/销毁物体是安全的。
//------------------------------------------------------------------------------
class IPhysicsEventListener {
public:
    virtual ~IPhysicsEventListener() = default;

    virtual void OnTriggerEnter(ColliderHandle /*trigger*/, ColliderHandle /*other*/) {}
    virtual void OnTriggerStay(ColliderHandle /*trigger*/, ColliderHandle /*other*/) {}
    virtual void OnTriggerExit(ColliderHandle /*trigger*/, ColliderHandle /*other*/) {}

    virtual void OnContactBegin(const ContactEvent& /*event*/) {}
    virtual void OnContactEnd(const ContactEvent& /*event*/) {}
};

/// 把队列里的事件派发给监听器。listener 为空时是空操作。
void DispatchEvents(const EventQueue& queue, IPhysicsEventListener* listener);

}  // namespace pe
