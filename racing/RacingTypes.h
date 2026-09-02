#pragma once
//==============================================================================
// racing/RacingTypes.h
//
// 赛车游戏的基础类型。**这是游戏，不是引擎** —— physengine 一行都不知道
// 这个目录的存在，和 `game/` 里那个 FPS 的关系也仅仅是"用了同一个引擎"。
//
//------------------------------------------------------------------------------
// 引擎里没有"车"这个东西
//------------------------------------------------------------------------------
// 这个引擎没有车辆约束、没有轮子关节、没有 `btRaycastVehicle`。它只有：
// 刚体、碰撞体、求解器、射线查询、层掩码。
//
// 而这几样**恰好就够**：整台车是一个刚体，四个轮子是四条向下的射线，
// 悬挂是弹簧阻尼力，抓地力是接触点上的冲量。这正是商业引擎里
// "raycast vehicle" 的标准做法 —— 不是简化版，是行业默认版（见 Vehicle.h）。
//==============================================================================

#include <cstdint>

#include "pe/pe.h"

namespace racing {

using namespace pe;

//------------------------------------------------------------------------------
// 碰撞层
//
// 分层在这个游戏里比在 FPS 里还关键，因为**悬挂射线绝不能打到自己**：
// 射线从车身内部往下打，第一个命中如果是自己的底盘，车就会浮在半空。
//------------------------------------------------------------------------------
inline constexpr LayerMask kLayerTrack = layers::kWorld;    ///< 路面、墙、跳台
inline constexpr LayerMask kLayerCar = 1u << 5;             ///< 车身（真的会撞）
inline constexpr LayerMask kLayerProp = layers::kDefault;   ///< 路障箱子
inline constexpr LayerMask kLayerDecor = 1u << 6;           ///< 纯装饰（轮子、门柱）

/// 悬挂射线只看路面和路障。**不含车身、不含轮子** —— 见上面。
inline constexpr LayerMask kSuspensionMask = kLayerTrack | kLayerProp;

/// 渲染射线看得见的一切。触发区（检查点）不在里面，它们是透明的。
//
// **不含 kLayerCar**：画面上的车是那层装饰车壳（kLayerDecor），
// 碰撞用的那个光板盒子不该被画出来 —— 否则一个方盒子会从漂亮的车壳里
// 支棱出来。碰撞代理比渲染模型简单，这是所有引擎的常态。
inline constexpr LayerMask kVisionMask = kLayerTrack | kLayerProp | kLayerDecor;

//------------------------------------------------------------------------------
// 实体
//
// 和 FPS 一样：引擎只认识刚体，游戏自己维护 刚体 -> 实体 的映射，
// 把"射线打中了 BodyHandle{7,1}"翻译成"那是路面"。
//------------------------------------------------------------------------------
enum class EntityKind : std::uint8_t {
    Road,       ///< 路面
    Wall,       ///< 护墙
    Ramp,       ///< 跳台
    Car,        ///< 车身
    Wheel,      ///< 轮子（纯装饰刚体）
    Prop,       ///< 可撞飞的路障
    Gate,       ///< 检查点门柱（纯装饰，告诉玩家该往哪走）
    Checkpoint, ///< 检查点触发区（看不见）
};

struct Entity {
    EntityKind kind = EntityKind::Road;
    BodyHandle body = BodyHandle::Null();
    /// 检查点用：这是第几个。0 号就是终点线。
    int checkpointIndex = -1;
};

}  // namespace racing
