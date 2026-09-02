#pragma once
//==============================================================================
// game/GameTypes.h
//
// 游戏侧的基础类型。**注意这是游戏，不是引擎** —— 这个目录里的东西全部依赖
// physengine，反过来引擎一行都不知道它们的存在。
//
//------------------------------------------------------------------------------
// 实体与刚体的关系
//------------------------------------------------------------------------------
// 引擎只认识"刚体"和"碰撞体"，它不知道什么是敌人、什么是弹药箱。游戏这一层
// 要自己维护一张 刚体句柄 -> 实体 的映射表。
//
// 这是刻意的分工，不是缺失的功能：如果引擎里塞一个 `void* userData`，
// 每个项目都会往里塞不同的东西，而引擎又必须假装不知道它是什么 ——
// 那还不如让游戏自己存一张表，类型安全、也不需要任何强制转换。
//
// 这张表最主要的用途是把**射线命中翻译成游戏语义**：
//     world.Raycast(...) -> BodyHandle -> Entity -> "这是个敌人，扣血"
//==============================================================================

#include <cstdint>

#include "pe/pe.h"

namespace game {

using namespace pe;

//------------------------------------------------------------------------------
// 碰撞层
//
// 引擎在 Types.h 里预留了 5 个常用层，这里补一个"敌人"层。
// 分层的意义在射线查询里最明显：
//   - 玩家开枪   -> 屏蔽 kPlayer（不然第一个命中永远是自己）
//   - 敌人开枪   -> 屏蔽 kEnemy（不然会打到自己和队友）
//   - 渲染射线   -> 屏蔽 kPlayer（第一人称看不见自己）+ kTrigger（拾取区是透明的）
//------------------------------------------------------------------------------
inline constexpr LayerMask kLayerEnemy = 1u << 5;

//------------------------------------------------------------------------------
// 碎块层
//
// 箱子被打碎之后飞出来的小块。它们是**真的刚体**（会飞、会滚、会叠在地上），
// 所以必须看得见；但它们不该改变任何玩法判定：
//
//   - 不挡路：一地木屑把玩家卡在原地，是最典型的"特效反过来毁掉手感"
//   - 不挡枪：飞过眼前的碎块吃掉子弹，玩家只会觉得"这枪明明打中了"
//   - 不挡视线：敌人的视线判定也不该被漫天碎屑随机改写
//
// 单独给一层就全解决了：**渲染掩码里有它，其余掩码里都没有**。
// 这正是分层存在的意义 —— 同一个物体在不同查询里可以有不同的可见性。
//------------------------------------------------------------------------------
inline constexpr LayerMask kLayerDebris = 1u << 6;

/// 玩家开枪能打中的东西：地图、箱子、敌人。不含自己、不含触发区、不含碎块。
inline constexpr LayerMask kPlayerShootMask =
    layers::kWorld | layers::kDefault | kLayerEnemy;

/// 敌人开枪能打中的东西：地图、箱子、玩家。不含其它敌人、不含碎块。
inline constexpr LayerMask kEnemyShootMask =
    layers::kWorld | layers::kDefault | layers::kPlayer;

/// 渲染射线看得见的东西。触发区是"看不见的区域"，必须排除，
/// 否则拾取区会像一堵实心墙一样挡住视线；碎块反过来必须在里面，
/// 不然打碎的箱子会凭空消失。
inline constexpr LayerMask kVisionMask =
    layers::kWorld | layers::kDefault | kLayerEnemy | kLayerDebris;

//------------------------------------------------------------------------------
// 实体
//------------------------------------------------------------------------------
enum class EntityKind : std::uint8_t {
    World,   ///< 静态地图几何
    Player,
    Enemy,
    Crate,   ///< 可推动、可击飞、**可打碎**的箱子
    Pickup,  ///< 触发区：补血 / 补弹药
    Debris,  ///< 箱子碎块 / 敌人碎块。有寿命，到点自己从世界里消失
};

enum class PickupKind : std::uint8_t { Health, Ammo };

/// 碎块是木头的还是血肉的 —— 只影响画出来的颜色。
enum class DebrisKind : std::uint8_t { Wood, Flesh };

//------------------------------------------------------------------------------
// 人形身体的两个关键尺寸。射手（玩家和机器人）要用它们，所以放在公共头文件里，
// 而不是藏在 Game::AddHumanoidColliders 里 —— 两边对不上的话，
// 机器人会一直瞄着一个身上没有的地方开枪。
//------------------------------------------------------------------------------

/// 躯干中心相对胶囊中心的高度。**瞄准点就是这里**：头只是个直径 27 厘米的球，
/// 瞄头会让任何有一点点误差的射手全部脱靶。
inline constexpr real kHumanoidChestOffsetY = real(0.34);

/// 躯干的半宽（米）。用来算"多大的角度偏差还打得中"。
inline constexpr real kHumanoidTorsoHalfWidth = real(0.185);

struct Entity {
    EntityKind kind = EntityKind::World;
    BodyHandle body = BodyHandle::Null();

    bool alive = true;

    //-- 敌人 -------------------------------------------------------------------
    real health = real(0);
    real fireCooldown = real(0);
    /// 已经发现玩家了吗。用来做"看见才开火"，以及让敌人在失去视线后
    /// 还追一小段（不然玩家一躲到柱子后面敌人就立刻发呆，很假）。
    bool alerted = false;
    real lastSeenTimer = real(0);
    /// 发现玩家之后的反应倒计时，归零才开第一枪
    real reactionTimer = real(0);
    Vec3 lastKnownPlayerPos = Vec3::Zero();
    /// 身体朝哪边（弧度，和玩家的 yaw 同一套约定：0 朝 +X）。
    ///
    /// 胶囊是轴对称的，转不转都一个样；换成人形之后**必须**有朝向 ——
    /// 一个永远侧着身子的人形比胶囊更假。它同时是给玩家的信息：
    /// 敌人转过身来了，说明它发现你了。
    real facingYaw = real(0);

    //-- 拾取物 -----------------------------------------------------------------
    PickupKind pickup = PickupKind::Health;
    real amount = real(0);
    real respawnTimer = real(0);

    //-- 箱子 / 碎块 ------------------------------------------------------------
    /// 立方体的半边长。箱子碎的时候要按它切成八块，所以必须记下来 ——
    /// 引擎里存的是 Shape，问它要尺寸不如游戏自己记一个数。
    real halfExtent = real(0);
    DebrisKind debrisKind = DebrisKind::Wood;
    /// 碎块的剩余寿命（秒）。归零就从世界里移除 —— 不回收的话，
    /// 打上十分钟地上会躺着几百个刚体，宽相位和求解器都会被拖垮。
    real life = real(0);
};

//------------------------------------------------------------------------------
// 手感参数
//
// 全是**玩法**参数，不是物理常数。调手感就调这里。
//------------------------------------------------------------------------------
struct GameConfig {
    //-- 玩家 -------------------------------------------------------------------
    real moveSpeed = real(5.5);
    real sprintMultiplier = real(1.6);
    real jumpSpeed = real(7.5);
    real turnSpeed = DegToRad(real(140));  ///< 每秒转多少度
    real playerMaxHealth = real(100);
    int magazineSize = 12;
    real reloadTime = real(1.4);
    real fireInterval = real(0.14);
    real weaponDamage = real(34);   ///< 三枪一个敌人
    real weaponRange = real(60);
    real weaponImpulse = real(14);  ///< 打在箱子上的冲量

    //-- 破坏与碎块 -------------------------------------------------------------
    /// 箱子的"结构完整度"。两枪（34x2 = 68）打不碎、三枪碎 —— 比打死一个敌人
    /// 稍微费一点，免得玩家顺手把掩体全拆了。
    real crateHealth = real(80);
    /// 碎块寿命。太短了看不清，太长了地上全是垃圾、还占着物理预算。
    real debrisLifetime = real(8);
    /// 世界上同时存在的碎块上限。超了就回收最老的一批 ——
    /// 特效永远不该让帧率掉下去。
    int debrisBudget = 64;
    /// 碎块炸开的初速度（m/s）
    real debrisScatterSpeed = real(3.2);
    /// 子弹方向额外推碎块的速度，让碎块朝着"子弹去的方向"飞
    real debrisBlastSpeed = real(4.5);
    real debrisSpin = real(9);  ///< 碎块自转（rad/s）

    //-- 特效时长（秒）----------------------------------------------------------
    /// 曳光弹。只有两三帧 —— 再长就成激光了。
    real tracerLifetime = real(0.06);
    real impactLifetime = real(0.25);   ///< 命中火花
    real hitMarkerLifetime = real(0.3); ///< 准星上的"打中了"提示

    //-- 敌人 -------------------------------------------------------------------
    real enemyHealth = real(100);
    real enemySpeed = real(3.2);
    real enemyFireInterval = real(1.1);
    real enemyDamage = real(9);
    real enemySightRange = real(35);
    real enemyPreferredRange = real(6);  ///< 走到这么近就停下开火
    real enemyMemory = real(3);          ///< 失去视线后还追多久
    /// 身体转向的角速度。不是瞬间转过去 —— 那样看着像贴纸在转，
    /// 而且玩家来不及看清"它正在转向我"这个信号。
    real enemyTurnSpeed = DegToRad(real(300));

    /// **反应时间**：第一次发现玩家之后要愣多久才开第一枪。
    /// 没有它，敌人一进视线就是瞬发命中，玩家从拐角探头会毫无预兆地掉血 ——
    /// 主观感受是"被作弊了"。这是 FPS 的基本公平性，不是难度调节。
    real enemyReactionTime = real(0.5);

    //-- 世界 -------------------------------------------------------------------
    /// FPS 惯例：重力比现实大得多，跳跃才不显得飘。CS 换算过来约 20 m/s^2。
    Vec3 gravity = Vec3(real(0), real(-20), real(0));
    real eyeHeight = real(1.62);  ///< 脚底往上多高是眼睛
};

}  // namespace game
