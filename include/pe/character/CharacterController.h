#pragma once
//==============================================================================
// pe/character/CharacterController.h
//
// 角色控制器：胶囊体的 collide-and-slide 移动。**不走刚体模拟。**
//
//------------------------------------------------------------------------------
// 为什么角色不能是刚体
//------------------------------------------------------------------------------
// 这是 FPS 里最反直觉、也最不能妥协的一条设计。把角色做成动态刚体，会得到：
//   - **贴墙抖动**：求解器每帧把角色推出墙一点点，重力又把它压回去
//   - **爬不上台阶**：一个 20 厘米的台阶，刚体只会被它挡住，除非跳
//   - **走斜坡打滑**：摩擦不够就往下溜，摩擦够了又转不了向
//   - **被撞飞**：一个箱子砸过来，玩家的视角会被物理接管，晕眩且失控
//   - **手感无法调**：想让"起步快一点"，只能去改摩擦和质量，牵一发动全身
//
// 所以业界标准做法是：角色是一个**被脚本驱动的运动学体**，
// 它自己做扫掠、自己决定怎么滑，完全不参与冲量求解。
// 代价是角色的运动不"物理正确"，但玩家要的从来不是物理正确，是**可预测**。
//
// 单向作用力：角色可以推动动态刚体（对刚体施加冲量），
// 但动态刚体**推不动角色**。CS 里箱子挡不住冲锋的玩家，正是这条约定。
//
//------------------------------------------------------------------------------
// collide-and-slide
//------------------------------------------------------------------------------
// 核心循环只有几行：
//
//     剩余位移 = 期望位移
//     重复至多 N 次：
//         扫掠(剩余位移) -> 撞到了吗？
//         没撞到 -> 走完，结束
//         撞到了 -> 走到接触点，把剩余位移投影到墙面切平面：
//                     剩余 = 剩余 - Dot(剩余, n) * n
//
// 那一行投影就是 FPS 贴墙滑行手感的全部来源：撞墙之后不是停下，而是沿着墙面
// 把剩下的力气用掉。迭代是因为滑动之后可能撞上第二面墙（墙角）。
//
// 迭代上限取 4：三面墙夹成的角落最多需要 3 次，第 4 次是余量。
// 再多没有意义 —— 走到那一步说明角色被夹死了，该停就停。
//==============================================================================

#include "pe/collision/ShapeCast.h"
#include "pe/collision/Shape.h"
#include "pe/core/Types.h"
#include "pe/math/MathUtil.h"
#include "pe/math/Transform.h"
#include "pe/math/Vec3.h"

namespace pe {

//------------------------------------------------------------------------------
// 角色对世界的查询接口
//
// 角色控制器按架构规定**不依赖 scene 层**（它和 solver 是平级的），
// 所以它不能直接持有 PhysicsWorld。需要什么就在这里声明什么，
// 由 M9 的 PhysicsWorld 来实现。
//
// 好处是角色控制器可以脱离整个引擎单独测试 —— 测试里给它一个只有几个盒子的
// 假世界就行，不需要把刚体、求解器、宽相位全都拉起来。
//------------------------------------------------------------------------------
class ICharacterWorld {
public:
    virtual ~ICharacterWorld() = default;

    /// 把形状从 start 沿 displacement 扫过去，返回**最先**撞到的那个。
    /// 返回 false 表示整段位移畅通。
    virtual bool SweepCharacter(const Shape& shape, const Transform& start,
                                const Vec3& displacement, LayerMask layerMask,
                                ShapeCastHit& out) const = 0;
};

//------------------------------------------------------------------------------
// 配置
//
// 这些全是**手感参数**，不是物理常数。调角色移动就调这里。
//------------------------------------------------------------------------------
struct CharacterConfig {
    real radius = real(0.4);       ///< 胶囊半径
    real height = real(1.8);       ///< 总身高（含两个半球端盖）

    /// 能自动跨上去的台阶高度（米）。
    ///
    /// 这是角色控制器存在的最主要理由之一。没有它，地图上每一道 10 厘米的门槛
    /// 都得跳过去。CS 系列大约是 18 单位 ≈ 0.35 米。
    real stepOffset = real(0.35);

    /// 可站立的最大坡度。超过它就不算地面，角色会往下滑。
    /// 50 度是常见取值：楼梯的等效坡度约 30-40 度，要能上；陡坡要滑下来。
    real maxSlopeAngle = DegToRad(real(50));

    /// 贴墙时保留的间隙（米）。
    ///
    /// 走到"恰好接触"是危险的：浮点误差会让下一帧的重叠判定在"碰"和"不碰"
    /// 之间抖动，表现为贴墙时角色微微颤抖。永远留一条比误差大一个量级的缝。
    real skinWidth = real(0.02);

    /// collide-and-slide 的迭代上限。理由见文件头。
    int maxSlideIterations = 4;

    /// 地面探测的向下扫掠距离（米）。要略大于 skinWidth，
    /// 否则角色离地一丁点就会被判定为悬空，下楼梯时会不停地"落地-离地"闪烁。
    real groundProbeDistance = real(0.1);

    /// 世界的"上"方向。做重力翻转之类的玩法时改它。
    Vec3 up = Vec3(real(0), real(1), real(0));

    /// 角色能撞到哪些层。
    LayerMask layerMask = kLayerAll;

    /// 由 radius / height 构造胶囊形状。
    /// 注意 Shape 的 halfHeight 不含端盖，换算见 Shape.h。
    Shape MakeShape() const noexcept {
        return Shape::MakeCapsuleFromHeight(radius, height);
    }
};

//------------------------------------------------------------------------------
// 一次移动的结果
//------------------------------------------------------------------------------
struct CharacterMoveResult {
    /// 实际走了多远（可能因为撞墙而短于期望位移，也可能因为滑行而方向不同）。
    Vec3 displacement;

    /// 这一路撞到过东西吗。
    bool collided;

    /// 撞到的是**侧面**（不可站立的陡面）吗。用来播放"贴墙"的动画/音效。
    bool collidedSides;

    /// 撞到的是可站立的地面吗。
    bool collidedBelow;

    /// 头撞到天花板了吗。跳跃时用它来提前终止上升。
    bool collidedAbove;

    /// 因为台阶而被自动抬升了吗。
    bool steppedUp;
};

//------------------------------------------------------------------------------
// 角色控制器
//
// 状态是公开的裸字段而不是 getter/setter：它本来就是一个状态块，
// 游戏逻辑需要随时读写（传送、播放动画时接管位置、观战模式切换）。
// 藏起来只会逼着写一堆没有意义的转发函数。
//------------------------------------------------------------------------------
class CharacterController {
public:
    //-- 状态 -------------------------------------------------------------------
    Vec3 position = Vec3::Zero();  ///< 胶囊**中心**（不是脚底）
    Vec3 velocity = Vec3::Zero();

    /// 站在可站立的地面上。注意它**不等于**"下面有东西"——
    /// 站在 70 度的陡坡上时 isGrounded 是 false，角色会往下滑。
    bool isGrounded = false;

    /// 脚下地面的法线。isGrounded 为 false 时无意义。
    /// 用途：让移动方向贴着斜坡走（否则上坡时会一顿一顿地起跳）。
    Vec3 groundNormal = Vec3(real(0), real(1), real(0));

    CharacterConfig config;

    CharacterController() = default;
    explicit CharacterController(const CharacterConfig& cfg) : config(cfg) {}

    //-- 核心 -------------------------------------------------------------------

    /// 把角色沿 displacement 移动，撞墙就滑，遇到台阶就抬。
    ///
    /// 这是最底层的原语，**不施加重力、不改 velocity**。
    /// 游戏可以直接用它做"瞬移一小段"之类的事。
    CharacterMoveResult Move(const ICharacterWorld& world, const Vec3& displacement);

    /// 一帧的完整更新：重力 -> 移动 -> 地面检测。
    ///
    /// desiredVelocity 是**水平方向**的期望速度（由输入决定），
    /// 竖直分量由控制器自己管（重力、跳跃）。这个拆分是刻意的：
    /// 水平移动是"想去哪儿"，竖直移动是"物理"，两者手感来源完全不同。
    CharacterMoveResult Update(const ICharacterWorld& world,
                               const Vec3& desiredHorizontalVelocity, real dt,
                               const Vec3& gravity);

    /// 起跳：直接设定竖直速度。站在地面上才有效。
    /// 返回是否真的跳了。
    bool Jump(real jumpSpeed);

    //-- 查询 -------------------------------------------------------------------

    Shape GetShape() const noexcept { return config.MakeShape(); }
    Transform GetTransform() const noexcept {
        return Transform(position, Quat::Identity());
    }

    /// 脚底的世界坐标。摆放角色时用的是脚底而不是中心，所以提供换算。
    Vec3 FootPosition() const noexcept {
        return position - config.up * (config.height * real(0.5));
    }
    void SetFootPosition(const Vec3& foot) noexcept {
        position = foot + config.up * (config.height * real(0.5));
    }

    /// 某个法线的表面算不算可站立的地面。
    bool IsWalkable(const Vec3& normal) const noexcept {
        return Dot(normal, config.up) >= Cos(config.maxSlopeAngle);
    }

    /// 向下探测地面，刷新 isGrounded / groundNormal。
    /// Update() 内部会调，单独暴露是为了让游戏在传送之后立刻重新判定。
    void UpdateGrounded(const ICharacterWorld& world);

private:
    /// 不处理台阶的纯 collide-and-slide。Move() 在它之上加台阶逻辑。
    /// 沿竖直方向把角色吸附回地面（不参与滑动投影）。见 .cpp 里的说明。
    void SnapToGround(const ICharacterWorld& world);

    CharacterMoveResult SlideMove(const ICharacterWorld& world, const Vec3& displacement,
                                  const Vec3& startPosition, Vec3& outPosition) const;
};

}  // namespace pe
