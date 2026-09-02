#pragma once
//==============================================================================
// pe/scene/PhysicsWorld.h
//
// 物理世界 —— 整个引擎的门面。它是唯一允许 include 全部下层的模块（见 §2）。
//
//------------------------------------------------------------------------------
// 它做的事只有一件：编排
//------------------------------------------------------------------------------
// 前面八个里程碑做出来的都是**互不知情的零件**：宽相位不知道刚体、窄相位不知道
// 材质、求解器不做碰撞检测、角色控制器不认识 World。它们之间的所有连线都在这里。
//
// 这种"零件互不知情、由一个编排者串起来"的结构不是洁癖 —— 它的价值在于
// 每个零件都能脱离其余部分单独测试。前八个里程碑的测试全都没有用到 World，
// 这不是巧合。
//
//------------------------------------------------------------------------------
// 固定步长
//------------------------------------------------------------------------------
// `Step(dt)` 收到的是**渲染帧的**时间间隔，它是抖动的（16.6ms、19ms、14ms…）。
// 但物理必须用**固定**步长推进，否则同一段输入在不同帧率下会得到不同结果 ——
// 跳跃高度随帧率变化是这类 bug 最经典的表现。
//
// 所以内部用累加器：把 dt 攒起来，每攒够一个 fixedTimeStep 就跑一次完整的
// 物理步。剩下不足一步的部分留到下一帧，同时通过 `Alpha()` 暴露给渲染层做插值。
//
// `maxSubSteps` 是**死亡螺旋**的保险丝：如果某一帧特别慢（比如 200ms），
// 累加器会要求跑 12 步，而跑 12 步又会让这一帧更慢，下一帧要跑更多步……
// 直接卡死。上限一到就丢弃多余的时间 —— 表现为"卡顿时物理走得慢一点"，
// 这远好于彻底卡死。
//==============================================================================

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "pe/character/CharacterController.h"
#include "pe/collision/Collider.h"
#include "pe/collision/NarrowPhase.h"
#include "pe/collision/RayCast.h"
#include "pe/collision/ShapeCast.h"
#include "pe/collision/UniformGrid.h"
#include "pe/core/SlotArray.h"
#include "pe/dynamics/Integrator.h"
#include "pe/scene/Events.h"
#include "pe/scene/TriggerSystem.h"
#include "pe/solver/SequentialImpulseSolver.h"

namespace pe {

//------------------------------------------------------------------------------
// 世界配置
//------------------------------------------------------------------------------
struct WorldConfig {
    Vec3 gravity = kDefaultGravity;

    /// 物理步长。1/60 是行业默认；格斗游戏或者高精度需求可以用 1/120。
    real fixedTimeStep = real(1) / real(60);

    /// 单次 Step 最多跑几个子步。见文件头关于死亡螺旋的说明。
    int maxSubSteps = 4;

    /// 宽相位格子边长。建议取"典型动态物体尺寸的 2 倍"。
    real cellSize = real(2);

    /// 宽相位 fat AABB 的固定外扩量。
    real aabbMargin = kAabbMargin;

    /// fat AABB 的速度预测倍数：AABB 会额外沿速度方向扫掠 `速度 * dt * 这个系数`。
    ///
    /// 这一条直接来自 M3 的实测教训：只用固定 margin（0.05 米）的话，
    /// 一个 5 m/s 的物体每帧走 0.083 米，**每帧都跨出膨胀壳**，
    /// 实测 76% 的 Update 都触发了格子重建 —— fat AABB 等于没开。
    /// 按速度预测几帧之后，重建率会掉到个位数。
    real aabbVelocityPrediction = real(3);

    SolverConfig solver;
    SleepConfig sleep;

    bool enableSleeping = true;

    /// 连续碰撞检测（CCD）。开启后，一帧位移超过自身包围球半径的刚体会先做一次
    /// 形状扫掠，命中就把位置截断到接触点 —— 抛射物不会再穿墙。
    ///
    /// 只对**高速**物体做，是因为扫掠比积分贵得多，而 99% 的物体根本不需要。
    bool enableCcd = true;
};

//------------------------------------------------------------------------------
// 世界级的射线命中
//
// 比 `RaycastHit` 多了身份信息。collision 层按设计不知道句柄的存在
// （见 Ray.h），所以身份是在这一层补上的 —— 这正是当初把 RaycastHit
// 设计成纯几何结果的原因。
//------------------------------------------------------------------------------
struct WorldRaycastHit {
    real distance;
    Vec3 point;
    Vec3 normal;
    ColliderHandle collider;
    BodyHandle body;
};

/// 世界级的扫掠命中。
struct WorldShapeCastHit {
    real fraction;
    Vec3 point;
    Vec3 normal;
    bool startPenetrating;
    real depth;
    ColliderHandle collider;
    BodyHandle body;
};

//------------------------------------------------------------------------------
// PhysicsWorld
//------------------------------------------------------------------------------
class PhysicsWorld final : public ICharacterWorld {
public:
    explicit PhysicsWorld(const WorldConfig& config = WorldConfig{});

    //--------------------------------------------------------------------------
    // 刚体与碰撞体
    //--------------------------------------------------------------------------

    /// `desc.mass <= 0` 表示"由碰撞体的形状和密度自动算"。
    /// 注意此时必须先 AddCollider 再有质量 —— 没有碰撞体的动态刚体质量为 0，
    /// World 会把它降级成运动学体（否则 invMass 为无穷大，一碰就飞）。
    BodyHandle CreateBody(const BodyDesc& desc);

    /// 销毁刚体及其所有碰撞体。会为受影响的触发器补发 Exit 事件。
    void DestroyBody(BodyHandle handle);

    ColliderHandle AddCollider(BodyHandle body, const ColliderDesc& desc);
    void RemoveCollider(ColliderHandle handle);

    /// 句柄失效时返回 nullptr。**不要长期保存返回的指针** ——
    /// 创建新刚体可能让底层数组扩容，指针随之失效。每次要用就重新取。
    RigidBody* GetBody(BodyHandle handle);
    const RigidBody* GetBody(BodyHandle handle) const;

    const Collider* GetCollider(ColliderHandle handle) const;

    /// 刚体**原点**的位姿（用户设定的那个）。
    ///
    /// 和 `RigidBody::position` 的区别：后者是**质心**。一个刚体挂多个碰撞体时
    /// 质心会偏离原点，渲染要用的是原点。单个居中碰撞体时两者相同。
    Transform GetBodyTransform(BodyHandle handle) const;

    /// 直接设定刚体位姿（传送）。会同步刷新世界惯量与宽相位代理。
    void SetBodyTransform(BodyHandle handle, const Transform& transform);

    //--------------------------------------------------------------------------
    // 推进
    //--------------------------------------------------------------------------

    /// 推进 deltaTime（秒）。内部用固定步长累加器，可能跑 0 到 maxSubSteps 步。
    void Step(real deltaTime);

    /// 距离下一个物理步还差多少（0 到 1）。渲染层拿它做插值：
    ///     `world.GetInterpolatedTransform(handle, world.Alpha())`
    real Alpha() const noexcept {
        return (m_config.fixedTimeStep > real(0))
                   ? Clamp(m_accumulator / m_config.fixedTimeStep, real(0), real(1))
                   : real(0);
    }

    /// 在上一物理步与当前物理步之间插值出的位姿。**固定步长 + 可变帧率必须用它**，
    /// 否则画面会有肉眼可见的抖动（时间走样）。
    Transform GetInterpolatedTransform(BodyHandle handle, real alpha) const;

    //--------------------------------------------------------------------------
    // 查询
    //--------------------------------------------------------------------------

    /// 最近命中。这是 hitscan 子弹、AI 视线、脚下探测的入口。
    bool Raycast(const Ray& ray, WorldRaycastHit& out,
                 LayerMask layerMask = kLayerAll) const;

    /// 把形状沿位移扫过去，返回最先撞到的。抛射物的 CCD 和角色移动都走这里。
    bool ShapeCastWorld(const Shape& shape, const Transform& start,
                        const Vec3& displacement, WorldShapeCastHit& out,
                        LayerMask layerMask = kLayerAll,
                        BodyHandle ignoreBody = BodyHandle::Null()) const;

    /// 与给定盒子重叠的所有碰撞体。爆炸范围伤害用它。
    void OverlapAABB(const AABB& box, std::vector<ColliderHandle>& out,
                     LayerMask layerMask = kLayerAll) const;

    /// 与给定形状重叠的所有碰撞体（精确，不只是 AABB）。
    void OverlapShape(const Shape& shape, const Transform& transform,
                      std::vector<ColliderHandle>& out,
                      LayerMask layerMask = kLayerAll) const;

    //-- ICharacterWorld ---------------------------------------------------------
    bool SweepCharacter(const Shape& shape, const Transform& start,
                        const Vec3& displacement, LayerMask layerMask,
                        ShapeCastHit& out) const override;

    //--------------------------------------------------------------------------
    // 事件
    //--------------------------------------------------------------------------

    /// 注册监听器。所有回调都在 `Step()` 的**最末尾**触发，
    /// 那时创建/销毁物体是安全的（见 Events.h）。
    void SetEventListener(IPhysicsEventListener* listener) noexcept {
        m_listener = listener;
    }

    //--------------------------------------------------------------------------
    // 调试绘制
    //
    // 引擎不持有任何渲染资源（§6），所以它能提供的只有**纯数据**：
    // 每个碰撞体的形状、世界位姿、以及几个用来上色的状态位。
    // 渲染层拿去自己画线框，物理层完全不关心怎么画。
    //--------------------------------------------------------------------------
    struct DebugShape {
        Shape shape;
        Transform transform;
        ColliderHandle collider;
        BodyHandle body;
        bool isStatic;
        bool isSleeping;
        bool isTrigger;
    };

    /// 所有碰撞体的线框数据。out 会先被清空。
    void GetDebugDrawData(std::vector<DebugShape>& out) const;

    /// 上一个物理步的所有接触点（位置 + 法线 + 累积法向冲量）。
    /// 画出来是排查"接触点为什么不对"最快的手段。
    struct DebugContact {
        Vec3 position;
        Vec3 normal;
        real impulse;
    };
    void GetDebugContacts(std::vector<DebugContact>& out) const;

    //--------------------------------------------------------------------------
    // 诊断
    //--------------------------------------------------------------------------

    struct Stats {
        std::size_t bodyCount = 0;
        std::size_t colliderCount = 0;
        std::size_t broadPhasePairs = 0;   ///< 宽相位吐出的候选对
        std::size_t narrowPhaseContacts = 0;  ///< 窄相位确认的接触
        std::size_t activeBodies = 0;      ///< 没睡着的动态体
        std::size_t ccdSweeps = 0;         ///< 这一步做了几次 CCD 扫掠
        int subSteps = 0;                  ///< 上一次 Step 跑了几个子步
        real maxPenetration = real(0);
        /// 宽相位 Update 触发格子重建的比例。长期高于三成说明
        /// aabbMargin / aabbVelocityPrediction 调小了（见 M3 的实测教训）。
        real proxyRelinkRatio = real(0);
    };

    const Stats& GetStats() const noexcept { return m_stats; }
    const WorldConfig& Config() const noexcept { return m_config; }
    const UniformGrid& BroadPhase() const noexcept { return m_broadPhase; }

    std::size_t BodyCount() const noexcept { return m_bodies.Size(); }
    std::size_t ColliderCount() const noexcept { return m_colliders.Size(); }

    /// 遍历所有存活刚体。渲染层每帧靠它取位姿。
    template <class Fn>
    void ForEachBody(Fn&& fn) const {
        m_bodies.ForEach([&](BodyHandle handle, const BodyEntry& entry) {
            fn(handle, entry.body);
        });
    }

    template <class Fn>
    void ForEachCollider(Fn&& fn) const {
        m_colliders.ForEach(
            [&](ColliderHandle handle, const Collider& c) { fn(handle, c); });
    }

private:
    //--------------------------------------------------------------------------
    // 刚体条目：刚体状态 + World 才需要知道的额外信息
    //--------------------------------------------------------------------------
    struct BodyEntry {
        RigidBody body;

        /// 挂在这个刚体上的碰撞体。
        std::vector<ColliderHandle> colliders;

        /// 复合质心相对刚体**原点**的偏移（局部空间）。
        /// 单个居中碰撞体时是零向量。
        Vec3 centerOfMassLocal = Vec3::Zero();

        /// 上一个物理步结束时的位姿，给渲染插值用。
        Transform previousTransform = Transform(Vec3::Zero(), Quat::Identity());

        /// 用户在 BodyDesc 里给的质量（<= 0 表示自动算）。
        /// 加/删碰撞体之后要重算质量属性时需要它。
        real requestedMass = real(-1);
    };

    //-- Step 的各个阶段（顺序见 ARCHITECTURE.md §4.8）---------------------------
    void FixedStep(real dt);
    void SyncBroadPhase(real dt);
    void CollectContacts();
    void SolveStep(real dt);
    void IntegrateAndFinalize(real dt);
    /// 返回 true 表示 CCD 已经代劳了位置积分，调用方不要再积分一次。
    bool ApplyCcd(BodyEntry& entry, BodyHandle handle, real dt);

    void RecomputeMassProperties(BodyHandle handle);
    void UpdateColliderTransforms(BodyEntry& entry);
    /// fat AABB 的速度预测量（宽相位重建壳时会沿它扫掠一段）。
    Vec3 PredictedDisplacement(const RigidBody& body, real dt) const;

    /// 刚体原点的位姿（由质心位姿反推）。
    static Transform OriginTransform(const RigidBody& body, const Vec3& comLocal) noexcept {
        return Transform(body.position - body.rotation.Rotate(comLocal), body.rotation);
    }

    //-- 数据 -------------------------------------------------------------------
    WorldConfig m_config;

    SlotArray<BodyEntry, BodyTag> m_bodies;
    SlotArray<Collider, ColliderTag> m_colliders;

    UniformGrid m_broadPhase;
    SequentialImpulseSolver m_solver;
    TriggerSystem m_triggers;
    EventQueue m_events;
    IPhysicsEventListener* m_listener = nullptr;

    real m_accumulator = real(0);
    Stats m_stats;

    //-- 每帧复用的缓冲区（避免每步都重新分配）---------------------------------
    std::vector<BroadPhasePair> m_pairs;
    std::vector<Manifold> m_manifolds;
    std::vector<ContactInput> m_contactInputs;
    mutable std::vector<ProxyId> m_queryResult;

    /// 上一步在接触的对，用来产生 ContactBegin / ContactEnd 事件。
    std::unordered_map<std::uint64_t, std::uint32_t> m_touching;
    std::uint32_t m_stepIndex = 0;
};

}  // namespace pe
