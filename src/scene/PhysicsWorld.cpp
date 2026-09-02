//==============================================================================
// src/scene/PhysicsWorld.cpp
//
// 编排。每一步做什么、为什么是这个顺序，见 ARCHITECTURE.md §4.8 和头文件。
//==============================================================================

#include "pe/scene/PhysicsWorld.h"

namespace pe {

namespace {

//------------------------------------------------------------------------------
// 句柄 <-> 宽相位的 userData
//
// 宽相位按设计不知道句柄的存在（见 BroadPhase.h），它只透传一个 64 位整数。
// 碰撞体句柄正好是两个 32 位字段，塞得进去。
//------------------------------------------------------------------------------
inline std::uint64_t PackCollider(ColliderHandle h) noexcept {
    return (static_cast<std::uint64_t>(h.generation) << 32) |
           static_cast<std::uint64_t>(h.index);
}

inline ColliderHandle UnpackCollider(std::uint64_t v) noexcept {
    return ColliderHandle(static_cast<std::uint32_t>(v & 0xFFFFFFFFu),
                          static_cast<std::uint32_t>(v >> 32));
}

}  // namespace

//==============================================================================
// 构造
//==============================================================================

PhysicsWorld::PhysicsWorld(const WorldConfig& config)
    : m_config(config),
      m_broadPhase(config.cellSize, config.aabbMargin),
      m_solver(config.solver) {}

//==============================================================================
// 刚体
//==============================================================================

BodyHandle PhysicsWorld::CreateBody(const BodyDesc& desc) {
    BodyEntry entry;

    // 先按"没有碰撞体"建：质量属性全零。加了碰撞体之后会重算。
    MassProperties empty;
    empty.mass = real(0);
    empty.centerOfMass = Vec3::Zero();
    empty.inertia = Mat3::Diagonal(Vec3::Zero());

    entry.body = MakeRigidBody(desc, empty);
    entry.requestedMass = desc.mass;
    entry.centerOfMassLocal = Vec3::Zero();
    entry.previousTransform = Transform(entry.body.position, entry.body.rotation);

    // 动态体在拿到碰撞体之前 invMass 为 0（相当于运动学）。
    // 这不是妥协 —— 一个没有形状的"动态"刚体本来就没有质量可言，
    // 给它一个凭空的 invMass 反而会让它一被碰就飞。
    return m_bodies.Emplace(std::move(entry));
}

void PhysicsWorld::DestroyBody(BodyHandle handle) {
    BodyEntry* entry = m_bodies.Get(handle);
    if (entry == nullptr) return;

    // 先把碰撞体一个个拆掉 —— RemoveCollider 会负责补发触发器的 Exit 事件。
    // 拷贝一份列表再遍历：RemoveCollider 会改动原列表。
    const std::vector<ColliderHandle> owned = entry->colliders;
    for (const ColliderHandle ch : owned) {
        RemoveCollider(ch);
    }

    m_bodies.Remove(handle);
}

RigidBody* PhysicsWorld::GetBody(BodyHandle handle) {
    BodyEntry* entry = m_bodies.Get(handle);
    return entry != nullptr ? &entry->body : nullptr;
}

const RigidBody* PhysicsWorld::GetBody(BodyHandle handle) const {
    const BodyEntry* entry = m_bodies.Get(handle);
    return entry != nullptr ? &entry->body : nullptr;
}

const Collider* PhysicsWorld::GetCollider(ColliderHandle handle) const {
    return m_colliders.Get(handle);
}

Transform PhysicsWorld::GetBodyTransform(BodyHandle handle) const {
    const BodyEntry* entry = m_bodies.Get(handle);
    if (entry == nullptr) return Transform(Vec3::Zero(), Quat::Identity());
    return OriginTransform(entry->body, entry->centerOfMassLocal);
}

void PhysicsWorld::SetBodyTransform(BodyHandle handle, const Transform& transform) {
    BodyEntry* entry = m_bodies.Get(handle);
    if (entry == nullptr) return;

    RigidBody& body = entry->body;
    body.rotation = transform.rotation.Normalized();
    // 传入的是**原点**位姿，内部存的是质心
    body.position = transform.position + body.rotation.Rotate(entry->centerOfMassLocal);

    // 姿态变了，世界惯量必须跟着刷新 —— 这正是 Integrator.h 里反复强调的
    // "任何直接改写 rotation 的地方都要调 UpdateWorldInertia"。
    UpdateWorldInertia(body);
    body.WakeUp();

    entry->previousTransform = Transform(body.position, body.rotation);

    // 位姿变了，缓存的碰撞体世界位姿和宽相位代理也要同步，
    // 否则下一次查询会用上一帧的位置
    UpdateColliderTransforms(*entry);
    for (const ColliderHandle ch : entry->colliders) {
        Collider* c = m_colliders.Get(ch);
        if (c == nullptr) continue;
        m_broadPhase.Update(c->proxy, ComputeWorldAABB(c->shape, c->worldTransform));
    }
}

//==============================================================================
// 碰撞体
//==============================================================================

ColliderHandle PhysicsWorld::AddCollider(BodyHandle body, const ColliderDesc& desc) {
    BodyEntry* entry = m_bodies.Get(body);
    if (entry == nullptr) return ColliderHandle::Null();

    Collider collider;
    collider.shape = desc.shape;
    collider.localTransform = desc.localTransform;
    collider.material = desc.material;
    collider.layer = desc.layer;
    collider.layerMask = desc.layerMask;
    collider.body = body;
    collider.isTrigger = desc.isTrigger;
    collider.density = desc.density;
    collider.proxy = kInvalidProxyId;
    collider.worldTransform =
        OriginTransform(entry->body, entry->centerOfMassLocal) * desc.localTransform;

    const ColliderHandle handle = m_colliders.Emplace(collider);

    // 注册宽相位代理
    BroadPhaseProxyDesc proxyDesc;
    proxyDesc.aabb = ComputeWorldAABB(collider.shape, collider.worldTransform);
    proxyDesc.userData = PackCollider(handle);
    proxyDesc.layer = collider.layer;
    proxyDesc.layerMask = collider.layerMask;
    proxyDesc.isStatic = (entry->body.type == BodyType::Static);
    proxyDesc.isSleeping = entry->body.isSleeping;

    m_colliders.Get(handle)->proxy = m_broadPhase.Insert(proxyDesc);

    entry->colliders.push_back(handle);
    RecomputeMassProperties(body);
    return handle;
}

void PhysicsWorld::RemoveCollider(ColliderHandle handle) {
    Collider* collider = m_colliders.Get(handle);
    if (collider == nullptr) return;

    // 触发器要补发 Exit —— 少了这一步，游戏侧的"区域内对象列表"会留下幽灵条目
    m_triggers.RemoveCollider(handle, m_events);

    m_broadPhase.Remove(collider->proxy);

    const BodyHandle owner = collider->body;
    if (BodyEntry* entry = m_bodies.Get(owner)) {
        for (std::size_t i = 0; i < entry->colliders.size(); ++i) {
            if (entry->colliders[i] == handle) {
                entry->colliders[i] = entry->colliders.back();
                entry->colliders.pop_back();
                break;
            }
        }
    }

    m_colliders.Remove(handle);

    if (m_bodies.Get(owner) != nullptr) RecomputeMassProperties(owner);
}

//==============================================================================
// 质量属性的合成
//==============================================================================
//
// 一个刚体挂多个碰撞体时，要把各个部件的质量属性合成一个：
//   1. 总质量 = 各部件质量之和
//   2. 复合质心 = 按质量加权的平均位置
//   3. 复合惯量 = 各部件惯量先旋转到刚体坐标系，再用**平行轴定理**搬到复合质心
//
// 第 3 步的两个函数（RotateInertia / TranslateInertia）在 M6 就写好并测过了，
// 这里只是第一次真正用上它们。
//
// 注意合成之后惯量张量**不再是对角阵**（惯量积不为零）。`invInertiaLocal`
// 本来就是 Mat3，所以接口不用改，只是 `Mat3::Inverse()` 会走一般路径。
//==============================================================================

void PhysicsWorld::RecomputeMassProperties(BodyHandle handle) {
    BodyEntry* entry = m_bodies.Get(handle);
    if (entry == nullptr) return;

    RigidBody& body = entry->body;

    if (body.type != BodyType::Dynamic) {
        body.invMass = real(0);
        body.invInertiaLocal = Mat3::Diagonal(Vec3::Zero());
        UpdateWorldInertia(body);
        return;
    }

    // 第一遍：总质量与复合质心
    real totalMass = real(0);
    Vec3 weightedCenter = Vec3::Zero();

    for (const ColliderHandle ch : entry->colliders) {
        const Collider* c = m_colliders.Get(ch);
        if (c == nullptr || c->isTrigger) continue;  // 触发器没有质量

        const MassProperties props = ComputeMassProperties(c->shape, c->density);
        if (props.mass <= real(0)) continue;

        totalMass += props.mass;
        // 形状的局部原点就是它的几何中心（Shape.h 的约定），
        // 所以部件质心在刚体空间里就是 localTransform.position
        weightedCenter += c->localTransform.position * props.mass;
    }

    if (totalMass <= real(0)) {
        // 没有任何有质量的碰撞体：降级成 invMass = 0（等价运动学）。
        // 比给一个凭空的质量安全 —— 后者会让物体一被碰就飞。
        body.invMass = real(0);
        body.invInertiaLocal = Mat3::Diagonal(Vec3::Zero());
        UpdateWorldInertia(body);
        return;
    }

    const Vec3 centerOfMass = weightedCenter * (real(1) / totalMass);

    // 第二遍：把每个部件的惯量搬到复合质心上累加
    Mat3 inertia = Mat3::Diagonal(Vec3::Zero());
    for (const ColliderHandle ch : entry->colliders) {
        const Collider* c = m_colliders.Get(ch);
        if (c == nullptr || c->isTrigger) continue;

        const MassProperties props = ComputeMassProperties(c->shape, c->density);
        if (props.mass <= real(0)) continue;

        const Mat3 rotated =
            RotateInertia(props.inertia, c->localTransform.rotation.ToMat3());
        const Vec3 offset = c->localTransform.position - centerOfMass;
        inertia = inertia + TranslateInertia(rotated, props.mass, offset);
    }

    // 用户指定了质量就整体等比缩放（惯量与质量成正比）
    const real finalMass =
        (entry->requestedMass > real(0)) ? entry->requestedMass : totalMass;
    inertia = inertia * (finalMass / totalMass);

    //--------------------------------------------------------------------------
    // 质心位置变了，刚体的 position（存的是质心）要跟着挪，
    // 好让用户看到的"物体原点"待在原地。
    //
    // 少了这一步，每加一个偏心的碰撞体，物体都会自己跳一下 ——
    // 而且跳的方向毫无规律，非常难查。
    //--------------------------------------------------------------------------
    const Vec3 origin = body.position - body.rotation.Rotate(entry->centerOfMassLocal);
    entry->centerOfMassLocal = centerOfMass;
    body.position = origin + body.rotation.Rotate(centerOfMass);

    body.invMass = real(1) / finalMass;
    body.invInertiaLocal = inertia.Inverse();
    UpdateWorldInertia(body);
}

void PhysicsWorld::UpdateColliderTransforms(BodyEntry& entry) {
    const Transform origin = OriginTransform(entry.body, entry.centerOfMassLocal);
    for (const ColliderHandle ch : entry.colliders) {
        Collider* c = m_colliders.Get(ch);
        if (c == nullptr) continue;
        c->worldTransform = origin * c->localTransform;
    }
}

//==============================================================================
// 推进
//==============================================================================

void PhysicsWorld::Step(real deltaTime) {
    m_events.Clear();
    m_stats.ccdSweeps = 0;
    m_stats.subSteps = 0;

    if (deltaTime > real(0)) m_accumulator += deltaTime;

    const real h = m_config.fixedTimeStep;
    if (h <= real(0)) return;

    int steps = 0;
    while (m_accumulator >= h && steps < m_config.maxSubSteps) {
        FixedStep(h);
        m_accumulator -= h;
        ++steps;
    }
    m_stats.subSteps = steps;

    //--------------------------------------------------------------------------
    // 死亡螺旋的保险丝：跑满了子步还有剩，就把多余的时间**丢掉**。
    //
    // 不丢的话：这一帧慢 -> 累加器涨 -> 下一帧要跑更多步 -> 更慢 -> …… 直到卡死。
    // 丢掉的表现是"卡顿时物理走得比现实慢一点"，玩家几乎察觉不到，
    // 远好于整个游戏冻住。
    //--------------------------------------------------------------------------
    if (m_accumulator >= h) {
        m_accumulator = real(0);
    }

    //--------------------------------------------------------------------------
    // 事件统一在最末尾派发。此时所有遍历都结束了，回调里创建/销毁物体是安全的
    // —— 这正是当初把事件做成队列而不是直接回调的原因（见 Events.h）。
    //--------------------------------------------------------------------------
    DispatchEvents(m_events, m_listener);
}

void PhysicsWorld::FixedStep(real dt) {
    ++m_stepIndex;

    SyncBroadPhase(dt);
    CollectContacts();
    SolveStep(dt);
    IntegrateAndFinalize(dt);

    // 触发器差分 -> 事件入队（派发在 Step 末尾）
    m_triggers.EndFrame(m_events);
}

//------------------------------------------------------------------------------
// 阶段 1：同步宽相位
//------------------------------------------------------------------------------
void PhysicsWorld::SyncBroadPhase(real dt) {
    std::size_t updated = 0;
    std::size_t relinked = 0;

    m_bodies.ForEach([&](BodyHandle, BodyEntry& entry) {
        UpdateColliderTransforms(entry);

        for (const ColliderHandle ch : entry.colliders) {
            Collider* c = m_colliders.Get(ch);
            if (c == nullptr) continue;

            ++updated;

            // 判定用**紧致** AABB，预测量单独传 —— 两者不能合并，
            // 理由见 BroadPhase.h 里 Update 的说明（合并了预测就完全失效）。
            const AABB tight = ComputeWorldAABB(c->shape, c->worldTransform);
            if (m_broadPhase.Update(c->proxy, tight,
                                    PredictedDisplacement(entry.body, dt))) {
                ++relinked;
            }
            m_broadPhase.SetSleeping(c->proxy, entry.body.isSleeping);
        }
    });

    m_stats.proxyRelinkRatio =
        updated > 0 ? static_cast<real>(relinked) / static_cast<real>(updated) : real(0);
}

//------------------------------------------------------------------------------
// 速度感知的 fat AABB
//
// 这一段直接来自 M3 的实测教训：只用固定 margin（0.05 米）的话，一个 5 m/s 的
// 物体每帧走 0.083 米，**每帧都跨出膨胀壳**，实测 76% 的 Update 都触发了格子
// 重建 —— fat AABB 这个机制等于没开。
//
// 修法是让宽相位在**重建壳的时候**额外沿速度方向扫掠一段，物体接下来几帧都待在
// 壳里。注意扫掠量只能加在壳上、不能加在判定用的 AABB 上（见 BroadPhase.h）。
//
// 速度是从**这一层**取的，宽相位仍然不知道速度的存在 —— 保持了 §2 的分层。
//------------------------------------------------------------------------------
Vec3 PhysicsWorld::PredictedDisplacement(const RigidBody& body, real dt) const {
    if (body.type == BodyType::Static || m_config.aabbVelocityPrediction <= real(0)) {
        return Vec3::Zero();
    }
    return body.linearVelocity * (dt * m_config.aabbVelocityPrediction);
}

//------------------------------------------------------------------------------
// 阶段 2：宽相位 -> 窄相位
//------------------------------------------------------------------------------
void PhysicsWorld::CollectContacts() {
    m_broadPhase.QueryPairs(m_pairs);
    m_stats.broadPhasePairs = m_pairs.size();

    m_manifolds.clear();
    m_contactInputs.clear();
    m_triggers.BeginFrame();

    for (const BroadPhasePair& pair : m_pairs) {
        const BroadPhaseProxy* pa = m_broadPhase.GetProxy(pair.a);
        const BroadPhaseProxy* pb = m_broadPhase.GetProxy(pair.b);
        if (pa == nullptr || pb == nullptr) continue;

        const ColliderHandle ha = UnpackCollider(pa->userData);
        const ColliderHandle hb = UnpackCollider(pb->userData);
        Collider* ca = m_colliders.Get(ha);
        Collider* cb = m_colliders.Get(hb);
        if (ca == nullptr || cb == nullptr) continue;

        // 同一个刚体上的两个碰撞体不该互相碰撞
        if (ca->body == cb->body) continue;

        BodyEntry* ea = m_bodies.Get(ca->body);
        BodyEntry* eb = m_bodies.Get(cb->body);
        if (ea == nullptr || eb == nullptr) continue;

        //----------------------------------------------------------------------
        // 触发器：只判重叠，**不生成约束**，物体直接穿过去
        //----------------------------------------------------------------------
        if (ca->isTrigger || cb->isTrigger) {
            if (Overlap(ca->shape, ca->worldTransform, cb->shape, cb->worldTransform)) {
                // 两个都是触发器时，把下标小的当作 trigger，保证顺序稳定
                if (ca->isTrigger) {
                    m_triggers.AddOverlap(ha, hb);
                } else {
                    m_triggers.AddOverlap(hb, ha);
                }
            }
            continue;
        }

        //----------------------------------------------------------------------
        // 一方醒着、另一方睡着 -> 必须把睡着的那个叫醒。
        //
        // 不叫醒的话会出一个很隐蔽的错：求解器只看 invMass，它会照常给睡着的
        // 物体施加冲量改速度，但 `IntegratePosition` 对睡着的物体直接返回 ——
        // 于是那些冲量**凭空消失**，接触约束等于失效，压在上面的东西会一路陷下去。
        // 实测就是这样：一摞 8 层的箱子会塌成一坨，穿透深到 0.7 米。
        //
        // 两边都睡着的对在宽相位那一层就被丢掉了（BroadPhaseShouldCollide 的
        // 第一条规则），所以经过这里之后，求解器再也见不到睡着的动态体。
        //
        // 代价是**成堆的物体睡不着**：A 先睡、被还醒着的 B 叫醒，然后 B 睡、
        // 又被 A 叫醒，来回拉锯。真正的解法是求解器孤岛（整组一起睡一起醒），
        // 那是 UPGRADE_NOTES 里 M6/M7 都记着的一条。单个物体躺在静态地面上
        // 不受影响 —— 静态体永远不"醒"，不会去叫醒谁。
        //----------------------------------------------------------------------
        const bool aAwake = ea->body.IsActive();
        const bool bAwake = eb->body.IsActive();
        if (aAwake != bAwake) {
            if (!aAwake && ea->body.type == BodyType::Dynamic) ea->body.WakeUp();
            if (!bAwake && eb->body.type == BodyType::Dynamic) eb->body.WakeUp();
        }

        Manifold manifold;
        if (!Collide(ca->shape, ca->worldTransform, cb->shape, cb->worldTransform,
                     manifold)) {
            continue;
        }

        m_manifolds.push_back(manifold);

        ContactInput input;
        input.bodyA = &ea->body;
        input.bodyB = &eb->body;
        input.manifold = nullptr;  // 稍后统一填，见下面的说明
        input.materialA = ca->material;
        input.materialB = cb->material;
        input.pairKey = MakePairKey(ha, hb);
        m_contactInputs.push_back(input);

        //----------------------------------------------------------------------
        // 接触开始事件
        //----------------------------------------------------------------------
        const std::uint64_t key = input.pairKey;
        const auto it = m_touching.find(key);
        if (it == m_touching.end()) {
            ContactEvent e;
            e.type = ContactEventType::Begin;
            e.a = ha;
            e.b = hb;
            e.point = manifold.points[0].position;
            e.normal = manifold.normal;
            e.impulse = real(0);  // 求解之后才知道，这里先留 0
            m_events.Push(e);
            m_touching.emplace(key, m_stepIndex);

            //------------------------------------------------------------------
            // 只在接触**刚刚建立**时唤醒，不能每帧都唤醒。
            //
            // 一个静止在地面上的箱子，接触是持续存在的、穿透稳定在 slop 附近。
            // 如果"有接触就唤醒"，它每帧都会被叫醒，休眠计时器永远清零 ——
            // 场景里的物体一个都睡不着，休眠这个机制等于没有。
            //
            // 新接触才唤醒是正确的判据：有东西**新砸过来**才需要醒。
            // 而已经贴在一起、双方都睡着的那些对，宽相位那一层就直接丢掉了
            // （BroadPhaseShouldCollide 的第一条规则），连这里都到不了。
            //------------------------------------------------------------------
            if (ea->body.type == BodyType::Dynamic) ea->body.WakeUp();
            if (eb->body.type == BodyType::Dynamic) eb->body.WakeUp();
        } else {
            it->second = m_stepIndex;
        }
    }

    //--------------------------------------------------------------------------
    // 流形的指针必须在 vector **填完之后**才能取 —— 中途 push_back 会触发扩容，
    // 之前取的指针就全悬垂了。这类 bug 只在容量恰好用尽的那一帧出现，
    // 极难复现，所以宁可多走一遍循环。
    //--------------------------------------------------------------------------
    for (std::size_t i = 0; i < m_contactInputs.size(); ++i) {
        m_contactInputs[i].manifold = &m_manifolds[i];
    }

    m_stats.narrowPhaseContacts = m_contactInputs.size();

    //--------------------------------------------------------------------------
    // 接触结束事件：这一步没被刷新过的对就是分开了
    //--------------------------------------------------------------------------
    for (auto it = m_touching.begin(); it != m_touching.end();) {
        if (it->second == m_stepIndex) {
            ++it;
            continue;
        }
        ContactEvent e;
        e.type = ContactEventType::End;
        e.a = ColliderHandle::Null();
        e.b = ColliderHandle::Null();
        e.point = Vec3::Zero();
        e.normal = Vec3::Zero();
        e.impulse = real(0);
        m_events.Push(e);
        it = m_touching.erase(it);
    }
}

//------------------------------------------------------------------------------
// 阶段 3：积分速度 + 求解
//------------------------------------------------------------------------------
void PhysicsWorld::SolveStep(real dt) {
    m_solver.BeginFrame();
    for (const ContactInput& input : m_contactInputs) {
        m_solver.AddContact(input);
    }

    //--------------------------------------------------------------------------
    // 积分速度必须在**碰撞检测之后、求解之前**。
    //
    // 之后：碰撞检测用的是上一步末的位置（离散引擎的标准做法，也正是隧穿的根源，
    //       所以才要 CCD）。
    // 之前：求解器的恢复系数要用"加上这一步重力之后"的接近速度，
    //       否则弹跳高度会偏。
    //--------------------------------------------------------------------------
    std::size_t active = 0;
    m_bodies.ForEach([&](BodyHandle, BodyEntry& entry) {
        IntegrateVelocity(entry.body, m_config.gravity, dt);
        if (entry.body.IsActive()) ++active;
    });
    m_stats.activeBodies = active;

    m_solver.Solve(dt);
    m_solver.EndFrame();
    m_stats.maxPenetration = m_solver.MaxPenetration();
}

//------------------------------------------------------------------------------
// 阶段 4：积分位置（含 CCD）+ 收尾
//------------------------------------------------------------------------------
void PhysicsWorld::IntegrateAndFinalize(real dt) {
    m_bodies.ForEach([&](BodyHandle handle, BodyEntry& entry) {
        // 渲染插值要用上一步的位姿，所以先存再动
        entry.previousTransform = Transform(entry.body.position, entry.body.rotation);

        if (!m_config.enableCcd || !ApplyCcd(entry, handle, dt)) {
            IntegratePosition(entry.body, dt);
        }

        ClearForces(entry.body);

        if (m_config.enableSleeping) {
            UpdateSleep(entry.body, dt, m_config.sleep);
        }
    });
}

//------------------------------------------------------------------------------
// CCD
//
// 只对**高速**物体做：判据是"这一步的位移超过自身包围球半径"。
// 剩下 99% 的物体走正常的离散积分 —— 扫掠比积分贵得多，给所有物体都做既慢又没用。
//
// 命中之后**只截断位置，不改速度**：下一步的离散检测会看到接触，
// 由求解器正常处理弹跳与摩擦。在这里动速度会让碰撞响应变成两套逻辑。
//------------------------------------------------------------------------------
bool PhysicsWorld::ApplyCcd(BodyEntry& entry, BodyHandle handle, real dt) {
    RigidBody& body = entry.body;
    if (body.type != BodyType::Dynamic || body.isSleeping) return false;
    if (entry.colliders.empty()) return false;

    const Vec3 displacement = body.linearVelocity * dt;
    const real distance = displacement.Length();
    if (distance <= kEpsilon) return false;

    const Collider* collider = m_colliders.Get(entry.colliders[0]);
    if (collider == nullptr || collider->isTrigger) return false;

    const real radius = collider->shape.BoundingRadius();
    if (radius <= real(0) || distance <= radius) return false;  // 不够快，不用管

    ++m_stats.ccdSweeps;

    WorldShapeCastHit hit;
    if (!ShapeCastWorld(collider->shape, collider->worldTransform, displacement, hit,
                        collider->layerMask, handle)) {
        return false;
    }
    if (hit.startPenetrating || hit.fraction >= real(1)) return false;

    // 留一点余量，别正好贴死在接触面上
    const real safeFraction = Max(real(0), hit.fraction - real(0.01));

    body.position += displacement * safeFraction;
    body.rotation = Quat::Integrate(body.rotation, body.angularVelocity, dt);
    UpdateWorldInertia(body);
    return true;
}

//==============================================================================
// 渲染插值
//==============================================================================

Transform PhysicsWorld::GetInterpolatedTransform(BodyHandle handle, real alpha) const {
    const BodyEntry* entry = m_bodies.Get(handle);
    if (entry == nullptr) return Transform(Vec3::Zero(), Quat::Identity());

    const Transform current(entry->body.position, entry->body.rotation);
    const Transform blended =
        InterpolateTransform(entry->previousTransform, current, alpha);

    // 插值出来的是**质心**位姿，对外要换算回原点
    return Transform(blended.position - blended.rotation.Rotate(entry->centerOfMassLocal),
                     blended.rotation);
}

//==============================================================================
// 查询
//==============================================================================

bool PhysicsWorld::Raycast(const Ray& ray, WorldRaycastHit& out,
                           LayerMask layerMask) const {
    m_queryResult.clear();
    m_broadPhase.QueryRay(ray, m_queryResult);

    bool found = false;
    out.distance = ray.maxDistance;

    for (const ProxyId id : m_queryResult) {
        const BroadPhaseProxy* proxy = m_broadPhase.GetProxy(id);
        if (proxy == nullptr) continue;

        const ColliderHandle handle = UnpackCollider(proxy->userData);
        const Collider* collider = m_colliders.Get(handle);
        if (collider == nullptr) continue;
        if (collider->isTrigger) continue;  // 子弹不该被触发区挡住
        if ((collider->layer & layerMask) == 0u) continue;

        RaycastHit hit;
        if (!RaycastShape(collider->shape, collider->worldTransform, ray, hit)) continue;
        if (found && hit.distance >= out.distance) continue;

        out.distance = hit.distance;
        out.point = hit.point;
        out.normal = hit.normal;
        out.collider = handle;
        out.body = collider->body;
        found = true;
    }

    return found;
}

bool PhysicsWorld::ShapeCastWorld(const Shape& shape, const Transform& start,
                                  const Vec3& displacement, WorldShapeCastHit& out,
                                  LayerMask layerMask, BodyHandle ignoreBody) const {
    // 用"扫过的整段区域"的 AABB 去做粗筛
    const AABB swept = Sweep(ComputeWorldAABB(shape, start), displacement);

    m_queryResult.clear();
    m_broadPhase.QueryAABB(swept, m_queryResult);

    bool found = false;
    out.fraction = real(1);
    out.startPenetrating = false;
    out.depth = real(0);

    for (const ProxyId id : m_queryResult) {
        const BroadPhaseProxy* proxy = m_broadPhase.GetProxy(id);
        if (proxy == nullptr) continue;

        const ColliderHandle handle = UnpackCollider(proxy->userData);
        const Collider* collider = m_colliders.Get(handle);
        if (collider == nullptr || collider->isTrigger) continue;
        if ((collider->layer & layerMask) == 0u) continue;
        if (ignoreBody.IsValid() && collider->body == ignoreBody) continue;

        ShapeCastHit hit;
        if (!ShapeCast(shape, start, displacement, collider->shape,
                       collider->worldTransform, hit)) {
            continue;
        }

        //----------------------------------------------------------------------
        // 起点重叠的优先级最高：必须先脱困才谈得上移动。
        // 多个重叠时取最深的那个 —— 先解决最要紧的。
        //----------------------------------------------------------------------
        const bool better =
            hit.startPenetrating
                ? (!out.startPenetrating || hit.depth > out.depth)
                : (!out.startPenetrating && hit.fraction < out.fraction);

        if (!found || better) {
            out.fraction = hit.fraction;
            out.point = hit.point;
            out.normal = hit.normal;
            out.startPenetrating = hit.startPenetrating;
            out.depth = hit.depth;
            out.collider = handle;
            out.body = collider->body;
            found = true;
        }
    }

    return found;
}

bool PhysicsWorld::SweepCharacter(const Shape& shape, const Transform& start,
                                  const Vec3& displacement, LayerMask layerMask,
                                  ShapeCastHit& out) const {
    WorldShapeCastHit hit;
    if (!ShapeCastWorld(shape, start, displacement, hit, layerMask)) return false;

    out.fraction = hit.fraction;
    out.point = hit.point;
    out.normal = hit.normal;
    out.startPenetrating = hit.startPenetrating;
    out.depth = hit.depth;
    return true;
}

void PhysicsWorld::OverlapAABB(const AABB& box, std::vector<ColliderHandle>& out,
                               LayerMask layerMask) const {
    out.clear();
    m_queryResult.clear();
    m_broadPhase.QueryAABB(box, m_queryResult);

    for (const ProxyId id : m_queryResult) {
        const BroadPhaseProxy* proxy = m_broadPhase.GetProxy(id);
        if (proxy == nullptr) continue;
        const ColliderHandle handle = UnpackCollider(proxy->userData);
        const Collider* collider = m_colliders.Get(handle);
        if (collider == nullptr) continue;
        if ((collider->layer & layerMask) == 0u) continue;
        out.push_back(handle);
    }
}

void PhysicsWorld::OverlapShape(const Shape& shape, const Transform& transform,
                                std::vector<ColliderHandle>& out,
                                LayerMask layerMask) const {
    out.clear();
    m_queryResult.clear();
    m_broadPhase.QueryAABB(ComputeWorldAABB(shape, transform), m_queryResult);

    for (const ProxyId id : m_queryResult) {
        const BroadPhaseProxy* proxy = m_broadPhase.GetProxy(id);
        if (proxy == nullptr) continue;
        const ColliderHandle handle = UnpackCollider(proxy->userData);
        const Collider* collider = m_colliders.Get(handle);
        if (collider == nullptr) continue;
        if ((collider->layer & layerMask) == 0u) continue;

        // 宽相位只保证 AABB 重叠，这里做精确判定
        if (!Overlap(shape, transform, collider->shape, collider->worldTransform)) {
            continue;
        }
        out.push_back(handle);
    }
}

//==============================================================================
// 调试绘制
//==============================================================================

void PhysicsWorld::GetDebugDrawData(std::vector<DebugShape>& out) const {
    out.clear();
    out.reserve(m_colliders.Size());

    m_colliders.ForEach([&](ColliderHandle handle, const Collider& collider) {
        const BodyEntry* entry = m_bodies.Get(collider.body);
        if (entry == nullptr) return;

        DebugShape item;
        item.shape = collider.shape;
        item.transform = collider.worldTransform;
        item.collider = handle;
        item.body = collider.body;
        item.isStatic = (entry->body.type == BodyType::Static);
        item.isSleeping = entry->body.isSleeping;
        item.isTrigger = collider.isTrigger;
        out.push_back(item);
    });
}

void PhysicsWorld::GetDebugContacts(std::vector<DebugContact>& out) const {
    out.clear();
    for (const ContactConstraint& c : m_solver.Contacts()) {
        for (std::uint8_t i = 0; i < c.pointCount; ++i) {
            // 接触点的位置存的是相对质心的力臂，画的时候要还原成世界坐标
            out.push_back(DebugContact{c.bodyA->position + c.points[i].rA, c.normal,
                                       c.points[i].normalImpulse});
        }
    }
}

}  // namespace pe
