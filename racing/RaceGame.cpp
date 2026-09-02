//==============================================================================
// racing/RaceGame.cpp
//==============================================================================

#include "racing/RaceGame.h"

#include <cstdio>

namespace racing {

namespace {

WorldConfig MakeWorldConfig(const RaceConfig& config) {
    WorldConfig world;
    world.gravity = config.gravity;
    // 格子取大一点（10 米）：这张图只有 44 个刚体，格子里装几个物体无所谓，
    // 而渲染射线要横穿整张图 —— 每条射线少走一半格子，比精确剔除值钱得多。
    world.cellSize = real(10);
    return world;
}

//------------------------------------------------------------------------------
// 赛道尺寸。一条椭圆形的环道，四角切 45 度。
//
// 尺寸是**按车速定的**，不是随手画的：极速 58 m/s，直道要跑得起来又不能长到
// 无聊，取 110 米；弯道半径要让 2.0 抓地力的车在 25 m/s 左右刚好压住，
// 于是赛道宽 16 米、外圈半宽 55 米。
//------------------------------------------------------------------------------
constexpr real kOuterX = real(55);
constexpr real kOuterZ = real(35);
constexpr real kInnerX = real(39);
constexpr real kInnerZ = real(19);
constexpr real kCorner = real(14);   ///< 四角切角的边长
// 护墙要够高。1.6 米时，车斜着撞上去会顺着墙面**爬上去**再翻过来 ——
// 车身半高 0.35 米、还带着速度，矮墙对它就是个跳板。2.6 米挡得住。
constexpr real kWallHeight = real(2.6);

}  // namespace

//==============================================================================
// 构造与赛道
//==============================================================================

RaceGame::RaceGame() : m_world(MakeWorldConfig(m_config)) {
    m_world.SetEventListener(this);
    m_countdown = m_config.countdownSeconds;
}

void RaceGame::RegisterEntity(const Entity& entity) {
    const int index = static_cast<int>(m_entities.size());
    m_entities.push_back(entity);
    if (entity.body.IsValid()) m_bodyToEntity[entity.body.index] = index;
}

const Entity* RaceGame::FindEntity(BodyHandle body) const {
    if (!body.IsValid()) return nullptr;
    const auto it = m_bodyToEntity.find(body.index);
    if (it == m_bodyToEntity.end()) return nullptr;
    const Entity& entity = m_entities[static_cast<std::size_t>(it->second)];
    return entity.body == body ? &entity : nullptr;
}

BodyHandle RaceGame::AddStaticBox(EntityKind kind, const Vec3& center, const Vec3& half,
                                  const Quat& rotation) {
    BodyDesc desc;
    desc.position = center;
    desc.rotation = rotation;
    desc.type = BodyType::Static;
    const BodyHandle body = m_world.CreateBody(desc);

    ColliderDesc collider;
    collider.shape = Shape::MakeBox(half);
    // 路面摩擦给得低：**抓地力全部由轮胎模型负责**（见 Vehicle.cpp 的摩擦圆）。
    // 车壳蹭到地面时不该突然多出一份摩擦，那会让翻车之后的车像粘在地上。
    collider.material = Material(real(0.4), real(0.05));
    collider.layer = kLayerTrack;
    m_world.AddCollider(body, collider);

    Entity entity;
    entity.kind = kind;
    entity.body = body;
    RegisterEntity(entity);
    return body;
}

void RaceGame::AddWallRun(const Vec3& from, const Vec3& to, real height) {
    const Vec3 delta = to - from;
    const real length = delta.Length();
    if (length < real(0.01)) return;

    const Vec3 center = (from + to) * real(0.5) + Vec3(0, height * real(0.5), 0);
    // 盒子默认沿 +Z 方向"长"，所以绕 Y 转到线段的方向上。
    // 注意负号：绕 +Y 的正向旋转把 +Z 转向 +X（右手系），而 atan2(x, z)
    // 给的正是从 +Z 量到目标的角，两者刚好差一个符号。
    const real yaw = Atan2(delta.x, delta.z);
    AddStaticBox(EntityKind::Wall, center,
                 Vec3(real(0.5), height * real(0.5), length * real(0.5)),
                 Quat::FromAxisAngle(Vec3(0, real(1), 0), yaw));
}

void RaceGame::AddProp(const Vec3& center, real half, real mass) {
    BodyDesc desc;
    desc.position = center;
    desc.type = BodyType::Dynamic;
    desc.mass = mass;
    const BodyHandle body = m_world.CreateBody(desc);

    ColliderDesc collider;
    collider.shape = Shape::MakeCube(half);
    collider.material = Material::Wood();
    collider.layer = kLayerProp;
    m_world.AddCollider(body, collider);

    Entity entity;
    entity.kind = EntityKind::Prop;
    entity.body = body;
    RegisterEntity(entity);
}

void RaceGame::AddCheckpoint(const Vec3& center, real yaw, real halfWidth, int index) {
    //--------------------------------------------------------------------------
    // 检查点是一块**触发区**：横跨整条赛道、足够高、车穿过去就算过。
    //
    // 用触发器而不是"每帧算车到某条线的距离"，是因为触发器是引擎在做连续的
    // 重叠判定：一辆 200 km/h 的车一帧要走 0.9 米，用距离判定很容易整帧跳过。
    //--------------------------------------------------------------------------
    BodyDesc desc;
    desc.position = center;
    desc.rotation = Quat::FromAxisAngle(Vec3(0, real(1), 0), yaw);
    desc.type = BodyType::Static;
    const BodyHandle body = m_world.CreateBody(desc);

    ColliderDesc collider;
    collider.shape = Shape::MakeBox(Vec3(halfWidth, real(4), real(1.2)));
    collider.isTrigger = true;
    collider.layer = layers::kTrigger;
    m_world.AddCollider(body, collider);

    Entity entity;
    entity.kind = EntityKind::Checkpoint;
    entity.body = body;
    entity.checkpointIndex = index;
    RegisterEntity(entity);

    m_checkpoints.push_back(Checkpoint{center, yaw, halfWidth});

    //--------------------------------------------------------------------------
    // 触发区是看不见的，所以两边各立一根门柱。
    //
    // 玩家必须**看得见**下一个检查点在哪 —— 否则"你漏过了一个检查点"这件事
    // 只能靠圈数不涨来事后发现，那是最让人火大的一类反馈。
    //--------------------------------------------------------------------------
    const Vec3 across(Cos(yaw), 0, -Sin(yaw));
    for (int side = -1; side <= 1; side += 2) {
        const Vec3 postCenter =
            center + across * (halfWidth * static_cast<real>(side)) + Vec3(0, real(1.5), 0);
        BodyDesc post;
        post.position = postCenter;
        post.type = BodyType::Static;
        const BodyHandle postBody = m_world.CreateBody(post);

        ColliderDesc postCollider;
        postCollider.shape = Shape::MakeBox(Vec3(real(0.35), real(1.5), real(0.35)));
        postCollider.layer = kLayerDecor;
        postCollider.layerMask = 0;  // 纯装饰：撞不到，只是给眼睛看的
        m_world.AddCollider(postBody, postCollider);

        Entity postEntity;
        postEntity.kind = EntityKind::Gate;
        postEntity.body = postBody;
        postEntity.checkpointIndex = index;
        RegisterEntity(postEntity);
    }
}

//------------------------------------------------------------------------------
// 赛道
//------------------------------------------------------------------------------
void RaceGame::BuildTrack() {
    //--------------------------------------------------------------------------
    // 路面是**一整块大平板**，赛道是靠内外两圈护墙"切"出来的。
    //
    // 为什么不用一段段的路面拼出弯道：那样每两段之间都会有一条接缝，而悬挂
    // 射线打在接缝上会得到跳变的法线，车会莫名其妙地弹一下。一整块平板则
    // 到处都是同一个平面 —— 赛车游戏里路面的连续性比形状重要得多。
    //--------------------------------------------------------------------------
    AddStaticBox(EntityKind::Road, Vec3(0, real(-0.5), 0),
                 Vec3(kOuterX + real(6), real(0.5), kOuterZ + real(6)));

    //--------------------------------------------------------------------------
    // 内外两圈护墙。四角切 45 度，跑起来才有弯道。
    //--------------------------------------------------------------------------
    const auto ring = [&](real ex, real ez, real cut) {
        const Vec3 corners[8] = {
            Vec3(ex, 0, ez - cut),   Vec3(ex - cut, 0, ez),
            Vec3(-ex + cut, 0, ez),  Vec3(-ex, 0, ez - cut),
            Vec3(-ex, 0, -ez + cut), Vec3(-ex + cut, 0, -ez),
            Vec3(ex - cut, 0, -ez),  Vec3(ex, 0, -ez + cut),
        };
        for (int i = 0; i < 8; ++i) {
            AddWallRun(corners[i], corners[(i + 1) % 8], kWallHeight);
        }
    };
    ring(kOuterX, kOuterZ, kCorner);
    ring(kInnerX, kInnerZ, kCorner);

    //--------------------------------------------------------------------------
    // 赛车线：内外圈**正中间**那一圈，每 4 米取一个点。
    //
    // 用和护墙同一份几何算出来，而不是另外手写一串坐标 —— 改赛道尺寸时，
    // 墙和 AI 的路线会一起变。两份各写各的，迟早会出现"改了赛道，AI 开进墙里"。
    //--------------------------------------------------------------------------
    {
        const real mx = (kOuterX + kInnerX) * real(0.5);
        const real mz = (kOuterZ + kInnerZ) * real(0.5);
        const real cut = kCorner;
        const Vec3 corners[8] = {
            Vec3(mx, 0, mz - cut),   Vec3(mx - cut, 0, mz),
            Vec3(-mx + cut, 0, mz),  Vec3(-mx, 0, mz - cut),
            Vec3(-mx, 0, -mz + cut), Vec3(-mx + cut, 0, -mz),
            Vec3(mx - cut, 0, -mz),  Vec3(mx, 0, -mz + cut),
        };
        // 注意顺序：赛车线的方向必须和检查点顺序一致（终点线往 +X 跑），
        // 所以从 (mx, mz-cut) 开始**倒着**走这个数组。
        for (int i = 8; i > 0; --i) {
            const Vec3& from = corners[i % 8];
            const Vec3& to = corners[i - 1];
            const Vec3 delta = to - from;
            const int samples = Max(1, static_cast<int>(delta.Length() / real(4)));
            for (int s = 0; s < samples; ++s) {
                m_racingLine.push_back(
                    from + delta * (static_cast<real>(s) / static_cast<real>(samples)));
            }
        }

        //----------------------------------------------------------------------
        // 把折线**磨圆**。八边形的角是一个瞬间的 45 度折点，照着开的话，
        // 车要在一个路点的距离里转完 45 度 —— 那不是过弯，那是撞墙。
        // 几遍邻域平均就能得到一条真正的弧线，而且它天然落在赛道中间。
        //----------------------------------------------------------------------
        const std::size_t count = m_racingLine.size();
        for (int pass = 0; pass < 6; ++pass) {
            std::vector<Vec3> smoothed = m_racingLine;
            for (std::size_t i = 0; i < count; ++i) {
                const Vec3& prev = m_racingLine[(i + count - 1) % count];
                const Vec3& next = m_racingLine[(i + 1) % count];
                smoothed[i] = prev * real(0.25) + m_racingLine[i] * real(0.5) +
                              next * real(0.25);
            }
            m_racingLine.swap(smoothed);
        }
    }

    //--------------------------------------------------------------------------
    // 后直道上的跳台：一段抬起来的斜面 + 一小段落差。
    //
    // 它存在的理由不是"好看"，是**它会把悬挂逼到极限**。落地那一下，四个
    // 弹簧要在几十毫秒里吸掉整台车的动能；参数不对的话车会原地弹起来两米。
    // 一个跳台就能把悬挂调好没调好暴露得干干净净。
    //--------------------------------------------------------------------------
    //
    // 两个不能想当然的地方：
    //
    //   - **绕哪个轴转**。上方直道是沿 X 跑的，所以斜面要绕 **Z** 轴倾斜。
    //     第一版绕 X 轴转，等于把跳台侧着放 —— 车迎面撞上的是一堵 1.2 米高的
    //     垂直墙，72 km/h 直接撞停。
    //   - **入口必须和路面齐平**。盒子转个角度之后两头都是斜的，得把入口那端
    //     算到 y = 0、另一端抬到跳台高度，再把整块往下埋进路面里，
    //     接缝处才不会有一道让车弹起来的台阶。
    //
    // 车在上方直道是往 -X 跑的，所以 +X 那头是入口、要低。
    //     top_y(入口) = cy + hx*sinθ + hy*cosθ = 0
    //     top_y(出口) = cy - hx*sinθ + hy*cosθ = 跳台高度
    // 两式一减就得到 sinθ = -高度 / (2*hx)。
    //
    // 跳台放在**赛车线外侧**，不压在最快的那条线上。
    //
    // 一开始它就摆在上方直道正中间，结果每一圈都是：飞起来 -> 落地还没稳住 ->
    // 直接进左弯 -> 翻车。跳台变成了纯粹的惩罚，而不是玩家想去踩的东西。
    // 挪到外侧之后它是一个**可选**动作：想飞就往外压一点，赶时间就直接过。
    // 顺便，冒烟测试里的机器人也不会再被它掀翻。
    {
        const real halfLength = real(7);
        const real halfThick = real(1.2);
        const real jumpHeight = real(1.1);
        const real angle = -SafeAsin(jumpHeight / (real(2) * halfLength));
        const real centerY = -halfLength * Sin(angle) - halfThick * Cos(angle);
        AddStaticBox(EntityKind::Ramp, Vec3(real(-8), centerY, -kOuterZ + real(3)),
                     Vec3(halfLength, halfThick, real(2.8)),
                     Quat::FromAxisAngle(Vec3(0, 0, real(1)), angle));
    }

    //-- 几个可以撞飞的路障：动态刚体，撞上去会翻会滚 ---------------------------
    for (int i = 0; i < 5; ++i) {
        AddProp(Vec3(real(28) + real(i) * real(1.3), real(0.5),
                     kOuterZ - real(8) + real(i % 2) * real(2)),
                real(0.5), real(24));
    }
    for (int i = 0; i < 4; ++i) {
        AddProp(Vec3(real(-30) - real(i) * real(1.4), real(0.5), -kOuterZ + real(7)),
                real(0.5), real(24));
    }

    //--------------------------------------------------------------------------
    // 检查点。0 号就是终点线，剩下的按行驶顺序排在赛道各处。
    // 车从终点线后面发车，逆时针跑（先往 +X）。
    //--------------------------------------------------------------------------
    const real midX = (kOuterX + kInnerX) * real(0.5);
    const real midZ = (kOuterZ + kInnerZ) * real(0.5);
    const real gateHalf = (kOuterX - kInnerX) * real(0.5) - real(1);

    //--------------------------------------------------------------------------
    // 每个检查点的 yaw 是**车穿过它时的行驶方向**，不是门的摆放角度。
    // 车头的局部朝向是 +Z（见 Vehicle::Forward），绕 +Y 转 yaw 之后指向
    // (sin yaw, 0, cos yaw) —— 所以"往 +X 跑"对应 yaw = 90 度。
    //
    // 这一个约定同时被三处用到：门的旋转、发车朝向、复位朝向。写错的表现是
    // 车对着墙发车，非常好认。
    //--------------------------------------------------------------------------
    AddCheckpoint(Vec3(real(0), real(2), midZ), kHalfPi, gateHalf, 0);   // 终点线，往 +X
    AddCheckpoint(Vec3(midX, real(2), real(0)), kPi, gateHalf, 1);       // 右侧，往 -Z
    AddCheckpoint(Vec3(real(0), real(2), -midZ), -kHalfPi, gateHalf, 2); // 上方，往 -X
    AddCheckpoint(Vec3(-midX, real(2), real(0)), real(0), gateHalf, 3);  // 左侧，往 +Z

    //-- 发车：终点线后面一点，车头朝 +X -----------------------------------------
    const Vec3 start(real(-16), real(1.2), midZ);
    m_recoveryPosition = start;
    m_recoveryYaw = kHalfPi;

    VehicleConfig car;
    m_vehicle.Spawn(m_world, car, start, kHalfPi);

    //--------------------------------------------------------------------------
    // 车身和四个轮子也要登记进 刚体 -> 实体 表，否则渲染器问"这是什么"时
    // 得到 nullptr，车会被画成默认的护墙灰 —— 一辆灰色的车混在灰色的护墙里，
    // 这是第一版跑起来最先发现的问题。
    //--------------------------------------------------------------------------
    Entity carEntity;
    carEntity.kind = EntityKind::Car;
    carEntity.body = m_vehicle.ShellBody();  // 画面上的车是车壳，不是碰撞盒
    RegisterEntity(carEntity);

    for (int i = 0; i < Vehicle::kWheelCount; ++i) {
        Entity wheel;
        wheel.kind = EntityKind::Wheel;
        wheel.body = m_vehicle.WheelBody(i);
        RegisterEntity(wheel);
    }

    Log("LAP 1 OF " + std::to_string(m_config.totalLaps));
}

const Checkpoint& RaceGame::NextCheckpoint() const {
    const std::size_t index =
        static_cast<std::size_t>(m_nextCheckpoint) % m_checkpoints.size();
    return m_checkpoints[index];
}

//==============================================================================
// 每帧
//==============================================================================

void RaceGame::Update(const DriveInput& input, real dt) {
    if (m_state == RaceState::Finished) return;

    //--------------------------------------------------------------------------
    // 固定步长循环。**车必须每个子步都被施力**，理由见 Vehicle.h。
    //
    // 每次只喂引擎一个子步的时间，于是 `world.Step(h)` 内部恰好跑一步 ——
    // 施力和积分严格一一对应，帧率再抖，车的行为也完全一样。
    //--------------------------------------------------------------------------
    const real h = m_world.Config().fixedTimeStep;
    if (h <= real(0)) return;

    m_accumulator += dt;

    int steps = 0;
    constexpr int kMaxSteps = 8;
    while (m_accumulator >= h && steps < kMaxSteps) {
        DriveInput stepInput = input;
        if (m_state == RaceState::Countdown) {
            // 倒计时里锁油门，但方向盘可以先摆好 —— 和真实发车一样
            stepInput.throttle = real(0);
            stepInput.brake = real(0);
        }

        m_vehicle.Update(m_world, stepInput, h);
        m_world.Step(h);
        StepRace(h);

        m_accumulator -= h;
        ++steps;
    }
    // 死亡螺旋保险丝，和引擎里那条同一个理由（见 PhysicsWorld::Step）
    if (m_accumulator >= h) m_accumulator = real(0);

    CheckRecovery(dt);
}

void RaceGame::StepRace(real h) {
    switch (m_state) {
        case RaceState::Countdown:
            m_countdown -= h;
            if (m_countdown <= real(0)) {
                m_countdown = real(0);
                m_state = RaceState::Racing;
                m_lap = 1;
                Log("GO");
            }
            break;
        case RaceState::Racing:
            m_lapTime += h;
            m_totalTime += h;
            break;
        case RaceState::Finished:
            break;
    }
}

//------------------------------------------------------------------------------
// 掉出赛道 / 翻车之后自动放回
//
// 赛车游戏里这个功能不是"贴心"，是**必需**：翻在墙角的车是开不动的，没有它
// 玩家只能重启程序。放回的位置是最后一个合法通过的检查点 —— 放回起点会让
// 跑了大半圈的人直接崩溃，放回原地则会立刻再翻一次。
//------------------------------------------------------------------------------
void RaceGame::CheckRecovery(real dt) {
    if (m_state == RaceState::Finished) return;

    //--------------------------------------------------------------------------
    // 三种需要救援的情况，代价完全不同，所以判据也不同：
    //
    //   掉出世界   -> 立刻。再等下去只是看着它继续往下掉。
    //   翻车       -> 等 2.5 秒。翻过来又自己翻回去的情况是有的，别抢答。
    //   顶死不动   -> 等 4 秒。**这一条最容易被忘掉，但它才是最常发生的。**
    //                 车斜靠在护墙上、驱动轮全部离地时，油门是彻底没用的
    //                 （见 Vehicle.cpp：drivenGrounded == 0 就没有驱动力），
    //                 玩家会以为游戏卡死了。
    //--------------------------------------------------------------------------
    const bool fell = m_vehicle.Position().y < m_config.fallLimit;

    //--------------------------------------------------------------------------
    // "翻了"的判据是**车顶不再朝上**，不是"车顶朝下"。
    //
    // 只查完全翻过来（up.y < 0）会漏掉最常见的那一种：车斜撞护墙之后**竖起来
    // 靠在墙上**，车顶朝着侧面，up.y ≈ 0。那时四个轮子都悬空、油门完全没用，
    // 而判据说它没翻 —— 车就永远立在那儿。第一版就是这么卡死的。
    //--------------------------------------------------------------------------
    if (m_vehicle.Up().y < real(0.35)) {
        m_recoveryTimer += dt;
    } else {
        m_recoveryTimer = real(0);
    }

    //--------------------------------------------------------------------------
    // "卡住了"的判据是**位移**，不是速度。
    //
    // 顶在墙上的车会一直小幅抖动，瞬时速度时不时窜过阈值，把计时器清零 ——
    // 于是永远也判不出它卡住了。改成"四秒内挪没挪过五米"就完全不受抖动影响。
    //--------------------------------------------------------------------------
    if (m_state == RaceState::Racing) {
        m_stalledTimer += dt;
        if ((m_vehicle.Position() - m_stallReference).Length() > real(5)) {
            m_stallReference = m_vehicle.Position();
            m_stalledTimer = real(0);
        }
    } else {
        m_stallReference = m_vehicle.Position();
        m_stalledTimer = real(0);
    }

    if (fell || m_recoveryRequested || m_recoveryTimer >= m_config.recoveryDelay ||
        m_stalledTimer >= m_config.stalledDelay) {
        m_vehicle.Respawn(m_world, m_recoveryPosition + Vec3(0, real(1.2), 0),
                          m_recoveryYaw);
        m_recoveryTimer = real(0);
        m_stalledTimer = real(0);
        m_stallReference = m_recoveryPosition;
        m_recoveryRequested = false;
        Log("RECOVERED");
    }
}

//------------------------------------------------------------------------------
// 检查点
//------------------------------------------------------------------------------
void RaceGame::OnTriggerEnter(ColliderHandle trigger, ColliderHandle other) {
    const Collider* triggerCollider = m_world.GetCollider(trigger);
    const Collider* otherCollider = m_world.GetCollider(other);
    if (triggerCollider == nullptr || otherCollider == nullptr) return;
    // 只有车算数：路障箱子被撞进检查点不该算一圈
    if (otherCollider->body != m_vehicle.Body()) return;

    const Entity* entity = FindEntity(triggerCollider->body);
    if (entity == nullptr || entity->kind != EntityKind::Checkpoint) return;
    if (m_state != RaceState::Racing) return;

    //--------------------------------------------------------------------------
    // **必须按顺序过。** 不检查顺序的话，在终点线前后来回蹭就能刷圈数 ——
    // 这是所有赛车游戏都要防的第一件事。
    //--------------------------------------------------------------------------
    if (entity->checkpointIndex != m_nextCheckpoint) return;

    // 记下复位点：从这儿放回车是最合理的
    m_recoveryPosition = m_checkpoints[static_cast<std::size_t>(entity->checkpointIndex)].center;
    m_recoveryPosition.y = real(1.2);
    m_recoveryYaw = m_checkpoints[static_cast<std::size_t>(entity->checkpointIndex)].yaw;

    const int count = static_cast<int>(m_checkpoints.size());
    m_nextCheckpoint = (m_nextCheckpoint + 1) % count;

    if (m_nextCheckpoint != 1) {
        // 中途的检查点：只更新进度
        return;
    }

    //-- 回到 1 号意味着刚刚过了终点线：一圈完成 --------------------------------
    m_lastLap = m_lapTime;
    if (m_bestLap <= real(0) || m_lastLap < m_bestLap) m_bestLap = m_lastLap;

    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "LAP %d: %.2fS", m_lap, double(m_lastLap));
    Log(buffer);

    m_lapTime = real(0);
    if (m_lap >= m_config.totalLaps) {
        m_state = RaceState::Finished;
        Log("FINISH");
    } else {
        ++m_lap;
    }
}

void RaceGame::Log(const std::string& line) {
    m_log.push_back(line);
    if (m_log.size() > 5) m_log.erase(m_log.begin());
}

}  // namespace racing
