//==============================================================================
// game/Game.cpp
//==============================================================================

#include "game/Game.h"

#include <algorithm>

namespace game {

namespace {

WorldConfig MakeWorldConfig(const GameConfig& config) {
    WorldConfig world;
    world.gravity = config.gravity;
    world.cellSize = real(3);  // 地图物件大多是 2-6 米，格子取 3 米
    return world;
}

CharacterConfig MakeCharacterConfig(LayerMask blockedBy) {
    CharacterConfig c;
    c.radius = real(0.38);
    c.height = real(1.8);
    c.stepOffset = real(0.4);
    c.maxSlopeAngle = DegToRad(real(50));
    c.layerMask = blockedBy;
    return c;
}

}  // namespace

//==============================================================================
// 构造与关卡
//==============================================================================

Game::Game() : m_world(MakeWorldConfig(m_config)) {
    m_world.SetEventListener(this);
    m_health = m_config.playerMaxHealth;
    m_ammo = m_config.magazineSize;
    m_reserve = m_config.magazineSize * 4;
}

int Game::RegisterEntity(const Entity& entity) {
    const int index = static_cast<int>(m_entities.size());
    m_entities.push_back(entity);
    if (entity.body.IsValid()) {
        m_bodyToEntity[entity.body.index] = index;
    }
    return index;
}

const Entity* Game::FindEntity(BodyHandle body) const {
    if (!body.IsValid()) return nullptr;
    const auto it = m_bodyToEntity.find(body.index);
    if (it == m_bodyToEntity.end()) return nullptr;
    const Entity& entity = m_entities[static_cast<std::size_t>(it->second)];
    // 句柄的代际号对不上说明这个槽位已经被别人复用了
    return entity.body == body ? &entity : nullptr;
}

BodyHandle Game::AddStaticBox(const Vec3& center, const Vec3& halfExtents,
                              const Quat& rotation) {
    BodyDesc desc;
    desc.position = center;
    desc.rotation = rotation;
    desc.type = BodyType::Static;
    const BodyHandle body = m_world.CreateBody(desc);

    ColliderDesc collider;
    collider.shape = Shape::MakeBox(halfExtents);
    collider.material = Material::Wood();
    collider.layer = layers::kWorld;
    m_world.AddCollider(body, collider);

    Entity entity;
    entity.kind = EntityKind::World;
    entity.body = body;
    RegisterEntity(entity);
    return body;
}

void Game::AddCrate(const Vec3& center, real half, real mass) {
    BodyDesc desc;
    desc.position = center;
    desc.type = BodyType::Dynamic;
    desc.mass = mass;
    const BodyHandle body = m_world.CreateBody(desc);

    ColliderDesc collider;
    collider.shape = Shape::MakeCube(half);
    collider.material = Material::Wood();
    collider.layer = layers::kDefault;
    m_world.AddCollider(body, collider);

    Entity entity;
    entity.kind = EntityKind::Crate;
    entity.body = body;
    // 箱子也有"血"：打够了就碎。半边长要记下来，碎的时候按它切成八块。
    entity.health = m_config.crateHealth;
    entity.halfExtent = half;
    RegisterEntity(entity);
}

//------------------------------------------------------------------------------
// 人形碰撞体
//
//------------------------------------------------------------------------------
// 为什么"移动用胶囊、身体用人形"不是自相矛盾
//------------------------------------------------------------------------------
// 这两件事本来就是两个东西，只是以前恰好用了同一个形状：
//
//   - **移动**用的是 CharacterController 里的胶囊。它必须是胶囊：圆柱侧面才能
//     贴着墙滑、圆底才能顺着台阶和斜坡上去。换成人形立刻就会卡门框、卡楼梯 ——
//     这也是所有商业引擎的角色控制器至今都只给胶囊的原因。
//
//   - **世界看见的**是这个运动学替身刚体上挂的碰撞体。射线渲染、玩家开枪、
//     AI 视线判定全都问它。它没有任何理由必须是胶囊。
//
// 所以这里只换后者：走路还是胶囊在走，但**画出来、打得到**的是一个人形。
// 代价是身体比胶囊窄了一点，也就是说"看着擦身而过"和"实际擦身而过"更一致了。
//
// 局部坐标：原点是胶囊中心（脚底在 -height/2），**+X 是正面**，+Z 是左右
// （肩膀连线）。朝向由替身刚体的旋转给，见 UpdateEnemies。
//------------------------------------------------------------------------------
void Game::AddHumanoidColliders(BodyHandle body, real height) {
    const real half = height * real(0.5);  // 脚底在 y = -half

    struct Part {
        Vec3 center;       ///< 相对胶囊中心
        Vec3 halfExtents;  ///< 盒子的半边长；radius > 0 时改用球
        real radius;
    };

    // 按 1.8 米身高的常见比例摆的：腿占下半身、躯干到胸口、头一个球。
    // 数值直接写成绝对米数而不是身高的比例 —— 这里只有一种敌人，
    // 多写一层缩放只会让每个数字都要在脑子里再乘一遍。
    const Part parts[] = {
        // 头
        {Vec3(real(0.02), half - real(0.145), real(0)), Vec3::Zero(), real(0.135)},
        // 躯干：前后薄、左右宽 —— 这个比例是"人形"最主要的来源
        {Vec3(real(0), kHumanoidChestOffsetY, real(0)),
         Vec3(real(0.115), real(0.27), kHumanoidTorsoHalfWidth), real(0)},
        // 胯
        {Vec3(real(0), real(-0.03), real(0)), Vec3(real(0.105), real(0.09), real(0.15)),
         real(0)},
        // 两条腿：下端正好落在脚底。左右分得开一点 —— 贴在一起的两条腿在
        // 256x144 上就是一根柱子，那正是"胶囊"给人的印象
        {Vec3(real(0), -half + real(0.39), real(0.105)),
         Vec3(real(0.09), real(0.39), real(0.075)), real(0)},
        {Vec3(real(0), -half + real(0.39), real(-0.105)),
         Vec3(real(0.09), real(0.39), real(0.075)), real(0)},
        // 两条胳膊：挂在躯干两侧
        {Vec3(real(0), real(0.32), real(0.255)),
         Vec3(real(0.075), real(0.24), real(0.06)), real(0)},
        {Vec3(real(0), real(0.32), real(-0.255)),
         Vec3(real(0.075), real(0.24), real(0.06)), real(0)},
    };

    for (const Part& part : parts) {
        ColliderDesc collider;
        collider.shape = part.radius > real(0) ? Shape::MakeSphere(part.radius)
                                               : Shape::MakeBox(part.halfExtents);
        collider.localTransform = Transform(part.center, Quat::Identity());
        collider.material = Material(real(0.6), real(0));
        collider.layer = kLayerEnemy;
        m_world.AddCollider(body, collider);
    }
}

void Game::AddEnemy(const Vec3& footPosition) {
    //--------------------------------------------------------------------------
    // 敌人和玩家一样：CharacterController 负责移动，外加一个**运动学替身刚体**
    // 让世界能看见它（射线打得到、箱子推得动）。
    //
    // 敌人不做成动态刚体的理由和玩家完全一样（见 CharacterController.h）：
    // 会被箱子撞飞、爬不上台阶、贴墙抖。AI 的移动必须是可预测的。
    //--------------------------------------------------------------------------
    CharacterConfig cfg = MakeCharacterConfig(layers::kWorld | layers::kDefault);
    CharacterController controller(cfg);
    controller.SetFootPosition(footPosition);

    BodyDesc desc;
    desc.position = controller.position;
    desc.type = BodyType::Kinematic;
    const BodyHandle body = m_world.CreateBody(desc);

    // 替身刚体挂的是一副人形，不是一个胶囊 —— 走路的那个胶囊在 controller 里
    AddHumanoidColliders(body, cfg.height);

    Entity entity;
    entity.kind = EntityKind::Enemy;
    entity.body = body;
    entity.health = m_config.enemyHealth;
    entity.fireCooldown = real(0);
    // 出生时面朝场地中心：五个人都朝 +X 站着像列队，朝中间站才像在守点
    entity.facingYaw = Atan2(-footPosition.z, -footPosition.x);
    const int index = RegisterEntity(entity);

    m_enemyControllers.emplace(index, controller);
    ++m_enemyTotal;
}

void Game::AddPickup(const Vec3& center, PickupKind kind, real amount) {
    BodyDesc desc;
    desc.position = center;
    desc.type = BodyType::Static;
    const BodyHandle body = m_world.CreateBody(desc);

    ColliderDesc collider;
    collider.shape = Shape::MakeBox(Vec3(real(0.6), real(0.9), real(0.6)));
    collider.isTrigger = true;
    collider.layer = layers::kTrigger;
    m_world.AddCollider(body, collider);

    Entity entity;
    entity.kind = EntityKind::Pickup;
    entity.body = body;
    entity.pickup = kind;
    entity.amount = amount;
    RegisterEntity(entity);
}

void Game::SpawnPlayer(const Vec3& footPosition, real yaw) {
    CharacterConfig cfg =
        MakeCharacterConfig(layers::kWorld | layers::kDefault | kLayerEnemy);
    m_player = CharacterController(cfg);
    m_player.SetFootPosition(footPosition);
    m_yaw = yaw;

    BodyDesc desc;
    desc.position = m_player.position;
    desc.type = BodyType::Kinematic;
    m_playerBody = m_world.CreateBody(desc);

    ColliderDesc collider;
    collider.shape = cfg.MakeShape();
    collider.material = Material(real(0.6), real(0));
    collider.layer = layers::kPlayer;
    m_world.AddCollider(m_playerBody, collider);

    Entity entity;
    entity.kind = EntityKind::Player;
    entity.body = m_playerBody;
    RegisterEntity(entity);

    m_player.UpdateGrounded(m_world);
}

//------------------------------------------------------------------------------
// 关卡：一个两室一廊的小竞技场
//------------------------------------------------------------------------------
void Game::BuildLevel() {
    //--------------------------------------------------------------------------
    // 关卡尺度是**为渲染器设计的**。
    //
    // 第一版做了个 52×36 米的大广场，画出来整个屏幕四分之三是地板 —— 几何上完全
    // 正确，但看着像在停机坪上走。光线投射的画面好不好看，取决于"视线多久会撞到
    // 一堵墙"：走廊和小房间会让墙填满画面，空旷的场地不会。
    //
    // 所以改成 34×26 米、被隔墙切成四个房间 + 中央走廊，并且**加了天花板**
    // （第一版忘了，抬头是一片空白）。
    //--------------------------------------------------------------------------
    const real wallHeight = real(3.2);
    const real halfWall = wallHeight * real(0.5);
    const real ex = real(17);  // 场地半宽
    const real ez = real(13);  // 场地半深

    // 地面与天花板
    AddStaticBox(Vec3(0, real(-0.5), 0), Vec3(ex, real(0.5), ez));
    AddStaticBox(Vec3(0, wallHeight + real(0.5), 0), Vec3(ex, real(0.5), ez));

    // 外墙
    AddStaticBox(Vec3(0, halfWall, -ez - real(0.4)), Vec3(ex, halfWall, real(0.4)));
    AddStaticBox(Vec3(0, halfWall, ez + real(0.4)), Vec3(ex, halfWall, real(0.4)));
    AddStaticBox(Vec3(-ex - real(0.4), halfWall, 0), Vec3(real(0.4), halfWall, ez));
    AddStaticBox(Vec3(ex + real(0.4), halfWall, 0), Vec3(real(0.4), halfWall, ez));

    //--------------------------------------------------------------------------
    // 十字隔墙，四个方向各留一个门洞 —— 门洞是玩法的核心：
    // 敌人会从门洞里出现，玩家可以卡住门口打
    //--------------------------------------------------------------------------
    // 竖墙（沿 Z），中间留门
    AddStaticBox(Vec3(0, halfWall, real(-8.5)), Vec3(real(0.4), halfWall, real(4.5)));
    AddStaticBox(Vec3(0, halfWall, real(8.5)), Vec3(real(0.4), halfWall, real(4.5)));
    // 横墙（沿 X），左右各留门
    AddStaticBox(Vec3(real(-11.5), halfWall, 0), Vec3(real(5.5), halfWall, real(0.4)));
    AddStaticBox(Vec3(real(11.5), halfWall, 0), Vec3(real(5.5), halfWall, real(0.4)));

    // 柱子：既是掩体，也让画面里有近处的参照物（不然墙都在远处，没有纵深感）
    AddStaticBox(Vec3(real(-8), real(1.6), real(-6)), Vec3(real(0.7), real(1.6), real(0.7)));
    AddStaticBox(Vec3(real(8), real(1.6), real(6)), Vec3(real(0.7), real(1.6), real(0.7)));
    AddStaticBox(Vec3(real(-8), real(1.6), real(6)), Vec3(real(0.7), real(1.6), real(0.7)));
    AddStaticBox(Vec3(real(8), real(1.6), real(-6)), Vec3(real(0.7), real(1.6), real(0.7)));

    // 半人高的矮掩体：能挡住敌人的下半身，但挡不住头 —— 逐像素射线才画得出这种
    // 高度差，经典的"每列一根射线"渲染器只能画等高的墙
    AddStaticBox(Vec3(real(-13), real(0.6), real(-4)), Vec3(real(2.5), real(0.6), real(0.4)));
    AddStaticBox(Vec3(real(13), real(0.6), real(4)), Vec3(real(2.5), real(0.6), real(0.4)));

    // 通往高台的台阶
    for (int i = 0; i < 4; ++i) {
        const real h = real(0.22) * real(i + 1);
        AddStaticBox(Vec3(real(9) + real(i) * real(1.1), h, real(-11)),
                     Vec3(real(0.55), h, real(2.2)));
    }
    AddStaticBox(Vec3(real(14), real(0.88), real(-11)), Vec3(real(2.5), real(0.88), real(2.2)));

    // 可推动、可击飞的箱子（其中三个叠成一摞，看堆叠稳不稳）
    AddCrate(Vec3(real(-4.5), real(0.45), real(-4.5)), real(0.45), real(12));
    AddCrate(Vec3(real(-3.5), real(0.45), real(-5.2)), real(0.45), real(12));
    AddCrate(Vec3(real(-4.0), real(1.35), real(-4.8)), real(0.45), real(12));
    AddCrate(Vec3(real(5), real(0.45), real(4)), real(0.45), real(12));
    AddCrate(Vec3(real(6), real(0.45), real(4.6)), real(0.45), real(12));
    AddCrate(Vec3(real(-12), real(0.45), real(9)), real(0.45), real(12));

    // 补给
    AddPickup(Vec3(real(-14), real(0.9), real(-10)), PickupKind::Health, real(35));
    AddPickup(Vec3(real(14), real(0.9), real(10)), PickupKind::Ammo,
              real(m_config.magazineSize * 3));
    AddPickup(Vec3(real(-14), real(0.9), real(10)), PickupKind::Ammo,
              real(m_config.magazineSize * 2));

    //--------------------------------------------------------------------------
    // 敌人分散在另外三个房间里。
    //
    // **刻意不让任何一个敌人在出生点就能看见玩家** —— 第一版把敌人放在了同一个
    // 房间里，玩家还没转过身就先掉了 18 点血，而且开火的敌人根本不在视野里。
    // 玩家得先看见威胁，才轮到威胁看见玩家。
    //--------------------------------------------------------------------------
    AddEnemy(Vec3(real(-11), real(0.05), real(8)));
    AddEnemy(Vec3(real(6), real(0.05), real(-8)));
    AddEnemy(Vec3(real(12), real(0.05), real(7)));
    AddEnemy(Vec3(real(13), real(0.05), real(-3)));
    AddEnemy(Vec3(real(3), real(0.05), real(9)));

    // 玩家出生在西南房间的角落，面朝场地中心
    SpawnPlayer(Vec3(real(-14), real(0.05), real(-8)), DegToRad(real(20)));

    Log("OBJECTIVE: eliminate all 5 enemies");
}

//==============================================================================
// 每帧
//==============================================================================

Vec3 Game::EyePosition() const noexcept {
    return m_player.FootPosition() + Vec3(0, m_config.eyeHeight, 0);
}

Vec3 Game::MuzzlePosition() const noexcept {
    //--------------------------------------------------------------------------
    // 枪口在眼睛的右下前方一点。
    //
    // 弹道本身是从**眼睛**出发的（准星在哪就打哪，这一点不能动），
    // 只有画出来的曳光线从这里起笔。两者错开一点点，画面上才有"枪在手里"的感觉；
    // 完全从眼睛出发的话，那条线看起来像是从脑门里射出去的。
    //--------------------------------------------------------------------------
    const real cp = Cos(m_pitch);
    const Vec3 forward(Cos(m_yaw) * cp, Sin(m_pitch), Sin(m_yaw) * cp);
    const Vec3 right(-Sin(m_yaw), real(0), Cos(m_yaw));
    const Vec3 up = Cross(right, forward).Normalized();
    return EyePosition() + right * real(0.16) - up * real(0.14) + forward * real(0.35);
}

void Game::Update(const PlayerIntent& intent, real dt) {
    if (m_state != GameState::Playing) return;

    m_time += dt;
    m_muzzleFlash = Max(real(0), m_muzzleFlash - dt * real(6));
    m_damageFlash = Max(real(0), m_damageFlash - dt * real(2));
    m_hitMarker = Max(real(0), m_hitMarker - dt);

    //--------------------------------------------------------------------------
    // 顺序：先推进物理世界（箱子、堆叠），再驱动角色。
    // 角色不参与求解，所以它读到的是这一步之后的世界状态。
    //--------------------------------------------------------------------------
    m_world.Step(dt);

    UpdatePlayer(intent, dt);
    UpdateWeapon(intent, dt);
    UpdateEnemies(dt);
    UpdatePickups(dt);
    UpdateDebris(dt);
    UpdateEffects(dt);

    // 胜负判定
    if (m_health <= real(0)) {
        m_state = GameState::Lost;
        Log("YOU ARE DOWN");
    } else if (EnemiesAlive() == 0) {
        m_state = GameState::Won;
        Log("ALL ENEMIES ELIMINATED");
    }
}

void Game::SyncProxyBody(BodyHandle body, const CharacterController& controller,
                         const Quat& rotation) {
    // 把控制器的位姿和速度抄给替身刚体。速度也要抄 —— 求解器靠它算接触点的
    // 相对速度，不抄的话角色推箱子推不出应有的力道。
    //
    // 旋转是**替身自己的**：控制器的胶囊轴对称，它根本不记朝向（GetTransform()
    // 永远返回单位四元数）。人形身体要朝哪边，是游戏这一层的事。
    m_world.SetBodyTransform(body, Transform(controller.position, rotation));
    if (RigidBody* rb = m_world.GetBody(body)) {
        rb->linearVelocity = controller.velocity;
    }
}

void Game::UpdatePlayer(const PlayerIntent& intent, real dt) {
    // 键盘的 turn/look 要乘 dt（"按住的比例"），鼠标的 delta 直接叠加
    m_yaw += intent.turn * m_config.turnSpeed * dt + intent.turnDelta;
    m_pitch = Clamp(
        m_pitch + intent.look * m_config.turnSpeed * real(0.6) * dt + intent.lookDelta,
        DegToRad(real(-70)), DegToRad(real(70)));

    // 朝向决定移动方向：这就是 FPS 的"相对相机移动"
    const Vec3 forward(Cos(m_yaw), real(0), Sin(m_yaw));
    const Vec3 right(-Sin(m_yaw), real(0), Cos(m_yaw));

    Vec3 wish = forward * intent.forward + right * intent.strafe;
    if (wish.LengthSq() > real(1e-6)) {
        // 归一化之后再乘速度：不然斜着走会比直走快 sqrt(2) 倍（经典的斜走加速 bug）
        wish = wish.Normalized() *
               (m_config.moveSpeed * (intent.sprint ? m_config.sprintMultiplier : real(1)));
    }

    if (intent.jump) m_player.Jump(m_config.jumpSpeed);
    m_player.Update(m_world, wish, dt, m_config.gravity);

    SyncProxyBody(m_playerBody, m_player);
}

//------------------------------------------------------------------------------
// 武器
//------------------------------------------------------------------------------

void Game::UpdateWeapon(const PlayerIntent& intent, real dt) {
    m_fireTimer = Max(real(0), m_fireTimer - dt);

    if (m_reloadTimer > real(0)) {
        m_reloadTimer -= dt;
        if (m_reloadTimer <= real(0)) {
            const int need = m_config.magazineSize - m_ammo;
            const int take = Min(need, m_reserve);
            m_ammo += take;
            m_reserve -= take;
            Log("Reloaded");
        }
        return;
    }

    const bool wantReload =
        intent.reload || (intent.fire && m_ammo == 0 && m_reserve > 0);
    if (wantReload && m_ammo < m_config.magazineSize && m_reserve > 0) {
        m_reloadTimer = m_config.reloadTime;
        Log("Reloading...");
        return;
    }

    if (intent.fire && m_fireTimer <= real(0) && m_ammo > 0) {
        FirePlayerWeapon();
        --m_ammo;
        m_fireTimer = m_config.fireInterval;
        m_muzzleFlash = real(1);
    }
}

void Game::FirePlayerWeapon() {
    //--------------------------------------------------------------------------
    // hitscan：一条射线定生死。
    //
    // 层掩码屏蔽掉 kPlayer，否则射线起点就在自己的胶囊里，第一个命中永远是自己。
    // FPS 里"子弹不会打到自己"就是这一行。
    //--------------------------------------------------------------------------
    const real cp = Cos(m_pitch);
    const Vec3 dir(Cos(m_yaw) * cp, Sin(m_pitch), Sin(m_yaw) * cp);

    const Vec3 muzzle = MuzzlePosition();
    const Ray ray(EyePosition(), dir, m_config.weaponRange);
    WorldRaycastHit hit;
    if (!m_world.Raycast(ray, hit, kPlayerShootMask)) {
        // 打空了也要画曳光弹 —— 玩家得看得见"这一枪飞哪儿去了"，
        // 不然打不中的时候完全没有可以修正的线索。
        SpawnTracer(muzzle, EyePosition() + dir * m_config.weaponRange, false);
        Log("Miss");
        return;
    }

    SpawnTracer(muzzle, hit.point, false);

    const auto it = m_bodyToEntity.find(hit.body.index);
    const int index = (it != m_bodyToEntity.end()) ? it->second : -1;
    const Entity* target = FindEntity(hit.body);
    SpawnImpact(hit.point, hit.normal,
                target != nullptr ? target->kind : EntityKind::World);
    if (target == nullptr || index < 0) return;

    switch (target->kind) {
        case EntityKind::Enemy: {
            Entity& enemy = m_entities[static_cast<std::size_t>(index)];
            if (!enemy.alive) break;

            enemy.health -= m_config.weaponDamage;
            enemy.alerted = true;
            enemy.lastKnownPlayerPos = EyePosition();
            enemy.lastSeenTimer = m_config.enemyMemory;
            m_hitMarker = m_config.hitMarkerLifetime;

            if (enemy.health <= real(0)) {
                KillEnemy(index, dir);
                Log("KILL!  " + std::to_string(EnemiesAlive()) + " left");
            } else {
                Log("Hit  " + std::to_string(static_cast<int>(hit.distance)) + "m");
            }
            break;
        }
        case EntityKind::Crate: {
            // 打箱子：给一个冲量。这是引擎里 ApplyImpulseAtPoint 的直接用法 ——
            // 打在偏离质心的地方，箱子会跟着转起来。
            if (RigidBody* rb = m_world.GetBody(hit.body)) {
                rb->WakeUp();
                rb->ApplyImpulseAtPoint(dir * m_config.weaponImpulse, hit.point);
            }

            Entity& crate = m_entities[static_cast<std::size_t>(index)];
            if (!crate.alive) break;
            crate.health -= m_config.weaponDamage;
            if (crate.health <= real(0)) {
                // 注意传下标而不是引用：ShatterCrate 里会往 m_entities 追加碎块，
                // 这里的 crate 引用在那之后就是野的了
                ShatterCrate(index, dir);
                m_hitMarker = m_config.hitMarkerLifetime;
                Log("CRATE DESTROYED");
            }
            break;
        }
        default:
            break;
    }
}

//------------------------------------------------------------------------------
// 命中反馈
//
// 这一节做的全是"让玩家知道刚才发生了什么"。它们不改变任何数值，但没有它们，
// hitscan 武器打出去是**完全没有反馈**的：子弹瞬时到达、没有实体、
// 敌人掉血也只是屏幕角落一个数字在跳。玩家的主观感受是"枪好像坏了"。
//------------------------------------------------------------------------------

void Game::SpawnTracer(const Vec3& from, const Vec3& to, bool hostile) {
    Tracer tracer;
    tracer.from = from;
    tracer.to = to;
    tracer.life = m_config.tracerLifetime;
    tracer.maxLife = m_config.tracerLifetime;
    tracer.hostile = hostile;
    m_tracers.push_back(tracer);
}

void Game::SpawnImpact(const Vec3& position, const Vec3& normal, EntityKind target) {
    Impact impact;
    // 火花沿法线往外挪一点点，免得和被打中的那个面 z-fighting，
    // 一半的火花被墙"吃掉"看起来像闪烁
    impact.position = position + normal * real(0.02);
    impact.normal = normal;
    impact.life = m_config.impactLifetime;
    impact.maxLife = m_config.impactLifetime;
    impact.target = target;
    m_impacts.push_back(impact);
}

void Game::UpdateEffects(real dt) {
    const auto expire = [dt](auto& list) {
        for (auto& effect : list) effect.life -= dt;
        list.erase(std::remove_if(list.begin(), list.end(),
                                  [](const auto& e) { return e.life <= real(0); }),
                   list.end());
    };
    expire(m_tracers);
    expire(m_impacts);
}

//------------------------------------------------------------------------------
// 破碎
//
// 一个箱子被打碎 = 把它从世界里删掉，换成八个小刚体。**没有任何特殊的
// "破碎系统"**：碎块就是普通的动态刚体，会飞、会翻滚、会互相碰、会叠在地上，
// 全部由求解器负责。这也是为什么它们看起来是对的 —— 它们本来就是真的。
//
// 唯一需要自己管的是"别让碎块无限堆积"：每块有寿命，总数有预算。
//------------------------------------------------------------------------------

void Game::ShatterCrate(int entityIndex, const Vec3& impactDirection) {
    //--------------------------------------------------------------------------
    // 先把要用的东西全部**按值**取出来。下面 SpawnDebrisPiece 会往 m_entities
    // 里 push_back，vector 一扩容，任何指向它的引用/指针就全废了。
    //--------------------------------------------------------------------------
    const BodyHandle body = m_entities[static_cast<std::size_t>(entityIndex)].body;
    const real half = m_entities[static_cast<std::size_t>(entityIndex)].halfExtent;
    const Transform transform = m_world.GetBodyTransform(body);

    Vec3 inherited = Vec3::Zero();
    if (const RigidBody* rb = m_world.GetBody(body)) inherited = rb->linearVelocity;

    m_entities[static_cast<std::size_t>(entityIndex)].alive = false;
    DestroyEntityBody(entityIndex);

    //--------------------------------------------------------------------------
    // 2x2x2 切八块：每块的中心就是原立方体八分之一的中心。
    // 这么切的好处是碎块合起来正好填满原来的体积 —— 碎的瞬间不会"变多"或"变少"，
    // 观感上就是这个箱子裂开了，而不是凭空冒出一堆东西。
    //--------------------------------------------------------------------------
    const real childHalf = half * real(0.5);
    for (int i = 0; i < 8; ++i) {
        const Vec3 offset((i & 1) ? childHalf : -childHalf,
                          (i & 2) ? childHalf : -childHalf,
                          (i & 4) ? childHalf : -childHalf);
        const Vec3 center = transform.position + transform.rotation.Rotate(offset);

        // 速度 = 原箱子的速度 + 从中心向外炸开 + 子弹推着走
        const Vec3 outward = offset.Normalized();
        const Vec3 velocity = inherited +
                              outward * (m_config.debrisScatterSpeed *
                                         (real(0.6) + Random01() * real(0.8))) +
                              impactDirection * m_config.debrisBlastSpeed;
        SpawnDebrisPiece(center, childHalf, DebrisKind::Wood, velocity);
    }
}

void Game::KillEnemy(int entityIndex, const Vec3& impactDirection) {
    const std::size_t slot = static_cast<std::size_t>(entityIndex);

    Vec3 center = Vec3::Zero();
    const auto ctrl = m_enemyControllers.find(entityIndex);
    if (ctrl != m_enemyControllers.end()) center = ctrl->second.position;

    m_entities[slot].alive = false;
    // 尸体从世界里移除：射线不再打到它，也不再挡路
    DestroyEntityBody(entityIndex);
    m_enemyControllers.erase(entityIndex);

    // 一小把碎块代替尸体。它同时是**击杀确认** —— 敌人凭空消失的话，
    // 玩家会怀疑自己是不是打死了它。
    for (int i = 0; i < 6; ++i) {
        const Vec3 spread = RandomDirection();
        const Vec3 velocity = spread * (m_config.debrisScatterSpeed * real(0.8)) +
                              Vec3(0, real(2.5), 0) +
                              impactDirection * (m_config.debrisBlastSpeed * real(0.5));
        SpawnDebrisPiece(center + spread * real(0.3), real(0.11), DebrisKind::Flesh,
                         velocity);
    }
}

void Game::SpawnDebrisPiece(const Vec3& center, real half, DebrisKind kind,
                            const Vec3& velocity) {
    if (m_debrisAlive >= m_config.debrisBudget) RetireOldestDebris();

    BodyDesc desc;
    desc.position = center;
    desc.type = BodyType::Dynamic;
    // 让引擎按形状和密度自己算质量（mass <= 0 就是这个意思）：
    // 小块自然就轻，弹起来的感觉不用调
    desc.mass = real(-1);
    desc.linearVelocity = velocity;
    desc.angularVelocity = RandomDirection() * m_config.debrisSpin;
    const BodyHandle body = m_world.CreateBody(desc);

    ColliderDesc collider;
    collider.shape = Shape::MakeCube(half);
    collider.material = Material::Wood();
    collider.layer = kLayerDebris;  // 看得见、但不挡路不挡枪，见 GameTypes.h
    m_world.AddCollider(body, collider);

    Entity entity;
    entity.kind = EntityKind::Debris;
    entity.body = body;
    entity.debrisKind = kind;
    entity.halfExtent = half;
    entity.life = m_config.debrisLifetime;
    RegisterEntity(entity);
    ++m_debrisAlive;
}

void Game::RetireOldestDebris() {
    // m_entities 是按创建顺序追加的，所以第一个还活着的碎块就是最老的那个
    for (std::size_t i = 0; i < m_entities.size(); ++i) {
        Entity& entity = m_entities[i];
        if (entity.kind != EntityKind::Debris || !entity.alive) continue;
        entity.alive = false;
        DestroyEntityBody(static_cast<int>(i));
        --m_debrisAlive;
        return;
    }
}

void Game::UpdateDebris(real dt) {
    for (std::size_t i = 0; i < m_entities.size(); ++i) {
        Entity& entity = m_entities[i];
        if (entity.kind != EntityKind::Debris || !entity.alive) continue;

        entity.life -= dt;
        if (entity.life > real(0)) continue;

        entity.alive = false;
        DestroyEntityBody(static_cast<int>(i));
        --m_debrisAlive;
    }
}

void Game::DestroyEntityBody(int entityIndex) {
    Entity& entity = m_entities[static_cast<std::size_t>(entityIndex)];
    if (!entity.body.IsValid()) return;

    //--------------------------------------------------------------------------
    // 反查表只用 index 做键，而 index 会被下一个创建的刚体复用。所以这里
    // **必须确认这一条还是指向自己**再删 —— 否则会把后来者的条目误删掉，
    // 症状是"打碎箱子之后，某个碎块渲染成了墙的颜色"。
    //--------------------------------------------------------------------------
    const auto it = m_bodyToEntity.find(entity.body.index);
    if (it != m_bodyToEntity.end() && it->second == entityIndex) {
        m_bodyToEntity.erase(it);
    }
    m_world.DestroyBody(entity.body);
    entity.body = BodyHandle::Null();
}

real Game::Random01() {
    // 经典的 LCG。要的不是统计学上的好随机，而是**可复现** ——
    // 机器人模式的冒烟测试必须每次跑出同一场画面。
    m_rng = m_rng * 1664525u + 1013904223u;
    return static_cast<real>((m_rng >> 8) & 0xFFFFFFu) / static_cast<real>(0x1000000u);
}

Vec3 Game::RandomDirection() {
    const Vec3 v(Random01() * real(2) - real(1), Random01() * real(2) - real(1),
                 Random01() * real(2) - real(1));
    const real length = v.Length();
    return length > real(1e-4) ? v * (real(1) / length) : Vec3(0, real(1), 0);
}

//------------------------------------------------------------------------------
// 敌人 AI
//------------------------------------------------------------------------------

bool Game::HasLineOfSight(const Vec3& from, const Vec3& to, LayerMask mask,
                          BodyHandle expected) const {
    // 视线遮挡判定的标准写法：从 A 打一条到 B 的射线（射程恰好是两点距离），
    // 看**第一个**命中是不是 B。中间挡着墙或者箱子就说明看不见。
    const Ray ray = Ray::FromTo(from, to);
    WorldRaycastHit hit;
    if (!m_world.Raycast(ray, hit, mask)) return false;
    return hit.body == expected;
}

void Game::UpdateEnemies(real dt) {
    const Vec3 playerEye = EyePosition();

    for (std::size_t i = 0; i < m_entities.size(); ++i) {
        Entity& enemy = m_entities[i];
        if (enemy.kind != EntityKind::Enemy || !enemy.alive) continue;

        const auto ctrlIt = m_enemyControllers.find(static_cast<int>(i));
        if (ctrlIt == m_enemyControllers.end()) continue;
        CharacterController& controller = ctrlIt->second;

        enemy.fireCooldown = Max(real(0), enemy.fireCooldown - dt);

        const Vec3 enemyEye =
            controller.FootPosition() + Vec3(0, m_config.eyeHeight, 0);
        const real distance = (playerEye - enemyEye).Length();

        //----------------------------------------------------------------------
        // 视线判定
        //----------------------------------------------------------------------
        bool canSee = false;
        if (distance < m_config.enemySightRange) {
            canSee = HasLineOfSight(enemyEye, playerEye, kEnemyShootMask, m_playerBody);
        }

        if (canSee) {
            if (!enemy.alerted) {
                // 刚发现玩家：先愣一下再开枪（见 GameConfig::enemyReactionTime）
                enemy.reactionTimer = m_config.enemyReactionTime;
            }
            enemy.alerted = true;
            enemy.lastSeenTimer = m_config.enemyMemory;
            enemy.lastKnownPlayerPos = m_player.position;
        } else {
            enemy.lastSeenTimer = Max(real(0), enemy.lastSeenTimer - dt);
        }

        //----------------------------------------------------------------------
        // 移动：朝最后已知的位置走，走到 preferredRange 就停下
        //
        // 这里没有寻路 —— 敌人直着走，撞墙就靠 collide-and-slide 蹭过去。
        // 对这个规模的地图够用；真要做绕柱子包抄得上导航网格（见下面的注释）。
        //----------------------------------------------------------------------
        Vec3 wish = Vec3::Zero();
        if (enemy.alerted && enemy.lastSeenTimer > real(0)) {
            Vec3 toTarget = enemy.lastKnownPlayerPos - controller.position;
            toTarget.y = real(0);
            const real planarDistance = toTarget.Length();
            if (planarDistance > m_config.enemyPreferredRange) {
                wish = toTarget * (m_config.enemySpeed / planarDistance);
            }
        }

        controller.Update(m_world, wish, dt, m_config.gravity);

        //----------------------------------------------------------------------
        // 朝向：警觉了就转向玩家，否则朝着走的方向。
        //
        // 限速转身而不是瞬间对齐：瞬间对齐看着像一张贴纸在原地翻面，而且玩家
        // 来不及读到"它正在转向我"这个信号 —— 那是从掩体后探头时唯一的预警。
        //----------------------------------------------------------------------
        Vec3 facing = (enemy.alerted && enemy.lastSeenTimer > real(0))
                          ? enemy.lastKnownPlayerPos - controller.position
                          : wish;
        facing.y = real(0);
        if (facing.LengthSq() > real(1e-6)) {
            real diff = Atan2(facing.z, facing.x) - enemy.facingYaw;
            // 绕到 [-pi, pi]，否则从 179 度转到 -179 度会绕一大圈
            while (diff > kPi) diff -= kTwoPi;
            while (diff < -kPi) diff += kTwoPi;
            const real step = m_config.enemyTurnSpeed * dt;
            enemy.facingYaw += Clamp(diff, -step, step);
        }

        // 游戏的 yaw 约定是 0 朝 +X、逆时针为正，而绕 +Y 的正向旋转把 +X 转向
        // -Z —— 差一个负号。写反的症状是敌人永远背对着你。
        SyncProxyBody(enemy.body, controller,
                      Quat::FromAxisAngle(Vec3(0, real(1), 0), -enemy.facingYaw));

        //----------------------------------------------------------------------
        // 开火：只在真正看得见的时候
        //----------------------------------------------------------------------
        //------------------------------------------------------------------
        // 开火：只在真正看得见的时候，而且**第一次发现玩家要愣一下**。
        //
        // 没有这个反应时间的话，敌人一进视线就是瞬发命中 —— 玩家从拐角
        // 探头会毫无预兆地掉血，主观感受是"被作弊了"。给 reactionTime 之后
        // 玩家有机会先开枪或者缩回掩体，这是 FPS 的基本公平性。
        //------------------------------------------------------------------
        enemy.reactionTimer = Max(real(0), enemy.reactionTimer - dt);
        if (canSee && enemy.reactionTimer <= real(0)) {
            if (enemy.fireCooldown <= real(0)) {
                EnemyFire(enemy, controller);
                enemy.fireCooldown = m_config.enemyFireInterval;
            }
        }
    }
}

void Game::EnemyFire(Entity& enemy, const CharacterController& controller) {
    (void)enemy;
    const Vec3 from = controller.FootPosition() + Vec3(0, m_config.eyeHeight, 0);
    const Ray ray = Ray::FromTo(from, EyePosition());

    WorldRaycastHit hit;
    if (!m_world.Raycast(ray, hit, kEnemyShootMask)) return;

    //--------------------------------------------------------------------------
    // 敌人的子弹也画曳光弹。这不是装饰：第一人称视野只有 60 度，
    // 大部分火力是从画面外打过来的，**红色的弹道是玩家判断"敌人在哪个方向"的
    // 唯一线索**。没有它，掉血就只是莫名其妙的一件事。
    //
    // 终点收在离眼睛半米的地方 —— 一直画到眼球会让那条线糊满整个屏幕。
    //--------------------------------------------------------------------------
    const Vec3 toEye = EyePosition() - from;
    const real distance = toEye.Length();
    const Vec3 direction = distance > real(1e-4) ? toEye * (real(1) / distance)
                                                 : Vec3(0, 0, real(1));
    const bool blocked = hit.body != m_playerBody;
    SpawnTracer(from, blocked ? hit.point : EyePosition() - direction * real(0.5), true);
    if (blocked) {
        // 打在掩体上：火花能让玩家看见"这堵墙正在替我挨枪"
        SpawnImpact(hit.point, hit.normal, EntityKind::World);
        return;
    }

    Damage(m_config.enemyDamage, "敌人");
}

//------------------------------------------------------------------------------
// 拾取物
//------------------------------------------------------------------------------

void Game::OnTriggerEnter(ColliderHandle trigger, ColliderHandle other) {
    //--------------------------------------------------------------------------
    // 这个回调是在 `Step()` 的**最末尾**被引擎调用的，所以在这里销毁刚体是安全的
    // —— 那正是引擎把事件做成队列而不是即时回调的原因（见 Events.h）。
    //--------------------------------------------------------------------------
    const Collider* triggerCollider = m_world.GetCollider(trigger);
    const Collider* otherCollider = m_world.GetCollider(other);
    if (triggerCollider == nullptr || otherCollider == nullptr) return;

    // 只有玩家能捡
    if (otherCollider->body != m_playerBody) return;

    const auto it = m_bodyToEntity.find(triggerCollider->body.index);
    if (it == m_bodyToEntity.end()) return;

    Entity& pickup = m_entities[static_cast<std::size_t>(it->second)];
    if (pickup.kind != EntityKind::Pickup || !pickup.alive) return;

    if (pickup.pickup == PickupKind::Health) {
        if (m_health >= m_config.playerMaxHealth) return;  // 满血就不捡
        m_health = Min(m_config.playerMaxHealth, m_health + pickup.amount);
        Log("+" + std::to_string(static_cast<int>(pickup.amount)) + " HEALTH");
    } else {
        m_reserve += static_cast<int>(pickup.amount);
        Log("+" + std::to_string(static_cast<int>(pickup.amount)) + " AMMO");
    }

    pickup.alive = false;
    pickup.respawnTimer = real(20);
    //--------------------------------------------------------------------------
    // 在触发器回调里销毁刚体。引擎保证这是安全的（事件是在 Step 末尾统一派发的），
    // 但这条路径曾经**必崩** —— 销毁碰撞体会往正在派发的事件队列里补发 Exit，
    // 而当时的派发循环是 range-for，队列一扩容就在野指针上继续走。
    // 修在 src/scene/TriggerSystem.cpp，回归测试见 tests/test_world.cpp。
    //--------------------------------------------------------------------------
    DestroyEntityBody(it->second);
}

void Game::UpdatePickups(real dt) {
    for (Entity& entity : m_entities) {
        if (entity.kind != EntityKind::Pickup || entity.alive) continue;
        entity.respawnTimer -= dt;
        // 重生留给将来 —— 现在拿走就没了，先不刷新
    }
}

//------------------------------------------------------------------------------
// 杂项
//------------------------------------------------------------------------------

void Game::Damage(real amount, const char* source) {
    m_health = Max(real(0), m_health - amount);
    m_damageFlash = real(1);
    (void)source;
}

std::vector<Game::EnemyView> Game::AliveEnemyViews() const {
    std::vector<EnemyView> views;
    const Vec3 eye = EyePosition();

    for (std::size_t i = 0; i < m_entities.size(); ++i) {
        const Entity& entity = m_entities[i];
        if (entity.kind != EntityKind::Enemy || !entity.alive) continue;

        const auto it = m_enemyControllers.find(static_cast<int>(i));
        if (it == m_enemyControllers.end()) continue;

        EnemyView view;
        view.position = it->second.position;
        view.eye = it->second.FootPosition() + Vec3(0, m_config.eyeHeight, 0);
        view.aim = it->second.position + Vec3(0, kHumanoidChestOffsetY, 0);
        view.body = entity.body;
        view.distance = (view.aim - eye).Length();
        // 复用和敌人 AI 完全同一套视线判定 —— 保证"机器人能打到的"
        // 和"玩家能打到的"是同一件事
        view.visible = HasLineOfSight(eye, view.aim, kPlayerShootMask, entity.body);
        views.push_back(view);
    }
    return views;
}

int Game::EnemiesAlive() const noexcept {
    int count = 0;
    for (const Entity& entity : m_entities) {
        if (entity.kind == EntityKind::Enemy && entity.alive) ++count;
    }
    return count;
}

void Game::Log(const std::string& line) {
    m_log.push_back(line);
    // 只留最近几条，免得无限增长
    if (m_log.size() > 6) m_log.erase(m_log.begin());
}

}  // namespace game
