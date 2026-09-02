#pragma once
//==============================================================================
// game/Game.h
//
// 游戏状态与玩法逻辑。
//
//------------------------------------------------------------------------------
// 引擎与游戏的分界线在哪儿
//------------------------------------------------------------------------------
// 引擎负责："这条射线打到了哪个刚体""这个胶囊能往前走多远""这两个箱子怎么弹开"。
// 游戏负责："打到的那个刚体是不是敌人""扣多少血""敌人现在该不该开火"。
//
// 具体到代码上，这个类做的事只有三件：
//   1. 维护 刚体句柄 -> 实体 的映射（引擎不知道什么叫"敌人"）
//   2. 每帧驱动玩家和敌人的**意图**（往哪走、开不开枪），
//      移动本身交给 CharacterController
//   3. 把引擎吐出来的事件（触发器）翻译成玩法（补血、补弹药）
//==============================================================================

#include <string>
#include <unordered_map>
#include <vector>

#include "game/GameTypes.h"

namespace game {

//------------------------------------------------------------------------------
// 玩家意图（由输入层填，游戏层消费）
//
// 刻意不是"按键状态"而是"想干什么"：这样换成手柄、网络指令、或者 AI 托管，
// 游戏逻辑一行都不用改。
//------------------------------------------------------------------------------
struct PlayerIntent {
    real forward = real(0);  ///< [-1, 1]
    real strafe = real(0);   ///< [-1, 1]
    real turn = real(0);     ///< [-1, 1]，每秒转 turnSpeed 的比例（键盘用）
    real look = real(0);     ///< [-1, 1]，俯仰（键盘用）

    /// 直接叠加的角度增量（弧度）。**鼠标用这个**。
    ///
    /// 为什么不复用 turn/look：那两个是"按住方向键的比例"，会再乘一遍
    /// turnSpeed*dt。鼠标给的是已经确定的位移，再乘一次 dt 就会变成
    /// "帧率越高转得越慢" —— 鼠标手感必须与帧率无关。
    real turnDelta = real(0);
    real lookDelta = real(0);
    bool sprint = false;
    bool jump = false;
    bool fire = false;
    bool reload = false;
};

enum class GameState : std::uint8_t { Playing, Won, Lost };

//------------------------------------------------------------------------------
// 开火留下的画面痕迹
//
// 特效是**游戏状态**，不是渲染器的私有数据。理由很实在：
//   - ASCII 版、像素版、无头截图模式要画的是同一批火花，
//     放在某一个渲染器里另外两个就看不见
//   - 它们的寿命按游戏时间走，而渲染器不推进时间
//
// 所以它们和血量、弹药一样存在 Game 里，渲染器只负责读出来画。
// 注意这里存的是**世界坐标**，不是屏幕坐标 —— 屏幕坐标是渲染器的事，
// 而且同一份数据要能被不同分辨率、不同视角的渲染器各自投影。
//------------------------------------------------------------------------------

/// 曳光弹：一条从枪口到命中点的线。没有它，开枪除了数字变化毫无反馈 ——
/// hitscan 武器的子弹是瞬时的，不画出来玩家根本不知道自己打了哪儿。
struct Tracer {
    Vec3 from;
    Vec3 to;
    real life = real(0);
    real maxLife = real(1);
    /// 敌人打过来的。画成红色，而且这是玩家判断"火力从哪个方向来"的唯一线索。
    bool hostile = false;
};

/// 命中点的火花。它同时告诉玩家两件事：打中了、打中的是什么材质。
struct Impact {
    Vec3 position;
    Vec3 normal;
    real life = real(0);
    real maxLife = real(1);
    EntityKind target = EntityKind::World;
};

//------------------------------------------------------------------------------
// 游戏
//------------------------------------------------------------------------------
class Game : public IPhysicsEventListener {
public:
    Game();

    void BuildLevel();

    /// 推进一帧。dt 是真实帧间隔，物理内部会自己按固定步长切分。
    void Update(const PlayerIntent& intent, real dt);

    //-- 给渲染器读的 -----------------------------------------------------------
    const PhysicsWorld& World() const noexcept { return m_world; }
    const CharacterController& Player() const noexcept { return m_player; }
    real PlayerYaw() const noexcept { return m_yaw; }
    real PlayerPitch() const noexcept { return m_pitch; }
    Vec3 EyePosition() const noexcept;

    /// 射线打中的刚体是什么？返回 nullptr 表示是静态地图或者不认识的东西。
    const Entity* FindEntity(BodyHandle body) const;

    //-- 给 AI / 机器人玩家读的 -------------------------------------------------

    struct EnemyView {
        Vec3 position;   ///< 胶囊中心
        Vec3 eye;        ///< 眼睛高度（做视线判定要用这个，不是脚底）
        /// **瞄准点**：躯干中心。身体是人形之后，头只是个 27 厘米的球，
        /// 照着它开枪基本全脱靶；视线判定也一并用这个点，
        /// 才能保证"判定看得见的地方"就是"子弹要去的地方"。
        Vec3 aim;
        BodyHandle body;
        real distance;   ///< 到玩家眼睛的距离
        bool visible;    ///< 玩家现在看得见它吗
    };

    /// 所有还活着的敌人，附带"玩家看不看得见"。
    ///
    /// 视线判定复用的是和敌人 AI 完全同一套逻辑（一条射线，看第一个命中是不是它）
    /// —— 这保证了"机器人能打到的"和"玩家能打到的"是同一件事。
    std::vector<EnemyView> AliveEnemyViews() const;

    GameState State() const noexcept { return m_state; }
    real Health() const noexcept { return m_health; }
    real MaxHealth() const noexcept { return m_config.playerMaxHealth; }
    int Ammo() const noexcept { return m_ammo; }
    int MagazineSize() const noexcept { return m_config.magazineSize; }
    int Reserve() const noexcept { return m_reserve; }
    bool Reloading() const noexcept { return m_reloadTimer > real(0); }
    int EnemiesAlive() const noexcept;
    int EnemiesTotal() const noexcept { return m_enemyTotal; }
    real ElapsedTime() const noexcept { return m_time; }
    real MuzzleFlash() const noexcept { return m_muzzleFlash; }
    real DamageFlash() const noexcept { return m_damageFlash; }
    /// 刚打中敌人？渲染器拿它在准星上画一个"命中标记"。
    /// FPS 里这个反馈的重要性被严重低估：没有它，玩家分不清"打中了但没死"
    /// 和"打偏了"，于是每一枪都要靠猜。
    real HitMarker() const noexcept { return m_hitMarker; }
    const std::vector<Tracer>& Tracers() const noexcept { return m_tracers; }
    const std::vector<Impact>& Impacts() const noexcept { return m_impacts; }
    /// 枪口在世界里的位置。曳光弹从这里出发，画出来才像是"从枪里打出去的"，
    /// 而不是从眼球中间射出去的。
    Vec3 MuzzlePosition() const noexcept;
    const std::vector<std::string>& Log() const noexcept { return m_log; }

    const GameConfig& Config() const noexcept { return m_config; }

    //-- IPhysicsEventListener --------------------------------------------------
    void OnTriggerEnter(ColliderHandle trigger, ColliderHandle other) override;

private:
    //-- 关卡搭建 ---------------------------------------------------------------
    BodyHandle AddStaticBox(const Vec3& center, const Vec3& halfExtents,
                            const Quat& rotation = Quat::Identity());
    void AddCrate(const Vec3& center, real half, real mass);
    void AddEnemy(const Vec3& footPosition);

    /// 给敌人的替身刚体挂一副人形碰撞体（头、躯干、胯、两腿、两臂）。
    /// 局部坐标以胶囊**中心**为原点、局部 +X 是身体正面。
    void AddHumanoidColliders(BodyHandle body, real height);
    void AddPickup(const Vec3& center, PickupKind kind, real amount);
    void SpawnPlayer(const Vec3& footPosition, real yaw);

    /// 登记一个实体并建立 刚体 -> 实体 的反查
    int RegisterEntity(const Entity& entity);

    //-- 每帧 -------------------------------------------------------------------
    void UpdatePlayer(const PlayerIntent& intent, real dt);
    void UpdateWeapon(const PlayerIntent& intent, real dt);
    void UpdateEnemies(real dt);
    void UpdatePickups(real dt);
    void UpdateEffects(real dt);
    void UpdateDebris(real dt);
    void SyncProxyBody(BodyHandle body, const CharacterController& controller,
                       const Quat& rotation = Quat::Identity());
    void FirePlayerWeapon();
    void EnemyFire(Entity& enemy, const CharacterController& controller);

    //-- 命中反馈与破坏 ---------------------------------------------------------
    void SpawnTracer(const Vec3& from, const Vec3& to, bool hostile);
    void SpawnImpact(const Vec3& position, const Vec3& normal, EntityKind target);

    /// 把一个箱子换成八个小块。参数用**实体下标**而不是引用：
    /// 下面会往 m_entities 里 push_back，任何引用都会在中途失效。
    void ShatterCrate(int entityIndex, const Vec3& impactDirection);
    void KillEnemy(int entityIndex, const Vec3& impactDirection);

    /// 生成一块碎块。超出预算时会先回收最老的一块。
    void SpawnDebrisPiece(const Vec3& center, real half, DebrisKind kind,
                          const Vec3& velocity);
    void RetireOldestDebris();
    /// 销毁实体的刚体并断开 刚体 -> 实体 的反查。
    void DestroyEntityBody(int entityIndex);

    /// [0,1) 的确定性随机数。**不用 std::rand**：机器人模式的跑分必须可复现，
    /// 而全局随机状态会被任何一处调用悄悄改掉。
    real Random01();
    Vec3 RandomDirection();

    /// 从 from 能不能看见 to？用一条射线判遮挡 —— AI 视线判定的标准做法。
    bool HasLineOfSight(const Vec3& from, const Vec3& to, LayerMask mask,
                        BodyHandle expected) const;

    void Damage(real amount, const char* source);
    void Log(const std::string& line);

    //-- 数据 -------------------------------------------------------------------
    GameConfig m_config;
    PhysicsWorld m_world;

    CharacterController m_player;
    BodyHandle m_playerBody = BodyHandle::Null();
    real m_yaw = real(0);
    real m_pitch = real(0);

    std::vector<Entity> m_entities;
    /// BodyHandle.index -> m_entities 下标。
    /// 只用 index 不用 generation：句柄失效时实体也已经标记 alive=false，
    /// 而 index 会在下一个刚体创建时被复用 —— 所以销毁实体时必须把这条也删掉。
    std::unordered_map<std::uint32_t, int> m_bodyToEntity;

    /// 敌人各自的角色控制器。和 m_entities 里的敌人一一对应，用实体下标索引。
    std::unordered_map<int, CharacterController> m_enemyControllers;

    GameState m_state = GameState::Playing;
    real m_health = real(100);
    int m_ammo = 0;
    int m_reserve = 0;
    real m_fireTimer = real(0);
    real m_reloadTimer = real(0);
    real m_muzzleFlash = real(0);
    real m_damageFlash = real(0);
    real m_hitMarker = real(0);
    real m_time = real(0);
    int m_enemyTotal = 0;

    //-- 特效 -------------------------------------------------------------------
    std::vector<Tracer> m_tracers;
    std::vector<Impact> m_impacts;
    /// 当前活着的碎块数。单独记一个数，免得每帧为了限流去扫整张实体表。
    int m_debrisAlive = 0;
    /// 特效用的随机数状态（LCG）。种子写死 -> 同一段输入永远得到同一场画面。
    std::uint32_t m_rng = 0x9E3779B9u;

    std::vector<std::string> m_log;
};

}  // namespace game
