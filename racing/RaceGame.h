#pragma once
//==============================================================================
// racing/RaceGame.h
//
// 赛道、圈数、计时。
//
//------------------------------------------------------------------------------
// 这一层和车、和引擎的分工
//------------------------------------------------------------------------------
//   引擎  ：这条射线打到了哪个刚体、这两个刚体怎么弹开、谁进了触发区
//   Vehicle：悬挂多长、轮胎给多少力（见 Vehicle.h）
//   这里  ：赛道长什么样、过没过检查点、这一圈几秒、掉出赛道了要放回哪
//
//------------------------------------------------------------------------------
// 固定步长循环在这一层，不在引擎里
//------------------------------------------------------------------------------
// FPS 那边是 `world.Step(dt)` 一把梭，引擎内部自己切分子步 —— 角色控制器是
// 运动学的，一帧算一次没问题。
//
// 车不行：车是**靠力驱动的动态刚体**，力必须每个子步都加一次（见 Vehicle.h）。
// 所以这里自己攒时间，然后
//     while (攒够一个子步) { vehicle.Update(h); world.Step(h); }
// 每次只喂一个子步的时间，引擎内部的累加器就恰好跑一步。
//==============================================================================

#include <string>
#include <unordered_map>
#include <vector>

#include "racing/RacingTypes.h"
#include "racing/Vehicle.h"

namespace racing {

enum class RaceState : std::uint8_t {
    Countdown,  ///< 发车倒计时，油门是锁着的
    Racing,
    Finished,
};

//------------------------------------------------------------------------------
// 一个检查点
//
// 检查点是**触发区**，不是几何。它们必须按顺序过 —— 否则玩家在起跑线前后
// 来回蹭就能刷圈数，这是所有赛车游戏都要防的第一件事。
//------------------------------------------------------------------------------
struct Checkpoint {
    Vec3 center;
    real yaw = real(0);   ///< 门的朝向（车该沿这个方向穿过去）
    real halfWidth = real(9);
};

//------------------------------------------------------------------------------
// 赛事参数
//------------------------------------------------------------------------------
struct RaceConfig {
    int totalLaps = 3;
    real countdownSeconds = real(3);
    /// 翻车之后多久自动放回赛道（秒）
    real recoveryDelay = real(2.5);
    /// 顶在墙上一动不动多久之后自动放回（秒）。比翻车宽限久一点 ——
    /// 真的只是在弯里慢了下来的车不该被强行传送。
    real stalledDelay = real(4);
    /// 低于这个高度就算掉出去了
    real fallLimit = real(-8);
    Vec3 gravity = Vec3(real(0), real(-9.81), real(0));
};

//------------------------------------------------------------------------------
// 比赛
//------------------------------------------------------------------------------
class RaceGame : public IPhysicsEventListener {
public:
    RaceGame();

    void BuildTrack();

    /// 推进一帧真实时间。内部按固定步长切成若干子步（见文件头）。
    void Update(const DriveInput& input, real dt);

    //-- 给渲染器 / HUD 读的 ----------------------------------------------------
    const PhysicsWorld& World() const noexcept { return m_world; }
    const Vehicle& Car() const noexcept { return m_vehicle; }
    const Entity* FindEntity(BodyHandle body) const;

    RaceState State() const noexcept { return m_state; }
    int Lap() const noexcept { return m_lap; }
    int TotalLaps() const noexcept { return m_config.totalLaps; }
    /// 已经过了这一圈的第几个检查点（0 = 刚过终点线）
    int CheckpointsPassed() const noexcept { return m_nextCheckpoint; }
    int CheckpointCount() const noexcept { return static_cast<int>(m_checkpoints.size()); }
    const Checkpoint& CheckpointAt(int i) const { return m_checkpoints[static_cast<std::size_t>(i)]; }
    /// 下一个该过的检查点。HUD 上的箭头指着它。
    const Checkpoint& NextCheckpoint() const;

    //--------------------------------------------------------------------------
    // 赛车线：沿赛道中线一圈的路点（首尾相接）。
    //
    // 它属于**赛道**，不属于机器人 —— 赛道的形状只有这一层知道。机器人自己
    // 猜路线的话，一旦赛道改形状，AI 就会开进墙里；而这条线是和护墙同一份
    // 几何算出来的，改赛道尺寸时两者一起变。
    //
    // 只有 4 个检查点是不够的：直接朝下一个检查点开，会在椭圆赛道上抄近道，
    // 一头撞进内圈护墙 —— 这正是第一版机器人的死法。
    //--------------------------------------------------------------------------
    const std::vector<Vec3>& RacingLine() const noexcept { return m_racingLine; }

    /// 手动把车放回赛道（玩家按 R）。
    void RequestRecovery() noexcept { m_recoveryRequested = true; }

    real LapTime() const noexcept { return m_lapTime; }
    real BestLap() const noexcept { return m_bestLap; }
    real LastLap() const noexcept { return m_lastLap; }
    real TotalTime() const noexcept { return m_totalTime; }
    real Countdown() const noexcept { return m_countdown; }
    /// 复位倒计时（>0 表示正在等自动放回赛道）
    real RecoveryTimer() const noexcept { return m_recoveryTimer; }

    const std::vector<std::string>& Log() const noexcept { return m_log; }
    const RaceConfig& Config() const noexcept { return m_config; }

    //-- IPhysicsEventListener --------------------------------------------------
    void OnTriggerEnter(ColliderHandle trigger, ColliderHandle other) override;

private:
    //-- 搭赛道 -----------------------------------------------------------------
    BodyHandle AddStaticBox(EntityKind kind, const Vec3& center, const Vec3& half,
                            const Quat& rotation = Quat::Identity());
    void AddProp(const Vec3& center, real half, real mass);
    void AddCheckpoint(const Vec3& center, real yaw, real halfWidth, int index);
    /// 一段护墙：给起点和终点，自己算长度和朝向。
    /// 赛道是一圈折线，一段一段拼出来的 —— 直接写盒子的中心和旋转角
    /// 会让"改一下弯道半径"变成重算十几个数。
    void AddWallRun(const Vec3& from, const Vec3& to, real height);
    void RegisterEntity(const Entity& entity);

    //-- 每个子步 ---------------------------------------------------------------
    void StepRace(real h);
    void CheckRecovery(real h);
    void Log(const std::string& line);

    RaceConfig m_config;
    PhysicsWorld m_world;
    Vehicle m_vehicle;

    std::vector<Entity> m_entities;
    std::unordered_map<std::uint32_t, int> m_bodyToEntity;
    std::vector<Checkpoint> m_checkpoints;

    RaceState m_state = RaceState::Countdown;
    int m_lap = 0;
    int m_nextCheckpoint = 1;  ///< 0 号是终点线，发车后先去 1 号
    real m_countdown = real(0);
    real m_lapTime = real(0);
    real m_lastLap = real(0);
    real m_bestLap = real(0);
    real m_totalTime = real(0);
    real m_recoveryTimer = real(0);
    /// 连续几乎不动了多久。顶在墙上、四轮离地的车靠它自动脱困 ——
    /// 见 CheckRecovery。
    real m_stalledTimer = real(0);
    /// 上一次判定"确实在前进"时的位置。见 CheckRecovery 里的位移判据。
    Vec3 m_stallReference = Vec3::Zero();
    bool m_recoveryRequested = false;
    std::vector<Vec3> m_racingLine;

    /// 上一次合法通过的检查点 —— 复位就放回那里
    Vec3 m_recoveryPosition = Vec3::Zero();
    real m_recoveryYaw = real(0);

    real m_accumulator = real(0);
    std::vector<std::string> m_log;
};

}  // namespace racing
