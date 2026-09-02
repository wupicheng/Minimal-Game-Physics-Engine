//==============================================================================
// demo/main.cpp
//
// 整合演示：一个胶囊角色在由静态盒子拼成的小地图里走动、上台阶、爬斜坡、
// 推箱子、开枪（hitscan），全程用控制台俯视图和状态转储呈现。
//
//------------------------------------------------------------------------------
// 为什么是"脚本化"而不是真的读键盘
//------------------------------------------------------------------------------
// 引擎不含渲染、也不该含输入。做成读键盘的交互程序有两个坏处：
//   - 跑不了 CI —— 没人按键它就什么都不做
//   - 不可复现 —— 出了问题没法让别人跑出同一个结果
//
// 所以这里把"玩家输入"写成一段固定的脚本（前进、转向、开枪…）。
// 它同时也是一个**冒烟测试**：跑完不崩、数值不发散、角色最终站在该站的地方。
// 真要接手柄/键盘，把 `ScriptedInput` 换成读设备即可，引擎侧一行都不用改。
//
//------------------------------------------------------------------------------
// 接自己的渲染层
//------------------------------------------------------------------------------
// 看 `RenderTopDown()` 里怎么用 `world.GetDebugDrawData()` —— 那就是引擎能给的
// 全部：每个碰撞体的形状、世界位姿、几个状态位。把 ASCII 换成画线框即可。
// 位姿要用 `GetInterpolatedTransform(handle, world.Alpha())`，
// 不然固定步长物理配可变帧率渲染会抖（见 PhysicsWorld.h）。
//==============================================================================

#include <cstdio>
#include <string>
#include <vector>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

#include "pe/pe.h"

using namespace pe;

namespace {

//==============================================================================
// 地图
//==============================================================================

struct Demo {
    PhysicsWorld world;
    CharacterController player;
    std::vector<BodyHandle> crates;
    BodyHandle triggerZone = BodyHandle::Null();

    //--------------------------------------------------------------------------
    // 角色在世界里的"替身"：一个运动学刚体。
    //
    // 角色控制器本身不在物理世界里（它按设计就不参与求解），但世界得知道它在哪儿，
    // 否则：触发区检测不到它、箱子也不会被它推动。
    //
    // 标准做法就是给它配一个 **Kinematic** 刚体：每帧把控制器算出来的位姿和速度
    // 抄过去。Kinematic 的 invMass 是 0，所以
    //   - 它能推动动态箱子（求解器照常施加冲量）
    //   - 箱子推不动它（invMass = 0，冲量对它无效）
    // 正好就是 CS 类游戏那条"单向作用力"的手感约定，一行特判都不用写。
    //--------------------------------------------------------------------------
    BodyHandle playerBody = BodyHandle::Null();

    Demo() : world(MakeConfig()) {}

    static WorldConfig MakeConfig() {
        WorldConfig config;
        // FPS 一般用比现实更大的重力，跳跃才不显得飘（见 Integrator.h）
        config.gravity = Vec3(real(0), real(-20), real(0));
        config.cellSize = real(2);
        return config;
    }

    BodyHandle AddStatic(const Vec3& center, const Vec3& halfExtents,
                         const Quat& rotation = Quat::Identity(),
                         const Material& material = Material::Wood()) {
        BodyDesc desc;
        desc.position = center;
        desc.rotation = rotation;
        desc.type = BodyType::Static;
        const BodyHandle body = world.CreateBody(desc);

        ColliderDesc collider;
        collider.shape = Shape::MakeBox(halfExtents);
        collider.material = material;
        collider.layer = layers::kWorld;
        world.AddCollider(body, collider);
        return body;
    }

    BodyHandle AddCrate(const Vec3& center, real half, real mass) {
        BodyDesc desc;
        desc.position = center;
        desc.type = BodyType::Dynamic;
        desc.mass = mass;
        const BodyHandle body = world.CreateBody(desc);

        ColliderDesc collider;
        collider.shape = Shape::MakeCube(half);
        collider.material = Material::Wood();
        world.AddCollider(body, collider);
        return body;
    }

    //--------------------------------------------------------------------------
    // 地图：地面 + 四面墙 + 一段三级台阶 + 一个斜坡（8 个静态盒）
    //--------------------------------------------------------------------------
    void BuildMap() {
        AddStatic(Vec3(0, real(-0.5), 0), Vec3(16, real(0.5), 11));  // 地面，顶面 y=0

        AddStatic(Vec3(0, 2, real(-10.5)), Vec3(16, 2, real(0.5)));  // 南墙
        AddStatic(Vec3(0, 2, real(10.5)), Vec3(16, 2, real(0.5)));   // 北墙
        AddStatic(Vec3(real(-15.5), 2, 0), Vec3(real(0.5), 2, 11));  // 西墙
        AddStatic(Vec3(real(15.5), 2, 0), Vec3(real(0.5), 2, 11));   // 东墙

        // 三级台阶，每级 0.3 米（在默认 stepOffset 0.35 之内，角色能自己走上去）
        for (int i = 0; i < 3; ++i) {
            const real h = real(0.15) * real(i + 1);
            AddStatic(Vec3(real(6) + real(i) * real(1.2), h, real(-4)),
                      Vec3(real(0.6), h, real(2)));
        }

        // 斜坡：25 度，绕 Z 转正角度 -> 沿 +X 升高（走上去是上坡）
        AddStatic(Vec3(real(-7), real(1.2), real(4)), Vec3(4, real(0.3), real(2)),
                  Quat::FromAxisAngle(Vec3(0, 0, 1), DegToRad(real(25))));

        // 一个触发区（拾取点）
        BodyDesc zoneDesc;
        zoneDesc.position = Vec3(real(-2), real(0.9), real(-6));
        zoneDesc.type = BodyType::Static;
        triggerZone = world.CreateBody(zoneDesc);
        ColliderDesc zoneCollider;
        zoneCollider.shape = Shape::MakeCube(real(1));
        zoneCollider.isTrigger = true;
        zoneCollider.layer = layers::kTrigger;
        world.AddCollider(triggerZone, zoneCollider);

        // 可推动的箱子
        crates.push_back(AddCrate(Vec3(real(3), real(0.4), real(1)), real(0.4), real(8)));
        crates.push_back(AddCrate(Vec3(real(4), real(0.4), real(2)), real(0.4), real(8)));
        crates.push_back(AddCrate(Vec3(real(1), real(0.4), real(3)), real(0.4), real(8)));
        // 叠两个，看堆叠稳不稳
        crates.push_back(AddCrate(Vec3(real(-4), real(0.4), real(-2)), real(0.4), real(8)));
        crates.push_back(AddCrate(Vec3(real(-4), real(1.3), real(-2)), real(0.4), real(8)));

        // 角色
        CharacterConfig playerConfig;
        playerConfig.radius = real(0.4);
        playerConfig.height = real(1.8);
        playerConfig.layerMask = layers::kWorld | layers::kDefault;
        player = CharacterController(playerConfig);
        player.SetFootPosition(Vec3(real(-12), real(0.05), real(-6)));

        // 角色的运动学替身（理由见上面 playerBody 的说明）
        BodyDesc playerDesc;
        playerDesc.position = player.position;
        playerDesc.type = BodyType::Kinematic;
        playerBody = world.CreateBody(playerDesc);

        ColliderDesc playerCollider;
        playerCollider.shape = playerConfig.MakeShape();
        playerCollider.material = Material(real(0.6), real(0));
        playerCollider.layer = layers::kPlayer;
        world.AddCollider(playerBody, playerCollider);

        player.UpdateGrounded(world);
    }

    /// 把控制器的状态同步给替身刚体。每帧在 `player.Update()` 之后调。
    void SyncPlayerBody() {
        world.SetBodyTransform(playerBody, player.GetTransform());
        // 速度也要抄过去 —— 求解器靠它算接触点的相对速度，
        // 不抄的话角色会像一堵"缓缓移动的墙"，推箱子推不出应有的力道。
        if (RigidBody* body = world.GetBody(playerBody)) {
            body->linearVelocity = player.velocity;
        }
    }
};

//==============================================================================
// 事件监听
//==============================================================================

class DemoListener final : public IPhysicsEventListener {
public:
    int triggerEnters = 0;
    int triggerExits = 0;
    int contactBegins = 0;
    std::vector<std::string> log;

    void OnTriggerEnter(ColliderHandle, ColliderHandle) override {
        ++triggerEnters;
        log.push_back("  [事件] 角色进入了拾取区");
    }
    void OnTriggerExit(ColliderHandle, ColliderHandle) override {
        ++triggerExits;
        log.push_back("  [事件] 角色离开了拾取区");
    }
    void OnContactBegin(const ContactEvent&) override { ++contactBegins; }
};

//==============================================================================
// 控制台俯视图
//
// X-Z 平面，Y 是高度（俯视图看不到）。这就是"无渲染依赖"的兜底可视化。
//==============================================================================

void RenderTopDown(const Demo& demo, const DemoListener&) {
    constexpr int kWidth = 66;
    constexpr int kHeight = 24;
    constexpr real kMinX = real(-16.5);
    constexpr real kMaxX = real(16.5);
    constexpr real kMinZ = real(-11.5);
    constexpr real kMaxZ = real(11.5);

    std::vector<char> canvas(static_cast<std::size_t>(kWidth * kHeight), ' ');

    const auto plot = [&](const Vec3& p, char c) {
        const real u = (p.x - kMinX) / (kMaxX - kMinX);
        const real v = (p.z - kMinZ) / (kMaxZ - kMinZ);
        const int x = static_cast<int>(u * real(kWidth));
        const int y = static_cast<int>(v * real(kHeight));
        if (x < 0 || x >= kWidth || y < 0 || y >= kHeight) return;
        canvas[static_cast<std::size_t>(y * kWidth + x)] = c;
    };

    //--------------------------------------------------------------------------
    // 这一段就是"接自己的渲染层"的样板：GetDebugDrawData 给出每个碰撞体的
    // 形状 + 世界位姿 + 状态位，剩下的完全是渲染层的事。
    //--------------------------------------------------------------------------
    std::vector<PhysicsWorld::DebugShape> shapes;
    demo.world.GetDebugDrawData(shapes);

    for (const PhysicsWorld::DebugShape& item : shapes) {
        char glyph = 'o';
        if (item.isTrigger) {
            glyph = '~';
        } else if (item.isStatic) {
            glyph = '#';
        } else if (item.isSleeping) {
            glyph = '.';  // 睡着的箱子画成点，一眼能看出休眠在起作用
        }

        // 把碰撞体的世界 AABB 铺满 —— 俯视图不需要精确形状
        const AABB box = ComputeWorldAABB(item.shape, item.transform);
        for (real x = box.min.x; x <= box.max.x; x += real(0.25)) {
            for (real z = box.min.z; z <= box.max.z; z += real(0.25)) {
                plot(Vec3(x, 0, z), glyph);
            }
        }
    }

    plot(demo.player.position, '@');

    std::printf("  +");
    for (int i = 0; i < kWidth; ++i) std::printf("-");
    std::printf("+\n");
    for (int y = 0; y < kHeight; ++y) {
        std::printf("  |");
        for (int x = 0; x < kWidth; ++x) {
            std::putchar(canvas[static_cast<std::size_t>(y * kWidth + x)]);
        }
        std::printf("|\n");
    }
    std::printf("  +");
    for (int i = 0; i < kWidth; ++i) std::printf("-");
    std::printf("+\n");
    std::printf("  # 静态几何   o 动态箱子   . 已休眠   ~ 触发区   @ 角色"
                "      （俯视图，向右是 +X，向下是 +Z）\n");
}

//==============================================================================
// hitscan
//==============================================================================

void FireHitscan(Demo& demo, const Vec3& direction, const char* label) {
    // 从眼睛的高度打出去（脚底往上 1.6 米），和 FPS 里一样
    const Vec3 eye = demo.player.FootPosition() + Vec3(0, real(1.6), 0);
    const Ray ray(eye, direction, real(100));

    // 屏蔽掉玩家自己那一层 —— 不然射线起点就在自己的胶囊里，
    // 第一个命中永远是自己（距离 0）。FPS 里"不会打到自己"就是这么实现的。
    WorldRaycastHit hit;
    if (!demo.world.Raycast(ray, hit, ~layers::kPlayer)) {
        std::printf("  [开枪] %-10s 打空了\n", label);
        return;
    }

    const RigidBody* body = demo.world.GetBody(hit.body);
    const char* kind = (body != nullptr && body->type == BodyType::Static)
                           ? "静态几何"
                           : "动态箱子";

    std::printf("  [开枪] %-10s 命中%s  距离 %5.2f m  命中点 (%6.2f, %5.2f, %6.2f)"
                "  法线 (%5.2f, %5.2f, %5.2f)\n",
                label, kind, double(hit.distance), double(hit.point.x),
                double(hit.point.y), double(hit.point.z), double(hit.normal.x),
                double(hit.normal.y), double(hit.normal.z));

    // 打中动态物体就给它一个冲量 —— 子弹是有动量的
    if (body != nullptr && body->type == BodyType::Dynamic) {
        if (RigidBody* target = demo.world.GetBody(hit.body)) {
            target->WakeUp();
            target->ApplyImpulseAtPoint(ray.direction * real(12), hit.point);
        }
    }
}

//==============================================================================
// 脚本化的玩家输入
//
// 换成真的键盘只需要替换这个函数。
//==============================================================================

struct Input {
    Vec3 move = Vec3::Zero();  ///< 期望的水平速度（世界空间）
    bool jump = false;
};

//------------------------------------------------------------------------------
// 路点导航
//
// 用"朝着下一个路点走"而不是"第几秒往哪个方向走"：后者是把结果硬编码进时间轴，
// 引擎行为一变（走得快一点、被箱子挡了一下）整条路线就全错位了。
// 路点是自我修正的 —— 走慢了就多走几帧，撞上东西了会绕，到了就换下一个。
//------------------------------------------------------------------------------
struct Waypoint {
    Vec3 target;       ///< 只看 X/Z，Y 由物理决定
    const char* what;  ///< 打日志用
};

const Waypoint kRoute[] = {
    {Vec3(real(-2), 0, real(-6)), "穿过拾取区"},
    {Vec3(real(1.2), 0, real(0.9)), "走向箱子堆"},
    {Vec3(real(7), 0, real(1.2)), "把箱子推向东边"},
    {Vec3(real(5.2), 0, real(-4)), "走到台阶前"},
    {Vec3(real(11), 0, real(-4)), "拾级而上"},
    // 注意这里要**绕开**斜坡再折向北，不能直接斜插过去：
    // 斜坡是架空的，它的底面沿 -X 方向越来越低，角色（1.8 米高）走到一半就会
    // 被斜坡背面顶住。第一次写这条路线时就是这么卡住的 —— 引擎的判断没错，
    // 是路线画错了。
    {Vec3(real(-12), 0, real(0)), "折返到地图西侧"},
    {Vec3(real(-12), 0, real(4)), "对准斜坡入口"},
    {Vec3(real(-4), 0, real(4)), "爬上 25 度斜坡"},
};
constexpr int kRouteCount = static_cast<int>(sizeof(kRoute) / sizeof(kRoute[0]));

struct Pilot {
    int current = 0;
    real jumpAt = real(7.5);
    bool jumped = false;

    Input Update(const Vec3& position, real time, const char** arrived) {
        Input input;
        *arrived = nullptr;
        if (current >= kRouteCount) return input;

        const Vec3 target = kRoute[current].target;
        Vec3 delta(target.x - position.x, real(0), target.z - position.z);
        const real distance = delta.Length();

        if (distance < real(0.8)) {
            *arrived = kRoute[current].what;
            ++current;
            if (current >= kRouteCount) return input;
            delta = Vec3(kRoute[current].target.x - position.x, real(0),
                         kRoute[current].target.z - position.z);
        }

        if (delta.LengthSq() > real(1e-4)) {
            input.move = delta.Normalized() * real(4);
        }

        if (!jumped && time >= jumpAt) {
            input.jump = true;
            jumped = true;
        }
        return input;
    }
};

void PrintState(const Demo& demo, real time) {
    const PhysicsWorld::Stats& stats = demo.world.GetStats();
    const Vec3 foot = demo.player.FootPosition();

    std::printf(
        "  t=%5.2fs  角色(%6.2f,%5.2f,%6.2f) %s  速度 %5.2f m/s  |  "
        "候选对 %3zu  接触 %2zu  活跃体 %zu  穿透 %.4f  重建率 %4.1f%%\n",
        double(time), double(foot.x), double(foot.y), double(foot.z),
        demo.player.isGrounded ? "着地" : "腾空",
        double(demo.player.velocity.Length()), stats.broadPhasePairs,
        stats.narrowPhaseContacts, stats.activeBodies, double(stats.maxPenetration),
        double(stats.proxyRelinkRatio * real(100)));
}

}  // namespace

//==============================================================================

int main() {
    // Windows 控制台默认按系统的传统代码页（简体中文机器上是 GBK）解释输出，
    // 而源码里的中文字面量是 UTF-8 —— 不切代码页的话打出来全是乱码。
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif

    std::printf("\n");
    std::printf("==========================================================="
                "=====================\n");
    std::printf(" PhysEngine 整合演示  ——  %s\n", pe::VersionString());
    std::printf("==========================================================="
                "=====================\n\n");

    Demo demo;
    demo.BuildMap();

    DemoListener listener;
    demo.world.SetEventListener(&listener);

    std::printf("场景：8 个静态盒（地面 / 四面墙 / 三级台阶 / 一个 25 度斜坡）"
                " + 1 个触发区 + 5 个可推动箱子 + 1 个胶囊角色\n");
    std::printf("刚体 %zu 个，碰撞体 %zu 个\n\n", demo.world.BodyCount(),
                demo.world.ColliderCount());

    std::printf("--- 初始状态 ---\n");
    RenderTopDown(demo, listener);
    std::printf("\n");

    //--------------------------------------------------------------------------
    // 主循环
    //--------------------------------------------------------------------------
    const real frameTime = real(1) / real(60);
    const int totalFrames = 60 * 40;
    real time = real(0);
    int nextShot = 0;
    const real shotTimes[4] = {real(1.0), real(6.0), real(11.0), real(37.0)};
    const char* shotLabels[4] = {"向东", "向北", "向下", "向西"};
    const Vec3 shotDirs[4] = {Vec3(1, 0, 0), Vec3(0, 0, 1), Vec3(0, -1, real(0.2)),
                              Vec3(-1, 0, 0)};

    std::printf("--- 逐帧状态（每半秒打一行）---\n");

    Pilot pilot;

    for (int frame = 0; frame < totalFrames; ++frame) {
        const char* arrived = nullptr;
        const Input input = pilot.Update(demo.player.position, time, &arrived);
        if (arrived != nullptr) {
            std::printf("  [路点] %s\n", arrived);
        }

        if (input.jump && demo.player.Jump(real(8))) {
            std::printf("  [动作] 起跳\n");
        }

        // 顺序：先推进世界（箱子、堆叠），再更新角色。
        // 角色不参与求解，所以它读的是这一步之后的世界状态。
        demo.world.Step(frameTime);
        demo.player.Update(demo.world, input.move, frameTime,
                           demo.world.Config().gravity);

        //----------------------------------------------------------------------
        // 把角色的位姿和速度同步给它的运动学替身。
        //
        // 推箱子不需要任何额外代码 —— 替身是 Kinematic（invMass = 0），
        // 求解器天然做到"能推别人、推不动自己"。
        //----------------------------------------------------------------------
        demo.SyncPlayerBody();

        time += frameTime;

        // 开枪
        if (nextShot < 4 && time >= shotTimes[nextShot]) {
            FireHitscan(demo, shotDirs[nextShot], shotLabels[nextShot]);
            ++nextShot;
        }

        // 事件日志
        for (const std::string& line : listener.log) std::printf("%s\n", line.c_str());
        listener.log.clear();

        if (frame % 60 == 59) PrintState(demo, time);
    }

    //--------------------------------------------------------------------------
    // 收尾
    //--------------------------------------------------------------------------
    std::printf("\n--- 最终状态 ---\n");
    RenderTopDown(demo, listener);

    std::printf("\n触发器：进入 %d 次，离开 %d 次；接触开始事件 %d 次\n",
                listener.triggerEnters, listener.triggerExits, listener.contactBegins);

    std::printf("\n箱子最终位置：\n");
    for (std::size_t i = 0; i < demo.crates.size(); ++i) {
        const RigidBody* body = demo.world.GetBody(demo.crates[i]);
        if (body == nullptr) continue;
        std::printf("  箱子 %zu  (%6.2f, %5.2f, %6.2f)  %s\n", i,
                    double(body->position.x), double(body->position.y),
                    double(body->position.z), body->isSleeping ? "已休眠" : "活跃");
    }

    const UniformGridStats grid = demo.world.BroadPhase().ComputeStats();
    std::printf("\n宽相位：%zu 个代理，静态格子 %zu / 动态格子 %zu，"
                "最挤 %zu 个/格，平均 %.2f 格/proxy\n",
                grid.proxyCount, grid.staticCellCount, grid.dynamicCellCount,
                grid.maxProxiesPerCell, double(grid.avgCellsPerProxy));

    const Vec3 foot = demo.player.FootPosition();
    std::printf("角色最终：脚底 (%.2f, %.2f, %.2f)，%s\n\n", double(foot.x),
                double(foot.y), double(foot.z),
                demo.player.isGrounded ? "站在地面上" : "腾空");

    // 简单的自检：跑完之后数值必须都是有限的
    bool ok = IsFinite(foot.x) && IsFinite(foot.y) && IsFinite(foot.z);
    demo.world.ForEachBody([&](BodyHandle, const RigidBody& body) {
        ok = ok && IsFinite(body.position.x) && IsFinite(body.position.y) &&
             IsFinite(body.position.z);
    });
    std::printf("自检：%s\n\n", ok ? "通过（所有数值有限，模拟没有发散）" : "失败");
    return ok ? 0 : 1;
}
