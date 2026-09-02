//==============================================================================
// racing/main.cpp
//
// 入口与主循环。
//
//------------------------------------------------------------------------------
// 三种模式
//------------------------------------------------------------------------------
//   --gui     真窗口 + 真像素（Win32 GDI），键盘开车
//   --bot     机器人自己跑完整场比赛，跑完打印圈速（CI / 冒烟测试用）
//   --shot    无头跑一段，把画面存成 PPM 图片
//
// 渲染都是同一件事：每个像素一次 `world.Raycast()`（见 RaceRenderer.h）。
//
//------------------------------------------------------------------------------
// 主循环
//------------------------------------------------------------------------------
//     while (跑着):
//         dt = 真实帧间隔
//         input = 采输入
//         race.Update(input, dt)   <- 内部按固定步长切子步：施力 -> Step(h)
//         camera.Follow(car, dt)
//         renderer.RenderScene(...)
//         present
//
// **物理固定步长、渲染跟真实帧率**，两者由 RaceGame 里的累加器解耦。
// 车尤其不能省这一条：力是按子步积分的，见 Vehicle.h。
//==============================================================================

#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include "platform/Window.h"
#include "racing/Input.h"
#include "racing/RaceGame.h"
#include "racing/RaceRenderer.h"

using namespace racing;

namespace {

//------------------------------------------------------------------------------
// 分辨率被**射线预算**定死，不是审美选择。
//
// 实测这台机器上一条 `world.Raycast` 约 1.0 微秒（穿过宽相位 DDA + 逐碰撞体
// 求交），于是：
//     320×180 = 57,600 条 -> 59 ms/帧 -> 17 fps   开起来已经发飘
//     240×135 = 32,400 条 -> 33 ms/帧 -> 30 fps   够开了
//
// 赛车比 FPS 更吃不消低帧率：车是一直在动的，30 fps 以下方向感就散了。
// 所以这里按 240×135 渲染，再整数倍放大贴到窗口上。
//
// 顺带一提，射程也是**线性**成本（DDA 要走多少个格子）：把射程从 200 米砍到
// 110 米就省掉了三分之一，见 RaceRenderer.cpp。
//------------------------------------------------------------------------------
constexpr int kWidth = 240;
constexpr int kHeight = 135;
constexpr int kScale = 4;  // 窗口 960×540

enum class Mode { Gui, Bot, Shot };

void PrintIntro() {
    std::printf("\n");
    std::printf("================================================================\n");
    std::printf(" 一个跑在 PhysEngine 上的赛车游戏 —— %s\n", pe::VersionString());
    std::printf("================================================================\n\n");
    std::printf("  操作：W/↑ 油门   S/↓ 刹车（停住后继续踩 = 倒车）\n");
    std::printf("        A/D 或 ←/→ 转向   空格 手刹漂移   R 复位   Esc 退出\n\n");
    std::printf("  车不是引擎里的东西：整台车是**一个刚体 + 四条向下的射线**，\n");
    std::printf("  悬挂是弹簧阻尼力，抓地力是接触点上受摩擦圆限制的冲量。\n");
    std::printf("  引擎只提供 Raycast / ApplyForceAtPoint / ApplyImpulseAtPoint。\n\n");
    std::printf("  目标：跑完 3 圈。跳台在上方直道外侧，想飞就往外压一点。\n\n");
}

void PrintSummary(const RaceGame& race, double renderMs, double physicsMs, int frames,
                  std::size_t rays) {
    std::printf("\n");
    std::printf("================================================================\n");
    if (race.State() == RaceState::Finished) {
        std::printf(" 完赛 —— %d 圈用时 %.2f 秒，最快单圈 %.2f 秒\n", race.TotalLaps(),
                    double(race.TotalTime()), double(race.BestLap()));
    } else {
        std::printf(" 结束 —— 第 %d/%d 圈，已用时 %.1f 秒，最快单圈 %.2f 秒\n",
                    race.Lap(), race.TotalLaps(), double(race.TotalTime()),
                    double(race.BestLap()));
    }

    if (frames > 0) {
        const double total = (renderMs + physicsMs) / frames;
        std::printf("\n 性能（%d 帧平均）：\n", frames);
        std::printf("   物理 %.3f ms/帧\n", physicsMs / frames);
        if (rays > 0) {
            std::printf("   渲染 %.3f ms/帧   （%zu 条射线/帧，%.0f ns/条）\n",
                        renderMs / frames, rays / static_cast<std::size_t>(frames),
                        renderMs * 1e6 / static_cast<double>(rays));
        }
        std::printf("   合计 %.3f ms/帧  ->  上限约 %.0f fps\n", total, 1000.0 / total);
    }

    const PhysicsWorld::Stats& stats = race.World().GetStats();
    std::printf("\n 世界：刚体 %zu，碰撞体 %zu，候选对 %zu，接触 %zu，活跃体 %zu\n",
                race.World().BodyCount(), race.World().ColliderCount(),
                stats.broadPhasePairs, stats.narrowPhaseContacts, stats.activeBodies);
    std::printf("================================================================\n\n");
}

}  // namespace

//==============================================================================

int main(int argc, char** argv) {
    Mode mode = Mode::Gui;
    int frameLimit = 0;
    std::string shotPath = "race.ppm";
    real shotTime = real(8);

    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        if (std::strcmp(a, "--gui") == 0) mode = Mode::Gui;
        else if (std::strcmp(a, "--bot") == 0) mode = Mode::Bot;
        else if (std::strcmp(a, "--frames") == 0 && i + 1 < argc) frameLimit = std::atoi(argv[++i]);
        else if (std::strcmp(a, "--shot") == 0) {
            mode = Mode::Shot;
            if (i + 1 < argc && argv[i + 1][0] != '-') shotPath = argv[++i];
        } else if (std::strcmp(a, "--at") == 0 && i + 1 < argc) {
            shotTime = static_cast<real>(std::atof(argv[++i]));
        }
    }

    EnableUtf8Console();

    // 没有窗口环境（无头会话、CI）就自动退回机器人模式
    if (mode == Mode::Gui && !platform::Window::IsAvailable()) mode = Mode::Bot;

    const auto NewRace = [] {
        auto race = std::make_unique<RaceGame>();
        race->BuildTrack();
        return race;
    };
    std::unique_ptr<RaceGame> race = NewRace();

    using Clock = std::chrono::steady_clock;
    double physicsMs = 0;
    double renderMs = 0;
    std::size_t rays = 0;
    int frames = 0;

    //==========================================================================
    // 机器人模式：无头跑完一整场，打印圈速
    //
    // 它是这个游戏的**冒烟测试**。跑完三圈意味着悬挂、轮胎摩擦圆、固定步长
    // 施力、触发器、圈数、自动复位这一整条链路都真的在工作 —— 这些是单元
    // 测试覆盖不到的"接在一起还能不能用"。
    //==========================================================================
    if (mode == Mode::Bot) {
        PrintIntro();
        std::printf("  [机器人模式] 自动跑完 %d 圈。\n\n", race->TotalLaps());

        BotDriver bot;
        const real dt = real(1) / real(60);  // 固定 dt：结果必须可复现
        const int limit = frameLimit > 0 ? frameLimit : 9000;

        for (int i = 0; i < limit; ++i) {
            const auto t0 = Clock::now();
            race->Update(bot.Poll(*race, dt), dt);
            physicsMs += std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
            ++frames;
            if (race->State() == RaceState::Finished) break;
        }

        for (const std::string& line : race->Log()) std::printf("  %s\n", line.c_str());
        PrintSummary(*race, renderMs, physicsMs, frames, rays);
        return race->State() == RaceState::Finished ? 0 : 1;
    }

    //==========================================================================
    // 截图模式：无头跑一段再存一张 PPM
    //
    // 没有显示器的环境里（CI、远程会话），这是唯一能确认"画出来的东西对不对"
    // 的手段 —— 赛车游戏尤其需要，因为很多问题（车卡在墙里、跳台朝向反了）
    // 只有看一眼才发现得了。
    //==========================================================================
    if (mode == Mode::Shot) {
        PrintIntro();
        std::printf("  [截图模式] 机器人跑到第 %.1f 秒，把画面存到 %s\n\n",
                    double(shotTime), shotPath.c_str());

        RaceRenderer renderer(kWidth, kHeight);
        ChaseCamera camera;
        BotDriver bot;
        const real dt = real(1) / real(60);

        const int steps = static_cast<int>(shotTime * real(60));
        for (int i = 0; i < steps; ++i) {
            race->Update(bot.Poll(*race, dt), dt);
            camera.Follow(race->Car(), dt);
            if (race->State() == RaceState::Finished) break;
        }

        const auto t0 = Clock::now();
        renderer.RenderScene(*race, camera);
        renderMs = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        renderer.DrawHud(*race);

        if (!renderer.Frame().SavePpm(shotPath)) {
            std::printf("  写文件失败：%s\n\n", shotPath.c_str());
            return 1;
        }
        std::printf("  已保存 %s（%dx%d，%zu 条射线，%.1f ms）\n\n", shotPath.c_str(),
                    kWidth, kHeight, renderer.LastRayCount(), renderMs);

        frames = 1;
        rays = renderer.LastRayCount();
        PrintSummary(*race, renderMs, physicsMs, frames, rays);
        return 0;
    }

    //==========================================================================
    // 图形模式
    //==========================================================================
    PrintIntro();

    platform::Window window("PhysEngine Racing", kWidth * kScale, kHeight * kScale);
    if (!window.IsOpen()) {
        std::printf("  开窗口失败。\n\n");
        return 1;
    }

    RaceRenderer renderer(kWidth, kHeight);
    ChaseCamera camera;
    KeyboardDriver driver;

    auto previous = Clock::now();

    while (window.IsOpen()) {
        window.PumpMessages();
        if (!window.IsOpen()) break;

        const auto now = Clock::now();
        real dt = std::chrono::duration<real>(now - previous).count();
        previous = now;
        // dt 封顶：拖窗口、切出去再回来时 dt 可能是好几秒
        dt = Clamp(dt, real(0.001), real(0.1));

        const DriveInput input = driver.Poll(*race, dt);
        if (driver.WantsQuit()) break;
        if (driver.WantsRespawn()) race->RequestRecovery();

        const auto t0 = Clock::now();
        race->Update(input, dt);
        const auto t1 = Clock::now();

        camera.Follow(race->Car(), dt);
        renderer.RenderScene(*race, camera);
        renderer.DrawHud(*race);

        if (race->State() == RaceState::Finished) {
            char line[64];
            std::snprintf(line, sizeof(line), "BEST %.2fS", double(race->BestLap()));
            renderer.DrawBanner("FINISH", std::string(line) + "   R RESTART   ESC QUIT",
                                platform::Canvas::Pack(120, 230, 120));
        }

        window.Present(renderer.Frame().Data(), kWidth, kHeight);
        const auto t2 = Clock::now();

        physicsMs += std::chrono::duration<double, std::milli>(t1 - t0).count();
        renderMs += std::chrono::duration<double, std::milli>(t2 - t1).count();
        rays += renderer.LastRayCount();
        ++frames;

        //----------------------------------------------------------------------
        // 完赛之后停在结算画面上等一个决定：R 重开、Esc 退出。
        //
        // 和 FPS 那边同一个理由：不给这一步的话，"跑完了"和"程序崩了"在玩家
        // 眼里是一样的 —— 画面一停、窗口就没了。
        //----------------------------------------------------------------------
        if (race->State() == RaceState::Finished) {
            bool restart = false;
            bool armed = false;  // 等按键先松开，见 game/main.cpp 里同样的处理
            while (window.IsOpen()) {
                window.PumpMessages();
                if (!window.IsOpen()) break;
                window.Present(renderer.Frame().Data(), kWidth, kHeight);

                const bool r = window.IsKeyDown(kKeyRestart);
                const bool esc = window.IsKeyDown(kKeyEscape);
                if (!r && !esc) armed = true;
                if (armed && r) { restart = true; break; }
                if (armed && esc) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(16));
            }
            if (!restart) break;

            race = NewRace();
            camera = ChaseCamera{};
            // dt 必须重新起算：玩家在结算画面上待了多久，这里就会攒下多长的
            // 一个 dt，新一局第一帧会直接把车弹出赛道
            previous = Clock::now();
            continue;
        }

        if (frameLimit > 0 && frames >= frameLimit) break;
    }

    PrintSummary(*race, renderMs, physicsMs, frames, rays);
    return 0;
}
