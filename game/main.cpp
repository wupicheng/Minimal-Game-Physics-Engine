//==============================================================================
// game/main.cpp
//
// 入口与主循环。
//
//------------------------------------------------------------------------------
// 三种输出方式，**同一套渲染算法**
//------------------------------------------------------------------------------
//   --gui     真窗口 + 真像素（Win32 GDI）。内部 256×144 逐像素投射，放大到窗口
//   --ascii   控制台字符画。内部 100×34 逐像素投射，每个像素输出一个灰度字符
//   --shot    无头渲染若干帧，把画面存成 PPM 图片
//
// 三者的**渲染算法完全一样** —— 每个像素一次 `world.Raycast()`。区别只在最后
// 一步把命中结果写成什么：一个 RGB 值、一个字符、还是一个文件。
// ASCII 版从来不是"简化版渲染"，它只是输出到了一个只有 10 级灰度的设备上。
//
//------------------------------------------------------------------------------
// 主循环
//------------------------------------------------------------------------------
//     while (跑着):
//         dt = 真实帧间隔（有抖动）
//         intent = 采输入
//         game.Update(intent, dt)     <- 内部：world.Step(dt) 用固定步长切分
//         renderer.RenderScene(...)   <- 每个像素一次 world.Raycast()
//         present
//
// **物理用固定步长、渲染用真实帧率**，两者由 `PhysicsWorld` 内部的累加器解耦。
// 这不是可选的讲究：不这么做的话跳跃高度会随帧率变化（见 PhysicsWorld.h）。
//==============================================================================

#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include "game/Game.h"
#include "game/Input.h"
#include "game/PixelRenderer.h"
#include "game/Renderer.h"
#include "platform/Window.h"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

using namespace game;

namespace {

//-- ASCII 模式 -----------------------------------------------------------------
constexpr int kAsciiWidth = 100;
constexpr int kAsciiHeight = 34;

//------------------------------------------------------------------------------
// 图形模式的分辨率
//
// 这个尺寸是被**射线预算**定死的，不是审美选择：实测一条 `world.Raycast` 约
// 733 ns，256×144 = 36864 条 -> 27 ms/帧 -> 37 fps。再往上就掉到 30 以下：
//     320×180 -> 24 fps      640×360 -> 6 fps
// 所以低分辨率渲染 + 整数倍放大，得到清晰的复古颗粒感。
//------------------------------------------------------------------------------
constexpr int kPixelWidth = 256;
constexpr int kPixelHeight = 144;
constexpr int kPixelScale = 4;  // 窗口 1024×576

constexpr real kTargetFps = real(60);

enum class Mode { Gui, Ascii, Shot };

void PrintIntro(Mode mode) {
    std::printf("\n");
    std::printf("================================================================\n");
    std::printf(" 一个跑在 PhysEngine 上的 FPS —— %s\n", pe::VersionString());
    std::printf("================================================================\n\n");

    if (mode == Mode::Gui) {
        std::printf("  操作：W/A/S/D 移动   鼠标 转视角   鼠标左键 开火\n");
        std::printf("        Shift 冲刺   空格 跳跃   R 换弹   Esc 释放鼠标 / 退出\n");
        std::printf("        一局结束后：R 重开   Esc 退出\n\n");
        std::printf("  内部按 %dx%d 逐像素光线投射，放大 %d 倍到窗口。\n", kPixelWidth,
                    kPixelHeight, kPixelScale);
    } else {
        std::printf("  操作：W/A/S/D 移动   Q/E 或 左右方向键 转向   上/下 抬低头\n");
        std::printf("        Shift 冲刺   空格 跳跃   Ctrl / F 开火   R 换弹   Esc 退出\n");
        std::printf("        一局结束后：R 重开   Esc 退出\n\n");
        std::printf("  内部按 %dx%d 逐像素光线投射，每个像素输出一个灰度字符。\n",
                    kAsciiWidth, kAsciiHeight);
    }
    std::printf("  渲染器没有任何自己的几何数据 —— 每一个像素都是一次\n");
    std::printf("  world.Raycast()，画面里看到的就是物理里存在的。\n\n");
    std::printf("  目标：清除地图上全部 5 个敌人。\n\n");
}

//------------------------------------------------------------------------------
// 窗口输入：键盘 + 鼠标视角
//------------------------------------------------------------------------------
class WindowInput final : public IInputSource {
public:
    explicit WindowInput(platform::Window& window) : m_window(window) {}

    PlayerIntent Poll(const Game&, real, real) override {
        PlayerIntent intent;
#ifdef _WIN32
        const auto down = [&](int vk) { return m_window.IsKeyDown(vk); };

        if (down('W')) intent.forward += real(1);
        if (down('S')) intent.forward -= real(1);
        if (down('D')) intent.strafe += real(1);
        if (down('A')) intent.strafe -= real(1);

        // 键盘也留一套转向，方便没有鼠标时用
        if (down(VK_RIGHT) || down('E')) intent.turn += real(1);
        if (down(VK_LEFT) || down('Q')) intent.turn -= real(1);

        intent.sprint = down(VK_SHIFT);
        intent.jump = down(VK_SPACE);
        intent.fire = down(VK_LBUTTON) || down(VK_CONTROL);
        intent.reload = down('R');

        // 鼠标视角
        int dx = 0;
        int dy = 0;
        m_window.GetMouseDelta(dx, dy);
        constexpr real kSensitivity = real(0.0022);
        intent.turnDelta = static_cast<real>(dx) * kSensitivity;
        intent.lookDelta = static_cast<real>(-dy) * kSensitivity;

        // Esc：第一次释放鼠标，再按一次退出。
        // 直接退出的话玩家想找回鼠标就只能杀进程。
        const bool escape = down(VK_ESCAPE);
        if (escape && !m_escapeWasDown) {
            if (m_window.MouseCaptured()) {
                m_window.SetMouseCaptured(false);
            } else {
                m_quit = true;
            }
        }
        m_escapeWasDown = escape;

        // 鼠标已释放时，点一下窗口重新捕获
        if (!m_window.MouseCaptured() && down(VK_LBUTTON)) {
            m_window.SetMouseCaptured(true);
        }
#endif
        return intent;
    }

    bool WantsQuit() const override { return m_quit || !m_window.IsOpen(); }

private:
    platform::Window& m_window;
    bool m_quit = false;
    bool m_escapeWasDown = false;
};

//------------------------------------------------------------------------------
// 统计输出
//------------------------------------------------------------------------------
struct Timings {
    double physicsMs = 0;
    double renderMs = 0;
    std::size_t rays = 0;
    int frames = 0;
};

void PrintSummary(const Game& game, const Timings& t) {
    std::printf("\n");
    std::printf("================================================================\n");
    switch (game.State()) {
        case GameState::Won:
            std::printf(" 任务完成 —— 清除全部 %d 个敌人，用时 %.1f 秒\n",
                        game.EnemiesTotal(), double(game.ElapsedTime()));
            break;
        case GameState::Lost:
            std::printf(" 阵亡 —— 还剩 %d 个敌人，坚持了 %.1f 秒\n", game.EnemiesAlive(),
                        double(game.ElapsedTime()));
            break;
        default:
            std::printf(" 结束 —— 剩余敌人 %d/%d，生命 %d，用时 %.1f 秒\n",
                        game.EnemiesAlive(), game.EnemiesTotal(),
                        static_cast<int>(game.Health()), double(game.ElapsedTime()));
            break;
    }

    if (t.frames > 0) {
        const double total = (t.physicsMs + t.renderMs) / t.frames;
        std::printf("\n 性能（%d 帧平均）：\n", t.frames);
        std::printf("   物理 %.3f ms/帧\n", t.physicsMs / t.frames);
        std::printf("   渲染 %.3f ms/帧   （%zu 条射线/帧，%.0f ns/条）\n",
                    t.renderMs / t.frames, t.rays / static_cast<std::size_t>(t.frames),
                    t.renderMs * 1e6 / static_cast<double>(t.rays));
        std::printf("   合计 %.3f ms/帧  ->  上限约 %.0f fps\n", total, 1000.0 / total);
    }

    const PhysicsWorld::Stats& stats = game.World().GetStats();
    std::printf("\n 世界：刚体 %zu，碰撞体 %zu，候选对 %zu，接触 %zu，活跃体 %zu\n",
                game.World().BodyCount(), game.World().ColliderCount(),
                stats.broadPhasePairs, stats.narrowPhaseContacts, stats.activeBodies);
    std::printf("================================================================\n\n");
}

}  // namespace

//==============================================================================

int main(int argc, char** argv) {
    Mode mode = Mode::Gui;
    bool modeExplicit = false;
    bool noColor = false;
    bool forceBot = false;
    int frameLimit = 0;
    std::string shotPath = "shot.ppm";
    int shotWarmupFrames = 90;

    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        if (std::strcmp(a, "--gui") == 0) { mode = Mode::Gui; modeExplicit = true; }
        else if (std::strcmp(a, "--ascii") == 0) { mode = Mode::Ascii; modeExplicit = true; }
        else if (std::strcmp(a, "--script") == 0) { mode = Mode::Ascii; modeExplicit = true; forceBot = true; }
        else if (std::strcmp(a, "--no-color") == 0) noColor = true;
        else if (std::strcmp(a, "--bot") == 0) forceBot = true;
        else if (std::strcmp(a, "--frames") == 0 && i + 1 < argc) frameLimit = std::atoi(argv[++i]);
        else if (std::strcmp(a, "--shot") == 0) {
            mode = Mode::Shot;
            modeExplicit = true;
            if (i + 1 < argc && argv[i + 1][0] != '-') shotPath = argv[++i];
        }
        else if (std::strcmp(a, "--warmup") == 0 && i + 1 < argc) shotWarmupFrames = std::atoi(argv[++i]);
    }

    EnableUtf8Console();

    // 没显式指定模式时：能开窗口就开，开不了（无头会话、输出被重定向）退回 ASCII
    if (!modeExplicit && !platform::Window::IsAvailable()) mode = Mode::Ascii;

    //--------------------------------------------------------------------------
    // 用 unique_ptr 拿着这一局，是为了**重开**：结算之后按 R 就整个换一个新的
    // `Game`。
    //
    // 为什么不写一个 `Game::Reset()` 挨个把成员清回去：那种写法的失败模式是
    // "漏了一个成员"，而漏掉的偏偏是最难发现的那种 —— 上一局的碎块还躺在地上、
    // 敌人的警觉状态还留着。重新构造一个对象则不可能漏，编译器负责。
    //
    // 顺带：`Game` 在构造函数里把自己注册成物理世界的事件监听者
    // （`SetEventListener(this)`），所以它**不能**用赋值来"重置" ——
    // 那会让世界里存着一个指向临时对象的指针。只能整个换掉。
    //--------------------------------------------------------------------------
    const auto NewGame = [] {
        auto fresh = std::make_unique<Game>();
        fresh->BuildLevel();
        return fresh;
    };
    std::unique_ptr<Game> game = NewGame();

    Timings timings;
    using Clock = std::chrono::steady_clock;

    //==========================================================================
    // 截图模式：无头渲染几帧存成 PPM
    //
    // 它存在的理由很实在 —— 在没有显示器的环境里（CI、远程会话），
    // 这是**唯一**能确认"画出来的东西对不对"的手段。
    //==========================================================================
    if (mode == Mode::Shot) {
        PrintIntro(Mode::Gui);
        std::printf("  [截图模式] 先让机器人跑 %d 帧，再把画面存到 %s\n\n",
                    shotWarmupFrames, shotPath.c_str());

        PixelRenderer renderer(kPixelWidth, kPixelHeight);
        BotInput bot(real(90));
        const real dt = real(1) / kTargetFps;
        real gameTime = real(0);

        for (int i = 0; i < shotWarmupFrames; ++i) {
            game->Update(bot.Poll(*game, gameTime, dt), dt);
            gameTime += dt;
            if (game->State() != GameState::Playing) break;
        }

        Camera camera;
        camera.position = game->EyePosition();
        camera.yaw = game->PlayerYaw();
        camera.pitch = game->PlayerPitch();

        const auto t0 = Clock::now();
        renderer.RenderScene(*game, camera);
        const auto t1 = Clock::now();
        renderer.DrawHud(*game);

        const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        if (renderer.Frame().SavePpm(shotPath)) {
            std::printf("  已保存 %s（%dx%d，%zu 条射线，%.1f ms）\n\n", shotPath.c_str(),
                        kPixelWidth, kPixelHeight, renderer.LastRayCount(), ms);
        } else {
            std::printf("  写文件失败：%s\n\n", shotPath.c_str());
            return 1;
        }

        timings.frames = 1;
        timings.renderMs = ms;
        timings.rays = renderer.LastRayCount();
        PrintSummary(*game, timings);
        return 0;
    }

    //==========================================================================
    // 图形模式
    //==========================================================================
    if (mode == Mode::Gui) {
        PrintIntro(Mode::Gui);

        platform::Window window("PhysEngine FPS", kPixelWidth * kPixelScale,
                      kPixelHeight * kPixelScale);
        if (!window.IsOpen()) {
            std::printf("  开窗口失败，退回 ASCII 模式。\n\n");
            mode = Mode::Ascii;
        } else {
            PixelRenderer renderer(kPixelWidth, kPixelHeight);
            WindowInput windowInput(window);
            BotInput bot(real(90));
            IInputSource* input = forceBot ? static_cast<IInputSource*>(&bot)
                                           : static_cast<IInputSource*>(&windowInput);
            if (!forceBot) window.SetMouseCaptured(true);

            auto previous = Clock::now();
            real gameTime = real(0);

            while (window.IsOpen()) {
                window.PumpMessages();
                if (!window.IsOpen()) break;

                const auto now = Clock::now();
                real dt = std::chrono::duration<real>(now - previous).count();
                previous = now;
                // dt 封顶：拖动窗口、切出去再回来时 dt 可能是好几秒
                dt = Clamp(dt, real(0.001), real(0.1));

                const PlayerIntent intent = input->Poll(*game, gameTime, dt);
                if (input->WantsQuit()) break;

                const auto t0 = Clock::now();
                game->Update(intent, dt);
                const auto t1 = Clock::now();

                Camera camera;
                camera.position = game->EyePosition();
                camera.yaw = game->PlayerYaw();
                camera.pitch = game->PlayerPitch();

                renderer.RenderScene(*game, camera);
                renderer.DrawHud(*game);
                window.Present(renderer.Frame().Data(), kPixelWidth, kPixelHeight);
                const auto t2 = Clock::now();

                timings.physicsMs += std::chrono::duration<double, std::milli>(t1 - t0).count();
                timings.renderMs += std::chrono::duration<double, std::milli>(t2 - t1).count();
                timings.rays += renderer.LastRayCount();
                ++timings.frames;
                gameTime += dt;

                if (game->State() != GameState::Playing) {
                    //----------------------------------------------------------
                    // 结算画面：横幅 + 等玩家决定重开还是退出。
                    //
                    // 横幅本身是**必须**的：没有它，"被打死了"和"程序崩了"在玩家
                    // 眼里完全一样 —— 画面一停、窗口就没了。
                    //
                    // 而"死了就只能重新启动程序"同样是不能接受的：死亡在 FPS 里
                    // 是常态，不是一局的终点。所以这里停在结算画面上等按键，
                    // 按 R 换一局全新的，按 Esc（或者关窗口）才真的退出。
                    //----------------------------------------------------------
                    const bool won = game->State() == GameState::Won;
                    renderer.DrawBanner(won ? "MISSION COMPLETE" : "YOU DIED",
                                        forceBot ? "" : "R RESTART    ESC QUIT",
                                        won ? Framebuffer::Pack(120, 230, 120)
                                            : Framebuffer::Pack(230, 70, 60));

                    // 结算时把鼠标还给玩家。锁着光标等按键的话，玩家连窗口都点不掉。
                    window.SetMouseCaptured(false);

                    bool restart = false;
                    // R 同时是换弹键、Esc 同时是释放鼠标键：死的那一刻手里多半正
                    // 按着其中一个。所以要等它们**先松开**，再认下一次按下 ——
                    // 否则玩家会看到结算画面一闪而过，又变成"闪退"。
                    bool armed = false;
                    // 机器人模式（CI）不等人：亮两秒结算画面就退出
                    int botCountdown = forceBot ? 120 : -1;
                    while (window.IsOpen()) {
                        window.PumpMessages();
                        if (!window.IsOpen()) break;
                        window.Present(renderer.Frame().Data(), kPixelWidth, kPixelHeight);

                        if (botCountdown >= 0 && --botCountdown <= 0) break;
                        if (botCountdown < 0) {
                            const bool r = window.IsKeyDown(kKeyRestart);
                            const bool esc = window.IsKeyDown(kKeyEscape);
                            if (!r && !esc) armed = true;
                            if (armed && r) { restart = true; break; }
                            if (armed && esc) break;
                        }
                        std::this_thread::sleep_for(std::chrono::milliseconds(16));
                    }
                    if (!restart) break;

                    game = NewGame();
                    gameTime = real(0);
                    // dt 必须重新起算：玩家在结算画面上待了多久，这里就会攒下
                    // 多长的一个 dt，新一局的第一帧会直接把角色弹出地图
                    previous = Clock::now();
                    window.SetMouseCaptured(true);
                    continue;
                }
                if (frameLimit > 0 && timings.frames >= frameLimit) break;
            }

            window.SetMouseCaptured(false);
            PrintSummary(*game, timings);
            return 0;
        }
    }

    //==========================================================================
    // ASCII 模式
    //==========================================================================
    {
        const bool interactive = !forceBot && HasInteractiveConsole();
        PrintIntro(Mode::Ascii);

        Renderer renderer(kAsciiWidth, kAsciiHeight);
        renderer.SetColorEnabled(!noColor && EnableAnsiColors());

        KeyboardInput keyboard;
        BotInput bot(real(90));
        IInputSource* input = interactive ? static_cast<IInputSource*>(&keyboard)
                                          : static_cast<IInputSource*>(&bot);

        if (interactive) {
            std::printf("  按回车开始…");
            std::fflush(stdout);
            std::getchar();
            std::printf("\x1b[2J");
        } else {
            std::printf("  [无交互控制台，机器人自动接管]\n\n");
        }

        auto previous = Clock::now();
        const real frameBudget = real(1) / real(30);
        real gameTime = real(0);

        while (true) {
            const auto now = Clock::now();
            real dt = std::chrono::duration<real>(now - previous).count();
            previous = now;
            // 机器人模式用固定 dt：结果必须可复现，不能受机器快慢影响
            if (!interactive) dt = frameBudget;
            dt = Clamp(dt, real(0.001), real(0.1));

            const PlayerIntent intent = input->Poll(*game, gameTime, dt);
            if (input->WantsQuit()) break;

            const auto t0 = Clock::now();
            game->Update(intent, dt);
            const auto t1 = Clock::now();

            Camera camera;
            camera.position = game->EyePosition();
            camera.yaw = game->PlayerYaw();
            camera.pitch = game->PlayerPitch();

            renderer.RenderScene(*game, camera);
            renderer.DrawHud(*game);
            if (game->State() != GameState::Playing) {
                const bool won = game->State() == GameState::Won;
                renderer.DrawCenteredText(kAsciiHeight / 2 - 1,
                                          won ? "MISSION COMPLETE" : "YOU DIED",
                                          won ? 92 : 91);
                if (interactive) {
                    renderer.DrawCenteredText(kAsciiHeight / 2 + 1,
                                              "PRESS R TO RESTART   ESC TO QUIT", 90);
                }
            }
            renderer.Present();
            const auto t2 = Clock::now();

            timings.physicsMs += std::chrono::duration<double, std::milli>(t1 - t0).count();
            timings.renderMs += std::chrono::duration<double, std::milli>(t2 - t1).count();
            timings.rays += renderer.LastRayCount();
            ++timings.frames;
            gameTime += dt;

            if (game->State() != GameState::Playing) {
                // 机器人模式（CI）直接结束；真人玩的话等一个决定：R 重开 / Esc 退出
                if (!interactive) break;

                bool restart = false;
                // 先等按键松开再认下一次按下：R 就是换弹键，死的那一刻手里多半
                // 正按着它，不然结算画面会一闪而过
                bool armed = false;
                while (true) {
                    const bool r = IsKeyPressed(kKeyRestart);
                    const bool esc = IsKeyPressed(kKeyEscape);
                    if (!r && !esc) armed = true;
                    if (armed && r) { restart = true; break; }
                    if (armed && esc) break;
                    std::this_thread::sleep_for(std::chrono::milliseconds(30));
                }
                if (!restart) break;

                game = NewGame();
                gameTime = real(0);
                previous = Clock::now();  // 同上：不重置的话新一局第一帧 dt 巨大
                std::printf("\x1b[2J");
                continue;
            }
            if (frameLimit > 0 && timings.frames >= frameLimit) break;

            if (interactive) {
                const auto elapsed = Clock::now() - now;
                const auto budget = std::chrono::duration<real>(frameBudget);
                if (elapsed < budget) {
                    std::this_thread::sleep_for(
                        std::chrono::duration_cast<std::chrono::milliseconds>(budget - elapsed));
                }
            }
        }

        std::printf("\x1b[0m");
        PrintSummary(*game, timings);
    }

    return 0;
}
