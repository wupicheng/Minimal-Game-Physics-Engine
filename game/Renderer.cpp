//==============================================================================
// game/Renderer.cpp
//
// 逐像素光线投射。原理见 Renderer.h —— 一句话：每个像素一次 world.Raycast()。
//==============================================================================

#include "game/Renderer.h"

#include <cstdio>

#include "game/Game.h"

namespace game {

namespace {

//------------------------------------------------------------------------------
// 亮度梯度：从近（亮/实）到远（暗/虚）
//
// 控制台没有颜色深浅，只能靠字符的"墨水量"造出明暗。这一串是按视觉密度排的。
//------------------------------------------------------------------------------
constexpr char kShades[] = "@%#*+=~-:.";
constexpr int kShadeCount = static_cast<int>(sizeof(kShades)) - 1;

//------------------------------------------------------------------------------
// ANSI 颜色（前景）
//------------------------------------------------------------------------------
constexpr std::uint8_t kGray = 37;
constexpr std::uint8_t kDarkGray = 90;
constexpr std::uint8_t kRed = 91;
constexpr std::uint8_t kGreen = 92;
constexpr std::uint8_t kYellow = 93;
constexpr std::uint8_t kBlue = 94;
constexpr std::uint8_t kCyan = 96;
constexpr std::uint8_t kWhite = 97;

/// 控制台的字符是竖着长的（大约 1:2），所以水平方向要按这个比例拉伸，
/// 否则画面看起来会被压扁。
constexpr real kAspectCorrection = real(0.5);

/// 近平面。比它更近的东西不画 —— 除以一个趋近于零的深度会让坐标炸掉。
constexpr real kNearPlane = real(0.06);

}  // namespace

//==============================================================================
// 投影（两个渲染器共用，声明在 Renderer.h）
//==============================================================================

bool ProjectToScreen(const Camera& camera, const Vec3& world, int width, int height,
                     real aspectCorrection, real& outX, real& outY, real& outDepth) {
    const Vec3 offset = world - camera.position;
    const Vec3 right = camera.Right();
    const Vec3 up = camera.Up();
    const Vec3 forward = camera.Forward();

    const real depth = Dot(offset, forward);
    if (depth <= kNearPlane) return false;  // 在身后

    const real tanHalfFovY = Tan(camera.fovY * real(0.5));
    const real tanHalfFovX =
        tanHalfFovY * (static_cast<real>(width) / static_cast<real>(height)) *
        aspectCorrection;

    // 这是 RenderScene 里"屏幕 -> 射线"那套公式的逆运算，两边必须严格对应，
    // 否则火花会和它该贴的那面墙差开几个像素
    const real ndcX = Dot(offset, right) / (depth * tanHalfFovX);
    const real ndcY = Dot(offset, up) / (depth * tanHalfFovY);

    outX = (ndcX + real(1)) * real(0.5) * static_cast<real>(width) - real(0.5);
    outY = (real(1) - ndcY) * real(0.5) * static_cast<real>(height) - real(0.5);
    outDepth = depth;
    return true;
}

bool ClipSegmentToCamera(const Camera& camera, Vec3& from, Vec3& to) {
    const Vec3 forward = camera.Forward();
    const real depthFrom = Dot(from - camera.position, forward);
    const real depthTo = Dot(to - camera.position, forward);

    if (depthFrom <= kNearPlane && depthTo <= kNearPlane) return false;
    if (depthFrom >= kNearPlane && depthTo >= kNearPlane) return true;

    // 一端在前一端在后：把在后面的那一端拉到近平面上
    const real t = (kNearPlane - depthFrom) / (depthTo - depthFrom);
    const Vec3 crossing = from + (to - from) * t;
    if (depthFrom < kNearPlane) {
        from = crossing;
    } else {
        to = crossing;
    }
    return true;
}

//==============================================================================

Renderer::Renderer(int width, int height)
    : m_width(width), m_height(height),
      m_cells(static_cast<std::size_t>(width * height), Cell{' ', kGray}),
      m_depth(static_cast<std::size_t>(width * height), real(1e9)) {
    m_out.reserve(static_cast<std::size_t>(width * height) * 12);
}

void Renderer::Clear() {
    for (Cell& cell : m_cells) {
        cell.glyph = ' ';
        cell.color = kGray;
    }
}

void Renderer::Put(int x, int y, char glyph, std::uint8_t color) {
    if (x < 0 || x >= m_width || y < 0 || y >= m_height) return;
    Cell& cell = m_cells[static_cast<std::size_t>(y * m_width + x)];
    cell.glyph = glyph;
    cell.color = color;
}

//------------------------------------------------------------------------------
// 往网格里写字。
//
// **只能写 ASCII。** 这不是偷懒，是字符网格的固有约束：
// 这里一个格子存一个 `char`，也就是 1 字节 = 1 格 = 屏幕上 1 列。
// 而一个中文字是 3 个 UTF-8 字节、却只占 2 列显示宽度 —— 塞进来的话
//   - 它会吃掉 3 个格子却只显示 2 列，后面所有内容左移一列，HUD 各字段互相挤歪
//   - 场景或者别的文字只要覆盖掉中间任何一个字节，整个字就变成乱码
//
// 所以画面**内**的文字（HUD、日志、结算）一律用 ASCII；
// 画面**外**的自由文本（开场说明、结束统计）是直接 printf 的，不受这个约束，
// 照常用中文。
//------------------------------------------------------------------------------
void Renderer::PutText(int x, int y, const std::string& text, std::uint8_t color) {
    for (std::size_t i = 0; i < text.size(); ++i) {
        Put(x + static_cast<int>(i), y, text[i], color);
    }
}

//==============================================================================
// 场景
//==============================================================================

void Renderer::RenderScene(const Game& game, const Camera& camera) {
    const PhysicsWorld& world = game.World();
    m_rayCount = 0;

    //--------------------------------------------------------------------------
    // 相机基
    //
    // 先算出视锥的三个方向向量，之后每个像素只要做两次加法就能得到射线方向 ——
    // 不需要每个像素都跑一遍三角函数。
    //--------------------------------------------------------------------------
    const Vec3 forward = camera.Forward();
    const Vec3 right = camera.Right();
    // 真正的"上"要由 forward 和 right 叉出来，不能直接用世界的 +Y ——
    // 抬头/低头时用世界 +Y 会让画面产生剪切变形。
    const Vec3 up = Cross(right, forward).Normalized();

    const real tanHalfFovY = Tan(camera.fovY * real(0.5));
    const real aspect =
        static_cast<real>(m_width) / static_cast<real>(m_height) * kAspectCorrection;
    const real tanHalfFovX = tanHalfFovY * aspect;

    const Vec3 eye = camera.position;
    const real maxDistance = real(80);

    // 明暗梯度的量程。**不等于**射线射程 —— 射程要够远（不然远处的墙会消失），
    // 但明暗要按"这张图上东西大多有多远"来定。按 80 米铺开的话，
    // 2 到 20 米这个真正有内容的区间只用掉梯度的前四档，画面糊成一片。
    const real shadeRange = real(26);

    for (int y = 0; y < m_height; ++y) {
        // 屏幕坐标 -> [-1, 1]，注意 y 要翻转（屏幕向下是正，世界向上是正）
        const real ndcY =
            real(1) - real(2) * (static_cast<real>(y) + real(0.5)) /
                          static_cast<real>(m_height);

        for (int x = 0; x < m_width; ++x) {
            const real ndcX =
                real(2) * (static_cast<real>(x) + real(0.5)) /
                    static_cast<real>(m_width) - real(1);

            const Vec3 dir =
                (forward + right * (ndcX * tanHalfFovX) + up * (ndcY * tanHalfFovY))
                    .Normalized();

            //------------------------------------------------------------------
            // 这一行就是整个渲染器
            //------------------------------------------------------------------
            WorldRaycastHit hit;
            ++m_rayCount;
            if (!world.Raycast(Ray(eye, dir, maxDistance), hit, kVisionMask)) {
                // 什么都没打到：上半屏是天空，下半屏是远处的地面
                Put(x, y, ndcY > real(0) ? ' ' : '.',
                    ndcY > real(0) ? kDarkGray : kDarkGray);
                m_depth[static_cast<std::size_t>(y * m_width + x)] = real(1e9);
                continue;
            }

            //------------------------------------------------------------------
            // 明暗：按距离选字符
            //
            // 用平方根压缩而不是线性：近处的层次会更细腻，远处快速并进最暗一档。
            // 线性映射的话，整个画面会有一大半挤在最暗的两三个字符里。
            //------------------------------------------------------------------
            const real normalized = Clamp(Sqrt(hit.distance / shadeRange), real(0), real(1));
            int shade = static_cast<int>(normalized * static_cast<real>(kShadeCount));
            shade = shade < 0 ? 0 : (shade >= kShadeCount ? kShadeCount - 1 : shade);

            //------------------------------------------------------------------
            // 面朝向：给墙面加一点明暗差，不然所有墙糊成一片、看不出转角
            //
            // 法线越"正对"某个坐标轴，越亮 —— 这是最廉价的伪光照。
            //------------------------------------------------------------------
            const real facing = Abs(hit.normal.x) * real(0.75) +
                                Abs(hit.normal.y) * real(1.0) +
                                Abs(hit.normal.z) * real(0.5);
            if (facing < real(0.7) && shade < kShadeCount - 1) ++shade;

            //------------------------------------------------------------------
            // 颜色：按打中的是什么。
            //
            // 这里体现了 GameTypes.h 里那张 刚体 -> 实体 映射表的价值：
            // 引擎只告诉我们"打中了 BodyHandle{3,1}"，是游戏这一层把它翻译成
            // "那是个敌人"。
            //------------------------------------------------------------------
            std::uint8_t color = kGray;
            char glyph = kShades[shade];

            const Entity* entity = game.FindEntity(hit.body);
            if (entity != nullptr) {
                switch (entity->kind) {
                    case EntityKind::Enemy: {
                        color = kRed;
                        // 敌人用实心字符，远了也看得见 —— 玩法可读性优先于写实
                        glyph = (shade < kShadeCount - 3) ? '@' : 'A';
                        // 人形的头单独换个字符和颜色。字符网格只有十级灰度，
                        // 光靠明暗分不出头和肩膀，换字符才看得出这是个人
                        if (const Collider* part = world.GetCollider(hit.collider)) {
                            if (part->localTransform.position.y > real(0.6)) {
                                color = kYellow;
                                glyph = 'O';
                            }
                        }
                        break;
                    }
                    case EntityKind::Crate:
                        color = kYellow;
                        break;
                    case EntityKind::Debris:
                        // 碎块用固定字符：它们通常只有几个像素大，
                        // 走明暗梯度的话会淹没在背景里看不出来
                        color = entity->debrisKind == DebrisKind::Wood ? kYellow : kRed;
                        glyph = '%';
                        break;
                    case EntityKind::World:
                        // 地板（法线朝上）画暗一点，和墙区分开
                        color = (hit.normal.y > real(0.7)) ? kDarkGray : kGray;
                        break;
                    default:
                        break;
                }
            }

            Put(x, y, glyph, color);
            m_depth[static_cast<std::size_t>(y * m_width + x)] = hit.distance;
        }
    }

    //--------------------------------------------------------------------------
    // 特效叠在场景上，用的是和像素版**同一份**投影和同一份游戏数据 ——
    // 只是最后写下去的是字符而不是 RGB。这里再一次说明 ASCII 版不是
    // "简化版渲染"，它只是输出设备的色深低。
    //--------------------------------------------------------------------------
    DrawEffects(game, camera);
}

//------------------------------------------------------------------------------
// 特效：曳光弹与命中火花
//------------------------------------------------------------------------------
void Renderer::DrawEffects(const Game& game, const Camera& camera) {
    const auto occluded = [this](int x, int y, real depth) {
        if (x < 0 || x >= m_width || y < 0 || y >= m_height) return true;
        return depth > m_depth[static_cast<std::size_t>(y * m_width + x)];
    };

    for (const Tracer& tracer : game.Tracers()) {
        Vec3 from = tracer.from;
        Vec3 to = tracer.to;
        if (!ClipSegmentToCamera(camera, from, to)) continue;

        real x0 = 0, y0 = 0, d0 = 0, x1 = 0, y1 = 0, d1 = 0;
        if (!ProjectToScreen(camera, from, m_width, m_height, kAspectCorrection, x0, y0,
                             d0)) {
            continue;
        }
        if (!ProjectToScreen(camera, to, m_width, m_height, kAspectCorrection, x1, y1,
                             d1)) {
            continue;
        }

        const real dx = x1 - x0;
        const real dy = y1 - y0;
        const int steps = Max(1, static_cast<int>(Sqrt(dx * dx + dy * dy)));
        for (int i = 0; i <= steps; ++i) {
            const real t = static_cast<real>(i) / static_cast<real>(steps);
            const int px = static_cast<int>(x0 + dx * t + real(0.5));
            const int py = static_cast<int>(y0 + dy * t + real(0.5));
            if (occluded(px, py, d0 + (d1 - d0) * t)) continue;
            Put(px, py, tracer.hostile ? '!' : '-', tracer.hostile ? kRed : kYellow);
        }
    }

    for (const Impact& impact : game.Impacts()) {
        real sx = 0, sy = 0, depth = 0;
        if (!ProjectToScreen(camera, impact.position, m_width, m_height,
                             kAspectCorrection, sx, sy, depth)) {
            continue;
        }
        const int px = static_cast<int>(sx + real(0.5));
        const int py = static_cast<int>(sy + real(0.5));
        // 容差和像素版一致：火花贴在墙上，深度会和墙几乎相等
        if (occluded(px, py, depth - real(0.15))) continue;

        const std::uint8_t color =
            impact.target == EntityKind::Enemy ? kRed : kWhite;
        Put(px, py, '*', color);
        // 命中后半程再往外炸开一圈，让"打中了"这件事在字符画里也看得出来
        if (impact.life < impact.maxLife * real(0.6)) {
            Put(px - 1, py, '-', color);
            Put(px + 1, py, '-', color);
            Put(px, py - 1, '\'', color);
            Put(px, py + 1, ',', color);
        }
    }
}

//==============================================================================
// HUD
//==============================================================================

void Renderer::DrawHud(const Game& game) {
    const int cx = m_width / 2;
    const int cy = m_height / 2;

    //-- 准星 -------------------------------------------------------------------
    const std::uint8_t crossColor = game.MuzzleFlash() > real(0.1) ? kYellow : kWhite;
    Put(cx, cy, '+', crossColor);
    Put(cx - 2, cy, '-', crossColor);
    Put(cx + 2, cy, '-', crossColor);
    Put(cx, cy - 1, '|', crossColor);
    Put(cx, cy + 1, '|', crossColor);

    //-- 命中标记：真正扣到血的那一枪才出现 -------------------------------------
    if (game.HitMarker() > real(0)) {
        Put(cx - 3, cy - 1, char(92), kWhite);  // '\'
        Put(cx + 3, cy - 1, char(47), kWhite);  // '/'
        Put(cx - 3, cy + 1, char(47), kWhite);
        Put(cx + 3, cy + 1, char(92), kWhite);
    }

    //-- 受伤提示 ---------------------------------------------------------------
    // 只画四个角，不画整圈边框：整圈会盖掉大量画面内容，而且没有颜色的时候
    // 根本看不出那是"受伤"，只会以为墙就长那样。
    if (game.DamageFlash() > real(0.05)) {
        const int m = 3;
        for (int i = 0; i < m; ++i) {
            Put(i, 0, char(92), kRed);
            Put(m_width - 1 - i, 0, char(47), kRed);
            Put(i, m_height - 1, char(47), kRed);
            Put(m_width - 1 - i, m_height - 1, char(92), kRed);
        }
    }

    //-- 顶部状态栏 -------------------------------------------------------------
    const int hp = static_cast<int>(game.Health());
    const int hpMax = static_cast<int>(game.MaxHealth());

    std::string bar = "HP ";
    const int barLength = 20;
    const int filled = hpMax > 0 ? (hp * barLength) / hpMax : 0;
    for (int i = 0; i < barLength; ++i) bar += (i < filled) ? '=' : ' ';
    bar += " " + std::to_string(hp);

    const std::uint8_t hpColor =
        hp > hpMax / 2 ? kGreen : (hp > hpMax / 4 ? kYellow : kRed);
    PutText(1, 0, bar, hpColor);

    std::string ammo = game.Reloading()
                           ? std::string("RELOADING")
                           : ("AMMO " + std::to_string(game.Ammo()) + "/" +
                              std::to_string(game.MagazineSize()) + "  RSV " +
                              std::to_string(game.Reserve()));
    PutText(m_width - static_cast<int>(ammo.size()) - 1, 0, ammo,
            game.Ammo() == 0 ? kRed : kCyan);

    const std::string enemies = "ENEMY " + std::to_string(game.EnemiesAlive()) + "/" +
                                std::to_string(game.EnemiesTotal());
    PutText(m_width / 2 - static_cast<int>(enemies.size()) / 2, 0, enemies, kRed);

    //-- 底部：位置与日志 -------------------------------------------------------
    const Vec3 foot = game.Player().FootPosition();
    char buf[128];
    std::snprintf(buf, sizeof(buf), "(%.1f, %.1f, %.1f) %s  t=%.0fs", double(foot.x),
                  double(foot.y), double(foot.z),
                  game.Player().isGrounded ? "GROUND" : "AIR", double(game.ElapsedTime()));
    PutText(1, m_height - 1, buf, kDarkGray);

    const std::vector<std::string>& log = game.Log();
    int row = m_height - 2;
    for (auto it = log.rbegin(); it != log.rend() && row > m_height - 5; ++it, --row) {
        PutText(1, row, *it, kDarkGray);
    }
}

void Renderer::DrawCenteredText(int row, const std::string& text, int colorCode) {
    // 见 PutText 的说明：网格里只放 ASCII，所以字节数就是显示宽度
    const int start = (m_width - static_cast<int>(text.size())) / 2;
    PutText(start, row, text, static_cast<std::uint8_t>(colorCode));
}

//==============================================================================
// 输出
//==============================================================================

void Renderer::Present() const {
    m_out.clear();

    // 把光标移回左上角重画，而不是清屏 —— 清屏会闪
    m_out += "\x1b[H";

    std::uint8_t currentColor = 0;
    for (int y = 0; y < m_height; ++y) {
        for (int x = 0; x < m_width; ++x) {
            const Cell& cell = m_cells[static_cast<std::size_t>(y * m_width + x)];
            if (m_color && cell.color != currentColor) {
                currentColor = cell.color;
                m_out += "\x1b[";
                m_out += std::to_string(currentColor);
                m_out += 'm';
            }
            m_out += cell.glyph;
        }
        m_out += '\n';
    }
    if (m_color) m_out += "\x1b[0m";

    // 整帧一次写出。逐字符 putchar 在 Windows 控制台上慢到会闪。
    std::fwrite(m_out.data(), 1, m_out.size(), stdout);
    std::fflush(stdout);
}

}  // namespace game
