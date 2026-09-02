//==============================================================================
// game/PixelRenderer.cpp
//==============================================================================

#include "game/PixelRenderer.h"

#include <cstdio>

#include "game/Game.h"
#include "game/Renderer.h"  // Camera

namespace game {

namespace {

//------------------------------------------------------------------------------
// 调色板
//------------------------------------------------------------------------------
struct Rgb {
    int r, g, b;
};

constexpr Rgb kWallColor{150, 148, 140};
constexpr Rgb kFloorColor{88, 80, 70};
constexpr Rgb kCeilingColor{52, 54, 62};
constexpr Rgb kCrateColor{190, 140, 55};
constexpr Rgb kEnemyColor{205, 55, 45};      ///< 躯干和胳膊
constexpr Rgb kEnemyHeadColor{215, 170, 140};///< 头：肤色。低分辨率下这一点
                                             ///< 色差就足以让人形一眼读出来
constexpr Rgb kEnemyLegColor{110, 42, 38};   ///< 腿：暗一档，分出上下半身
constexpr Rgb kWoodDebrisColor{150, 105, 45};   ///< 木碎块：比整箱子暗一点
constexpr Rgb kFleshDebrisColor{140, 35, 35};
constexpr Rgb kFogColor{28, 30, 38};   ///< 远处渐隐到的颜色
constexpr Rgb kSkyColor{18, 20, 26};

constexpr real kFogStart = real(4);
constexpr real kFogEnd = real(34);

/// 线性插值两个颜色
inline Rgb Mix(const Rgb& a, const Rgb& b, real t) noexcept {
    const real u = Clamp(t, real(0), real(1));
    return Rgb{static_cast<int>(real(a.r) + (real(b.r) - real(a.r)) * u),
               static_cast<int>(real(a.g) + (real(b.g) - real(a.g)) * u),
               static_cast<int>(real(a.b) + (real(b.b) - real(a.b)) * u)};
}

inline Rgb Scale(const Rgb& c, real s) noexcept {
    return Rgb{static_cast<int>(real(c.r) * s), static_cast<int>(real(c.g) * s),
               static_cast<int>(real(c.b) * s)};
}

}  // namespace


//==============================================================================
// 场景
//==============================================================================

void PixelRenderer::RenderScene(const Game& game, const Camera& camera) {
    const PhysicsWorld& world = game.World();
    m_rayCount = 0;

    const int width = m_frame.Width();
    const int height = m_frame.Height();

    //--------------------------------------------------------------------------
    // 相机基。先算好三个方向向量，之后每个像素只做两次乘加。
    //--------------------------------------------------------------------------
    const Vec3 forward = camera.Forward();
    const Vec3 right = camera.Right();
    // "上"必须由 forward × right 叉出来。直接用世界 +Y 的话，
    // 抬头/低头时画面会产生剪切变形。
    const Vec3 up = Cross(right, forward).Normalized();

    const real tanHalfFovY = Tan(camera.fovY * real(0.5));
    // 这里是**真正的**方形像素，不需要 ASCII 版那个字符宽高比的补偿
    const real aspect = static_cast<real>(width) / static_cast<real>(height);
    const real tanHalfFovX = tanHalfFovY * aspect;

    const Vec3 eye = camera.position;
    const real maxDistance = real(60);

    for (int y = 0; y < height; ++y) {
        const real ndcY =
            real(1) - real(2) * (static_cast<real>(y) + real(0.5)) /
                          static_cast<real>(height);

        for (int x = 0; x < width; ++x) {
            const real ndcX =
                real(2) * (static_cast<real>(x) + real(0.5)) /
                    static_cast<real>(width) - real(1);

            const Vec3 dir =
                (forward + right * (ndcX * tanHalfFovX) + up * (ndcY * tanHalfFovY))
                    .Normalized();

            //------------------------------------------------------------------
            // 整个渲染器就是这一行。和 ASCII 版**完全一样**，
            // 区别只在于下面把结果写成 RGB 而不是写成一个字符。
            //------------------------------------------------------------------
            WorldRaycastHit hit;
            ++m_rayCount;
            if (!world.Raycast(Ray(eye, dir, maxDistance), hit, kVisionMask)) {
                m_frame.Set(x, y, Framebuffer::Pack(kSkyColor.r, kSkyColor.g, kSkyColor.b));
                m_depth[static_cast<std::size_t>(y * width + x)] = real(1e9);
                continue;
            }

            // 顺手记下深度。特效要靠它做遮挡 —— 光线投射本来就算出来了，
            // 存一下不额外花任何代价。
            m_depth[static_cast<std::size_t>(y * width + x)] = hit.distance;

            //------------------------------------------------------------------
            // 基色：按打中的是什么。
            // 这里用到了 刚体 -> 实体 的映射表 —— 引擎只说"打中了 BodyHandle{7,1}"，
            // 是游戏这一层把它翻译成"那是个敌人"。
            //------------------------------------------------------------------
            Rgb base = kWallColor;
            const Entity* entity = game.FindEntity(hit.body);
            if (entity != nullptr) {
                switch (entity->kind) {
                    case EntityKind::Enemy: {
                        //------------------------------------------------------
                        // 敌人是七块碰撞体拼的人形（见 Game::AddHumanoidColliders）。
                        // 打中的是哪一块，问引擎要那块碰撞体的**局部**高度就知道 ——
                        // 游戏这边不用另存一份"哪个句柄是头"的表。
                        //------------------------------------------------------
                        base = kEnemyColor;
                        if (const Collider* part = world.GetCollider(hit.collider)) {
                            const real localY = part->localTransform.position.y;
                            if (localY > real(0.6)) {
                                base = kEnemyHeadColor;
                            } else if (localY < real(-0.2)) {
                                base = kEnemyLegColor;
                            }
                        }
                        break;
                    }
                    case EntityKind::Crate: base = kCrateColor; break;
                    case EntityKind::Debris:
                        base = entity->debrisKind == DebrisKind::Wood
                                   ? kWoodDebrisColor
                                   : kFleshDebrisColor;
                        break;
                    case EntityKind::World:
                        // 地板朝上、天花板朝下，分别给不同的颜色 ——
                        // 靠法线区分，不需要知道是哪个刚体
                        if (hit.normal.y > real(0.7)) {
                            base = kFloorColor;
                        } else if (hit.normal.y < real(-0.7)) {
                            base = kCeilingColor;
                        }
                        break;
                    default: break;
                }
            }

            //------------------------------------------------------------------
            // 打光：一盏跟着玩家走的"矿灯" + 一点环境光。
            //
            // 用 |N·L| 而不是 max(N·L, 0)：光源就在眼睛位置，所有可见表面
            // 本来就都朝着我们，取绝对值可以省掉一次判断，结果一样。
            //------------------------------------------------------------------
            const real ndotl = Abs(Dot(hit.normal, dir));
            const real lighting = real(0.35) + real(0.65) * ndotl;

            // 竖直面比水平面暗一点，让墙角有明确的分界（廉价的方向光）
            const real facing = Abs(hit.normal.y) > real(0.7) ? real(1.0) : real(0.82);

            Rgb color = Scale(base, lighting * facing);

            //------------------------------------------------------------------
            // 距离雾：远处渐隐到背景色。它同时解决两件事 ——
            // 给画面纵深感，以及把射程边缘的突兀截断藏起来。
            //------------------------------------------------------------------
            const real fog =
                Clamp((hit.distance - kFogStart) / (kFogEnd - kFogStart), real(0), real(1));
            color = Mix(color, kFogColor, fog * fog);

            m_frame.Set(x, y, Framebuffer::Pack(color.r, color.g, color.b));
        }
    }

    //--------------------------------------------------------------------------
    // 世界里的特效叠在场景之上。
    //
    // 它们**不能**用光线投射画：曳光弹是一条没有厚度的线、火花是一团光，
    // 都不是能被射线打中的几何体。所以这里换成正向投影（世界点 -> 屏幕格子），
    // 再用刚存下来的深度缓冲做遮挡 —— 于是墙后面的枪火不会穿墙透出来。
    //--------------------------------------------------------------------------
    for (const Tracer& tracer : game.Tracers()) DrawTracer(camera, tracer);
    for (const Impact& impact : game.Impacts()) DrawImpact(camera, impact);
}

//==============================================================================
// 特效
//==============================================================================

real PixelRenderer::DepthAt(int x, int y) const {
    if (x < 0 || x >= m_frame.Width() || y < 0 || y >= m_frame.Height()) {
        return real(-1);  // 屏幕外：当成"被挡住"
    }
    return m_depth[static_cast<std::size_t>(y * m_frame.Width() + x)];
}

void PixelRenderer::DrawTracer(const Camera& camera, const Tracer& tracer) {
    Vec3 from = tracer.from;
    Vec3 to = tracer.to;
    if (!ClipSegmentToCamera(camera, from, to)) return;

    real x0 = 0, y0 = 0, d0 = 0, x1 = 0, y1 = 0, d1 = 0;
    if (!ProjectToScreen(camera, from, m_frame.Width(), m_frame.Height(), real(1), x0,
                         y0, d0)) {
        return;
    }
    if (!ProjectToScreen(camera, to, m_frame.Width(), m_frame.Height(), real(1), x1, y1,
                         d1)) {
        return;
    }

    // 越接近寿命末尾越暗。曳光弹只活两三帧，这一下淡出就是"嗖"的那个感觉。
    const real fade = Clamp(tracer.life / tracer.maxLife, real(0), real(1));
    const int r = static_cast<int>((tracer.hostile ? real(255) : real(255)) * fade);
    const int g = static_cast<int>((tracer.hostile ? real(90) : real(215)) * fade);
    const int b = static_cast<int>((tracer.hostile ? real(60) : real(120)) * fade);

    //--------------------------------------------------------------------------
    // 沿线段等距取样，而不是 Bresenham：这里同时要在两个端点之间插值**深度**，
    // 按参数 t 走一遍最直接。步长取 1 像素，画出来是连续的。
    //--------------------------------------------------------------------------
    const real dx = x1 - x0;
    const real dy = y1 - y0;
    const real length = Sqrt(dx * dx + dy * dy);
    const int steps = Max(1, static_cast<int>(length));

    for (int i = 0; i <= steps; ++i) {
        const real t = static_cast<real>(i) / static_cast<real>(steps);
        const int px = static_cast<int>(x0 + dx * t + real(0.5));
        const int py = static_cast<int>(y0 + dy * t + real(0.5));
        // 深度也要跟着插值，不然一条斜穿房间的弹道会被整条剔掉或者整条穿墙
        const real depth = d0 + (d1 - d0) * t;
        if (depth > DepthAt(px, py)) continue;

        m_frame.AddLight(px, py, r, g, b);
        // 上下各补一格暗一些的余辉，线看起来才有粗细、不至于在颗粒感里消失
        m_frame.AddLight(px, py - 1, r / 4, g / 4, b / 4);
        m_frame.AddLight(px, py + 1, r / 4, g / 4, b / 4);
    }
}

void PixelRenderer::DrawImpact(const Camera& camera, const Impact& impact) {
    real sx = 0, sy = 0, depth = 0;
    if (!ProjectToScreen(camera, impact.position, m_frame.Width(), m_frame.Height(),
                         real(1), sx, sy, depth)) {
        return;
    }

    const int cx = static_cast<int>(sx + real(0.5));
    const int cy = static_cast<int>(sy + real(0.5));
    // 火花点已经沿法线往外挪过 2 厘米（见 Game::SpawnImpact），这里再放宽一点
    // 深度容差，免得贴着墙的火花被自己打中的那面墙挡掉
    if (depth - real(0.15) > DepthAt(cx, cy)) return;

    //--------------------------------------------------------------------------
    // 一次命中的火花分两部分：一个由亮变暗的核心，加上一圈往外飞散的火星。
    // "先亮后散"是所有撞击特效的通用形状 —— 它模仿的是能量在一瞬间释放完。
    //--------------------------------------------------------------------------
    const real age = Clamp(real(1) - impact.life / impact.maxLife, real(0), real(1));
    const real fade = real(1) - age;

    // 屏幕上的大小按距离缩：近处打一枪是一团火，二十米外只是一个亮点
    const real size = Clamp(real(26) / depth, real(1.5), real(11));

    // 材质决定颜色：石头墙是白黄色的火星，木箱是橙色木屑，打在敌人身上是血红
    int r = 255, g = 225, b = 150;
    if (impact.target == EntityKind::Crate || impact.target == EntityKind::Debris) {
        r = 235; g = 165; b = 70;
    } else if (impact.target == EntityKind::Enemy) {
        r = 255; g = 60; b = 45;
    }

    const int core = Max(1, static_cast<int>(size * real(0.35) * fade));
    for (int j = -core; j <= core; ++j) {
        for (int i = -core; i <= core; ++i) {
            if (i * i + j * j > core * core) continue;
            m_frame.AddLight(cx + i, cy + j, static_cast<int>(real(r) * fade),
                     static_cast<int>(real(g) * fade), static_cast<int>(real(b) * fade));
        }
    }

    // 火星：八个方向，随时间往外飞、同时变暗
    const real radius = size * (real(0.35) + age * real(1.1));
    for (int k = 0; k < 8; ++k) {
        const real angle = real(k) * real(0.785398);  // 2*pi/8
        const int px = cx + static_cast<int>(Cos(angle) * radius);
        const int py = cy + static_cast<int>(Sin(angle) * radius);
        const real sparkFade = fade * fade;
        m_frame.AddLight(px, py, static_cast<int>(real(r) * sparkFade),
                 static_cast<int>(real(g) * sparkFade),
                 static_cast<int>(real(b) * sparkFade));
    }
}

//==============================================================================
// HUD
//==============================================================================


//------------------------------------------------------------------------------
// 手里的枪
//
// 为什么值得画：第一人称视角里，枪是**唯一**一直在画面上、且完全属于玩家的
// 东西。它开火时往后一顿、枪口喷一团光，这一下就把"我开了一枪"从一个数字
// 变成了一个动作。没有它，屏幕上除了准星变个色什么都没发生。
//
// 它是纯粹的 HUD 贴图（几个矩形），不参与光线投射也不参与物理 ——
// 真实 FPS 里的手部模型同样是这么处理的：单独一层，永远画在最前面。
//------------------------------------------------------------------------------
void PixelRenderer::DrawWeapon(const Game& game) {
    const int width = m_frame.Width();
    const int height = m_frame.Height();

    const std::uint32_t metal = Framebuffer::Pack(58, 60, 68);
    const std::uint32_t metalDark = Framebuffer::Pack(38, 39, 45);
    const std::uint32_t metalLit = Framebuffer::Pack(104, 109, 119);
    const std::uint32_t polymer = Framebuffer::Pack(52, 48, 44);
    const std::uint32_t glove = Framebuffer::Pack(96, 76, 58);
    const std::uint32_t gloveLit = Framebuffer::Pack(126, 101, 78);

    //--------------------------------------------------------------------------
    // 整把枪只由一条**轴**决定：从后膛（画面右下、贴着画面外）指向枪口
    // （画面中偏上）。所有零件都挂在这条轴的某个位置上。
    //
    // 这一条就是"看着像枪"和"看着像一根横棍"的全部区别：第一人称里枪是朝着
    // 屏幕**里面**指的，投影出来必然是一条斜的、越远越细的形状。之前那版把
    // 枪管画成了一条水平矩形，等于把枪横过来端着。
    //--------------------------------------------------------------------------
    // 后坐：开火瞬间整把枪沿着轴往后（右下）一顿，然后弹回来。
    // MuzzleFlash 本身就是个衰减的 1 -> 0，直接拿来当后坐曲线用。
    const real kick = game.MuzzleFlash();

    //--------------------------------------------------------------------------
    // 整把枪的大小只有这**一个**数。
    //
    // 下面所有的长度、宽度、偏移都写成"满尺寸下的像素数"，再统一乘上它 ——
    // 于是调大调小不会把比例调歪（手比枪大、弹匣比机匣宽之类）。
    // 视野占比是纯粹的观感问题，本来就该是一个可以随手改的旋钮。
    //--------------------------------------------------------------------------
    constexpr real kWeaponScale = real(0.66);

    //--------------------------------------------------------------------------
    // 缩放的锚点是**后膛**，不是枪口。
    //
    // 枪是从画面右下角伸进来的（那是你的肩膀所在的方向），所以缩小的时候
    // 该往那个角上收。锚在枪口的话，缩小会把后膛拽到画面中央，变成一把
    // 飘在屏幕正中的小枪。
    //
    // 后膛本身故意留在画面外：枪托抵在你自己肩上，不该出现在视野里。
    //--------------------------------------------------------------------------
    const Vec3 breech(static_cast<real>(width) * real(0.90) + kick * real(4) * kWeaponScale,
                      static_cast<real>(height) * real(0.98) + kick * real(6) * kWeaponScale,
                      real(0));
    const Vec3 muzzle(breech.x - real(88) * kWeaponScale, breech.y - real(62) * kWeaponScale,
                      real(0));

    const real length = Sqrt((muzzle.x - breech.x) * (muzzle.x - breech.x) +
                             (muzzle.y - breech.y) * (muzzle.y - breech.y));
    const Vec3 dir((muzzle.x - breech.x) / length, (muzzle.y - breech.y) / length,
                   real(0));
    // 垂直于枪管、指向枪的"上面"（屏幕上是右上方）。整把枪的上下左右都由它定，
    // 于是姿态也只有一个参数：改 muzzle 相对 breech 的偏移，全枪一起转。
    const Vec3 up(-dir.y, dir.x, real(0));

    /// 沿枪管走到 t（0 = 后膛，1 = 枪口），再往枪的上方偏 offset 像素（满尺寸）
    const auto P = [&](real t, real offset) {
        return Vec3(breech.x + dir.x * length * t + up.x * offset * kWeaponScale,
                    breech.y + dir.y * length * t + up.y * offset * kWeaponScale, real(0));
    };
    /// 宽度同样按满尺寸写，这里统一缩
    const auto Bar = [&](const Vec3& a, real wa, const Vec3& b, real wb,
                         std::uint32_t color) {
        m_frame.TaperedBar(a.x, a.y, wa * kWeaponScale, b.x, b.y, wb * kWeaponScale, color);
    };

    //--------------------------------------------------------------------------
    // 先铺一层比本体宽 4 像素的近黑色，当作**描边**。
    //
    // 这不是风格化：场景本身就是暗的，枪也是暗的，不描边的话枪身会和地板糊在
    // 一起，只剩高光那一条线能看见。低分辨率的东西要读得出形状，靠的就是轮廓。
    //--------------------------------------------------------------------------
    const std::uint32_t outline = Framebuffer::Pack(12, 12, 15);
    Bar(P(real(0.32), real(-6)), real(21), P(real(0.40), real(-32)), real(18), outline);
    Bar(P(real(0.16), real(-4)), real(19), P(real(0.07), real(-28)), real(17), outline);
    Bar(breech, real(35), P(real(0.75), real(0)), real(18), outline);
    Bar(P(real(0.75), real(0)), real(14), P(real(1.03), real(0)), real(12), outline);

    //-- 弹匣：从机匣下方伸出来，微微前倾 ---------------------------------------
    Bar(P(real(0.32), real(-6)), real(17), P(real(0.40), real(-32)), real(14), polymer);
    // 侧面一道暗缝，弹匣才不是糊成一整块
    Bar(P(real(0.32), real(-6)), real(4), P(real(0.40), real(-32)), real(3), metalDark);

    //-- 握把 -------------------------------------------------------------------
    Bar(P(real(0.16), real(-4)), real(15), P(real(0.07), real(-28)), real(13), polymer);

    //-- 机匣 -> 护木 -> 枪管：越远越细，这就是透视 -----------------------------
    Bar(breech, real(31), P(real(0.40), real(0)), real(22), metal);
    Bar(P(real(0.40), real(0)), real(20), P(real(0.75), real(0)), real(14), polymer);
    Bar(P(real(0.75), real(0)), real(10), muzzle, real(8), metalDark);

    // 上沿高光：贴着枪身上边的一条细线。廉价，但它让枪从"一块死色"变成有厚度的
    // 东西 —— 这个分辨率下几乎是唯一能表达体积的手段
    Bar(P(real(0.05), real(12)), real(4), P(real(0.98), real(3.5)), real(2), metalLit);

    //-- 枪口装置、准星、照门 ---------------------------------------------------
    Bar(P(real(0.95), real(0)), real(13), P(real(1.02), real(0)), real(12), metal);
    // 准星座和照门：两个立在枪身上方的小块。真枪的剪影里这两块最显眼，
    // 而且它们连成的线正好指着准星
    Bar(P(real(0.90), real(4)), real(5), P(real(0.90), real(12)), real(4), metalDark);
    Bar(P(real(0.36), real(9)), real(8), P(real(0.36), real(17)), real(6), metalDark);

    //-- 握枪的两只手 -----------------------------------------------------------
    //
    // 手不是装饰。第一人称里"这把枪是我的"这件事全靠手来说 ——
    // 没有手的枪看着像一张浮在屏幕前面的 HUD 贴图。
    // 前手：手指从下面绕上来扣住护木，所以主体偏在枪管**下方**
    Bar(P(real(0.63), real(6)), real(17), P(real(0.57), real(-18)), real(20), outline);
    Bar(P(real(0.62), real(5)), real(13), P(real(0.57), real(-16)), real(16), glove);
    Bar(P(real(0.64), real(2)), real(5), P(real(0.60), real(-10)), real(4), gloveLit);
    // 一道暗缝当作手指之间的分界，不然是一整块木头
    Bar(P(real(0.585), real(4)), real(2), P(real(0.565), real(-14)), real(2), polymer);

    // 后手：握着握把，大半被画面下沿裁掉 —— 真实 FPS 模型也是这样
    Bar(P(real(0.19), real(2)), real(22), P(real(0.10), real(-22)), real(19), outline);
    Bar(P(real(0.19), real(1)), real(18), P(real(0.10), real(-21)), real(16), glove);
    Bar(P(real(0.17), real(-2)), real(7), P(real(0.11), real(-18)), real(6), gloveLit);

    //-- 枪口焰：从枪口**沿着枪管**喷出去，不是在原地开一朵花 -------------------
    if (kick > real(0.15)) {
        const real reach = (kick * real(13) + real(4)) * kWeaponScale;
        const int span = static_cast<int>(real(16) * kWeaponScale) + 2;
        for (int j = -span; j <= span; ++j) {
            for (int i = -span; i <= span; ++i) {
                // 换算到"沿枪管 / 垂直枪管"的坐标：沿枪管拉长、垂直方向收窄，
                // 才是一团从枪口喷出去的火，而不是一个圆点
                const real along = real(i) * dir.x + real(j) * dir.y;
                const real across = real(i) * up.x + real(j) * up.y;
                // 往前拉长、往后几乎不出来：枪口焰是**朝前喷**的，
                // 对称的一团光看着像枪口上挂了个灯泡
                const real stretch = along > real(0) ? real(0.45) : real(3.0);
                const real d =
                    Sqrt(along * along * stretch + across * across * real(2.4));
                if (d > reach) continue;
                const real fade = (real(1) - d / reach) * (real(1) - d / reach) * kick;
                m_frame.AddLight(static_cast<int>(muzzle.x) + i, static_cast<int>(muzzle.y) + j,
                         static_cast<int>(real(255) * fade),
                         static_cast<int>(real(205) * fade),
                         static_cast<int>(real(95) * fade));
            }
        }
    }
}

//------------------------------------------------------------------------------
// 命中标记
//
// 四道斜杠围住准星，只在真正扣到血的那一枪出现。这在 FPS 里的作用被严重
// 低估：没有它，玩家分不清"打中了但没打死"和"打偏了"，只能靠猜。
//------------------------------------------------------------------------------
void PixelRenderer::DrawHitMarker(const Game& game, int cx, int cy) {
    const real strength = Clamp(game.HitMarker() / Max(game.Config().hitMarkerLifetime,
                                                       real(0.001)),
                                real(0), real(1));
    if (strength <= real(0)) return;

    const int r = static_cast<int>(real(255) * strength);
    const int g = static_cast<int>(real(230) * strength);
    const int b = static_cast<int>(real(210) * strength);

    for (int i = 3; i <= 6; ++i) {
        m_frame.AddLight(cx - i, cy - i, r, g, b);
        m_frame.AddLight(cx + i, cy - i, r, g, b);
        m_frame.AddLight(cx - i, cy + i, r, g, b);
        m_frame.AddLight(cx + i, cy + i, r, g, b);
    }
}

void PixelRenderer::DrawBanner(const std::string& text, const std::string& hint,
                               std::uint32_t color) {
    const int width = m_frame.Width();
    const int height = m_frame.Height();

    constexpr int kTextScale = 3;
    const int textWidth = static_cast<int>(text.size()) * 4 * kTextScale;
    const int y = height / 2 - 20;
    const int bandHeight = 5 * kTextScale + 12 + (hint.empty() ? 0 : 14);

    // 先压暗一层底，字才在乱七八糟的场景上读得出来
    m_frame.Rect(0, y - 6, width, bandHeight, Framebuffer::Pack(0, 0, 0));
    m_frame.Text((width - textWidth) / 2, y, text, color, kTextScale);

    if (!hint.empty()) {
        const int hintWidth = static_cast<int>(hint.size()) * 4;
        m_frame.Text((width - hintWidth) / 2, y + 5 * kTextScale + 5, hint,
                 Framebuffer::Pack(180, 180, 185));
    }
}

void PixelRenderer::DrawHud(const Game& game) {
    const int width = m_frame.Width();
    const int height = m_frame.Height();
    const int cx = width / 2;
    const int cy = height / 2;

    // 枪画在最前面，但要在准星和红晕之前 —— 准星必须压在枪上，
    // 受伤红晕则要连枪一起罩住，否则受伤时枪看起来像是浮在画面外面的
    DrawWeapon(game);

    const std::uint32_t white = Framebuffer::Pack(235, 235, 235);
    const std::uint32_t dim = Framebuffer::Pack(30, 32, 38);
    const std::uint32_t red = Framebuffer::Pack(210, 60, 50);
    const std::uint32_t green = Framebuffer::Pack(90, 200, 90);
    const std::uint32_t cyan = Framebuffer::Pack(90, 190, 210);

    //-- 准星 -------------------------------------------------------------------
    const bool firing = game.MuzzleFlash() > real(0.15);
    const std::uint32_t crossColor = firing ? Framebuffer::Pack(255, 220, 120) : white;
    const int gap = 3;
    const int len = 4;
    m_frame.Rect(cx - gap - len, cy, len, 1, crossColor);
    m_frame.Rect(cx + gap + 1, cy, len, 1, crossColor);
    m_frame.Rect(cx, cy - gap - len, 1, len, crossColor);
    m_frame.Rect(cx, cy + gap + 1, 1, len, crossColor);

    DrawHitMarker(game, cx, cy);

    //-- 枪口闪光：屏幕下方一小片暖光 -------------------------------------------
    if (firing) {
        const int flashHeight = 10;
        for (int j = 0; j < flashHeight; ++j) {
            const real t = real(1) - real(j) / real(flashHeight);
            const int a = static_cast<int>(t * real(70) * game.MuzzleFlash());
            for (int i = 0; i < width; ++i) {
                const std::uint32_t p = m_frame.Get(i, height - 1 - j);
                const int r = static_cast<int>((p >> 16) & 0xFF) + a;
                const int g = static_cast<int>((p >> 8) & 0xFF) + a * 3 / 4;
                const int b = static_cast<int>(p & 0xFF) + a / 3;
                m_frame.Set(i, height - 1 - j, Framebuffer::Pack(r, g, b));
            }
        }
    }

    //-- 受伤红晕：屏幕四周压一层红 ---------------------------------------------
    if (game.DamageFlash() > real(0.02)) {
        const real strength = game.DamageFlash();
        const int band = height / 5;
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                // 到边缘的距离越近，红得越厉害
                const int dx = Min(x, width - 1 - x);
                const int dy = Min(y, height - 1 - y);
                const int edge = Min(dx, dy);
                if (edge >= band) continue;
                const real t = (real(1) - real(edge) / real(band)) * strength;
                const std::uint32_t p = m_frame.Get(x, y);
                const int r = static_cast<int>((p >> 16) & 0xFF);
                const int g = static_cast<int>((p >> 8) & 0xFF);
                const int b = static_cast<int>(p & 0xFF);
                m_frame.Set(x, y,
                            Framebuffer::Pack(static_cast<int>(real(r) + (real(200) - real(r)) * t),
                                              static_cast<int>(real(g) * (real(1) - t * real(0.8))),
                                              static_cast<int>(real(b) * (real(1) - t * real(0.8)))));
            }
        }
    }

    //-- 血条 -------------------------------------------------------------------
    const real hpFraction = game.Health() / Max(game.MaxHealth(), real(1));
    m_frame.Rect(3, height - 12, 62, 9, dim);
    m_frame.Bar(4, height - 11, 60, 7, hpFraction, hpFraction > real(0.3) ? green : red, dim);
    m_frame.Text(6, height - 20, "HP " + std::to_string(static_cast<int>(game.Health())),
             white);

    //-- 弹药 -------------------------------------------------------------------
    const std::string ammo =
        game.Reloading() ? std::string("RELOAD")
                         : (std::to_string(game.Ammo()) + "/" +
                            std::to_string(game.Reserve()));
    m_frame.Text(width - 4 - static_cast<int>(ammo.size()) * 4, height - 12, ammo,
             game.Ammo() == 0 ? red : cyan);

    //-- 剩余敌人 ---------------------------------------------------------------
    const std::string enemies = std::to_string(game.EnemiesAlive()) + "/" +
                                std::to_string(game.EnemiesTotal());
    m_frame.Text(width - 4 - static_cast<int>(enemies.size()) * 4, 5, enemies, red);
    m_frame.Text(width - 4 - 5 * 4 - static_cast<int>(enemies.size()) * 4, 5, "ENEMY", red);
}

}  // namespace game
