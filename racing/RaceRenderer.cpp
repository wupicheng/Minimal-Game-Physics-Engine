//==============================================================================
// racing/RaceRenderer.cpp
//==============================================================================

#include "racing/RaceRenderer.h"

#include <cmath>
#include <cstdio>

namespace racing {

namespace {

using platform::Canvas;

//------------------------------------------------------------------------------
// 调色板
//------------------------------------------------------------------------------
struct Rgb {
    int r, g, b;
};

constexpr Rgb kAsphalt{58, 58, 62};
constexpr Rgb kAsphaltDark{46, 46, 50};
constexpr Rgb kLineWhite{205, 205, 195};
constexpr Rgb kKerbRed{170, 60, 55};
constexpr Rgb kWall{140, 140, 146};
constexpr Rgb kWallTop{175, 175, 180};
constexpr Rgb kRamp{120, 108, 80};
constexpr Rgb kCar{200, 45, 40};
constexpr Rgb kCarCabin{48, 56, 68};   ///< 座舱：深色，读起来像车窗
constexpr Rgb kCarWing{34, 34, 38};    ///< 尾翼
constexpr Rgb kWheel{28, 28, 30};
constexpr Rgb kProp{170, 130, 60};
constexpr Rgb kGate{225, 190, 60};
constexpr Rgb kGrass{48, 62, 44};
constexpr Rgb kSkyHigh{58, 92, 150};
constexpr Rgb kSkyLow{150, 172, 200};
constexpr Rgb kFog{120, 138, 160};

constexpr real kFogStart = real(30);
constexpr real kFogEnd = real(100);

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

inline std::uint32_t Pack(const Rgb& c) noexcept { return Canvas::Pack(c.r, c.g, c.b); }

//------------------------------------------------------------------------------
// 路面的颜色是**算**出来的，不是贴图，也不是几何。
//
// 整条赛道就是一块大平板（见 RaceGame::BuildTrack），所以路面上的一切标记
// 只能由命中点的坐标算出来。这反而是这个渲染器最合适的做法：
// 逐像素光线投射本来就给了每个像素精确的世界坐标，拿来做程序化纹理是免费的。
//
// 画三样东西，每一样都在回答玩家的一个问题：
//   - 赛道边缘的白线 -> 路在哪儿结束（护墙很矮，远处看不清）
//   - 起点线的方格   -> 终点在哪儿
//   - 赛道外的草地   -> 这里不是路，别开出去
//------------------------------------------------------------------------------
Rgb RoadColor(const Vec3& point) {
    // 赛道是一圈"椭圆环"，用两个轴向的归一化距离判断在不在环内。
    // 这和搭赛道时用的是同一组尺寸（见 RaceGame.cpp 顶部的常量）。
    constexpr real outerX = real(55);
    constexpr real outerZ = real(35);
    constexpr real innerX = real(39);
    constexpr real innerZ = real(19);

    const real ox = Abs(point.x) / outerX;
    const real oz = Abs(point.z) / outerZ;
    const real ix = Abs(point.x) / innerX;
    const real iz = Abs(point.z) / innerZ;
    const real outside = Max(ox, oz);   // >1 表示在外圈之外
    const real inside = Max(ix, iz);    // <1 表示在内圈之内

    if (outside > real(1.02) || inside < real(0.98)) return kGrass;

    // 边缘白线：贴着内外边界的一条窄带
    if (outside > real(0.985) || inside < real(1.015)) {
        // 红白相间的路肩：沿着周长每隔几米换一次颜色
        const real along = point.x + point.z;
        const bool red = static_cast<int>(Abs(along) / real(3)) % 2 == 0;
        return red ? kKerbRed : kLineWhite;
    }

    // 起点线：终点线在 z = 27 附近、x 靠近 0 的一段方格
    if (point.z > real(25.4) && point.z < real(28.6) && Abs(point.x) < real(2.2)) {
        const bool dark = (static_cast<int>((point.x + real(100)) / real(1.1)) +
                           static_cast<int>((point.z + real(100)) / real(1.1))) % 2 == 0;
        return dark ? Rgb{30, 30, 32} : Rgb{210, 210, 205};
    }

    // 路面本身：加一点点随位置变化的噪声，免得整片死板
    const bool patch = (static_cast<int>(point.x / real(7)) +
                        static_cast<int>(point.z / real(7))) % 2 == 0;
    return patch ? kAsphalt : kAsphaltDark;
}

}  // namespace

//==============================================================================
// 追车相机
//==============================================================================

Vec3 ChaseCamera::Forward() const noexcept {
    const Vec3 delta = target - position;
    const real length = delta.Length();
    return length > real(1e-4) ? delta * (real(1) / length) : Vec3(0, 0, real(1));
}

Vec3 ChaseCamera::Right() const noexcept {
    // 用世界 +Y 叉出来的右方向：相机不做侧倾，画面才不会跟着车一起翻。
    const Vec3 forward = Forward();
    const Vec3 right = Cross(Vec3(0, real(1), 0), forward);
    const real length = right.Length();
    return length > real(1e-4) ? right * (real(1) / length) : Vec3(real(1), 0, 0);
}

Vec3 ChaseCamera::Up() const noexcept { return Cross(Forward(), Right()).Normalized(); }

void ChaseCamera::Follow(const Vehicle& car, real dt) {
    const real speed = car.Speed();

    //--------------------------------------------------------------------------
    // 相机挂在车的**行进方向**后面，而不是车头朝向后面。
    //
    // 这两者在漂移时能差出四十度：车头朝着弯心，车却在往外滑。跟着车头的话，
    // 画面会转向弯心，玩家反而看不见自己正在滑向的那面墙 —— 而那才是他要
    // 判断的东西。跟着行进方向，漂移时就能看到车横过来的侧面，姿态一目了然。
    //--------------------------------------------------------------------------
    Vec3 heading = car.Velocity();
    heading.y = real(0);
    if (heading.Length() < real(2)) {
        // 车快停了，速度方向没有意义，退回车头朝向
        heading = car.Forward();
        heading.y = real(0);
    }
    heading = heading.Normalized();

    // 速度越快，相机拉得越远、抬得越高 —— 速度感主要来自这里
    const real distance = real(7.5) + Clamp(speed * real(0.10), real(0), real(3.5));
    const real height = real(3.0) + Clamp(speed * real(0.02), real(0), real(1.2));

    const Vec3 desiredPosition =
        car.Position() - heading * distance + Vec3(0, height, 0);
    // 看向车的前方一点：玩家要看的是即将到达的地方，不是车尾
    const Vec3 desiredTarget = car.Position() + heading * real(6) + Vec3(0, real(0.8), 0);

    if (!m_initialised) {
        position = desiredPosition;
        target = desiredTarget;
        m_initialised = true;
        return;
    }

    //--------------------------------------------------------------------------
    // 指数平滑，而且**和帧率无关**：
    //     alpha = 1 - exp(-rate * dt)
    // 直接写 `pos += (want - pos) * 0.1` 的话，帧率一变平滑强度就变，
    // 60 帧和 30 帧下的跟随手感会完全不同。
    //--------------------------------------------------------------------------
    const auto smooth = [dt](const Vec3& current, const Vec3& desired, real rate) {
        const real alpha =
            real(1) - static_cast<real>(std::exp(-double(rate) * double(dt)));
        return current + (desired - current) * alpha;
    };
    position = smooth(position, desiredPosition, real(7));
    // 注视点跟得比机位快一点，画面才不会"甩不回来"
    target = smooth(target, desiredTarget, real(11));
}

//==============================================================================
// 场景
//==============================================================================

void RaceRenderer::RenderScene(const RaceGame& race, const ChaseCamera& camera) {
    const PhysicsWorld& world = race.World();
    m_rayCount = 0;

    const int width = m_canvas.Width();
    const int height = m_canvas.Height();

    const Vec3 forward = camera.Forward();
    const Vec3 right = camera.Right();
    const Vec3 up = camera.Up();

    const real tanHalfFovY = Tan(camera.fovY * real(0.5));
    const real tanHalfFovX = tanHalfFovY * static_cast<real>(width) /
                             static_cast<real>(height);

    const Vec3 eye = camera.position;
    // 射程 110 米就够：雾在 100 米处已经把画面完全糊成背景色了，再往前打
    // 只是在宽相位格子里空跑。这一个数把渲染时间砍掉了三分之一 ——
    // 逐像素光线投射的开销和射程是**线性**关系（DDA 要走多少个格子）。
    const real maxDistance = real(110);

    for (int y = 0; y < height; ++y) {
        const real ndcY = real(1) - real(2) * (static_cast<real>(y) + real(0.5)) /
                                        static_cast<real>(height);
        for (int x = 0; x < width; ++x) {
            const real ndcX = real(2) * (static_cast<real>(x) + real(0.5)) /
                                  static_cast<real>(width) - real(1);

            const Vec3 dir =
                (forward + right * (ndcX * tanHalfFovX) + up * (ndcY * tanHalfFovY))
                    .Normalized();

            //------------------------------------------------------------------
            // 整个渲染器就是这一行 —— 和 FPS 那边一模一样
            //------------------------------------------------------------------
            WorldRaycastHit hit;
            ++m_rayCount;
            if (!world.Raycast(Ray(eye, dir, maxDistance), hit, kVisionMask)) {
                // 天空：抬头深、地平线浅
                const real t = Clamp(dir.y * real(2.2), real(0), real(1));
                const Rgb sky = Mix(kSkyLow, kSkyHigh, t);
                m_canvas.Set(x, y, Pack(sky));
                continue;
            }

            Rgb base = kWall;
            const Entity* entity = race.FindEntity(hit.body);
            if (entity != nullptr) {
                switch (entity->kind) {
                    case EntityKind::Road:
                        // 朝上的面才是路，侧面是路基
                        base = hit.normal.y > real(0.7) ? RoadColor(hit.point)
                                                        : kAsphaltDark;
                        break;
                    case EntityKind::Wall:
                        base = hit.normal.y > real(0.7) ? kWallTop : kWall;
                        break;
                    case EntityKind::Ramp: base = kRamp; break;
                    case EntityKind::Car: {
                        //------------------------------------------------------
                        // 车壳是几块盒子拼的（见 Vehicle::Spawn）。打中的是
                        // 哪一块，问引擎要那块碰撞体的**局部高度**就知道 ——
                        // 和 FPS 里给人形的头/躯干/腿上色是同一个办法，
                        // 游戏这边不用另存一张"哪个句柄是座舱"的表。
                        //------------------------------------------------------
                        base = kCar;
                        if (const Collider* part = world.GetCollider(hit.collider)) {
                            const real localY = part->localTransform.position.y;
                            if (localY > real(0.7)) {
                                base = kCarWing;    // 尾翼
                            } else if (localY > real(0.4)) {
                                base = kCarCabin;   // 座舱：深色，像车窗
                            }
                        }
                        // 车顶亮、侧面暗，低分辨率下这点差别就能看出车的姿态
                        if (hit.normal.y <= real(0.5)) base = Scale(base, real(0.72));
                        break;
                    }
                    case EntityKind::Wheel: base = kWheel; break;
                    case EntityKind::Prop: base = kProp; break;
                    case EntityKind::Gate: base = kGate; break;
                    default: break;
                }
            }

            //------------------------------------------------------------------
            // 打光：一盏斜上方的太阳 + 环境光。
            // 户外场景不能用 FPS 那盏"跟着眼睛走的矿灯"—— 那样所有面一样亮，
            // 赛道会糊成一整片，完全看不出护墙的立体感。
            //------------------------------------------------------------------
            const Vec3 sun = Vec3(real(0.45), real(0.8), real(0.35)).Normalized();
            const real lambert = Max(real(0), Dot(hit.normal, sun));
            Rgb color = Scale(base, real(0.55) + real(0.45) * lambert);

            const real fog =
                Clamp((hit.distance - kFogStart) / (kFogEnd - kFogStart), real(0), real(1));
            color = Mix(color, kFog, fog * fog);

            m_canvas.Set(x, y, Pack(color));
        }
    }
}

//==============================================================================
// HUD
//==============================================================================

namespace {

std::string FormatTime(real seconds) {
    if (seconds <= real(0)) return "--.--";
    char buffer[32];
    const int minutes = static_cast<int>(seconds) / 60;
    const real rest = seconds - static_cast<real>(minutes * 60);
    if (minutes > 0) {
        std::snprintf(buffer, sizeof(buffer), "%d:%05.2f", minutes, double(rest));
    } else {
        std::snprintf(buffer, sizeof(buffer), "%.2f", double(rest));
    }
    return buffer;
}

}  // namespace

void RaceRenderer::DrawHud(const RaceGame& race) {
    const int width = m_canvas.Width();
    const int height = m_canvas.Height();

    const std::uint32_t white = Canvas::Pack(235, 235, 235);
    const std::uint32_t dim = Canvas::Pack(24, 26, 30);
    const std::uint32_t amber = Canvas::Pack(240, 190, 70);
    const std::uint32_t red = Canvas::Pack(225, 70, 60);
    const std::uint32_t green = Canvas::Pack(120, 220, 120);

    const Vehicle& car = race.Car();

    //-- 速度表：一个数字 + 一条随速度增长的条 ----------------------------------
    // 三块东西的纵向位置要**互不重叠**：大数字、单位、速度条。
    // 第一版单位和速度条压在了一起，"KMH"被条盖掉一半，看着像乱码。
    const int speed = static_cast<int>(car.SpeedKmh() + real(0.5));
    const std::string speedText = std::to_string(speed);
    m_canvas.Rect(4, height - 30, 76, 26, dim);
    m_canvas.Text(8, height - 26, speedText, white, 3);
    m_canvas.Text(8 + Canvas::TextWidth(speedText, 3) + 4, height - 16, "KMH", amber, 1);
    m_canvas.Bar(8, height - 9, 64, 3,
                 car.Speed() / Max(car.Config().maxSpeed, real(1)), amber,
                 Canvas::Pack(60, 62, 68));

    //-- 圈数与计时 -------------------------------------------------------------
    m_canvas.Rect(width - 82, 4, 78, 30, dim);
    const std::string lapText =
        // 注意用 int 版的下限，不要写 Max(1, ...)：pe::Max 是 real 的，
        // 结果会变成 "LAP 1.000000/3"
        "LAP " + std::to_string(race.Lap() < 1 ? 1 : race.Lap()) + "/" +
        std::to_string(race.TotalLaps());
    m_canvas.Text(width - 78, 7, lapText, white, 1);
    m_canvas.Text(width - 78, 15, FormatTime(race.LapTime()), amber, 1);
    m_canvas.Text(width - 78, 24, "BEST " + FormatTime(race.BestLap()), green, 1);

    //--------------------------------------------------------------------------
    // 下一个检查点的方向箭头。
    //
    // 光靠画面里的门柱不够：门柱可能在弯道后面、被护墙挡着。玩家任何时候都
    // 该知道"该往哪开"，这在环形赛道上尤其重要 —— 掉头之后很容易反着跑。
    //--------------------------------------------------------------------------
    if (race.State() != RaceState::Finished) {
        const Vec3 toGate = race.NextCheckpoint().center - car.Position();
        const Vec3 forward = car.Forward();
        // 车头坐标系里的方位角：正数表示目标在右边
        const real bearing = Atan2(Dot(toGate, car.Right()), Dot(toGate, forward));
        const int cx = width / 2;
        const int cy = 12;
        const int px = cx + static_cast<int>(Clamp(bearing / kHalfPi, real(-1), real(1)) *
                                             real(40));
        const std::uint32_t arrow = Abs(bearing) < real(0.4) ? green : amber;
        m_canvas.Rect(cx - 44, cy - 2, 88, 1, Canvas::Pack(70, 74, 82));
        m_canvas.Rect(px - 2, cy - 4, 5, 5, arrow);
    }

    //-- 打滑提示：轮胎已经在滑了 -----------------------------------------------
    if (car.MaxSlip() > real(0.35) && car.GroundedWheels() > 0) {
        m_canvas.Text(width / 2 - 8, height - 34, "SLIP", red, 1);
    }
    if (car.GroundedWheels() == 0) {
        m_canvas.Text(width / 2 - 6, height - 34, "AIR", amber, 1);
    }

    //-- 倒计时 / 结算 ----------------------------------------------------------
    if (race.State() == RaceState::Countdown) {
        const int count = static_cast<int>(race.Countdown()) + 1;
        m_canvas.CenteredText(height / 2 - 12, std::to_string(count), amber, 4);
    }

    //-- 日志（最近几条）--------------------------------------------------------
    int row = 6;
    for (const std::string& line : race.Log()) {
        m_canvas.Text(5, row, line, Canvas::Pack(190, 195, 205), 1);
        row += 8;
    }
}

void RaceRenderer::DrawBanner(const std::string& title, const std::string& hint,
                              std::uint32_t color) {
    const int width = m_canvas.Width();
    const int height = m_canvas.Height();

    constexpr int kTextScale = 3;
    const int y = height / 2 - 18;
    const int band = 5 * kTextScale + 12 + (hint.empty() ? 0 : 14);

    m_canvas.Rect(0, y - 6, width, band, Canvas::Pack(0, 0, 0));
    m_canvas.CenteredText(y, title, color, kTextScale);
    if (!hint.empty()) {
        m_canvas.CenteredText(y + 5 * kTextScale + 5, hint, Canvas::Pack(180, 180, 185),
                              1);
    }
}

}  // namespace racing
