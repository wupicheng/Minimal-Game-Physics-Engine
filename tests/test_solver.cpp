//==============================================================================
// tests/test_solver.cpp
//
// M7 顺序冲量求解器的测试。
//
//------------------------------------------------------------------------------
// 这一层只能"端到端"地测
//------------------------------------------------------------------------------
// 求解器是迭代的、耦合的：单看某一次迭代施加了多大冲量，根本判断不出对错。
// 唯一有意义的检验方式是把它放进一个完整的模拟循环里跑，看**宏观行为**
// 是否符合物理：
//
//   - 弹跳高度是不是 e^2 倍（这是恢复系数的定义）
//   - 静止的箱子堆会不会往下沉
//   - 两个自由物体对撞，总动量守不守恒
//   - 摩擦系数大小是否真的决定斜坡上滑不滑
//
// 所以文件开头有一个 MiniWorld —— 一个最小的模拟循环
// （宽相位用暴力、窄相位用 M4、积分用 M6、求解用 M7）。
// 它同时也是 M8 里 PhysicsWorld::Step() 的一份可执行草稿：
// 步骤顺序、谁先谁后，在这里已经定型了。
//==============================================================================

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <vector>

#include "pe/collision/NarrowPhase.h"
#include "pe/dynamics/Integrator.h"
#include "pe/solver/SequentialImpulseSolver.h"

using namespace pe;
using Catch::Approx;

namespace {

constexpr real kTol = real(1e-4);

bool Eq(const Vec3& a, const Vec3& b, real tol = kTol) { return NearlyEqual(a, b, tol); }

//------------------------------------------------------------------------------
// MiniWorld —— 最小模拟循环
//
// 步骤顺序刻意与 ARCHITECTURE.md §4.8 里 PhysicsWorld::Step() 的规划一致：
//     1. 碰撞检测（用**上一帧末**的位置）
//     2. 积分速度（重力、外力）
//     3. 求解器（只改速度）
//     4. 积分位置（用求解后的速度）
//     5. 清力、更新休眠
//
// 注意第 1 步在第 2 步之前 —— 这是离散引擎的标准做法，也正是隧穿的根源。
//------------------------------------------------------------------------------
struct MiniWorld {
    struct Entry {
        RigidBody body;
        Shape shape;
        Material material;
    };

    std::vector<Entry> entries;
    SequentialImpulseSolver solver;
    Vec3 gravity = Vec3(0, real(-10), 0);
    bool enableSleep = false;

    std::size_t Add(const Shape& shape, const Transform& transform, real mass,
                    const Material& material, BodyType type = BodyType::Dynamic) {
        BodyDesc desc;
        desc.position = transform.position;
        desc.rotation = transform.rotation;
        desc.type = type;
        desc.mass = mass;
        // 阻尼默认关掉：测试要验证的是求解器，不希望阻尼把误差偷偷吃掉
        desc.linearDamping = real(0);
        desc.angularDamping = real(0);

        const MassProperties props = ComputeMassPropertiesFromMass(shape, mass);
        entries.push_back(Entry{MakeRigidBody(desc, props), shape, material});
        return entries.size() - 1;
    }

    RigidBody& Body(std::size_t i) { return entries[i].body; }

    void Step(real dt) {
        solver.BeginFrame();

        // 1. 碰撞检测（暴力配对，规模小无所谓）
        std::vector<Manifold> manifolds;
        manifolds.reserve(entries.size() * 2);
        struct PairInfo {
            std::size_t a, b;
        };
        std::vector<PairInfo> pairs;

        for (std::size_t i = 0; i < entries.size(); ++i) {
            for (std::size_t j = i + 1; j < entries.size(); ++j) {
                RigidBody& ba = entries[i].body;
                RigidBody& bb = entries[j].body;
                if (ba.invMass <= real(0) && bb.invMass <= real(0)) continue;
                if (!ba.IsActive() && !bb.IsActive()) continue;

                Manifold m;
                if (!Collide(entries[i].shape, ba.GetTransform(), entries[j].shape,
                             bb.GetTransform(), m)) {
                    continue;
                }
                manifolds.push_back(m);
                pairs.push_back(PairInfo{i, j});
            }
        }

        // manifolds 已经建完，指针稳定了才能交给求解器
        for (std::size_t k = 0; k < pairs.size(); ++k) {
            ContactInput input;
            input.bodyA = &entries[pairs[k].a].body;
            input.bodyB = &entries[pairs[k].b].body;
            input.manifold = &manifolds[k];
            input.materialA = entries[pairs[k].a].material;
            input.materialB = entries[pairs[k].b].material;
            // 稳定的对键：两个下标拼成 64 位
            input.pairKey = (static_cast<std::uint64_t>(pairs[k].a) << 32) |
                            static_cast<std::uint64_t>(pairs[k].b);
            solver.AddContact(input);

            // 有新接触就唤醒双方
            if (enableSleep) {
                entries[pairs[k].a].body.WakeUp();
                entries[pairs[k].b].body.WakeUp();
            }
        }

        // 2. 积分速度
        for (Entry& e : entries) IntegrateVelocity(e.body, gravity, dt);

        // 3. 求解（只改速度）
        solver.Solve(dt);
        solver.EndFrame();

        // 4. 积分位置
        for (Entry& e : entries) IntegratePosition(e.body, dt);

        // 5. 收尾
        SleepConfig sleepConfig;
        for (Entry& e : entries) {
            ClearForces(e.body);
            if (enableSleep) UpdateSleep(e.body, dt, sleepConfig);
        }
    }

    void Run(real duration, real dt = real(1) / real(60)) {
        const int steps = static_cast<int>(duration / dt + real(0.5));
        for (int i = 0; i < steps; ++i) Step(dt);
    }
};

/// 造一个静态地面（顶面在 y = 0）
std::size_t AddGround(MiniWorld& world, const Material& material) {
    return world.Add(Shape::MakeBox(Vec3(50, 5, 50)),
                     Transform(Vec3(0, real(-5), 0), Quat::Identity()), real(0),
                     material, BodyType::Static);
}

}  // namespace

//==============================================================================
// A. 单个物体落地
//==============================================================================

TEST_CASE("求解器 物体落到地面上会停住", "[solver]") {
    MiniWorld world;
    AddGround(world, Material(real(0.5), real(0)));

    // 盒子半高 0.5，从 5 米高落下 -> 静止时中心应该在 y = 0.5 附近
    const std::size_t box =
        world.Add(Shape::MakeCube(real(0.5)),
                  Transform(Vec3(0, 5, 0), Quat::Identity()), real(10),
                  Material(real(0.5), real(0)));

    world.Run(real(3));

    const RigidBody& b = world.Body(box);
    INFO("最终位置 y = " << b.position.y << "，速度 " << b.linearVelocity.y);

    // 停在地面上：允许 slop 量级的残留穿透
    REQUIRE(b.position.y == Approx(real(0.5)).margin(real(0.02)));
    // 速度基本归零
    REQUIRE(Abs(b.linearVelocity.y) < real(0.05));
    // 没穿过去
    REQUIRE(b.position.y > real(0.4));
}

TEST_CASE("求解器 静止穿透量稳定在 slop 附近", "[solver]") {
    // 这条盯的是 Baumgarte + slop 的配合。穿透量应该收敛到 slop 附近的一个常数：
    //   - 持续增长 -> 求解器没跟上（迭代不够 / warm starting 失效）
    //   - 收敛到 0 -> slop 没生效，物体会在完美接触处高频抖动
    MiniWorld world;
    AddGround(world, Material::Wood());
    world.Add(Shape::MakeCube(real(0.5)),
              Transform(Vec3(0, real(0.6), 0), Quat::Identity()), real(50),
              Material::Wood());

    world.Run(real(2));
    const real p2 = world.solver.MaxPenetration();
    world.Run(real(3));
    const real p5 = world.solver.MaxPenetration();

    INFO("2 秒时穿透 " << p2 << "，5 秒时 " << p5);
    REQUIRE(p5 < kLinearSlop * real(3));
    // 不再增长
    REQUIRE(p5 < p2 + real(0.001));
}

//==============================================================================
// B. 恢复系数 —— M7 的验收条件之一
//==============================================================================

TEST_CASE("求解器 弹跳高度符合恢复系数", "[solver]") {
    // 恢复系数的定义：分离速度 / 接近速度 = e。
    // 由能量关系，弹起的高度是 e^2 倍原高度。
    //
    // 这条用例直接量弹跳高度，是恢复系数实现是否正确的最直白的检验。
    const real dropHeight = real(4);
    const real radius = real(0.5);

    struct Case {
        real restitution;
        real tolerance;  // 相对容差
    };
    const Case cases[] = {
        {real(0.8), real(0.15)},
        {real(0.5), real(0.20)},
    };

    for (const Case& c : cases) {
        MiniWorld world;
        // 地面的恢复系数取 0，组合规则是取最大值，所以生效的是球的 e
        AddGround(world, Material(real(0.5), real(0)));
        const std::size_t ball =
            world.Add(Shape::MakeSphere(radius),
                      Transform(Vec3(0, radius + dropHeight, 0), Quat::Identity()),
                      real(1), Material(real(0.5), c.restitution));

        // 跑到落地并弹起，记录最高点
        real peak = real(0);
        bool hasBounced = false;
        const real dt = real(1) / real(240);  // 细步长，减少落地时刻的离散误差
        for (int i = 0; i < 240 * 4; ++i) {
            world.Step(dt);
            const RigidBody& b = world.Body(ball);
            // 弹起之后（速度转正）开始记录高度
            if (b.linearVelocity.y > real(0.1)) hasBounced = true;
            if (hasBounced) peak = Max(peak, b.position.y - radius);
            // 弹到顶点开始下落就收工
            if (hasBounced && b.linearVelocity.y < real(0)) break;
        }

        const real expected = dropHeight * c.restitution * c.restitution;
        INFO("e = " << c.restitution << "：期望弹到 " << expected << "，实测 " << peak);
        REQUIRE(hasBounced);
        REQUIRE(peak == Approx(expected).epsilon(c.tolerance));
    }
}

TEST_CASE("求解器 恢复系数为 0 时不弹", "[solver]") {
    MiniWorld world;
    AddGround(world, Material(real(0.5), real(0)));
    const std::size_t ball =
        world.Add(Shape::MakeSphere(real(0.5)),
                  Transform(Vec3(0, 5, 0), Quat::Identity()), real(1),
                  Material(real(0.5), real(0)));

    real peakAfterLanding = real(-100);
    bool landed = false;
    for (int i = 0; i < 240 * 3; ++i) {
        world.Step(real(1) / real(240));
        const RigidBody& b = world.Body(ball);
        if (!landed && b.position.y < real(0.55)) landed = true;
        if (landed) peakAfterLanding = Max(peakAfterLanding, b.position.y);
    }

    INFO("落地后的最高点 " << peakAfterLanding);
    // 落地之后不该再弹起来超过 2 厘米
    REQUIRE(peakAfterLanding < real(0.55));
}

TEST_CASE("求解器 恢复阈值让静止物体不抖", "[solver]") {
    // 没有 restitutionThreshold 的话，静止在地面上的物体会因为每帧重力带来的
    // 那一丁点接近速度而永远微微弹跳。这条用例盯着"最终速度必须归零"。
    MiniWorld world;
    AddGround(world, Material(real(0.5), real(0.8)));  // 地面很弹
    const std::size_t ball =
        world.Add(Shape::MakeSphere(real(0.5)),
                  Transform(Vec3(0, real(0.5), 0), Quat::Identity()), real(1),
                  Material(real(0.5), real(0.8)));

    world.Run(real(5));

    const RigidBody& b = world.Body(ball);
    INFO("5 秒后速度 " << b.linearVelocity.y);
    REQUIRE(Abs(b.linearVelocity.y) < real(0.1));
}

//==============================================================================
// C. 堆叠 —— M7 的另一条验收条件
//==============================================================================

TEST_CASE("求解器 10 层盒堆 5 秒不下沉", "[solver]") {
    // ARCHITECTURE.md 第 5 节给 M7 定的验收条件。
    //
    // 这是整个求解器最严苛的检验：最下面那个箱子要撑住上面 9 个的重量，
    // 而 8 轮 Gauss-Seidel 迭代远不足以精确解出来。**能撑住全靠 warm starting**
    // —— 下一条用例会把它关掉做对照。
    constexpr int kCount = 10;
    const real half = real(0.5);
    const real gap = real(0.01);  // 留一点缝，避免初始就互相穿透

    MiniWorld world;
    AddGround(world, Material::Wood());

    std::vector<std::size_t> boxes;
    for (int i = 0; i < kCount; ++i) {
        const real y = half + real(i) * (real(2) * half + gap);
        boxes.push_back(world.Add(Shape::MakeCube(half),
                                  Transform(Vec3(0, y, 0), Quat::Identity()), real(10),
                                  Material::Wood()));
    }

    const real initialBottom = world.Body(boxes[0]).position.y;
    world.Run(real(5));

    const RigidBody& bottom = world.Body(boxes[0]);
    const RigidBody& top = world.Body(boxes[kCount - 1]);

    INFO("最底层 y: " << initialBottom << " -> " << bottom.position.y);
    INFO("最顶层 y = " << top.position.y);
    INFO("最大穿透 " << world.solver.MaxPenetration());

    // 最下面那个不能被压进地面
    REQUIRE(bottom.position.y > half - real(0.02));

    // 整摞不能塌：顶层应该还在接近初始高度的位置
    // （每层允许沉 gap + 一点 slop）
    const real expectedTop = half + real(kCount - 1) * (real(2) * half + gap);
    REQUIRE(top.position.y > expectedTop - real(kCount) * real(0.02));

    // 没有横向溃散。
    //
    // 阈值给到 0.4 而不是更紧，是因为顺序冲量在这个规模下有肉眼可见的**摇晃**：
    // 实测顶层在 ±0.24 米之间来回晃，但**不是单向漂移**（分别在 1/3/5/7/9 秒采样，
    // 符号来回变）。摇晃幅度直接由迭代次数决定：实测 8 轮 ±0.24、16 轮 ±0.10。
    // 这是 Gauss-Seidel 收敛不足的固有表现，不是 bug，调 velocityIterations 即可。
    REQUIRE(Abs(top.position.x) < real(0.4));
    REQUIRE(Abs(top.position.z) < real(0.4));

    // 最终基本静止
    REQUIRE(bottom.linearVelocity.Length() < real(0.1));
    REQUIRE(top.linearVelocity.Length() < real(0.2));

    // 穿透量稳定在 slop 量级
    REQUIRE(world.solver.MaxPenetration() < kLinearSlop * real(4));
}

TEST_CASE("求解器 warm starting 确实是堆叠稳定的关键", "[solver]") {
    // 对照实验：同一个 10 层盒堆，唯一的区别是开不开 warm starting。
    //
    // 量的是**顶层的最终高度**，也就是"塌没塌"。
    // 注意别拿"最后一帧的穿透量"当指标 —— 塔塌了之后箱子摊平在地上，
    // 穿透反而变小了，看上去像是"没什么区别"。塌掉的证据是高度，不是穿透。
    constexpr int kCount = 10;
    const real expectedTop = real(0.5) + real(kCount - 1) * real(1.01);

    const auto runStack = [&](bool warmStarting) {
        MiniWorld world;
        SolverConfig config;
        config.warmStarting = warmStarting;
        world.solver.SetConfig(config);

        AddGround(world, Material::Wood());
        std::vector<std::size_t> boxes;
        for (int i = 0; i < kCount; ++i) {
            boxes.push_back(world.Add(
                Shape::MakeCube(real(0.5)),
                Transform(Vec3(0, real(0.5) + real(i) * real(1.01), 0), Quat::Identity()),
                real(10), Material::Wood()));
        }
        world.Run(real(5));
        return world.Body(boxes[kCount - 1]).position.y;
    };

    const real withWarm = runStack(true);
    const real withoutWarm = runStack(false);

    INFO("期望顶层高度 " << expectedTop << "；开 warm starting 实测 " << withWarm
                        << "，关掉之后 " << withoutWarm);

    // 开着：基本维持原高度（只沉掉初始留的那点缝）
    REQUIRE(withWarm > expectedTop - real(0.3));

    // 关掉：8 轮迭代撑不住 10 层的重量，整座塔在 5 秒内彻底塌掉。
    // 这就是"warm starting 不是优化，是承重结构"的量化证据。
    REQUIRE(withoutWarm < real(3));
}

TEST_CASE("求解器 金字塔堆叠", "[solver]") {
    // 比竖直堆叠更难：每个箱子被下面两个支撑，接触点更多、耦合更复杂。
    MiniWorld world;
    AddGround(world, Material::Wood());

    const real half = real(0.5);
    std::vector<std::size_t> topRow;
    for (int level = 0; level < 4; ++level) {
        const int count = 4 - level;
        for (int i = 0; i < count; ++i) {
            const real x = (real(i) - real(count - 1) * real(0.5)) * real(1.05);
            const real y = half + real(level) * real(1.01);
            const std::size_t id =
                world.Add(Shape::MakeCube(half),
                          Transform(Vec3(x, y, 0), Quat::Identity()), real(10),
                          Material::Wood());
            if (level == 3) topRow.push_back(id);
        }
    }

    world.Run(real(4));

    REQUIRE(topRow.size() == 1);
    const RigidBody& apex = world.Body(topRow[0]);
    INFO("顶点位置 " << apex.position.x << ", " << apex.position.y);

    // 塔尖没有塌下来
    REQUIRE(apex.position.y > half + real(3) * real(1.01) - real(0.15));
    REQUIRE(Abs(apex.position.x) < real(0.3));
}

//==============================================================================
// D. 摩擦
//==============================================================================

TEST_CASE("求解器 摩擦系数决定斜坡上滑不滑", "[solver]") {
    // 物理判据：物体在倾角 theta 的斜坡上静止，当且仅当 mu >= tan(theta)。
    // 取 30 度，tan(30) ≈ 0.577。
    const real angle = DegToRad(real(30));
    const real tanAngle = Tan(angle);

    const auto slideDistance = [&](real friction) {
        MiniWorld world;
        // 斜坡：一块绕 Z 转了 -30 度的大盒子
        const Quat slopeRot = Quat::FromAxisAngle(Vec3(0, 0, 1), -angle);
        world.Add(Shape::MakeBox(Vec3(20, real(0.5), 20)),
                  Transform(Vec3(0, 0, 0), slopeRot), real(0),
                  Material(friction, real(0)), BodyType::Static);

        // 箱子贴着斜面放，姿态与斜面一致
        const Vec3 up = slopeRot.Rotate(Vec3(0, 1, 0));
        const std::size_t box =
            world.Add(Shape::MakeCube(real(0.4)),
                      Transform(up * real(0.91), slopeRot), real(10),
                      Material(friction, real(0)));

        const Vec3 start = world.Body(box).position;
        world.Run(real(3));
        return (world.Body(box).position - start).Length();
    };

    const real highFriction = slideDistance(real(0.9));   // mu > tan(30)
    const real lowFriction = slideDistance(real(0.1));    // mu < tan(30)

    INFO("tan(30) = " << tanAngle);
    INFO("mu=0.9 滑了 " << highFriction << " 米；mu=0.1 滑了 " << lowFriction << " 米");

    // 摩擦足够时基本不滑
    REQUIRE(highFriction < real(0.15));
    // 摩擦不够时明显滑下去
    REQUIRE(lowFriction > real(1));
}

TEST_CASE("求解器 摩擦让滑动物体减速停下", "[solver]") {
    MiniWorld world;
    AddGround(world, Material(real(0.5), real(0)));
    const std::size_t box =
        world.Add(Shape::MakeCube(real(0.5)),
                  Transform(Vec3(0, real(0.5), 0), Quat::Identity()), real(10),
                  Material(real(0.5), real(0)));
    world.Body(box).linearVelocity = Vec3(8, 0, 0);

    world.Run(real(4));

    const RigidBody& b = world.Body(box);
    INFO("4 秒后速度 " << b.linearVelocity.x << "，位移 " << b.position.x);
    REQUIRE(Abs(b.linearVelocity.x) < real(0.3));
    REQUIRE(b.position.x > real(1));  // 确实滑出去了一段
}

TEST_CASE("求解器 零摩擦的物体一直滑", "[solver]") {
    MiniWorld world;
    AddGround(world, Material(real(0), real(0)));
    const std::size_t box =
        world.Add(Shape::MakeCube(real(0.5)),
                  Transform(Vec3(0, real(0.5), 0), Quat::Identity()), real(10),
                  Material(real(0), real(0)));
    world.Body(box).linearVelocity = Vec3(5, 0, 0);

    world.Run(real(2));

    // 摩擦组合取几何平均，任何一方为 0 结果就是 0 -> 速度几乎不变
    INFO("2 秒后速度 " << world.Body(box).linearVelocity.x);
    REQUIRE(world.Body(box).linearVelocity.x == Approx(real(5)).epsilon(real(0.05)));
}

//==============================================================================
// E. 守恒律与约束语义
//==============================================================================

TEST_CASE("求解器 自由对撞时动量守恒", "[solver]") {
    // 没有外力（关掉重力）、没有静态体时，系统总动量必须严格守恒 ——
    // 求解器施加的是**成对的等大反向冲量**，天然不改变总动量。
    // 这条能抓住"某一侧忘了施加冲量"或者"符号写反"这类错误。
    MiniWorld world;
    world.gravity = Vec3::Zero();

    const std::size_t a =
        world.Add(Shape::MakeSphere(real(0.5)),
                  Transform(Vec3(real(-3), 0, 0), Quat::Identity()), real(2),
                  Material(real(0.3), real(0.5)));
    const std::size_t b =
        world.Add(Shape::MakeSphere(real(0.5)),
                  Transform(Vec3(real(3), 0, 0), Quat::Identity()), real(5),
                  Material(real(0.3), real(0.5)));

    world.Body(a).linearVelocity = Vec3(4, 0, 0);
    world.Body(b).linearVelocity = Vec3(real(-1), 0, 0);

    const auto momentum = [&]() {
        return world.Body(a).linearVelocity * world.Body(a).Mass() +
               world.Body(b).linearVelocity * world.Body(b).Mass();
    };

    const Vec3 p0 = momentum();
    world.Run(real(3));
    const Vec3 p1 = momentum();

    INFO("初始动量 " << p0.x << "，之后 " << p1.x);
    REQUIRE(p0.x == Approx(real(3)).margin(kTol));  // 2*4 + 5*(-1) = 3
    REQUIRE(Eq(p1, p0, real(1e-3)));

    // 确认它们真的撞上了（速度变了）
    REQUIRE(world.Body(a).linearVelocity.x < real(3.9));
}

TEST_CASE("求解器 法向冲量永远非负", "[solver]") {
    // 接触只能推开、不能拉住。累积冲量钳到 >= 0 就是在保证这一点。
    // 出现负值意味着物体被"吸"在一起，玩家会看到诡异的粘连。
    MiniWorld world;
    AddGround(world, Material::Wood());
    for (int i = 0; i < 5; ++i) {
        world.Add(Shape::MakeCube(real(0.5)),
                  Transform(Vec3(real(i) * real(0.3), real(0.5) + real(i) * real(1.02), 0),
                            Quat::Identity()),
                  real(10), Material::Wood());
    }

    for (int step = 0; step < 300; ++step) {
        world.Step(real(1) / real(60));
        for (const ContactConstraint& c : world.solver.Contacts()) {
            for (std::uint8_t i = 0; i < c.pointCount; ++i) {
                REQUIRE(c.points[i].normalImpulse >= real(0));
                // 摩擦冲量必须在库仑锥的方盒内
                const real maxFriction = c.friction * c.points[i].normalImpulse;
                REQUIRE(Abs(c.points[i].tangentImpulse[0]) <= maxFriction + kTol);
                REQUIRE(Abs(c.points[i].tangentImpulse[1]) <= maxFriction + kTol);
            }
        }
    }
}

TEST_CASE("求解器 静态体不会被推动", "[solver]") {
    MiniWorld world;
    const std::size_t ground = AddGround(world, Material::Wood());
    // 一个很重的箱子砸下来
    world.Add(Shape::MakeCube(real(1)), Transform(Vec3(0, 8, 0), Quat::Identity()),
              real(5000), Material::Wood());

    const Vec3 groundStart = world.Body(ground).position;
    world.Run(real(3));

    REQUIRE(Eq(world.Body(ground).position, groundStart));
    REQUIRE(Eq(world.Body(ground).linearVelocity, Vec3::Zero()));
    REQUIRE(Eq(world.Body(ground).angularVelocity, Vec3::Zero()));
}

TEST_CASE("求解器 质量差异体现在推动效果上", "[solver]") {
    // 轻的撞重的，轻的应该被弹回来，重的几乎不动。
    MiniWorld world;
    world.gravity = Vec3::Zero();

    const std::size_t light =
        world.Add(Shape::MakeSphere(real(0.5)),
                  Transform(Vec3(real(-2), 0, 0), Quat::Identity()), real(1),
                  Material(real(0), real(0.9)));
    const std::size_t heavy =
        world.Add(Shape::MakeSphere(real(0.5)),
                  Transform(Vec3(real(2), 0, 0), Quat::Identity()), real(100),
                  Material(real(0), real(0.9)));

    world.Body(light).linearVelocity = Vec3(5, 0, 0);
    world.Run(real(2));

    INFO("轻的 " << world.Body(light).linearVelocity.x << "，重的 "
                 << world.Body(heavy).linearVelocity.x);
    // 轻的被弹回（速度转负）
    REQUIRE(world.Body(light).linearVelocity.x < real(0));
    // 重的只被推动一点点
    REQUIRE(world.Body(heavy).linearVelocity.x > real(0));
    REQUIRE(world.Body(heavy).linearVelocity.x < real(0.5));
}

//==============================================================================
// F. 接触缓存
//==============================================================================

TEST_CASE("求解器 接触缓存会清理消失的接触", "[solver]") {
    // 少了清理，一局游戏下来哈希表会长满几万条早已不存在的接触。
    MiniWorld world;
    AddGround(world, Material::Wood());
    const std::size_t box =
        world.Add(Shape::MakeCube(real(0.5)),
                  Transform(Vec3(0, real(0.5), 0), Quat::Identity()), real(10),
                  Material::Wood());

    world.Run(real(1));
    REQUIRE(world.solver.Cache().Size() >= 1);

    // 把箱子扔到很远的地方，接触消失
    world.Body(box).position = Vec3(1000, 1000, 1000);
    world.Body(box).linearVelocity = Vec3::Zero();
    world.Step(real(1) / real(60));
    world.Step(real(1) / real(60));

    REQUIRE(world.solver.Cache().Size() == 0);
}

TEST_CASE("求解器 深度穿透不会把物体弹飞", "[solver]") {
    // 物体生成时嵌在墙里一米深，是很常见的情况（关卡摆放失误、传送到墙里）。
    // maxLinearCorrection 就是为这个准备的：没有它，Baumgarte 会给出
    // 0.2 * 1 / (1/60) = 12 m/s 的分离速度，把物体一脚踹飞。
    MiniWorld world;
    AddGround(world, Material::Wood());
    // 中心在 y = -0.5，也就是整个箱子都埋在地面里
    const std::size_t box =
        world.Add(Shape::MakeCube(real(0.5)),
                  Transform(Vec3(0, real(-0.5), 0), Quat::Identity()), real(10),
                  Material::Wood());

    real maxSpeed = real(0);
    for (int i = 0; i < 180; ++i) {
        world.Step(real(1) / real(60));
        maxSpeed = Max(maxSpeed, world.Body(box).linearVelocity.Length());
    }

    const RigidBody& b = world.Body(box);
    INFO("推出过程中的最大速度 " << maxSpeed << "，最终 y = " << b.position.y);

    // 被平滑推出来，而不是弹飞
    REQUIRE(maxSpeed < real(6));
    // 最终停在地面上
    REQUIRE(b.position.y == Approx(real(0.5)).margin(real(0.05)));
}

TEST_CASE("求解器 休眠之后物体不再抖动", "[solver]") {
    MiniWorld world;
    world.enableSleep = true;
    AddGround(world, Material::Wood());
    const std::size_t box =
        world.Add(Shape::MakeCube(real(0.5)),
                  Transform(Vec3(0, real(0.5), 0), Quat::Identity()), real(10),
                  Material::Wood());

    world.Run(real(6));

    const RigidBody& b = world.Body(box);
    INFO("6 秒后 y = " << b.position.y << "，睡着了吗 " << b.isSleeping);
    REQUIRE(b.position.y == Approx(real(0.5)).margin(real(0.03)));
}

//==============================================================================
// G. 迭代次数的影响
//==============================================================================

TEST_CASE("求解器 迭代次数越多堆叠越稳", "[solver]") {
    // 迭代次数是画质与耗时之间的旋钮。用一座 20 层的塔把差别放大：
    // 层数越多，最底层要撑的重量越大，收敛不足的表现就越明显。
    //
    // 这条同时是回归哨兵 —— 哪天迭代次数不再起作用，说明求解循环被改坏了。
    constexpr int kCount = 20;
    const real expectedTop = real(0.5) + real(kCount - 1) * real(1.01);

    const auto stackSag = [&](int iterations) {
        MiniWorld world;
        SolverConfig config;
        config.velocityIterations = iterations;
        world.solver.SetConfig(config);

        AddGround(world, Material::Wood());
        std::vector<std::size_t> boxes;
        for (int i = 0; i < kCount; ++i) {
            boxes.push_back(world.Add(
                Shape::MakeCube(real(0.5)),
                Transform(Vec3(0, real(0.5) + real(i) * real(1.01), 0), Quat::Identity()),
                real(10), Material::Wood()));
        }
        world.Run(real(5));
        return expectedTop - world.Body(boxes[kCount - 1]).position.y;
    };

    const real sag8 = stackSag(8);
    const real sag24 = stackSag(24);

    INFO("8 轮迭代下沉 " << sag8 << " 米，24 轮下沉 " << sag24 << " 米");
    REQUIRE(sag24 < sag8);
    // 两者都不能塌
    REQUIRE(sag8 < real(2));
}
