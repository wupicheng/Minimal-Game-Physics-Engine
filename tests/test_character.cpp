//==============================================================================
// tests/test_character.cpp
//
// M8 形状扫掠 + 角色控制器 + 触发器的测试。
//
//------------------------------------------------------------------------------
// 三层
//------------------------------------------------------------------------------
//   A. **ShapeCast**：保守推进。它是角色控制器的地基，也是 M2 那条 P0
//      （抛射物穿墙）的解法。这一层能对着解析解逐项核对 —— 一个球沿直线撞平面，
//      接触位置是可以手算出来的。
//
//   B. **CharacterController**：只能看**行为**。斜坡上滑不滑、台阶上不上得去、
//      贴着墙走会不会卡住 —— 这些正是 ARCHITECTURE.md 给 M8 定的验收条件。
//      测试里给它一个只有几个静态盒子的假世界（FakeWorld），
//      不需要把刚体、求解器、宽相位全拉起来。
//
//   C. **TriggerSystem**：时序。Enter 只能报一次、Stay 每帧报、Exit 在离开那帧报，
//      以及销毁物体时必须补发 Exit。
//==============================================================================

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <vector>

#include "pe/character/CharacterController.h"
#include "pe/collision/NarrowPhase.h"
#include "pe/collision/ShapeCast.h"
#include "pe/scene/TriggerSystem.h"

using namespace pe;
using Catch::Approx;

namespace {

constexpr real kTol = real(1e-3);

bool Eq(const Vec3& a, const Vec3& b, real tol = kTol) { return NearlyEqual(a, b, tol); }

Transform At(const Vec3& p) { return Transform(p, Quat::Identity()); }

//------------------------------------------------------------------------------
// 假世界：一堆静态形状，扫掠时取最近的那个命中
//
// 角色控制器按架构规定不依赖 scene 层，所以它只认 ICharacterWorld 这个接口。
// 好处正体现在这里 —— 测试可以给它一个二十行的假实现。
//------------------------------------------------------------------------------
class FakeWorld : public ICharacterWorld {
public:
    struct Item {
        Shape shape;
        Transform transform;
    };
    std::vector<Item> items;

    void AddBox(const Vec3& center, const Vec3& halfExtents,
                const Quat& rotation = Quat::Identity()) {
        items.push_back(Item{Shape::MakeBox(halfExtents), Transform(center, rotation)});
    }

    bool SweepCharacter(const Shape& shape, const Transform& start,
                        const Vec3& displacement, LayerMask, ShapeCastHit& out) const override {
        bool found = false;
        ShapeCastHit best;
        best.fraction = real(2);

        for (const Item& item : items) {
            ShapeCastHit hit;
            if (!ShapeCast(shape, start, displacement, item.shape, item.transform, hit)) {
                continue;
            }
            // 起点重叠优先：必须先脱困再谈移动
            const bool better = hit.startPenetrating
                                    ? (!best.startPenetrating || hit.depth > best.depth)
                                    : (!best.startPenetrating && hit.fraction < best.fraction);
            if (!found || better) {
                best = hit;
                found = true;
            }
        }

        if (found) out = best;
        return found;
    }
};

}  // namespace

//==============================================================================
// A. 形状扫掠
//==============================================================================

TEST_CASE("扫掠 球撞墙的接触位置可以手算", "[shapecast][collision]") {
    // 墙：x ∈ [4, 6] 的一块板。球半径 0.5 从原点沿 +X 走 10 米。
    // 球心走到 x = 3.5 时表面刚好贴上 x = 4 -> fraction = 0.35
    const Shape ball = Shape::MakeSphere(real(0.5));
    const Shape wall = Shape::MakeBox(Vec3(1, 10, 10));

    ShapeCastHit hit;
    REQUIRE(ShapeCast(ball, At(Vec3(0, 0, 0)), Vec3(10, 0, 0), wall, At(Vec3(5, 0, 0)),
                      hit));

    REQUIRE_FALSE(hit.startPenetrating);
    REQUIRE(hit.fraction == Approx(real(0.35)).margin(real(0.002)));
    // 法线朝向球来的一侧
    REQUIRE(Eq(hit.normal, Vec3(-1, 0, 0), real(0.01)));
    REQUIRE(hit.point.x == Approx(real(4)).margin(real(0.02)));
}

TEST_CASE("扫掠 够不着就不报命中", "[shapecast][collision]") {
    const Shape ball = Shape::MakeSphere(real(0.5));
    const Shape wall = Shape::MakeBox(Vec3(1, 10, 10));
    ShapeCastHit hit;

    SECTION("位移太短") {
        REQUIRE_FALSE(
            ShapeCast(ball, At(Vec3(0, 0, 0)), Vec3(1, 0, 0), wall, At(Vec3(5, 0, 0)), hit));
    }

    SECTION("方向不对（背对着走）") {
        REQUIRE_FALSE(ShapeCast(ball, At(Vec3(0, 0, 0)), Vec3(-10, 0, 0), wall,
                                At(Vec3(5, 0, 0)), hit));
    }

    SECTION("平行擦过，错开了") {
        REQUIRE_FALSE(ShapeCast(ball, At(Vec3(0, 20, 0)), Vec3(10, 0, 0), wall,
                                At(Vec3(5, 0, 0)), hit));
    }

    SECTION("零位移且不重叠") {
        REQUIRE_FALSE(ShapeCast(ball, At(Vec3(0, 0, 0)), Vec3::Zero(), wall,
                                At(Vec3(5, 0, 0)), hit));
    }
}

TEST_CASE("扫掠 高速穿墙必须被抓住", "[shapecast][collision]") {
    // 这就是 M2 标着 P0 的那条：一帧移动 100 米的火箭 vs 20 厘米厚的墙。
    // 离散检测（只看两端）完全看不见这次碰撞，扫掠必须看得见。
    const Shape rocket = Shape::MakeSphere(real(0.05));
    const Shape thinWall = Shape::MakeBox(Vec3(real(0.1), 10, 10));

    ShapeCastHit hit;
    REQUIRE(ShapeCast(rocket, At(Vec3(0, 0, 0)), Vec3(100, 0, 0), thinWall,
                      At(Vec3(50, 0, 0)), hit));
    // 墙的近面在 x = 49.9，球心停在 49.85 -> fraction ≈ 0.4985
    REQUIRE(hit.fraction == Approx(real(0.4985)).margin(real(0.002)));

    // 对照：只测两端的离散做法什么都发现不了
    Manifold m;
    REQUIRE_FALSE(Collide(rocket, At(Vec3(0, 0, 0)), thinWall, At(Vec3(50, 0, 0)), m));
    REQUIRE_FALSE(Collide(rocket, At(Vec3(100, 0, 0)), thinWall, At(Vec3(50, 0, 0)), m));
}

TEST_CASE("扫掠 起点就重叠时给出脱困信息", "[shapecast][collision]") {
    const Shape ball = Shape::MakeSphere(real(0.5));
    const Shape wall = Shape::MakeBox(Vec3(1, 10, 10));

    // 球心在 x = 5.8，墙占 x ∈ [4,6]，球陷进去了
    ShapeCastHit hit;
    REQUIRE(ShapeCast(ball, At(Vec3(real(5.8), 0, 0)), Vec3(10, 0, 0), wall,
                      At(Vec3(5, 0, 0)), hit));

    REQUIRE(hit.startPenetrating);
    REQUIRE(hit.fraction == Approx(real(0)));
    // 最近的面是 x = 6，脱困方向是 +X，深度 = 6 - 5.8 + 0.5 = 0.7
    REQUIRE(Eq(hit.normal, Vec3(1, 0, 0), real(0.01)));
    REQUIRE(hit.depth == Approx(real(0.7)).margin(real(0.02)));

    // 沿法线推出去之后就不该再重叠了
    Manifold m;
    const Vec3 escaped = Vec3(real(5.8), 0, 0) + hit.normal * (hit.depth + real(0.01));
    REQUIRE_FALSE(Collide(ball, At(escaped), wall, At(Vec3(5, 0, 0)), m));
}

TEST_CASE("扫掠 胶囊斜着撞盒子", "[shapecast][collision]") {
    const Shape capsule = Shape::MakeCapsule(real(0.3), real(0.5));
    const Shape box = Shape::MakeCube(real(1));

    ShapeCastHit hit;
    REQUIRE(ShapeCast(capsule, At(Vec3(-5, 3, 0)), Vec3(10, -6, 0), box,
                      At(Vec3(0, 0, 0)), hit));
    REQUIRE_FALSE(hit.startPenetrating);
    REQUIRE(hit.fraction > real(0));
    REQUIRE(hit.fraction < real(1));
    REQUIRE(NearlyEqual(hit.normal.Length(), real(1), real(0.01)));

    // 停下来的位置不该已经穿进去了（扫掠必须是保守的）
    const Vec3 stopped = Vec3(-5, 3, 0) + Vec3(10, -6, 0) * hit.fraction;
    Manifold m;
    if (Collide(capsule, At(stopped), box, At(Vec3(0, 0, 0)), m)) {
        REQUIRE(m.MaxPenetration() < kLinearSlop);
    }
}

TEST_CASE("扫掠 停下的位置总是保守的", "[shapecast][collision]") {
    // 随机撒一批扫掠，要求：停在 fraction 处时穿透不超过容差，
    // 而且再往前走一点点就一定会真的碰上。
    struct Rng {
        std::uint32_t s;
        real U() {
            s = s * 1664525u + 1013904223u;
            return real(s >> 8) / real(1 << 24);
        }
        real R(real a, real b) { return a + (b - a) * U(); }
        Vec3 C(real h) { return Vec3(R(-h, h), R(-h, h), R(-h, h)); }
    } rng{424242u};

    const Shape target = Shape::MakeCube(real(1));
    int hits = 0;

    for (int i = 0; i < 500; ++i) {
        const Shape moving = (i % 2 == 0) ? Shape::MakeSphere(rng.R(real(0.2), real(0.6)))
                                          : Shape::MakeCapsule(real(0.25), real(0.4));
        const Vec3 start = rng.C(real(6));
        const Vec3 displacement = -start * rng.R(real(0.8), real(1.5));

        ShapeCastHit hit;
        if (!ShapeCast(moving, At(start), displacement, target, At(Vec3::Zero()), hit)) {
            continue;
        }
        if (hit.startPenetrating) continue;
        ++hits;

        INFO("iteration " << i << " fraction " << hit.fraction);

        // 停在接触点：穿透不超过容差
        Manifold m;
        if (Collide(moving, At(start + displacement * hit.fraction), target,
                    At(Vec3::Zero()), m)) {
            REQUIRE(m.MaxPenetration() < kLinearSlop);
        }
    }

    REQUIRE(hits > 100);
}

//==============================================================================
// B. 角色控制器
//==============================================================================

TEST_CASE("角色 走在平地上", "[character]") {
    FakeWorld world;
    world.AddBox(Vec3(0, real(-0.5), 0), Vec3(50, real(0.5), 50));  // 地面顶在 y=0

    CharacterController character;
    character.SetFootPosition(Vec3(0, 0, 0));
    character.UpdateGrounded(world);

    REQUIRE(character.isGrounded);
    REQUIRE(Eq(character.groundNormal, Vec3(0, 1, 0), real(0.05)));

    // 往前走一秒
    for (int i = 0; i < 60; ++i) {
        character.Update(world, Vec3(3, 0, 0), real(1) / real(60), Vec3(0, real(-10), 0));
    }

    INFO("走了 " << character.position.x << " 米，脚底 y = " << character.FootPosition().y);
    REQUIRE(character.position.x == Approx(real(3)).epsilon(real(0.05)));
    // 没陷进地里也没浮起来
    REQUIRE(character.FootPosition().y == Approx(real(0)).margin(real(0.05)));
    REQUIRE(character.isGrounded);
}

TEST_CASE("角色 贴着墙走不会卡住", "[character]") {
    // ARCHITECTURE.md 给 M8 定的验收条件之一。
    //
    // 角色斜着往墙里走：应该沿着墙面滑过去，而不是被顶住原地不动。
    // 那一行"把剩余位移投影到墙面切平面"就是这个手感的全部来源。
    FakeWorld world;
    world.AddBox(Vec3(0, real(-0.5), 0), Vec3(50, real(0.5), 50));  // 地面
    world.AddBox(Vec3(2, 5, 0), Vec3(real(0.5), 5, 50));            // x=1.5 处一面墙

    CharacterController character;
    character.SetFootPosition(Vec3(0, 0, -5));
    character.UpdateGrounded(world);

    // 朝右前方走：右边是墙，应该被挡住 x、但 z 方向照走不误
    for (int i = 0; i < 120; ++i) {
        character.Update(world, Vec3(3, 0, 3), real(1) / real(60), Vec3(0, real(-10), 0));
    }

    INFO("最终位置 " << character.position.x << ", " << character.position.z);

    // 没有穿墙
    REQUIRE(character.position.x < real(1.5) - character.config.radius + real(0.05));
    // 但是沿着墙滑过去了 —— 这才是关键。卡住的话 z 几乎不会动。
    REQUIRE(character.position.z > real(-5) + real(4));
}

TEST_CASE("角色 正面撞墙会停下但不穿墙", "[character]") {
    FakeWorld world;
    world.AddBox(Vec3(0, real(-0.5), 0), Vec3(50, real(0.5), 50));
    world.AddBox(Vec3(3, 5, 0), Vec3(real(0.5), 5, 50));  // 近面在 x = 2.5

    CharacterController character;
    character.SetFootPosition(Vec3(0, 0, 0));
    character.UpdateGrounded(world);

    for (int i = 0; i < 180; ++i) {
        character.Update(world, Vec3(5, 0, 0), real(1) / real(60), Vec3(0, real(-10), 0));
    }

    const real limit = real(2.5) - character.config.radius;
    INFO("停在 x = " << character.position.x << "，理论极限 " << limit);
    REQUIRE(character.position.x < limit + real(0.05));
    REQUIRE(character.position.x > limit - real(0.2));
}

TEST_CASE("角色 能自动跨上台阶", "[character]") {
    // 验收条件之一。台阶高度在 stepOffset 之内就该自动上去，不需要跳。
    FakeWorld world;
    world.AddBox(Vec3(0, real(-0.5), 0), Vec3(50, real(0.5), 50));  // 地面 y=0

    const real stepHeight = real(0.3);  // 小于默认 stepOffset 0.35
    world.AddBox(Vec3(5, stepHeight * real(0.5), 0),
                 Vec3(2, stepHeight * real(0.5), 10));  // 台阶顶面 y=0.3

    CharacterController character;
    character.SetFootPosition(Vec3(0, 0, 0));
    character.UpdateGrounded(world);

    // 量的是"走到台阶正上方时脚底在哪儿"，而不是"跑完之后停在哪儿"——
    // 角色上了台阶还会继续往前走，走过台阶另一头就又回到地面了，
    // 那是对的行为，拿终点位置当判据只会测出一个假结论。
    bool stoodOnStep = false;
    real maxFoot = real(-100);

    for (int i = 0; i < 180; ++i) {
        character.Update(world, Vec3(3, 0, 0), real(1) / real(60), Vec3(0, real(-10), 0));
        maxFoot = Max(maxFoot, character.FootPosition().y);

        // 台阶占 x ∈ [3, 7]，取中段判定，避开上下台阶的过渡
        if (character.position.x > real(4.5) && character.position.x < real(6.5)) {
            stoodOnStep = stoodOnStep || (Abs(character.FootPosition().y - stepHeight) <
                                          real(0.06) && character.isGrounded);
        }
    }

    INFO("最高脚底 y = " << maxFoot << "，终点 x = " << character.position.x);
    REQUIRE(stoodOnStep);                                            // 确实站上去过
    REQUIRE(maxFoot == Approx(stepHeight).margin(real(0.06)));       // 没有飞过头
    REQUIRE(character.position.x > real(7));                         // 一路走过去了
    REQUIRE(character.isGrounded);
}

TEST_CASE("角色 太高的台阶上不去", "[character]") {
    // 对照组：超过 stepOffset 就该被挡住。没有这一条的话，
    // "台阶逻辑"会退化成"角色能爬任意高的墙"。
    FakeWorld world;
    world.AddBox(Vec3(0, real(-0.5), 0), Vec3(50, real(0.5), 50));
    world.AddBox(Vec3(5, real(0.6), 0), Vec3(2, real(0.6), 10));  // 顶面 y=1.2，太高

    CharacterController character;
    character.SetFootPosition(Vec3(0, 0, 0));
    character.UpdateGrounded(world);

    for (int i = 0; i < 180; ++i) {
        character.Update(world, Vec3(3, 0, 0), real(1) / real(60), Vec3(0, real(-10), 0));
    }

    INFO("脚底 y = " << character.FootPosition().y << "，x = " << character.position.x);
    REQUIRE(character.FootPosition().y < real(0.3));       // 没上去
    REQUIRE(character.position.x < real(3) + real(0.15));  // 被挡在台阶前
}

TEST_CASE("角色 缓坡能走上去，陡坡会滑下来", "[character]") {
    // 验收条件之一。判据是 maxSlopeAngle（默认 50 度）。
    const auto climbResult = [](real slopeDegrees) {
        FakeWorld world;
        // 绕 +Z 转正角度：+Y 朝 -X 倾斜，于是坡面沿 +X 方向**升高**，
        // 角色朝 +X 走就是上坡。转负角度的话 +X 反而是下坡，
        // 那样测出来的"能走"毫无意义 —— 下坡当然谁都能走。
        const Quat rot = Quat::FromAxisAngle(Vec3(0, 0, 1), DegToRad(slopeDegrees));
        // 一块很大的斜板，中心在原点
        world.AddBox(Vec3(0, 0, 0), Vec3(30, real(0.5), 30), rot);

        CharacterController character;
        // 放在斜面上：沿斜面法线抬到表面之上
        const Vec3 up = rot.Rotate(Vec3(0, 1, 0));
        character.SetFootPosition(up * real(0.5) + Vec3(real(-5), 0, 0));
        // 沿坡面往下挪一点，确保脚确实贴在坡上（斜板中心在原点，
        // 从 x=-5 处的坡面高度 = -5 * tan(theta)）
        character.position += Vec3(real(0), real(-5) * Tan(DegToRad(slopeDegrees)), real(0));
        character.UpdateGrounded(world);

        const real startX = character.position.x;
        for (int i = 0; i < 240; ++i) {
            character.Update(world, Vec3(3, 0, 0), real(1) / real(60),
                             Vec3(0, real(-10), 0));
        }
        return character.position.x - startX;
    };

    const real gentle = climbResult(real(25));  // 缓坡，能上
    const real steep = climbResult(real(70));   // 陡坡，上不去

    INFO("25 度坡前进了 " << gentle << " 米；70 度坡前进了 " << steep << " 米");
    REQUIRE(gentle > real(2));
    // 陡坡上不去（甚至可能被推回去）
    REQUIRE(steep < gentle * real(0.5));
}

TEST_CASE("角色 站在陡坡上不算 grounded", "[character]") {
    FakeWorld world;
    const Quat rot = Quat::FromAxisAngle(Vec3(0, 0, 1), -DegToRad(real(70)));
    world.AddBox(Vec3(0, 0, 0), Vec3(30, real(0.5), 30), rot);

    CharacterController character;
    const Vec3 up = rot.Rotate(Vec3(0, 1, 0));
    character.SetFootPosition(up * real(0.51));
    character.UpdateGrounded(world);

    // 70 度 > maxSlopeAngle 50 度 -> 站不住
    REQUIRE_FALSE(character.isGrounded);
}

TEST_CASE("角色 会掉下来并落地", "[character]") {
    FakeWorld world;
    world.AddBox(Vec3(0, real(-0.5), 0), Vec3(50, real(0.5), 50));

    CharacterController character;
    character.SetFootPosition(Vec3(0, 10, 0));
    character.UpdateGrounded(world);
    REQUIRE_FALSE(character.isGrounded);

    for (int i = 0; i < 300; ++i) {
        character.Update(world, Vec3::Zero(), real(1) / real(60), Vec3(0, real(-10), 0));
    }

    INFO("脚底 y = " << character.FootPosition().y);
    REQUIRE(character.isGrounded);
    REQUIRE(character.FootPosition().y == Approx(real(0)).margin(real(0.05)));
    // 落地之后竖直速度被清掉，不会继续累积
    REQUIRE(Abs(Dot(character.velocity, Vec3(0, 1, 0))) < real(1));
}

TEST_CASE("角色 跳跃", "[character]") {
    FakeWorld world;
    world.AddBox(Vec3(0, real(-0.5), 0), Vec3(50, real(0.5), 50));
    world.AddBox(Vec3(0, real(6), 0), Vec3(50, real(0.5), 50));  // 天花板底面 y=5.5

    CharacterController character;
    character.SetFootPosition(Vec3(0, 0, 0));
    character.UpdateGrounded(world);

    SECTION("站在地上才能跳") {
        REQUIRE(character.Jump(real(5)));
        REQUIRE_FALSE(character.isGrounded);
        // 空中不能再跳
        REQUIRE_FALSE(character.Jump(real(5)));
    }

    SECTION("跳起来又落回地面") {
        REQUIRE(character.Jump(real(5)));
        real peak = real(0);
        for (int i = 0; i < 300; ++i) {
            character.Update(world, Vec3::Zero(), real(1) / real(60),
                             Vec3(0, real(-10), 0));
            peak = Max(peak, character.FootPosition().y);
        }
        // v^2/(2g) = 25/20 = 1.25 米
        INFO("跳到 " << peak << " 米");
        REQUIRE(peak == Approx(real(1.25)).epsilon(real(0.2)));
        REQUIRE(character.isGrounded);
    }

    SECTION("撞到天花板会停止上升") {
        character.velocity = Vec3(0, 20, 0);
        character.isGrounded = false;
        real peak = real(0);
        for (int i = 0; i < 60; ++i) {
            character.Update(world, Vec3::Zero(), real(1) / real(60),
                             Vec3(0, real(-10), 0));
            peak = Max(peak, character.position.y);
        }
        // 头顶（中心 + 半身高）不能穿过 y = 5.5
        INFO("最高时中心 y = " << peak);
        REQUIRE(peak + character.config.height * real(0.5) < real(5.55));
    }
}

TEST_CASE("角色 卡在墙里能自己脱困", "[character]") {
    // 出生点摆在了墙里、被传送进几何体 —— 这在真实项目里并不罕见。
    // 角色必须能自己走出来，而不是永远卡死。
    FakeWorld world;
    world.AddBox(Vec3(0, real(-0.5), 0), Vec3(50, real(0.5), 50));
    world.AddBox(Vec3(0, 2, 0), Vec3(2, 2, 2));  // 一个大方块

    CharacterController character;
    character.position = Vec3(0, 2, 0);  // 正中心，完全陷在里面

    for (int i = 0; i < 240; ++i) {
        character.Update(world, Vec3::Zero(), real(1) / real(60), Vec3(0, real(-10), 0));
    }

    // 脱出来了：不再和方块重叠
    Manifold m;
    const bool stillStuck =
        Collide(character.GetShape(), character.GetTransform(), Shape::MakeBox(Vec3(2, 2, 2)),
                At(Vec3(0, 2, 0)), m) &&
        m.MaxPenetration() > real(0.05);

    INFO("最终位置 " << character.position.x << ", " << character.position.y << ", "
                    << character.position.z);
    REQUIRE_FALSE(stillStuck);
}

TEST_CASE("角色 走进墙角不会被弹飞", "[character]") {
    // 两面墙夹成的角落是 collide-and-slide 最容易出事的地方：
    // 投影两次之后位移可能变成零，也可能因为反复投影而累积出异常的大位移。
    FakeWorld world;
    world.AddBox(Vec3(0, real(-0.5), 0), Vec3(50, real(0.5), 50));
    world.AddBox(Vec3(3, 5, 0), Vec3(real(0.5), 5, 50));  // x 方向的墙
    world.AddBox(Vec3(0, 5, 3), Vec3(50, 5, real(0.5)));  // z 方向的墙

    CharacterController character;
    character.SetFootPosition(Vec3(0, 0, 0));
    character.UpdateGrounded(world);

    for (int i = 0; i < 240; ++i) {
        character.Update(world, Vec3(5, 0, 5), real(1) / real(60), Vec3(0, real(-10), 0));
        // 每一帧都不能跑出角落
        REQUIRE(character.position.x < real(2.6));
        REQUIRE(character.position.z < real(2.6));
        REQUIRE(IsFinite(character.position.x));
        REQUIRE(IsFinite(character.position.z));
    }
}

//==============================================================================
// C. 触发器
//==============================================================================

namespace {

ColliderHandle MakeCollider(std::uint32_t index) {
    return ColliderHandle(index, 1u);
}

/// 数一数队列里某种事件出现了几次
int CountEvents(const EventQueue& queue, TriggerEventType type) {
    int n = 0;
    for (const TriggerEvent& e : queue.TriggerEvents()) {
        if (e.type == type) ++n;
    }
    return n;
}

}  // namespace

TEST_CASE("触发器 Enter/Stay/Exit 的时序", "[trigger][scene]") {
    // ARCHITECTURE.md 给 M8 定的验收条件之一。
    TriggerSystem triggers;
    const ColliderHandle zone = MakeCollider(1);
    const ColliderHandle player = MakeCollider(2);

    EventQueue queue;

    SECTION("第一帧进入：只报 Enter") {
        triggers.BeginFrame();
        triggers.AddOverlap(zone, player);
        triggers.EndFrame(queue);

        REQUIRE(queue.TriggerEvents().size() == 1);
        REQUIRE(queue.TriggerEvents()[0].type == TriggerEventType::Enter);
        REQUIRE(queue.TriggerEvents()[0].trigger == zone);
        REQUIRE(queue.TriggerEvents()[0].other == player);
    }

    SECTION("持续在里面：Enter 只报一次，之后每帧 Stay") {
        triggers.BeginFrame();
        triggers.AddOverlap(zone, player);
        triggers.EndFrame(queue);
        REQUIRE(CountEvents(queue, TriggerEventType::Enter) == 1);

        for (int frame = 0; frame < 5; ++frame) {
            queue.Clear();
            triggers.BeginFrame();
            triggers.AddOverlap(zone, player);
            triggers.EndFrame(queue);

            INFO("frame " << frame);
            REQUIRE(CountEvents(queue, TriggerEventType::Enter) == 0);
            REQUIRE(CountEvents(queue, TriggerEventType::Stay) == 1);
            REQUIRE(CountEvents(queue, TriggerEventType::Exit) == 0);
        }
    }

    SECTION("离开的那一帧报 Exit，之后什么都不报") {
        triggers.BeginFrame();
        triggers.AddOverlap(zone, player);
        triggers.EndFrame(queue);

        // 这一帧没有报告重叠 -> 离开
        queue.Clear();
        triggers.BeginFrame();
        triggers.EndFrame(queue);
        REQUIRE(queue.TriggerEvents().size() == 1);
        REQUIRE(queue.TriggerEvents()[0].type == TriggerEventType::Exit);
        REQUIRE(triggers.OverlapCount() == 0);

        // 再下一帧安静了
        queue.Clear();
        triggers.BeginFrame();
        triggers.EndFrame(queue);
        REQUIRE(queue.Empty());
    }

    SECTION("重新进入会再报一次 Enter") {
        triggers.BeginFrame();
        triggers.AddOverlap(zone, player);
        triggers.EndFrame(queue);

        queue.Clear();
        triggers.BeginFrame();
        triggers.EndFrame(queue);  // 出去

        queue.Clear();
        triggers.BeginFrame();
        triggers.AddOverlap(zone, player);
        triggers.EndFrame(queue);  // 又进来
        REQUIRE(CountEvents(queue, TriggerEventType::Enter) == 1);
    }
}

TEST_CASE("触发器 对键是无序的", "[trigger][scene]") {
    // (a,b) 和 (b,a) 必须是同一对。分不清的话同一次重叠会同时报出
    // Enter 和 Exit，每帧闪烁 —— 一个非常隐蔽的 bug。
    TriggerSystem triggers;
    const ColliderHandle a = MakeCollider(7);
    const ColliderHandle b = MakeCollider(3);
    EventQueue queue;

    triggers.BeginFrame();
    triggers.AddOverlap(a, b);
    triggers.EndFrame(queue);
    REQUIRE(CountEvents(queue, TriggerEventType::Enter) == 1);

    // 下一帧反过来报告
    queue.Clear();
    triggers.BeginFrame();
    triggers.AddOverlap(b, a);
    triggers.EndFrame(queue);

    REQUIRE(CountEvents(queue, TriggerEventType::Enter) == 0);
    REQUIRE(CountEvents(queue, TriggerEventType::Stay) == 1);
    REQUIRE(CountEvents(queue, TriggerEventType::Exit) == 0);
    REQUIRE(triggers.OverlapCount() == 1);

    REQUIRE(triggers.IsOverlapping(a, b));
    REQUIRE(triggers.IsOverlapping(b, a));
}

TEST_CASE("触发器 同一帧重复报告是幂等的", "[trigger][scene]") {
    // 一个刚体挂多个碰撞体时，宽相位可能对同一对报告多次。
    TriggerSystem triggers;
    EventQueue queue;
    const ColliderHandle zone = MakeCollider(1);
    const ColliderHandle player = MakeCollider(2);

    triggers.BeginFrame();
    triggers.AddOverlap(zone, player);
    triggers.AddOverlap(zone, player);
    triggers.AddOverlap(zone, player);
    triggers.EndFrame(queue);

    REQUIRE(queue.TriggerEvents().size() == 1);
    REQUIRE(triggers.OverlapCount() == 1);
}

TEST_CASE("触发器 销毁碰撞体会补发 Exit", "[trigger][scene]") {
    // 玩家死在买枪区里、道具被捡走 —— 少了这一条，游戏侧的"区域内对象列表"
    // 会永远留着一个幽灵条目。
    TriggerSystem triggers;
    EventQueue queue;
    const ColliderHandle zone = MakeCollider(1);
    const ColliderHandle player = MakeCollider(2);
    const ColliderHandle other = MakeCollider(3);

    triggers.BeginFrame();
    triggers.AddOverlap(zone, player);
    triggers.AddOverlap(zone, other);
    triggers.EndFrame(queue);
    REQUIRE(triggers.OverlapCount() == 2);

    // 玩家没了
    queue.Clear();
    triggers.RemoveCollider(player, queue);

    REQUIRE(queue.TriggerEvents().size() == 1);
    REQUIRE(queue.TriggerEvents()[0].type == TriggerEventType::Exit);
    REQUIRE(queue.TriggerEvents()[0].other == player);
    REQUIRE(triggers.OverlapCount() == 1);
    REQUIRE_FALSE(triggers.IsOverlapping(zone, player));
    REQUIRE(triggers.IsOverlapping(zone, other));
}

TEST_CASE("触发器 多个区域互不干扰", "[trigger][scene]") {
    TriggerSystem triggers;
    EventQueue queue;
    const ColliderHandle buyZone = MakeCollider(1);
    const ColliderHandle bombZone = MakeCollider(2);
    const ColliderHandle player = MakeCollider(10);

    triggers.BeginFrame();
    triggers.AddOverlap(buyZone, player);
    triggers.EndFrame(queue);

    // 走进炸弹区，同时还在买枪区里
    queue.Clear();
    triggers.BeginFrame();
    triggers.AddOverlap(buyZone, player);
    triggers.AddOverlap(bombZone, player);
    triggers.EndFrame(queue);

    REQUIRE(CountEvents(queue, TriggerEventType::Enter) == 1);
    REQUIRE(CountEvents(queue, TriggerEventType::Stay) == 1);

    // 走出买枪区，留在炸弹区
    queue.Clear();
    triggers.BeginFrame();
    triggers.AddOverlap(bombZone, player);
    triggers.EndFrame(queue);

    REQUIRE(CountEvents(queue, TriggerEventType::Exit) == 1);
    REQUIRE(CountEvents(queue, TriggerEventType::Stay) == 1);
    for (const TriggerEvent& e : queue.TriggerEvents()) {
        if (e.type == TriggerEventType::Exit) REQUIRE(e.trigger == buyZone);
        if (e.type == TriggerEventType::Stay) REQUIRE(e.trigger == bombZone);
    }
}

TEST_CASE("触发器 事件派发给监听器", "[trigger][scene]") {
    struct Listener : IPhysicsEventListener {
        int enters = 0, stays = 0, exits = 0;
        void OnTriggerEnter(ColliderHandle, ColliderHandle) override { ++enters; }
        void OnTriggerStay(ColliderHandle, ColliderHandle) override { ++stays; }
        void OnTriggerExit(ColliderHandle, ColliderHandle) override { ++exits; }
    };

    TriggerSystem triggers;
    EventQueue queue;
    Listener listener;
    const ColliderHandle zone = MakeCollider(1);
    const ColliderHandle player = MakeCollider(2);

    triggers.BeginFrame();
    triggers.AddOverlap(zone, player);
    triggers.EndFrame(queue);
    DispatchEvents(queue, &listener);
    REQUIRE(listener.enters == 1);

    queue.Clear();
    triggers.BeginFrame();
    triggers.AddOverlap(zone, player);
    triggers.EndFrame(queue);
    DispatchEvents(queue, &listener);
    REQUIRE(listener.stays == 1);

    queue.Clear();
    triggers.BeginFrame();
    triggers.EndFrame(queue);
    DispatchEvents(queue, &listener);
    REQUIRE(listener.exits == 1);

    // 空监听器是安全的
    DispatchEvents(queue, nullptr);
}
