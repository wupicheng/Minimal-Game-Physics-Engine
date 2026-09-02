//==============================================================================
// tests/test_world.cpp
//
// M9 PhysicsWorld 的测试。
//
//------------------------------------------------------------------------------
// 这一层测什么
//------------------------------------------------------------------------------
// World 自己没有算法，它只是**编排**。所以这里不重复验证前八个里程碑已经验过的
// 物理（那些测试压根没用到 World，这不是巧合），而是盯着"接线对不对"：
//
//   - 生命周期：句柄代际失效、销毁刚体会连带碰撞体、id 复用
//   - 质量合成：多个碰撞体的复合质心与惯量
//   - 固定步长：同一段时间在不同帧率下走出同样的结果、死亡螺旋不会卡死
//   - CCD：抛射物不再穿墙（M8 留下的 P0）
//   - fat AABB 的速度预测：格子重建率必须掉下来（M3 留下的 P0）
//   - 查询：射线、扫掠、重叠，以及"结果带正确的句柄"
//   - 事件：触发器与接触事件在 Step 末尾统一派发
//   - 端到端：完整场景跑几秒不崩、不发散、结果合理
//==============================================================================

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <vector>

#include "pe/scene/PhysicsWorld.h"

using namespace pe;
using Catch::Approx;

namespace {

constexpr real kTol = real(1e-3);

bool Eq(const Vec3& a, const Vec3& b, real tol = kTol) { return NearlyEqual(a, b, tol); }

/// 造一个静态盒（顶面在 topY）
BodyHandle AddStaticBox(PhysicsWorld& world, const Vec3& center, const Vec3& halfExtents,
                        const Material& material = Material::Wood()) {
    BodyDesc desc;
    desc.position = center;
    desc.type = BodyType::Static;
    const BodyHandle body = world.CreateBody(desc);

    ColliderDesc collider;
    collider.shape = Shape::MakeBox(halfExtents);
    collider.material = material;
    collider.layer = layers::kWorld;
    world.AddCollider(body, collider);
    return body;
}

BodyHandle AddDynamicBox(PhysicsWorld& world, const Vec3& center, real half, real mass,
                         const Material& material = Material::Wood()) {
    BodyDesc desc;
    desc.position = center;
    desc.type = BodyType::Dynamic;
    desc.mass = mass;
    desc.linearDamping = real(0);
    desc.angularDamping = real(0);
    const BodyHandle body = world.CreateBody(desc);

    ColliderDesc collider;
    collider.shape = Shape::MakeCube(half);
    collider.material = material;
    world.AddCollider(body, collider);
    return body;
}

/// 铺一块地面，顶面在 y = 0
BodyHandle AddGround(PhysicsWorld& world) {
    return AddStaticBox(world, Vec3(0, real(-5), 0), Vec3(50, 5, 50));
}

void RunSeconds(PhysicsWorld& world, real seconds, real frameTime = real(1) / real(60)) {
    const int frames = static_cast<int>(seconds / frameTime + real(0.5));
    for (int i = 0; i < frames; ++i) world.Step(frameTime);
}

}  // namespace

//==============================================================================
// A. 生命周期
//==============================================================================

TEST_CASE("World 刚体与碰撞体的生命周期", "[world][scene]") {
    PhysicsWorld world;

    SECTION("创建与销毁") {
        const BodyHandle body = AddDynamicBox(world, Vec3(0, 5, 0), real(0.5), real(10));
        REQUIRE(world.BodyCount() == 1);
        REQUIRE(world.ColliderCount() == 1);
        REQUIRE(world.GetBody(body) != nullptr);

        world.DestroyBody(body);
        REQUIRE(world.BodyCount() == 0);
        // 销毁刚体会连带销毁它的碰撞体
        REQUIRE(world.ColliderCount() == 0);
        REQUIRE(world.GetBody(body) == nullptr);
    }

    SECTION("句柄的代际号能挡住 复用后误用") {
        const BodyHandle first = AddDynamicBox(world, Vec3(0, 5, 0), real(0.5), real(10));
        world.DestroyBody(first);
        const BodyHandle second = AddDynamicBox(world, Vec3(0, 9, 0), real(0.5), real(10));

        // 槽位被复用了，但旧句柄必须失效 —— 这正是代际号存在的理由
        REQUIRE(second.index == first.index);
        REQUIRE(second.generation != first.generation);
        REQUIRE(world.GetBody(first) == nullptr);
        REQUIRE(world.GetBody(second) != nullptr);
    }

    SECTION("销毁不存在的刚体是安全的") {
        world.DestroyBody(BodyHandle::Null());
        const BodyHandle body = AddDynamicBox(world, Vec3(0, 5, 0), real(0.5), real(10));
        world.DestroyBody(body);
        world.DestroyBody(body);  // 重复销毁
        REQUIRE(world.BodyCount() == 0);
    }

    SECTION("销毁之后宽相位里也不该留东西") {
        const BodyHandle body = AddDynamicBox(world, Vec3(0, 5, 0), real(0.5), real(10));
        REQUIRE(world.BroadPhase().ProxyCount() == 1);
        world.DestroyBody(body);
        REQUIRE(world.BroadPhase().ProxyCount() == 0);
    }

    SECTION("传送会同步刷新宽相位") {
        const BodyHandle body = AddDynamicBox(world, Vec3(0, 5, 0), real(0.5), real(10));
        world.SetBodyTransform(body, Transform(Vec3(100, 5, 0), Quat::Identity()));

        std::vector<ColliderHandle> found;
        world.OverlapAABB(AABB::FromCenterHalfExtents(Vec3(100, 5, 0), Vec3(1, 1, 1)),
                          found);
        REQUIRE(found.size() == 1);

        // 旧位置查不到了
        world.OverlapAABB(AABB::FromCenterHalfExtents(Vec3(0, 5, 0), Vec3(1, 1, 1)),
                          found);
        REQUIRE(found.empty());
    }
}

//==============================================================================
// B. 质量属性的合成
//==============================================================================

TEST_CASE("World 多碰撞体的质量合成", "[world][scene]") {
    PhysicsWorld world;

    SECTION("单个居中碰撞体：质心就在原点") {
        const BodyHandle body = AddDynamicBox(world, Vec3(0, 5, 0), real(0.5), real(10));
        const RigidBody* rb = world.GetBody(body);
        REQUIRE(rb->Mass() == Approx(real(10)).epsilon(real(1e-4)));
        // 原点位姿与质心位姿重合
        REQUIRE(Eq(world.GetBodyTransform(body).position, rb->position));
    }

    SECTION("两个对称摆放的碰撞体：质心仍在中间") {
        BodyDesc desc;
        desc.position = Vec3(0, 5, 0);
        desc.type = BodyType::Dynamic;
        const BodyHandle body = world.CreateBody(desc);

        ColliderDesc left;
        left.shape = Shape::MakeCube(real(0.5));
        left.localTransform = Transform(Vec3(-1, 0, 0), Quat::Identity());
        world.AddCollider(body, left);

        ColliderDesc right = left;
        right.localTransform = Transform(Vec3(1, 0, 0), Quat::Identity());
        world.AddCollider(body, right);

        // 对称 -> 质心在原点，物体原点没有跳
        REQUIRE(Eq(world.GetBodyTransform(body).position, Vec3(0, 5, 0)));
        // 质量是两块之和
        const real oneBlock = densities::kDefault * Shape::MakeCube(real(0.5)).Volume();
        REQUIRE(world.GetBody(body)->Mass() ==
                Approx(oneBlock * real(2)).epsilon(real(1e-3)));
    }

    SECTION("偏心的碰撞体：质心偏移，但物体原点不动") {
        BodyDesc desc;
        desc.position = Vec3(0, 5, 0);
        desc.type = BodyType::Dynamic;
        const BodyHandle body = world.CreateBody(desc);

        ColliderDesc offset;
        offset.shape = Shape::MakeCube(real(0.5));
        offset.localTransform = Transform(Vec3(2, 0, 0), Quat::Identity());
        world.AddCollider(body, offset);

        // 这是关键：加一个偏心碰撞体不该让物体"自己跳一下"
        REQUIRE(Eq(world.GetBodyTransform(body).position, Vec3(0, 5, 0)));
        // 但内部的质心确实偏到了碰撞体那边
        REQUIRE(Eq(world.GetBody(body)->position, Vec3(2, 5, 0)));
    }

    SECTION("复合形状的惯量比单块大（质量被推远了）") {
        BodyDesc desc;
        desc.type = BodyType::Dynamic;
        const BodyHandle single = world.CreateBody(desc);
        ColliderDesc c;
        c.shape = Shape::MakeCube(real(0.5));
        world.AddCollider(single, c);
        world.AddCollider(single, c);  // 两块叠在原点

        const BodyHandle spread = world.CreateBody(desc);
        ColliderDesc a = c;
        a.localTransform = Transform(Vec3(-2, 0, 0), Quat::Identity());
        ColliderDesc b = c;
        b.localTransform = Transform(Vec3(2, 0, 0), Quat::Identity());
        world.AddCollider(spread, a);
        world.AddCollider(spread, b);

        // 质量一样，但摊开的那个绕 Y 轴更难转 -> invInertia 更小
        REQUIRE(world.GetBody(single)->Mass() ==
                Approx(world.GetBody(spread)->Mass()).epsilon(real(1e-4)));
        REQUIRE(world.GetBody(spread)->invInertiaLocal.rows[1].y <
                world.GetBody(single)->invInertiaLocal.rows[1].y);
    }

    SECTION("没有碰撞体的动态体不会因为无穷大质量而乱飞") {
        BodyDesc desc;
        desc.type = BodyType::Dynamic;
        const BodyHandle body = world.CreateBody(desc);
        REQUIRE(world.GetBody(body)->invMass == real(0));
        RunSeconds(world, real(1));
        REQUIRE(IsFinite(world.GetBody(body)->position.y));
    }
}

//==============================================================================
// C. 固定步长
//==============================================================================

TEST_CASE("World 固定步长让结果与帧率无关", "[world][scene]") {
    // 这是固定步长存在的全部理由：同一段真实时间，无论渲染帧率多少，
    // 物理必须走出（几乎）一样的结果。跳跃高度随帧率变化是这类 bug 的经典症状。
    const auto fall = [](real frameTime) {
        PhysicsWorld world;
        AddGround(world);
        const BodyHandle box = AddDynamicBox(world, Vec3(0, 10, 0), real(0.5), real(10));
        RunSeconds(world, real(1), frameTime);
        return world.GetBody(box)->position.y;
    };

    const real at60 = fall(real(1) / real(60));
    const real at30 = fall(real(1) / real(30));
    const real at144 = fall(real(1) / real(144));

    INFO("60fps: " << at60 << "  30fps: " << at30 << "  144fps: " << at144);
    // 累加器的余数会带来不到一个物理步的差异，所以容差取一步的位移量级
    REQUIRE(at30 == Approx(at60).margin(real(0.2)));
    REQUIRE(at144 == Approx(at60).margin(real(0.2)));
}

TEST_CASE("World 大延迟不会引发死亡螺旋", "[world][scene]") {
    PhysicsWorld world;
    AddGround(world);
    AddDynamicBox(world, Vec3(0, 5, 0), real(0.5), real(10));

    // 塞一个荒唐的 dt（模拟一次 2 秒的卡顿）
    world.Step(real(2));

    // 子步数被 maxSubSteps 限住，多余的时间被丢弃
    REQUIRE(world.GetStats().subSteps == world.Config().maxSubSteps);
    // 下一帧不该还背着一屁股债
    world.Step(real(1) / real(60));
    REQUIRE(world.GetStats().subSteps <= world.Config().maxSubSteps);
}

TEST_CASE("World 累加器不足一步时不推进", "[world][scene]") {
    PhysicsWorld world;
    AddGround(world);
    const BodyHandle box = AddDynamicBox(world, Vec3(0, 5, 0), real(0.5), real(10));
    const Vec3 start = world.GetBody(box)->position;

    world.Step(real(0.001));  // 远小于 1/60
    REQUIRE(world.GetStats().subSteps == 0);
    REQUIRE(Eq(world.GetBody(box)->position, start));
    // 但 alpha 应该动了 —— 渲染层靠它插值
    REQUIRE(world.Alpha() > real(0));
}

TEST_CASE("World 渲染插值", "[world][scene]") {
    PhysicsWorld world;
    const BodyHandle box = AddDynamicBox(world, Vec3(0, 10, 0), real(0.5), real(10));

    world.Step(real(1) / real(60));
    world.Step(real(1) / real(60));

    const Transform t0 = world.GetInterpolatedTransform(box, real(0));
    const Transform t1 = world.GetInterpolatedTransform(box, real(1));
    const Transform mid = world.GetInterpolatedTransform(box, real(0.5));

    // 自由落体：alpha=1 比 alpha=0 更低，中点在两者之间
    REQUIRE(t1.position.y < t0.position.y);
    REQUIRE(mid.position.y < t0.position.y);
    REQUIRE(mid.position.y > t1.position.y);
    REQUIRE(mid.position.y == Approx((t0.position.y + t1.position.y) * real(0.5))
                                  .margin(real(1e-4)));
}

//==============================================================================
// D. CCD —— M8 留下的 P0
//==============================================================================

TEST_CASE("World CCD 让高速抛射物不穿墙", "[world][scene]") {
    // 一颗小球以 300 m/s 撞 20 厘米厚的墙。一帧走 5 米，墙厚 0.2 米 ——
    // 纯离散积分必然穿过去。
    const auto runProjectile = [](bool enableCcd) {
        WorldConfig config;
        config.enableCcd = enableCcd;
        config.gravity = Vec3::Zero();  // 排除重力干扰，只看穿不穿墙
        PhysicsWorld world(config);

        AddStaticBox(world, Vec3(10, 0, 0), Vec3(real(0.1), 5, 5));

        BodyDesc desc;
        desc.position = Vec3(0, 0, 0);
        desc.type = BodyType::Dynamic;
        desc.mass = real(0.1);
        desc.linearVelocity = Vec3(300, 0, 0);
        desc.linearDamping = real(0);
        const BodyHandle bullet = world.CreateBody(desc);

        ColliderDesc collider;
        collider.shape = Shape::MakeSphere(real(0.05));
        world.AddCollider(bullet, collider);

        RunSeconds(world, real(0.5));
        return world.GetBody(bullet)->position.x;
    };

    const real withCcd = runProjectile(true);
    const real withoutCcd = runProjectile(false);

    INFO("开 CCD 停在 x = " << withCcd << "；关掉之后 x = " << withoutCcd);

    // 开着 CCD：被墙拦在近面（x = 9.9）之前
    REQUIRE(withCcd < real(10));
    // 关掉：直接穿过去飞走了
    REQUIRE(withoutCcd > real(20));
}

TEST_CASE("World CCD 只对高速物体生效", "[world][scene]") {
    WorldConfig config;
    PhysicsWorld world(config);
    AddGround(world);
    // 一堆慢速的箱子：不该触发任何扫掠
    for (int i = 0; i < 10; ++i) {
        AddDynamicBox(world, Vec3(real(i) * real(2), real(0.5), 0), real(0.5), real(10));
    }

    RunSeconds(world, real(1));
    INFO("CCD 扫掠次数 " << world.GetStats().ccdSweeps);
    REQUIRE(world.GetStats().ccdSweeps == 0);
}

//==============================================================================
// E. 速度感知的 fat AABB —— M3 留下的 P0
//==============================================================================

TEST_CASE("World 速度预测把宽相位重建率压下来", "[world][scene]") {
    // M3 的实测教训：只用固定 margin（0.05 米）时，5 m/s 的物体每帧走 0.083 米，
    // 每帧都跨出膨胀壳 —— 实测 76% 的 Update 触发重建，fat AABB 等于没开。
    //
    // 这条用例把"关掉速度预测"和"开着"放在一起跑，量化那个修复。
    const auto relinkRatio = [](real prediction) {
        WorldConfig config;
        config.aabbVelocityPrediction = prediction;
        PhysicsWorld world(config);
        AddGround(world);

        // 一批横着匀速飞的箱子（关掉重力，保证速度稳定）
        for (int i = 0; i < 20; ++i) {
            BodyDesc desc;
            desc.position = Vec3(real(i) * real(3), 5, 0);
            desc.type = BodyType::Dynamic;
            desc.mass = real(10);
            desc.linearVelocity = Vec3(5, 0, 0);
            desc.linearDamping = real(0);
            desc.useGravity = false;
            const BodyHandle body = world.CreateBody(desc);
            ColliderDesc c;
            c.shape = Shape::MakeCube(real(0.5));
            world.AddCollider(body, c);
        }

        real sum = real(0);
        int samples = 0;
        for (int i = 0; i < 120; ++i) {
            world.Step(real(1) / real(60));
            sum += world.GetStats().proxyRelinkRatio;
            ++samples;
        }
        return sum / real(samples);
    };

    const real withoutPrediction = relinkRatio(real(0));
    const real withPrediction = relinkRatio(real(3));

    INFO("关掉速度预测的重建率 " << withoutPrediction << "，开着 " << withPrediction);
    // 修复必须是**显著**的，不是聊胜于无
    REQUIRE(withoutPrediction > real(0.3));
    REQUIRE(withPrediction < withoutPrediction * real(0.5));
}

//==============================================================================
// F. 查询
//==============================================================================

TEST_CASE("World 射线查询", "[world][scene]") {
    PhysicsWorld world;
    AddGround(world);
    const BodyHandle near = AddStaticBox(world, Vec3(5, 1, 0), Vec3(1, 1, 1));
    AddStaticBox(world, Vec3(15, 1, 0), Vec3(1, 1, 1));

    SECTION("取最近的那个，并且带回句柄") {
        WorldRaycastHit hit;
        REQUIRE(world.Raycast(Ray(Vec3(0, 1, 0), Vec3(1, 0, 0), real(100)), hit));
        REQUIRE(hit.distance == Approx(real(4)).margin(real(0.01)));
        REQUIRE(hit.body == near);
        REQUIRE(Eq(hit.normal, Vec3(-1, 0, 0), real(0.01)));
        // 身份信息是 World 这一层补上的（collision 层不知道句柄的存在）
        REQUIRE(world.GetCollider(hit.collider) != nullptr);
    }

    SECTION("射程不够就打不到") {
        WorldRaycastHit hit;
        REQUIRE_FALSE(world.Raycast(Ray(Vec3(0, 1, 0), Vec3(1, 0, 0), real(3)), hit));
    }

    SECTION("层过滤") {
        WorldRaycastHit hit;
        // 所有静态体都在 kWorld 层，屏蔽掉就什么都打不到
        REQUIRE_FALSE(
            world.Raycast(Ray(Vec3(0, 1, 0), Vec3(1, 0, 0), real(100)), hit,
                          ~layers::kWorld));
    }

    SECTION("触发器不挡子弹") {
        BodyDesc desc;
        desc.position = Vec3(real(2.5), 1, 0);
        desc.type = BodyType::Static;
        const BodyHandle zone = world.CreateBody(desc);
        ColliderDesc trigger;
        trigger.shape = Shape::MakeCube(real(0.5));
        trigger.isTrigger = true;
        trigger.layer = layers::kWorld;
        world.AddCollider(zone, trigger);

        WorldRaycastHit hit;
        REQUIRE(world.Raycast(Ray(Vec3(0, 1, 0), Vec3(1, 0, 0), real(100)), hit));
        // 打中的还是后面那个实心盒，不是触发区
        REQUIRE(hit.body == near);
    }
}

TEST_CASE("World 重叠查询", "[world][scene]") {
    PhysicsWorld world;
    AddStaticBox(world, Vec3(0, 0, 0), Vec3(1, 1, 1));
    AddStaticBox(world, Vec3(10, 0, 0), Vec3(1, 1, 1));

    std::vector<ColliderHandle> found;

    SECTION("AABB 重叠") {
        world.OverlapAABB(AABB::FromCenterHalfExtents(Vec3(0, 0, 0), Vec3(2, 2, 2)),
                          found);
        REQUIRE(found.size() == 1);
    }

    SECTION("形状重叠是精确的，不只是 AABB") {
        // 一个小球放在盒子的角外面：AABB 会重叠，精确判定不该重叠
        world.OverlapShape(Shape::MakeSphere(real(0.1)),
                           Transform(Vec3(real(1.5), real(1.5), real(1.5)),
                                     Quat::Identity()),
                           found);
        REQUIRE(found.empty());

        // 挪到盒子里面就该找到
        world.OverlapShape(Shape::MakeSphere(real(0.1)),
                           Transform(Vec3(0, 0, 0), Quat::Identity()), found);
        REQUIRE(found.size() == 1);
    }

    SECTION("爆炸范围：一次拿到所有受影响的碰撞体") {
        for (int i = 0; i < 5; ++i) {
            AddDynamicBox(world, Vec3(real(i) * real(0.5), 5, 0), real(0.2), real(1));
        }
        world.OverlapAABB(AABB::FromCenterHalfExtents(Vec3(1, 5, 0), Vec3(3, 3, 3)),
                          found);
        REQUIRE(found.size() == 5);
    }
}

TEST_CASE("World 形状扫掠查询", "[world][scene]") {
    PhysicsWorld world;
    AddStaticBox(world, Vec3(10, 0, 0), Vec3(1, 5, 5));

    WorldShapeCastHit hit;
    REQUIRE(world.ShapeCastWorld(Shape::MakeSphere(real(0.5)),
                                 Transform(Vec3(0, 0, 0), Quat::Identity()),
                                 Vec3(20, 0, 0), hit));
    // 墙近面 x=9，球心停在 8.5 -> fraction = 0.425
    REQUIRE(hit.fraction == Approx(real(0.425)).margin(real(0.005)));
    REQUIRE(world.GetCollider(hit.collider) != nullptr);
}

//==============================================================================
// G. 事件
//==============================================================================

namespace {

struct RecordingListener : IPhysicsEventListener {
    int triggerEnters = 0;
    int triggerStays = 0;
    int triggerExits = 0;
    int contactBegins = 0;
    int contactEnds = 0;

    void OnTriggerEnter(ColliderHandle, ColliderHandle) override { ++triggerEnters; }
    void OnTriggerStay(ColliderHandle, ColliderHandle) override { ++triggerStays; }
    void OnTriggerExit(ColliderHandle, ColliderHandle) override { ++triggerExits; }
    void OnContactBegin(const ContactEvent&) override { ++contactBegins; }
    void OnContactEnd(const ContactEvent&) override { ++contactEnds; }
};

}  // namespace

TEST_CASE("World 触发器事件", "[world][scene]") {
    PhysicsWorld world;
    AddGround(world);

    // 一个悬空的触发区
    BodyDesc zoneDesc;
    zoneDesc.position = Vec3(0, 5, 0);
    zoneDesc.type = BodyType::Static;
    const BodyHandle zone = world.CreateBody(zoneDesc);
    ColliderDesc zoneCollider;
    zoneCollider.shape = Shape::MakeCube(real(1));
    zoneCollider.isTrigger = true;
    world.AddCollider(zone, zoneCollider);

    // 一个从上面掉下来、穿过触发区的箱子
    const BodyHandle box = AddDynamicBox(world, Vec3(0, 10, 0), real(0.3), real(5));

    RecordingListener listener;
    world.SetEventListener(&listener);

    RunSeconds(world, real(3));

    INFO("Enter " << listener.triggerEnters << " Stay " << listener.triggerStays
                  << " Exit " << listener.triggerExits);

    // 穿过去一次：进一次、出一次，中间 Stay 若干帧
    REQUIRE(listener.triggerEnters == 1);
    REQUIRE(listener.triggerExits == 1);
    REQUIRE(listener.triggerStays > 0);

    // 触发器不产生约束，箱子该直接穿过去落到地面
    REQUIRE(world.GetBody(box)->position.y == Approx(real(0.3)).margin(real(0.05)));
}

TEST_CASE("World 接触事件", "[world][scene]") {
    PhysicsWorld world;
    AddGround(world);
    AddDynamicBox(world, Vec3(0, 3, 0), real(0.5), real(10));

    RecordingListener listener;
    world.SetEventListener(&listener);

    RunSeconds(world, real(2));

    INFO("Begin " << listener.contactBegins << " End " << listener.contactEnds);
    // 落地产生一次接触开始；停在地上之后不该反复 Begin/End
    REQUIRE(listener.contactBegins >= 1);
    REQUIRE(listener.contactBegins <= 3);
}

TEST_CASE("World 事件回调里增删物体是安全的", "[world][scene]") {
    // 这是把事件做成队列（而不是直接回调）的全部理由：回调里创建/销毁物体
    // 会让引擎正在遍历的容器失效。事件在 Step 末尾统一派发，那时是安全的。
    struct SpawningListener : IPhysicsEventListener {
        PhysicsWorld* world = nullptr;
        int spawned = 0;

        void OnContactBegin(const ContactEvent&) override {
            if (spawned >= 5) return;
            BodyDesc desc;
            desc.position = Vec3(real(spawned) * real(2), 8, 0);
            desc.type = BodyType::Dynamic;
            desc.mass = real(1);
            const BodyHandle body = world->CreateBody(desc);
            ColliderDesc c;
            c.shape = Shape::MakeCube(real(0.2));
            world->AddCollider(body, c);
            ++spawned;
        }
    };

    PhysicsWorld world;
    AddGround(world);
    AddDynamicBox(world, Vec3(0, 3, 0), real(0.5), real(10));

    SpawningListener listener;
    listener.world = &world;
    world.SetEventListener(&listener);

    RunSeconds(world, real(3));

    REQUIRE(listener.spawned > 0);
    REQUIRE(world.BodyCount() == 2u + static_cast<std::size_t>(listener.spawned));
}

TEST_CASE("World 回调里销毁触发器本身也是安全的", "[world][scene][trigger]") {
    //--------------------------------------------------------------------------
    // 回归测试：拾取物的标准写法是"OnTriggerEnter 里把自己销毁掉"。
    //
    // 销毁碰撞体会让 TriggerSystem 往**正在派发的那个队列**里补发 Exit ——
    // 队列一扩容，用 range-for 遍历它的派发循环就拿着野指针继续走。
    // 症状是"走过弹药箱就闪退"，而且崩溃点在引擎里、离原因隔着好几层栈。
    //--------------------------------------------------------------------------
    struct SelfDestroyingPickup : IPhysicsEventListener {
        PhysicsWorld* world = nullptr;
        int picked = 0;
        int exits = 0;

        void OnTriggerEnter(ColliderHandle trigger, ColliderHandle) override {
            const Collider* collider = world->GetCollider(trigger);
            if (collider == nullptr) return;
            ++picked;
            world->DestroyBody(collider->body);  // <- 这里会往队列里补发 Exit
        }
        void OnTriggerExit(ColliderHandle, ColliderHandle) override { ++exits; }
    };

    PhysicsWorld world;

    // 一排叠在一起的拾取区，让同一帧里产生多条 Enter —— 单条事件的队列
    // 不一定会扩容，扩容才会踩到那根野指针
    constexpr int kPickupCount = 16;
    for (int i = 0; i < kPickupCount; ++i) {
        BodyDesc desc;
        desc.position = Vec3(0, 1, 0);
        desc.type = BodyType::Static;
        const BodyHandle body = world.CreateBody(desc);

        ColliderDesc collider;
        collider.shape = Shape::MakeCube(real(1));
        collider.isTrigger = true;
        collider.layer = layers::kTrigger;
        world.AddCollider(body, collider);
    }

    // 一个"玩家"停在拾取区里
    BodyDesc playerDesc;
    playerDesc.position = Vec3(0, 1, 0);
    playerDesc.type = BodyType::Kinematic;
    const BodyHandle player = world.CreateBody(playerDesc);
    ColliderDesc playerCollider;
    playerCollider.shape = Shape::MakeCube(real(0.4));
    playerCollider.layer = layers::kPlayer;
    world.AddCollider(player, playerCollider);

    SelfDestroyingPickup listener;
    listener.world = &world;
    world.SetEventListener(&listener);

    RunSeconds(world, real(0.5));

    // 每个拾取区都被捡到，而且都从世界里消失了（只剩玩家）
    REQUIRE(listener.picked == kPickupCount);
    REQUIRE(world.BodyCount() == 1u);
    // 补发的 Exit 也确实送到了游戏手里 —— 少了它，游戏侧的"区域内对象列表"
    // 会留下幽灵条目
    REQUIRE(listener.exits == kPickupCount);
}

//==============================================================================
// H. 端到端
//==============================================================================

TEST_CASE("World 箱堆在完整管线里也稳定", "[world][scene]") {
    // M7 是在测试自建的 MiniWorld 里验的堆叠。这里换成真正的 World ——
    // 宽相位、休眠、固定步长、CCD 全都在，验证接线没有破坏物理。
    PhysicsWorld world;
    AddGround(world);

    std::vector<BodyHandle> boxes;
    for (int i = 0; i < 8; ++i) {
        boxes.push_back(AddDynamicBox(
            world, Vec3(0, real(0.5) + real(i) * real(1.02), 0), real(0.5), real(10)));
    }

    RunSeconds(world, real(5));

    const RigidBody* bottom = world.GetBody(boxes.front());
    const RigidBody* top = world.GetBody(boxes.back());

    INFO("最底层 y = " << bottom->position.y << "，最顶层 y = " << top->position.y
                      << "，最大穿透 " << world.GetStats().maxPenetration);

    REQUIRE(bottom->position.y > real(0.45));
    REQUIRE(top->position.y > real(0.5) + real(7) * real(1.02) - real(0.3));
    REQUIRE(Abs(top->position.x) < real(0.5));
    REQUIRE(world.GetStats().maxPenetration < kLinearSlop * real(4));
}

TEST_CASE("World 休眠会让物体退出模拟", "[world][scene]") {
    PhysicsWorld world;
    AddGround(world);
    AddDynamicBox(world, Vec3(0, real(0.5), 0), real(0.5), real(10));

    RunSeconds(world, real(5));

    INFO("活跃刚体数 " << world.GetStats().activeBodies);
    // 静止的箱子应该睡着了，不再参与积分与求解
    REQUIRE(world.GetStats().activeBodies == 0);
}

TEST_CASE("World 角色在真实世界里走动", "[world][scene][character]") {
    // PhysicsWorld 实现了 ICharacterWorld，所以角色控制器可以直接用它。
    // M8 里角色是在二十行的假世界里测的，这里换成真的。
    PhysicsWorld world;
    AddGround(world);
    AddStaticBox(world, Vec3(5, 5, 0), Vec3(real(0.5), 5, 20));  // 一面墙

    CharacterController character;
    character.SetFootPosition(Vec3(0, 0, 0));
    character.UpdateGrounded(world);
    REQUIRE(character.isGrounded);

    for (int i = 0; i < 180; ++i) {
        world.Step(real(1) / real(60));
        character.Update(world, Vec3(3, 0, 0), real(1) / real(60), world.Config().gravity);
    }

    INFO("角色停在 x = " << character.position.x);
    // 被墙拦住（墙近面 x=4.5，减去角色半径 0.4）
    REQUIRE(character.position.x < real(4.2));
    REQUIRE(character.position.x > real(3.8));
    REQUIRE(character.isGrounded);
}

TEST_CASE("World 大场景跑几秒不发散", "[world][scene]") {
    // 冒烟测试：接近性能目标的规模，跑几秒，只要求"所有数值都是有限的"。
    // 数值发散（NaN / inf）是物理引擎最致命的失败模式 —— 它会像瘟疫一样
    // 通过接触传播，一帧之内让整个场景消失。
    PhysicsWorld world;
    AddGround(world);

    // 静态地图
    for (int i = 0; i < 40; ++i) {
        AddStaticBox(world, Vec3(real(i % 8) * 6 - 21, 1, real(i / 8) * 6 - 12),
                     Vec3(1, 1, 1));
    }
    // 动态物体
    for (int i = 0; i < 60; ++i) {
        AddDynamicBox(world, Vec3(real(i % 10) * real(1.5) - 7, real(3 + i / 10),
                                  real((i / 10) % 3) * real(1.5)),
                      real(0.4), real(5));
    }

    RunSeconds(world, real(4));

    bool allFinite = true;
    world.ForEachBody([&](BodyHandle, const RigidBody& body) {
        allFinite = allFinite && IsFinite(body.position.x) && IsFinite(body.position.y) &&
                    IsFinite(body.position.z) && IsFinite(body.linearVelocity.x) &&
                    IsFinite(body.angularVelocity.x);
        // 也不该有物体飞到天上去
        allFinite = allFinite && body.position.y > real(-50) && body.position.y < real(200);
    });

    INFO("候选对 " << world.GetStats().broadPhasePairs << "，接触 "
                  << world.GetStats().narrowPhaseContacts << "，活跃体 "
                  << world.GetStats().activeBodies);
    REQUIRE(allFinite);
    REQUIRE(world.BodyCount() == 101);
}
