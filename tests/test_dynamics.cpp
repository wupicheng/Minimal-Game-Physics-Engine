//==============================================================================
// tests/test_dynamics.cpp
//
// M6 刚体动力学（质量属性 + 积分器）的测试。
//
//------------------------------------------------------------------------------
// 这一层最难验证的是什么
//------------------------------------------------------------------------------
// 惯量张量的公式里那一串系数（2/5、1/12、3/8…）单看数字完全无法判断对错，
// 而写错之后的症状是"物体翻滚起来感觉怪怪的"—— 一种没人能准确描述、
// 也没人知道该从哪儿查起的 bug。
//
// 所以这里用**数值积分**做独立验证：在形状里撒几十万个均匀分布的点，
// 直接按定义 I_xx = Σ m_i * (y_i^2 + z_i^2) 把张量加出来，和解析公式对比。
// 这是唯一真正独立的检查手段 —— 蒙特卡洛积分和闭式公式之间没有任何共享代码。
//
// 积分器那边则对着**有解析解的物理问题**验：
//   - 自由落体：位置误差必须是 O(dt^2) 而不是 O(dt)
//   - 无外力自由旋转：角动量必须守恒（这一条专门盯着陀螺项）
//   - 阻尼：必须是指数衰减，且与步长无关
//   - 半隐式 vs 显式：简谐振子的能量必须有界，不能单调增长
//==============================================================================

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cstdint>

#include "pe/dynamics/Integrator.h"
#include "pe/dynamics/MassProperties.h"
#include "pe/dynamics/Material.h"

using namespace pe;
using Catch::Approx;

namespace {

constexpr real kTol = real(1e-4);

bool Eq(const Vec3& a, const Vec3& b, real tol = kTol) { return NearlyEqual(a, b, tol); }

struct Rng {
    std::uint32_t state;
    explicit Rng(std::uint32_t seed) : state(seed) {}
    std::uint32_t NextU32() {
        state = state * 1664525u + 1013904223u;
        return state;
    }
    real Unit() { return real(NextU32() >> 8) / real(1 << 24); }
    real Range(real lo, real hi) { return lo + (hi - lo) * Unit(); }
    Vec3 InCube(real half) {
        return Vec3(Range(-half, half), Range(-half, half), Range(-half, half));
    }
};

/// 点在不在形状里（独立实现，不依赖窄相位）
bool PointInside(const Shape& s, const Vec3& p) {
    switch (s.type) {
        case ShapeType::Sphere:
            return p.LengthSq() <= s.sphere.radius * s.sphere.radius;
        case ShapeType::Capsule: {
            const real hh = s.capsule.halfHeight;
            const Vec3 closest(real(0), Clamp(p.y, -hh, hh), real(0));
            return (p - closest).LengthSq() <= s.capsule.radius * s.capsule.radius;
        }
        case ShapeType::Box: {
            const Vec3& h = s.box.halfExtents;
            return Abs(p.x) <= h.x && Abs(p.y) <= h.y && Abs(p.z) <= h.z;
        }
    }
    return false;
}

//------------------------------------------------------------------------------
// 蒙特卡洛法算惯量张量
//
// 在形状的包围盒里均匀撒点，落在形状内的点各代表一小块等质量的体积元，
// 直接按定义累加：
//     I_xx = Σ m_i (y_i^2 + z_i^2)
//     I_xy = -Σ m_i x_i y_i
// 这和解析公式完全没有共享代码，所以它是一个真正独立的判据。
//
// 收敛速度是 O(1/sqrt(N))，所以对比时容差要给到百分之一量级 ——
// 这已经足够抓住"系数写成 1/12 还是 1/3""下标搞反"这类错误了，
// 它们的偏差都在几十个百分点以上。
//------------------------------------------------------------------------------
Mat3 MonteCarloInertia(const Shape& shape, real density, int samples,
                       std::uint32_t seed, real& outMass) {
    Rng rng(seed);
    const AABB box = shape.LocalAABB();
    const Vec3 size = box.Size();
    const real boxVolume = size.x * size.y * size.z;

    int inside = 0;
    real ixx = real(0), iyy = real(0), izz = real(0);
    real ixy = real(0), ixz = real(0), iyz = real(0);

    for (int i = 0; i < samples; ++i) {
        const Vec3 p(rng.Range(box.min.x, box.max.x), rng.Range(box.min.y, box.max.y),
                     rng.Range(box.min.z, box.max.z));
        if (!PointInside(shape, p)) continue;
        ++inside;
        ixx += p.y * p.y + p.z * p.z;
        iyy += p.x * p.x + p.z * p.z;
        izz += p.x * p.x + p.y * p.y;
        ixy -= p.x * p.y;
        ixz -= p.x * p.z;
        iyz -= p.y * p.z;
    }

    // 每个采样点代表的质量 = 总体积 * 命中率 / 命中数 * 密度
    const real volume = boxVolume * real(inside) / real(samples);
    outMass = volume * density;
    const real perSample = (inside > 0) ? outMass / real(inside) : real(0);

    return Mat3(Vec3(ixx, ixy, ixz) * perSample, Vec3(ixy, iyy, iyz) * perSample,
                Vec3(ixz, iyz, izz) * perSample);
}

/// 相对误差
real RelativeError(real actual, real expected) {
    if (Abs(expected) < real(1e-9)) return Abs(actual);
    return Abs(actual - expected) / Abs(expected);
}

}  // namespace

//==============================================================================
// A. 质量属性
//==============================================================================

TEST_CASE("动力学 质量由密度和体积算出", "[dynamics]") {
    SECTION("球") {
        const Shape s = Shape::MakeSphere(real(2));
        const MassProperties mp = ComputeMassProperties(s, real(1000));
        // V = 4/3 pi r^3 = 33.5103
        REQUIRE(mp.mass == Approx(real(1000) * s.Volume()).epsilon(real(1e-5)));
        REQUIRE(Eq(mp.centerOfMass, Vec3::Zero()));
    }

    SECTION("按指定质量反推") {
        const Shape s = Shape::MakeBox(Vec3(1, 2, 3));
        const MassProperties mp = ComputeMassPropertiesFromMass(s, real(20));
        REQUIRE(mp.mass == Approx(real(20)).margin(kTol));

        // 惯量与质量成正比：同一形状、两倍质量 -> 惯量也两倍
        const MassProperties doubled = mp.WithMass(real(40));
        REQUIRE(doubled.inertia.rows[0].x ==
                Approx(mp.inertia.rows[0].x * real(2)).epsilon(real(1e-5)));
    }

    SECTION("退化输入返回零而不是 NaN") {
        const MassProperties zeroDensity =
            ComputeMassProperties(Shape::MakeSphere(real(1)), real(0));
        REQUIRE(zeroDensity.mass == Approx(real(0)));
        REQUIRE(zeroDensity.InverseMass() == Approx(real(0)));

        const MassProperties zeroRadius =
            ComputeMassProperties(Shape::MakeSphere(real(0)), real(1000));
        REQUIRE(zeroRadius.mass == Approx(real(0)));
        REQUIRE(IsFinite(zeroRadius.inertia.rows[0].x));
    }
}

TEST_CASE("动力学 惯量张量的解析值", "[dynamics]") {
    SECTION("实心球 I = 2/5 m r^2，三轴相同") {
        const real r = real(2);
        const Shape s = Shape::MakeSphere(r);
        const MassProperties mp = ComputeMassProperties(s, real(1000));
        const real expected = (real(2) / real(5)) * mp.mass * r * r;

        REQUIRE(mp.inertia.rows[0].x == Approx(expected).epsilon(real(1e-5)));
        REQUIRE(mp.inertia.rows[1].y == Approx(expected).epsilon(real(1e-5)));
        REQUIRE(mp.inertia.rows[2].z == Approx(expected).epsilon(real(1e-5)));
    }

    SECTION("长方体：每根轴的惯量由另外两根轴的尺寸决定") {
        // 这是最容易把下标写反的公式。用一个三边完全不同的盒子，
        // 写反了必然对不上。
        const Vec3 h(1, 2, 4);
        const Shape s = Shape::MakeBox(h);
        const MassProperties mp = ComputeMassProperties(s, real(1000));
        const real k = mp.mass / real(3);

        REQUIRE(mp.inertia.rows[0].x ==
                Approx(k * (h.y * h.y + h.z * h.z)).epsilon(real(1e-5)));
        REQUIRE(mp.inertia.rows[1].y ==
                Approx(k * (h.x * h.x + h.z * h.z)).epsilon(real(1e-5)));
        REQUIRE(mp.inertia.rows[2].z ==
                Approx(k * (h.x * h.x + h.y * h.y)).epsilon(real(1e-5)));

        // 最长的那根轴（z）绕自己转最容易 -> I_zz 最小
        REQUIRE(mp.inertia.rows[2].z < mp.inertia.rows[1].y);
        REQUIRE(mp.inertia.rows[1].y < mp.inertia.rows[0].x);
    }

    SECTION("惯量积全为零（形状关于三个坐标平面对称）") {
        const Shape shapes[3] = {Shape::MakeSphere(real(1)),
                                 Shape::MakeCapsule(real(0.5), real(1)),
                                 Shape::MakeBox(Vec3(1, 2, 3))};
        for (const Shape& s : shapes) {
            const MassProperties mp = ComputeMassProperties(s, real(1000));
            REQUIRE(mp.inertia.rows[0].y == Approx(real(0)).margin(kTol));
            REQUIRE(mp.inertia.rows[0].z == Approx(real(0)).margin(kTol));
            REQUIRE(mp.inertia.rows[1].z == Approx(real(0)).margin(kTol));
        }
    }

    SECTION("halfHeight=0 的胶囊 == 同半径的球") {
        // 交叉验证：胶囊的分段积分公式在退化时必须落回球的公式
        const real r = real(1.5);
        const MassProperties cap =
            ComputeMassProperties(Shape::MakeCapsule(r, real(0)), real(1000));
        const MassProperties sph =
            ComputeMassProperties(Shape::MakeSphere(r), real(1000));

        REQUIRE(cap.mass == Approx(sph.mass).epsilon(real(1e-5)));
        REQUIRE(cap.inertia.rows[0].x == Approx(sph.inertia.rows[0].x).epsilon(real(1e-4)));
        REQUIRE(cap.inertia.rows[1].y == Approx(sph.inertia.rows[1].y).epsilon(real(1e-4)));
    }

    SECTION("胶囊：轴向惯量小于横向惯量") {
        // 细长的胶囊绕自身轴线转最容易，这是定性的完备性检查
        const MassProperties mp =
            ComputeMassProperties(Shape::MakeCapsule(real(0.3), real(2)), real(1000));
        REQUIRE(mp.inertia.rows[1].y < mp.inertia.rows[0].x);
        // 横向的两根轴（X、Z）由对称性必须相等
        REQUIRE(mp.inertia.rows[0].x == Approx(mp.inertia.rows[2].z).epsilon(real(1e-5)));
    }
}

TEST_CASE("动力学 惯量张量与数值积分对拍", "[dynamics]") {
    // 独立判据：蒙特卡洛积分直接按定义把张量加出来，
    // 和解析公式之间没有任何共享代码。
    //
    // 收敛速度 O(1/sqrt(N))，40 万个采样点对应百分之一量级的误差，
    // 足以抓住任何系数或下标写错（那些偏差都在几十个百分点以上）。
    constexpr int kSamples = 400000;
    constexpr real kMaxRelError = real(0.02);

    struct Case {
        const char* name;
        Shape shape;
    };
    const Case cases[] = {
        {"球", Shape::MakeSphere(real(1.3))},
        {"立方体", Shape::MakeCube(real(1))},
        {"细长盒", Shape::MakeBox(Vec3(real(0.4), real(2), real(0.7)))},
        {"胶囊", Shape::MakeCapsule(real(0.5), real(1.2))},
        {"胖胶囊", Shape::MakeCapsule(real(1), real(0.3))},
    };

    for (const Case& c : cases) {
        const MassProperties analytic = ComputeMassProperties(c.shape, real(1000));

        real mcMass = real(0);
        const Mat3 mc =
            MonteCarloInertia(c.shape, real(1000), kSamples, 0xABCDEFu, mcMass);

        INFO(c.name);
        REQUIRE(RelativeError(mcMass, analytic.mass) < kMaxRelError);
        REQUIRE(RelativeError(mc.rows[0].x, analytic.inertia.rows[0].x) < kMaxRelError);
        REQUIRE(RelativeError(mc.rows[1].y, analytic.inertia.rows[1].y) < kMaxRelError);
        REQUIRE(RelativeError(mc.rows[2].z, analytic.inertia.rows[2].z) < kMaxRelError);
    }
}

TEST_CASE("动力学 平行轴定理", "[dynamics]") {
    const Shape s = Shape::MakeCube(real(1));
    const MassProperties mp = ComputeMassProperties(s, real(1000));

    SECTION("沿平移方向那根轴的惯量不变") {
        // 这是三维平行轴定理与标量版本最大的区别，也是最容易写错的地方：
        // 沿 X 平移之后，绕 X 轴的转动惯量**不该**增加 ——
        // 质量分布相对 X 轴没有任何变化。
        const Vec3 offset(3, 0, 0);
        const Mat3 moved = TranslateInertia(mp.inertia, mp.mass, offset);

        REQUIRE(moved.rows[0].x == Approx(mp.inertia.rows[0].x).epsilon(real(1e-5)));
        // 另外两根轴按 m*d^2 增加
        const real expected = mp.inertia.rows[1].y + mp.mass * real(9);
        REQUIRE(moved.rows[1].y == Approx(expected).epsilon(real(1e-5)));
        REQUIRE(moved.rows[2].z == Approx(expected).epsilon(real(1e-5)));
    }

    SECTION("零偏移是恒等变换") {
        const Mat3 same = TranslateInertia(mp.inertia, mp.mass, Vec3::Zero());
        REQUIRE(same.rows[0].x == Approx(mp.inertia.rows[0].x));
        REQUIRE(same.rows[1].y == Approx(mp.inertia.rows[1].y));
    }
}

TEST_CASE("动力学 惯量张量的旋转", "[dynamics]") {
    SECTION("球的惯量张量旋转之后不变（各向同性）") {
        const MassProperties mp =
            ComputeMassProperties(Shape::MakeSphere(real(1)), real(1000));
        const Mat3 r =
            Quat::FromAxisAngle(Vec3(1, 2, 3).Normalized(), real(1.1)).ToMat3();
        const Mat3 rotated = RotateInertia(mp.inertia, r);

        REQUIRE(rotated.rows[0].x == Approx(mp.inertia.rows[0].x).epsilon(real(1e-4)));
        REQUIRE(rotated.rows[0].y == Approx(real(0)).margin(real(1e-3)));
    }

    SECTION("绕 Z 转 90 度会交换 X、Y 两轴的惯量") {
        const MassProperties mp =
            ComputeMassProperties(Shape::MakeBox(Vec3(1, 3, 5)), real(1000));
        const Mat3 r = Quat::FromAxisAngle(Vec3(0, 0, 1), kHalfPi).ToMat3();
        const Mat3 rotated = RotateInertia(mp.inertia, r);

        REQUIRE(rotated.rows[0].x == Approx(mp.inertia.rows[1].y).epsilon(real(1e-4)));
        REQUIRE(rotated.rows[1].y == Approx(mp.inertia.rows[0].x).epsilon(real(1e-4)));
        REQUIRE(rotated.rows[2].z == Approx(mp.inertia.rows[2].z).epsilon(real(1e-4)));
    }

    SECTION("旋转后仍然对称（惯量张量的基本性质）") {
        const MassProperties mp =
            ComputeMassProperties(Shape::MakeBox(Vec3(1, 2, 4)), real(1000));
        const Mat3 r =
            Quat::FromAxisAngle(Vec3(3, -1, 2).Normalized(), real(0.7)).ToMat3();
        const Mat3 rotated = RotateInertia(mp.inertia, r);

        REQUIRE(rotated.rows[0].y == Approx(rotated.rows[1].x).epsilon(real(1e-4)));
        REQUIRE(rotated.rows[0].z == Approx(rotated.rows[2].x).epsilon(real(1e-4)));
        REQUIRE(rotated.rows[1].z == Approx(rotated.rows[2].y).epsilon(real(1e-4)));
    }
}

//==============================================================================
// B. 刚体基础
//==============================================================================

TEST_CASE("动力学 刚体的三种类型", "[dynamics]") {
    SECTION("静态与运动学体的 invMass 都是 0") {
        const RigidBody s = RigidBody::Make(BodyType::Static);
        const RigidBody k = RigidBody::Make(BodyType::Kinematic);
        REQUIRE(s.invMass == real(0));
        REQUIRE(k.invMass == real(0));
        REQUIRE_FALSE(s.IsMovableByImpulse());
        REQUIRE_FALSE(k.IsMovableByImpulse());
    }

    SECTION("invMass=0 让冲量自动失效，不需要任何特判") {
        RigidBody s = RigidBody::Make(BodyType::Static);
        s.ApplyImpulseAtPoint(Vec3(1000, 0, 0), Vec3(0, 1, 0));
        REQUIRE(Eq(s.linearVelocity, Vec3::Zero()));
        REQUIRE(Eq(s.angularVelocity, Vec3::Zero()));
    }

    SECTION("动态体会被冲量推动") {
        RigidBody d = RigidBody::Make(BodyType::Dynamic);
        d.invMass = real(0.5);  // 质量 2
        d.ApplyImpulse(Vec3(10, 0, 0));
        REQUIRE(Eq(d.linearVelocity, Vec3(5, 0, 0)));
    }

    SECTION("偏离质心的冲量同时产生角速度") {
        RigidBody d = RigidBody::Make(BodyType::Dynamic);
        d.position = Vec3::Zero();
        // r = (0,1,0)，冲量 (1,0,0) -> 角冲量 r x J = (0,1,0)x(1,0,0) = (0,0,-1)
        d.ApplyImpulseAtPoint(Vec3(1, 0, 0), Vec3(0, 1, 0));
        REQUIRE(Eq(d.linearVelocity, Vec3(1, 0, 0)));
        REQUIRE(Eq(d.angularVelocity, Vec3(0, 0, -1)));
    }

    SECTION("接触点速度 = v + w x r") {
        RigidBody d = RigidBody::Make(BodyType::Dynamic);
        d.position = Vec3::Zero();
        d.linearVelocity = Vec3(1, 0, 0);
        d.angularVelocity = Vec3(0, 0, 2);  // 绕 Z 逆时针
        // 点 (0,1,0)：w x r = (0,0,2)x(0,1,0) = (-2,0,0)
        REQUIRE(Eq(d.VelocityAtPoint(Vec3(0, 1, 0)), Vec3(-1, 0, 0)));
    }

    SECTION("MakeRigidBody 由形状算出 invMass 与 invInertia") {
        BodyDesc desc;
        desc.position = Vec3(1, 2, 3);
        desc.mass = real(10);
        const MassProperties mp =
            ComputeMassPropertiesFromMass(Shape::MakeCube(real(1)), real(10));
        const RigidBody b = MakeRigidBody(desc, mp);

        REQUIRE(b.invMass == Approx(real(0.1)).epsilon(real(1e-5)));
        REQUIRE(Eq(b.position, Vec3(1, 2, 3)));
        // 立方体：I = 1/3 * m * (h^2+h^2) = 2/3*10*1 = 6.667
        REQUIRE(b.invInertiaLocal.rows[0].x ==
                Approx(real(1) / (real(20) / real(3))).epsilon(real(1e-4)));
    }
}

//==============================================================================
// C. 积分器 —— 对着解析解验
//==============================================================================

TEST_CASE("动力学 自由落体对解析解", "[dynamics]") {
    // 解析解：v = g*t，x = 0.5*g*t^2
    //
    // 半隐式欧拉的位置有系统性偏差（每步多走 0.5*g*dt^2），累计到时刻 T 是
    // 0.5*g*dt*T —— 与 dt **一次方**成正比。所以步长减半，误差也该减半。
    // 下面第二个 SECTION 就是在验这个收敛阶：验的是"误差怎么随 dt 变"，
    // 而不是"误差有多大"，这样才能真正确认积分器的阶数没写错。
    const Vec3 g(0, real(-10), 0);
    const real duration = real(1);

    SECTION("速度是精确的") {
        RigidBody b = RigidBody::Make();
        b.linearDamping = real(0);
        b.useGravity = true;

        const real dt = real(1) / real(60);
        const int steps = static_cast<int>(duration / dt);
        for (int i = 0; i < steps; ++i) {
            IntegrateVelocity(b, g, dt);
            IntegratePosition(b, dt);
        }

        // 速度的积分对常加速度是精确的（没有累积误差）
        REQUIRE(b.linearVelocity.y == Approx(real(-10) * real(steps) * dt).epsilon(real(1e-4)));
    }

    SECTION("位置误差随步长线性收敛（确认是一阶积分器）") {
        const real dts[3] = {real(1) / real(60), real(1) / real(120), real(1) / real(240)};
        real errors[3];

        for (int k = 0; k < 3; ++k) {
            RigidBody b = RigidBody::Make();
            b.linearDamping = real(0);

            const real dt = dts[k];
            const int steps = static_cast<int>(duration / dt);
            for (int i = 0; i < steps; ++i) {
                IntegrateVelocity(b, g, dt);
                IntegratePosition(b, dt);
            }

            const real t = real(steps) * dt;
            const real exact = real(0.5) * g.y * t * t;
            errors[k] = Abs(b.position.y - exact);
        }

        INFO("误差 " << errors[0] << " -> " << errors[1] << " -> " << errors[2]);
        // 步长减半，误差应该也大致减半
        REQUIRE(errors[1] < errors[0] * real(0.6));
        REQUIRE(errors[2] < errors[1] * real(0.6));
        // 1/60 步长下一秒的绝对误差也就几厘米
        REQUIRE(errors[0] < real(0.1));
    }

    SECTION("useGravity 关掉之后不下落") {
        RigidBody b = RigidBody::Make();
        b.useGravity = false;
        for (int i = 0; i < 60; ++i) IntegrateVelocity(b, g, real(1) / real(60));
        REQUIRE(Eq(b.linearVelocity, Vec3::Zero()));
    }

    SECTION("静态与运动学体不受重力影响") {
        RigidBody s = RigidBody::Make(BodyType::Static);
        RigidBody k = RigidBody::Make(BodyType::Kinematic);
        for (int i = 0; i < 60; ++i) {
            IntegrateVelocity(s, g, real(1) / real(60));
            IntegrateVelocity(k, g, real(1) / real(60));
        }
        REQUIRE(Eq(s.linearVelocity, Vec3::Zero()));
        REQUIRE(Eq(k.linearVelocity, Vec3::Zero()));
    }

    SECTION("运动学体会被积分位置，静态体不会") {
        RigidBody s = RigidBody::Make(BodyType::Static);
        RigidBody k = RigidBody::Make(BodyType::Kinematic);
        s.linearVelocity = Vec3(1, 0, 0);
        k.linearVelocity = Vec3(1, 0, 0);

        for (int i = 0; i < 60; ++i) {
            IntegratePosition(s, real(1) / real(60));
            IntegratePosition(k, real(1) / real(60));
        }
        REQUIRE(Eq(s.position, Vec3::Zero()));
        REQUIRE(k.position.x == Approx(real(1)).epsilon(real(1e-3)));
    }
}

TEST_CASE("动力学 无外力自由旋转时角动量守恒", "[dynamics]") {
    // 这一条专门盯着**陀螺项**。少了 `-w x (I*w)`，角动量 L = I_world * w
    // 会随着姿态改变而漂移 —— 而它在物理上必须是常量。
    //
    // 用一个三边都不同的盒子（惯量张量三个特征值都不同），
    // 并且让初始角速度不与任何一根主轴对齐 —— 这是唯一能让陀螺项真正起作用的构型。
    // 对称物体（球、立方体）或者绕主轴旋转时陀螺项恒为零，测不出任何东西。
    const MassProperties mp =
        ComputeMassPropertiesFromMass(Shape::MakeBox(Vec3(real(0.5), 1, 2)), real(10));

    RigidBody b = RigidBody::Make();
    b.invMass = mp.InverseMass();
    b.invInertiaLocal = mp.InverseInertia();
    b.angularDamping = real(0);
    b.linearDamping = real(0);
    b.useGravity = false;
    b.angularVelocity = Vec3(real(1.3), real(2.1), real(0.7));
    UpdateWorldInertia(b);

    const auto angularMomentum = [](const RigidBody& body) {
        return body.invInertiaWorld.Inverse() * body.angularVelocity;
    };

    const Vec3 L0 = angularMomentum(b);
    const real L0len = L0.Length();
    REQUIRE(L0len > real(1));  // 确认这个构型确实有可观的角动量

    const real dt = real(1) / real(240);  // 陀螺项对步长敏感，用细一点的步长
    for (int i = 0; i < 240 * 4; ++i) {   // 模拟 4 秒
        IntegrateVelocity(b, Vec3::Zero(), dt);
        IntegratePosition(b, dt);
    }

    const Vec3 L1 = angularMomentum(b);

    INFO("L0 = " << L0.x << ", " << L0.y << ", " << L0.z);
    INFO("L1 = " << L1.x << ", " << L1.y << ", " << L1.z);

    // 一阶显式积分不可能精确守恒，但 4 秒之后漂移应该在百分之几以内。
    // 去掉陀螺项的话这个误差会是几十个百分点。
    REQUIRE(RelativeError(L1.Length(), L0len) < real(0.05));
    REQUIRE(Dot(L0.Normalized(), L1.Normalized()) > real(0.98));
}

TEST_CASE("动力学 绕主轴的自由旋转保持轴向不变", "[dynamics]") {
    // 角速度沿一根主轴时，陀螺项 w x (I*w) 恒为零（I*w 与 w 共线），
    // 所以角速度必须一直不变。这是陀螺项**不该**起作用的对照组：
    // 如果实现里符号写反或者少了投影，这条会立刻失败。
    const MassProperties mp =
        ComputeMassPropertiesFromMass(Shape::MakeBox(Vec3(real(0.5), 1, 2)), real(10));

    RigidBody b = RigidBody::Make();
    b.invMass = mp.InverseMass();
    b.invInertiaLocal = mp.InverseInertia();
    b.angularDamping = real(0);
    b.useGravity = false;
    b.angularVelocity = Vec3(0, 3, 0);  // 沿局部 Y 主轴
    UpdateWorldInertia(b);

    const real dt = real(1) / real(120);
    for (int i = 0; i < 120 * 2; ++i) {
        IntegrateVelocity(b, Vec3::Zero(), dt);
        IntegratePosition(b, dt);
    }

    REQUIRE(Eq(b.angularVelocity, Vec3(0, 3, 0), real(1e-3)));
}

TEST_CASE("动力学 半隐式欧拉的能量有界", "[dynamics]") {
    // 这条用例是整个积分器选型的理由所在。
    //
    // 简谐振子（弹簧）在显式欧拉下能量**单调增长**，几百步就发散；
    // 半隐式欧拉是辛积分器，能量在真值附近有界振荡。
    // 这里手工用两种方式各积一遍同一个弹簧，对比能量的走向。
    const real k = real(50);  // 弹簧刚度
    const real m = real(1);
    const real dt = real(1) / real(60);
    const int steps = 6000;  // 100 秒

    // 显式欧拉：先用旧速度更新位置
    real xExplicit = real(1), vExplicit = real(0);
    // 半隐式：先更新速度，再用新速度更新位置
    real xSymplectic = real(1), vSymplectic = real(0);

    const auto energy = [&](real x, real v) {
        return real(0.5) * m * v * v + real(0.5) * k * x * x;
    };
    const real e0 = energy(xExplicit, vExplicit);

    real maxSymplecticEnergy = e0;
    for (int i = 0; i < steps; ++i) {
        // 显式
        const real aExp = -k * xExplicit / m;
        xExplicit += vExplicit * dt;
        vExplicit += aExp * dt;

        // 半隐式（引擎用的就是这个顺序）
        const real aSym = -k * xSymplectic / m;
        vSymplectic += aSym * dt;
        xSymplectic += vSymplectic * dt;

        maxSymplecticEnergy = Max(maxSymplecticEnergy, energy(xSymplectic, vSymplectic));
    }

    const real eExplicit = energy(xExplicit, vExplicit);

    INFO("初始能量 " << e0 << "，显式欧拉 100 秒后 " << eExplicit
                    << "，半隐式峰值 " << maxSymplecticEnergy);

    // 显式欧拉的能量爆炸式增长
    REQUIRE(eExplicit > e0 * real(10));
    // 半隐式的能量始终被限制在初值附近几个百分点内
    REQUIRE(maxSymplecticEnergy < e0 * real(1.1));
}

TEST_CASE("动力学 阻尼是指数衰减且与步长无关", "[dynamics]") {
    // 用 pow(1-d, dt) 而不是 (1 - d*dt)：同样的阻尼系数在不同帧率下
    // 必须给出同样的衰减。写成线性形式的话，30fps 和 144fps 的手感会差一大截。
    const real duration = real(2);
    real finalSpeeds[3];

    // 步数写死、dt 由它反推，而不是 `steps = duration / dt`——
    // 后者会踩进浮点截断：1/30 存成 float 之后 2/(1/30) = 59.99999，
    // 转成 int 得到 59 步，三种步长覆盖的总时长就对不齐了，
    // 于是一个正确的积分器也会被判成"与步长有关"。
    const int stepCounts[3] = {60, 120, 480};

    for (int k = 0; k < 3; ++k) {
        RigidBody b = RigidBody::Make();
        b.useGravity = false;
        b.linearDamping = real(0.5);
        b.linearVelocity = Vec3(10, 0, 0);

        const real dt = duration / real(stepCounts[k]);
        for (int i = 0; i < stepCounts[k]; ++i) IntegrateVelocity(b, Vec3::Zero(), dt);
        finalSpeeds[k] = b.linearVelocity.x;
    }

    INFO(finalSpeeds[0] << " / " << finalSpeeds[1] << " / " << finalSpeeds[2]);
    // 三种步长的结果必须几乎一致
    REQUIRE(finalSpeeds[1] == Approx(finalSpeeds[0]).epsilon(real(1e-4)));
    REQUIRE(finalSpeeds[2] == Approx(finalSpeeds[0]).epsilon(real(1e-4)));
    // 解析解：v = 10 * (1-0.5)^2 = 2.5
    REQUIRE(finalSpeeds[0] == Approx(real(2.5)).epsilon(real(1e-3)));
}

TEST_CASE("动力学 外力累积与清零", "[dynamics]") {
    RigidBody b = RigidBody::Make();
    b.useGravity = false;
    b.linearDamping = real(0);

    SECTION("力在同一帧内累加") {
        b.ApplyForce(Vec3(1, 0, 0));
        b.ApplyForce(Vec3(2, 0, 0));
        REQUIRE(Eq(b.force, Vec3(3, 0, 0)));
    }

    SECTION("不清零的话力会无限累积") {
        // 这条用例是为了把"必须每帧清零"这个约定钉死
        for (int i = 0; i < 10; ++i) b.ApplyForce(Vec3(1, 0, 0));
        REQUIRE(b.force.x == Approx(real(10)));
        ClearForces(b);
        REQUIRE(Eq(b.force, Vec3::Zero()));
        REQUIRE(Eq(b.torque, Vec3::Zero()));
    }

    SECTION("偏离质心的力产生力矩") {
        b.ApplyForceAtPoint(Vec3(0, 1, 0), Vec3(1, 0, 0));
        REQUIRE(Eq(b.force, Vec3(0, 1, 0)));
        // r x F = (1,0,0) x (0,1,0) = (0,0,1)
        REQUIRE(Eq(b.torque, Vec3(0, 0, 1)));
    }

    SECTION("恒力产生匀加速") {
        b.invMass = real(0.5);  // 质量 2
        b.ApplyForce(Vec3(4, 0, 0));  // a = F/m = 2
        IntegrateVelocity(b, Vec3::Zero(), real(1));
        REQUIRE(b.linearVelocity.x == Approx(real(2)).epsilon(real(1e-5)));
    }
}

//==============================================================================
// D. 世界惯量与休眠
//==============================================================================

TEST_CASE("动力学 世界惯量随姿态刷新", "[dynamics]") {
    const MassProperties mp =
        ComputeMassPropertiesFromMass(Shape::MakeBox(Vec3(1, 3, 5)), real(10));

    RigidBody b = RigidBody::Make();
    b.invInertiaLocal = mp.InverseInertia();

    SECTION("单位姿态下世界惯量等于局部惯量") {
        UpdateWorldInertia(b);
        REQUIRE(b.invInertiaWorld.rows[0].x ==
                Approx(b.invInertiaLocal.rows[0].x).epsilon(real(1e-5)));
    }

    SECTION("绕 Z 转 90 度会交换 X、Y") {
        b.rotation = Quat::FromAxisAngle(Vec3(0, 0, 1), kHalfPi);
        UpdateWorldInertia(b);
        REQUIRE(b.invInertiaWorld.rows[0].x ==
                Approx(b.invInertiaLocal.rows[1].y).epsilon(real(1e-4)));
    }

    SECTION("IntegratePosition 会自动刷新世界惯量") {
        b.angularVelocity = Vec3(0, 0, kHalfPi);  // 1 秒转 90 度
        UpdateWorldInertia(b);
        const real before = b.invInertiaWorld.rows[0].x;

        const real dt = real(1) / real(240);
        for (int i = 0; i < 240; ++i) IntegratePosition(b, dt);

        REQUIRE(b.invInertiaWorld.rows[0].x !=
                Approx(before).epsilon(real(1e-3)));
        REQUIRE(b.invInertiaWorld.rows[0].x ==
                Approx(b.invInertiaLocal.rows[1].y).epsilon(real(1e-2)));
    }
}

TEST_CASE("动力学 休眠", "[dynamics]") {
    SleepConfig config;
    RigidBody b = RigidBody::Make();
    b.useGravity = false;

    SECTION("持续低速才会睡着") {
        b.linearVelocity = Vec3(real(0.01), 0, 0);
        const real dt = real(1) / real(60);

        int stepsUntilSleep = 0;
        for (int i = 0; i < 120; ++i) {
            if (UpdateSleep(b, dt, config)) {
                stepsUntilSleep = i + 1;
                break;
            }
        }
        REQUIRE(b.isSleeping);
        // 0.5 秒 = 30 步（第 30 步累计到 0.5）
        REQUIRE(stepsUntilSleep == 30);
        // 睡着时速度被清零，免得醒来的瞬间窜一下
        REQUIRE(Eq(b.linearVelocity, Vec3::Zero()));
    }

    SECTION("速度超标会把计时器清零") {
        const real dt = real(1) / real(60);
        b.linearVelocity = Vec3(real(0.01), 0, 0);
        for (int i = 0; i < 20; ++i) UpdateSleep(b, dt, config);
        REQUIRE_FALSE(b.isSleeping);

        // 中途动一下 -> 重新计时
        b.linearVelocity = Vec3(5, 0, 0);
        UpdateSleep(b, dt, config);
        REQUIRE(b.sleepTimer == Approx(real(0)));

        b.linearVelocity = Vec3(real(0.01), 0, 0);
        for (int i = 0; i < 20; ++i) UpdateSleep(b, dt, config);
        REQUIRE_FALSE(b.isSleeping);  // 只攒了 20 步，还不够 30
    }

    SECTION("只有角速度超标也不会睡") {
        b.linearVelocity = Vec3::Zero();
        b.angularVelocity = Vec3(0, 5, 0);
        for (int i = 0; i < 120; ++i) UpdateSleep(b, real(1) / real(60), config);
        REQUIRE_FALSE(b.isSleeping);
    }

    SECTION("睡着的物体不被积分") {
        b.ForceSleep();
        b.linearVelocity = Vec3(10, 0, 0);  // 强行塞一个速度进去
        IntegrateVelocity(b, Vec3(0, -10, 0), real(1));
        IntegratePosition(b, real(1));
        REQUIRE(Eq(b.position, Vec3::Zero()));
        REQUIRE(b.linearVelocity.y == Approx(real(0)));
    }

    SECTION("唤醒会清空计时器") {
        b.linearVelocity = Vec3(real(0.01), 0, 0);
        for (int i = 0; i < 20; ++i) UpdateSleep(b, real(1) / real(60), config);
        REQUIRE(b.sleepTimer > real(0));
        b.WakeUp();
        REQUIRE(b.sleepTimer == Approx(real(0)));
        REQUIRE_FALSE(b.isSleeping);
    }
}

//==============================================================================
// E. 材质与渲染插值
//==============================================================================

TEST_CASE("动力学 材质组合规则", "[dynamics]") {
    SECTION("摩擦取几何平均：任何一方为 0，结果就是 0") {
        const Material ice(real(0), real(0));
        const Material rubber(real(0.9), real(0.7));
        REQUIRE(CombineFriction(ice, rubber) == Approx(real(0)).margin(real(1e-6)));

        const Material a(real(0.4), real(0));
        const Material b(real(0.9), real(0));
        REQUIRE(CombineFriction(a, b) ==
                Approx(Sqrt(real(0.36))).epsilon(real(1e-5)));
    }

    SECTION("恢复取最大值：弹力球砸到不弹的地面照样弹") {
        const Material ground(real(0.8), real(0));
        const Material bouncy(real(0.8), real(0.9));
        REQUIRE(CombineRestitution(ground, bouncy) == Approx(real(0.9)));
    }

    SECTION("组合与顺序无关") {
        const Material a = Material::Wood();
        const Material b = Material::Metal();
        REQUIRE(CombineFriction(a, b) == Approx(CombineFriction(b, a)));
        REQUIRE(CombineRestitution(a, b) == Approx(CombineRestitution(b, a)));
    }
}

TEST_CASE("动力学 渲染插值", "[dynamics]") {
    const Transform t0(Vec3(0, 0, 0), Quat::Identity());
    const Transform t1(Vec3(10, 0, 0), Quat::FromAxisAngle(Vec3(0, 1, 0), kHalfPi));

    SECTION("两端取到精确值") {
        REQUIRE(Eq(InterpolateTransform(t0, t1, real(0)).position, t0.position));
        REQUIRE(Eq(InterpolateTransform(t0, t1, real(1)).position, t1.position));
    }

    SECTION("中点") {
        const Transform mid = InterpolateTransform(t0, t1, real(0.5));
        REQUIRE(Eq(mid.position, Vec3(5, 0, 0)));
        // 姿态走 slerp，中点应该是转 45 度
        const Vec3 axis = mid.rotation.Rotate(Vec3(1, 0, 0));
        REQUIRE(axis.x == Approx(Sqrt(real(2)) / real(2)).epsilon(real(1e-3)));
    }

    SECTION("alpha 超出 [0,1] 会被夹住") {
        REQUIRE(Eq(InterpolateTransform(t0, t1, real(-5)).position, t0.position));
        REQUIRE(Eq(InterpolateTransform(t0, t1, real(5)).position, t1.position));
    }
}
