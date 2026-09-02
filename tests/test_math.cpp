//==============================================================================
// tests/test_math.cpp
//
// 数学层测试。
//
// 这一层被后面所有模块复用，一个符号错误（比如叉积方向反了、四元数复合顺序
// 反了）会以"物体朝奇怪的方向弹开"这种形式在很久以后才暴露出来，极难定位。
// 所以这里除了常规用例，重点测三类东西：
//   1. **约定**：坐标系手性、复合顺序、行优先/列向量 —— 这些是口头约定，
//      只有测试能把它们钉死。
//   2. **恒等式**：M * M^-1 == I、Skew(a)*b == a×b、q 与矩阵表示等价 ——
//      两条独立路径互相对拍，比和硬编码数字比对更能发现问题。
//   3. **退化输入**：零向量归一化、奇异矩阵求逆、180 度旋转的四元数恢复 ——
//      这些是 NaN 的主要来源。
//==============================================================================

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <type_traits>

#include "pe/math/Mat3.h"
#include "pe/math/Mat4.h"
#include "pe/math/MathUtil.h"
#include "pe/math/Quat.h"
#include "pe/math/Transform.h"
#include "pe/math/Vec2.h"
#include "pe/math/Vec3.h"
#include "pe/math/Vec4.h"

using namespace pe;
using Catch::Approx;

namespace {
/// float 精度下，经过若干次乘加之后合理的比较容差。
constexpr real kTol = real(1e-5);

bool ApproxEq(const Vec3& a, const Vec3& b, real tol = kTol) {
    return NearlyEqual(a, b, tol);
}
}  // namespace

//==============================================================================
// MathUtil
//==============================================================================

TEST_CASE("MathUtil 基础函数", "[math][util]") {
    REQUIRE(Abs(real(-3)) == real(3));
    REQUIRE(Clamp(real(5), real(0), real(1)) == real(1));
    REQUIRE(Clamp(real(-5), real(0), real(1)) == real(0));
    REQUIRE(Lerp(real(0), real(10), real(0.25)) == Approx(real(2.5)));
    REQUIRE(Sign(real(0)) == real(0));
    REQUIRE(DegToRad(real(180)) == Approx(kPi));
    REQUIRE(RadToDeg(kPi) == Approx(real(180)));
}

TEST_CASE("SafeAcos 钳制定义域，不产生 NaN", "[math][util]") {
    // 这正是实际会发生的情况：两个单位向量点积因浮点误差略微超出 [-1,1]。
    REQUIRE(SafeAcos(real(1.0000001)) == Approx(real(0)).margin(real(1e-6)));
    REQUIRE(SafeAcos(real(-1.0000001)) == Approx(kPi));
    REQUIRE(IsFinite(SafeAcos(real(2))));
    REQUIRE(IsFinite(SafeAcos(real(-2))));
}

TEST_CASE("SafeDivide 不产生 inf", "[math][util]") {
    REQUIRE(SafeDivide(real(1), real(0)) == real(0));
    REQUIRE(SafeDivide(real(6), real(2)) == Approx(real(3)));
}

//==============================================================================
// Vec3
//==============================================================================

TEST_CASE("Vec3 基本运算", "[math][vec3]") {
    const Vec3 a(1, 2, 3);
    const Vec3 b(4, 5, 6);

    REQUIRE(ApproxEq(a + b, Vec3(5, 7, 9)));
    REQUIRE(ApproxEq(b - a, Vec3(3, 3, 3)));
    REQUIRE(ApproxEq(a * real(2), Vec3(2, 4, 6)));
    REQUIRE(ApproxEq(real(2) * a, Vec3(2, 4, 6)));
    REQUIRE(ApproxEq(-a, Vec3(-1, -2, -3)));

    REQUIRE(a[0] == real(1));
    REQUIRE(a[2] == real(3));

    REQUIRE(Vec3{}.IsZero());  // 值初始化必须清零
}

TEST_CASE("Vec3 点积与叉积", "[math][vec3]") {
    const Vec3 a(1, 2, 3);
    const Vec3 b(4, 5, 6);

    REQUIRE(Dot(a, b) == Approx(real(32)));  // 1*4 + 2*5 + 3*6

    // 右手坐标系的定义性检查：X × Y == Z。
    // 如果这一条挂了，整个引擎的力矩方向都会反。
    REQUIRE(ApproxEq(Cross(Vec3::UnitX(), Vec3::UnitY()), Vec3::UnitZ()));
    REQUIRE(ApproxEq(Cross(Vec3::UnitY(), Vec3::UnitZ()), Vec3::UnitX()));
    REQUIRE(ApproxEq(Cross(Vec3::UnitZ(), Vec3::UnitX()), Vec3::UnitY()));

    // 反交换律
    REQUIRE(ApproxEq(Cross(a, b), -Cross(b, a)));
    // 叉积垂直于两个操作数
    const Vec3 c = Cross(a, b);
    REQUIRE(Dot(c, a) == Approx(real(0)).margin(kTol));
    REQUIRE(Dot(c, b) == Approx(real(0)).margin(kTol));
    // 平行向量叉积为零
    REQUIRE(Cross(a, a * real(3)).IsZero(kTol));
}

TEST_CASE("Vec3 归一化", "[math][vec3]") {
    Vec3 v(3, 4, 0);
    REQUIRE(v.Length() == Approx(real(5)));
    REQUIRE(v.LengthSq() == Approx(real(25)));
    REQUIRE(ApproxEq(v.Normalized(), Vec3(real(0.6), real(0.8), real(0))));

    // Normalize() 返回归一化前的长度
    const real len = v.Normalize();
    REQUIRE(len == Approx(real(5)));
    REQUIRE(v.Length() == Approx(real(1)));
}

TEST_CASE("Vec3 零向量归一化返回零而不是 NaN", "[math][vec3][degenerate]") {
    // 两个物体完全重合时，碰撞法线就是零向量。这里若返回 NaN，
    // NaN 会顺着 位置 -> AABB -> 宽相位 一路扩散，整个世界报废。
    const Vec3 zero = Vec3::Zero();
    REQUIRE(zero.Normalized().IsZero());
    REQUIRE(zero.Normalized().IsFinite());

    Vec3 tiny(real(1e-20), real(1e-20), real(1e-20));
    REQUIRE(tiny.Normalized().IsFinite());
    REQUIRE(tiny.Normalize() == Approx(real(0)).margin(real(1e-6)));
}

TEST_CASE("Vec3 逐分量工具", "[math][vec3]") {
    const Vec3 a(1, -5, 3);
    const Vec3 b(2, 4, -1);

    REQUIRE(ApproxEq(MinPerComponent(a, b), Vec3(1, -5, -1)));
    REQUIRE(ApproxEq(MaxPerComponent(a, b), Vec3(2, 4, 3)));
    REQUIRE(ApproxEq(AbsPerComponent(a), Vec3(1, 5, 3)));
    REQUIRE(MaxAbsComponentIndex(a) == 1);  // |-5| 最大
    REQUIRE(ApproxEq(Lerp(Vec3::Zero(), Vec3(10, 10, 10), real(0.5)), Vec3(5, 5, 5)));
}

TEST_CASE("BuildOrthonormalBasis 对任意法线都不退化", "[math][vec3][degenerate]") {
    // 覆盖各个坐标轴（含 ±Y —— 角色站在平地上时法线就是 +Y，
    // 朴素实现在这里会因为叉积得零向量而崩掉）以及一些斜方向。
    const Vec3 normals[] = {
        Vec3::UnitX(),  -Vec3::UnitX(), Vec3::UnitY(), -Vec3::UnitY(),
        Vec3::UnitZ(),  -Vec3::UnitZ(),
        Vec3(1, 1, 1).Normalized(), Vec3(real(-0.3), real(0.9), real(0.31)).Normalized(),
        Vec3(real(0.577), real(0.577), real(0.577)).Normalized(),
    };

    for (const Vec3& n : normals) {
        Vec3 t1, t2;
        BuildOrthonormalBasis(n, t1, t2);

        INFO("normal = (" << n.x << ", " << n.y << ", " << n.z << ")");
        REQUIRE(t1.Length() == Approx(real(1)).margin(real(1e-4)));
        REQUIRE(t2.Length() == Approx(real(1)).margin(real(1e-4)));
        REQUIRE(Dot(n, t1) == Approx(real(0)).margin(real(1e-5)));
        REQUIRE(Dot(n, t2) == Approx(real(0)).margin(real(1e-5)));
        REQUIRE(Dot(t1, t2) == Approx(real(0)).margin(real(1e-5)));
        // 右手性：n × t1 == t2
        REQUIRE(ApproxEq(Cross(n, t1), t2, real(1e-4)));
    }
}

//==============================================================================
// Vec2 / Vec4
//==============================================================================

TEST_CASE("Vec2 基本运算与 2D 叉积", "[math][vec2]") {
    const Vec2 a(1, 0);
    const Vec2 b(0, 1);

    REQUIRE(Dot(a, b) == Approx(real(0)));
    REQUIRE(Cross(a, b) == Approx(real(1)));   // b 在 a 的逆时针方向
    REQUIRE(Cross(b, a) == Approx(real(-1)));
    REQUIRE(NearlyEqual(Perpendicular(a), b, kTol));
    REQUIRE(Vec2(3, 4).Length() == Approx(real(5)));
    REQUIRE(Vec2::Zero().Normalized().IsZero());
}

TEST_CASE("Vec4 与 Vec3 互转", "[math][vec4]") {
    const Vec4 p(Vec3(1, 2, 3), real(1));
    REQUIRE(ApproxEq(p.XYZ(), Vec3(1, 2, 3)));
    REQUIRE(p.w == real(1));
    REQUIRE(Dot(p, Vec4(1, 1, 1, 1)) == Approx(real(7)));
}

//==============================================================================
// Mat3
//==============================================================================

TEST_CASE("Mat3 构造与访问遵循行优先约定", "[math][mat3][convention]") {
    const Mat3 m(1, 2, 3,
                 4, 5, 6,
                 7, 8, 9);

    // At(行, 列)
    REQUIRE(m.At(0, 0) == real(1));
    REQUIRE(m.At(0, 2) == real(3));
    REQUIRE(m.At(2, 0) == real(7));

    REQUIRE(ApproxEq(m.Row(1), Vec3(4, 5, 6)));
    REQUIRE(ApproxEq(m.Column(1), Vec3(2, 5, 8)));
}

TEST_CASE("Mat3 矩阵乘向量是列向量右乘", "[math][mat3][convention]") {
    // 这个用例把 v' = M * v 的约定钉死。
    // 若哪天有人把存储改成列优先而没改乘法，这里必挂。
    const Mat3 m(1, 2, 3,
                 4, 5, 6,
                 7, 8, 9);
    const Vec3 v(1, 0, 0);

    // M * (1,0,0) 应该取出 M 的第一**列**
    REQUIRE(ApproxEq(m * v, Vec3(1, 4, 7)));
}

TEST_CASE("Mat3 FromColumns 把局部轴变换到世界", "[math][mat3]") {
    const Vec3 ex(0, 1, 0);
    const Vec3 ey(-1, 0, 0);
    const Vec3 ez(0, 0, 1);
    const Mat3 r = Mat3::FromColumns(ex, ey, ez);

    REQUIRE(ApproxEq(r * Vec3::UnitX(), ex));
    REQUIRE(ApproxEq(r * Vec3::UnitY(), ey));
    REQUIRE(ApproxEq(r * Vec3::UnitZ(), ez));
}

TEST_CASE("Mat3 转置与乘法", "[math][mat3]") {
    const Mat3 m(1, 2, 3,
                 4, 5, 6,
                 7, 8, 9);
    const Mat3 t = m.Transposed();

    REQUIRE(t.At(0, 1) == m.At(1, 0));
    REQUIRE(NearlyEqual(t.Transposed(), m, kTol));
    REQUIRE(NearlyEqual(m * Mat3::Identity(), m, kTol));
    REQUIRE(NearlyEqual(Mat3::Identity() * m, m, kTol));
}

TEST_CASE("Mat3 逆矩阵满足 M * M^-1 == I", "[math][mat3]") {
    const Mat3 m(2, -1, 0,
                 1, 3, 2,
                 0, 1, 4);
    const Mat3 inv = m.Inverse();

    REQUIRE(NearlyEqual(m * inv, Mat3::Identity(), real(1e-4)));
    REQUIRE(NearlyEqual(inv * m, Mat3::Identity(), real(1e-4)));

    // 对角阵（惯量张量最常见的形态）的逆就是各元素取倒数
    const Mat3 diag = Mat3::Diagonal(Vec3(2, 4, 8));
    REQUIRE(NearlyEqual(diag.Inverse(),
                        Mat3::Diagonal(Vec3(real(0.5), real(0.25), real(0.125))), kTol));
}

TEST_CASE("Mat3 奇异矩阵求逆返回零矩阵而不是 NaN", "[math][mat3][degenerate]") {
    // 第三行 = 第一行 + 第二行，行列式为 0
    const Mat3 singular(1, 2, 3,
                        4, 5, 6,
                        5, 7, 9);
    REQUIRE(Abs(singular.Determinant()) < real(1e-5));

    const Mat3 inv = singular.Inverse();
    REQUIRE(NearlyEqual(inv, Mat3::Zero(), kTol));
    // 关键：结果里没有 NaN/inf
    REQUIRE(IsFinite(inv.At(0, 0)));
}

TEST_CASE("Mat3 Skew(a) * b == Cross(a, b)", "[math][mat3]") {
    // 反对称矩阵是约束求解推导 K 矩阵的基础工具，这条恒等式必须成立。
    const Vec3 a(1, -2, 3);
    const Vec3 b(4, 5, -6);

    REQUIRE(ApproxEq(Mat3::Skew(a) * b, Cross(a, b)));
    // 反对称性：S^T == -S
    REQUIRE(NearlyEqual(Mat3::Skew(a).Transposed(), Mat3::Skew(a) * real(-1), kTol));
}

TEST_CASE("Mat3 Outer 外积", "[math][mat3]") {
    const Vec3 a(1, 2, 3);
    const Vec3 b(4, 5, 6);
    const Mat3 o = Mat3::Outer(a, b);

    // (a ⊗ b)(i,j) == a[i] * b[j]
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            REQUIRE(o.At(i, j) == Approx(a[i] * b[j]));
        }
    }
}

TEST_CASE("TransformInertia 把惯量张量转到世界空间", "[math][mat3][inertia]") {
    // 局部惯量 diag(1, 2, 3)，物体绕 Z 轴转 90 度。
    // 局部 X 轴转到了世界 Y 轴，所以世界的 Iyy 应该等于局部的 Ixx = 1，
    // 世界的 Ixx 等于局部的 Iyy = 2，Izz 不变。
    const Mat3 inertiaLocal = Mat3::Diagonal(Vec3(1, 2, 3));
    const Mat3 r = Quat::FromAxisAngle(Vec3::UnitZ(), kHalfPi).ToMat3();

    const Mat3 world = TransformInertia(r, inertiaLocal);

    REQUIRE(world.At(0, 0) == Approx(real(2)).margin(real(1e-5)));
    REQUIRE(world.At(1, 1) == Approx(real(1)).margin(real(1e-5)));
    REQUIRE(world.At(2, 2) == Approx(real(3)).margin(real(1e-5)));

    // 惯量张量必须保持对称
    REQUIRE(NearlyEqual(world, world.Transposed(), real(1e-5)));
}

//==============================================================================
// Quat
//==============================================================================

TEST_CASE("Quat 单位四元数不改变向量", "[math][quat]") {
    const Vec3 v(1, 2, 3);
    REQUIRE(ApproxEq(Quat::Identity().Rotate(v), v));
    REQUIRE(NearlyEqual(Quat::Identity().ToMat3(), Mat3::Identity(), kTol));
}

TEST_CASE("Quat 绕轴旋转方向符合右手定则", "[math][quat][convention]") {
    // 绕 +Z 转 +90 度，+X 应该转到 +Y。
    // 这一条钉死了"正角度 = 逆时针（从轴正方向朝原点看）"的约定。
    const Quat q = Quat::FromAxisAngle(Vec3::UnitZ(), kHalfPi);
    REQUIRE(ApproxEq(q.Rotate(Vec3::UnitX()), Vec3::UnitY()));
    REQUIRE(ApproxEq(q.Rotate(Vec3::UnitY()), -Vec3::UnitX()));
    REQUIRE(ApproxEq(q.Rotate(Vec3::UnitZ()), Vec3::UnitZ()));  // 轴自身不变
}

TEST_CASE("Quat Rotate 与 ToMat3 结果一致", "[math][quat]") {
    // 两条独立的实现路径互相对拍：Rotate 用的是展开式，ToMat3 用的是矩阵元素。
    // 任何一边写错，这里都会挂。
    const Quat q = Quat::FromAxisAngle(Vec3(1, 2, 3).Normalized(), real(0.7));
    const Mat3 m = q.ToMat3();

    const Vec3 samples[] = {Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1),
                            Vec3(1, 2, 3), Vec3(-5, real(0.5), 2)};
    for (const Vec3& v : samples) {
        REQUIRE(ApproxEq(q.Rotate(v), m * v, real(1e-4)));
    }
}

TEST_CASE("Quat 旋转矩阵是正交阵", "[math][quat]") {
    const Quat q = Quat::FromAxisAngle(Vec3(real(-1), real(2), real(0.5)).Normalized(),
                                       real(2.1));
    const Mat3 m = q.ToMat3();

    REQUIRE(NearlyEqual(m * m.Transposed(), Mat3::Identity(), real(1e-4)));
    // 行列式为 +1（纯旋转，不含反射）
    REQUIRE(m.Determinant() == Approx(real(1)).margin(real(1e-4)));
}

TEST_CASE("Quat 复合顺序：q1 * q2 表示先做 q2", "[math][quat][convention]") {
    const Quat q1 = Quat::FromAxisAngle(Vec3::UnitZ(), kHalfPi);
    const Quat q2 = Quat::FromAxisAngle(Vec3::UnitX(), kHalfPi);
    const Vec3 v(1, 2, 3);

    REQUIRE(ApproxEq((q1 * q2).Rotate(v), q1.Rotate(q2.Rotate(v)), real(1e-4)));

    // 旋转不可交换 —— 顺序反了结果就不同（这也是四元数乘法不可交换的物理意义）
    REQUIRE_FALSE(ApproxEq((q1 * q2).Rotate(v), (q2 * q1).Rotate(v), real(1e-3)));

    // 矩阵表示下的复合顺序必须一致
    REQUIRE(NearlyEqual((q1 * q2).ToMat3(), q1.ToMat3() * q2.ToMat3(), real(1e-4)));
}

TEST_CASE("Quat 共轭即逆", "[math][quat]") {
    const Quat q = Quat::FromAxisAngle(Vec3(1, 1, 0).Normalized(), real(1.3));
    const Vec3 v(2, -1, 4);

    REQUIRE(ApproxEq(q.Conjugate().Rotate(q.Rotate(v)), v, real(1e-4)));
    REQUIRE(ApproxEq(q.RotateInverse(q.Rotate(v)), v, real(1e-4)));
    REQUIRE(SameRotation(q * q.Conjugate(), Quat::Identity()));
}

TEST_CASE("Quat 与矩阵互转往返一致（含 180 度退化情形）", "[math][quat][degenerate]") {
    // FromMat3 用 Shepperd 方法，就是为了处理接近 180 度时朴素公式除零的问题。
    // 这里专门覆盖绕三个轴各转 180 度，以及若干普通角度。
    const Quat cases[] = {
        Quat::Identity(),
        Quat::FromAxisAngle(Vec3::UnitX(), kPi),
        Quat::FromAxisAngle(Vec3::UnitY(), kPi),
        Quat::FromAxisAngle(Vec3::UnitZ(), kPi),
        Quat::FromAxisAngle(Vec3(1, 1, 1).Normalized(), kPi),
        Quat::FromAxisAngle(Vec3::UnitY(), real(0.001)),
        Quat::FromAxisAngle(Vec3(real(0.3), real(-0.7), real(0.6)).Normalized(), real(2.9)),
    };

    for (const Quat& q : cases) {
        const Quat back = Quat::FromMat3(q.ToMat3());
        INFO("angle = " << q.Angle());
        // 用 SameRotation 而不是逐分量比较：q 和 -q 是同一个旋转，
        // Shepperd 方法可能返回符号相反的那一个，这完全正确。
        REQUIRE(SameRotation(q, back, real(1e-4)));
        REQUIRE(NearlyEqual(back.ToMat3(), q.ToMat3(), real(1e-4)));
    }
}

TEST_CASE("Quat 轴角分解", "[math][quat]") {
    const Vec3 axis = Vec3(real(0.2), real(-0.4), real(0.9)).Normalized();
    const real angle = real(1.234);
    const Quat q = Quat::FromAxisAngle(axis, angle);

    REQUIRE(q.Angle() == Approx(angle).margin(real(1e-4)));
    REQUIRE(ApproxEq(q.Axis(), axis, real(1e-4)));

    // 零旋转时轴无意义，但不能是 NaN
    REQUIRE(Quat::Identity().Angle() == Approx(real(0)).margin(real(1e-5)));
    REQUIRE(Quat::Identity().Axis().IsFinite());
}

TEST_CASE("Quat 归一化处理退化输入", "[math][quat][degenerate]") {
    const Quat zero(0, 0, 0, 0);
    REQUIRE(NearlyEqual(zero.Normalized(), Quat::Identity(), kTol));
    REQUIRE(NearlyEqual(zero.Inverse(), Quat::Identity(), kTol));

    // 非单位四元数归一化后长度为 1
    const Quat scaled = Quat(1, 2, 3, 4);
    REQUIRE(scaled.Normalized().Length() == Approx(real(1)));
}

TEST_CASE("Quat Slerp 端点与匀角速度", "[math][quat][interp]") {
    const Quat a = Quat::Identity();
    const Quat b = Quat::FromAxisAngle(Vec3::UnitY(), kHalfPi);

    REQUIRE(SameRotation(Quat::Slerp(a, b, real(0)), a));
    REQUIRE(SameRotation(Quat::Slerp(a, b, real(1)), b));

    // 匀角速度：t=0.5 处必须恰好是 45 度，t=0.25 处是 22.5 度。
    // 这正是 slerp 相对 nlerp 的价值所在 —— nlerp 在这里会偏离。
    REQUIRE(Quat::Slerp(a, b, real(0.5)).Angle() == Approx(kHalfPi * real(0.5)).margin(real(1e-4)));
    REQUIRE(Quat::Slerp(a, b, real(0.25)).Angle() == Approx(kHalfPi * real(0.25)).margin(real(1e-4)));
}

TEST_CASE("Quat Slerp 走短弧", "[math][quat][interp]") {
    const Quat a = Quat::FromAxisAngle(Vec3::UnitY(), real(0.1));
    // 用 -b 表示同一个旋转，但点积为负。Slerp 必须识别出来并取反，
    // 否则会绕远路转 300 多度（视觉上物体会突然反向猛转一圈）。
    const Quat b = Quat::FromAxisAngle(Vec3::UnitY(), real(0.3));
    const Quat negB = -b;

    const Quat viaB = Quat::Slerp(a, b, real(0.5));
    const Quat viaNegB = Quat::Slerp(a, negB, real(0.5));

    REQUIRE(SameRotation(viaB, viaNegB, real(1e-4)));
}

TEST_CASE("Quat 角速度积分逼近解析解", "[math][quat][integrate]") {
    // 绕 +Y 以 1 rad/s 匀速自转 1 秒，结果必须接近"绕 +Y 转 1 弧度"。
    //
    // 一阶积分每步的角度亏损约为 a^3/12（a 是单步转角），
    // dt = 1/240 时单步亏损约 6e-9，240 步累计约 1.4e-6 —— 远小于容差。
    // 若哪天有人把 0.5 系数写错或漏掉归一化，这个用例会立刻挂。
    const Vec3 omega(0, 1, 0);
    const real dt = real(1) / real(240);

    Quat q = Quat::Identity();
    for (int i = 0; i < 240; ++i) {
        q = Quat::Integrate(q, omega, dt);
    }

    const Quat expected = Quat::FromAxisAngle(Vec3::UnitY(), real(1));
    REQUIRE(SameRotation(q, expected, real(1e-3)));
    REQUIRE(q.Angle() == Approx(real(1)).margin(real(1e-3)));

    // 积分后必须仍是单位四元数（归一化生效）
    REQUIRE(q.Length() == Approx(real(1)).margin(real(1e-5)));
}

TEST_CASE("Quat 零角速度积分不改变姿态", "[math][quat][integrate]") {
    const Quat q0 = Quat::FromAxisAngle(Vec3(1, 1, 1).Normalized(), real(0.6));
    const Quat q1 = Quat::Integrate(q0, Vec3::Zero(), real(0.016));
    REQUIRE(SameRotation(q0, q1, real(1e-6)));
}

//==============================================================================
// Transform
//==============================================================================

TEST_CASE("Transform 点与方向的变换不同", "[math][transform][convention]") {
    // 把方向当点来变换（多加了一次平移）是引擎里最常见的一类 bug，
    // 表现为法线随物体位置漂移。这个用例把两者的区别钉死。
    const Transform t(Vec3(10, 0, 0), Quat::FromAxisAngle(Vec3::UnitZ(), kHalfPi));

    // 点：先转再平移
    REQUIRE(ApproxEq(t.TransformPoint(Vec3::UnitX()), Vec3(10, 1, 0), real(1e-4)));
    // 方向：只转，不平移
    REQUIRE(ApproxEq(t.TransformDirection(Vec3::UnitX()), Vec3(0, 1, 0), real(1e-4)));
}

TEST_CASE("Transform 正逆变换往返", "[math][transform]") {
    const Transform t(Vec3(3, -2, 7),
                      Quat::FromAxisAngle(Vec3(real(0.3), real(1), real(-0.2)).Normalized(),
                                          real(1.1)));
    const Vec3 p(5, 6, -7);

    REQUIRE(ApproxEq(t.InverseTransformPoint(t.TransformPoint(p)), p, real(1e-4)));
    REQUIRE(ApproxEq(t.Inverse().TransformPoint(t.TransformPoint(p)), p, real(1e-4)));
    REQUIRE(ApproxEq(t.InverseTransformDirection(t.TransformDirection(p)), p, real(1e-4)));
}

TEST_CASE("Transform 复合等价于依次施加", "[math][transform][convention]") {
    // (parent * child).TransformPoint(p) 必须等于 parent(child(p))，
    // 也就是"先做 child 再做 parent"。碰撞体的局部偏移就靠这条语义。
    const Transform parent(Vec3(1, 2, 3), Quat::FromAxisAngle(Vec3::UnitY(), real(0.4)));
    const Transform child(Vec3(-1, 0, 2), Quat::FromAxisAngle(Vec3::UnitX(), real(0.9)));
    const Vec3 p(4, -5, 6);

    REQUIRE(ApproxEq((parent * child).TransformPoint(p),
                     parent.TransformPoint(child.TransformPoint(p)), real(1e-4)));

    // 结合律
    const Transform g(Vec3(0, 7, 0), Quat::FromAxisAngle(Vec3::UnitZ(), real(-0.5)));
    REQUIRE(ApproxEq(((g * parent) * child).TransformPoint(p),
                     (g * (parent * child)).TransformPoint(p), real(1e-4)));
}

TEST_CASE("Transform 与自身的逆复合得到单位变换", "[math][transform]") {
    const Transform t(Vec3(-4, 8, 1), Quat::FromAxisAngle(Vec3::UnitX(), real(2.0)));
    const Transform id = t * t.Inverse();

    REQUIRE(id.position.IsZero(real(1e-4)));
    REQUIRE(SameRotation(id.rotation, Quat::Identity(), real(1e-4)));
}

TEST_CASE("Transform 插值端点正确", "[math][transform][interp]") {
    const Transform a(Vec3(0, 0, 0), Quat::Identity());
    const Transform b(Vec3(10, 0, 0), Quat::FromAxisAngle(Vec3::UnitY(), kHalfPi));

    REQUIRE(ApproxEq(Transform::Interpolate(a, b, real(0)).position, a.position));
    REQUIRE(ApproxEq(Transform::Interpolate(a, b, real(1)).position, b.position));

    const Transform mid = Transform::Interpolate(a, b, real(0.5));
    REQUIRE(ApproxEq(mid.position, Vec3(5, 0, 0), real(1e-5)));
    REQUIRE(mid.rotation.Angle() == Approx(kHalfPi * real(0.5)).margin(real(1e-4)));
}

//==============================================================================
// Mat4（渲染层边界）
//==============================================================================

TEST_CASE("Mat4 与 Transform 等价", "[math][mat4]") {
    const Transform t(Vec3(3, -1, 5),
                      Quat::FromAxisAngle(Vec3(1, 2, -1).Normalized(), real(0.8)));
    const Mat4 m = Mat4::FromTransform(t);
    const Vec3 p(2, 4, -6);

    REQUIRE(ApproxEq(m.TransformPoint(p), t.TransformPoint(p), real(1e-4)));
    REQUIRE(ApproxEq(m.TransformDirection(p), t.TransformDirection(p), real(1e-4)));
    REQUIRE(ApproxEq(m.Translation(), t.position, real(1e-5)));
}

TEST_CASE("Mat4 刚体求逆", "[math][mat4]") {
    const Transform t(Vec3(-2, 6, 3),
                      Quat::FromAxisAngle(Vec3::UnitZ(), real(1.9)));
    const Mat4 m = Mat4::FromTransform(t);
    const Mat4 inv = m.InverseRigid();

    REQUIRE(NearlyEqual(m * inv, Mat4::Identity(), real(1e-4)));

    const Vec3 p(7, -3, 1);
    REQUIRE(ApproxEq(inv.TransformPoint(m.TransformPoint(p)), p, real(1e-4)));
}

TEST_CASE("Mat4 列优先导出符合 OpenGL 布局", "[math][mat4][render-interop]") {
    // OpenGL / glm 期望平移量落在数组下标 12、13、14。
    // 这个用例是"和渲染层对接不出错"的书面凭证 —— 如果你的渲染层不是这个布局，
    // 改 ToColumnMajorArray 的同时把这里一起改，别在调用点临时转置。
    const Mat4 m = Mat4::FromTranslation(Vec3(1, 2, 3));

    float col[16];
    m.ToColumnMajorArray(col);
    REQUIRE(col[12] == Approx(1.0f));
    REQUIRE(col[13] == Approx(2.0f));
    REQUIRE(col[14] == Approx(3.0f));
    REQUIRE(col[15] == Approx(1.0f));

    float row[16];
    m.ToRowMajorArray(row);
    REQUIRE(row[3] == Approx(1.0f));
    REQUIRE(row[7] == Approx(2.0f));
    REQUIRE(row[11] == Approx(3.0f));
}

//==============================================================================
// POD 性质 —— 为将来的 ECS 化 / 序列化 / 网络同步兜底
//==============================================================================

TEST_CASE("数学类型保持平凡可复制", "[math][pod]") {
    // 这些静态断言保证：
    //   - 可以整块 memcpy（存档、网络同步、GPU 上传）
    //   - 可以放进 union（Shape 的形状参数联合体，M2 要用）
    //   - 大数组分配时不会有隐藏的构造开销
    STATIC_REQUIRE(std::is_trivially_copyable_v<Vec2>);
    STATIC_REQUIRE(std::is_trivially_copyable_v<Vec3>);
    STATIC_REQUIRE(std::is_trivially_copyable_v<Vec4>);
    STATIC_REQUIRE(std::is_trivially_copyable_v<Mat3>);
    STATIC_REQUIRE(std::is_trivially_copyable_v<Mat4>);
    STATIC_REQUIRE(std::is_trivially_copyable_v<Quat>);
    STATIC_REQUIRE(std::is_trivially_copyable_v<Transform>);

    STATIC_REQUIRE(std::is_trivially_default_constructible_v<Vec3>);
    STATIC_REQUIRE(std::is_trivially_default_constructible_v<Quat>);
    STATIC_REQUIRE(std::is_trivially_default_constructible_v<Transform>);

    STATIC_REQUIRE(std::is_standard_layout_v<Vec3>);
    STATIC_REQUIRE(sizeof(Vec3) == 3 * sizeof(real));
    STATIC_REQUIRE(sizeof(Quat) == 4 * sizeof(real));
    STATIC_REQUIRE(sizeof(Mat3) == 9 * sizeof(real));
}
