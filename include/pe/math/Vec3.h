#pragma once
//==============================================================================
// pe/math/Vec3.h
//
// 三维向量。引擎里最核心的类型：位置、速度、力、法线、力臂全是它。
//
// 坐标系约定（整个引擎统一，改这里等于改一切）：
//   - 右手坐标系
//   - +Y 朝上（重力默认是 (0, -9.81, 0)）
//   - 胶囊体的轴向沿局部 +Y
//
// POD 性：Vec3 是平凡类型（trivial）。默认构造**不初始化**成员，这是刻意的：
//   1. 保持平凡默认构造，才能安全放进 union（Shape 的形状参数联合体要用），
//      也才能整块 memcpy。
//   2. 大数组分配时不做无谓的清零。
//   需要零值请显式写 Vec3 v{} 或 Vec3::Zero()。
//==============================================================================

#include "pe/core/Types.h"
#include "pe/math/MathUtil.h"

namespace pe {

struct Vec3 {
    real x, y, z;

    /// 不初始化。要零值请用 Vec3{} 或 Vec3::Zero()。
    Vec3() = default;

    constexpr Vec3(real x_, real y_, real z_) noexcept : x(x_), y(y_), z(z_) {}

    /// 三个分量同值（常用于 halfExtents 之类的立方体尺寸）。
    explicit constexpr Vec3(real s) noexcept : x(s), y(s), z(s) {}

    //--------------------------------------------------------------------------
    // 常量
    //--------------------------------------------------------------------------
    static constexpr Vec3 Zero() noexcept { return Vec3(0, 0, 0); }
    static constexpr Vec3 One() noexcept { return Vec3(1, 1, 1); }
    static constexpr Vec3 UnitX() noexcept { return Vec3(1, 0, 0); }
    static constexpr Vec3 UnitY() noexcept { return Vec3(0, 1, 0); }
    static constexpr Vec3 UnitZ() noexcept { return Vec3(0, 0, 1); }

    //--------------------------------------------------------------------------
    // 分量访问
    //--------------------------------------------------------------------------
    constexpr real& operator[](int i) noexcept { return (&x)[i]; }
    constexpr const real& operator[](int i) const noexcept { return (&x)[i]; }

    //--------------------------------------------------------------------------
    // 一元与复合赋值
    //--------------------------------------------------------------------------
    constexpr Vec3 operator-() const noexcept { return Vec3(-x, -y, -z); }

    constexpr Vec3& operator+=(const Vec3& v) noexcept {
        x += v.x; y += v.y; z += v.z; return *this;
    }
    constexpr Vec3& operator-=(const Vec3& v) noexcept {
        x -= v.x; y -= v.y; z -= v.z; return *this;
    }
    constexpr Vec3& operator*=(real s) noexcept {
        x *= s; y *= s; z *= s; return *this;
    }
    constexpr Vec3& operator/=(real s) noexcept {
        // 乘倒数：一次除法 + 三次乘法，比三次除法快，且三个分量误差一致。
        const real inv = real(1) / s;
        x *= inv; y *= inv; z *= inv; return *this;
    }

    //--------------------------------------------------------------------------
    // 长度与归一化
    //--------------------------------------------------------------------------

    /// 长度的平方。比较长度大小时优先用它 —— 省一次 sqrt，且没有精度损失。
    constexpr real LengthSq() const noexcept { return x * x + y * y + z * z; }

    real Length() const noexcept { return Sqrt(LengthSq()); }

    /// 归一化。零向量（或接近零）返回零向量而不是 NaN。
    /// 这个防御非常重要：碰撞法线在两物体完全重合时会退化成零向量，
    /// 若返回 NaN 会立刻污染整个刚体状态且极难定位。
    Vec3 Normalized() const noexcept {
        const real lenSq = LengthSq();
        if (lenSq < kNormalizeEpsilonSq) {
            return Zero();
        }
        // 直接构造而不是写 `*this * inv`：自由函数 operator* 在本结构体之后才声明，
        // 类内定义的成员函数看不到它（C++ 的非限定名字查找发生在定义点）。
        const real inv = real(1) / Sqrt(lenSq);
        return Vec3(x * inv, y * inv, z * inv);
    }

    /// 原地归一化，返回归一化前的长度（调用方常常需要这个长度，
    /// 比如"移动方向 + 移动距离"，这样只算一次 sqrt）。
    real Normalize() noexcept {
        const real len = Length();
        if (len < real(1e-6)) {
            *this = Zero();
            return real(0);
        }
        *this *= (real(1) / len);
        return len;
    }

    constexpr bool IsZero(real eps = kEpsilon) const noexcept {
        return LengthSq() <= eps * eps;
    }

    bool IsFinite() const noexcept {
        return pe::IsFinite(x) && pe::IsFinite(y) && pe::IsFinite(z);
    }
};

//------------------------------------------------------------------------------
// 二元运算
//------------------------------------------------------------------------------

inline constexpr Vec3 operator+(const Vec3& a, const Vec3& b) noexcept {
    return Vec3(a.x + b.x, a.y + b.y, a.z + b.z);
}
inline constexpr Vec3 operator-(const Vec3& a, const Vec3& b) noexcept {
    return Vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}
inline constexpr Vec3 operator*(const Vec3& v, real s) noexcept {
    return Vec3(v.x * s, v.y * s, v.z * s);
}
inline constexpr Vec3 operator*(real s, const Vec3& v) noexcept { return v * s; }
inline constexpr Vec3 operator/(const Vec3& v, real s) noexcept {
    const real inv = real(1) / s;
    return Vec3(v.x * inv, v.y * inv, v.z * inv);
}

/// 逐分量乘。注意这不是任何有物理意义的乘法，只在缩放包围盒之类的地方用。
inline constexpr Vec3 MulPerComponent(const Vec3& a, const Vec3& b) noexcept {
    return Vec3(a.x * b.x, a.y * b.y, a.z * b.z);
}

inline constexpr bool operator==(const Vec3& a, const Vec3& b) noexcept {
    return a.x == b.x && a.y == b.y && a.z == b.z;
}
inline constexpr bool operator!=(const Vec3& a, const Vec3& b) noexcept {
    return !(a == b);
}

//------------------------------------------------------------------------------
// 点积与叉积
//------------------------------------------------------------------------------

/// 点积 a·b = |a||b|cos(theta)。
/// 物理引擎里的两大用途：
///   1. 把一个向量投影到某个方向上（相对速度沿法线的分量 = Dot(v, n)）
///   2. 判断朝向（Dot(v, n) < 0 表示 v 指向 n 的背面，即正在接近）
inline constexpr real Dot(const Vec3& a, const Vec3& b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

/// 叉积 a×b：结果垂直于 a、b 张成的平面，长度 = |a||b|sin(theta)，方向遵右手定则。
/// 物理引擎里的两大用途：
///   1. 力矩 tau = r × F，以及接触点的速度 v_p = v + omega × r
///   2. 从两条边求面法线（SAT 的边叉积轴、EPA 的面法线）
inline constexpr Vec3 Cross(const Vec3& a, const Vec3& b) noexcept {
    return Vec3(a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x);
}

inline real Distance(const Vec3& a, const Vec3& b) noexcept { return (a - b).Length(); }
inline constexpr real DistanceSq(const Vec3& a, const Vec3& b) noexcept {
    return (a - b).LengthSq();
}

//------------------------------------------------------------------------------
// 逐分量工具
//------------------------------------------------------------------------------

inline constexpr Vec3 MinPerComponent(const Vec3& a, const Vec3& b) noexcept {
    return Vec3(Min(a.x, b.x), Min(a.y, b.y), Min(a.z, b.z));
}
inline constexpr Vec3 MaxPerComponent(const Vec3& a, const Vec3& b) noexcept {
    return Vec3(Max(a.x, b.x), Max(a.y, b.y), Max(a.z, b.z));
}
inline constexpr Vec3 AbsPerComponent(const Vec3& v) noexcept {
    return Vec3(Abs(v.x), Abs(v.y), Abs(v.z));
}
inline constexpr Vec3 Lerp(const Vec3& a, const Vec3& b, real t) noexcept {
    return a + (b - a) * t;
}
inline constexpr bool NearlyEqual(const Vec3& a, const Vec3& b,
                                  real eps = kEpsilon) noexcept {
    return NearlyEqual(a.x, b.x, eps) && NearlyEqual(a.y, b.y, eps) &&
           NearlyEqual(a.z, b.z, eps);
}

/// 返回绝对值最大的分量下标（0/1/2）。SAT 找主轴、盒子找参考面时要用。
inline constexpr int MaxAbsComponentIndex(const Vec3& v) noexcept {
    const Vec3 a = AbsPerComponent(v);
    if (a.x >= a.y && a.x >= a.z) return 0;
    return a.y >= a.z ? 1 : 2;
}

//------------------------------------------------------------------------------
// 正交基构造
//------------------------------------------------------------------------------

/// 给定单位向量 n，构造出 (n, t1, t2) 这组右手正交基。
///
/// 用途：摩擦力需要在接触平面内取两个正交的切向。切向具体指哪个方向不重要
/// （各向同性摩擦），重要的是必须**始终数值稳定**。
///
/// 朴素做法是拿 n 和某个固定轴（比如 +Y）做叉积，但当 n 恰好平行于 +Y 时
/// 叉积为零向量，归一化就炸了 —— 而"角色站在水平地面上"恰恰就是 n = +Y，
/// 是最常见的情况，所以这个坑必然会踩到。
///
/// 这里用的是 Erin Catto（Box2D 作者）的做法：根据 n.x 的大小在两个构造方式
/// 之间切换，保证被选中的那个构造出的向量长度平方至少是 1/3（0.57735 = 1/sqrt(3)
/// 正是三个分量均分时的分界点），因此永远不会退化。
inline void BuildOrthonormalBasis(const Vec3& n, Vec3& t1, Vec3& t2) noexcept {
    if (Abs(n.x) >= real(0.57735)) {
        // n 在 x 上分量够大，用 (n.y, -n.x, 0)：它与 n 的点积恒为 0。
        t1 = Vec3(n.y, -n.x, real(0));
    } else {
        // 否则 n 在 y、z 上分量够大，用 (0, n.z, -n.y)：点积同样恒为 0。
        t1 = Vec3(real(0), n.z, -n.y);
    }
    t1 = t1.Normalized();
    t2 = Cross(n, t1);  // n 与 t1 都是单位且正交，叉积自动是单位向量
}

}  // namespace pe
