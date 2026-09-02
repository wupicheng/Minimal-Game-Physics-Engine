#pragma once
//==============================================================================
// pe/math/Vec2.h
//
// 二维向量。3D 引擎里它不是主角，但下面这些地方确实需要：
//   - 摩擦力的两个切向分量打包成一对 (lambda_t0, lambda_t1)
//   - 接触流形裁剪时，把多边形投影到接触平面上做 2D 裁剪
//   - 角色控制器的水平面移动输入（把 XZ 平面当 2D 处理）
//   - 均匀网格在 XZ 平面上的 2D 查询
//
// POD 语义与 Vec3 一致：默认构造不初始化。
//==============================================================================

#include "pe/core/Types.h"
#include "pe/math/MathUtil.h"

namespace pe {

struct Vec2 {
    real x, y;

    Vec2() = default;
    constexpr Vec2(real x_, real y_) noexcept : x(x_), y(y_) {}
    explicit constexpr Vec2(real s) noexcept : x(s), y(s) {}

    static constexpr Vec2 Zero() noexcept { return Vec2(0, 0); }
    static constexpr Vec2 One() noexcept { return Vec2(1, 1); }
    static constexpr Vec2 UnitX() noexcept { return Vec2(1, 0); }
    static constexpr Vec2 UnitY() noexcept { return Vec2(0, 1); }

    constexpr real& operator[](int i) noexcept { return (&x)[i]; }
    constexpr const real& operator[](int i) const noexcept { return (&x)[i]; }

    constexpr Vec2 operator-() const noexcept { return Vec2(-x, -y); }
    constexpr Vec2& operator+=(const Vec2& v) noexcept { x += v.x; y += v.y; return *this; }
    constexpr Vec2& operator-=(const Vec2& v) noexcept { x -= v.x; y -= v.y; return *this; }
    constexpr Vec2& operator*=(real s) noexcept { x *= s; y *= s; return *this; }
    constexpr Vec2& operator/=(real s) noexcept {
        const real inv = real(1) / s;
        x *= inv; y *= inv; return *this;
    }

    constexpr real LengthSq() const noexcept { return x * x + y * y; }
    real Length() const noexcept { return Sqrt(LengthSq()); }

    Vec2 Normalized() const noexcept {
        const real lenSq = LengthSq();
        if (lenSq < kNormalizeEpsilonSq) return Zero();
        // 直接构造，理由同 Vec3::Normalized()：自由函数 operator* 尚未声明。
        const real inv = real(1) / Sqrt(lenSq);
        return Vec2(x * inv, y * inv);
    }

    constexpr bool IsZero(real eps = kEpsilon) const noexcept {
        return LengthSq() <= eps * eps;
    }
};

inline constexpr Vec2 operator+(const Vec2& a, const Vec2& b) noexcept {
    return Vec2(a.x + b.x, a.y + b.y);
}
inline constexpr Vec2 operator-(const Vec2& a, const Vec2& b) noexcept {
    return Vec2(a.x - b.x, a.y - b.y);
}
inline constexpr Vec2 operator*(const Vec2& v, real s) noexcept {
    return Vec2(v.x * s, v.y * s);
}
inline constexpr Vec2 operator*(real s, const Vec2& v) noexcept { return v * s; }
inline constexpr Vec2 operator/(const Vec2& v, real s) noexcept {
    const real inv = real(1) / s;
    return Vec2(v.x * inv, v.y * inv);
}

inline constexpr bool operator==(const Vec2& a, const Vec2& b) noexcept {
    return a.x == b.x && a.y == b.y;
}
inline constexpr bool operator!=(const Vec2& a, const Vec2& b) noexcept { return !(a == b); }

inline constexpr real Dot(const Vec2& a, const Vec2& b) noexcept {
    return a.x * b.x + a.y * b.y;
}

/// 2D "叉积"：返回标量 a.x*b.y - a.y*b.x，即三维叉积的 z 分量。
/// 几何含义是两向量张成的平行四边形有向面积。
/// 用途：判断点在有向边的哪一侧（接触流形的 2D 裁剪要反复用），
/// 正值表示 b 在 a 的逆时针方向。
inline constexpr real Cross(const Vec2& a, const Vec2& b) noexcept {
    return a.x * b.y - a.y * b.x;
}

/// 逆时针旋转 90 度。2D 里求法线的最快方式。
inline constexpr Vec2 Perpendicular(const Vec2& v) noexcept { return Vec2(-v.y, v.x); }

inline constexpr Vec2 Lerp(const Vec2& a, const Vec2& b, real t) noexcept {
    return a + (b - a) * t;
}
inline constexpr bool NearlyEqual(const Vec2& a, const Vec2& b,
                                  real eps = kEpsilon) noexcept {
    return NearlyEqual(a.x, b.x, eps) && NearlyEqual(a.y, b.y, eps);
}

}  // namespace pe
